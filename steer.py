#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import sys
import threading
from datetime import datetime
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist


# ============================================================
# カット＆トライ用パラメータ
# ============================================================

# /cmd_vel の angular.z に出す旋回速度
# 設定範囲: -6.0 ～ +6.0
# 符号で旋回方向を決める
#
# value1 < value2 のとき: ANGULAR_SPEED と同じ向き
# value1 > value2 のとき: ANGULAR_SPEED と逆向き
ANGULAR_SPEED = -0.30

# 目標値に対する偏差の許容値
# 例: 3なら、value1とvalue2の差が3以内なら停止
ALLOWABLE_ERROR = 3

# この範囲外の値では旋回させない
# value1またはvalue2がこの範囲外なら安全のため停止
ROTATE_VALUE_MIN = 14
ROTATE_VALUE_MAX = 242

# cmd_velを出す周期
# 0.1～1.0秒程度で調整
CMD_INTERVAL_SEC = 0.5

# 旋回指令を出している時間
# 例: 0.02秒なら、20msecだけ旋回指令を出してすぐ停止する
ROTATE_PULSE_SEC = 0.3

# ローパスフィルタ係数
# 小さいほど値の変化がゆっくりになる
# 0.1: 強めに平滑化
# 0.3: 中程度
# 1.0: フィルタなし
LOW_PASS_ALPHA = 1.0

# 急激な値飛びを無視するしきい値
# 前回値からこれ以上飛んだ生値は異常値として無視する
# 不要なら 999 など大きくする
MAX_RAW_JUMP = 80

# WebAPI受信ポート
HTTP_PORT = 5001

# WebAPI受信パス
HTTP_PATH = "/plc_d_register"

# publishするROSトピック
TOPIC_VALUE1 = "/plc/d_register1"
TOPIC_VALUE2 = "/plc/d_register2"
TOPIC_CMD_VEL = "/cmd_vel"


# ============================================================
# グローバル変数
# ============================================================

ros_publisher_value1 = None
ros_publisher_value2 = None
ros_publisher_cmd_vel = None

lock = threading.Lock()

latest_raw_value1 = None
latest_raw_value2 = None

filtered_value1 = None
filtered_value2 = None

last_accepted_raw_value1 = None
last_accepted_raw_value2 = None

last_cmd_angular = 0.0


# ============================================================
# ユーティリティ
# ============================================================

def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value


def low_pass_filter(previous, new_value, alpha):
    """
    一次遅れローパスフィルタ
    previousがNoneのときは初回値としてそのまま採用
    """
    if previous is None:
        return float(new_value)

    return previous + alpha * (float(new_value) - previous)


def accept_raw_value(previous_raw, new_raw):
    """
    急激な値飛びを無視する
    previous_rawがNoneのときは初回値として採用
    """
    if previous_raw is None:
        return True

    if abs(new_raw - previous_raw) > MAX_RAW_JUMP:
        return False

    return True


def make_stop_twist():
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    return msg

def publish_rotate_pulse(cmd_angular):
    """
    短時間だけ旋回指令を出し、その後すぐ停止する
    """

    global last_cmd_angular

    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = cmd_angular

    ros_publisher_cmd_vel.publish(msg)
    last_cmd_angular = cmd_angular

    rospy.sleep(ROTATE_PULSE_SEC)

    publish_stop()

def publish_stop():
    global last_cmd_angular

    msg = make_stop_twist()
    ros_publisher_cmd_vel.publish(msg)
    last_cmd_angular = 0.0


# ============================================================
# WebAPIで受け取った値の処理
# ============================================================

def publish_values(value1, value2):
    """
    受信した2つの値をROS 1トピックへpublishし、
    ローパスフィルタ用の最新値として保持し、
    画面にも表示する
    """

    global latest_raw_value1
    global latest_raw_value2
    global filtered_value1
    global filtered_value2
    global last_accepted_raw_value1
    global last_accepted_raw_value2

    # 元コードと同じく、生値もROS topicへpublish
    msg1 = Int32()
    msg1.data = value1

    msg2 = Int32()
    msg2.data = value2

    ros_publisher_value1.publish(msg1)
    ros_publisher_value2.publish(msg2)

    with lock:
        latest_raw_value1 = value1
        latest_raw_value2 = value2

        accepted1 = accept_raw_value(last_accepted_raw_value1, value1)
        accepted2 = accept_raw_value(last_accepted_raw_value2, value2)

        if accepted1:
            filtered_value1 = low_pass_filter(
                filtered_value1,
                value1,
                LOW_PASS_ALPHA
            )
            last_accepted_raw_value1 = value1

        if accepted2:
            filtered_value2 = low_pass_filter(
                filtered_value2,
                value2,
                LOW_PASS_ALPHA
            )
            last_accepted_raw_value2 = value2

        f1 = filtered_value1
        f2 = filtered_value2

    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    text = (
        "[{}] 受信値: value1={}, value2={}  "
        "filtered_value1={:.2f}, filtered_value2={:.2f}  "
        "accepted1={}, accepted2={}  "
        "ROS publish: {}, {}"
    ).format(
        now,
        value1,
        value2,
        f1 if f1 is not None else 0.0,
        f2 if f2 is not None else 0.0,
        accepted1,
        accepted2,
        TOPIC_VALUE1,
        TOPIC_VALUE2
    )

    print(text)
    sys.stdout.flush()


# ============================================================
# cmd_vel制御
# ============================================================

def cmd_vel_timer_callback(event):
    """
    一定周期で呼ばれ、現在のfiltered_value1/value2から
    /cmd_velへ旋回指令を出す
    """

    global last_cmd_angular

    with lock:
        f1 = filtered_value1
        f2 = filtered_value2

    # まだ値を受信していない場合は停止
    if f1 is None or f2 is None:
        publish_stop()
        return

    # 旋回許可範囲外なら停止
    if f1 < ROTATE_VALUE_MIN or f1 > ROTATE_VALUE_MAX:
        publish_stop()
        rospy.logwarn("value1 out of range. stop. filtered_value1=%.2f", f1)
        return

    if f2 < ROTATE_VALUE_MIN or f2 > ROTATE_VALUE_MAX:
        publish_stop()
        rospy.logwarn("value2 out of range. stop. filtered_value2=%.2f", f2)
        return

    error = f2 - f1

    # 許容偏差内なら停止
    if abs(error) <= ALLOWABLE_ERROR:
        publish_stop()
        cmd_angular = 0.0
    else:
        # ANGULAR_SPEEDの絶対値を使い、偏差の向きで正負を切り替える
        speed = abs(ANGULAR_SPEED)

        # 安全のため -6～+6 に制限
        speed = clamp(speed, 0.0, 6.0)

        if error > 0:
            cmd_angular = speed
        else:
            cmd_angular = -speed

        # ANGULAR_SPEED自体が負なら、旋回方向を反転する
        if ANGULAR_SPEED < 0:
            cmd_angular = -cmd_angular

        # msg = Twist()
        # msg.linear.x = 0.0
        # msg.linear.y = 0.0
        # msg.linear.z = 0.0
        # msg.angular.x = 0.0
        # msg.angular.y = 0.0
        # msg.angular.z = cmd_angular

        # ros_publisher_cmd_vel.publish(msg)
        # last_cmd_angular = cmd_angular
        publish_rotate_pulse(cmd_angular)

    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    print(
        "[{}] cmd_vel制御: filtered_value1={:.2f}, filtered_value2={:.2f}, "
        "error={:.2f}, allowable_error={}, angular.z={:.3f}"
        .format(
            now,
            f1,
            f2,
            error,
            ALLOWABLE_ERROR,
            cmd_angular,
            last_cmd_angular
        )
    )
    sys.stdout.flush()


# ============================================================
# HTTPサーバ
# ============================================================

class PlcDRegisterRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        """
        Node-REDから以下のようなJSONを受け取る想定

        {
            "value1": 123,
            "value2": 150
        }
        """

        if self.path != HTTP_PATH:
            self.send_error(404, "Not Found")
            return

        content_length = int(self.headers.getheader("Content-Length", 0))

        if content_length <= 0:
            self.send_json_response(400, {
                "status": "error",
                "message": "Content-Length が不正です"
            })
            return

        body = self.rfile.read(content_length)

        try:
            data = json.loads(body)
        except Exception:
            self.send_json_response(400, {
                "status": "error",
                "message": "JSONとして解釈できません"
            })
            return

        if "value1" not in data:
            self.send_json_response(400, {
                "status": "error",
                "message": "value1 がありません"
            })
            return

        if "value2" not in data:
            self.send_json_response(400, {
                "status": "error",
                "message": "value2 がありません"
            })
            return

        try:
            value1 = int(data["value1"])
        except Exception:
            self.send_json_response(400, {
                "status": "error",
                "message": "value1 を整数に変換できません"
            })
            return

        try:
            value2 = int(data["value2"])
        except Exception:
            self.send_json_response(400, {
                "status": "error",
                "message": "value2 を整数に変換できません"
            })
            return

        # 0～255以外の値は受け付けない
        if value1 < 0 or value1 > 255:
            self.send_json_response(400, {
                "status": "error",
                "message": "value1 は0～255の範囲にしてください"
            })
            return

        if value2 < 0 or value2 > 255:
            self.send_json_response(400, {
                "status": "error",
                "message": "value2 は0～255の範囲にしてください"
            })
            return

        publish_values(value1, value2)

        self.send_json_response(200, {
            "status": "ok",
            "received_value1": value1,
            "received_value2": value2
        })

    def do_GET(self):
        """
        ブラウザでアクセスしたときの確認用
        """

        if self.path == "/" or self.path == "/status":
            with lock:
                f1 = filtered_value1
                f2 = filtered_value2

            self.send_json_response(200, {
                "status": "running",
                "message": "PLC D register bridge with cmd_vel control is running",
                "post_url": HTTP_PATH,
                "ros_topic1": TOPIC_VALUE1,
                "ros_topic2": TOPIC_VALUE2,
                "cmd_vel_topic": TOPIC_CMD_VEL,
                "filtered_value1": f1,
                "filtered_value2": f2,
                "last_cmd_angular": last_cmd_angular,
                "angular_speed_setting": ANGULAR_SPEED,
                "allowable_error": ALLOWABLE_ERROR,
                "cmd_interval_sec": CMD_INTERVAL_SEC,
                "low_pass_alpha": LOW_PASS_ALPHA
            })
        else:
            self.send_error(404, "Not Found")

    def send_json_response(self, status_code, data):
        response_body = json.dumps(data, ensure_ascii=False).encode("utf-8")

        self.send_response(status_code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(response_body)))
        self.end_headers()
        self.wfile.write(response_body)

    def log_message(self, format, *args):
        """
        標準のHTTPアクセスログを抑制する。
        必要なら pass をコメントアウトして標準出力に出す。
        """
        pass


# ============================================================
# main
# ============================================================

if __name__ == "__main__":
    rospy.init_node("plc_d_register_cmd_vel_bridge", anonymous=False)

    # パラメータの安全チェック
    if CMD_INTERVAL_SEC < 0.1:
        rospy.logwarn("CMD_INTERVAL_SECが小さすぎるため0.1秒に補正します")
        CMD_INTERVAL_SEC = 0.1

    if CMD_INTERVAL_SEC > 1.0:
        rospy.logwarn("CMD_INTERVAL_SECが大きすぎるため1.0秒に補正します")
        CMD_INTERVAL_SEC = 1.0

    if LOW_PASS_ALPHA <= 0.0:
        rospy.logwarn("LOW_PASS_ALPHAが0以下のため0.1に補正します")
        LOW_PASS_ALPHA = 0.1

    if LOW_PASS_ALPHA > 1.0:
        rospy.logwarn("LOW_PASS_ALPHAが1.0超のため1.0に補正します")
        LOW_PASS_ALPHA = 1.0

    if ANGULAR_SPEED < -6.0 or ANGULAR_SPEED > 6.0:
        rospy.logwarn("ANGULAR_SPEEDは-6.0～+6.0に制限します")
        ANGULAR_SPEED = clamp(ANGULAR_SPEED, -6.0, 6.0)

    ros_publisher_value1 = rospy.Publisher(
        TOPIC_VALUE1,
        Int32,
        queue_size=10
    )

    ros_publisher_value2 = rospy.Publisher(
        TOPIC_VALUE2,
        Int32,
        queue_size=10
    )

    ros_publisher_cmd_vel = rospy.Publisher(
        TOPIC_CMD_VEL,
        Twist,
        queue_size=10
    )

    # 一定周期でcmd_velをpublish
    rospy.Timer(
        rospy.Duration(CMD_INTERVAL_SEC),
        cmd_vel_timer_callback
    )

    server_address = ("0.0.0.0", HTTP_PORT)
    httpd = HTTPServer(server_address, PlcDRegisterRequestHandler)

    print("PLC Dレジスタ受信 + cmd_vel旋回制御サーバを起動しました")
    print("受信URL: http://0.0.0.0:{}{}".format(HTTP_PORT, HTTP_PATH))
    print("確認URL: http://0.0.0.0:{}/status".format(HTTP_PORT))
    print("ROS topic1: {}".format(TOPIC_VALUE1))
    print("ROS topic2: {}".format(TOPIC_VALUE2))
    print("cmd_vel topic: {}".format(TOPIC_CMD_VEL))
    print("")
    print("設定値:")
    print("  ANGULAR_SPEED     = {}".format(ANGULAR_SPEED))
    print("  ALLOWABLE_ERROR   = {}".format(ALLOWABLE_ERROR))
    print("  ROTATE_VALUE_MIN  = {}".format(ROTATE_VALUE_MIN))
    print("  ROTATE_VALUE_MAX  = {}".format(ROTATE_VALUE_MAX))
    print("  CMD_INTERVAL_SEC  = {}".format(CMD_INTERVAL_SEC))
    print("  LOW_PASS_ALPHA    = {}".format(LOW_PASS_ALPHA))
    print("  MAX_RAW_JUMP      = {}".format(MAX_RAW_JUMP))
    print("")
    print("終了するには Ctrl + C を押してください")
    sys.stdout.flush()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("")
        print("終了します")
    finally:
        publish_stop()
        httpd.server_close()