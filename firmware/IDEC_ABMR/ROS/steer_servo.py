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

# /cmd_vel の linear.x に出す前後進速度
# value1 == 1 のとき FORWARD_LINEAR_SPEED
# value1 == 2 のとき REVERSE_LINEAR_SPEED
# value1 == 0 のとき linear.x = 0.0
#
# 注意:
# 使用しているロボット側の仕様に合わせて符号を確認してください。
# 一般的なROSでは linear.x 正方向が前進、負方向が後進です。
FORWARD_LINEAR_SPEED = 0.6
REVERSE_LINEAR_SPEED = -0.6

# cmd_velはパルスではなく連続的に出す。
# value1=1/2の間は、タイマー周期ごとにこの前後進速度を出し続ける。

# /cmd_vel の angular.z に出す旋回速度
# 設定範囲: -6.0 ～ +6.0
# 符号で旋回方向を決める
#
# value2 < value3 のとき: ANGULAR_SPEED と同じ向き
# value2 > value3 のとき: ANGULAR_SPEED と逆向き
ANGULAR_SPEED = -2.4

# 目標値に対する偏差の許容値
# 例: 3なら、value2とvalue3の差が3以内なら旋回停止
ALLOWABLE_ERROR = 4

# この範囲外の値では旋回させない
# value2またはvalue3がこの範囲外なら安全のため旋回停止
# ただし value1 による前後進は可能
ROTATE_VALUE_MIN = 1	#14
ROTATE_VALUE_MAX = 254	#242

# cmd_velを出す周期
# 0.1～1.0秒程度で調整
CMD_INTERVAL_SEC = 0.05

# 簡易サーボ制御用パラメータ
# パルス的に「動く→止まる」を繰り返すのではなく、
# 偏差 error = value3 - value2 に比例した旋回速度を連続的に出す。
#
# cmd_angular = error * ANGULAR_GAIN
# ただし、MIN/MAXで制限する。
ANGULAR_GAIN = 0.02
MIN_ANGULAR_SPEED = 0.01
MAX_ANGULAR_SPEED = abs(ANGULAR_SPEED)

# 目標付近でギクシャクする場合は True を推奨。
# True の場合、目標近傍では MIN_ANGULAR_SPEED を強制せず、
# 小さい偏差なら小さい速度のまま出す。
# False の場合、許容偏差外では最低速度 MIN_ANGULAR_SPEED を必ず出す。
USE_DEADZONE_SMOOTHING = False

# ローパスフィルタ係数
# 小さいほど値の変化がゆっくりになる
# 0.1: 強めに平滑化
# 0.3: 中程度
# 1.0: フィルタなし
LOW_PASS_ALPHA = 1.0

# 急激な値飛びを無視するしきい値
# 前回値からこれ以上飛んだ生値は異常値として無視する
# 不要なら 999 など大きくする
#
# value1は前後進モード 0/1/2 なので、この判定はvalue2/value3だけに適用する
MAX_RAW_JUMP = 999

# WebAPI受信ポート
# Node-REDから /plc_d_register と /velmode の両方をこのPythonで受ける
HTTP_PORT = 5001

# WebAPI受信パス
HTTP_PATH = "/plc_d_register"

# 手動/自動モード受信パス
# Node-RED -> Python で manual / auto を送る
VELMODE_PATH = "/velmode"

# 起動直後は安全側として auto 扱いにする
# manual を受信するまで /cmd_vel はpublishしない
DEFAULT_VELMODE = "auto"

# publishするROSトピック
TOPIC_VALUE1 = "/plc/d_register1"   # 前後進モード: 0=停止, 1=前進, 2=後進
TOPIC_VALUE2 = "/plc/d_register2"   # 旋回用 現在値
TOPIC_VALUE3 = "/plc/d_register3"   # 旋回用 目標値
TOPIC_CMD_VEL = "/cmd_vel"


# ============================================================
# グローバル変数
# ============================================================

ros_publisher_value1 = None
ros_publisher_value2 = None
ros_publisher_value3 = None
ros_publisher_cmd_vel = None

lock = threading.Lock()

latest_raw_value1 = None
latest_raw_value2 = None
latest_raw_value3 = None

filtered_value2 = None
filtered_value3 = None

last_accepted_raw_value2 = None
last_accepted_raw_value3 = None

last_cmd_linear = 0.0
last_cmd_angular = 0.0

# Node-REDから最後に受信した手動/自動モード
# manual のときだけ /cmd_vel をpublishする
latest_velmode = DEFAULT_VELMODE
last_velmode_update_time = None


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


def normalize_velmode(value):
    """
    Node-REDから受け取ったモード文字列を正規化する。
    許可する値は manual / auto のみ。
    """
    if value is None:
        return None

    try:
        mode = str(value).strip().lower()
    except Exception:
        return None

    if mode in ["manual", "auto"]:
        return mode

    return None


def update_velmode(mode):
    """
    Node-REDから受信した manual / auto を内部状態として保持する。
    """
    global latest_velmode
    global last_velmode_update_time

    normalized = normalize_velmode(mode)
    if normalized is None:
        return False

    with lock:
        latest_velmode = normalized
        last_velmode_update_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    print("[{}] velmode受信: {}".format(last_velmode_update_time, normalized))
    sys.stdout.flush()
    return True


def is_manual_mode():
    """
    現在のvelmodeがmanualならTrue。
    manual以外では /cmd_vel をpublishしてはいけない。
    """
    with lock:
        mode = latest_velmode

    return mode == "manual"


def make_stop_twist():
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    return msg


def publish_cmd_vel(cmd_linear, cmd_angular):
    """
    /cmd_velへ速度指令を1回publishする。

    以前のように rospy.sleep() 後に停止を出すパルス方式ではなく、
    タイマー周期ごとに現在必要な速度を連続的にpublishする。
    これにより目標付近の「動く→止まる→動く」のギクシャクを減らす。
    """

    global last_cmd_linear
    global last_cmd_angular

    if not is_manual_mode():
        # manual以外では停止指令も含めて /cmd_vel をpublishしない
        return False

    msg = Twist()
    msg.linear.x = cmd_linear
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = cmd_angular

    ros_publisher_cmd_vel.publish(msg)
    last_cmd_linear = cmd_linear
    last_cmd_angular = cmd_angular
    return True


def publish_stop():
    global last_cmd_linear
    global last_cmd_angular

    if not is_manual_mode():
        # manual以外では停止指令も含めて /cmd_vel をpublishしない
        return False

    msg = make_stop_twist()
    ros_publisher_cmd_vel.publish(msg)
    last_cmd_linear = 0.0
    last_cmd_angular = 0.0
    return True


def forward_mode_to_linear(value1):
    """
    value1から前後進速度を決める。
    value1 == 0: 前後進なし
    value1 == 1: 前進
    value1 == 2: 後進
    """
    if value1 == 1:
        return FORWARD_LINEAR_SPEED

    if value1 == 2:
        return REVERSE_LINEAR_SPEED

    return 0.0


# ============================================================
# WebAPIで受け取った値の処理
# ============================================================

def publish_values(value1, value2, value3):
    """
    受信した3つの値をROS 1トピックへpublishする。

    value1: 前後進モード 0=停止, 1=前進, 2=後進
    value2: 旋回用 現在値
    value3: 旋回用 目標値

    value2/value3はローパスフィルタ用の最新値として保持する。
    value1は0/1/2のモード値なのでフィルタしない。
    """

    global latest_raw_value1
    global latest_raw_value2
    global latest_raw_value3
    global filtered_value2
    global filtered_value3
    global last_accepted_raw_value2
    global last_accepted_raw_value3

    # 生値をROS topicへpublish
    msg1 = Int32()
    msg1.data = value1

    msg2 = Int32()
    msg2.data = value2

    msg3 = Int32()
    msg3.data = value3

    ros_publisher_value1.publish(msg1)
    ros_publisher_value2.publish(msg2)
    ros_publisher_value3.publish(msg3)

    with lock:
        latest_raw_value1 = value1
        latest_raw_value2 = value2
        latest_raw_value3 = value3

        accepted2 = accept_raw_value(last_accepted_raw_value2, value2)
        accepted3 = accept_raw_value(last_accepted_raw_value3, value3)

        if accepted2:
            filtered_value2 = low_pass_filter(
                filtered_value2,
                value2,
                LOW_PASS_ALPHA
            )
            last_accepted_raw_value2 = value2

        if accepted3:
            filtered_value3 = low_pass_filter(
                filtered_value3,
                value3,
                LOW_PASS_ALPHA
            )
            last_accepted_raw_value3 = value3

        f2 = filtered_value2
        f3 = filtered_value3

    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    text = (
        "[{}] 受信値: value1={}, value2={}, value3={}  "
        "filtered_value2={:.2f}, filtered_value3={:.2f}  "
        "accepted2={}, accepted3={}  "
        "ROS publish: {}, {}, {}"
    ).format(
        now,
        value1,
        value2,
        value3,
        f2 if f2 is not None else 0.0,
        f3 if f3 is not None else 0.0,
        accepted2,
        accepted3,
        TOPIC_VALUE1,
        TOPIC_VALUE2,
        TOPIC_VALUE3
    )

#    print(text)
    sys.stdout.flush()


# ============================================================
# cmd_vel制御
# ============================================================

def calc_servo_angular(error):
    """
    偏差から簡易サーボ的に旋回速度を計算する。

    error > 0: value3がvalue2より大きい
    error < 0: value3がvalue2より小さい
    ANGULAR_SPEEDの符号で実機に合わせて旋回方向を反転できる。
    """
    abs_error = abs(error)

    if abs_error <= ALLOWABLE_ERROR:
        return 0.0

    speed = abs_error * ANGULAR_GAIN

    if USE_DEADZONE_SMOOTHING:
        # 目標付近では速度を無理にMINへ持ち上げない。
        # ただし遠いところではMAXで制限する。
        speed = clamp(speed, 0.0, MAX_ANGULAR_SPEED)
    else:
        # 静止摩擦で動かない場合はこちら。
        # 許容偏差外なら最低速度を確保する。
        speed = clamp(speed, MIN_ANGULAR_SPEED, MAX_ANGULAR_SPEED)

    # 安全のため 0～6 に制限
    speed = clamp(speed, 0.0, 6.0)

    if error > 0:
        cmd_angular = speed
    else:
        cmd_angular = -speed

    # ANGULAR_SPEED自体が負なら、旋回方向を反転する
    if ANGULAR_SPEED < 0:
        cmd_angular = -cmd_angular

    return cmd_angular


def cmd_vel_timer_callback(event):
    """
    一定周期で呼ばれ、現在のvalue1/filtered_value2/filtered_value3から
    /cmd_velへ前後進 + 旋回指令を連続的に出す。

    重要:
    ここではrospy.sleep()で待ってから停止するパルス制御はしない。
    毎周期、現在の偏差に応じた速度を出し続ける。
    """

    with lock:
        v1 = latest_raw_value1
        f2 = filtered_value2
        f3 = filtered_value3
        mode = latest_velmode

    # manual以外では /cmd_vel を一切publishしない
    if mode != "manual":
        rospy.loginfo_throttle(1.0, "velmode is %s. cmd_vel publish inhibited.", str(mode))
        return

    # まだ値を受信していない場合は停止
    if v1 is None or f2 is None or f3 is None:
        publish_stop()
        return

    # value1は0/1/2だけ許可
    if v1 not in [0, 1, 2]:
        publish_stop()
        rospy.logwarn("value1 must be 0, 1, or 2. stop. value1=%s", str(v1))
        return

    cmd_linear = forward_mode_to_linear(v1)

    rotate_enabled = True
    cmd_angular = 0.0
    error = 0.0

    # 旋回許可範囲外なら旋回だけ停止
    # value1による前後進は継続可能にする
    if f2 < ROTATE_VALUE_MIN or f2 > ROTATE_VALUE_MAX:
        rotate_enabled = False
        rospy.logwarn("value2 out of range. rotation stop. filtered_value2=%.2f", f2)

    if f3 < ROTATE_VALUE_MIN or f3 > ROTATE_VALUE_MAX:
        rotate_enabled = False
        rospy.logwarn("value3 out of range. rotation stop. filtered_value3=%.2f", f3)

    if rotate_enabled:
        error = f3 - f2
        cmd_angular = calc_servo_angular(error)

    # パルス停止はしない。必要速度をそのまま連続publishする。
    # cmd_linear/cmd_angularが両方0なら停止指令と同じ。
    published = publish_cmd_vel(cmd_linear, cmd_angular)

    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

#    print(
#        "[{}] cmd_vel制御: value1={}, filtered_value2={:.2f}, filtered_value3={:.2f}, "
#        "error={:.2f}, allowable_error={}, linear.x={:.3f}, angular.z={:.3f}, "
#        "rotate_enabled={}, servo_mode=continuous"
#        .format(
#            now,
#            v1,
#            f2,
#            f3,
#            error,
#            ALLOWABLE_ERROR,
#            cmd_linear,
#            cmd_angular,
#            rotate_enabled
#        )
    print(
        "[{}] cmd_vel制御:, "
        "velmode={}, error={:.2f}, linear.x={:.3f}, angular.z={:.3f}, "
        "rotate_enabled={}, servo_mode=continuous, cmd_vel_published={}"
        .format(
            now,
            mode,
            error,
            cmd_linear,
            cmd_angular,
            rotate_enabled,
            published
        )

    )
    sys.stdout.flush()


# ============================================================
# HTTPサーバ
# ============================================================

class PlcDRegisterRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        """
        Node-REDからPOSTを受け取る。

        1) /plc_d_register
           {
               "value1": 0,
               "value2": 123,
               "value3": 150
           }

        2) /velmode
           生文字列: manual または auto
           またはJSON: {"velmode": "manual"}
        """

        if self.path == HTTP_PATH:
            self.handle_plc_d_register_post()
            return

        if self.path == VELMODE_PATH:
            self.handle_velmode_post()
            return

        self.send_error(404, "Not Found")

    def read_request_body(self):
        content_length = int(self.headers.getheader("Content-Length", 0))

        if content_length <= 0:
            return None

        return self.rfile.read(content_length)

    def handle_velmode_post(self):
        """
        Node-REDから manual / auto を受け取る。
        Node-REDのHTTP requestノードでmsg.payloadに文字列を入れる運用を想定し、
        JSONでも生文字列でも受け取れるようにしている。
        """

        body = self.read_request_body()
        if body is None:
            self.send_json_response(400, {
                "status": "error",
                "message": "Content-Length が不正です"
            })
            return

        raw_text = body.decode("utf-8") if not isinstance(body, str) else body
        raw_text = raw_text.strip()

        mode = None

        # まずJSONとして解釈できるか試す
        try:
            data = json.loads(raw_text)
            if isinstance(data, dict):
                if "velmode" in data:
                    mode = data["velmode"]
                elif "mode" in data:
                    mode = data["mode"]
                elif "value" in data:
                    mode = data["value"]
            else:
                # JSON文字列 "manual" など
                mode = data
        except Exception:
            # JSONでなければ、生文字列 manual / auto として扱う
            mode = raw_text

        if not update_velmode(mode):
            self.send_json_response(400, {
                "status": "error",
                "message": "velmode は manual または auto にしてください",
                "received": raw_text
            })
            return

        with lock:
            mode_now = latest_velmode
            updated_at = last_velmode_update_time

        self.send_json_response(200, {
            "status": "ok",
            "velmode": mode_now,
            "updated_at": updated_at,
            "cmd_vel_publish_allowed": mode_now == "manual"
        })

    def handle_plc_d_register_post(self):
        """
        Node-REDから以下のようなJSONを受け取る想定

        {
            "value1": 0,
            "value2": 123,
            "value3": 150
        }

        value1: 前後進モード 0=停止, 1=前進, 2=後進
        value2: 旋回用 現在値
        value3: 旋回用 目標値
        """

        body = self.read_request_body()

        if body is None:
            self.send_json_response(400, {
                "status": "error",
                "message": "Content-Length が不正です"
            })
            return

        try:
            data = json.loads(body)
        except Exception:
            self.send_json_response(400, {
                "status": "error",
                "message": "JSONとして解釈できません"
            })
            return

        for key in ["value1", "value2", "value3"]:
            if key not in data:
                self.send_json_response(400, {
                    "status": "error",
                    "message": "{} がありません".format(key)
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

        try:
            value3 = int(data["value3"])
        except Exception:
            self.send_json_response(400, {
                "status": "error",
                "message": "value3 を整数に変換できません"
            })
            return

        # value1は前後進モードとして0/1/2のみ受け付ける
        if value1 not in [0, 1, 2]:
            self.send_json_response(400, {
                "status": "error",
                "message": "value1 は 0=停止, 1=前進, 2=後進 のいずれかにしてください"
            })
            return

        # value2/value3は従来のDレジスタ値として0～255を受け付ける
        if value2 < 0 or value2 > 255:
            self.send_json_response(400, {
                "status": "error",
                "message": "value2 は0～255の範囲にしてください"
            })
            return

        if value3 < 0 or value3 > 255:
            self.send_json_response(400, {
                "status": "error",
                "message": "value3 は0～255の範囲にしてください"
            })
            return

        publish_values(value1, value2, value3)

        self.send_json_response(200, {
            "status": "ok",
            "received_value1": value1,
            "received_value2": value2,
            "received_value3": value3
        })

    def do_GET(self):
        """
        ブラウザでアクセスしたときの確認用
        """

        if self.path == VELMODE_PATH:
            with lock:
                mode = latest_velmode
                updated_at = last_velmode_update_time

            self.send_json_response(200, {
                "status": "ok",
                "velmode": mode,
                "updated_at": updated_at,
                "cmd_vel_publish_allowed": mode == "manual"
            })
            return

        if self.path == "/" or self.path == "/status":
            with lock:
                v1 = latest_raw_value1
                f2 = filtered_value2
                f3 = filtered_value3
                mode = latest_velmode
                updated_at = last_velmode_update_time

            self.send_json_response(200, {
                "status": "running",
                "message": "PLC D register bridge with cmd_vel linear and angular control is running",
                "post_url": HTTP_PATH,
                "ros_topic1": TOPIC_VALUE1,
                "ros_topic2": TOPIC_VALUE2,
                "ros_topic3": TOPIC_VALUE3,
                "cmd_vel_topic": TOPIC_CMD_VEL,
                "velmode_path": VELMODE_PATH,
                "latest_velmode": mode,
                "last_velmode_update_time": updated_at,
                "cmd_vel_publish_allowed": mode == "manual",
                "latest_value1": v1,
                "filtered_value2": f2,
                "filtered_value3": f3,
                "last_cmd_linear": last_cmd_linear,
                "last_cmd_angular": last_cmd_angular,
                "forward_linear_speed": FORWARD_LINEAR_SPEED,
                "reverse_linear_speed": REVERSE_LINEAR_SPEED,
                "angular_speed_setting": ANGULAR_SPEED,
                "allowable_error": ALLOWABLE_ERROR,
                "cmd_interval_sec": CMD_INTERVAL_SEC,
                "servo_mode": "continuous",
                "angular_gain": ANGULAR_GAIN,
                "use_deadzone_smoothing": USE_DEADZONE_SMOOTHING,
                "min_angular_speed": MIN_ANGULAR_SPEED,
                "max_angular_speed": MAX_ANGULAR_SPEED,
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
    if CMD_INTERVAL_SEC < 0.01:
        rospy.logwarn("CMD_INTERVAL_SECが小さすぎるため0.01秒に補正します")
        CMD_INTERVAL_SEC = 0.01

    if CMD_INTERVAL_SEC > 1.0:
        rospy.logwarn("CMD_INTERVAL_SECが大きすぎるため1.0秒に補正します")
        CMD_INTERVAL_SEC = 1.0

    if FORWARD_LINEAR_SPEED < -6.0 or FORWARD_LINEAR_SPEED > 6.0:
        rospy.logwarn("FORWARD_LINEAR_SPEEDは-6.0～+6.0に制限します")
        FORWARD_LINEAR_SPEED = clamp(FORWARD_LINEAR_SPEED, -6.0, 6.0)

    if REVERSE_LINEAR_SPEED < -6.0 or REVERSE_LINEAR_SPEED > 6.0:
        rospy.logwarn("REVERSE_LINEAR_SPEEDは-6.0～+6.0に制限します")
        REVERSE_LINEAR_SPEED = clamp(REVERSE_LINEAR_SPEED, -6.0, 6.0)

    if LOW_PASS_ALPHA <= 0.0:
        rospy.logwarn("LOW_PASS_ALPHAが0以下のため0.1に補正します")
        LOW_PASS_ALPHA = 0.1

    if LOW_PASS_ALPHA > 1.0:
        rospy.logwarn("LOW_PASS_ALPHAが1.0超のため1.0に補正します")
        LOW_PASS_ALPHA = 1.0

    if ANGULAR_SPEED < -6.0 or ANGULAR_SPEED > 6.0:
        rospy.logwarn("ANGULAR_SPEEDは-6.0～+6.0に制限します")
        ANGULAR_SPEED = clamp(ANGULAR_SPEED, -6.0, 6.0)

    if ANGULAR_GAIN <= 0.0:
        rospy.logwarn("ANGULAR_GAINが0以下のため0.03に補正します")
        ANGULAR_GAIN = 0.01

    if MIN_ANGULAR_SPEED < 0.0:
        MIN_ANGULAR_SPEED = 0.0

    if MAX_ANGULAR_SPEED <= 0.0:
        MAX_ANGULAR_SPEED = abs(ANGULAR_SPEED)

    if MAX_ANGULAR_SPEED > 6.0:
        MAX_ANGULAR_SPEED = 6.0

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

    ros_publisher_value3 = rospy.Publisher(
        TOPIC_VALUE3,
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

    print("PLC Dレジスタ受信 + cmd_vel前後進/旋回制御サーバを起動しました")
    print("Dレジスタ受信URL: http://0.0.0.0:{}{}".format(HTTP_PORT, HTTP_PATH))
    print("velmode受信URL: http://0.0.0.0:{}{}".format(HTTP_PORT, VELMODE_PATH))
    print("確認URL: http://0.0.0.0:{}/status".format(HTTP_PORT))
    print("ROS topic1: {}  value1=0停止/1前進/2後進".format(TOPIC_VALUE1))
    print("ROS topic2: {}  旋回用現在値".format(TOPIC_VALUE2))
    print("ROS topic3: {}  旋回用目標値".format(TOPIC_VALUE3))
    print("cmd_vel topic: {}".format(TOPIC_CMD_VEL))
    print("")
    print("設定値:")
    print("  FORWARD_LINEAR_SPEED = {}".format(FORWARD_LINEAR_SPEED))
    print("  REVERSE_LINEAR_SPEED = {}".format(REVERSE_LINEAR_SPEED))
    print("  ANGULAR_SPEED        = {}".format(ANGULAR_SPEED))
    print("  ALLOWABLE_ERROR      = {}".format(ALLOWABLE_ERROR))
    print("  ROTATE_VALUE_MIN     = {}".format(ROTATE_VALUE_MIN))
    print("  ROTATE_VALUE_MAX     = {}".format(ROTATE_VALUE_MAX))
    print("  CMD_INTERVAL_SEC              = {}".format(CMD_INTERVAL_SEC))
    print("  SERVO_MODE                   = continuous")
    print("  ANGULAR_GAIN                 = {}".format(ANGULAR_GAIN))
    print("  USE_DEADZONE_SMOOTHING      = {}".format(USE_DEADZONE_SMOOTHING))
    print("  MIN_ANGULAR_SPEED            = {}".format(MIN_ANGULAR_SPEED))
    print("  MAX_ANGULAR_SPEED            = {}".format(MAX_ANGULAR_SPEED))
    print("  LOW_PASS_ALPHA               = {}".format(LOW_PASS_ALPHA))
    print("  MAX_RAW_JUMP                 = {}".format(MAX_RAW_JUMP))
    print("  DEFAULT_VELMODE             = {}".format(DEFAULT_VELMODE))
    print("")
    print("JSON例:")
    print('  POST /plc_d_register  {"value1": 1, "value2": 123, "value3": 150}')
    print('  POST /velmode         manual')
    print('  POST /velmode         {"velmode": "manual"}')
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
