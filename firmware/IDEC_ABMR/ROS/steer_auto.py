#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import os
import threading
import math
import rospy
from geometry_msgs.msg import Twist, Pose

try:
    from http.server import BaseHTTPRequestHandler, HTTPServer
except ImportError:
    from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer

# ============================================================
# パラメータ設定 (カット＆トライ用定数)
# ここの数値を変更して run_docker.sh を再実行してください
# ============================================================

# waypointファイルのパス
WAYPOINT_JSON_PATH = os.path.expanduser("~/map/0014/waypoint.json")

# HTTPサーバポート設定
HTTP_PORT_PLC  = 5001   # /plc_d_register  (PLCからのドライブユニット現在値受信)
HTTP_PORT_TURN = 5002   # /turn            (waypointコマンド受信)

# 旋回制御用の定数
ANGULAR_SPEED    = 1.5      # 最大旋回速度 (絶対値)
ANGULAR_GAIN     = 0.02
ALLOWABLE_ERROR  = 4
ROTATE_VALUE_MIN = 1
ROTATE_VALUE_MAX = 254
MIN_ANGULAR_SPEED = 0.05
CMD_INTERVAL_SEC  = 0.1

# Step1 ドライブユニット目標値 (後で調整可能)
# ★ ここを変更してカット＆トライしてください ★
TARGET_VALUE_RIGHT    = 50    # 右旋回時のドライブユニット目標値 (固定値)
TARGET_VALUE_STRAIGHT = 120   # 直進時のドライブユニット目標値 (固定値)
TARGET_VALUE_LEFT     = 190   # 左旋回時のドライブユニット目標値 (固定値)

# Step2, Step3用パラメータ
LINEAR_SPEED   = -0.2   # 走行速度 (負=後退)
STEP2_DISTANCE = 2.0    # STEP2（円弧走行）の固定移動距離 (m) ★調整可★
# STEP3の移動距離はSTEP2完了時の座標からEndまでの残り距離を自動算出

# ============================================================
# 目標座標 (起動時はNone。/turnリクエスト受信後に設定される)
# ============================================================
END_X = None   # 目標座標 X (m)  /turn受信後に設定
END_Y = None   # 目標座標 Y (m)  /turn受信後に設定

# プログラムで自動決定される目標値
TARGET_VALUE_A = None

# ============================================================

def load_params():
    global ANGULAR_SPEED, ANGULAR_GAIN, ALLOWABLE_ERROR
    global ROTATE_VALUE_MIN, ROTATE_VALUE_MAX, MIN_ANGULAR_SPEED
    global CMD_INTERVAL_SEC, HTTP_PORT_PLC, HTTP_PORT_TURN
    global LINEAR_SPEED, STEP2_DISTANCE
    global TARGET_VALUE_STRAIGHT
    global TARGET_VALUE_RIGHT, TARGET_VALUE_LEFT
    global WAYPOINT_JSON_PATH

    ANGULAR_SPEED     = rospy.get_param('~angular_speed',     ANGULAR_SPEED)
    ANGULAR_GAIN      = rospy.get_param('~angular_gain',      ANGULAR_GAIN)
    ALLOWABLE_ERROR   = rospy.get_param('~allowable_error',   ALLOWABLE_ERROR)
    ROTATE_VALUE_MIN  = rospy.get_param('~rotate_value_min',  ROTATE_VALUE_MIN)
    ROTATE_VALUE_MAX  = rospy.get_param('~rotate_value_max',  ROTATE_VALUE_MAX)
    MIN_ANGULAR_SPEED = rospy.get_param('~min_angular_speed', MIN_ANGULAR_SPEED)
    CMD_INTERVAL_SEC  = rospy.get_param('~cmd_interval_sec',  CMD_INTERVAL_SEC)
    HTTP_PORT_PLC     = rospy.get_param('~http_port_plc',     HTTP_PORT_PLC)
    HTTP_PORT_TURN    = rospy.get_param('~http_port_turn',    HTTP_PORT_TURN)
    LINEAR_SPEED      = rospy.get_param('~linear_speed',      LINEAR_SPEED)
    STEP2_DISTANCE    = rospy.get_param('~step2_distance',    STEP2_DISTANCE)
    TARGET_VALUE_STRAIGHT = rospy.get_param('~target_value_straight', TARGET_VALUE_STRAIGHT)
    TARGET_VALUE_RIGHT    = rospy.get_param('~target_value_right',    TARGET_VALUE_RIGHT)
    TARGET_VALUE_LEFT     = rospy.get_param('~target_value_left',     TARGET_VALUE_LEFT)
    WAYPOINT_JSON_PATH    = rospy.get_param('~waypoint_json_path',    WAYPOINT_JSON_PATH)

# ============================================================
# waypoint.json ローダー
# ============================================================

def load_waypoint(waypoint_no):
    """
    waypoint.jsonから指定した番号のX,Y座標を返す。
    見つからない場合は None, None を返す。
    """
    try:
        path = os.path.expanduser(WAYPOINT_JSON_PATH)
        with open(path, 'r') as f:
            waypoints = json.load(f)
        for wp in waypoints:
            if wp.get("waypoint") == waypoint_no:
                return float(wp["x"]), float(wp["y"])
        rospy.logwarn("waypoint %s not found in %s", waypoint_no, path)
        return None, None
    except Exception as e:
        rospy.logerr("Failed to load waypoint.json: %s", str(e))
        return None, None

# ============================================================
# グローバル状態管理
# ============================================================

latest_value2 = None
lock = threading.Lock()

# 状態一覧:
#   WAIT_WAYPOINT  : /turnリクエストによる目標座標の受信待ち
#   WAIT_POSE      : robot_poseからの初期位置取得待ち
#   STEP1          : ドライブユニットを旋回目標角度へ回転（停車中）
#   STEP2          : 旋回角度を維持しながら円弧走行（固定距離）
#   STEP3          : ハンドルを直進に戻しながら直進走行（残り距離）
#   STOP           : 完了・停止
current_state = 'WAIT_WAYPOINT'

start_x   = None
start_y   = None
start_yaw = None

accumulated_distance  = 0.0
prev_x = None
prev_y = None

step3_target_distance = None   # STEP3の走行目標距離（STEP2完了時に算出）
current_robot_x = None         # 最新ロボット位置X（全ステップで随時更新）
current_robot_y = None         # 最新ロボット位置Y（全ステップで随時更新）

# ============================================================
# ユーティリティ
# ============================================================

def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

def euler_from_quaternion(x, y, z, w):
    # クォータニオンからYaw角(Z軸周りの回転)を算出
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def reset_for_new_goal():
    """新しい目標座標を受け取ったときに状態をリセットする。ロック外から呼ぶこと。"""
    global current_state, start_x, start_y, start_yaw
    global accumulated_distance, prev_x, prev_y
    global step3_target_distance, TARGET_VALUE_A
    with lock:
        current_state         = 'WAIT_POSE'
        start_x               = None
        start_y               = None
        start_yaw             = None
        accumulated_distance  = 0.0
        prev_x                = None
        prev_y                = None
        step3_target_distance = None
        TARGET_VALUE_A        = None

# ============================================================
# HTTPハンドラ
# ============================================================

class PlcHandler(BaseHTTPRequestHandler):
    """ポート5001: PLCからドライブユニット現在値 (value2) を受信する。"""

    def do_POST(self):
        global latest_value2

        if self.path == "/plc_d_register":
            content_length = self.headers.get("Content-Length")
            if content_length is None:
                content_length = self.headers.get("content-length")

            content_length = int(content_length) if content_length else 0

            if content_length == 0:
                self.send_error(400, "Empty body")
                return

            body = self.rfile.read(content_length)
            try:
                body_str = body.decode('utf-8') if type(body) is not str else body
                data = json.loads(body_str)

                if "value2" in data:
                    with lock:
                        latest_value2 = int(data["value2"])

                    response_str   = json.dumps({"status": "ok", "value2": latest_value2})
                    response_bytes = response_str.encode('utf-8') if type(response_str) is not bytes else response_str

                    self.send_response(200)
                    self.send_header("Content-type", "application/json")
                    self.send_header("Content-Length", str(len(response_bytes)))
                    self.end_headers()
                    self.wfile.write(response_bytes)
                else:
                    self.send_error(400, "Missing value2")
            except Exception:
                self.send_error(400, "Invalid JSON")
        else:
            self.send_error(404, "Not Found")

    def log_message(self, format, *args):
        pass


class TurnHandler(BaseHTTPRequestHandler):
    """
    ポート5002: waypointコマンドを受信する。

    期待するPOSTデータ (JSON):
        {
            "waypoint_no": 1,
            "status": "up"
        }

    処理:
      1. waypoint_no で waypoint.json を検索し END_X, END_Y を設定
      2. 状態を WAIT_POSE にリセットして走行シーケンスを再スタート
    """

    def do_POST(self):
        global END_X, END_Y

        if self.path != "/turn":
            self.send_error(404, "Not Found")
            return

        content_length = self.headers.get("Content-Length")
        if content_length is None:
            content_length = self.headers.get("content-length")
        content_length = int(content_length) if content_length else 0

        if content_length == 0:
            self._respond(400, {"status": "error", "message": "Empty body"})
            return

        body = self.rfile.read(content_length)
        try:
            body_str = body.decode('utf-8') if type(body) is not str else body
            data = json.loads(body_str)
        except Exception:
            self._respond(400, {"status": "error", "message": "Invalid JSON"})
            return

        if "waypoint_no" not in data:
            self._respond(400, {"status": "error", "message": "Missing waypoint_no"})
            return

        waypoint_no = data["waypoint_no"]
        wp_x, wp_y = load_waypoint(waypoint_no)

        if wp_x is None or wp_y is None:
            self._respond(404, {
                "status": "error",
                "message": "waypoint_no %s not found" % str(waypoint_no)
            })
            return

        # 目標座標を更新し、走行シーケンスをリセット
        END_X = wp_x
        END_Y = wp_y
        reset_for_new_goal()

        rospy.loginfo("=== /turn received ===")
        rospy.loginfo("  waypoint_no : %s", str(waypoint_no))
        rospy.loginfo("  status      : %s", str(data.get("status", "")))
        rospy.loginfo("  END_X=%.3f, END_Y=%.3f", END_X, END_Y)
        rospy.loginfo("  State -> WAIT_POSE  (waiting for robot_pose)")

        self._respond(200, {
            "status": "ok",
            "waypoint_no": waypoint_no,
            "end_x": END_X,
            "end_y": END_Y
        })

    def _respond(self, code, data):
        body = json.dumps(data).encode('utf-8')
        self.send_response(code)
        self.send_header("Content-type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        pass

# ============================================================
# ROSコールバック
# ============================================================

def robot_pose_callback(msg):
    """絶対座標トピック /atmobi/robot_pose (geometry_msgs/Pose) のコールバック。
    メッセージ型が PoseStamped の場合は msg.pose.position / msg.pose.orientation に変更してください。
    """
    global prev_x, prev_y, accumulated_distance
    global start_x, start_y, start_yaw
    global current_robot_x, current_robot_y

    with lock:
        curr_x   = msg.position.x
        curr_y   = msg.position.y
        q        = msg.orientation
        curr_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        # 常に最新のロボット位置を保持（STEP3距離算出などに使用）
        current_robot_x = curr_x
        current_robot_y = curr_y

        # WAIT_POSE 状態のとき: 初回受信でStart座標を確定し STEP0 へ遷移
        if current_state == 'WAIT_POSE' and start_x is None:
            start_x   = curr_x
            start_y   = curr_y
            start_yaw = curr_yaw
            rospy.loginfo("robot_pose received: Start pos(%.3f, %.3f), yaw=%.3f rad (%.1f deg)",
                          curr_x, curr_y, curr_yaw, math.degrees(curr_yaw))
            # STEP0（旋回方向計算）へ遷移
            # ※ current_state の更新は cmd_vel_loop 内で行う（ロック内二重更新を避けるため）
            #    ここでは start_x をセットするだけでよい

        # STEP2/STEP3 用距離計測
        if current_state in ['STEP2', 'STEP3']:
            if prev_x is not None and prev_y is not None:
                dx = curr_x - prev_x
                dy = curr_y - prev_y
                accumulated_distance += math.sqrt(dx**2 + dy**2)

            prev_x = curr_x
            prev_y = curr_y

# ============================================================
# cmd_vel制御ループ
# ============================================================

def cmd_vel_loop():
    global current_state, accumulated_distance, prev_x, prev_y
    global TARGET_VALUE_A
    global TARGET_VALUE_RIGHT, TARGET_VALUE_LEFT, TARGET_VALUE_STRAIGHT
    global step3_target_distance

    ros_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/atmobi/robot_pose', Pose, robot_pose_callback)
    rate = rospy.Rate(1.0 / CMD_INTERVAL_SEC)

    rospy.loginfo("Start publishing cmd_vel every %.3f sec", CMD_INTERVAL_SEC)

    while not rospy.is_shutdown():
        with lock:
            val2         = latest_value2
            state        = current_state
            current_dist = accumulated_distance
            sx           = start_x
            sy           = start_y
            syaw         = start_yaw
            crx          = current_robot_x
            cry          = current_robot_y
            end_x        = END_X
            end_y        = END_Y

        msg = Twist()
        msg.linear.y  = 0.0
        msg.linear.z  = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.linear.x  = 0.0
        msg.angular.z = 0.0

        # ----------------------------------------------------------
        # WAIT_WAYPOINT: /turn POSTを待機中
        # ----------------------------------------------------------
        if state == 'WAIT_WAYPOINT':
            rospy.loginfo_throttle(5.0,
                "State: WAIT_WAYPOINT - Waiting for POST to http://localhost:%d/turn ...", HTTP_PORT_TURN)

        # ----------------------------------------------------------
        # WAIT_POSE: robot_poseからの初期位置取得待ち
        # ----------------------------------------------------------
        elif state == 'WAIT_POSE':
            if sx is not None:
                # Start座標取得済み → STEP0（旋回方向計算）へ
                with lock:
                    current_state = 'STEP0'
                    state = 'STEP0'
                rospy.loginfo("Start pos confirmed. -> STEP0 (direction calculation)")
            else:
                rospy.loginfo_throttle(2.0,
                    "State: WAIT_POSE - Waiting for /atmobi/robot_pose ...")

        # ----------------------------------------------------------
        # STEP0: 旋回方向計算 → STEP1へ
        # ----------------------------------------------------------
        if state == 'STEP0':
            if sx is not None and sy is not None and syaw is not None and end_x is not None and end_y is not None:
                dx_to_end = end_x - sx
                dy_to_end = end_y - sy

                # ロボットの前進方向ベクトル
                fwd_x = math.cos(syaw)
                fwd_y = math.sin(syaw)

                # Endへの方向角度
                angle_to_end = math.atan2(dy_to_end, dx_to_end)

                # ロボット座標系でのEnd位置を計算
                #   local_x = 前方成分 (正=前方)
                #   local_y = 側方成分 (標準右手系では正=左, 負=右)
                local_x = fwd_x * dx_to_end + fwd_y * dy_to_end
                local_y = -fwd_y * dx_to_end + fwd_x * dy_to_end

                # 旋回方向の判定
                # ★重要★ local_y の符号と 右旋回/左旋回 の対応関係:
                #   TURN_SIGN = +1  → local_y > 0 のとき 右旋回 (TARGET_VALUE_RIGHT=50)
                #   TURN_SIGN = -1  → local_y > 0 のとき 左旋回 (TARGET_VALUE_LEFT=190)
                TURN_SIGN = 1   # +1 or -1 で旋回方向を反転できます

                if local_y * TURN_SIGN > 0:
                    TARGET_VALUE_A = TARGET_VALUE_RIGHT   # 右旋回: 固定値50
                    direction_str = "local_y * TURN_SIGN > 0 -> Right Turn (右旋回) -> target=%d" % TARGET_VALUE_RIGHT
                else:
                    TARGET_VALUE_A = TARGET_VALUE_LEFT    # 左旋回: 固定値190
                    direction_str = "local_y * TURN_SIGN <= 0 -> Left Turn (左旋回) -> target=%d" % TARGET_VALUE_LEFT

                rospy.loginfo("--- Direction Initialization ---")
                rospy.loginfo("Start         : pos(%.3f, %.3f), yaw=%.3f rad (%.1f deg)",
                              sx, sy, syaw, math.degrees(syaw))
                rospy.loginfo("End           : pos(%.3f, %.3f)", end_x, end_y)
                rospy.loginfo("Vector to End : dx=%.3f  dy=%.3f", dx_to_end, dy_to_end)
                rospy.loginfo("angle_to_end  : %.3f rad (%.1f deg)",
                              angle_to_end, math.degrees(angle_to_end))
                rospy.loginfo("Robot local frame:")
                rospy.loginfo("  local_x (forward) = %.4f  (+ = ahead, - = behind)", local_x)
                rospy.loginfo("  local_y (lateral) = %.4f  (standard: + = left, - = right)", local_y)
                rospy.loginfo("TURN_SIGN=%d  ->  local_y*TURN_SIGN = %.4f", TURN_SIGN, local_y * TURN_SIGN)
                rospy.loginfo("Decision      : %s", direction_str)
                rospy.loginfo("TARGET_VALUE_A set to: %d", TARGET_VALUE_A)

                # Start→Endのユークリッド直線距離（参考値）
                total_distance = math.sqrt(dx_to_end**2 + dy_to_end**2)
                rospy.loginfo("Start->End 直線距離 (参考) = %.3f m", total_distance)
                rospy.loginfo("  STEP2 (円弧走行) : 固定 %.3f m", STEP2_DISTANCE)
                rospy.loginfo("  STEP3 (直進走行) : STEP2完了時の座標からEndまでの残り距離を自動算出")
                rospy.loginfo("--------------------------------")

                with lock:
                    current_state = 'STEP1'
                    state = 'STEP1'

        # ----------------------------------------------------------
        # STOP: 完了・停止
        # ----------------------------------------------------------
        elif state == 'STOP':
            msg.linear.x  = 0.0
            msg.angular.z = 0.0
            rospy.loginfo_throttle(5.0,
                "State: STOP - Task completed. Waiting for next /turn command.")

        # ----------------------------------------------------------
        # STEP1 / STEP2 / STEP3: ドライブユニット制御
        # ----------------------------------------------------------
        elif val2 is not None and TARGET_VALUE_A is not None:
            # 安全範囲外の場合は強制停止
            if val2 < ROTATE_VALUE_MIN or val2 > ROTATE_VALUE_MAX:
                rospy.logwarn_throttle(2.0,
                    "value2 (%d) is out of safe range [%d, %d]. Stopping.",
                    val2, ROTATE_VALUE_MIN, ROTATE_VALUE_MAX)
                msg.angular.z = 0.0
                msg.linear.x  = 0.0
            else:
                # 状態に応じて目標ドライブユニット値を切り替え
                current_target = TARGET_VALUE_A if state in ['STEP1', 'STEP2'] else TARGET_VALUE_STRAIGHT
                diff = val2 - current_target

                # -------------------------------------------------------
                # 状態遷移（距離・角度に基づく）
                # -------------------------------------------------------
                if state == 'STEP1':
                    # ドライブユニットが目標角度に到達 → STEP2へ
                    if abs(diff) <= ALLOWABLE_ERROR:
                        rospy.loginfo("STEP1完了: ドライブユニット %d に到達。STEP2（カーブ走行）へ移行。", TARGET_VALUE_A)
                        with lock:
                            accumulated_distance = 0.0
                            prev_x = None
                            prev_y = None
                            current_state = 'STEP2'
                            state = 'STEP2'

                elif state == 'STEP2':
                    # 固定距離(STEP2_DISTANCE)の円弧走行完了 → STEP3へ
                    if current_dist >= STEP2_DISTANCE:
                        # STEP3の走行距離 = 現在位置からEndまでの残り距離を算出
                        if crx is not None and cry is not None:
                            dx_s3  = end_x - crx
                            dy_s3  = end_y - cry
                            s3_dist = math.sqrt(dx_s3**2 + dy_s3**2)
                        else:
                            s3_dist = 1.0  # フォールバック（現在位置不明時）
                        rospy.loginfo("STEP2完了: %.3f m 移動（円弧）。STEP3（直進走行）へ移行。", current_dist)
                        rospy.loginfo("  現在位置: (%.3f, %.3f) -> End: (%.3f, %.3f)  残り %.3f m",
                                      crx if crx is not None else 0.0,
                                      cry if cry is not None else 0.0,
                                      end_x, end_y, s3_dist)
                        with lock:
                            step3_target_distance = s3_dist
                            accumulated_distance  = 0.0
                            current_state = 'STEP3'
                            state = 'STEP3'

                elif state == 'STEP3':
                    # STEP2完了時に算出した残り距離に達したら停止
                    s3_target = step3_target_distance if step3_target_distance is not None else 1.0
                    if current_dist >= s3_target:
                        rospy.loginfo("STEP3完了: %.3f m / %.3f m 移動。停止します。", current_dist, s3_target)
                        with lock:
                            current_state = 'STOP'
                            state = 'STOP'

                # -------------------------------------------------------
                # 角速度の計算（比例制御）
                # -------------------------------------------------------
                if abs(diff) <= ALLOWABLE_ERROR:
                    msg.angular.z = 0.0
                else:
                    calc_speed    = abs(diff) * ANGULAR_GAIN
                    max_speed     = abs(ANGULAR_SPEED)
                    calc_speed    = clamp(calc_speed, MIN_ANGULAR_SPEED, max_speed)

                    # ANGULAR_SPEEDの符号でモーター回転方向を調整
                    base_direction = 1.0 if ANGULAR_SPEED >= 0 else -1.0

                    if current_target < val2:
                        msg.angular.z = calc_speed * base_direction
                    else:
                        msg.angular.z = calc_speed * (-base_direction)

                # -------------------------------------------------------
                # 直進速度の設定
                #   STEP1: 停止（ドライブユニット回転のみ）
                #   STEP2/3: LINEAR_SPEED で走行
                # -------------------------------------------------------
                if state in ['STEP2', 'STEP3']:
                    msg.linear.x = LINEAR_SPEED
                # STEP1 は msg.linear.x = 0.0 (ループ先頭で初期化済み)

            if state not in ['STOP', 'WAIT_WAYPOINT', 'WAIT_POSE', 'STEP0']:
                if state == 'STEP2':
                    dist_limit_log = STEP2_DISTANCE
                elif state == 'STEP3':
                    dist_limit_log = step3_target_distance if step3_target_distance is not None else 0.0
                else:
                    dist_limit_log = 0.0
                rospy.loginfo_throttle(1.0,
                    "State: %s | value2: %d -> target: %d (diff: %+d) | dist: %.3f/%.3f m | linear: %.2f cmd_z: %.3f",
                    state, val2, current_target, diff, current_dist, dist_limit_log,
                    msg.linear.x, msg.angular.z)
        else:
            # val2 が未受信の場合 (STEP1〜STEP3)
            if state in ['STEP1', 'STEP2', 'STEP3']:
                rospy.loginfo_throttle(2.0, "State: %s - Waiting for value2 from PLC ...", state)
            msg.linear.x  = 0.0
            msg.angular.z = 0.0

        ros_publisher.publish(msg)
        rate.sleep()

# ============================================================
# main
# ============================================================

def main():
    rospy.init_node('steer_auto_node', anonymous=True)
    load_params()

    # ポート5001: PLC (ドライブユニット現在値)
    plc_address  = ('0.0.0.0', HTTP_PORT_PLC)
    httpd_plc    = HTTPServer(plc_address, PlcHandler)
    plc_thread   = threading.Thread(target=httpd_plc.serve_forever)
    plc_thread.daemon = True
    plc_thread.start()

    # ポート5002: turn コマンド (waypoint受信)
    turn_address = ('0.0.0.0', HTTP_PORT_TURN)
    httpd_turn   = HTTPServer(turn_address, TurnHandler)
    turn_thread  = threading.Thread(target=httpd_turn.serve_forever)
    turn_thread.daemon = True
    turn_thread.start()

    rospy.loginfo("=== steer_auto_node started ===")
    rospy.loginfo("  PLC server  : http://0.0.0.0:%d/plc_d_register", HTTP_PORT_PLC)
    rospy.loginfo("  Turn server : http://0.0.0.0:%d/turn", HTTP_PORT_TURN)
    rospy.loginfo("  Waypoint JSON: %s", WAYPOINT_JSON_PATH)
    rospy.loginfo("--- Drive Unit Target Values (Fixed) ---")
    rospy.loginfo("  TARGET_VALUE_RIGHT    = %d  (右旋回時)", TARGET_VALUE_RIGHT)
    rospy.loginfo("  TARGET_VALUE_STRAIGHT = %d  (直進時 / STEP3)", TARGET_VALUE_STRAIGHT)
    rospy.loginfo("  TARGET_VALUE_LEFT     = %d  (左旋回時)", TARGET_VALUE_LEFT)
    rospy.loginfo("  STEP2_DISTANCE        = %.2f m  (円弧走行固定距離)", STEP2_DISTANCE)
    rospy.loginfo("----------------------------------------")
    rospy.loginfo("Waiting for POST to http://localhost:%d/turn ...", HTTP_PORT_TURN)

    try:
        cmd_vel_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        httpd_plc.shutdown()
        httpd_turn.shutdown()

if __name__ == '__main__':
    main()
