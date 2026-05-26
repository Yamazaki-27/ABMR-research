#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import threading
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

try:
    from http.server import BaseHTTPRequestHandler, HTTPServer
except ImportError:
    from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer

# ============================================================
# パラメータ設定 (カット＆トライ用定数)
# ここ数値を変更して run_docker.sh を再実行してください
# ============================================================

# Start座標からEnd座標に向かうための設定
END_X = -1.28     # End座標 X (m)
END_Y = 0.6  #0.27      # End座標 Y (m)

# 旋回制御用の定数
ANGULAR_SPEED = 1.5      # 最大旋回速度 (絶対値)
ANGULAR_GAIN = 0.02
ALLOWABLE_ERROR = 4
ROTATE_VALUE_MIN = 1
ROTATE_VALUE_MAX = 254
MIN_ANGULAR_SPEED = 0.05
CMD_INTERVAL_SEC = 0.1
HTTP_PORT = 5001

# Step2, Step3用パラメータ (今回はStep1までですが、構成として残しておきます)
LINEAR_SPEED = -0.2
TARGET_DISTANCE = 1.9
TARGET_VALUE_STRAIGHT = 120
STRAIGHT_DISTANCE = 1.0

# プログラムで自動決定される目標値
TARGET_VALUE_A = None

def load_params():
    global END_X, END_Y, ANGULAR_SPEED, ANGULAR_GAIN, ALLOWABLE_ERROR
    global ROTATE_VALUE_MIN, ROTATE_VALUE_MAX, MIN_ANGULAR_SPEED
    global CMD_INTERVAL_SEC, HTTP_PORT, LINEAR_SPEED, TARGET_DISTANCE
    global TARGET_VALUE_STRAIGHT, STRAIGHT_DISTANCE
    
    END_X = rospy.get_param('~end_x', END_X)
    END_Y = rospy.get_param('~end_y', END_Y)
    ANGULAR_SPEED = rospy.get_param('~angular_speed', ANGULAR_SPEED)
    ANGULAR_GAIN = rospy.get_param('~angular_gain', ANGULAR_GAIN)
    ALLOWABLE_ERROR = rospy.get_param('~allowable_error', ALLOWABLE_ERROR)
    ROTATE_VALUE_MIN = rospy.get_param('~rotate_value_min', ROTATE_VALUE_MIN)
    ROTATE_VALUE_MAX = rospy.get_param('~rotate_value_max', ROTATE_VALUE_MAX)
    MIN_ANGULAR_SPEED = rospy.get_param('~min_angular_speed', MIN_ANGULAR_SPEED)
    CMD_INTERVAL_SEC = rospy.get_param('~cmd_interval_sec', CMD_INTERVAL_SEC)
    HTTP_PORT = rospy.get_param('~http_port', HTTP_PORT)

# ============================================================

# グローバル状態管理
latest_value2 = None
lock = threading.Lock()

current_state = 'STEP0' # odom受信および旋回方向計算待ち状態

start_x = None
start_y = None
start_yaw = None

accumulated_distance = 0.0
prev_x = None
prev_y = None

def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

def euler_from_quaternion(x, y, z, w):
    # クォータニオンからYaw角(Z軸周りの回転)を算出
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class WebAPIHandler(BaseHTTPRequestHandler):
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
                    
                    response_str = json.dumps({"status": "ok", "value2": latest_value2})
                    response_bytes = response_str.encode('utf-8') if type(response_str) is not bytes else response_str
                    
                    self.send_response(200)
                    self.send_header("Content-type", "application/json")
                    self.send_header("Content-Length", str(len(response_bytes)))
                    self.end_headers()
                    self.wfile.write(response_bytes)
                else:
                    self.send_error(400, "Missing value2")
            except Exception as e:
                self.send_error(400, "Invalid JSON")
        else:
            self.send_error(404, "Not Found")
            
    def log_message(self, format, *args):
        pass

def odom_callback(msg):
    global prev_x, prev_y, accumulated_distance
    global start_x, start_y, start_yaw
    
    with lock:
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        
        # 初回受信時にStart座標として記録
        if start_x is None:
            start_x = curr_x
            start_y = curr_y
            q = msg.pose.pose.orientation
            start_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        
        # 以降のステップ用距離計測 (今回はStep1で終了しますが機能は残します)
        if current_state in ['STEP2', 'STEP3']:
            if prev_x is not None and prev_y is not None:
                dx = curr_x - prev_x
                dy = curr_y - prev_y
                accumulated_distance += math.sqrt(dx**2 + dy**2)
            
            prev_x = curr_x
            prev_y = curr_y

def cmd_vel_loop():
    global current_state, accumulated_distance, prev_x, prev_y
    global TARGET_VALUE_A
    
    ros_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rate = rospy.Rate(1.0 / CMD_INTERVAL_SEC)
    
    rospy.loginfo("Start publishing cmd_vel every %.3f sec", CMD_INTERVAL_SEC)
    
    while not rospy.is_shutdown():
        with lock:
            val2 = latest_value2
            state = current_state
            current_dist = accumulated_distance
            sx = start_x
            sy = start_y
            syaw = start_yaw
            
        msg = Twist()
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        
        if state == 'STEP0':
            # odomからの初期位置受信待ち ＆ 旋回方向の決定
            if sx is not None and sy is not None and syaw is not None:
                dx_to_end = END_X - sx
                dy_to_end = END_Y - sy

                # ロボットの前進方向ベクトル (odom座標系)
                fwd_x = math.cos(syaw)
                fwd_y = math.sin(syaw)

                # Endへの方向角度 (odom座標系)
                angle_to_end = math.atan2(dy_to_end, dx_to_end)

                # -------------------------------------------------------
                # ロボット座標系でのEnd位置を計算
                #   local_x = 前方成分 (正=前方)
                #   local_y = 側方成分 (標準右手系では正=左, 負=右)
                # -------------------------------------------------------
                local_x = fwd_x * dx_to_end + fwd_y * dy_to_end
                local_y = -fwd_y * dx_to_end + fwd_x * dy_to_end

                # -------------------------------------------------------
                # 旋回方向の判定
                # ★重要★ local_y の符号と 90/150 の対応関係は
                # 実機動作で確認が必要。想定通りでなければ下記を変更:
                #   TURN_SIGN = +1  → local_y > 0 のとき value2=90
                #   TURN_SIGN = -1  → local_y > 0 のとき value2=150
                # -------------------------------------------------------
                TURN_SIGN = 1   # +1 or -1 で旋回方向を反転できます

                if local_y * TURN_SIGN > 0:
                    TARGET_VALUE_A = 90
                    direction_str = "local_y * TURN_SIGN > 0 -> value2=90"
                else:
                    TARGET_VALUE_A = 150
                    direction_str = "local_y * TURN_SIGN <= 0 -> value2=150"

                rospy.loginfo("--- Direction Initialization ---")
                rospy.loginfo("Start         : pos(%.3f, %.3f), yaw=%.3f rad (%.1f deg)",
                              sx, sy, syaw, math.degrees(syaw))
                rospy.loginfo("End           : pos(%.3f, %.3f)", END_X, END_Y)
                rospy.loginfo("Vector to End : dx=%.3f  dy=%.3f", dx_to_end, dy_to_end)
                rospy.loginfo("angle_to_end  : %.3f rad (%.1f deg)",
                              angle_to_end, math.degrees(angle_to_end))
                rospy.loginfo("Robot local frame:")
                rospy.loginfo("  local_x (forward) = %.4f  (+ = ahead, - = behind)", local_x)
                rospy.loginfo("  local_y (lateral) = %.4f  (standard: + = left, - = right)", local_y)
                rospy.loginfo("TURN_SIGN=%d  ->  local_y*TURN_SIGN = %.4f", TURN_SIGN, local_y * TURN_SIGN)
                rospy.loginfo("Decision      : %s", direction_str)
                rospy.loginfo("TARGET_VALUE_A set to: %d", TARGET_VALUE_A)
                rospy.loginfo("--------------------------------")
                
                with lock:
                    current_state = 'STEP1'
                    state = 'STEP1'
            else:
                rospy.loginfo_throttle(2.0, "Debug - State: STEP0, waiting for /odom...")
                
        elif state == 'STOP':
            # 停止状態
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            rospy.loginfo_throttle(2.0, "Debug - State: STOP, Task completed.")
            
        elif val2 is not None and TARGET_VALUE_A is not None:
            # 安全範囲外の場合は強制停止
            if val2 < ROTATE_VALUE_MIN or val2 > ROTATE_VALUE_MAX:
                rospy.logwarn_throttle(2.0, "value2 (%d) is out of safe range [%d, %d]. Stopping.", 
                                       val2, ROTATE_VALUE_MIN, ROTATE_VALUE_MAX)
                msg.angular.z = 0.0
                msg.linear.x = 0.0
            else:
                # 状態に応じて目標値を切り替え（今回はStep1のみ使用）
                current_target = TARGET_VALUE_A if state in ['STEP1', 'STEP2'] else TARGET_VALUE_STRAIGHT
                diff = val2 - current_target
                
                # 角速度の計算
                if abs(diff) <= ALLOWABLE_ERROR:
                    msg.angular.z = 0.0
                    
                    # 状態遷移 (今回はSTEP1完了後にSTOPへ遷移して終了)
                    if state == 'STEP1':
                        rospy.loginfo("Target angle (%d) reached! Transitioning to STOP (as requested for this test).", TARGET_VALUE_A)
                        with lock:
                            current_state = 'STOP'
                            state = 'STOP'
                else:
                    # 比例制御
                    calc_speed = abs(diff) * ANGULAR_GAIN
                    max_speed = abs(ANGULAR_SPEED)
                    calc_speed = clamp(calc_speed, MIN_ANGULAR_SPEED, max_speed)
                    
                    # ANGULAR_SPEEDの符号でモーターの回転方向（プラスマイナス）の反転に対応
                    base_direction = 1.0 if ANGULAR_SPEED >= 0 else -1.0
                    
                    if current_target < val2:
                        msg.angular.z = calc_speed * base_direction
                    else:
                        msg.angular.z = calc_speed * (-base_direction)
                
                # 直進速度 (Step1中は0)
                if state == 'STEP1':
                    msg.linear.x = 0.0

            if state != 'STOP':
                rospy.loginfo("Debug - State: %s, value2: %d, Target: %d, cmd_z: %.3f", 
                              state, val2, current_target, msg.angular.z)
        else:
            # val2 が未受信の場合
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            
        ros_publisher.publish(msg)
        rate.sleep()

def main():
    rospy.init_node('steer_auto_node', anonymous=True)
    load_params()
    
    server_address = ('0.0.0.0', HTTP_PORT)
    httpd = HTTPServer(server_address, WebAPIHandler)
    server_thread = threading.Thread(target=httpd.serve_forever)
    server_thread.daemon = True
    server_thread.start()
    
    rospy.loginfo("Web API Server started on port %d", HTTP_PORT)
    rospy.loginfo("END_X = %.2f, END_Y = %.2f", END_X, END_Y)
    rospy.loginfo("ANGULAR_SPEED (Max speed) = %.2f", ANGULAR_SPEED)
    
    try:
        cmd_vel_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        httpd.shutdown()

if __name__ == '__main__':
    main()
