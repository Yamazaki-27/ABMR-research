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

TARGET_VALUE_A = 50
ANGULAR_SPEED = 1.5
ANGULAR_GAIN = 0.02
ALLOWABLE_ERROR = 4
ROTATE_VALUE_MIN = 1
ROTATE_VALUE_MAX = 254
MIN_ANGULAR_SPEED = 0.05
CMD_INTERVAL_SEC = 0.1
HTTP_PORT = 5001

# Step2用追加パラメータ
LINEAR_SPEED = -0.2      # Step2, Step3での直進方向速度
TARGET_DISTANCE = 2.0    # Step2で移動する距離(m)

# Step3用追加パラメータ
TARGET_VALUE_STRAIGHT = 120 # Step3での直進方向となるvalue2の目標値
STRAIGHT_DISTANCE = 0.8     # Step3で直進状態のまま維持する移動距離(m)

def load_params():
    global TARGET_VALUE_A, ANGULAR_SPEED, ANGULAR_GAIN, ALLOWABLE_ERROR
    global ROTATE_VALUE_MIN, ROTATE_VALUE_MAX, MIN_ANGULAR_SPEED
    global CMD_INTERVAL_SEC, HTTP_PORT, LINEAR_SPEED, TARGET_DISTANCE
    global TARGET_VALUE_STRAIGHT, STRAIGHT_DISTANCE
    
    TARGET_VALUE_A = rospy.get_param('~target_value', TARGET_VALUE_A)
    ANGULAR_SPEED = rospy.get_param('~angular_speed', ANGULAR_SPEED)
    ANGULAR_GAIN = rospy.get_param('~angular_gain', ANGULAR_GAIN)
    ALLOWABLE_ERROR = rospy.get_param('~allowable_error', ALLOWABLE_ERROR)
    ROTATE_VALUE_MIN = rospy.get_param('~rotate_value_min', ROTATE_VALUE_MIN)
    ROTATE_VALUE_MAX = rospy.get_param('~rotate_value_max', ROTATE_VALUE_MAX)
    MIN_ANGULAR_SPEED = rospy.get_param('~min_angular_speed', MIN_ANGULAR_SPEED)
    CMD_INTERVAL_SEC = rospy.get_param('~cmd_interval_sec', CMD_INTERVAL_SEC)
    HTTP_PORT = rospy.get_param('~http_port', HTTP_PORT)
    LINEAR_SPEED = rospy.get_param('~linear_speed', LINEAR_SPEED)
    TARGET_DISTANCE = rospy.get_param('~target_distance', TARGET_DISTANCE)
    TARGET_VALUE_STRAIGHT = rospy.get_param('~target_value_straight', TARGET_VALUE_STRAIGHT)
    STRAIGHT_DISTANCE = rospy.get_param('~straight_distance', STRAIGHT_DISTANCE)

# ============================================================

# グローバル状態管理
latest_value2 = None
lock = threading.Lock()

current_state = 'STEP1'
accumulated_distance = 0.0
prev_x = None
prev_y = None

def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

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
    
    with lock:
        # STEP2 と STEP3 の両方で距離を計測する
        if current_state in ['STEP2', 'STEP3']:
            curr_x = msg.pose.pose.position.x
            curr_y = msg.pose.pose.position.y
            
            if prev_x is not None and prev_y is not None:
                dx = curr_x - prev_x
                dy = curr_y - prev_y
                accumulated_distance += math.sqrt(dx**2 + dy**2)
            
            prev_x = curr_x
            prev_y = curr_y

def cmd_vel_loop():
    global current_state, accumulated_distance, prev_x, prev_y
    
    ros_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rate = rospy.Rate(1.0 / CMD_INTERVAL_SEC)
    
    rospy.loginfo("Start publishing cmd_vel every %.3f sec", CMD_INTERVAL_SEC)
    
    while not rospy.is_shutdown():
        with lock:
            val2 = latest_value2
            state = current_state
            current_dist = accumulated_distance
            
        msg = Twist()
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        
        if state == 'STOP':
            # 停止状態
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            rospy.loginfo_throttle(2.0, "Debug - State: STOP, Waiting... Distance covered in last step: %.3f m", current_dist)
            
        elif val2 is not None:
            # 安全範囲外の場合は強制停止
            if val2 < ROTATE_VALUE_MIN or val2 > ROTATE_VALUE_MAX:
                rospy.logwarn_throttle(2.0, "value2 (%d) is out of safe range [%d, %d]. Stopping.", 
                                       val2, ROTATE_VALUE_MIN, ROTATE_VALUE_MAX)
                msg.angular.z = 0.0
                msg.linear.x = 0.0
            else:
                # 状態に応じて目標値を切り替え
                current_target = TARGET_VALUE_A if state in ['STEP1', 'STEP2'] else TARGET_VALUE_STRAIGHT
                diff = val2 - current_target
                
                # 角速度の計算
                if abs(diff) <= ALLOWABLE_ERROR:
                    msg.angular.z = 0.0
                    
                    # 状態遷移 (STEP1 -> STEP2)
                    if state == 'STEP1':
                        rospy.loginfo("Target angle reached! Transitioning to STEP2.")
                        with lock:
                            current_state = 'STEP2'
                            state = 'STEP2'
                            accumulated_distance = 0.0
                            prev_x = None
                            prev_y = None
                else:
                    # 比例制御
                    calc_speed = abs(diff) * ANGULAR_GAIN
                    max_speed = abs(ANGULAR_SPEED)
                    calc_speed = clamp(calc_speed, MIN_ANGULAR_SPEED, max_speed)
                    
                    base_direction = 1.0 if ANGULAR_SPEED >= 0 else -1.0
                    if current_target < val2:
                        msg.angular.z = calc_speed * base_direction
                    else:
                        msg.angular.z = calc_speed * (-base_direction)
                
                # 直進速度と距離に基づく状態遷移
                if state == 'STEP1':
                    msg.linear.x = 0.0
                elif state == 'STEP2':
                    msg.linear.x = LINEAR_SPEED
                    
                    # 停止判定 (STEP2 -> STEP3)
                    if current_dist >= TARGET_DISTANCE:
                        rospy.loginfo("Target distance (%.3f m) reached! Transitioning to STEP3.", TARGET_DISTANCE)
                        with lock:
                            current_state = 'STEP3'
                            state = 'STEP3'
                            accumulated_distance = 0.0  # Step3用の距離計測としてリセット
                            current_dist = 0.0
                            prev_x = None
                            prev_y = None
                elif state == 'STEP3':
                    msg.linear.x = LINEAR_SPEED
                    
                    # 停止判定 (STEP3 -> STOP)
                    if current_dist >= STRAIGHT_DISTANCE:
                        rospy.loginfo("Straight distance (%.3f m) reached! Transitioning to STOP.", STRAIGHT_DISTANCE)
                        with lock:
                            current_state = 'STOP'
                            state = 'STOP'
                        msg.linear.x = 0.0
                        msg.angular.z = 0.0

            rospy.loginfo("Debug - State: %s, value2: %s, Target: %d, cmd_x: %.2f, cmd_z: %.3f, Dist: %.3fm", 
                          state, str(val2), current_target if val2 is not None and state != 'STOP' else 0, 
                          msg.linear.x, msg.angular.z, current_dist)
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
    rospy.loginfo("TARGET_VALUE_A = %d", TARGET_VALUE_A)
    rospy.loginfo("TARGET_VALUE_STRAIGHT = %d", TARGET_VALUE_STRAIGHT)
    rospy.loginfo("ANGULAR_SPEED = %.2f", ANGULAR_SPEED)
    rospy.loginfo("LINEAR_SPEED = %.2f", LINEAR_SPEED)
    rospy.loginfo("TARGET_DISTANCE = %.2f", TARGET_DISTANCE)
    rospy.loginfo("STRAIGHT_DISTANCE = %.2f", STRAIGHT_DISTANCE)
    
    try:
        cmd_vel_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        httpd.shutdown()

if __name__ == '__main__':
    main()
