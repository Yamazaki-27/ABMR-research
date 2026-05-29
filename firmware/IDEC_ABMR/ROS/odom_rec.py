#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading
import json
import os

# 【修正】Odometry ではなく geometry_msgs の Pose をインポート
from geometry_msgs.msg import Pose

try:
    from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
except ImportError:
    from http.server import BaseHTTPRequestHandler, HTTPServer

# グローバル変数
current_x = 0.0
current_y = 0.0
pose_lock = threading.Lock()

SAVE_DIR = os.path.expanduser("~/map/0014")
FILE_PATH = os.path.join(SAVE_DIR, "waypoint.json")

def odom_callback(msg):
    """ ROSトピックから現在位置を取得し更新する """
    global current_x, current_y
    with pose_lock:
        try:
            # 【修正】以前成功したという msg.position.x / y の形に統一
            current_x = msg.position.x
            current_y = msg.position.y
        except Exception as e:
            rospy.logerr("座標の代入に失敗しました: %s", str(e))

class RequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path != '/rec':
            self.send_error(404, "Not Found")
            return

        content_length_str = self.headers.get('content-length')
        if content_length_str is None:
            try:
                content_length_str = self.headers.getheader('content-length')
            except AttributeError:
                content_length_str = '0'
        
        content_length = int(content_length_str) if content_length_str else 0
        post_data = self.rfile.read(content_length)

        try:
            data = json.loads(post_data.decode('utf-8'))
            if 'waypoint' not in data:
                self.send_error(400, "Bad Request: JSON body must contain 'waypoint' key")
                return

            waypoint_num = data['waypoint']

            # 最新の座標を取得
            with pose_lock:
                x = current_x
                y = current_y

            if not os.path.exists(SAVE_DIR):
                os.makedirs(SAVE_DIR)

            saved_data = []
            if os.path.exists(FILE_PATH):
                with open(FILE_PATH, 'r') as f:
                    try:
                        saved_data = json.load(f)
                    except ValueError:
                        saved_data = []

            # --- 【修正ロジック】既存の経由点があるか検索し、あれば上書き、なければ追加 ---
            updated = False
            for record in saved_data:
                if record.get("waypoint") == waypoint_num:
                    record["x"] = x
                    record["y"] = y
                    updated = True
                    break
            
            if not updated:
                saved_data.append({
                    "waypoint": waypoint_num,
                    "x": x,
                    "y": y
                })
            # -----------------------------------------------------------------

            with open(FILE_PATH, 'w') as f:
                json.dump(saved_data, f, indent=4)

            rospy.loginfo("Saved Waypoint: %s, X: %.3f, Y: %.3f", waypoint_num, x, y)

            # APIのレスポンスデータを作成
            response_record = {"waypoint": waypoint_num, "x": x, "y": y}
            response_body = json.dumps({"status": "success", "data": response_record})
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Content-length', str(len(response_body)))
            self.end_headers()
            self.wfile.write(response_body.encode('utf-8'))

        except Exception as e:
            rospy.logerr("API Error: %s", str(e))
            self.send_error(500, "Internal Server Error")

    def log_message(self, format, *args):
        rospy.logdebug("%s - - [%s] %s\n" % (self.client_address[0], self.log_date_time_string(), format%args))

def run_server():
    server_address = ('0.0.0.0', 5002)
    httpd = HTTPServer(server_address, RequestHandler)
    httpd.serve_forever()

if __name__ == '__main__':
    rospy.init_node('waypoint_recorder_node', anonymous=True)
    
    # 【修正】メッセージ型を Pose に変更
    rospy.Subscriber('atmobi/robot_pose', Pose, odom_callback)

    server_thread = threading.Thread(target=run_server)
    server_thread.daemon = True
    server_thread.start()

    rospy.loginfo("Waypoint recorder initialized. Listening built-in HTTP API on port 5002...")
    rospy.spin()