#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
import threading
import json
import os
import BaseHTTPServer
import urlparse

# 最新の座標を保持するグローバル変数とロック
current_pose = {"x": None, "y": None}
pose_lock = threading.Lock()

# --- 保存先パスの設定 ---
SAVE_DIR = os.path.expanduser('~/map/0014/')
JSON_FILE = os.path.join(SAVE_DIR, "waypoints_record.jsonl")

# --- 標準ライブラリを使用したHTTPリクエストハンドラ ---
class WaypointRequestHandler(BaseHTTPServer.BaseHTTPRequestHandler):
    
    def do_GET(self):
        """GETリクエストの処理 (?index=1 など)"""
        parsed_path = urlparse.urlparse(self.path)
        if parsed_path.path == '/record/waypoint':
            query = urlparse.parse_qs(parsed_path.query)
            index_value = query.get('index', [None])[0]
            self.process_record(index_value)
        else:
            self.send_error(404, "Not Found")

    def do_POST(self):
        """POSTリクエストの処理 (JSON形式での送信など)"""
        parsed_path = urlparse.urlparse(self.path)
        if parsed_path.path == '/record/waypoint':
            content_length = int(self.headers.getheader('content-length', 0))
            post_data = self.rfile.read(content_length)
            
            index_value = None
            if post_data:
                try:
                    # JSONとしてパースを試みる
                    data = json.loads(post_data)
                    index_value = data.get('index')
                except ValueError:
                    # JSONでなければフォームデータとしてパースを試みる
                    query = urlparse.parse_qs(post_data)
                    index_value = query.get('index', [None])[0]
                    
            self.process_record(index_value)
        else:
            self.send_error(404, "Not Found")

    def process_record(self, index_value):
        """座標を保存する共通処理"""
        global current_pose
        
        if index_value is None:
            self.send_json_response(400, {"status": "error", "message": "Index value is missing."})
            return

        # 最新の座標を取得
        with pose_lock:
            x = current_pose["x"]
            y = current_pose["y"]

        if x is None or y is None:
            self.send_json_response(503, {"status": "error", "message": "No odometry data received yet."})
            return

        # 保存するデータ構造
        waypoint_data = {
            "index": str(index_value),
            "x": x,
            "y": y
        }

        # JSON Linesとして追記
        try:
            with open(JSON_FILE, mode='a') as f:
                f.write(json.dumps(waypoint_data) + '\n')
            
            rospy.loginfo("Saved -> Index: %s, X: %f, Y: %f", index_value, x, y)
            self.send_json_response(200, {"status": "success", "recorded": waypoint_data})
        except Exception as e:
            rospy.logerr("File save error: %s", str(e))
            self.send_json_response(500, {"status": "error", "message": str(e)})

    def send_json_response(self, status_code, data_dict):
        """JSON形式でレスポンスを返すヘルパー関数"""
        self.send_response(status_code)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data_dict))

    def log_message(self, format, *args):
        """標準出力へのアクセスログを抑制したい場合はここをpassにするか、rospy.logdebugに変更"""
        pass

# --- ROS関連処理 ---
def odom_callback(msg):
    """常時流れてくるトピックから最新座標を更新"""
    global current_pose
    with pose_lock:
        current_pose["x"] = msg.pose.pose.position.x
        current_pose["y"] = msg.pose.pose.position.y

def http_server_thread():
    """HTTPサーバーを起動するスレッド用関数"""
    server_address = ('0.0.0.0', 5000)
    httpd = BaseHTTPServer.HTTPServer(server_address, WaypointRequestHandler)
    httpd.serve_forever()

if __name__ == '__main__':
    rospy.init_node('waypoint_recorder_node', anonymous=True)
    
    # 保存先ディレクトリの作成チェック
    if not os.path.exists(SAVE_DIR):
        try:
            os.makedirs(SAVE_DIR)
            rospy.loginfo("ディレクトリを作成しました: %s", SAVE_DIR)
        except Exception as e:
            rospy.logerr("ディレクトリの作成に失敗しました: %s", str(e))
            
    # Odometryのサブスクライブ開始
    rospy.Subscriber("atmobi/robot_pose", Odometry, odom_callback)
    
    # HTTPサーバーを別スレッドでバックグラウンド起動
    server_thread = threading.Thread(target=http_server_thread)
    server_thread.daemon = True
    server_thread.start()
    
    rospy.loginfo("Waypoint Recorder (標準ライブラリ版) が起動しました。")
    rospy.loginfo("待機中ポート: 5000, 保存先: %s", JSON_FILE)
    
    # ROSのメインループ（トピック受信と維持）
    rospy.spin()