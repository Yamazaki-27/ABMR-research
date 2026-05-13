#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import sys
from datetime import datetime
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer

import rospy
from std_msgs.msg import Int32


ros_publisher_value1 = None
ros_publisher_value2 = None


def publish_values(value1, value2):
    """
    受信した2つのDレジスタ値をROS 1トピックへpublishし、
    画面にも表示する
    """

    msg1 = Int32()
    msg1.data = value1

    msg2 = Int32()
    msg2.data = value2

    ros_publisher_value1.publish(msg1)
    ros_publisher_value2.publish(msg2)

    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    text = "[{}] 受信Dレジスタ値: value1={}, value2={}  -> ROS topic publish: /plc/d_register1, /plc/d_register2".format(
        now,
        value1,
        value2
    )

    print(text)
    sys.stdout.flush()


class PlcDRegisterRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        """
        Node-REDから以下のようなJSONを受け取る想定

        {
            "value1": 1234,
            "value2": 5678
        }
        """

        if self.path != "/plc_d_register":
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
            self.send_json_response(200, {
                "status": "running",
                "message": "PLC D register bridge is running",
                "post_url": "/plc_d_register",
                "ros_topic1": "/plc/d_register1",
                "ros_topic2": "/plc/d_register2"
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


if __name__ == "__main__":
    rospy.init_node("plc_d_register_bridge", anonymous=False)

    ros_publisher_value1 = rospy.Publisher(
        "/plc/d_register1",
        Int32,
        queue_size=10
    )

    ros_publisher_value2 = rospy.Publisher(
        "/plc/d_register2",
        Int32,
        queue_size=10
    )

    server_address = ("0.0.0.0", 5001)
    httpd = HTTPServer(server_address, PlcDRegisterRequestHandler)

    print("PLC Dレジスタ受信サーバを起動しました")
    print("受信URL: http://0.0.0.0:5001/plc_d_register")
    print("確認URL: http://0.0.0.0:5001/status")
    print("ROS topic1: /plc/d_register1")
    print("ROS topic2: /plc/d_register2")
    print("Node-REDからPOSTされるたびに、2つの値をここに表示します")
    print("終了するには Ctrl + C を押してください")
    sys.stdout.flush()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("")
        print("終了します")
    finally:
        httpd.server_close()