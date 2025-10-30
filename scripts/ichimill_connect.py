#!/usr/bin/env python
# -*- coding: utf-8 -*-

import base64
import socket

import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence
from std_msgs.msg import String


class IchimillConnectNode(Node):
    def __init__(self):
        super().__init__('ichimill_connect')

        # パラメータの宣言と取得
        self.declare_parameter('debug', False)
        self.declare_parameter('username', '')
        self.declare_parameter('password', '')
        self.declare_parameter('port', 2101)
        self.declare_parameter('host', "ntrip.ales-corp.co.jp")
        self.declare_parameter('mountpoint', "32M7NHS")

        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.username = self.get_parameter('username').get_parameter_value().string_value
        self.password = self.get_parameter('password').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.host_url = self.get_parameter('host').get_parameter_value().string_value
        self.mountpoint = self.get_parameter('mountpoint').get_parameter_value().string_value

        # Publisher, Subscriber, Socketの初期化
        self.pub = self.create_publisher(String, '/softbank/rtcm_data', 10)
        self.tcpip = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcpip.settimeout(10.0)

        self.mutex_server = False

    def connect_to_caster(self):
        pwd = base64.b64encode(f"{self.username}:{self.password}".encode('ascii')).decode('ascii')
        header = (
            f"GET /{self.mountpoint} HTTP/1.1\r\n"
            f"Authorization: Basic {pwd}\r\n"
            "User-Agent: NTRIP ROS2Client/1.0\r\n"
            "Connection: close\r\n\r\n"
        )

        try:
            self.get_logger().info(f"NTRIP Caster connecting to {self.host_url}:{self.port}...")
            self.tcpip.connect((self.host_url, self.port))
            self.get_logger().info("OK")

            self.get_logger().info("Header sending...")
            self.tcpip.send(header.encode('ascii'))

            data = self.tcpip.recv(1024).decode('ascii')
            self.get_logger().info(f"Caster Response: {data.strip()}")
            if "ICY 200 OK" in data:
                self.subscription = self.create_subscription(Sentence, "/nmea_gga", self.cb_gga, 10)
                return True
            else:
                self.get_logger().error(f"Caster ResponseError!! : {data}")
                return False
        except socket.error as ex:
            self.get_logger().error(f"NTRIP Caster connect error: {ex}")
            return False
        except Exception as ex:
            self.get_logger().error(f"Exception error during connection: {ex}")
            return False

    def cb_gga(self, data):
        send_data = data.sentence

        if send_data.split(',').count('') > 1:
            self.get_logger().warn(f"Missing the necessary elements in GGA sentence: {send_data}")
            return

        if self.debug:
            self.get_logger().info(f"Sending to ichimill server: {send_data}")

        if self.mutex_server:
            return

        self.mutex_server = True
        try:
            self.tcpip.send(send_data.encode('ascii'))
            send_time = self.get_clock().now()

            rtk_datas_bytes = self.tcpip.recv(4096)
            if self.debug:
                self.get_logger().info(f"NTRIP data received: {len(rtk_datas_bytes)} bytes")

            response_delay = self.get_clock().now() - send_time
            if response_delay.nanoseconds > 3.0 * 1e9:
                self.get_logger().warn(f"NTRIP Caster Response Delay > 3.0s: {response_delay.nanoseconds} nsec on host: {self.host_url}")

            if rtk_datas_bytes:
                msg = String()
                # RTCMデータはバイナリなので、そのままstringに格納するのはベストではないが、
                # ROS1の挙動を維持するため、受信したbytesをそのままstringとして扱う
                msg.data = rtk_datas_bytes.decode('latin-1') # エラーを起こさずにbytesをstringに変換
                self.pub.publish(msg)

        except socket.timeout:
            self.get_logger().warn("NTRIP Caster timeout.")
        except Exception as ex:
            self.get_logger().error(f"Exception error: {ex}")
        finally:
            self.mutex_server = False

    def shutdown(self):
        self.tcpip.close()
        self.get_logger().info("NTRIP Caster disconnected")


def main(args=None):
    rclpy.init(args=args)
    node = IchimillConnectNode()
    if node.connect_to_caster():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
