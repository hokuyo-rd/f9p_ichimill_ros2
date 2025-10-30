#!/usr/bin/env python
# -*- coding: utf-8 -*-

import base64
import socket

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

CLIENT_AGENT = "ros2_ntripcaster_connect"
BUF_SIZE = 4096

class NtripCasterConnectNode(Node):
    def __init__(self):
        super().__init__('ntripcaster_connect')

        # パラメータの宣言と取得
        self.declare_parameter('debug', False)
        self.declare_parameter('username', '')
        self.declare_parameter('password', 'BETATEST')
        self.declare_parameter('port', 2101)
        self.declare_parameter('host', "rtk2go.com")
        self.declare_parameter('mountpoint', "MIE_UNIV")

        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.username = self.get_parameter('username').get_parameter_value().string_value
        self.password = self.get_parameter('password').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.host_url = self.get_parameter('host').get_parameter_value().string_value
        self.mountpoint = self.get_parameter('mountpoint').get_parameter_value().string_value

        # Publisherの初期化
        self.pub = self.create_publisher(String, '/caster/rtcm_data', 10)
        self.tcpip = None

    def run(self):
        idpwd = base64.b64encode(f"{self.username}:{self.password}".encode('ascii')).decode('ascii')
        header = (
            f"GET /{self.mountpoint} HTTP/1.1\r\n"
            f"User-Agent: {CLIENT_AGENT}\r\n"
            f"Authorization: Basic {idpwd}\r\n\r\n"
        )

        while rclpy.ok():
            try:
                self.tcpip = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tcpip.settimeout(10.0)

                self.get_logger().info("NTRIP Caster connecting...")
                self.tcpip.connect((self.host_url, self.port))
                self.get_logger().info("OK")

                self.get_logger().info("Header sending...")
                self.tcpip.send(header.encode('ascii'))

                data = self.tcpip.recv(BUF_SIZE).decode('ascii', errors='ignore')

                if not data:
                    self.get_logger().warn("Header Response Fail... Retrying connection...")
                    self.tcpip.close()
                    self.create_timer(1.0, lambda: None) # 1秒待機
                    continue

                self.get_logger().info(f"Caster Response: {data.strip()}")
                response_header = data.strip()
                if "ICY 200 OK" in response_header or "HTTP/1.1 200 OK" in response_header:
                    self.get_logger().info("Header Response OK")
                else:
                    self.get_logger().error(f"NTRIP Caster ResponseError!! : {data}")
                    break # エラーの場合は終了

                # データ受信ループ
                rtk_datas = b''
                while rclpy.ok():
                    ntrip_response = self.tcpip.recv(BUF_SIZE)
                    if not ntrip_response:
                        self.get_logger().warn("NTRIP Caster Disconnected! Retrying connection...")
                        break # 接続が切れたら外側のループに戻って再接続

                    rtk_datas += ntrip_response
                    if len(ntrip_response) < BUF_SIZE:
                        if self.debug:
                            self.get_logger().info(f"NTRIP data received: {len(rtk_datas)} bytes")
                        
                        msg = String()
                        msg.data = rtk_datas.decode('latin-1')
                        self.pub.publish(msg)
                        rtk_datas = b''

            except socket.timeout:
                self.get_logger().warn("NTRIP Caster timeout. Retrying...")
            except socket.error as ex:
                self.get_logger().error(f"NTRIP Caster connect error: {ex}. Retrying...")
            except Exception as ex:
                self.get_logger().error(f"An unexpected error occurred: {ex}")
                break
            finally:
                if self.tcpip:
                    self.tcpip.close()
                if rclpy.ok():
                    self.get_logger().info("Waiting 1 second before reconnecting...")
                    # ROS 2ではrospy.Rateの代わりにタイマーやsleepを使う
                    # ここでは単純化のためrclpy.spin_onceを使い擬似的なsleepを実現
                    executor = SingleThreadedExecutor()
                    executor.add_node(self)
                    start_time = self.get_clock().now()
                    while (self.get_clock().now() - start_time).nanoseconds < 1e9:
                         executor.spin_once(timeout_sec=0.1)

    def on_shutdown(self):
        if self.tcpip:
            self.tcpip.close()
        self.get_logger().info("NTRIP Caster disconnected")

def main(args=None):
    rclpy.init(args=args)
    node = NtripCasterConnectNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
