#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import NavSatFix

class Fix2CsvNode(Node):
    def __init__(self):
        super().__init__('fix2csv')

        # パラメータの宣言と取得
        self.declare_parameter('delay', 1.0)
        self.stamp_delay = self.get_parameter('delay').get_parameter_value().double_value

        # ファイルの準備
        try:
            share_dir = get_package_share_directory('f9p_ichimill')
        except Exception:
            share_dir = os.path.join(os.environ['HOME'], 'ros2_ws', 'src', 'f9p_ichimill')
            os.makedirs(share_dir, exist_ok=True)

        self.fname = os.path.join(share_dir, f"fix_{datetime.now():%Y%m%d_%H%M}.csv")
        self.file = open(self.fname, "w")
        self.get_logger().info(f"Start GPS /fix topic to csv file: {self.fname}")
        self.get_logger().info(f"  delay = {self.stamp_delay:.2f} sec")

        # 変数の初期化
        self.seq = 0
        self.old_time = Time(seconds=0, nanoseconds=0, clock_type=self.get_clock().clock_type)

        # Subscriberの作成
        self.subscription = self.create_subscription(NavSatFix, '/fix', self.cb_fix, 10)

    def cb_fix(self, msg):
        current_time = Time.from_msg(msg.header.stamp)
        
        # 設定した遅延時間以上経過しているかチェック
        if (current_time - self.old_time).nanoseconds / 1e9 > self.stamp_delay:
            self.seq += 1
            log_time = datetime.fromtimestamp(current_time.seconds_nanoseconds()[0])
            line_str = (
                f"{self.seq},{msg.latitude},{msg.longitude},"
                f"status={msg.status.status},service={msg.status.service},"
                f"stamp={log_time}\n"
            )
            self.file.write(line_str)
            self.old_time = current_time

    def on_shutdown(self):
        if self.file:
            self.file.close()
            self.get_logger().info(f"Closed {self.fname}")

def main(args=None):
    rclpy.init(args=args)
    node = Fix2CsvNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
