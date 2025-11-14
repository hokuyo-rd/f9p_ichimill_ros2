#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial

import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence
from std_msgs.msg import String


def calcultateCheckSum(stringToCheck):
    xsum_calc = 0
    for char in stringToCheck:
        xsum_calc = xsum_calc ^ ord(char)
    return "%02X" % xsum_calc


class F9PDriverNode(Node):
    def __init__(self):
        super().__init__('f9p_driver')

        # パラメータの宣言と取得
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 230400)
        self.declare_parameter('debug', False)

        self.serial_port = self.get_parameter('port').get_parameter_value().string_value
        self.serial_baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        # Publisher, Subscriberの初期化
        self.pub_gga = self.create_publisher(Sentence, 'nmea_gga', 10)
        self.pub_nmea = self.create_publisher(Sentence, 'nmea_sentence', 10)
        self.pub_zda = self.create_publisher(Sentence, 'nmea_zda', 10)
        self.pub_rmc = self.create_publisher(Sentence, 'nmea_rmc', 10)
        self.subscription = self.create_subscription(String, "/softbank/rtcm_data", self.cb_rtcm_data, 10)

        self.seq_gga = 0
        self.seq_nmea = 0
        self.seq_zda = 0
        self.seq_rmc = 0
        self.rtcm_data = b""

    def cb_rtcm_data(self, msg):
        # ROS 2のichimill_connect.pyはlatin-1でエンコードしているため、
        # それに合わせてデコードしてbytesに戻す
        self.rtcm_data = msg.data.encode('latin-1')

    def run(self):
        try:
            self.get_logger().info("Serial port opening...")
            gps_serial = serial.Serial(port=self.serial_port, baudrate=self.serial_baud, timeout=2)
            self.get_logger().info(f"OK. Port: {self.serial_port}, Baudrate: {self.serial_baud}")

            try:
                while rclpy.ok():
                    try:
                        gps_line = gps_serial.readline()
                        gps_str = gps_line.decode('ascii').strip()
                    except (UnicodeDecodeError, serial.SerialException):
                        # デコードエラーやシリアルエラーは無視して次の行を読む
                        continue

                    if not gps_str:
                        continue

                    if self.debug:
                        self.get_logger().info(f"Received: {gps_str}")

                    # GGAセンテンスの処理と発行
                    if "GGA" in gps_str:
                        send_data = gps_str
                        if "$GNGGA" in gps_str:
                            # チェックサム再計算
                            gga_string = gps_str.replace('$GNGGA', 'GPGGA')
                            if '*' in gga_string:
                                gga_string = gga_string.split('*')[0]
                            checksum = calcultateCheckSum(gga_string[1:]) # $を除いて計算
                            send_data = f"{gga_string}*{checksum}\r\n"

                        gga_sentence = Sentence()
                        gga_sentence.header.stamp = self.get_clock().now().to_msg()
                        gga_sentence.header.frame_id = 'gps'
                        # seqはROS 2では非推奨だが、互換性のために残す
                        # gga_sentence.header.seq = self.seq_gga
                        gga_sentence.sentence = send_data
                        self.pub_gga.publish(gga_sentence)
                        self.seq_gga += 1

                    # ZDAセンテンスの処理と発行
                    if "ZDA" in gps_str:
                        zda_sentence = Sentence()
                        zda_sentence.header.stamp = self.get_clock().now().to_msg()
                        zda_sentence.header.frame_id = 'gps'
                        # seqはROS 2では非推奨
                        zda_sentence.sentence = gps_str
                        self.pub_zda.publish(zda_sentence)
                        self.seq_zda += 1

                    # RMCセンテンスの処理と発行
                    if "RMC" in gps_str:
                        rmc_sentence = Sentence()
                        rmc_sentence.header.stamp = self.get_clock().now().to_msg()
                        rmc_sentence.header.frame_id = 'gps'
                        # seqはROS 2では非推奨
                        rmc_sentence.sentence = gps_str
                        self.pub_rmc.publish(rmc_sentence)
                        self.seq_rmc += 1

                    # 全てのNMEAセンテンスを発行
                    nmea_sentence = Sentence()
                    nmea_sentence.header.stamp = self.get_clock().now().to_msg()
                    nmea_sentence.header.frame_id = 'gps'
                    # nmea_sentence.header.seq = self.seq_nmea
                    nmea_sentence.sentence = gps_str
                    self.pub_nmea.publish(nmea_sentence)
                    self.seq_nmea += 1

                    # Ntrip CasterからのRTCMデータをF9Pに送信
                    if len(self.rtcm_data) > 0:
                        gps_serial.write(self.rtcm_data)
                        if self.debug:
                            self.get_logger().info(f"Wrote {len(self.rtcm_data)} bytes of RTCM data to F9P.")
                        self.rtcm_data = b""

            except serial.SerialException as ex:
                self.get_logger().error(f"SerialException error: {ex}")
            finally:
                gps_serial.close()
                self.get_logger().info("Serial port closed.")

        except serial.SerialException as ex:
            self.get_logger().error(f"Could not open serial port: {ex}")

def main(args=None):
    rclpy.init(args=args)
    node = F9PDriverNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
