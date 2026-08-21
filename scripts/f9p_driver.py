#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial

import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence
from std_msgs.msg import UInt8MultiArray
from ublox_msgs.msg import NavPVT
from rclpy.executors import SingleThreadedExecutor

from scripts.ubx_nav_pvt import (
    NAV_PVT_CLASS,
    NAV_PVT_ID,
    UbxNmeaParser,
    unpack_nav_pvt,
)

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
        self.pub_nav_pvt = self.create_publisher(NavPVT, 'nav_pvt', 10)
        self.subscription = self.create_subscription(UInt8MultiArray, "/softbank/rtcm_data", self.cb_rtcm_data, 10)

        self.seq_gga = 0
        self.seq_nmea = 0
        self.seq_zda = 0
        self.seq_rmc = 0
        self.rtcm_data = b""
        self.serial_parser = UbxNmeaParser()

    def cb_rtcm_data(self, msg: UInt8MultiArray):
        self.rtcm_data = bytes(msg.data)

    def publish_nav_pvt(self, payload):
        try:
            fields = unpack_nav_pvt(payload)
        except ValueError as ex:
            self.get_logger().warning(str(ex))
            return

        message = NavPVT()
        for name, value in fields.items():
            setattr(message, name, value)
        self.pub_nav_pvt.publish(message)

        if self.debug:
            self.get_logger().info(
                f"Published NAV-PVT: iTOW={message.i_tow}, "
                f"fixType={message.fix_type}, numSV={message.num_sv}"
            )

    def publish_nmea(self, gps_str):
        """Publish one NMEA sentence to its specific and aggregate topics."""
        if self.debug:
            self.get_logger().info(f"Received: {gps_str}")

        # GGAセンテンスの処理と発行
        if "GGA" in gps_str:
            send_data = gps_str
            if "$GNGGA" in gps_str:
                gga_string = gps_str.replace('$GNGGA', 'GPGGA')
                if '*' in gga_string:
                    gga_string = gga_string.split('*')[0]
                checksum = calcultateCheckSum(gga_string)
                send_data = f"${gga_string}*{checksum}\r\n"

            gga_sentence = Sentence()
            gga_sentence.header.stamp = self.get_clock().now().to_msg()
            gga_sentence.header.frame_id = 'gps'
            gga_sentence.sentence = send_data
            self.pub_gga.publish(gga_sentence)
            self.seq_gga += 1

        for marker, publisher, sequence_name in (
                ("ZDA", self.pub_zda, "seq_zda"),
                ("RMC", self.pub_rmc, "seq_rmc")):
            if marker in gps_str:
                sentence = Sentence()
                sentence.header.stamp = self.get_clock().now().to_msg()
                sentence.header.frame_id = 'gps'
                sentence.sentence = gps_str
                publisher.publish(sentence)
                setattr(self, sequence_name, getattr(self, sequence_name) + 1)

        nmea_sentence = Sentence()
        nmea_sentence.header.stamp = self.get_clock().now().to_msg()
        nmea_sentence.header.frame_id = 'gps'
        nmea_sentence.sentence = gps_str
        self.pub_nmea.publish(nmea_sentence)
        self.seq_nmea += 1

    def run(self):
        try:
            self.get_logger().info("Serial port opening...")
            gps_serial = serial.Serial(port=self.serial_port, baudrate=self.serial_baud, timeout=0.1)
            self.get_logger().info(f"OK. Port: {self.serial_port}, Baudrate: {self.serial_baud}")
            
            executor = SingleThreadedExecutor()
            executor.add_node(self)

            try:
                while rclpy.ok():
                
                    executor.spin_once(timeout_sec=0.0)
                    
                    serial_data = gps_serial.read(gps_serial.in_waiting or 1)
                    nmea_sentences, ubx_frames = self.serial_parser.feed(serial_data)

                    for gps_str in nmea_sentences:
                        self.publish_nmea(gps_str)

                    for message_class, message_id, payload in ubx_frames:
                        if (message_class, message_id) == (NAV_PVT_CLASS, NAV_PVT_ID):
                            self.publish_nav_pvt(payload)

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
