#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial

import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import NavSatFix, NavSatStatus
from ublox_msgs.msg import NavPVT
from rclpy.executors import SingleThreadedExecutor

from scripts.ubx import GnssStreamParser, decode_nav_pvt

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
        self.pub_nav_pvt = self.create_publisher(NavPVT, 'ubx_nav_pvt', 10)
        self.pub_nav_sat_fix = self.create_publisher(NavSatFix, 'nav_pvt_fix', 10)
        self.subscription = self.create_subscription(UInt8MultiArray, "/softbank/rtcm_data", self.cb_rtcm_data, 10)

        self.seq_gga = 0
        self.seq_nmea = 0
        self.seq_zda = 0
        self.seq_rmc = 0
        self.rtcm_data = b""
        self.stream_parser = GnssStreamParser()

    def cb_rtcm_data(self, msg: UInt8MultiArray):
        self.rtcm_data = bytes(msg.data)

    def process_nav_pvt(self, frame):
        """Publish NAV-PVT using its native message and as a standard fix."""
        try:
            pvt = decode_nav_pvt(frame)
        except ValueError as ex:
            self.get_logger().warning(f"Invalid UBX-NAV-PVT message: {ex}")
            return

        nav_pvt = NavPVT()
        nav_pvt.header.stamp = self.get_clock().now().to_msg()
        nav_pvt.header.frame_id = 'gps'
        for field in (
                'i_tow', 'year', 'month', 'day', 'hour', 'min', 'sec', 'valid',
                't_acc', 'nano', 'fix_type', 'flags', 'flags2', 'num_sv', 'lon',
                'lat', 'h_msl', 'h_acc', 'v_acc', 'vel_n', 'vel_e', 'vel_d',
                'g_speed', 'head_mot', 's_acc', 'head_acc', 'p_dop', 'reserved1',
                'head_veh', 'mag_dec', 'mag_acc'):
            setattr(nav_pvt, field, pvt[field])
        nav_pvt.height = pvt['height_raw']
        self.pub_nav_pvt.publish(nav_pvt)

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps'
        fix.status.status = (NavSatStatus.STATUS_FIX
                             if pvt["gnss_fix_ok"] and pvt["fix_type"] >= 2
                             else NavSatStatus.STATUS_NO_FIX)
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = pvt["latitude"]
        fix.longitude = pvt["longitude"]
        fix.altitude = pvt["height"]
        fix.position_covariance[0] = pvt["horizontal_accuracy"] ** 2
        fix.position_covariance[4] = pvt["horizontal_accuracy"] ** 2
        fix.position_covariance[8] = pvt["vertical_accuracy"] ** 2
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_nav_sat_fix.publish(fix)

        if self.debug:
            self.get_logger().info(
                f"UBX-NAV-PVT: lat={fix.latitude:.7f}, "
                f"lon={fix.longitude:.7f}, height={fix.altitude:.3f} m, "
                f"fixType={pvt['fix_type']}")

    def process_nmea(self, gps_str):
        """Process and publish an NMEA sentence."""
        if not gps_str:
            return

        if self.debug:
            self.get_logger().info(f"Received NMEA: {gps_str}")

        # GGAセンテンスの処理と発行
        if "GGA" in gps_str:
            send_data = gps_str
            if "$GNGGA" in gps_str:
                gga_string = gps_str.replace('$GNGGA', 'GPGGA')
                if '*' in gga_string:
                    gga_string = gga_string.split('*')[0]
                checksum = calcultateCheckSum(gga_string)
                send_data = f"${gga_string}*{checksum}\r\n"

            gga_sentence = self.make_nmea_message(send_data)
            self.pub_gga.publish(gga_sentence)
            self.seq_gga += 1

        if "ZDA" in gps_str:
            self.pub_zda.publish(self.make_nmea_message(gps_str))
            self.seq_zda += 1

        if "RMC" in gps_str:
            self.pub_rmc.publish(self.make_nmea_message(gps_str))
            self.seq_rmc += 1

        self.pub_nmea.publish(self.make_nmea_message(gps_str))
        self.seq_nmea += 1

    def make_nmea_message(self, sentence):
        message = Sentence()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'gps'
        message.sentence = sentence
        return message

    def run(self):
        try:
            self.get_logger().info("Serial port opening...")
            gps_serial = serial.Serial(port=self.serial_port, baudrate=self.serial_baud, timeout=2)
            self.get_logger().info(f"OK. Port: {self.serial_port}, Baudrate: {self.serial_baud}")
            
            executor = SingleThreadedExecutor()
            executor.add_node(self)

            try:
                while rclpy.ok():
                
                    executor.spin_once(timeout_sec=0.0)
                    
                    try:
                        serial_data = gps_serial.read(gps_serial.in_waiting or 1)
                    except serial.SerialException:
                        continue

                    for message_type, message in self.stream_parser.feed(serial_data):
                        if message_type == "nmea":
                            try:
                                self.process_nmea(message.decode('ascii'))
                            except UnicodeDecodeError:
                                continue
                        elif message[2] == 0x01 and message[3] == 0x07:
                            self.process_nav_pvt(message)

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
