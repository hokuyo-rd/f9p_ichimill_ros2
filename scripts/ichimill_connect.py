#!/usr/bin/env python
# -*- coding: utf-8 -*-

import base64
import math
import select
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nmea_msgs.msg import Sentence
from std_msgs.msg import UInt8MultiArray


class IchimillConnectNode(Node):
    def __init__(self):
        super().__init__('ichimill_connect')
        defaults = {
            'debug': False, 'username': '', 'password': '',
            'port': 2101, 'host': 'ntrip.ales-corp.co.jp',
            'mountpoint': '32M7NHS', 'reconnect_interval': 5.0,
            'connect_timeout': 10.0, 'rtcm_timeout': 15.0,
            'gga_interval': 1.0, 'gga_max_age': 5.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.debug = self.get_parameter('debug').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value
        self.port = self.get_parameter('port').value
        self.host_url = self.get_parameter('host').value
        self.mountpoint = self.get_parameter('mountpoint').value
        for name in ('reconnect_interval', 'connect_timeout', 'rtcm_timeout',
                     'gga_interval', 'gga_max_age'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
            setattr(self, name, value)

        self.pub = self.create_publisher(UInt8MultiArray, '/softbank/rtcm_data', 10)
        # Only keep the newest GGA; never create another subscription on reconnect.
        self.subscription = self.create_subscription(
            Sentence, '/nmea_gga', self.cb_gga, 1)
        self._gga_lock = threading.Lock()
        self._latest_gga = None
        self._stop = threading.Event()
        self._worker = None

    def cb_gga(self, data):
        sentence = data.sentence.rstrip('\r\n')
        fields = sentence.split(',')
        try:
            if (len(fields) < 10 or fields[0] not in ('$GPGGA', '$GNGGA')
                    or not all(fields[i] for i in (1, 2, 3, 4, 5, 6))
                    or int(fields[6]) <= 0):
                return
            body, checksum = sentence[1:].split('*')
            calculated = 0
            for char in body:
                calculated ^= ord(char)
            if len(checksum) != 2 or calculated != int(checksum, 16):
                return
            # Reject queued, old fixes as well as fixes that go stale in our cache.
            stamp = data.header.stamp
            stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
            if stamp_ns:
                age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
                if age > self.gga_max_age or age < -self.gga_max_age:
                    return
            encoded = (sentence + '\r\n').encode('ascii')
        except (ValueError, UnicodeEncodeError):
            return
        with self._gga_lock:
            self._latest_gga = (encoded, time.monotonic())

    def _current_gga(self):
        with self._gga_lock:
            latest = self._latest_gga
        if latest and time.monotonic() - latest[1] <= self.gga_max_age:
            return latest[0]
        return None

    def _read_response(self, sock):
        """Read only the response header, preserving any coalesced binary RTCM."""
        buffer = b''
        deadline = time.monotonic() + self.connect_timeout
        while not self._stop.is_set():
            if time.monotonic() >= deadline:
                raise TimeoutError('NTRIP response header timeout')
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError('Caster closed during handshake')
            buffer += chunk
            if b'\r\n' not in buffer:
                if len(buffer) > 16384:
                    raise ValueError('NTRIP response header too large')
                continue
            status, rest = buffer.split(b'\r\n', 1)
            parts = status.split()
            if len(parts) < 2 or parts[1] != b'200':
                raise ConnectionError(
                    f'Caster rejected request: {status.decode("ascii", errors="replace")}')
            if parts[0] == b'ICY':
                # NTRIP v1 terminates its response with the status-line CRLF.
                return rest
            if parts[0] not in (b'HTTP/1.0', b'HTTP/1.1'):
                raise ConnectionError('Unsupported NTRIP response')
            if b'\r\n\r\n' in buffer:
                header, payload = buffer.split(b'\r\n\r\n', 1)
                if b'transfer-encoding:' in header.lower():
                    raise ConnectionError('Transfer-Encoding is unsupported; use an NTRIP v1 stream')
                return payload
            if len(buffer) > 16384:
                raise ValueError('NTRIP response header too large')
        raise ConnectionAbortedError('Stopping NTRIP client')

    def _publish_rtcm(self, payload):
        if payload and not self._stop.is_set() and rclpy.ok():
            msg = UInt8MultiArray()
            msg.data = list(payload)
            self.pub.publish(msg)
            if self.debug:
                self.get_logger().info(f'NTRIP data received: {len(payload)} bytes')

    def _stream(self, sock, initial_payload):
        self._publish_rtcm(initial_payload)
        last_received = time.monotonic()
        next_gga = 0.0  # Send the latest position immediately after every handshake.
        while not self._stop.is_set() and rclpy.ok():
            now = time.monotonic()
            if now >= next_gga:
                gga = self._current_gga()
                if gga:
                    sock.sendall(gga)
                    if self.debug:
                        self.get_logger().info('GGA sent to ichimill')
                next_gga = now + self.gga_interval
            if now - last_received >= self.rtcm_timeout:
                raise TimeoutError('No RTCM data received within rtcm_timeout')
            readable, _, _ = select.select(
                [sock], [], [], min(0.2, max(0.0, next_gga - now)))
            if readable:
                payload = sock.recv(4096)
                if not payload:
                    raise ConnectionError('NTRIP caster closed the connection')
                self._publish_rtcm(payload)
                last_received = time.monotonic()

    def _run(self):
        credentials = base64.b64encode(
            f'{self.username}:{self.password}'.encode('ascii')).decode('ascii')
        request = (
            f'GET /{self.mountpoint} HTTP/1.0\r\n'
            f'Host: {self.host_url}:{self.port}\r\n'
            f'Authorization: Basic {credentials}\r\n'
            'User-Agent: NTRIP ROS2Client/1.0\r\n'
            'Connection: close\r\n\r\n'
        ).encode('ascii')
        while not self._stop.is_set() and rclpy.ok():
            try:
                self.get_logger().info(
                    f'NTRIP connecting to {self.host_url}:{self.port}/{self.mountpoint}')
                # A fresh socket and a fresh authentication request on every attempt.
                with socket.create_connection(
                        (self.host_url, self.port), timeout=self.connect_timeout) as sock:
                    sock.settimeout(0.5)
                    sock.sendall(request)
                    payload = self._read_response(sock)
                    self.get_logger().info('NTRIP authenticated; receiving RTCM')
                    self._stream(sock, payload)
            except (OSError, ValueError) as ex:
                if not self._stop.is_set():
                    self.get_logger().warn(
                        f'NTRIP disconnected: {ex}; reconnecting in '
                        f'{self.reconnect_interval}s')
            # Event.wait allows shutdown to interrupt the reconnect delay.
            self._stop.wait(self.reconnect_interval)

    def start(self):
        if self._worker is None:
            self._worker = threading.Thread(
                target=self._run, name='ichimill-network', daemon=True)
            self._worker.start()

    def shutdown(self):
        self._stop.set()
        if self._worker is not None:
            # Join before destroying the ROS publisher used by the worker.
            self._worker.join()
            self._worker = None


def main(args=None):
    rclpy.init(args=args)
    node = IchimillConnectNode()
    try:
        node.start()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
