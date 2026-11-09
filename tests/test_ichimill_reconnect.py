"""Network regression tests without ROS installation: python -m unittest discover -s tests."""
import ast
import base64
import math
from pathlib import Path
import select
import socket
import struct
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch


class FakeNode:
    def __init__(self, name):
        self.params = {}
        self.messages = []
        self.subscriptions = 0

    def declare_parameter(self, name, value):
        self.params[name] = value

    def get_parameter(self, name):
        return SimpleNamespace(value=self.params[name])

    def create_publisher(self, *args):
        return SimpleNamespace(publish=lambda msg: self.messages.append(bytes(msg.data)))

    def create_subscription(self, *args):
        self.subscriptions += 1
        return object()

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=time.time_ns()))

    def get_logger(self):
        return SimpleNamespace(info=lambda _: None, warn=lambda _: None)


# Execute the actual class with just the ROS boundary replaced.
source = Path(__file__).resolve().parents[1] / 'scripts' / 'ichimill_connect.py'
tree = ast.parse(source.read_text())
node_class = next(n for n in tree.body if isinstance(n, ast.ClassDef))
namespace = dict(Node=FakeNode, Sentence=object, UInt8MultiArray=SimpleNamespace,
                 base64=base64, math=math, select=select, socket=socket,
                 threading=threading, time=time,
                 rclpy=SimpleNamespace(ok=lambda: True))
exec(compile(ast.Module(body=[node_class], type_ignores=[]), str(source), 'exec'), namespace)
Client = namespace['IchimillConnectNode']


def gga(utc='123456', quality='1'):
    body = f'GPGGA,{utc},3500.000,N,13500.000,E,{quality},10,1.0,10.0,M,30.0,M,,'
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return SimpleNamespace(
        sentence=f'${body}*{checksum:02X}',
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)))


class ResponseSocket:
    def __init__(self, chunks):
        self.chunks = iter(chunks)

    def recv(self, size):
        chunk = next(self.chunks, b'')
        if isinstance(chunk, Exception):
            raise chunk
        return chunk


class ReconnectTests(unittest.TestCase):
    def test_fragmented_icy_binary(self):
        node = Client()
        payload = b'\xd3\x00\xff'
        self.assertEqual(node._read_response(
            ResponseSocket([b'IC', b'Y 200 OK\r', b'\n' + payload])), payload)

    def test_fragmented_http_binary(self):
        node = Client()
        payload = b'\xd3\xff\x80'
        self.assertEqual(node._read_response(ResponseSocket([
            b'HTTP/1.1 200 OK\r\nServer: test\r\n', b'\r\n' + payload])), payload)

    def test_rejected_and_closed_handshake(self):
        for response in (b'HTTP/1.1 401 Unauthorized\r\n\r\n', b''):
            with self.assertRaises(ConnectionError):
                Client()._read_response(ResponseSocket([response]))

    def test_handshake_timeout(self):
        node = Client()
        node.connect_timeout = 0.01
        class Silent:
            def recv(self, size):
                raise socket.timeout()
        with self.assertRaises(TimeoutError):
            node._read_response(Silent())

    def test_gga_validity_freshness_and_terminator(self):
        node = Client()
        node.cb_gga(gga())
        self.assertTrue(node._current_gga().endswith(b'\r\n'))
        self.assertFalse(node._current_gga().endswith(b'\r\n\r\n'))
        previous = node._latest_gga
        node.cb_gga(gga(quality='0'))
        bad = gga()
        bad.sentence = bad.sentence[:-2] + 'FF'
        node.cb_gga(bad)
        old = gga()
        old.header.stamp.sec = int(time.time()) - 60
        node.cb_gga(old)
        self.assertEqual(node._latest_gga, previous)
        node._latest_gga = (previous[0], time.monotonic() - 60)
        self.assertIsNone(node._current_gga())

    def exercise_reconnect(self, failure):
        node = Client()
        node.host_url = '127.0.0.1'
        node.reconnect_interval = 0.02
        node.connect_timeout = 0.5
        node.rtcm_timeout = 0.15
        node.gga_interval = 0.02
        node.cb_gga(gga())
        requests, positions, errors = [], [], []
        done = threading.Event()
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen()
        listener.settimeout(3)
        node.port = listener.getsockname()[1]

        def read_until(conn, marker):
            buf = b''
            while marker not in buf:
                part = conn.recv(4096)
                if not part:
                    raise AssertionError('Unexpected EOF')
                buf += part
            return buf

        def serve():
            try:
                for attempt in range(2):
                    conn, _ = listener.accept()
                    with conn:
                        conn.settimeout(3)
                        requests.append(read_until(conn, b'\r\n\r\n'))
                        conn.sendall(b'ICY 200 OK\r\n')
                        positions.append(read_until(conn, b'\r\n'))
                        if attempt == 0:
                            node.cb_gga(gga(utc='123457'))
                            if failure == 'reset':
                                conn.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,
                                                struct.pack('ii', 1, 0))
                            elif failure == 'silent':
                                # Keep TCP open until the client's RTCM watchdog closes it.
                                while conn.recv(4096):
                                    pass
                        else:
                            conn.sendall(b'\xd3\x00\xff')
                            done.set()
                            while not node._stop.wait(0.01):
                                pass
            except Exception as ex:
                errors.append(ex)
                done.set()

        server = threading.Thread(target=serve, daemon=True)
        server.start()
        try:
            node.start()
            self.assertTrue(done.wait(4), 'Reconnect did not complete')
            deadline = time.monotonic() + 1
            while not node.messages and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(errors, [])
            self.assertEqual(len(requests), 2)
            self.assertTrue(all(b'Authorization: Basic ' in r for r in requests))
            self.assertIn(b'123457', positions[1])
            self.assertEqual(b''.join(node.messages), b'\xd3\x00\xff')
            self.assertEqual(node.subscriptions, 1)
        finally:
            node.shutdown()
            server.join(timeout=4)
            listener.close()
        self.assertFalse(server.is_alive())

    def test_reconnect_after_eof(self):
        self.exercise_reconnect('eof')

    def test_reconnect_after_reset(self):
        self.exercise_reconnect('reset')

    def test_reconnect_after_silent_connection(self):
        self.exercise_reconnect('silent')

    def test_initial_connect_failure_is_retried(self):
        node = Client()
        node.reconnect_interval = 0.01
        attempts = []
        def fail(*args, **kwargs):
            attempts.append(1)
            if len(attempts) == 2:
                node._stop.set()
            raise ConnectionRefusedError('offline')
        with patch.object(socket, 'create_connection', side_effect=fail):
            node._run()
        self.assertEqual(len(attempts), 2)


if __name__ == '__main__':
    unittest.main()
