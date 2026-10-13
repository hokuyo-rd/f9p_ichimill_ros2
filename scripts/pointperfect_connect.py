#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""u-blox PointPerfect (Thingstream) から SPARTN 補正を取得する ROS 2 ノード。

PointPerfect には配信方式が 2 つあり、Thingstream の Location Thing の
Credentials タブに表示されている方を使う。

  mode: "ntrip" (既定)
    NTRIP Credentials (Endpoint / Mountpoint / Username / Password) が
    表示されている契約。ppntrip.services.u-blox.com へ NTRIP 接続し、
    SPARTN をそのまま受け取る。NEAR-* マウントポイントは受信機位置に
    応じたタイルを配信するため、GGA をキャスタへ送る必要がある。
    NTRIP 配信では認証がキャスタ側で行われ、動的キー (UBX-RXM-SPARTNKEY)
    は配布されないため、受信機へキーを投入する必要はない。

  mode: "mqtt"
    MQTT Credentials (Client ID / 証明書 / 秘密鍵) が表示されている契約。
    pp.services.u-blox.com へ TLS 接続し、以下を購読する。
      /pp/ip/<region>   : SPARTN 2.0.1 補正データ
      /pp/ubx/0236/ip   : UBX-RXM-SPARTNKEY (動的復号キー。補正の利用に必須)
      /pp/ubx/mga       : AssistNow Online (任意)

いずれの方式でも、受信したバイト列は加工せず UInt8MultiArray として
発行し、f9p_driver が ZED-F9P のシリアルポートへ書き込む。
"""

import base64
import json
import os
import socket
import ssl
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence
from std_msgs.msg import UInt8MultiArray

# --- NTRIP 配信 (PointPerfect NTRIP service) ---
DEFAULT_NTRIP_HOST = "ppntrip.services.u-blox.com"
DEFAULT_NTRIP_PORT = 2102          # 2101 = HTTP, 2102 = HTTPS(TLS)
DEFAULT_MOUNTPOINT = "NEAR-SPARTN"
CLIENT_AGENT = "NTRIP ros2_pointperfect_connect/1.0"
RECV_SIZE = 4096

# --- MQTT 配信 ---
DEFAULT_MQTT_HOST = "pp.services.u-blox.com"
DEFAULT_MQTT_PORT = 8883
DEFAULT_REGION = "jp"


def find_in_json(data, target_key):
    """ネストした dict/list から指定キーの値を再帰的に探す。

    Thingstream が配布する JSON 設定は階層構造が版によって
    異なるため、キー名で走査して取り出す。
    """
    if isinstance(data, dict):
        for key, value in data.items():
            if key.lower() == target_key.lower() and isinstance(value, str):
                return value
            found = find_in_json(value, target_key)
            if found is not None:
                return found
    elif isinstance(data, list):
        for item in data:
            found = find_in_json(item, target_key)
            if found is not None:
                return found
    return None


class ChunkedDecoder:
    """NTRIP v2 (HTTP/1.1) の chunked transfer-encoding を解く。

    NTRIP v1 の生ストリームと異なり、v2 ではチャンク長のヘッダが
    挟まる。これを取り除かないと SPARTN が壊れて受信機に渡ってしまう。
    """

    def __init__(self):
        self.buf = b""
        self.state = "size"
        self.remaining = 0
        self.finished = False

    def feed(self, data: bytes) -> bytes:
        self.buf += data
        out = bytearray()
        while not self.finished:
            if self.state == "size":
                index = self.buf.find(b"\r\n")
                if index < 0:
                    break
                line = self.buf[:index]
                self.buf = self.buf[index + 2:]
                token = line.split(b";")[0].strip()
                if not token:
                    continue  # チャンク間の空行
                try:
                    self.remaining = int(token, 16)
                except ValueError:
                    raise ValueError(f"Invalid chunk size: {line!r}")
                if self.remaining == 0:
                    self.finished = True
                    break
                self.state = "data"
            elif self.state == "data":
                if not self.buf:
                    break
                take = min(self.remaining, len(self.buf))
                out += self.buf[:take]
                self.buf = self.buf[take:]
                self.remaining -= take
                if self.remaining == 0:
                    self.state = "crlf"
            else:  # "crlf": チャンク末尾の CRLF を捨てる
                if len(self.buf) < 2:
                    break
                self.buf = self.buf[2:]
                self.state = "size"
        return bytes(out)


class PointPerfectConnectNode(Node):
    def __init__(self):
        super().__init__('pointperfect_connect')

        # --- 共通 ---
        self.declare_parameter('mode', 'ntrip')          # ntrip / mqtt
        self.declare_parameter('debug', False)
        self.declare_parameter('correction_topic', '/pointperfect/correction_data')

        # --- NTRIP 配信 ---
        self.declare_parameter('ntrip_host', DEFAULT_NTRIP_HOST)
        self.declare_parameter('ntrip_port', DEFAULT_NTRIP_PORT)
        self.declare_parameter('ntrip_mountpoint', DEFAULT_MOUNTPOINT)
        self.declare_parameter('ntrip_username', '')
        self.declare_parameter('ntrip_password', '')
        # HTTPS ポート (2102) を使う場合は true
        self.declare_parameter('ntrip_use_tls', True)
        # NEAR-* マウントポイントは位置に応じたタイルを配信するため GGA が要る
        self.declare_parameter('gga_topic', '/nmea_gga')
        self.declare_parameter('gga_interval_sec', 5.0)
        # 接続前に GGA を待つか。PointPerfect の NEAR-* は位置が無くても
        # 配信を始めるため既定では待たない (u-center と同じ挙動)。
        self.declare_parameter('require_gga', False)
        # 受信機が測位できていないときにキャスタへ送る代替位置 [緯度, 経度, 標高]。
        # NEAR-* は最初に有効な位置を受け取ってタイルを決めるため、未測位の
        # まま新規に接続すると配信が始まらないことがある。空なら無効。
        self.declare_parameter('fallback_position', [0.0, 0.0, 0.0])
        # この秒数より古い GGA は「測位を失った」とみなす。古い位置を
        # 送り続けてもキャスタは配信を止めるため、待ち状態へ戻す。
        self.declare_parameter('gga_timeout_sec', 30.0)
        # この秒数、補正データが届かなければ接続を張り直す
        self.declare_parameter('data_timeout_sec', 30.0)
        self.declare_parameter('reconnect_interval_sec', 5.0)

        # --- MQTT 配信 ---
        self.declare_parameter('host', DEFAULT_MQTT_HOST)
        self.declare_parameter('port', DEFAULT_MQTT_PORT)
        self.declare_parameter('region', DEFAULT_REGION)
        self.declare_parameter('client_id', '')
        self.declare_parameter('cert_file', '')
        self.declare_parameter('key_file', '')
        self.declare_parameter('ca_file', '')
        self.declare_parameter('credentials_file', '')
        self.declare_parameter('correction_mqtt_topics', [''])
        self.declare_parameter('key_mqtt_topic', '/pp/ubx/0236/ip')
        self.declare_parameter('enable_assistnow', False)
        self.declare_parameter('assistnow_mqtt_topic', '/pp/ubx/mga')
        self.declare_parameter('keepalive', 60)

        get = self.get_parameter
        self.mode = get('mode').get_parameter_value().string_value.lower()
        self.debug = get('debug').get_parameter_value().bool_value
        self.correction_topic = get('correction_topic').get_parameter_value().string_value

        self.ntrip_host = get('ntrip_host').get_parameter_value().string_value
        self.ntrip_port = get('ntrip_port').get_parameter_value().integer_value
        self.mountpoint = get('ntrip_mountpoint').get_parameter_value().string_value
        self.ntrip_username = get('ntrip_username').get_parameter_value().string_value
        self.ntrip_password = get('ntrip_password').get_parameter_value().string_value
        self.ntrip_use_tls = get('ntrip_use_tls').get_parameter_value().bool_value
        self.gga_topic = get('gga_topic').get_parameter_value().string_value
        self.gga_interval = get('gga_interval_sec').get_parameter_value().double_value
        self.require_gga = get('require_gga').get_parameter_value().bool_value
        position = list(get('fallback_position').get_parameter_value().double_array_value)
        self.fallback_gga = None
        if len(position) >= 2 and (position[0] or position[1]):
            altitude = position[2] if len(position) > 2 else 0.0
            self.fallback_position = (position[0], position[1], altitude)
        else:
            self.fallback_position = None
        self.gga_timeout = get('gga_timeout_sec').get_parameter_value().double_value
        self.data_timeout = get('data_timeout_sec').get_parameter_value().double_value
        self.reconnect_interval = get('reconnect_interval_sec').get_parameter_value().double_value

        self.host = get('host').get_parameter_value().string_value
        self.port = get('port').get_parameter_value().integer_value
        self.region = get('region').get_parameter_value().string_value.lower()
        self.client_id = get('client_id').get_parameter_value().string_value
        self.cert_file = get('cert_file').get_parameter_value().string_value
        self.key_file = get('key_file').get_parameter_value().string_value
        self.ca_file = get('ca_file').get_parameter_value().string_value
        self.credentials_file = get('credentials_file').get_parameter_value().string_value
        self.key_mqtt_topic = get('key_mqtt_topic').get_parameter_value().string_value
        self.enable_assistnow = get('enable_assistnow').get_parameter_value().bool_value
        self.assistnow_mqtt_topic = get('assistnow_mqtt_topic').get_parameter_value().string_value
        self.keepalive = get('keepalive').get_parameter_value().integer_value

        mqtt_topics = [t for t in
                       get('correction_mqtt_topics').get_parameter_value().string_array_value if t]
        self.correction_mqtt_topics = mqtt_topics or [f"/pp/ip/{self.region}"]

        self.pub = self.create_publisher(UInt8MultiArray, self.correction_topic, 50)

        self.correction_bytes_total = 0
        self.spartn_key_received = False

        # NTRIP 用の状態
        self.socket = None
        self.socket_lock = threading.Lock()
        self.latest_gga = None
        self.latest_gga_time = 0.0
        self.last_gga_sent = 0.0
        self.gga_warned = False
        self.running = True
        self.thread = None

        # MQTT 用の状態
        self.client = None
        self._temp_files = []

        if self.mode == 'ntrip':
            self.create_subscription(Sentence, self.gga_topic, self.cb_gga, 10)
        elif self.credentials_file:
            self.load_credentials_file(self.credentials_file)

    # ======================================================================
    # NTRIP 配信
    # ======================================================================
    def cb_gga(self, msg: Sentence):
        """受信機の GGA を保持する。キャスタへの送信は受信スレッドが行う。

        測位できていない GGA (緯度経度が空) もそのまま保持して送る。
        PointPerfect の NEAR-* マウントポイントは位置が無くても配信を
        始めるため、u-center も同様に中身を問わず送っている。
        """
        sentence = msg.sentence.strip()
        if not sentence.startswith('$'):
            return
        if self.debug and sentence.split(',').count('') > 1:
            self.get_logger().info(
                f"GGA has no position yet (sent anyway): {sentence}",
                throttle_duration_sec=30.0)
        self.latest_gga = sentence + "\r\n"
        self.latest_gga_time = time.monotonic()

    @staticmethod
    def build_gga(latitude: float, longitude: float, altitude: float) -> str:
        """緯度経度から有効な GGA センテンスを組み立てる。"""
        now = time.gmtime()
        hhmmss = f"{now.tm_hour:02d}{now.tm_min:02d}{now.tm_sec:02d}.00"
        lat_hemi = 'N' if latitude >= 0 else 'S'
        lon_hemi = 'E' if longitude >= 0 else 'W'
        lat, lon = abs(latitude), abs(longitude)
        lat_field = f"{int(lat):02d}{(lat - int(lat)) * 60:08.5f}"
        lon_field = f"{int(lon):03d}{(lon - int(lon)) * 60:08.5f}"
        body = (f"GPGGA,{hhmmss},{lat_field},{lat_hemi},{lon_field},{lon_hemi},"
                f"1,12,1.0,{altitude:.1f},M,0.0,M,,")
        checksum = 0
        for char in body:
            checksum ^= ord(char)
        return f"${body}*{checksum:02X}\r\n"

    @staticmethod
    def gga_has_position(sentence: str) -> bool:
        """GGA に緯度経度が入っているか。"""
        fields = sentence.split(',')
        return len(fields) > 5 and bool(fields[2]) and bool(fields[4])

    def gga_to_send(self):
        """キャスタへ送る GGA を選ぶ。

        受信機が測位していればそれを使う。測位していない場合、代替位置が
        設定されていればそちらを送る。NEAR-* は最初の位置でタイルを決める
        ため、未測位のまま新規接続すると配信が始まらないことがあるため。
        """
        if self.latest_gga is not None and self.gga_has_position(self.latest_gga):
            return self.latest_gga, 'receiver'
        if self.fallback_position is not None:
            return self.build_gga(*self.fallback_position), 'fallback'
        return self.latest_gga, 'receiver'

    def gga_is_fresh(self) -> bool:
        """有効な測位解が今も得られているか。

        受信機が測位を失うと GGA の必須項目が欠けるようになり cb_gga で
        弾かれる。その場合 latest_gga は古いままなので、時刻で判定する。
        """
        return (self.latest_gga is not None
                and (time.monotonic() - self.latest_gga_time) < self.gga_timeout)

    def build_ntrip_request(self) -> bytes:
        credential = base64.b64encode(
            f"{self.ntrip_username}:{self.ntrip_password}".encode('ascii')).decode('ascii')
        return (
            f"GET /{self.mountpoint} HTTP/1.1\r\n"
            f"Host: {self.ntrip_host}:{self.ntrip_port}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"User-Agent: {CLIENT_AGENT}\r\n"
            f"Authorization: Basic {credential}\r\n"
            f"\r\n"
        ).encode('ascii')

    def connect_ntrip(self):
        """NTRIP キャスタへ接続し (ソケット, chunkedデコーダ, 残バイト) を返す。

        認証エラー等の恒久的な失敗では None を返す。
        """
        raw = socket.create_connection((self.ntrip_host, self.ntrip_port), timeout=15.0)
        if self.ntrip_use_tls:
            context = ssl.create_default_context()
            sock = context.wrap_socket(raw, server_hostname=self.ntrip_host)
        else:
            sock = raw
        sock.settimeout(15.0)

        sock.sendall(self.build_ntrip_request())

        # ヘッダを終端まで読む
        header = b""
        while b"\r\n\r\n" not in header:
            part = sock.recv(RECV_SIZE)
            if not part:
                sock.close()
                raise ConnectionError("Caster closed the connection while sending headers")
            header += part
            if len(header) > 16384:
                sock.close()
                raise ConnectionError("NTRIP response header is too large")

        head, _, leftover = header.partition(b"\r\n\r\n")
        head_text = head.decode('ascii', errors='ignore')
        status_line = head_text.split("\r\n")[0]
        self.get_logger().info(f"Caster Response: {status_line}")

        if not ("ICY 200 OK" in head_text or " 200 " in status_line or "200 OK" in status_line):
            sock.close()
            if "401" in status_line:
                self.get_logger().error(
                    "NTRIP authentication failed (401). "
                    "Check ntrip_username / ntrip_password.")
            elif "404" in status_line:
                self.get_logger().error(
                    f"Mountpoint '{self.mountpoint}' not found (404). "
                    "Check ntrip_mountpoint.")
            else:
                self.get_logger().error(f"NTRIP Caster ResponseError!! : {status_line}")
            return None

        # ヘッダ受信後は短いタイムアウトにする。recv をブロックさせたままに
        # すると GGA の定期送信がデータ到着に引きずられてしまう。
        sock.settimeout(1.0)

        chunked = "chunked" in head_text.lower()
        decoder = ChunkedDecoder() if chunked else None
        self.get_logger().info(
            f"PointPerfect NTRIP connected: {self.ntrip_host}:{self.ntrip_port}"
            f"/{self.mountpoint} (tls={self.ntrip_use_tls}, chunked={chunked})")
        return sock, decoder, leftover

    def send_gga_if_due(self, force: bool = False) -> bool:
        """NEAR-* マウントポイント向けに GGA を定期送信する。

        送信に失敗した場合は False を返す。PC のネットワークが切り替わって
        IP アドレスが変わると既存の TCP 接続は無効になり、recv がタイムアウト
        するまで数十秒気づけないため、送信失敗を接続断とみなして即座に
        張り直せるようにしている。
        """
        if self.latest_gga is None and self.fallback_position is None:
            if not self.gga_warned:
                self.gga_warned = True
                self.get_logger().warn(
                    f"No GGA received on {self.gga_topic} yet. "
                    "Connecting anyway; the caster streams without a position.")
            return True

        if not self.gga_is_fresh():
            # 受信機から GGA が出てこなくなった状態。接続自体は維持する。
            self.get_logger().warn(
                f"No GGA from the receiver for {self.gga_timeout:.0f} s. "
                "Still sending the last one.",
                throttle_duration_sec=30.0)

        now = time.monotonic()
        if not force and (now - self.last_gga_sent) < self.gga_interval:
            return True

        sentence, source = self.gga_to_send()
        if sentence is None:
            return True

        with self.socket_lock:
            if self.socket is None:
                return False
            try:
                self.socket.sendall(sentence.encode('ascii'))
            except OSError as ex:
                if self.running and rclpy.ok():
                    self.get_logger().warn(
                        f"Failed to send GGA to the caster: {ex}. "
                        "The connection looks dead. Reconnecting...")
                return False
        self.last_gga_sent = now
        if self.debug:
            self.get_logger().info(f"Sent GGA to caster ({source}): {sentence.strip()}")
        return True

    def wait_for_gga(self) -> bool:
        """測位解が得られるまで接続を待つ。

        NEAR-* マウントポイントは受信機位置が届くまで配信を始めず、
        数秒でキャスタ側から接続を切られる。GGA が無いまま接続を
        繰り返すとサーバを無駄に叩くだけなので、先に測位解を待つ。
        """
        if self.gga_is_fresh():
            return True

        self.get_logger().warn(
            f"Waiting for a GGA on {self.gga_topic} before connecting "
            "(require_gga is set).")
        while self.running and rclpy.ok():
            if self.gga_is_fresh():
                self.get_logger().info("GGA received. Connecting to the caster.")
                return True
            time.sleep(1.0)
        return False

    def log_session_end(self, reason: str, session_start: float, bytes_at_start: int):
        """切断時にセッションの継続時間と受信量を残す。

        データが流れていたのに切られたのか、無言のまま切られたのかを
        後から切り分けられるようにする。
        """
        elapsed = time.monotonic() - session_start
        received = self.correction_bytes_total - bytes_at_start
        self.get_logger().warn(
            f"{reason} after {elapsed:.1f} s ({received} bytes received "
            f"in this session). Reconnecting...")

    def ntrip_loop(self):
        """接続・受信・再接続を行うワーカースレッド。"""
        while self.running and rclpy.ok():
            decoder = None
            if self.require_gga and not self.wait_for_gga():
                break
            try:
                self.get_logger().info(
                    f"PointPerfect NTRIP connecting to "
                    f"{self.ntrip_host}:{self.ntrip_port}/{self.mountpoint} ...")
                result = self.connect_ntrip()
                if result is None:
                    # 認証・マウントポイントの誤りは再試行しても直らない
                    break
                sock, decoder, leftover = result
                with self.socket_lock:
                    self.socket = sock

                self.last_gga_sent = 0.0
                if not self.send_gga_if_due(force=True):
                    continue
                if leftover:
                    self.publish_payload(decoder.feed(leftover) if decoder else leftover)

                session_start = time.monotonic()
                session_bytes = self.correction_bytes_total
                last_data = time.monotonic()

                while self.running and rclpy.ok():
                    try:
                        data = sock.recv(RECV_SIZE)
                    except socket.timeout:
                        data = None
                    else:
                        if not data:
                            self.log_session_end(
                                "NTRIP Caster disconnected",
                                session_start, session_bytes)
                            break
                        last_data = time.monotonic()
                        self.publish_payload(decoder.feed(data) if decoder else data)

                    # GGA の送信はデータ到着とは独立に時間で判断する。
                    # 受信が止まっている間こそ位置を送り続ける必要がある。
                    if not self.send_gga_if_due():
                        break

                    if time.monotonic() - last_data > self.data_timeout:
                        self.log_session_end(
                            f"No correction data for {self.data_timeout:.0f} s",
                            session_start, session_bytes)
                        break

            except (OSError, ValueError, ConnectionError) as ex:
                # shutdown() がソケットを閉じた場合の例外は正常終了なので無視する
                if self.running and rclpy.ok():
                    self.get_logger().error(f"NTRIP error: {ex}. Retrying...")
            finally:
                with self.socket_lock:
                    if self.socket is not None:
                        try:
                            self.socket.close()
                        except OSError:
                            pass
                        self.socket = None

            if self.running and rclpy.ok():
                time.sleep(self.reconnect_interval)

    # ======================================================================
    # MQTT 配信
    # ======================================================================
    def load_credentials_file(self, path: str):
        """Thingstream の JSON 設定から MQTT 接続情報を読み込む。"""
        if not os.path.exists(path):
            self.get_logger().error(f"Credentials file not found: {path}")
            return

        with open(path, 'r', encoding='utf-8') as stream:
            config = json.load(stream)

        client_id = find_in_json(config, 'ClientID')
        if client_id and not self.client_id:
            self.client_id = client_id

        # 証明書・鍵は JSON に PEM 文字列として埋め込まれている
        for target_key, attr in (('Cert', 'cert_file'),
                                 ('Key', 'key_file'),
                                 ('RootCA', 'ca_file')):
            if getattr(self, attr):
                continue  # 個別にファイル指定があればそちらを優先
            pem = find_in_json(config, target_key)
            if pem and 'BEGIN' in pem:
                setattr(self, attr, self._write_temp_pem(pem, target_key))

        self.get_logger().info(f"Loaded PointPerfect credentials from {path}")

    def _write_temp_pem(self, pem: str, name: str) -> str:
        """PEM 文字列を一時ファイルへ書き出しパスを返す (paho はパスを要求する)。"""
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix=f"_{name}.pem", delete=False, encoding='utf-8')
        handle.write(pem if pem.endswith('\n') else pem + '\n')
        handle.close()
        os.chmod(handle.name, 0o600)
        self._temp_files.append(handle.name)
        return handle.name

    def validate_credentials(self) -> bool:
        missing = [name for name, value in
                   (('client_id', self.client_id),
                    ('cert_file', self.cert_file),
                    ('key_file', self.key_file)) if not value]
        if missing:
            self.get_logger().error(
                f"Missing PointPerfect credentials: {', '.join(missing)}. "
                "Set them in config/pointperfect.yaml or specify credentials_file.")
            return False

        for name, path in (('cert_file', self.cert_file), ('key_file', self.key_file)):
            if not os.path.exists(path):
                self.get_logger().error(f"{name} does not exist: {path}")
                return False
        if self.ca_file and not os.path.exists(self.ca_file):
            self.get_logger().error(f"ca_file does not exist: {self.ca_file}")
            return False
        return True

    def create_client(self):
        """paho-mqtt クライアントを生成する (paho 1.x / 2.x 両対応)。"""
        import paho.mqtt.client as mqtt

        if hasattr(mqtt, 'CallbackAPIVersion'):  # paho-mqtt 2.x
            client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
                client_id=self.client_id)
        else:  # paho-mqtt 1.x
            client = mqtt.Client(client_id=self.client_id)

        client.tls_set(
            ca_certs=self.ca_file or None,
            certfile=self.cert_file,
            keyfile=self.key_file,
            cert_reqs=ssl.CERT_REQUIRED,
            tls_version=ssl.PROTOCOL_TLSv1_2)

        client.on_connect = self.on_connect
        client.on_disconnect = self.on_disconnect
        client.on_message = self.on_message
        client.reconnect_delay_set(min_delay=1, max_delay=30)
        return client

    def subscribe_topics(self, client):
        topics = list(self.correction_mqtt_topics)
        if self.key_mqtt_topic:
            topics.append(self.key_mqtt_topic)
        if self.enable_assistnow and self.assistnow_mqtt_topic:
            topics.append(self.assistnow_mqtt_topic)

        for topic in topics:
            client.subscribe(topic, qos=0)
            self.get_logger().info(f"Subscribed to MQTT topic: {topic}")

    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            self.get_logger().info(
                f"PointPerfect connected: {self.host}:{self.port} (region={self.region})")
            self.subscribe_topics(client)
        else:
            self.get_logger().error(
                f"PointPerfect connection refused (code {reason_code}). "
                "Check the client certificate and the client_id.")

    def on_disconnect(self, client, userdata, reason_code, properties=None):
        if reason_code != 0:
            self.get_logger().warn(
                f"PointPerfect disconnected unexpectedly (code {reason_code}). Reconnecting...")
        else:
            self.get_logger().info("PointPerfect disconnected.")

    def on_message(self, client, userdata, msg):
        """MQTT で受信したデータをそのまま ROS トピックへ流す。"""
        if msg.topic == self.key_mqtt_topic and not self.spartn_key_received and msg.payload:
            self.spartn_key_received = True
            self.get_logger().info(
                f"SPARTN dynamic keys received ({len(msg.payload)} bytes, UBX-RXM-SPARTNKEY).")
        self.publish_payload(bytes(msg.payload), source=msg.topic)

    # ======================================================================
    # 共通
    # ======================================================================
    def publish_payload(self, payload: bytes, source: str = 'ntrip'):
        """受信したバイト列を加工せず ROS トピックへ発行する。"""
        if not payload:
            return
        first = self.correction_bytes_total == 0
        self.correction_bytes_total += len(payload)
        if first:
            self.get_logger().info(
                f"Receiving SPARTN correction data ({len(payload)} bytes in the first chunk).")
        if self.debug:
            self.get_logger().info(
                f"{source}: {len(payload)} bytes (total {self.correction_bytes_total} bytes)")

        msg = UInt8MultiArray()
        msg.data = list(payload)
        self.pub.publish(msg)

    def start(self) -> bool:
        if self.mode == 'ntrip':
            if not self.ntrip_username or not self.ntrip_password:
                self.get_logger().error(
                    "Missing NTRIP credentials. Set ntrip_username / ntrip_password "
                    "from the Credentials tab of your Thingstream Location Thing.")
                return False
            self.thread = threading.Thread(target=self.ntrip_loop, daemon=True)
            self.thread.start()
            return True

        if self.mode == 'mqtt':
            if not self.validate_credentials():
                return False
            try:
                self.client = self.create_client()
            except ImportError:
                self.get_logger().error(
                    "paho-mqtt is required for mode 'mqtt'. Install it with: "
                    "sudo apt install python3-paho-mqtt")
                return False
            try:
                self.get_logger().info(
                    f"PointPerfect connecting to {self.host}:{self.port} ...")
                self.client.connect(self.host, self.port, keepalive=self.keepalive)
            except Exception as ex:
                self.get_logger().error(f"PointPerfect connect error: {ex}")
                return False
            # 受信は paho のワーカースレッドに任せ、本スレッドは rclpy を回す
            self.client.loop_start()
            return True

        self.get_logger().error(f"Unknown mode '{self.mode}'. Use 'ntrip' or 'mqtt'.")
        return False

    def shutdown(self):
        self.running = False
        with self.socket_lock:
            if self.socket is not None:
                # recv() でブロックしている受信スレッドを解放する
                try:
                    self.socket.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    self.socket.close()
                except OSError:
                    pass
                self.socket = None
        if self.thread is not None:
            self.thread.join(timeout=3.0)
            self.thread = None
        if self.client is not None:
            self.client.loop_stop()
            try:
                self.client.disconnect()
            except Exception:
                pass
            self.client = None
        for path in self._temp_files:
            try:
                os.remove(path)
            except OSError:
                pass
        self._temp_files = []


def main(args=None):
    rclpy.init(args=args)
    node = PointPerfectConnectNode()

    if not node.start():
        node.get_logger().error("Failed to start PointPerfect client. Shutting down.")
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
