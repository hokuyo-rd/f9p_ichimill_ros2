#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ZED-F9P シリアルドライバ。

* 受信機からの NMEA センテンスを購読可能なトピックへ発行する
* 補正データ (PointPerfect の SPARTN / ichimill・NTRIP の RTCM3) を
  受信機のシリアルポートへ書き戻す
* 起動時に UBX-CFG-VALSET で受信機を設定する (SPARTN 入力の有効化、
  ZDA/GGA/RMC 出力の有効化、測位レート)

補正データは UInt8MultiArray で受け取るため、SPARTN のようなバイナリでも
そのまま透過的に受信機へ渡せる。
"""

import threading
import time
from collections import deque

import serial

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from nmea_msgs.msg import Sentence
from std_msgs.msg import UInt8MultiArray

try:  # colcon でインストールした場合とスクリプト直実行の両方に対応
    from scripts import ubx
except ImportError:  # pragma: no cover
    import ubx

# シリアル読み出しバッファの上限。これを超えたら同期崩れとみなして捨てる。
MAX_BUFFER_SIZE = 16384


def calcultateCheckSum(stringToCheck):
    """NMEA センテンス本体 ('$' と '*' を除く) のチェックサムを計算する。"""
    xsum_calc = 0
    for char in stringToCheck:
        xsum_calc = xsum_calc ^ ord(char)
    return "%02X" % xsum_calc


def nmea_checksum_ok(sentence: str) -> bool:
    """NMEA センテンスのチェックサムが正しいかを判定する。

    取りこぼしや UBX バイナリの誤同期で壊れた行を購読側へ流さないために使う。
    """
    if not sentence.startswith('$'):
        return False
    star = sentence.rfind('*')
    if star < 2 or len(sentence) - star < 3:
        return False
    try:
        expected = int(sentence[star + 1:star + 3], 16)
    except ValueError:
        return False
    actual = 0
    for char in sentence[1:star]:
        actual ^= ord(char)
    return actual == expected


class F9PDriverNode(Node):
    def __init__(self):
        super().__init__('f9p_driver')

        # パラメータの宣言と取得
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 230400)
        self.declare_parameter('debug', False)
        # 補正データを購読するトピック
        #   PointPerfect : /pointperfect/correction_data (SPARTN)
        #   ichimill/NTRIP: /softbank/rtcm_data (RTCM3)
        self.declare_parameter('correction_topic', '/softbank/rtcm_data')
        # 受信機のどのインタフェースに接続しているか (usb / uart1 / uart2 / i2c / spi)
        self.declare_parameter('interface', 'usb')
        # 起動時に UBX-CFG-VALSET で受信機を設定するか。
        # 既定は False (u-center 等で設定済みの受信機を書き換えない)。
        # PointPerfect 用 launch では True にして SPARTN 入力を有効化する。
        self.declare_parameter('configure_receiver', False)
        # 設定を保存するレイヤ (ram / bbr / flash のカンマ区切り)
        self.declare_parameter('config_layers', 'ram')
        # SPARTN 補正源 ip / lband / none (none は設定を変更しない)
        self.declare_parameter('spartn_source', 'none')
        # 測位レート [Hz]
        self.declare_parameter('nav_rate_hz', 5.0)
        # 有効化する NMEA センテンス
        self.declare_parameter('nmea_sentences', ['gga', 'zda', 'rmc'])
        # シリアル書き込みのタイムアウト [s]
        self.declare_parameter('write_timeout_sec', 2.0)
        # この秒数 NMEA が来なければ警告する
        self.declare_parameter('silence_warn_sec', 5.0)
        # 設定投入後に UBX-CFG-VALGET で実際の値を読み戻して確認する
        self.declare_parameter('verify_configuration', True)
        # 測位品質と補正データ流量を定期的に出す間隔 [s] (0 で無効)
        self.declare_parameter('status_interval_sec', 10.0)
        # UBX-MON-RF / UBX-NAV-SAT を有効化してアンテナ状態と衛星の
        # 信号強度を読む。測位できない原因の切り分けに使う。
        self.declare_parameter('enable_diagnostics', True)

        self.serial_port = self.get_parameter('port').get_parameter_value().string_value
        self.serial_baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.correction_topic = self.get_parameter('correction_topic').get_parameter_value().string_value
        self.interface = self.get_parameter('interface').get_parameter_value().string_value.lower()
        self.configure_receiver = self.get_parameter('configure_receiver').get_parameter_value().bool_value
        self.config_layers = self.get_parameter('config_layers').get_parameter_value().string_value
        self.spartn_source = self.get_parameter('spartn_source').get_parameter_value().string_value.lower()
        self.nav_rate_hz = self.get_parameter('nav_rate_hz').get_parameter_value().double_value
        self.nmea_sentences = [s.lower() for s in
                               self.get_parameter('nmea_sentences').get_parameter_value().string_array_value]
        self.write_timeout = self.get_parameter('write_timeout_sec').get_parameter_value().double_value
        self.silence_warn = self.get_parameter('silence_warn_sec').get_parameter_value().double_value
        self.verify_configuration = self.get_parameter('verify_configuration').get_parameter_value().bool_value
        self.status_interval = self.get_parameter('status_interval_sec').get_parameter_value().double_value
        self.enable_diagnostics = self.get_parameter('enable_diagnostics').get_parameter_value().bool_value

        if self.interface not in ubx.INPROT_KEYS:
            self.get_logger().warn(
                f"Unknown interface '{self.interface}'. Falling back to 'usb'.")
            self.interface = 'usb'

        # Publisher, Subscriberの初期化
        self.pub_gga = self.create_publisher(Sentence, 'nmea_gga', 10)
        self.pub_nmea = self.create_publisher(Sentence, 'nmea_sentence', 10)
        self.pub_zda = self.create_publisher(Sentence, 'nmea_zda', 10)
        self.pub_rmc = self.create_publisher(Sentence, 'nmea_rmc', 10)
        self.subscription = self.create_subscription(
            UInt8MultiArray, self.correction_topic, self.cb_correction_data, 50)

        # 補正データはコールバックで到着するのでキューに溜め、
        # シリアルループ側でまとめて書き出す (取りこぼし防止)
        self.correction_queue = deque(maxlen=200)
        self.queue_lock = threading.Lock()
        self.correction_bytes_total = 0

        self.gps_serial = None
        self.rx_buffer = b""
        self.invalid_nmea_count = 0
        self.last_nmea_time = 0.0
        self.gga_quality = None
        self.gga_numsv = None
        self.nmea_count = 0
        self.last_status_time = 0.0
        self.status_bytes_mark = 0
        self.valget_pending = False
        self.valget_sent_time = 0.0
        self.valset_labels = []
        self.antenna = None
        self.antenna_source = None
        self.sat_count = 0
        self.sat_cnos = []

        self.get_logger().info(
            f"Correction data topic: {self.correction_topic} (interface={self.interface})")

    # --- ROS コールバック ---------------------------------------------------
    def cb_correction_data(self, msg: UInt8MultiArray):
        """補正データ (SPARTN / RTCM3) を受け取りキューへ積む。"""
        data = bytes(msg.data)
        if not data:
            return
        with self.queue_lock:
            if len(self.correction_queue) == self.correction_queue.maxlen:
                self.get_logger().warn(
                    "Correction queue is full. Dropping the oldest chunk.",
                    throttle_duration_sec=5.0)
            self.correction_queue.append(data)

    # --- 受信機の設定 -------------------------------------------------------
    def _parse_layers(self) -> int:
        layers = 0
        mapping = {'ram': ubx.LAYER_RAM, 'bbr': ubx.LAYER_BBR, 'flash': ubx.LAYER_FLASH}
        for name in self.config_layers.split(','):
            name = name.strip().lower()
            if name in mapping:
                layers |= mapping[name]
            elif name:
                self.get_logger().warn(f"Unknown config layer '{name}' is ignored.")
        return layers or ubx.LAYER_RAM

    def build_config_key_values(self):
        """受信機に投入する設定 (キーID, 値) の一覧を組み立てる。"""
        key_values = []
        inprot = ubx.INPROT_KEYS[self.interface]
        # UBX / NMEA / RTCM3 / SPARTN の入力を許可する
        key_values.append((inprot[0], 1))  # UBX
        key_values.append((inprot[2], 1))  # RTCM3X
        key_values.append((inprot[3], 1))  # SPARTN

        # SPARTN 補正源 (PointPerfect IP 配信なら ip)
        if self.spartn_source == 'ip':
            key_values.append((ubx.CFG_SPARTN_USE_SOURCE, ubx.SPARTN_SOURCE_IP))
        elif self.spartn_source == 'lband':
            key_values.append((ubx.CFG_SPARTN_USE_SOURCE, ubx.SPARTN_SOURCE_LBAND))

        # NMEA 出力。ZDA は時刻同期に使うため既定で有効化する。
        for name, ports in ubx.MSGOUT_NMEA_KEYS.items():
            if self.interface not in ports:
                continue
            key_values.append((ports[self.interface], 1 if name in self.nmea_sentences else 0))

        # 測位レート
        if self.nav_rate_hz > 0.0:
            meas_ms = int(round(1000.0 / self.nav_rate_hz))
            meas_ms = max(25, min(65535, meas_ms))
            key_values.append((ubx.CFG_RATE_MEAS, meas_ms))
            key_values.append((ubx.CFG_RATE_NAV, 1))

        return key_values

    def send_configuration(self):
        """UBX-CFG-VALSET を受信機へ送信する。"""
        key_values = self.build_config_key_values()
        layers = self._parse_layers()
        frames = ubx.split_valset(key_values, layers=layers)

        self.get_logger().info(
            f"Configuring receiver: {len(key_values)} items, "
            f"nav_rate={self.nav_rate_hz} Hz, spartn_source={self.spartn_source}, "
            f"nmea={self.nmea_sentences}")
        self.valset_labels = ['main'] * len(frames)
        for frame in frames:
            # flush() は tcdrain() を呼び送信完了まで無期限に待つため使わない
            try:
                self.gps_serial.write(frame)
            except serial.SerialTimeoutException:
                self.get_logger().error(
                    "Timed out while writing the configuration to the receiver. "
                    "Check the serial connection.")
                return
        self.get_logger().info(
            "Configuration sent. Waiting for UBX-ACK-ACK from the receiver...")

        if self.enable_diagnostics:
            # 診断用メッセージは別フレームで送る。キーが受け付けられなくても
            # 本体の設定まで巻き添えで拒否されないようにするため。
            diag = [(ports[self.interface], 5)
                    for ports in ubx.MSGOUT_UBX_KEYS.values()
                    if self.interface in ports]
            try:
                self.gps_serial.write(ubx.build_valset(diag, layers=self._parse_layers()))
                self.valset_labels.append('diagnostics')
            except serial.SerialTimeoutException:
                self.get_logger().warn("Timed out while enabling the diagnostic messages.")

        if self.verify_configuration:
            # 投入した値が実際に効いているかを受信機から読み戻す。
            # ACK は「キーが存在する」ことしか保証しないため。
            try:
                self.gps_serial.write(ubx.build_valget([k for k, _ in key_values]))
                # アンテナ給電の設定は書き換えず、現在値だけ別要求で読む。
                # キーIDが FW と合わなくても本体の読み戻しを巻き添えにしない。
                self.gps_serial.write(ubx.build_valget(list(ubx.CFG_HW_ANT_KEYS)))
                self.valget_pending = True
                self.valget_sent_time = time.monotonic()
            except serial.SerialTimeoutException:
                self.get_logger().warn("Timed out while requesting the configuration readback.")

    # --- シリアル受信 -------------------------------------------------------
    def read_serial(self):
        """シリアルから読み出し、NMEA センテンスと UBX フレームに分解する。

        SPARTN を使うと受信機から UBX (ACK や RXM-SPARTN) も返るため、
        行単位の readline ではなくバイト列として扱い両者を切り分ける。
        """
        waiting = self.gps_serial.in_waiting
        chunk = self.gps_serial.read(waiting if waiting > 0 else 1)
        if not chunk:
            return
        self.rx_buffer += chunk

        while self.rx_buffer:
            if self.rx_buffer.startswith(ubx.UBX_SYNC):
                msg_class, msg_id, payload, consumed = ubx.parse_frame(self.rx_buffer)
                if consumed == 0:
                    break  # フレームが未完成。次の読み出しを待つ。
                self.rx_buffer = self.rx_buffer[consumed:]
                if msg_class is not None:
                    self.handle_ubx(msg_class, msg_id, payload)
                continue

            if self.rx_buffer.startswith(b"$"):
                index = self.rx_buffer.find(b"\n")
                if index < 0:
                    break  # センテンスが未完成
                line = self.rx_buffer[:index]
                self.rx_buffer = self.rx_buffer[index + 1:]
                # 取りこぼしや UBX バイナリの誤同期で先頭にゴミが残ることが
                # あるため、行内の最後の '$' から切り出して同期を取り直す
                start = line.rfind(b"$")
                if start > 0:
                    line = line[start:]
                try:
                    sentence = line.decode('ascii').strip()
                except UnicodeDecodeError:
                    self.count_invalid_nmea()
                    continue
                if not nmea_checksum_ok(sentence):
                    self.count_invalid_nmea(sentence)
                    continue
                self.handle_nmea(sentence)
                continue

            # '$' でも UBX 同期文字でもない → 次の開始位置まで読み飛ばす
            next_nmea = self.rx_buffer.find(b"$", 1)
            next_ubx = self.rx_buffer.find(ubx.UBX_SYNC, 1)
            candidates = [i for i in (next_nmea, next_ubx) if i > 0]
            if not candidates:
                self.rx_buffer = b""
                break
            self.rx_buffer = self.rx_buffer[min(candidates):]

        if len(self.rx_buffer) > MAX_BUFFER_SIZE:
            self.get_logger().warn("Serial RX buffer overflow. Clearing.")
            self.rx_buffer = b""

    def report_configuration(self, payload: bytes):
        """読み戻した設定値をログに出す。

        SPARTN 入力が本当に有効になっているかなど、補正が効かないときの
        切り分けに必要な情報をここで確定させる。
        """
        items = ubx.parse_valget(payload)
        if not items:
            self.get_logger().warn("Configuration readback returned no items.")
            return

        self.get_logger().info("Receiver configuration readback:")
        for key, value in items:
            name = ubx.key_name(key)
            note = ""
            if key == ubx.CFG_SPARTN_USE_SOURCE:
                note = "  <- 0=IP, 1=L-band"
            elif key == 0x10A3002E and value == 0:
                note = ("  <- antenna supply is OFF. An active antenna gets no power, "
                        "which costs 20+ dB of signal.")
            elif key == 0x10A3002E:
                note = "  <- antenna supply is ON"
            elif name.endswith("INPROT-SPARTN") and value == 0:
                note = "  <- SPARTN input is DISABLED. Corrections will be ignored."
            self.get_logger().info(f"  {name} = {value}{note}")

    def check_valget_timeout(self):
        """読み戻しの応答が返ってこないことを可視化する。"""
        if not self.valget_pending:
            return
        if time.monotonic() - self.valget_sent_time < 5.0:
            return
        self.valget_pending = False
        self.get_logger().warn(
            "No configuration readback from the receiver (UBX-CFG-VALGET). "
            "Cannot verify whether SPARTN input is enabled.")

    def poll_antenna_status(self):
        """アンテナ状態を問い合わせる (出力設定キーに依存しない)。

        MON-RF に応答しない FW があるため、旧来の MON-HW も併せて問い合わせる。
        """
        if not self.enable_diagnostics:
            return
        for msg_id in (ubx.UBX_MON_RF, ubx.UBX_MON_HW):
            try:
                self.gps_serial.write(ubx.poll_request(ubx.UBX_CLASS_MON, msg_id))
            except serial.SerialTimeoutException:
                self.get_logger().warn(
                    "Timed out while polling the antenna status.",
                    throttle_duration_sec=30.0)
                return

    def report_status(self):
        """測位品質と補正データの流量を定期的に出す。"""
        if self.status_interval <= 0.0:
            return
        now = time.monotonic()
        if now - self.last_status_time < self.status_interval:
            return
        self.last_status_time = now

        labels = {'0': 'no fix', '1': 'single', '2': 'DGNSS', '4': 'RTK FIXED',
                  '5': 'RTK float', '6': 'dead reckoning'}
        quality = self.gga_quality
        label = labels.get(quality, 'unknown')
        written = self.correction_bytes_total - self.status_bytes_mark
        self.status_bytes_mark = self.correction_bytes_total

        self.get_logger().info(
            f"Status: fix={label}({quality}) sats={self.gga_numsv} "
            f"nmea={self.nmea_count} corrections={written} B/{self.status_interval:.0f}s "
            f"(total {self.correction_bytes_total} B)")

        # 次回のステータスに間に合うよう、今のうちに問い合わせておく
        self.poll_antenna_status()

        if self.antenna is not None:
            note = ""
            if self.antenna['ant_status'] == 'OPEN':
                note = "  <- no antenna current. Check the antenna and its power."
            elif self.antenna['ant_status'] == 'SHORT':
                note = "  <- antenna short circuit."
            self.get_logger().info(
                f"  Antenna[{self.antenna_source}]: status={self.antenna['ant_status']} "
                f"power={self.antenna['ant_power']} agc={self.antenna['agc']} "
                f"noise={self.antenna['noise']} jamming={self.antenna['jamming']}{note}")

        else:
            self.get_logger().info(
                "  Antenna: no response to UBX-MON-RF / UBX-MON-HW polls.")

        if self.sat_count:
            tracked = [c for c in self.sat_cnos if c > 0]
            strong = [c for c in self.sat_cnos if c >= 35]
            best = max(self.sat_cnos) if self.sat_cnos else 0
            note = "" if tracked else "  <- no signal at all. The antenna sees nothing."
            self.get_logger().info(
                f"  Signals: {self.sat_count} in view, {len(tracked)} with C/N0>0, "
                f"{len(strong)} >=35 dBHz, best={best} dBHz{note}")

    def check_receiver_silence(self):
        """受信機から NMEA が来ていないことを可視化する。

        無言のまま止まっていると原因の切り分けができないため、
        一定時間データが無ければ警告を出す。
        """
        if self.silence_warn <= 0.0:
            return
        silent = time.monotonic() - self.last_nmea_time
        if silent < self.silence_warn:
            return
        self.get_logger().warn(
            f"No NMEA received from the receiver for {silent:.0f} s. "
            f"Check that the receiver outputs NMEA on {self.interface} "
            "and that no other process is holding the serial port.",
            throttle_duration_sec=10.0)

    def count_invalid_nmea(self, sentence: str = ''):
        """チェックサム不正のセンテンスを数え、頻発していれば警告する。"""
        self.invalid_nmea_count += 1
        if self.debug and sentence:
            self.get_logger().warn(f"Dropped a corrupt NMEA sentence: {sentence}")
        self.get_logger().warn(
            f"Dropped {self.invalid_nmea_count} corrupt NMEA sentences so far. "
            "Check the baud rate and the NMEA output load on the receiver.",
            throttle_duration_sec=30.0)

    def handle_ubx(self, msg_class: int, msg_id: int, payload: bytes):
        """受信機からの UBX メッセージを処理する (主に設定の ACK 確認)。"""
        if msg_class == ubx.UBX_CLASS_ACK and len(payload) >= 2:
            acked_class, acked_id = payload[0], payload[1]
            if acked_class == ubx.UBX_CLASS_CFG and acked_id == ubx.UBX_CFG_VALSET:
                # ACK は送った順に返るのでラベルを先頭から対応させる
                label = self.valset_labels.pop(0) if self.valset_labels else 'unknown'
                if msg_id == ubx.UBX_ACK_ACK:
                    self.get_logger().info(
                        f"Receiver configuration accepted (UBX-ACK-ACK, {label}).")
                elif label == 'diagnostics':
                    self.get_logger().warn(
                        "Receiver rejected the diagnostic messages (UBX-ACK-NAK). "
                        "Antenna status will not be available; the main configuration is unaffected.")
                else:
                    self.get_logger().error(
                        "Receiver rejected the configuration (UBX-ACK-NAK). "
                        "Check the firmware version and the interface parameter.")
            elif acked_class == ubx.UBX_CLASS_CFG and acked_id == ubx.UBX_CFG_VALGET:
                if msg_id == ubx.UBX_ACK_NAK:
                    # NAK なら読み戻しは返ってこない。黙って待ち続けないよう明示する。
                    self.valget_pending = False
                    self.get_logger().warn(
                        "Receiver rejected the configuration readback (UBX-CFG-VALGET NAK). "
                        "The readback is diagnostic only; the configuration itself was applied.")
            elif self.debug:
                self.get_logger().info(
                    f"UBX-ACK-{'ACK' if msg_id else 'NAK'} for 0x{acked_class:02x}"
                    f" 0x{acked_id:02x}")
        elif msg_class == ubx.UBX_CLASS_MON and msg_id == ubx.UBX_MON_RF:
            blocks = ubx.parse_mon_rf(payload)
            if blocks:
                self.antenna = blocks[0]
                self.antenna_source = 'MON-RF'
        elif msg_class == ubx.UBX_CLASS_MON and msg_id == ubx.UBX_MON_HW:
            # MON-RF が応答した場合はそちらを優先する
            info = ubx.parse_mon_hw(payload)
            if info and self.antenna_source != 'MON-RF':
                self.antenna = info
                self.antenna_source = 'MON-HW'
        elif msg_class == ubx.UBX_CLASS_NAV and msg_id == ubx.UBX_NAV_SAT:
            self.sat_count, self.sat_cnos = ubx.parse_nav_sat(payload)
        elif msg_class == ubx.UBX_CLASS_CFG and msg_id == ubx.UBX_CFG_VALGET:
            self.valget_pending = False
            self.report_configuration(payload)
        elif self.debug:
            self.get_logger().info(
                f"UBX message received: class=0x{msg_class:02x} id=0x{msg_id:02x} "
                f"len={len(payload)}")

    def handle_nmea(self, gps_str: str):
        """NMEA センテンスを対応するトピックへ発行する。"""
        if not gps_str:
            return

        self.last_nmea_time = time.monotonic()

        if self.debug:
            self.get_logger().info(f"Received: {gps_str}")

        self.nmea_count += 1

        # GGAセンテンスの処理と発行
        if "GGA" in gps_str:
            fields = gps_str.split(',')
            if len(fields) > 7:
                self.gga_quality = fields[6]
                self.gga_numsv = fields[7]
            send_data = gps_str
            if "$GNGGA" in gps_str:
                # NTRIP キャスタ向けに GPGGA へ書き換えてチェックサムを再計算
                gga_string = gps_str.replace('$GNGGA', 'GPGGA')
                if '*' in gga_string:
                    gga_string = gga_string.split('*')[0]
                checksum = calcultateCheckSum(gga_string)
                send_data = f"${gga_string}*{checksum}\r\n"

            self.pub_gga.publish(self.make_sentence(send_data))

        # ZDAセンテンスの処理と発行
        if "ZDA" in gps_str:
            self.pub_zda.publish(self.make_sentence(gps_str))

        # RMCセンテンスの処理と発行
        if "RMC" in gps_str:
            self.pub_rmc.publish(self.make_sentence(gps_str))

        # 全てのNMEAセンテンスを発行
        self.pub_nmea.publish(self.make_sentence(gps_str))

    def make_sentence(self, sentence: str) -> Sentence:
        msg = Sentence()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.sentence = sentence
        return msg

    # --- 補正データ送出 -----------------------------------------------------
    def write_corrections(self):
        """キューに溜まった補正データをまとめて受信機へ書き込む。"""
        with self.queue_lock:
            if not self.correction_queue:
                return
            payload = b"".join(self.correction_queue)
            self.correction_queue.clear()

        try:
            self.gps_serial.write(payload)
        except serial.SerialTimeoutException:
            self.get_logger().warn(
                "Timed out while writing correction data to the receiver.",
                throttle_duration_sec=10.0)
            return
        self.correction_bytes_total += len(payload)
        if self.debug:
            self.get_logger().info(
                f"Wrote {len(payload)} bytes of correction data to F9P "
                f"(total {self.correction_bytes_total} bytes).")

    # --- メインループ -------------------------------------------------------
    def run(self):
        try:
            self.get_logger().info("Serial port opening...")
            # timeout は読み込み専用。write_timeout を指定しないと書き込みが
            # 無期限にブロックし、ドライバ全体が停止する。
            self.gps_serial = serial.Serial(
                port=self.serial_port, baudrate=self.serial_baud,
                timeout=0.1, write_timeout=self.write_timeout)
            self.get_logger().info(
                f"OK. Port: {self.serial_port}, Baudrate: {self.serial_baud}")
        except serial.SerialException as ex:
            self.get_logger().error(f"Could not open serial port: {ex}")
            return

        executor = SingleThreadedExecutor()
        executor.add_node(self)

        try:
            if self.configure_receiver:
                self.send_configuration()

            self.poll_antenna_status()
            self.last_nmea_time = time.monotonic()
            self.last_status_time = time.monotonic()
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.0)
                self.read_serial()
                self.write_corrections()
                self.check_receiver_silence()
                self.check_valget_timeout()
                self.report_status()

        except serial.SerialException as ex:
            self.get_logger().error(f"SerialException error: {ex}")
        finally:
            executor.remove_node(self)
            self.gps_serial.close()
            self.get_logger().info("Serial port closed.")


def main(args=None):
    rclpy.init(args=args)
    node = F9PDriverNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
