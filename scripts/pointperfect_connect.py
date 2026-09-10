#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""u-blox PointPerfect (Thingstream) から SPARTN 補正を取得する ROS 2 ノード。

PointPerfect の IP 配信は MQTT over TLS (クライアント証明書認証) で提供される。
本ノードは以下のトピックを購読し、受信したバイト列をそのまま
UInt8MultiArray として ROS トピックへ流す。f9p_driver がそれを
ZED-F9P のシリアルポートへ書き込む。

  /pp/ip/<region>   : SPARTN 2.0.1 補正データ
  /pp/ubx/0236/ip   : UBX-RXM-SPARTNKEY (動的復号キー。補正の利用に必須)
  /pp/ubx/mga       : AssistNow Online (任意。初期測位の高速化)

いずれも受信機がそのまま解釈できる形式で配信されるため、加工せずに転送する。
"""

import json
import os
import ssl
import tempfile

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

try:
    import paho.mqtt.client as mqtt
except ImportError as ex:  # pragma: no cover
    raise SystemExit(
        "paho-mqtt is required for pointperfect_connect. "
        "Install it with: pip3 install paho-mqtt") from ex

DEFAULT_HOST = "pp.services.u-blox.com"
DEFAULT_PORT = 8883
# PointPerfect が対応する地域: us / eu / kr / au / jp
DEFAULT_REGION = "jp"


def find_in_json(data, target_key):
    """ネストした dict/list から指定キーの値を再帰的に探す。

    Thingstream が配布する ucenter-config.json は階層構造が版によって
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


class PointPerfectConnectNode(Node):
    def __init__(self):
        super().__init__('pointperfect_connect')

        # パラメータの宣言
        self.declare_parameter('debug', False)
        self.declare_parameter('host', DEFAULT_HOST)
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('region', DEFAULT_REGION)
        self.declare_parameter('client_id', '')
        # Thingstream からダウンロードした証明書・秘密鍵
        self.declare_parameter('cert_file', '')
        self.declare_parameter('key_file', '')
        self.declare_parameter('ca_file', '')
        # ucenter-config.json をそのまま使う場合はこちらを指定する
        self.declare_parameter('credentials_file', '')
        # 補正データの発行先トピック
        self.declare_parameter('correction_topic', '/pointperfect/correction_data')
        # 購読トピックを明示指定したい場合 (localized distribution 等)。
        # 空なら region から自動生成する。
        self.declare_parameter('correction_mqtt_topics', [''])
        self.declare_parameter('key_mqtt_topic', '/pp/ubx/0236/ip')
        # AssistNow Online (UBX-MGA) も受信して受信機へ渡すか
        self.declare_parameter('enable_assistnow', False)
        self.declare_parameter('assistnow_mqtt_topic', '/pp/ubx/mga')
        self.declare_parameter('keepalive', 60)

        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.region = self.get_parameter('region').get_parameter_value().string_value.lower()
        self.client_id = self.get_parameter('client_id').get_parameter_value().string_value
        self.cert_file = self.get_parameter('cert_file').get_parameter_value().string_value
        self.key_file = self.get_parameter('key_file').get_parameter_value().string_value
        self.ca_file = self.get_parameter('ca_file').get_parameter_value().string_value
        self.credentials_file = self.get_parameter('credentials_file').get_parameter_value().string_value
        self.correction_topic = self.get_parameter('correction_topic').get_parameter_value().string_value
        self.key_mqtt_topic = self.get_parameter('key_mqtt_topic').get_parameter_value().string_value
        self.enable_assistnow = self.get_parameter('enable_assistnow').get_parameter_value().bool_value
        self.assistnow_mqtt_topic = self.get_parameter('assistnow_mqtt_topic').get_parameter_value().string_value
        self.keepalive = self.get_parameter('keepalive').get_parameter_value().integer_value

        mqtt_topics = [t for t in
                       self.get_parameter('correction_mqtt_topics').get_parameter_value().string_array_value
                       if t]
        self.correction_mqtt_topics = mqtt_topics or [f"/pp/ip/{self.region}"]

        # 一時ファイルに書き出した証明書 (終了時に削除する)
        self._temp_files = []

        self.pub = self.create_publisher(UInt8MultiArray, self.correction_topic, 50)
        self.spartn_key_received = False
        self.correction_bytes_total = 0

        if self.credentials_file:
            self.load_credentials_file(self.credentials_file)

        self.client = None

    # --- 認証情報 -----------------------------------------------------------
    def load_credentials_file(self, path: str):
        """Thingstream の ucenter-config.json から接続情報を読み込む。"""
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

    # --- MQTT ---------------------------------------------------------------
    def create_client(self):
        """paho-mqtt クライアントを生成する (paho 1.x / 2.x 両対応)。"""
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
        payload = bytes(msg.payload)
        if not payload:
            return

        if msg.topic == self.key_mqtt_topic and not self.spartn_key_received:
            self.spartn_key_received = True
            self.get_logger().info(
                f"SPARTN dynamic keys received ({len(payload)} bytes, UBX-RXM-SPARTNKEY).")

        self.correction_bytes_total += len(payload)
        if self.debug:
            self.get_logger().info(
                f"MQTT {msg.topic}: {len(payload)} bytes "
                f"(total {self.correction_bytes_total} bytes)")

        out = UInt8MultiArray()
        out.data = list(payload)
        self.pub.publish(out)

    # --- ライフサイクル -----------------------------------------------------
    def start(self) -> bool:
        if not self.validate_credentials():
            return False

        self.client = self.create_client()
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

    def shutdown(self):
        if self.client is not None:
            self.client.loop_stop()
            try:
                self.client.disconnect()
            except Exception:
                pass
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
