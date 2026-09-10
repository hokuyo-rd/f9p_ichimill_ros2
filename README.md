# f9p_ichimill_ros2

f9p_ichimill_ros2
北陽電機 R&D室 髙橋が対応

gps_ichimill.launch.py
シリアルポートに接続されたGPSレシーバF9Pで、ichimillサービスと連携し補正済み位置情報を取得する。<br>

![rosgraph_f9p_](https://user-images.githubusercontent.com/16064762/136687925-9b3c98f7-54e4-4dc8-8176-0805cb15f15a.png)

gps_ntripcaster.launch.py
または、善意の基準局等　公開されたNtripCasterのデータを使い補正済み位置情報を取得する。<br>

gps_pointperfect.launch.py
または、u-blox PointPerfect (Thingstream) のSPARTN補正を使い補正済み位置情報を取得する。<br>
ichimill/NtripCasterがRTCM3(OSR)であるのに対し、PointPerfectはSPARTN 2.0.1(SSR/PPP-RTK)をMQTTで配信する。<br>


## 確認環境
・PC
  Ubuntu 22.04 / ROS2 humble　<br>
　インターネット回線に接続出来ること。<br>

・ZED-F9P-04B-01 <br>
・ヘリカルアンテナ <br>

## Setup

USBシリアルポートを/dev/GNSS_SERIALに書き換えて、実行権限666を自動付与対応を必ず行うこと

```
sudo gedit /etc/udev/rules.d/99-gnss-serial.rules

KERNEL=="ttyACM[0-9]*", SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666", SYMLINK+="GNSS_SERIAL"
```

```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### 確認方法

/dev/GNSS_SERIAL が存在し、/dev/ttyACM0 を指していること。

```
ls -l /dev/GNSS_SERIAL
ls -l /dev/ttyACM0
```

例: lrwxrwxrwx 1 root root 7 Nov 16 11:45 /dev/GNSS_SERIAL -> ttyACM0/dev/ttyACM0 のパーミッションが 666 になっていること。
例: crw-rw-rw- 1 root dialout 166, 0 Nov 16 11:45 /dev/ttyACM0

### インストール

```
# PointPerfectを使う場合はMQTTクライアントが必要
sudo apt install python3-paho-mqtt
# もしくは pip3 install paho-mqtt

cd ~/{your-ros2-ws}/src
git clone https://github.com/hokuyo-rd/f9p_ichimill_ros2.git
git clone https://github.com/hokuyo-rd/nmea_msgs.git # Gpzda メッセージのため
git clone https://github.com/hokuyo-rd/nmea_navsat_driver_ros2.git # Gpgga, Gprmc, Gpzda トピック対応版
colcon build --packages-select nmea_msgs
colcon build --packages-select nmea_navsat_driver_ros2
colcon build --packages-select f9p_ichimill_ros2
source {your-ros2-ws}/devel/setup.bash
```

### 環境設定

```
# ichimillサービスへ接続する場合
gps_ichimill.launch

# 善意の基準局等、公開Ntrip casterへ接続する場合
gps_ntripcaster.launch 

# PointPerfectへ接続する場合
gps_pointperfect.launch
```

F9Pを接続しているシリアルポート名 <br>
ichimillユーザー名、ichimillパスワード、ホストURL、マウントポイントを適宜編集 <br>

## 使い方

#### ichimillサービスへ接続する場合

```
ros2 launch f9p_ichimill gps_ichimill.launch.py
```

#### 善意の基準局等、公開Ntrip casterへ接続する場合

```
ros2 launch f9p_ichimill gps_ntripcaster.launch.py
```

#### PointPerfectへ接続する場合

```
ros2 launch f9p_ichimill gps_pointperfect.launch.py
```

## PointPerfect (SPARTN) 対応について

ZED-F9P-04B (FW HPG 1.32) はSPARTN 2.0.1を入力プロトコルとしてサポートしており、
u-blox PointPerfectのSSR補正で PPP-RTK 測位ができる。<br>
水平精度 < 0.06 m (CEP)、収束時間 < 45 s (データシート Table 4)。

### ノード構成

| ノード | 役割 |
| --- | --- |
| `pointperfect_connect` | Thingstream の MQTT ブローカへ TLS 接続し、SPARTN補正と動的キーを取得して `/pointperfect/correction_data` (`UInt8MultiArray`) へ発行 |
| `f9p_driver` | NMEAの発行と、補正データのF9Pシリアルポートへの書き戻し。起動時にUBX-CFG-VALSETで受信機を設定 |
| `nmea_topic_driver` | NMEA から `/fix` などへ変換 |

購読するMQTTトピックは以下の3つ。いずれも受信機がそのまま解釈できる形式なので、加工せずシリアルへ流す。

| MQTTトピック | 内容 |
| --- | --- |
| `/pp/ip/<region>` | SPARTN 2.0.1 補正データ |
| `/pp/ubx/0236/ip` | UBX-RXM-SPARTNKEY (動的復号キー)。**これが無いと補正が効かない** |
| `/pp/ubx/mga` | AssistNow Online (任意。`enable_assistnow: true` で有効) |

### 認証情報の設定

Thingstream で PointPerfect の Location Thing を作成し、Credentials から
Client ID・クライアント証明書・秘密鍵・Amazon Root CA を取得する。<br>
`config/pointperfect.yaml` を編集するか、運用機では `/data/rsf_config/pointperfect_param/config.yaml`
に置く (存在すればそちらが優先される)。

```yaml
pointperfect_connect:
  ros__parameters:
    client_id: "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"
    cert_file: "/data/rsf_config/pointperfect_param/device-xxxx-pp-cert.crt"
    key_file:  "/data/rsf_config/pointperfect_param/device-xxxx-pp-key.pem"
    ca_file:   "/data/rsf_config/pointperfect_param/AmazonRootCA1.pem"
    region: "jp"     # us / eu / kr / au / jp
```

u-center用の `ucenter-config.json` をダウンロード済みなら、そのパスを `credentials_file`
に指定するだけでよい (Client ID と証明書・鍵をJSONから自動抽出し、一時ファイルへ展開する)。

```yaml
    credentials_file: "/data/rsf_config/pointperfect_param/ucenter-config.json"
```

### 受信機の自動設定

`gps_pointperfect.launch.py` では `configure_receiver:=True` (既定) で、起動時に
UBX-CFG-VALSET を送って以下を設定する。適用先は **RAMレイヤのみ** なので、
電源を入れ直せば受信機のFlash設定に戻る。

* SPARTN / RTCM3 / UBX 入力の有効化 (`CFG-<port>INPROT-*`)
* `CFG-SPARTN-USE_SOURCE = IP`
* NMEA GGA / **ZDA** / RMC 出力の有効化 (`CFG-MSGOUT-NMEA_ID_*`)
* 測位レート `CFG-RATE-MEAS` / `CFG-RATE-NAV` (既定 5 Hz)

u-center等で設定済みの受信機を書き換えたくない場合は無効化する。

```
ros2 launch f9p_ichimill gps_pointperfect.launch.py configure_receiver:=False
```

設定が受理されると `Receiver configuration accepted (UBX-ACK-ACK).` がログに出る。<br>
NAK が返る場合は `interface` パラメータ (usb / uart1 / uart2 / i2c / spi) と
ファームウェアバージョン (HPG 1.32以上) を確認すること。

### 主なlaunch引数

| 引数 | 既定値 | 説明 |
| --- | --- | --- |
| `port` | `/dev/GNSS_SERIAL` | F9Pのシリアルポート |
| `baud` | `230400` | ボーレート |
| `interface` | `usb` | F9P側の接続インタフェース |
| `nav_rate_hz` | `5.0` | 測位レート [Hz]。ZED-F9PのRTKは最大7 Hz |
| `configure_receiver` | `True` | 起動時のUBX-CFG-VALSET送信 |
| `debug_pointperfect` | `False` | MQTT受信のデバッグログ |
| `debug_driver` | `False` | シリアル入出力のデバッグログ |

### 動作確認

```
# SPARTN補正が流れているか
ros2 topic hz /pointperfect/correction_data

# 測位品質 (GGAの第6フィールド: 4=RTK Fix, 5=RTK Float/PPP-RTK, 1=単独測位)
ros2 topic echo /nmea_gga

# 変換後の測位結果
ros2 topic echo /fix
```

起動直後に `SPARTN dynamic keys received (... UBX-RXM-SPARTNKEY).` がログに出れば
動的キーの取得に成功している。

### 補足

* PointPerfectのL-band配信を使う場合はNEO-D9S等の受信機が別途必要。本パッケージはIP(MQTT)配信のみ対応。
* PointPerfectをNTRIP経由 (RTCM) で契約している場合は、既存の `gps_ntripcaster.launch.py` が利用できる。
* 補正データトピックは `f9p_driver` の `correction_topic` パラメータで切り替えられるため、
  ichimill / NtripCaster / PointPerfect で同じドライバを共用している。
