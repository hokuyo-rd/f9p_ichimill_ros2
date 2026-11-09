# f9p_ichimill_ros2

f9p_ichimill_ros2
北陽電機 R&D室 髙橋が対応

gps_ichimill.launch.py
シリアルポートに接続されたGPSレシーバF9Pで、ichimillサービスと連携し補正済み位置情報を取得する。<br>

![rosgraph_f9p_](https://user-images.githubusercontent.com/16064762/136687925-9b3c98f7-54e4-4dc8-8176-0805cb15f15a.png)

gps_ntripcaster.launch.py
または、善意の基準局等　公開されたNtripCasterのデータを使い補正済み位置情報を取得する。<br>


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

## ichimillの自動再接続

`ichimill_connect`はネットワーク処理を専用スレッドで実行し、GGA購読を
停止させずにRTCMを継続受信します。切断、送受信エラー、一定時間のRTCM
無受信を検出すると、ソケットを閉じてから新規接続・認証をやり直します。
起動時に回線が使えない場合も再試行します。

GGAは最新の有効な位置（測位品質が0以外、チェックサム正常）のみを保持し、
接続成功後および一定間隔で送信します。古いGGA、位置欠落、未測位のGGAは
送信しません。RTK未成立時の単独測位GGAも利用できます。
GGAの終端はGPGGA/GNGGAともにCRLFに統一します。

以下は`ichimill_connect`ノードのROSパラメータです。単位は秒で、
有限の正数を指定します。既存のlaunchを変更せずに使う場合は既定値で動作します。
調整する場合はlaunchのichimill_connect用parametersに追加してください。

| パラメータ | 既定値 | 内容 |
| --- | --- | --- |
| reconnect_interval | 5.0 | 切断・接続失敗後の再試行間隔 |
| connect_timeout | 10.0 | TCP接続、NTRIP応答ヘッダ受信のタイムアウト |
| rtcm_timeout | 15.0 | 補正データの無受信を切断と判断する時間 |
| gga_interval | 1.0 | GGA送信間隔 |
| gga_max_age | 5.0 | GGAの有効期間 |

NTRIP v1のICY応答とHTTP/1.0・HTTP/1.1の200応答に対応し、
分割受信や応答ヘッダと同時に届いたRTCMを扱います。
要求はHTTP/1.0で送信します。Transfer-Encoding付き応答には対応していません。

### 回帰テスト（ROS・実機不要）

```bash
python3 -m unittest discover -s tests -v
python3 -m compileall -q scripts tests
```

ROS境界を模擬し、実際の接続処理をローカルTCPサーバで検証します。
切断・接続リセット・無通信からの再認証、最新GGA送信、RTCM配信再開、
分割ヘッダとバイナリの保持などを確認します。
ichimillサービスおよびF9Pによる実機検証は別途必要です。

### 実機での確認

1. 屋外で起動し、RTCM受信とRTK測位を確認します。
2. GNSS受信機は接続したまま、ネット回線を30秒以上切断します。
3. 回線を復帰させ、再度`NTRIP authenticated; receiving RTCM`が出ること、
   `/softbank/rtcm_data`の配信とRTK測位が復帰することを確認します。
4. 回線なしでの起動、複数回の切断・復帰も確認します。

認証成功ログはRTK FIXの成立を意味しません。RTCM配信は
`ros2 topic hz /softbank/rtcm_data`、測位状態はGGAの品質値などで確認してください。
