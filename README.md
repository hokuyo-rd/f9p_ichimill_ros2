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

```
ls -l /dev/GNSS_SERIAL
ls -l /dev/ttyACM0
```

/dev/GNSS_SERIAL が存在し、/dev/ttyACM0 を指していること。

例: lrwxrwxrwx 1 root root 7 Nov 16 11:45 /dev/GNSS_SERIAL -> ttyACM0/dev/ttyACM0 のパーミッションが $666$ になっていること。
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