"""u-blox PointPerfect (SPARTN / MQTT) で補正済み位置情報を取得する launch。

  f9p_driver          : ZED-F9P とのシリアル通信、NMEA 発行、補正データ書き込み
  pointperfect_connect: Thingstream から SPARTN 補正と動的キーを MQTT で取得
  nmea_topic_driver   : NMEA から /fix などへ変換
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 運用機での設定ファイル配置先 (存在すればこちらを優先する)
DEPLOYED_CONFIG = '/data/rsf_config/pointperfect_param/config.yaml'


def resolve_config_file():
    if os.path.exists(DEPLOYED_CONFIG):
        return DEPLOYED_CONFIG
    return os.path.join(
        get_package_share_directory('f9p_ichimill'), 'config', 'pointperfect.yaml')


def generate_launch_description():
    config_file = resolve_config_file()

    # launchコマンドの引数を宣言
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/GNSS_SERIAL', description='Serial port for F9P receiver'
    )
    baud_arg = DeclareLaunchArgument(
        'baud', default_value='230400', description='Baud rate for F9P receiver'
    )
    interface_arg = DeclareLaunchArgument(
        'interface', default_value='usb',
        description='F9P interface in use: usb / uart1 / uart2 / i2c / spi'
    )
    nav_rate_arg = DeclareLaunchArgument(
        'nav_rate_hz', default_value='5.0',
        description='Navigation rate [Hz]. RTK/PPP-RTK is limited to 7 Hz on ZED-F9P.'
    )
    configure_arg = DeclareLaunchArgument(
        'configure_receiver', default_value='True',
        description='Send UBX-CFG-VALSET at startup to enable SPARTN input and NMEA output'
    )
    debug_pointperfect_arg = DeclareLaunchArgument(
        'debug_pointperfect', default_value='False', description='Enable debug logging'
    )
    debug_driver_arg = DeclareLaunchArgument(
        'debug_driver', default_value='False', description='Enable debug logging'
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        interface_arg,
        nav_rate_arg,
        configure_arg,
        debug_pointperfect_arg,
        debug_driver_arg,

        Node(
            package='f9p_ichimill',
            executable='f9p_driver',
            name='f9p_driver',
            output='screen',
            parameters=[
                {'port': LaunchConfiguration('port')},
                {'baud': LaunchConfiguration('baud')},
                {'debug': LaunchConfiguration('debug_driver')},
                # PointPerfect の SPARTN を受け取るトピック
                {'correction_topic': '/pointperfect/correction_data'},
                {'interface': LaunchConfiguration('interface')},
                {'configure_receiver': LaunchConfiguration('configure_receiver')},
                # 設定は RAM のみに適用する (電源再投入で受信機の元設定に戻る)
                {'config_layers': 'ram'},
                # PointPerfect は IP 配信なので SPARTN 源を IP にする
                {'spartn_source': 'ip'},
                {'nav_rate_hz': LaunchConfiguration('nav_rate_hz')},
                # ZDA は時刻同期に使うため有効化する
                {'nmea_sentences': ['gga', 'zda', 'rmc']},
            ]
        ),
        Node(
            package='f9p_ichimill',
            executable='pointperfect_connect',
            name='pointperfect_connect',
            output='screen',
            parameters=[
                config_file,
                {'debug': LaunchConfiguration('debug_pointperfect')},
            ]
        ),
        Node(
            package='nmea_navsat_driver',
            executable='nmea_topic_driver',
            name='nmea_topic_driver',
        ),
    ])
