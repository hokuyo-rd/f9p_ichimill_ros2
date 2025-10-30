import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='f9p_ichimill',
            executable='f9p_driver',
            name='f9p_driver',
            parameters=[
                # F9Pが接続されているシリアルポート
                {'port': '/dev/ttyACM0'},
                {'baud': 230400},
                {'debug': True},
            ]
        ),
        Node(
            package='f9p_ichimill',
            executable='ntripcaster_connect',
            name='ntripcaster_connect',
            output='screen',
            # ROS 1の<remap>はここに記述します
            remappings=[
                ('/caster/rtcm_data', '/softbank/rtcm_data')
            ],
            parameters=[
                # 接続するNtrip Casterの情報を設定します
                {'username': ''},
                {'password': 'BETATEST'},
                {'port': 2101},
                {'host': 'rtk2go.com'},
                {'mountpoint': 'MIE_UNIV'},
                {'debug': True},
            ]
        ),
        Node(
            package='nmea_navsat_driver',
            executable='nmea_topic_driver',
            name='nmea_topic_driver',
        ),
    ])