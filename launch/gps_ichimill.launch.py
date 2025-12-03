import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # launchコマンドの引数を宣言
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/GNSS_SERIAL', description='Serial port for F9P receiver'
    )
    baud_arg = DeclareLaunchArgument(
        'baud', default_value='230400', description='Baud rate for F9P receiver'
    )
    debug_arg_ichimill = DeclareLaunchArgument(
        'debug_ichimill', default_value='True', description='Enable debug logging'
    )
    debug_arg_driver = DeclareLaunchArgument(
        'debug_driver', default_value='False', description='Enable debug logging'
    )

    return LaunchDescription([
        # 宣言した引数をLaunchDescriptionに追加
        port_arg,
        baud_arg,
        debug_arg_ichimill,
        debug_arg_driver,

        Node(
            package='f9p_ichimill',
            executable='f9p_driver',
            name='f9p_driver',
            parameters=[
                {'port': LaunchConfiguration('port')},
                {'baud': LaunchConfiguration('baud')},
                {'debug': LaunchConfiguration('debug_driver')},
            ]
        ),
        Node(
            package='f9p_ichimill',
            executable='ichimill_connect',
            name='ichimill_connect',
            output='screen',
            parameters=[
                {'username': ''},
                {'password': ''},
                {'port': 2101},
                {'host': 'ntrip.ales-corp.co.jp'},
                {'mountpoint': '32M7NHS'},
                {'debug': LaunchConfiguration('debug_ichimill')},
            ]
        ),
        Node(
            package='nmea_navsat_driver',
            executable='nmea_topic_driver',
            name='nmea_topic_driver',
        ),
    ])
