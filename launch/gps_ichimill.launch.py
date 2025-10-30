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
                {'port': '/dev/ttyACM0'},
                {'baud': 230400},
                {'debug': True},
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
                {'debug': True},
            ]
        ),
        Node(
            package='nmea_navsat_driver',
            executable='nmea_topic_driver',
            name='nmea_topic_driver',
        ),
    ])