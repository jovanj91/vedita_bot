import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '',
                'frame_id': 'ultrasonic_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard',
                'serial_baudrate': 115200
        
            }]
        )
    ])