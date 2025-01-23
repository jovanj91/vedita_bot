from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rp_ultrasonic_sensor',
            executable='serial_publisher',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baud_rate': 115200
            }]
        )
    ])
