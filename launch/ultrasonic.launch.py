from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rp_ultrasonic_sensor',
            executable='serial_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-id/usb-Arduino_Nano-if00-port0',
                'baud_rate': 115200
            }]
        )
    ])
