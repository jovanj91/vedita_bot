from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rp_ultrasonic_sensor',
            executable='sensor_fusion',
            name='sensor_fusion_node',
        )
    ])
