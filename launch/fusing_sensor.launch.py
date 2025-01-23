from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rp_ultrasonic_sensor',
            executable='sensor_fusion',
            parameters=[
                {'laser_topic': '/scan'},
                {'ultrasonic_topics': ['/fr_mid_range', '/fr_left_range', '/fr_right_range', '/bc_mid_range']},
                {'output_topic': '/scan_fused'}
            ]
        )
    ])
