import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    params = get_package_share_directory('vedita_bot')
    return LaunchDescription([
        Node(
            package='vedita_bot_addon',
            executable='person_follower_yolo',
            name='person_follower_yolo',
            output='screen'
        )
    ])
