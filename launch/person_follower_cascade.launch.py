import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    package_dir = get_package_share_directory('vedita_bot')
    return LaunchDescription([
        Node(
            package='vedita_bot_addon',
            executable='person_follower_hc',
            name='person_follower_hc',
            output='screen',
            namespace='person_follower_hc',
            parameters=[f"{os.path.join(package_dir, 'config', 'person_follower_hc.yaml')}"],
        )
    ])
