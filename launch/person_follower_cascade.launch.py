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
            parameters=[{
                'cascade_path': f"{os.path.join(package_dir, 'models', 'haarcascade_fullbody.xml')}",
                'scale_factor': 1.1,
                'min_neighbors': 5,
                'min_size': [40, 40],
                'linear_speed': 0.2,
                'angular_gain': 0.005,
                'detection_threshold': 150
            }],
        )
    ])
