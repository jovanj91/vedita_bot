import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    package_dir = get_package_share_directory('vedita_bot')
    return LaunchDescription([
        Node(
            package='vedita_bot_addon',
            executable='person_follower_yolo',
            name='person_follower_yolo',
            output='screen',
            namespace='person_follower_yolo',
            parameters=[{
                'model_path': f"{os.path.join(package_dir, 'models', 'yolo11n.pt')}",
                'min_distance': 100.0 , #min threshold detected_person width
                'max_distance': 300.0, #max threshold detected_person width
                'image_width': 640.0,
            }],
        )
    ])
