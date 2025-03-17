import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.actions import Node

def generate_launch_description():

    v4l2_camera_node = Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            namespace='camera',
            parameters=[{
                'image_size': [640,480],
                'time_per_frame': [1, 6],
                'camera_frame_id': 'camera_link_optical',
                # 'output_encoding': 'yuv422_yuy2'
            }]
        ) 
 
    return LaunchDescription([
        v4l2_camera_node
    ])