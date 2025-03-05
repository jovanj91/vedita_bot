import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node

def generate_launch_description():

    declare_republish_arg = DeclareLaunchArgument(
        'republish_uncompressed',
        default_value='true',
        description='Whether to republish the compressed node'
    )

    def conditional_launch_actions(context, *args, **kwargs):
        launch_actions = []

        if context.launch_configurations['republish_uncompressed'] == 'true':
            republish_uncompressed = Node(
                package="image_transport",
                executable="republish",
                arguments=[
                    "compressed", "raw",
                    "--ros-args", "-r", "in/compressed:=/camera/image_raw/compressed",
                    "-r", "out:=/camera/image_raw/uncompressed"
                ],
                output="screen"
            )
            delayed_republish_uncompressed = TimerAction(period=3.0, actions=[republish_uncompressed])
            launch_actions.append(delayed_republish_uncompressed)


        return launch_actions

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
    
    delayed_camera_node = TimerAction(period=1.0, actions=[v4l2_camera_node])

    conditional_actions = OpaqueFunction(function=conditional_launch_actions)
    
    return LaunchDescription([
        declare_republish_arg,  
        delayed_camera_node,
        conditional_actions
    ])