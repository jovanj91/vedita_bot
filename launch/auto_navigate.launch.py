from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vedita_bot_addon',  
            executable='send_goal_loop',    
            output='screen',
        ),
    ])
