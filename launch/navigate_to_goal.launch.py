from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare a single parameter for the goal as a dictionary (JSON-like string)
        DeclareLaunchArgument(
            'goal',
            default_value='{"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}',
            description='Goal position and orientation as a dictionary'
        ),
 
        # Node execution with a single parameter
        Node(
            package='vedita_bot_addon',
            executable='send_goal',
            name='nav2_goal_sender',
            parameters=[{'goal': LaunchConfiguration('goal')}]
        )
    ])
