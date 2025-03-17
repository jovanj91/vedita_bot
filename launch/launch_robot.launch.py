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

    # Declare the launch arguments
    declare_use_rplidar_arg = DeclareLaunchArgument(
        'use_rplidar',
        default_value='true',
        description='Whether to launch the RPLIDAR node'
    )

    declare_use_ultrasonic_arg = DeclareLaunchArgument(
        'use_ultrasonic',
        default_value='true',
        description='Whether to launch the Ultrasonic node'
    )
    
    declare_use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Whether to launch the Camera node'
    )

    # Define a function to conditionally add launch actions
    def conditional_launch_actions(context, *args, **kwargs):
        launch_actions = []

        if context.launch_configurations['use_rplidar'] == 'true':
            rplidar_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'rplidar.launch.py'
                )])
            )
            delayed_rplidar_launch = TimerAction(period=5.0, actions=[rplidar_launch])
            launch_actions.append(delayed_rplidar_launch)

        if context.launch_configurations['use_ultrasonic'] == 'true':
            us_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'ultrasonic.launch.py'
                )])
            )
            delayed_us_launch = TimerAction(period=8.0, actions=[us_launch])
            launch_actions.append(delayed_us_launch)

        if context.launch_configurations['use_camera'] == 'true':
            camera_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'camera.launch.py'
                )])
            )
            delayed_camera_launch = TimerAction(period=8.0, actions=[camera_launch])
            launch_actions.append(delayed_camera_launch)

        return launch_actions

    # Include the robot_state_publisher launch file
    package_name = 'vedita_bot'  # <--- CHANGE ME

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'joystick.launch.py'
        )])
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Use OpaqueFunction to conditionally add launch actions
    conditional_actions = OpaqueFunction(function=conditional_launch_actions)

    # Launch them all!
    return LaunchDescription([
        declare_use_rplidar_arg,
        declare_use_ultrasonic_arg,
        declare_use_camera_arg,
        rsp,
        joystick,
        twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        conditional_actions
    ])