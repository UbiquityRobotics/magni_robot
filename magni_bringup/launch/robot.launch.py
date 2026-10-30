from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_magni_description = get_package_share_directory('magni_description')
    pkg_magni_bringup = get_package_share_directory('magni_bringup')

    arg_robot_type = DeclareLaunchArgument(
        'robot_type',
        default_value='gen_6_mini',
        choices=['gen_5_silver', 'gen_6_mini', 'gen_6_midi', 'gen_6_microtractor'],
        description='Robot variant to launch'
    )
    
    robot_type = LaunchConfiguration('robot_type')

    # Layer 1: Robot Description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_magni_description, 'launch', 'robot_description.launch.py'])
        ),
        launch_arguments={
            'robot_type': robot_type,
            'use_sim_time': 'false'
        }.items()
    )

    # Controller Manager
    controller_params_file = PathJoinSubstitution([pkg_magni_bringup, "config", "controllers.yaml"])

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file],
        output="screen",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-ros-args", "-r /diffbot_base_controller/cmd_vel:=/cmd_vel"],
    )
    
    delayed_jsb_spawner = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        arg_robot_type,
        robot_description_launch,
        controller_manager_node,
        delayed_jsb_spawner,
        delayed_diff_drive_spawner
    ])
