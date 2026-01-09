#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
import os

def generate_launch_description():
    pkg_magni_description = get_package_share_directory('magni_description')
    pkg_magni_gazebo = get_package_share_directory('magni_gazebo')

    # Arguments
    arg_robot_type = DeclareLaunchArgument('robot_type', default_value='gen_5_silver', choices=['gen_5_silver', 'gen_6_mini', 'gen_6_midi', 'gen_6_microtractor'])
    arg_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    arg_gui = DeclareLaunchArgument('gui', default_value='true', description='Start RViz2 automatically')
    
    world_file_name = "empty.world"
    world_path = os.path.join(pkg_magni_gazebo, "worlds", world_file_name)
    arg_world = DeclareLaunchArgument("world", default_value=world_path, description="Specify the Gazebo world file")
    
    arg_x = DeclareLaunchArgument("x", default_value="0.0")
    arg_y = DeclareLaunchArgument("y", default_value="0.0")
    arg_z = DeclareLaunchArgument("z", default_value="0.5") # Wheel radius is 0.1, so z=0.5 is safe drop
    arg_roll = DeclareLaunchArgument("R", default_value="0.0")
    arg_pitch = DeclareLaunchArgument("P", default_value="0.0")
    arg_yaw = DeclareLaunchArgument("Y", default_value="0.0")

    # Launch Configurations
    robot_type = LaunchConfiguration('robot_type')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world_file = LaunchConfiguration('world')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    roll, pitch, yaw = LaunchConfiguration('R'), LaunchConfiguration('P'), LaunchConfiguration('Y')

    # Layer 1: Robot Description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_magni_description, 'launch', 'robot_description.launch.py'])
        ),
        launch_arguments={
            'robot_type': robot_type,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Gazebo Launch
    gazebo_pkg_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )
    gazebo_launch = IncludeLaunchDescription(
        gazebo_pkg_launch,
        launch_arguments={'gz_args': ['-r -v4 ', world_file], 'on_exit_shutdown': 'true'}.items()
    )

    # Spawn Model
    spawn_model_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'magni_robot',
            '-topic', 'robot_description', # Uses the topic published by Layer 1
            '-x', x, '-y', y, '-z', z,
            '-R', roll, '-P', pitch, '-Y', yaw,
            '-allow_renaming', 'false'
        ],
        output='screen',
    )

    # Configs
    yaml_file_name = "gz_bridge.yaml"
    yaml_path = os.path.join(pkg_magni_gazebo, "config", yaml_file_name)
    rviz_config_file = PathJoinSubstitution([FindPackageShare("magni_gazebo"), "config", "robot_config.rviz"])

    # Ensure Gazebo can find the meshes
    pkg_share = get_package_share_directory('magni_description')
    install_share = os.path.dirname(pkg_share)
    
    # Add both the package specific share and the general install/share
    new_paths = [install_share, pkg_share]
    
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] = os.environ['GZ_SIM_RESOURCE_PATH'] + ':' + ':'.join(new_paths)
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = ':'.join(new_paths)
    
    print(f"DEBUG: GZ_SIM_RESOURCE_PATH set to: {os.environ['GZ_SIM_RESOURCE_PATH']}")

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
        output="screen",
    )

    # Bridge
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={yaml_path}'],
        output='screen'
    )

    # Battery Faker
    battery_faker = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub',
            '--qos-durability', 'transient_local',
            '/battery_state', 'sensor_msgs/msg/BatteryState',
            "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, voltage: 0.0, current: 0.0, charge: 0.0, capacity: 0.0, design_capacity: 0.0, percentage: 100.0, power_supply_status: 0, power_supply_health: 0, power_supply_technology: 0, present: false, cell_voltage: [0.0], location: '', serial_number: ''}"
        ],
        output='screen'
    )

    # Controllers
    # No standalone controller_manager_node here, assuming Gazebo plugin starts it.
    
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("magni_gazebo"), "config", "diff_drive_controller.yaml"]
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager-timeout", "15"],
            )
        ]
    )

    diff_drive_robot_controller_spawner = TimerAction(
        period=15.0, # Give Gazebo time to start CM
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "diffbot_base_controller",
                    "--param-file",
                    robot_controllers,
                    "--controller-manager-timeout",
                    "15",
                    "--controller-ros-args",
                    "-r /diffbot_base_controller/cmd_vel:=/cmd_vel",
                ],
                output="screen",
            )
        ]
    )

    # Watchdog removed: cmd_vel is driven directly to ros2_control; watchdog caused cmd_vel hijack/duplication

    return LaunchDescription([
        arg_robot_type, arg_use_sim_time, arg_gui, arg_world,
        arg_x, arg_y, arg_z, arg_roll, arg_pitch, arg_yaw,
        
        robot_description_launch,
        gazebo_launch,
        spawn_model_gazebo_node,
        gz_bridge_node,
        # controller_manager_node, # Removed
        joint_state_broadcaster_spawner,
        diff_drive_robot_controller_spawner,
        battery_faker,
        cmd_vel_watchdog_launch,
        rviz_node
    ])
