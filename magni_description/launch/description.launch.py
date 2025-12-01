#!/usr/bin/env python3


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, ExecuteProcess, RegisterEventHandler
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit

import xacro

import os

ARGUMENTS = [

        DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'], description='sim time'),
        DeclareLaunchArgument('tower_installed', default_value='false',
                          choices=['true', 'false'], description='Tower'),
        DeclareLaunchArgument('shell_installed', default_value='false',
                          choices=['true', 'false'],  description='shell'),
        DeclareLaunchArgument('sonars_installed', default_value='true', 
                          choices=['true', 'false'], description='sonars'),
        DeclareLaunchArgument('camera_extrinsics_file', default_value='extrinsics/camera_extrinsics_forward.yaml', 
                          choices=['extrinsics/camera_extrinsics_forward.yaml'], 
                          description='Path to camera extrinsics file'),
        DeclareLaunchArgument('lidar_extrinsics_file', default_value='extrinsics/lidar_extrinsics_top_plate_center.yaml', 
                          choices=['extrinsics/lidar_extrinsics_top_plate_center.yaml'],
                          description='Path to lidar extrinsics file'),
        DeclareLaunchArgument('gps_extrinsics_file', default_value='extrinsics/gps_extrinsics_top_plate.yaml', 
                          description='Path to gps extrinsics file'),
                                     ]

def generate_launch_description():
    pkg_magni_description = get_package_share_directory('magni_description')
    xacro_file = PathJoinSubstitution([pkg_magni_description,
                                       'urdf',
                                       'magni.urdf.xacro'])
    

    world_file_name = "empty.world"
    world_path = os.path.join(pkg_magni_description, "worlds", world_file_name)
    #debugging world path issue
    print(f"Computed world file path: {world_path}")

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time"),
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2 automatically"),
        # DeclareLaunchArgument("world", default_value="empty.sdf", description="Specify the Gazebo world file"),
        DeclareLaunchArgument("world", default_value=world_path, description="Specify the Gazebo world file"), #added the new world with configs
        DeclareLaunchArgument("x", default_value="0.0", description="Initial X position"),
        DeclareLaunchArgument("y", default_value="0.0", description="Initial Y position"),
        DeclareLaunchArgument("z", default_value="0.5", description="Initial Z position"),
        DeclareLaunchArgument("R", default_value="0.0", description="Initial Roll"),
        DeclareLaunchArgument("P", default_value="0.0", description="Initial Pitch"),
        DeclareLaunchArgument("Y", default_value="0.0", description="Initial Yaw"),
    ]


    # Retrieve launch configurations
    robot_description_package = 'magni_description'
    yaml_file_name = "gz_bridge.yaml"
    world_file = LaunchConfiguration('world')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    roll, pitch, yaw = LaunchConfiguration('R'), LaunchConfiguration('P'), LaunchConfiguration('Y')
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    urdf_file_name = "magni.urdf.xacro"
    urdf_path = os.path.join(pkg_magni_description, "urdf", urdf_file_name)
    yaml_path = os.path.join(pkg_magni_description, "config", yaml_file_name)
    rviz_config_file = PathJoinSubstitution([FindPackageShare(robot_description_package), "config", "robot_config.rviz"])


    xacro_mappings = {
    'tower_installed': LaunchConfiguration('tower_installed'),
    'shell_installed': LaunchConfiguration('shell_installed'),
    'sonars_installed': LaunchConfiguration('sonars_installed'),
    'lidar_extrinsics_file': LaunchConfiguration('lidar_extrinsics_file'),
    'camera_extrinsics_file': LaunchConfiguration('camera_extrinsics_file'),
    'gps_extrinsics_file': LaunchConfiguration('gps_extrinsics_file')
}




    # Process the xacro file into URDF directly
    try:
        robot_description_content = xacro.process_file(
        urdf_path
        # mappings=xacro_mappings   # This is not working for some reason. check xacro mappings
    ).toxml()
    except Exception as e:
        raise RuntimeError(f"Error processing xacro file {urdf_path}: {str(e)}")

    robot_description = {"robot_description": robot_description_content}



    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': Command([
                    'xacro',  ' ', xacro_file, ' ',

                    ' tower_installed:=', LaunchConfiguration('tower_installed'),
                    ' shell_installed:=', LaunchConfiguration('shell_installed'),
                    ' sonars_installed:=', LaunchConfiguration('sonars_installed'),
                    ' lidar_extrinsics_file:=', LaunchConfiguration('lidar_extrinsics_file'),
                    ' camera_extrinsics_file:=', LaunchConfiguration('camera_extrinsics_file'),
                    ' gps_extrinsics_file:=', LaunchConfiguration('gps_extrinsics_file')
                    ])},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    #gazebo
    gazebo_pkg_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )
    gazebo_launch = IncludeLaunchDescription(
        gazebo_pkg_launch,
        launch_arguments={'gz_args': ['-r -v4 ', world_file], 'on_exit_shutdown': 'true'}.items()
    )

    robotXacroName = 'magni_robot'
    # Spawn model in Gazebo
    spawn_model_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', '/robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
            '-allow_renaming', 'false'
        ],
        output='screen',
    )

    # RViz2 Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
        output="screen",
    )

    


    # Bridge for ROS2-Gazebo communication
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={yaml_path}'
        ],
        output='screen'
    )



    battery_faker = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub',
            '--qos-durability', 'transient_local',
            '/battery_state', 'sensor_msgs/msg/BatteryState',
            "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, voltage: 0.0, current: 0.0, charge: 0.0, capacity: 0.0, design_capacity: 0.0, percentage: 100.0, power_supply_status: 0, power_supply_health: 0, power_supply_technology: 0, present: false, cell_voltage: [0.0], location: '', serial_number: ''}"
        ],
        output='screen'
    )

    # modifications for the odom
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("magni_description"),
            "config",
            "diff_drive_controller.yaml",
        ]
    )


    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )



    # Spawn the diff_drive_controller
    diff_drive_robot_controller_spawner = TimerAction(
    period=5.0,  # INCREASED TO 5 SECONDS
    actions=[
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
            "diffbot_base_controller",
            "--param-file",
            robot_controllers,
            "--controller-ros-args",
            "-r /diffbot_base_controller/cmd_vel:=/cmd_vel",
            ],
            output="screen",
        )
        ]
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=ExecuteProcess(cmd=["ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster"]),
        on_exit=[rviz_node],
        )
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=ExecuteProcess(cmd=["ros2", "run", "controller_manager", "spawner", "diffbot_base_controller"]),
        on_exit=[joint_state_broadcaster_spawner],
        )
    )

    cmd_vel_watchdog_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("magni_description"), 'launch', 'cmd_vel_watchdog_launch.py')
        )
    )

    nodes = [
        LogInfo(msg=f"URDF Path: {urdf_path}"),
        LogInfo(msg=f"YAML Path: {yaml_path}"),
        LogInfo(msg=f"World Path: {world_file}"),
        # robot_state_publisher_node,
        controller_manager_node,
        robot_state_publisher,
        gazebo_launch,
        spawn_model_gazebo_node,
        rviz_node,
        gz_bridge_node,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        # teleop_twist_keyboard_node,
        # teleop_twist_keyboard_process,
        battery_faker,
        cmd_vel_watchdog_launch,


    ]

    return LaunchDescription(ARGUMENTS + declared_arguments + nodes)