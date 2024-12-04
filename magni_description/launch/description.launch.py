from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node

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
                                     ]

def generate_launch_description():
    pkg_magni_description = get_package_share_directory('magni_description')
    xacro_file = PathJoinSubstitution([pkg_magni_description,
                                       'urdf',
                                       'magni.urdf.xacro'])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': Command([
                    'xacro',  ' ', xacro_file, ' ',

                    ' tower_installed:=', LaunchConfiguration('tower_installed'),
                    ' shell_installed:=', LaunchConfiguration('shell_installed'),
                    ' sonars_installed:=', LaunchConfiguration('sonars_installed'),
                    ' lidar_extrinsics_file:=', LaunchConfiguration('lidar_extrinsics_file'),
                    ' camera_extrinsics_file:=', LaunchConfiguration('camera_extrinsics_file')
                    ])},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    return ld
