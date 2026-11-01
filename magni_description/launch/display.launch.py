from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pkg_magni_description = FindPackageShare('magni_description')

    # Arguments
    robot_type_arg = DeclareLaunchArgument('robot_type', default_value='gen_6_mini')
    
    # Robot Description (reuses your existing launch file)
    robot_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_magni_description, 'launch', 'robot_description.launch.py'])
        ),
        launch_arguments={'robot_type': LaunchConfiguration('robot_type')}.items()
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_magni_description, 'config', 'urdf.rviz'])]
    )

    return LaunchDescription([
        robot_type_arg,
        robot_desc_launch,
        joint_state_publisher_gui,
        rviz_node
    ])
