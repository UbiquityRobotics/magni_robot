from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    magni_gazebo_pkg = get_package_share_directory('magni_gazebo')

    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument('camera_position', default_value='tower', description='Camera position'),
        DeclareLaunchArgument('lidar_position', default_value='shell_left', description='Lidar position'),
        DeclareLaunchArgument('sonars_installed', default_value='true', description='Whether sonars are installed'),
        DeclareLaunchArgument('shell_installed', default_value='true', description='Whether shell is installed'),
        DeclareLaunchArgument('tower_installed', default_value='true', description='Whether tower is installed'),
        DeclareLaunchArgument('rviz_config', default_value='true', description='Whether to load RViz config'),
        DeclareLaunchArgument('gui', default_value='true', description='Enable GUI for Gazebo'),
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo in headless mode'),
        DeclareLaunchArgument('debug', default_value='false', description='Enable debug mode'),
        DeclareLaunchArgument('paused', default_value='false', description='Start Gazebo paused'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),

        # Include Gazebo empty world launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'headless': LaunchConfiguration('headless'),
                'paused': LaunchConfiguration('paused'),
                'debug': LaunchConfiguration('debug'),
                'use_sim_time': 'true'
            }.items()
        ),

        # Include Magni robot launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(magni_gazebo_pkg, 'launch', 'magni.launch.py')
            ),
            launch_arguments={
                'camera_position': LaunchConfiguration('camera_position'),
                'sonars_installed': LaunchConfiguration('sonars_installed'),
                'lidar_position': LaunchConfiguration('lidar_position'),
                'tower_installed': LaunchConfiguration('tower_installed'),
                'shell_installed': LaunchConfiguration('shell_installed')
            }.items()
        ),

        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=[LaunchConfiguration('rviz_config')],
            output='screen'
        ),

        # RQt Robot Steering Node
        Node(
            package='rqt_robot_steering',
            executable='rqt_robot_steering',
            name='magni_rqt_teleop',
            parameters=[{
                'default_topic': '/ubiquity_velocity_controller/cmd_vel',
                'default_vx_max': 0.8,
                'default_vx_min': -0.8,
                'default_vw_max': 1.5,
                'default_vw_min': -1.5
            }]
        ),
    ])
