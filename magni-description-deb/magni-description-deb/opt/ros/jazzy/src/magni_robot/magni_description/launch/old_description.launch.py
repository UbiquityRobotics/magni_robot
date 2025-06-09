from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    magni_description_pkg = get_package_share_directory('magni_description')

    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument('tower_installed', default_value='false', description='Whether tower is installed'),
        DeclareLaunchArgument('shell_installed', default_value='false', description='Whether shell is installed'),
        DeclareLaunchArgument('sonars_installed', default_value='true', description='Whether sonars are installed'),
#        DeclareLaunchArgument('camera_extrinsics_file', default_value='ahead.yaml', description='Path to camera extrinsics file'),
#        DeclareLaunchArgument('lidar_extrinsics_file', default_value='', description='Path to lidar extrinsics file'),

        # Robot description (xacro processing)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    'xacro ', os.path.join(magni_description_pkg, 'urdf', 'magni.urdf.xacro'),
                    ' tower_installed:=', LaunchConfiguration('tower_installed'),
#                    ' shell_installed:=', LaunchConfiguration('shell_installed'),
                    ' sonars_installed:=', LaunchConfiguration('sonars_installed'),
#                    ' lidar_extrinsics_file:=', LaunchConfiguration('lidar_extrinsics_file'),
#                   ' camera_extrinsics_file:=', LaunchConfiguration('camera_extrinsics_file')
                ])
            }]
        ),

        # Conditional group action to launch laser filters if tower is installed
        GroupAction(
            condition=IfCondition(LaunchConfiguration('tower_installed')),
            actions=[
                Node(
                    package='laser_filters',
                    executable='scan_to_scan_filter_chain',
                    name='laser_filter',
                    output='screen',
                    parameters=[{
                        'target_frame': 'laser',
                    }],
                    arguments=[os.path.join(magni_description_pkg, 'config', 'laser_filter_chain.yaml')]
                ),
            ]
        ),
    ])
