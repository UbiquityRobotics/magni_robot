from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    magni_nav_pkg = get_package_share_directory('magni_nav')

    # Launch configuration variables
    map_file = LaunchConfiguration('map_file')

    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument(
            'map_file',
            default_value=os.path.join(magni_nav_pkg, 'maps', 'blank_map.yaml'),
            description='Full path to the map file to load'
        ),

        # Run the map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}]
        ),

        # Static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_map',
            arguments=['0.0', '0', '0', '0', '0', '0', '/map', '/odom', '100']
        ),

        # Run the move_base node (assumed replaced by nav2 in ROS 2)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='move_base',
            output='screen',
            parameters=[
                os.path.join(magni_nav_pkg, 'param', 'costmap_common_param.yaml'),
                os.path.join(magni_nav_pkg, 'param', 'local_costmap_param.yaml'),
                os.path.join(magni_nav_pkg, 'param', 'global_costmap_param.yaml'),
                os.path.join(magni_nav_pkg, 'param', 'base_local_planner_param.yaml'),
            ]
        ),
    ])
