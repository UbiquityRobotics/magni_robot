from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    raspicam_pkg = get_package_share_directory('raspicam_node')
    aruco_detect_pkg = get_package_share_directory('aruco_detect')
    fiducial_slam_pkg = get_package_share_directory('fiducial_slam')

    # Launch configuration variables
    mapping_mode = LaunchConfiguration('mapping_mode')
    fiducial_len = LaunchConfiguration('fiducial_len')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'mapping_mode',
            default_value='false',
            description='Enable or disable mapping mode'
        ),
        DeclareLaunchArgument(
            'fiducial_len',
            default_value='0.140',
            description='Length of the fiducial marker'
        ),

        # Include camera launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(raspicam_pkg, 'launch', 'camerav2_1280x960_10fps.launch.py')
            )
        ),

        # Include fiducial detection launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(aruco_detect_pkg, 'launch', 'aruco_detect.launch.py')
            ),
            launch_arguments={
                'camera': '/raspicam_node',
                'image': 'image',
                'transport': 'compressed',
                'fiducial_len': fiducial_len
            }.items()
        ),

        # Include fiducial slam launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fiducial_slam_pkg, 'launch', 'fiducial_slam.launch.py')
            ),
            launch_arguments={
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'future_date_transforms': '0.5'
            }.items()
        ),
    ])

