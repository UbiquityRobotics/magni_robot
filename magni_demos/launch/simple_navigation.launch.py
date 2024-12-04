from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    magni_nav_pkg = get_package_share_directory('magni_nav')

    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument('fiducial_len', default_value='0.140', description='Fiducial length'),

        # Set the robot mode parameter
        SetParameter(name='ubiquity_robot_mode', value='navigation'),

        # Include the move_basic launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(magni_nav_pkg, 'launch', 'move_basic.launch.py')
            )
        ),

        # Include the aruco launch file with the fiducial length argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(magni_nav_pkg, 'launch', 'aruco.launch.py')
            ),
            launch_arguments={'fiducial_len': LaunchConfiguration('fiducial_len')}.items()
        ),
    ])
