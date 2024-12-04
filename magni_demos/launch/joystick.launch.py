from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory for magni_bringup
    magni_bringup_pkg = get_package_share_directory('magni_bringup')

    return LaunchDescription([

        # Include the base launch file from magni_bringup package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(magni_bringup_pkg, 'launch', 'base.launch.py')
            )
        ),
    ])
