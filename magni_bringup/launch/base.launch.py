from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    magni_bringup_pkg = get_package_share_directory('magni_bringup')
    #magni_teleop_pkg = get_package_share_directory('magni_teleop')

    return LaunchDescription([

        # Set the robot mode parameter
        #SetParameter(name='ubiquity_robot_mode', value='teleop'),

        # Launch core node from magni_bringup package
        Node(
            package='magni_bringup',
            executable='launch_core.py',
            name='magni_launch_core',
            output='screen'
        ),

        # # Include the logitech launch file from magni_teleop
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(magni_teleop_pkg, 'launch', 'logitech.launch.py')  # Assuming it's converted to a .launch.py file
        #     )
        # ),

        # # Include the rosbridge launch file from magni_teleop
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(magni_teleop_pkg, 'launch', 'rosbridge.launch.py')  # Assuming it's converted to a .launch.py file
        #     )
        # ),
    ])
