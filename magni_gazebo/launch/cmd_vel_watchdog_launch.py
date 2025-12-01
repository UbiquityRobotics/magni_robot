from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    script_path = os.path.join(
        get_package_share_directory('magni_gazebo'),
        'scripts',
        'cmd_vel_watchdog.py'
    )

    return LaunchDescription([
        Node(
            # package='magni_description',
            executable='cmd_vel_watchdog.py',
            name='cmd_vel_watchdog',
            output='screen',
            shell=True,
            prefix='python3 ' + script_path + ' '
        )
    ])
