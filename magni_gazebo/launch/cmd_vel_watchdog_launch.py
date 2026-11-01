from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    script_path = os.path.join(
        get_package_share_directory('magni_gazebo'),
        'scripts',
        'cmd_vel_watchdog.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            # package='magni_description',
            executable='cmd_vel_watchdog.py',
            name='cmd_vel_watchdog',
            output='screen',
            shell=True,
            prefix='python3 ' + script_path + ' ',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel', '/cmd_vel_stamped')]
        )
    ])
