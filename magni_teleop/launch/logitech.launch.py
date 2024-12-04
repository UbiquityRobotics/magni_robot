from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Find the path to the config file
    magni_teleop_pkg = get_package_share_directory('magni_teleop')

    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device'
        ),
        DeclareLaunchArgument(
            'config_filepath',
            default_value=os.path.join(magni_teleop_pkg, 'param', 'logitech.yaml'),
            description='Path to the config file'
        ),

        # Launch the joy_node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': LaunchConfiguration('joy_dev'),
                'autorepeat_rate': 20
            }]
        ),

        # Launch the teleop_twist_joy node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[LaunchConfiguration('config_filepath')]
        ),
    ])

