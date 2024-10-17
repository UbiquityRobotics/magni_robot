from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument('port', default_value='9090', description='Port for connection'),
        DeclareLaunchArgument('address', default_value='localhost', description='Address to connect to'),

        # Enable statistics as a global parameter
        Node(
            package='rcl_interfaces',
            executable='set_parameters',
            name='enable_statistics_param',
            parameters=[{'enable_statistics': True}]
        ),

        # Group 1: rosapi node
        GroupAction([
            Node(
                package='rosapi',
                executable='rosapi_node',
                name='rosapi',
                respawn=True,
                parameters=[{
                    'topics_glob': '[*]',
                    'services_glob': '[*]',
                    'params_glob': '[*]'
                }]
            )
        ]),

        # Group 2: roshub_manager node
        GroupAction([
            Node(
                package='roshub_manager',
                executable='manager',
                name='roshub_manager',
                output='screen'
            )
        ]),
    ])
