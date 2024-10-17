# <!-- 
#     This launch file brings up rosbridge server. It run 2 instances
#     of it, one on port 9090 without ssl, and one with ssl on
#     port 9443 with a self-signed ssl certificate.
# -->
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rosbridge_server_pkg = get_package_share_directory('rosbridge_server')
    
    return LaunchDescription([

        # Declare arguments for SSL configuration and port numbers
        DeclareLaunchArgument(
            'ssl',
            default_value='false',
            description='Enable SSL for rosbridge websocket'
        ),

        # Launch rosbridge websocket (non-SSL)
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rosbridge_server_pkg, 'launch', 'rosbridge_websocket.launch.py')
                ),
                launch_arguments={'ssl': 'false', 'port': '9090'}.items()
            ),
        ], namespace='rosbridge_ws'),

        # Launch rosbridge websocket (SSL-enabled)
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rosbridge_server_pkg, 'launch', 'rosbridge_websocket.launch.py')
                ),
                launch_arguments={
                    'ssl': 'true',
                    'port': '9443',
                    'certfile': '/etc/ssl/certs/ssl-cert-snakeoil.pem',
                    'keyfile': '/etc/ssl/private/ssl-cert-snakeoil.key'
                }.items()
            ),
        ], namespace='rosbridge_wss'),

        # Launch the tf2_web_republisher node
        Node(
            package='tf2_web_republisher',
            executable='tf2_web_republisher',
            name='tf2_web_republisher'
        ),
    ])

