from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the RViz configuration file
    magni_viz_pkg = get_package_share_directory('magni_viz')
    rviz_config_file = os.path.join(magni_viz_pkg, 'rviz', 'view_nav.rviz')
    
    return LaunchDescription([
        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])

