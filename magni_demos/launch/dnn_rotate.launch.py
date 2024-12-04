from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    raspicam_node_pkg = get_package_share_directory('raspicam_node')
    magni_nav_pkg = get_package_share_directory('magni_nav')
    dnn_detect_pkg = get_package_share_directory('dnn_detect')

    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument('camera', default_value='raspicam_node', description='Camera node'),
        DeclareLaunchArgument('image', default_value='image', description='Image topic'),
        DeclareLaunchArgument('transport', default_value='compressed', description='Transport type'),

        # Include the raspicam_node launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(raspicam_node_pkg, 'launch', 'camerav2_410x308_30fps.launch.py')
            )
        ),

        # Include the move_basic launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(magni_nav_pkg, 'launch', 'move_basic.launch.py')
            )
        ),

        # DNN detect node
        Node(
            package='dnn_detect',
            executable='dnn_detect',
            name='dnn_detect',
            output='screen',
            parameters=[{
                'image_transport': LaunchConfiguration('transport'),
                'publish_images': True,
                'data_dir': os.path.join(dnn_detect_pkg, 'model'),
                'rotate_flag': 1,
                'single_shot': True
            }],
            remappings=[
                ('/camera/compressed', [LaunchConfiguration('camera'), '/', LaunchConfiguration('image'), '/', LaunchConfiguration('transport')]),
                ('/camera_info', [LaunchConfiguration('camera'), '/camera_info'])
            ]
        ),

        # DNN rotate node
        Node(
            package='dnn_rotate',
            executable='rotate.py',
            name='dnn_rotate',
            output='screen'
        )
    ])
