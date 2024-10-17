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
    aruco_detect_pkg = get_package_share_directory('aruco_detect')
    fiducial_follow_pkg = get_package_share_directory('fiducial_follow')

    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument('fiducial_len', default_value='0.14', description='Length of the fiducial'),
        DeclareLaunchArgument('target_fiducial', default_value='fid49', description='Target fiducial'),
        DeclareLaunchArgument('do_pose_estimation', default_value='true', description='Perform pose estimation'),

        # Include the raspicam_node launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(raspicam_node_pkg, 'launch', 'camerav2_410x308_30fps.launch.py')
            )
        ),

        # Aruco detect node
        Node(
            package='aruco_detect',
            executable='aruco_detect',
            name='aruco_detect',
            output='screen',
            parameters=[{
                'image_transport': 'compressed',
                'publish_images': True,
                'fiducial_len': LaunchConfiguration('fiducial_len'),
                'do_pose_estimation': LaunchConfiguration('do_pose_estimation'),
                'adaptiveThreshWinSizeStep': 8,
                'adaptiveThreshWinSizeMin': 10,
                'adaptiveThreshWinSizeMax': 50,
                'doCornerRefinement': False
            }],
            remappings=[
                ('/camera/compressed', '/raspicam_node/image/compressed'),
                ('/camera_info', '/raspicam_node/camera_info')
            ]
        ),

        # Fiducial follow node
        Node(
            package='fiducial_follow',
            executable='follow.py',
            name='fiducial_follow',
            output='screen',
            parameters=[{
                'target_fiducial': LaunchConfiguration('target_fiducial')
            }]
        )
    ])
