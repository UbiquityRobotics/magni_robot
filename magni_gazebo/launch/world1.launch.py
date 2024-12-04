import os
from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, SetLaunchConfiguration, ExecuteProcess)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ros_gz_bridge.actions import RosGzBridge
import xacro


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    magni_description_pkg = get_package_share_directory('magni_description')
    magni_gazebo_pkg = get_package_share_directory('magni_gazebo')

    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gz_spawn_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_spawn_model.launch.py'])
    magni_urdf_path = os.path.join(magni_description_pkg, 'urdf/', 'magni.urdf.xacro')

    urdf_file = '/tmp/magni.urdf'

    install_dir = get_package_prefix('magni_description')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] =  os.environ['GZ_SIM_RESOURCE_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] =  install_dir + "/share"

    # if 'GAZEBO_PLUGIN_PATH' in os.environ:
    #     os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    # else:
    #     os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'


    # Define launch configurations
    # camera_position = LaunchConfiguration('camera_position')
    # lidar_position = LaunchConfiguration('lidar_position')
    # sonars_installed = LaunchConfiguration('sonars_installed')
    # shell_installed = LaunchConfiguration('shell_installed')
    # tower_installed = LaunchConfiguration('tower_installed')
    # lidar_extrinsics_file = os.path.join(magni_description_pkg, 'extrinsics', f'lidar_extrinsics_{lidar_position}.yaml')
    # camera_extrinsics_file = os.path.join(magni_description_pkg, 'extrinsics', f'camera_extrinsics_{camera_position}.yaml')

    return LaunchDescription([
        # Declare launch arguments
        # DeclareLaunchArgument('camera_position', default_value='forward', description='Camera position'),
        # DeclareLaunchArgument('lidar_position', default_value='top_plate', description='Lidar position'),
        # DeclareLaunchArgument('sonars_installed', default_value='true', description='Whether sonars are installed'),
        # DeclareLaunchArgument('shell_installed', default_value='false', description='Whether shell is installed'),
        # DeclareLaunchArgument('tower_installed', default_value='false', description='Whether tower is installed'),

        #Include description launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(magni_description_pkg, 'launch', 'description.launch.py')
            ),
            # launch_arguments={
            #     'camera_extrinsics_file': camera_extrinsics_file,
            #     'shell_installed': shell_installed,
            #     'tower_installed': tower_installed,
            #     'sonars_installed': sonars_installed,
            #     'lidar_extrinsics_file': lidar_extrinsics_file
            # }.items()
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'xacro', 'xacro', magni_urdf_path, '-o', urdf_file],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': ["empty.sdf"],
                'on_exit_shutdown': 'True'
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_spawn_path),
            launch_arguments={
                'world': 'empty',
                'file': urdf_file,
                'name': 'magni_robot'
            }.items(),
        ),

        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     arguments=[],
        #     remappings=[],
        #     output='screen'
        # ),

        # # Load ROS parameters from YAML file
        # Node(
        #     package='controller_manager',
        #     executable='ros2_control_node',
        #     name='controller_spawner',
        #     arguments=['ubiquity_velocity_controller', 'ubiquity_joint_publisher'],
        #     parameters=[os.path.join(magni_gazebo_pkg, 'config', 'magni_controllers.yaml')]
        # ),

        # # Topic relays for cmd_vel and odometry
        # Node(
        #     package='topic_tools',
        #     executable='relay',
        #     name='cmd_vel_relay',
        #     arguments=['/cmd_vel', '/ubiquity_velocity_controller/cmd_vel'],
        #     output='screen'
        # ),
        # Node(
        #     package='topic_tools',
        #     executable='relay',
        #     name='odom_relay',
        #     arguments=['/ubiquity_velocity_controller/odom', '/odom'],
        #     output='screen'
        # ),

        # # Sonar topic relays
        # Node(
        #     package='topic_tools',
        #     executable='relay',
        #     name='sonar0_relay',
        #     arguments=['/pi_sonar/sonar_0', '/sonars'],
        #     output='screen'
        # ),
        # Node(
        #     package='topic_tools',
        #     executable='relay',
        #     name='sonar1_relay',
        #     arguments=['/pi_sonar/sonar_1', '/sonars'],
        #     output='screen'
        # ),
        # Node(
        #     package='topic_tools',
        #     executable='relay',
        #     name='sonar2_relay',
        #     arguments=['/pi_sonar/sonar_2', '/sonars'],
        #     output='screen'
        # ),
        # Node(
        #     package='topic_tools',
        #     executable='relay',
        #     name='sonar3_relay',
        #     arguments=['/pi_sonar/sonar_3', '/sonars'],
        #     output='screen'
        # ),
        # Node(
        #     package='topic_tools',
        #     executable='relay',
        #     name='sonar4_relay',
        #     arguments=['/pi_sonar/sonar_4', '/sonars'],
        #     output='screen'
        # ),
    ])