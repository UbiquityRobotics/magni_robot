from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_magni_description = get_package_share_directory('magni_description')

    arg_robot_type = DeclareLaunchArgument(
        'robot_type',
        default_value='gen_6_mini',
        choices=['gen_5_silver', 'gen_6_mini', 'gen_6_midi', 'gen_6_microtractor'],
        description='Robot variant to launch'
    )
    
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        choices=['true', 'false'], description='sim time'
    )

    robot_type = LaunchConfiguration('robot_type')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Construct filename: robot_type + ".urdf.xacro"
    filename_sub = PythonExpression(["'", robot_type, ".urdf.xacro'"])
    
    xacro_file = PathJoinSubstitution([
        pkg_magni_description,
        'urdf',
        'robots',
        filename_sub
    ])

    sonars_installed = LaunchConfiguration('sonars_installed', default='true')
    shell_installed = LaunchConfiguration('shell_installed', default='false')
    tower_installed = LaunchConfiguration('tower_installed', default='false')
    lidar_extrinsics_file = LaunchConfiguration('lidar_extrinsics_file', default='')
    camera_extrinsics_file = LaunchConfiguration('camera_extrinsics_file', default='')
    gps_extrinsics_file = LaunchConfiguration('gps_extrinsics_file', default='')
    camera_installed = LaunchConfiguration('camera_installed', default='true')
    gps_installed = LaunchConfiguration('gps_installed', default='none')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {
                # Force xacro output to be treated as a plain string (not YAML) and ensure spacing
                'robot_description': ParameterValue(
                    Command([
                        'xacro', ' ', xacro_file,
                        ' ', 'use_sim:=', use_sim_time,
                        ' ', 'sonars_installed:=', sonars_installed,
                        ' ', 'shell_installed:=', shell_installed,
                        ' ', 'tower_installed:=', tower_installed,
                        ' ', 'lidar_extrinsics_file:=', lidar_extrinsics_file,
                        ' ', 'camera_extrinsics_file:=', camera_extrinsics_file,
                        ' ', 'gps_extrinsics_file:=', gps_extrinsics_file,
                        ' ', 'camera_installed:=', camera_installed,
                        ' ', 'gps_installed:=', gps_installed,
                    ]),
                    value_type=str,
                )
            },
        ]
    )

    return LaunchDescription([
        arg_robot_type,
        arg_use_sim_time,
        robot_state_publisher
    ])
