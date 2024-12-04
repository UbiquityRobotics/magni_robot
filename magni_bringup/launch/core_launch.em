
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define launch arguments
    sonars_installed_arg = DeclareLaunchArgument('sonars_installed', default_value='True')
    # oled_display_arg = DeclareLaunchArgument('oled_display', default_value='False')
    # controller_board_version_arg = DeclareLaunchArgument('controller_board_version', default_value='0')
    
    # # Include another launch file
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('magni_description'),
            'launch',
            'description.launch.py'
        ])),
        launch_arguments={
            'camera_extrinsics_file': '',
            'lidar_extrinsics_file': '',
            'sonars_installed': LaunchConfiguration('sonars_installed'),
            'shell_installed': 'False',
            'tower_installed': 'False'
        }.items()
    )
    #print(LaunchConfiguration('sonars_installed'))
    
    # # Sonar group
    sonars_group = GroupAction(
        actions=[
            Node(
                package='pi_sonar',
                executable='pi_sonar',
                name='pi_sonar'
            )
        ],
        condition=IfCondition(LaunchConfiguration('sonars_installed'))
    )
    
    # # OLED display group
    # oled_group = GroupAction(
    #     actions=[
    #         Node(
    #             package='oled_display_node',
    #             executable='oled_display_node',
    #             name='oled_display'
    #         )
    #     ],
    #     condition=IfCondition(LaunchConfiguration('oled_display'))
    # )
    
    # # Diagnostics aggregator node
    # diagnostics_agg_node = Node(
    #     package='diagnostic_aggregator',
    #     executable='aggregator_node',
    #     name='diagnostics_agg',
    #     parameters=[PathJoinSubstitution([
    #         FindPackageShare('magni_bringup'),
    #         'param',
    #         'diagnostics_agg.yaml'
    #     ])]
    # )
    
    # # Controller board group
    # controller_board_group = GroupAction(
    #     actions=[
    #         Node(
    #             package='ros2_param',
    #             executable='param_node',
    #             parameters=[{'/ubiquity_motor/controller_board_version': LaunchConfiguration('controller_board_version')}]
    #         )
    #     ],
    #     condition=IfCondition(LaunchConfiguration('controller_board_version') != '0')
    # )

    # Define the path to the xacro file
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('magni_description'), 'urdf', 'magni.urdf.xacro']
    )

    # Define the command to run xacro as a subprocess
    xacro_command = Command(
        [
            'xacro ', xacro_file,
        ]
    )

    # Use ExecuteProcess to run the xacro command
    run_xacro = ExecuteProcess(
        cmd=['xacro', xacro_file],
        # output='screen',
        shell=True
    )

    # Use the robot_description generated from the xacro command
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': xacro_command}]
    )

    # ros2_control_node
    # Step 1: Run the stty command to configure the serial port
    serial_config = ExecuteProcess(
        cmd=['sudo', 'stty', '-F', '/dev/ttyS0', 'sane'],
        shell=True
    )

        # Path to the test.yaml configuration file
    config_file = PathJoinSubstitution(
        [FindPackageShare('ubiquity_motor_ros2'), 'cfg', 'conf.yaml']
    )

    # Step 2: Run the ros2_control_node with parameters
    controller_node = launch_ros.actions.Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/ubiquity_velocity_controller/cmd_vel', '/cmd_vel')  # Remap the cmd_vel topic
        ]
    )
    
    # Spawning the controller using spawner command
    spawn_controller = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner', 'ubiquity_velocity_controller'
        ],
        output='screen'
    )

    return LaunchDescription([
        sonars_installed_arg,
        # oled_display_arg,
        # controller_board_version_arg,
        description_launch,
        sonars_group,
        # oled_group,
        # diagnostics_agg_node,
        # controller_board_group,
        run_xacro,
        robot_state_publisher,  # Starts the robot_state_publisher with xacro output
        serial_config,
        controller_node,
        spawn_controller
    ])
