from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Launch the move_basic node
        Node(
            package='move_basic',
            executable='move_basic',
            name='move_basic',
            output='screen',
            parameters=[{
                'robot_width': 0.20,
                'robot_front_length': 0.1,
                'robot_back_length': 0.32,
                
                # Lateral control parameters
                'min_side_dist': 0.3,
                'max_side_dist': 2.0,
                'max_follow_dist_without_wall': 1.0,
                'max_angular_deviation': 0.25,
                
                # PID controller parameters
                'lateral_kp': 5.0,
                'lateral_ki': 0.0,
                'lateral_kd': 60.0
            }]
        ),
    ])
