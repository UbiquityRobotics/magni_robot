from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the directory for the 'ubiquity_motor_ros2' package
    ubiquity_motor_ros2_pkg = get_package_share_directory('ubiquity_motor_ros2')

    camera_ros_pkg = get_package_share_directory('camera_ros')

    # Define the path to the motors.launch.py file
    motors_launch_path = os.path.join(ubiquity_motor_ros2_pkg, 'launch', 'motors.launch.py')

    camera_launch_path = os.path.join(camera_ros_pkg, 'launch', 'camera.launch.py')

    # Include motors.launch.py
    motors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(motors_launch_path)
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_path)
    )

    return LaunchDescription([
        motors_launch,
        camera_launch
    ])
