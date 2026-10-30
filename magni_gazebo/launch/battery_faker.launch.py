from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    battery_faker = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub',
            '--qos-durability', 'transient_local',
            '/battery_state', 'sensor_msgs/msg/BatteryState',
            "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, voltage: 0.0, current: 0.0, charge: 0.0, capacity: 0.0, design_capacity: 0.0, percentage: 100.0, power_supply_status: 0, power_supply_health: 0, power_supply_technology: 0, present: false, cell_voltage: [0.0], location: '', serial_number: ''}"
        ],
        output='screen'
    )

    return LaunchDescription([
        battery_faker
    ])
