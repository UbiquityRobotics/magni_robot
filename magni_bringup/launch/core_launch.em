<launch>

    <arg name="sonars_installed" default="@(sonars_installed)"/>
    <arg name="oled_display" default="@(oled_display)"/>
    <arg name="controller_board_version" default="@(controller_board_version)"/>
    
    <include file="$(find magni_description)/launch/description.launch">
        <arg name="camera_extrinsics_file" value="@(camera_extrinsics_file)"/>
        <arg name="lidar_extrinsics_file" value="@(lidar_extrinsics_file)"/>
        <arg name="sonars_installed" value="$(arg sonars_installed)"/>
        <arg name="shell_installed" value="@(shell_installed)"/>
        <arg name="tower_installed" value="@(tower_installed)"/>
    </include>

    <group if="$(arg sonars_installed)">
        <node pkg="pi_sonar" type="pi_sonar" name="pi_sonar"/>
    </group>

    <group if="$(arg oled_display)">
        <node pkg="oled_display_node" type="oled_display_node" name="oled_display"/>
    </group>

    <node pkg="diagnostic_aggregator" type="aggregator_node" name="diagnostics_agg">
        <!-- Load the file you made above -->
        <rosparam command="load" file="$(find magni_bringup)/param/diagnostics_agg.yaml"/>
    </node>

    <!-- If a board version was passed in, use it -->
    <group if="$(eval controller_board_version != 0)">
	    <param name="/ubiquity_motor/controller_board_version" value="$(arg controller_board_version)"/>
    </group>

    <!-- Ubiquity motor params -->
    <param name="/ubiquity_motor/serial_port" value="@(serial_port)"/>
    <param name="/ubiquity_motor/serial_baud" value="@(serial_baud)"/>
    <param name="/ubiquity_motor/serial_loop_rate" value="@(serial_loop_rate)"/>
    <param name="/ubiquity_motor/controller_loop_rate" value="@(controller_loop_rate)"/>
    <param name="/ubiquity_motor/pid_proportional" value="@(pid_proportional)"/>
    <param name="/ubiquity_motor/pid_integral" value="@(pid_integral)"/>
    <param name="/ubiquity_motor/pid_derivative" value="@(pid_derivative)"/>
    <param name="/ubiquity_motor/pid_denominator" value="@(pid_denominator)"/>
    <param name="/ubiquity_motor/pid_moving_buffer_size" value="@(pid_moving_buffer_size)"/>
    <param name="/ubiquity_motor/pid_velocity" value="@(pid_velocity)"/>
    <param name="/ubiquity_motor/pid_control" value="@(pid_control)"/>
    <param name="/ubiquity_motor/drive_type" value="@(drive_type)"/>
    <param name="/ubiquity_motor/wheel_type" value="@(wheel_type)"/>
    <param name="/ubiquity_motor/wheel_gear_ratio" value="@(wheel_gear_ratio)"/>
    <param name="/ubiquity_motor/fw_max_pwm" value="@(fw_max_pwm)"/>
    <param name="/ubiquity_motor/fw_max_speed_fwd" value="@(fw_max_speed_fwd)"/>
    <param name="/ubiquity_motor/fw_max_speed_rev" value="@(fw_max_speed_rev)"/>

    <!-- Joint publisher params -->
    <param name="/ubiquity_joint_publisher/type" value="@(joint_controller_type)"/>
    <param name="/ubiquity_joint_publisher/publish_rate" value="@(joint_controller_publish_rate)"/>

    <!-- Ablsolute velocity controller params-->
    <param name="/ubiquity_velocity_controller/type" value="@(vel_controller_type)"/>
    <param name="/ubiquity_velocity_controller/left_wheel" value="@(left_wheel)"/>
    <param name="/ubiquity_velocity_controller/right_wheel" value="@(right_wheel)"/>
    <param name="/ubiquity_velocity_controller/publish_rate" value="@(publish_rate)"/>
    <param name="/ubiquity_velocity_controller/cmd_vel_timeout" value="@(cmd_vel_timeout)"/>
    <param name="/ubiquity_velocity_controller/enable_odom_tf" value="@(enable_odom_tf)"/>
    <param name="/ubiquity_velocity_controller/wheel_separation" value="@(wheel_separation)"/>
    <param name="/ubiquity_velocity_controller/base_frame_id" value="@(base_frame_id)"/>
    <param name="/ubiquity_velocity_controller/wheel_separation_multiplier" value="@(wheel_separation_multiplier)"/>
    <param name="/ubiquity_velocity_controller/wheel_radius_multiplier" value="@(wheel_radius_multiplier)"/>
    <param name="/ubiquity_velocity_controller/wheel_radius" value="@(wheel_radius)"/>
    <param name="/ubiquity_velocity_controller/linear/x/has_velocity_limits" value="@(lin_has_velocity_limits)"/>
    <param name="/ubiquity_velocity_controller/linear/x/max_velocity" value="@(lin_max_velocity)"/>
    <param name="/ubiquity_velocity_controller/linear/x/has_acceleration_limits" value="@(lin_has_acceleration_limits)"/>
    <param name="/ubiquity_velocity_controller/linear/x/max_acceleration" value="@(lin_max_acceleration)"/>

    <rosparam param="/ubiquity_velocity_controller/pose_covariance_diagonal">@(pose_covariance_diagonal)</rosparam>
    <rosparam param="/ubiquity_velocity_controller/twist_covariance_diagonal">@(twist_covariance_diagonal)</rosparam>

    <param name="/ubiquity_velocity_controller/angular/z/has_velocity_limits" value="@(ang_has_velocity_limits)"/>
    <param name="/ubiquity_velocity_controller/angular/z/max_velocity" value="@(ang_max_velocity)"/>
    <param name="/ubiquity_velocity_controller/angular/z/has_acceleration_limits" value="@(ang_has_acceleration_limits)"/>
    <param name="/ubiquity_velocity_controller/angular/z/max_acceleration" value="@(ang_max_acceleration)"/>

    <!-- Launch the roscontrol controllers needed -->
    <node name="controller_spawner" pkg="controller_manager" type="spawner"
        args="ubiquity_velocity_controller ubiquity_joint_publisher"/>

    <!-- Launch the motor node with the topic remapped to standard names -->
    <node name="motor_node" pkg="ubiquity_motor" type="motor_node">
        <remap from="/ubiquity_velocity_controller/cmd_vel" to="/cmd_vel"/>
        <remap from="/ubiquity_velocity_controller/odom" to="/odom"/>
    </node>
</launch>
