<launch>

    <arg name="sonars_installed" default="@(sonars_installed)"/>
    <arg name="oled_display" default="@(oled_display)"/>
    <arg name="controller_board_version" default="@(controller_board_version)"/>
    
    <include file="$(find magni_description)/launch/description.launch">
        <arg name="camera_extrinsics_file" value="@(camera_extrinsics_file)"/>
        <arg name="lidar_extrinsics_file" value="@(lidar_extrinsics_file)"/>
        <arg name="sonars_installed" value="$(arg sonars_installed)"/>
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

    <!-- Load the parameters used by the following nodes -->
    <rosparam command="load" file="$(find magni_bringup)/param/base.yaml" />

    <!-- If a board version was passed in, use it -->
    <group if="$(eval controller_board_version != 0)">
	    <param name="/ubiquity_motor/controller_board_version" value="$(arg controller_board_version)"/>
    </group>

    <param name="/ubiquity_motor/serial_port" value="@(serial_port)"/>
    <param name="/ubiquity_motor/serial_baud" value="@(serial_baud)"/>

    <!-- PID Params -->
    <param name="/ubiquity_motor/pid_proportional" value="@(pid_proportional)"/>
    <param name="/ubiquity_motor/pid_integral" value="@(pid_integral)"/>
    <param name="/ubiquity_motor/pid_derivative" value="@(pid_derivative)"/>
    <param name="/ubiquity_motor/pid_denominator" value="@(pid_denominator)"/>
    <param name="/ubiquity_motor/pid_moving_buffer_size" value="@(pid_moving_buffer_size)"/>
    <param name="/ubiquity_motor/pid_velocity" value="@(pid_velocity)"/>


    <!-- Launch the roscontrol controllers needed -->
    <node name="controller_spawner" pkg="controller_manager" type="spawner"
        args="ubiquity_velocity_controller ubiquity_joint_publisher"/>

    <!-- Launch the motor node with the topic remapped to standard names -->
    <node name="motor_node" pkg="ubiquity_motor" type="motor_node">
        <remap from="/ubiquity_velocity_controller/cmd_vel" to="/cmd_vel"/>
        <remap from="/ubiquity_velocity_controller/odom" to="/odom"/>

        <param name="/ubiquity_velocity_controller/wheel_separation_multiplier" value="@(wheel_separation_multiplier)"/>
        <param name="/ubiquity_velocity_controller/wheel_radius_multiplier" value="@(wheel_radius_multiplier)"/>
        <param name="/ubiquity_velocity_controller/linear/x/has_velocity_limits" value="@(has_velocity_limits)"/>
        <param name="/ubiquity_velocity_controller/linear/x/max_velocity" value="@(has_velocity_limits)"/>
        <param name="/ubiquity_velocity_controller/linear/x/has_acceleration_limits" value="@(has_acceleration_limits)"/>
        <param name="/ubiquity_velocity_controller/linear/x/max_acceleration" value="@(max_acceleration)"/>

        <param name="/ubiquity_velocity_controller/angular/z/has_velocity_limits" value="@(has_velocity_limits)"/>
        <param name="/ubiquity_velocity_controller/angular/z/max_velocity" value="@(max_velocity)"/>
        <param name="/ubiquity_velocity_controller/angular/z/has_acceleration_limits" value="@(has_acceleration_limits)"/>
        <param name="/ubiquity_velocity_controller/angular/z/max_acceleration" value="@(max_acceleration)"/>
    </node>
</launch>
