# Magni Robot Simulation Stack

This repository contains the simulation and description files for the Ubiquity Robotics Magni robot, compatible with ROS 2 Jazzy.

## System Requirements

*   **Operating System:** Ubuntu 24.04 (Noble Numbat)
*   **ROS Version:** ROS 2 Jazzy Jalisco

## Installation Guide

### 1. Install ROS 2 Dependencies

To ensure all simulation, control, and navigation features work correctly, you must install the following system packages.

**Simulation & Control:**
*   `ros-jazzy-ros-gz`: Provides the bridge between ROS 2 and Gazebo, allowing topic communication.
*   `ros-jazzy-ros2-control`: The core framework for hardware abstraction and robot control in ROS 2.
*   `ros-jazzy-ros2-controllers`: Contains standard controllers like `diff_drive_controller` and `joint_state_broadcaster`.
*   `ros-jazzy-gz-ros2-control`: A Gazebo plugin that allows `ros2_control` to drive the simulated robot hardware.
*   `ros-jazzy-xacro`: The XML macro tool used to parse `.urdf.xacro` files into final URDF descriptions.
*   `ros-jazzy-robot-state-publisher`: Publishes the static and dynamic TF (transform) tree based on the URDF and joint states.
*   `ros-jazzy-joint-state-publisher`: Publishes joint state messages, essential for simulation visualization.

**Navigation & Perception:**
*   `ros-jazzy-navigation2`: The complete ROS 2 Navigation Stack (Nav2) for autonomous path planning and driving.
*   `ros-jazzy-nav2-bringup`: Contains standard launch files and configurations for bringing up the Nav2 stack.
*   `ros-jazzy-camera-info-manager`: A C++ interface for saving and restoring camera calibration data (required by perception nodes).
*   `ros-jazzy-rviz-visual-tools`: A helper library for displaying visual markers in RViz (required by `iris_lama` SLAM).
*   `ros-jazzy-gps-umd`: Tools for parsing and processing GPS data (required by `iris_lama` SLAM).
*   `libpcap-dev`: System library for network packet capture, required to compile the `lslidar_driver`.

**Installation Command:**

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-camera-info-manager \
  ros-jazzy-rviz-visual-tools \
  ros-jazzy-gps-umd \
  libpcap-dev
```

### 2. Build the Workspace

Navigate to your workspace root and build the packages using `colcon`. The `--symlink-install` flag is recommended for development to reflect changes in Python scripts and launch files without rebuilding.

```bash
cd ~/code  # Adjust to your workspace root
colcon build --symlink-install
```

### 3. Launching the Simulation

To launch the full simulation stack, including the robot description, Gazebo environment, and EzMap application layers:

1.  **Source the workspace:**
    ```bash
    source install/setup.bash
    ```

2.  **Run the main launch file:**
    ```bash
    ros2 launch ezmap_bringup gazebo.launch.py
    ```

## Repository Structure

*   **magni_description**: Contains the URDF robot description files and 3D mesh assets (`.dae`).
*   **magni_gazebo**: Contains Gazebo simulation configuration, worlds, and launch entry points.
*   **magni_bringup**: Contains hardware-specific configurations and bringup scripts for the physical robot.

## Troubleshooting

**Invisible Robot in Gazebo:**
If the robot model does not appear in the simulation, ensure that you have sourced the workspace (`source install/setup.bash`) in the terminal where you run the launch command. This ensures Gazebo can locate the mesh files via the package path.