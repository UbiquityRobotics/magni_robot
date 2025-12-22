# Magni Robot Refactoring Plan
**Target:** At least somewhat professional ROS Standards 

## 1. Why This is Necessary?
The current structure suffers from mix of ROS 1 and ROS 2, packaging artifacts in the root, and entangled launch logic. This makes supporting multiple robot variants (Sim/Real, different sensors) fragile and  very difficult.
Militant like like restructuring based on the **Composition over Inheritance** principle is needed, widely used by our competitors and other successful projects like `turtlebot4` and `nav2_bringup`(these guys create deep fucking value).


## 2. Repository Architecture
**Unbearable hell if not done like this:** A clean root directory where every folder is a ROS 2 package or a standard config folder.

### 2.1 Root Cleanup
Move all nonsource shit into dedicated directories.
*   **Create `packaging/`**: put `debian/`, `snap/`, `.deb`, `.snap`, and build scripts here.
*   **Create `deploy/`**: Put `ansible` scripts or setup scripts here.
*   **`ezmap_pro/`**: should ezmap pro be a submodule or should it be a submodule of magni_robot, but ensure it is treated as an "Application Overlay" .
*   **Delete `magni_desktop/`**: This is a legacy metapackage that serves no purpose in this new architecture. It just adds noise.

### 2.2 Package Structure
Adopt a flat, functional package layout.

```text
magni_robot/ (Repo Root)
├── magni_robot/             # Metapackage (dependencies only)
├── magni_description/       # URDFs, Meshes, Robot-specific configs (for mini, midi, microtractor)
├── magni_bringup/           # Real Hardware Launch files & Runtime Configs
├── magni_gazebo/            # Simulation Launch files & Worlds
├── magni_nav/               # Nav2, MoveSmooth Configs & Maps
├── magni_teleop/            # Joystick/Keyboard control configs (robot specific)
└── packaging/               # (Non-ROS) Deb/Snap generation
```

## 3. URDF/Xacro Architecture (The "Source of Truth")
**Current Fuck up:** A giant single  `magni.urdf.xacro` with too much conditional logic (`if tower_installed`, `if lidar_installed`). This scales poorly (O(N^2) complexity).

**How it should be:** Component-based Composition.
Look at this: *TurtleBot4 Description*

### 3.1 Directory Outlook
```text
magni_description/
├── urdf/
│   ├── common/              # Materials, Inertial macros
│   ├── chassis/             # Base link, get rid of caster macros
│   ├── sensors/             # Generic sensor macros (Lidar, Camera, GNSS)
│   │   ├── generic_lidar.xacro
│   │   └── raspicam.xacro
│   └── robots/              # Top-Level Entry Points (The "Variants")
│       ├── magni_base.urdf.xacro        # Just the base
│       ├── magni_lidar.urdf.xacro       # Base + Lidar
│       └── magni_full.urdf.xacro        # Base + Tower + Camera + Lidar
```

### 3.2 Implementation Changes
Instead of passing flags to one file, we must create specific top-level files that *compose* the robot.
*   **`magni_base.urdf.xacro`**: for chassis, wheels.
*   **`magni_lidar.urdf.xacro`**: Includes `magni_base.urdf.xacro`, then includes `lidar.xacro` and calls the macro with specific extrinsics.

### 3.3 Managing the 3 Robot Variants (Mini, Medi, MicroTractor)
To handle the specific fleet requirements without spaghetti code, we define three distinct "Top Level" URDFs in `magni_description/urdf/robots/`. Each file represents a **complete, immutable configuration**.

#### A. The "Mini" (Lidar Only)
*   **File:** `magni_mini.urdf.xacro`
*   **Composition:**
    1.  Include `common/chassis.xacro` (Base robot).
    2.  Include `sensors/generic_lidar.xacro`.
    3.  **NO** Sonars, **1 or 2** Cameras.
*   **Use Case:** Basic SLAM and Navigation in tight indoor spaces.

#### B. The "Midi" (Lidar + ""Sonar"" + )
*   **File:** `magni_medi.urdf.xacro`
*   **Composition:**
    1.  Include `common/chassis.xacro`.
    2.  Include `sensors/generic_lidar.xacro`.
    3.  Include `sensors/sonar_ring.xacro` (The 5-sonar array).
*   **Use Case:** Glass wall detection, safety stop redundancy.

#### C. The "MicroTractor" (The Farm-Beast: GNSS, Lidar, Camera, Radar, IMU)
*   **File:** `magni_microtractor.urdf.xacro`
*   **Composition:**
    1.  Include `common/chassis.xacro` (Possibly with `wheel_type="offroad"` arg).
    2.  Include `sensors/generic_lidar.xacro`.
    3.  Include `sensors/raspicam.xacro` (Forward facing).
    4.  Include `sensors/gps_antenna.xacro` (On side plate above the wheels).
    5.  Include `sensors/radar.xacro` (Front bumper, maybe).
    6.  Include `sensors/imu_external.xacro` (If not using internal in the MCBs).
*   **Crucial Detail:** This file defines the *static transforms* (extrinsics) for all these sensors relative to `base_link`.

## 4. Launch System Architecture
**Goal:** Separation of "Description", "Simulation", and "Real Hardware".
Look at: *Nav2 Bringup*

### 4.1 The Hierarchy
1.  **`magni_description/launch/robot_description.launch.py`**
    *   **Sole Responsibility:** Load URDF, publish `robot_description` topic, run `robot_state_publisher`.
    *   **Args:** `urdf_file` (path to the specific variant from Section 3).

2.  **`magni_gazebo/launch/sim.launch.py`**
    *   **Responsibility:**
        1.  Include `robot_description.launch.py`.
        2.  Launch Gazebo (`ros_gz_sim`).
        3.  Spawn Robot.
        4.  Launch `ros_gz_bridge`.
        5.  Launch **Simulated** Hardware Interface (ros2_control).

3.  **`magni_bringup/launch/robot.launch.py`** (Real Robot)
    *   **Responsibility:**
        1.  Include `robot_description.launch.py`.
        2.  Launch **Real** Hardware Drivers (Lidar, Camera, Motor Driver).
        3.  Launch **Real** Hardware Interface (ros2_control).

### 4.2 Configuration Management
*   **Strict Rule:** For the love of god NO! hardcoded paths in Python. Use `FindPackageShare`.
*   **Strict Rule:** All params must be in `.yaml` files in `config/` folders, loaded via `DeclareLaunchArgument`.

## 5. Application Layer (`ezmap_pro` Integration)
**Concept:** `ezmap_pro` is an "App" that runs *on top* of the robot. It should not launch the robot's hardware drivers under any circumnstances.

### 5.1 The "Sim-Web" Bridge
Create a dedicated launch file: `magni_gazebo/launch/sim_with_web.launch.py`.
*   **Logic:**
    1.  Include `magni_gazebo/launch/sim.launch.py`.
    2.  Include `ezmap_bringup/launch/web_stack.launch.py` (A new file we must create that *only* launches the Web/React nodes, not the hardware drivers).

## 6. Code Quality & CI
**Standard:** ROS 2 Rolling/Jazzy strict mode.

1.  **Linting:** Add `ament_lint_auto` and `ament_flake8` to all `package.xml` test dependencies.
2.  **Pre-commit:** Add a `.pre-commit-config.yaml` to the root to enforce formatting (black/flake8) before git commit.
3.  **CI:** GitHub Actions workflow that runs `colcon build` and `colcon test` on every PR.

## 8. Analysis of `jazzy-devel` & Extrinsics Strategy
**Current State of `jazzy-devel`:**
The upstream branch has committed a "Monolithic Simulation" anti-pattern:
*   Moved all URDFs/Meshes into `magni_gazebo`.
*   Emptied `magni_description` and `magni_bringup`.
*   Relies on a single `magni.urdf.xacro` with hardcoded paths to `magni_gazebo`.

**Why this is bad:**
*   **Real Robot Breakage:** The real robot needs the URDF, but it shouldn't depend on `magni_gazebo`.
*   **Extrinsics Nightmare:** Currently, extrinsics are passed as file paths or arguments. This is fragile.

### 8.1 The Extrinsics Solution (Per-Robot Composition)
Instead of passing `lidar_xyz="0 0 0"` arguments through 5 layers of launch files, we define the extrinsics **inside the Top-Level URDF** for each robot variant.

**Example: `magni_microtractor.urdf.xacro`**
```xml
<robot name="magni_microtractor" xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- 1. Include Base -->
  <xacro:include filename="$(find magni_description)/urdf/common/chassis.xacro" />
  
  <!-- 2. Define Extrinsics Properties (The "Source of Truth") -->
  <xacro:property name="lidar_xyz" value="0.1 0.0 0.25" />
  <xacro:property name="gps_xyz" value="-0.05 0.15 0.30" />
  
  <!-- 3. Instantiate Sensors with these properties -->
  <xacro:include filename="$(find magni_description)/urdf/sensors/generic_lidar.xacro" />
  <xacro:generic_lidar name="lidar" parent="base_link">
    <origin xyz="${lidar_xyz}" rpy="0 0 0" />
  </xacro:generic_lidar>

  <xacro:include filename="$(find magni_description)/urdf/sensors/gps_antenna.xacro" />
  <xacro:gps_antenna name="gps" parent="base_link">
    <origin xyz="${gps_xyz}" rpy="0 0 0" />
  </xacro:gps_antenna>
</robot>
```

**Benefits:**
1.  **Explicit:** You open `magni_microtractor.urdf.xacro` and see exactly where the sensors are.
2.  **No Launch Args:** You don't need to pass `lidar_x:=0.1` when launching. You just launch `robot_type:=microtractor`.
3.  **Version Control:** Changes to extrinsics are tracked in git as changes to the robot file.
