# Magni Robot Refactoring Framework (2025)

**Objective:** Transition `magni_robot` from a monolithic, simulation-coupled structure to a scalable, orthogonal, and hermetic ROS 2 Jazzy architecture. This framework defines how we support 3 distinct robot variants (Mini, Medi, MicroTractor) while maintaining a clean separation between Simulation, Real Hardware, and Application layers.

---

## 1. Architectural Analysis

### 1.1 Current State: The "Monolithic Simulation" Anti-Pattern
Currently, the repository suffers from high coupling and low cohesion.
*   **The `magni_gazebo` Black Hole:** This package has absorbed responsibilities it should not have. It contains the URDFs, meshes, and robot configurations.
*   **Consequence:** To run the *real* robot, you are forced to install the *simulation* package. This violates the principle of separation of concerns.
*   **Fragile Inheritance:** The single `magni.urdf.xacro` uses complex conditional logic (`if sonars`, `if tower`) to toggle features. As we add more variants (MicroTractor), this combinatorial logic becomes unmaintainable ($O(2^n)$ complexity).
*   **Leaky Abstractions:** Launch files rely on relative paths or assume specific directory structures, making them brittle.

### 1.2 Target State: Orthogonal & Hermetic
We aim for a system where components can be changed independently without breaking others.
*   **Orthogonality:** Changing the *Simulation* (e.g., switching from Gazebo to Isaac Sim) should not require changing the *Robot Description*. Changing the *Navigation* stack should not require changing the *Hardware Drivers*.
*   **Hermeticity:** Each launch file and package is self-contained. It declares its inputs (arguments) and outputs (nodes/topics) clearly. It uses `FindPackageShare` to locate resources, never hardcoded paths.

---

## 2. Package Responsibility Breakdown

In the new architecture, every package has a single, well-defined purpose.

### 📦 `magni_description` (The Source of Truth)
*   **Role:** Defines the physical reality of the robot.
*   **Contents:** URDF/Xacro files, Meshes (.dae/.stl), and Robot-specific configuration (extrinsics).
*   **Dependencies:** None (Pure data).
*   **Key Concept:** This package is the *only* place where the robot's geometry and sensor locations are defined. Both Sim and Real hardware depend on this.

###  `magni_gazebo` (The Simulation Consumer)
*   **Role:** Provides the virtual environment and simulation-specific plugins.
*   **Contents:** World files, Simulation Launch files (`sim.launch.py`), Gazebo-specific config (`bridge.yaml`).
*   **Dependencies:** `magni_description`.
*   **Key Concept:** This is a *consumer* of the description. It spawns the URDF into a world.

### 📦 `magni_bringup` (The Hardware Manager)
*   **Role:** Orchestrates the real robot hardware.
*   **Contents:** Real Hardware Launch files (`robot.launch.py`), `ros2_control` hardware configurations, Sensor drivers (Lidar, Camera).
*   **Dependencies:** `magni_description`, `ubiquity_motor`.
*   **Key Concept:** This package brings the physical machine to life. It is the "Real World" equivalent of `magni_gazebo`.

### 📦 `magni_nav` (The Application Layer)
*   **Role:** Provides autonomous capabilities.
*   **Contents:** Nav2 configuration (`nav2_params.yaml`), SLAM config, Maps.
*   **Dependencies:** Standard ROS 2 Nav stack.
*   **Key Concept:** This runs *on top* of either `magni_gazebo` or `magni_bringup`. It doesn't care if the robot is real or simulated, as long as it gets `/scan` and `/odom`.

---

## 3. The "Three-Robot" Composition Strategy

Instead of one file with 50 `if` statements, we use **Composition**. We define 3 immutable "Top-Level" URDFs.

### Variant A: Magni Mini (Indoor Scout)
*   **File:** `magni_description/urdf/robots/magni_mini.urdf.xacro`
*   **Composition:** `Base Chassis` + `Low Lidar`.
*   **Use Case:** Agile indoor mapping.

### Variant B: Magni Medi (Warehouse Standard)
*   **File:** `magni_description/urdf/robots/magni_medi.urdf.xacro`
*   **Composition:** `Base Chassis` + `Low Lidar` + `Sonar Ring` + `Tower`.
*   **Use Case:** Safety-critical warehouse operations.

### Variant C: Magni MicroTractor (Outdoor/Agri)
*   **File:** `magni_description/urdf/robots/magni_microtractor.urdf.xacro`
*   **Composition:** `Base Chassis` + `High Lidar` + `GPS` + `Radar` + `Camera`.
*   **Use Case:** Outdoor GPS waypoint navigation.
*   **Crucial Detail:** The **Extrinsics** (exact XYZ/RPY of sensors) are hardcoded in this file. This makes the URDF the single source of truth for sensor positions.

---

## 4. The Hermetic Launch System

We will implement a 3-Layer Launch System to ensure scalability.

### Layer 1: State Publisher (`magni_description`)
*   **File:** `robot_description.launch.py`
*   **Input:** `robot_type` (mini, medi, microtractor).
*   **Action:** Processes the specific Xacro variant and publishes `robot_description`.
*   **Output:** TF Tree (Static Transforms).

### Layer 2: Interface (`magni_gazebo` OR `magni_bringup`)
*   **Sim File:** `sim.launch.py`
    *   Calls Layer 1.
    *   Starts Gazebo.
    *   Starts `ros_gz_bridge`.
*   **Real File:** `robot.launch.py`
    *   Calls Layer 1.
    *   Starts Motor Drivers.
    *   Starts Lidar/Camera Drivers.

### Layer 3: Capabilities (`magni_nav`, `ezmap_pro`)
*   **File:** `navigation.launch.py`
*   **Input:** `use_sim_time` (true/false).
*   **Action:** Starts Nav2, SLAM, or Web Interface.
*   **Constraint:** NEVER starts hardware drivers. Assumes the robot is already running (Layer 2).

---

## 5. Detailed Migration Plan

### Phase 1: Root & Package Cleanup
*   [ ] **Create `packaging/`**: Move `debian/`, `snap/`, `magni-description-deb/`, `magni_sim_apt/` here.
*   [ ] **Delete `magni_desktop/`**: Remove legacy metapackage.
*   [ ] **Clean Root**: Ensure only packages and `README.md` remain.

### Phase 2: Restore `magni_description` (The Great Migration)
*   [x] **Move Assets**: Transfer `meshes/` and `urdf/` from `magni_gazebo` to `magni_description`. (Redundant copies deleted).
*   [x] **Refactor URDFs**:
    *   [x] Update all `package://` and `$(find ...)` paths to point to `magni_description`.
    *   [x] Split `magni.urdf.xacro` into atomic components (`chassis`, `sensors`). (Created `magni_base.urdf.xacro`).

### Phase 3: Implement Variants
*   [x] **Create Variants**: Implement the 3 Top-Level URDFs in `magni_description/urdf/robots/`.
*   [ ] **Verify**: Use `check_urdf` to validate the TF tree for each variant. (Skipped: `xacro` tool missing in environment).

### Phase 4: Launch System Implementation
*   [x] **Layer 1**: Create `magni_description/launch/robot_description.launch.py`.
*   [x] **Layer 2 (Sim)**: Update `magni_gazebo/launch/sim.launch.py`.
*   [x] **Layer 2 (Real)**: Create `magni_bringup/launch/robot.launch.py`.

---

## 6. Quality Assurance & CI (Professional Standards)

To ensure the repo remains stable and professional, we enforce the following:

### 6.1 Linting & Formatting
*   **Python:** Enforce `flake8` and `black` formatting.
*   **XML/Launch:** Enforce `xmllint` for package.xml and launch files.
*   **Action:** Add `ament_lint_auto` and `ament_flake8` to `package.xml` test dependencies.

### 6.2 Continuous Integration (GitHub Actions)
*   **Build Test:** Every PR must pass `colcon build`.
*   **Test Suite:** Every PR must pass `colcon test`.
*   **URDF Check:** Automated script to run `check_urdf` on all 3 variants to catch syntax errors.

---

## 7. Documentation & Developer Experience

### 7.1 Per-Package Documentation
*   Each package (`magni_description`, `magni_gazebo`, etc.) MUST have its own `README.md` describing:
    *   **Nodes:** What nodes are launched.
    *   **Topics:** Subscribed/Published topics.
    *   **Parameters:** Key configuration parameters.

### 7.2 Root Documentation
*   **Quickstart:** One-liner to launch Sim.
*   **Hardware Setup:** One-liner to launch Real Robot.
*   **Architecture Diagram:** A Mermaid chart showing the relationship between packages.
