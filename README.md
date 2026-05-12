# dVRK Visual Servoing & Dynamic Tissue Excision Simulation

**Author:** Gergely
**Context:** 6th Semester IT Engineering (Artificial Intelligence Specialization), Óbuda University (OE)
**Project:** Half-Year Robotics Assignment

## 📌 Project Overview
This project implements a closed-loop kinematic controller for the da Vinci Research Kit (dVRK) Patient Side Manipulator (PSM1) in a ROS 2 environment. The system demonstrates advanced robotic surgery primitives, specifically automated circumferential tissue cutting and dynamic object grasping, utilizing continuous visual feedback and spherical linear interpolation (SLERP) for smooth 6-DOF execution.

## 🏗️ Architecture & ROS 2 Communication
The project is divided into an environment layer and a task-execution layer, utilizing standard ROS 2 Data Distribution Service (DDS) communication protocols to ensure real-time responsiveness.

### 1. Launch System
To ensure scalability and eliminate hardcoded paths, the system uses Python-based ROS 2 launch files:
* **`dvrk_env.launch.py`**: Dynamically locates the `dvrk_config` and `dvrk_model` packages, initializing the PSM1 simulated kinematics, `robot_state_publisher`, and loading the custom RViz2 environment. Includes timed actions to ensure the dVRK core is fully online before spawning visualization tools.
* **`dvrk_tasks.launch.py`**: Concurrently boots the interactive target server and the dynamic PSM control logic, keeping terminal outputs synchronized.

### 2. Core Nodes
#### Node A: `interactive_marker.py` (DVRKInteractiveMarker)
This node generates a highly customizable, 6-DOF interactive target within RViz2, acting as the simulated "tissue" for the PSM.
* **ROS Concept:** Implements an `InteractiveMarkerServer`.
* **Topics Published:** `/dvrk_viz/interactive_target_pose` (Streams real-time 6-DOF coordinates to the robotic arm).
* **Topics Subscribed:** `/dvrk_viz/force_marker_pose` (Allows the robot to artificially drag/move the marker once grasped).
* **Logic:** Uses `scipy.spatial.transform.Rotation` to prevent gimbal lock when initializing orientations. It features 6 independent rings for manual X/Y/Z translation and rotation during runtime.

#### Node B: `psm_interactive_grasp.py` (PSMDynamicController)
This is the "brain" of the operation. It drives the PSM1 hardware/simulation through a complex, multi-stage surgical procedure.
* **Topics Subscribed:** * `/PSM1/measured_cp` (Live end-effector Cartesian pose)
    * `/PSM1/jaw/measured_js` (Live jaw angle)
    * `/dvrk_viz/interactive_target_pose` (Live target coordinates)
* **Topics Published:** `/PSM1/servo_cp` and `/PSM1/jaw/servo_jp`.
* **Logic & Algorithms:**
    * **Data Synchronization:** Pauses execution until all telemetry streams are verified to prevent blind movements.
    * **Closed-Loop Chasing:** Calculates the difference between `measured_cp` and the target vector at high frequency. This ensures the robot accurately hits moving targets instead of relying on open-loop blind predictions.
    * **SLERP Interpolation:** Uses Spherical Linear Interpolation to smoothly transition the tooltip's quaternion orientation downwards toward the target geometry.
    * **Task Sequence:** 1. Moves to the geometric edge of the marker.
        2. Executes a circumferential cut (dynamically calculating point-to-point arcs and operating the jaw mechanism).
        3. Dives to the absolute geometric center.
        4. Clamps the jaw to secure the grasp.
        5. Extracts the target, simultaneously publishing to the force topic to drag the visual marker with it.

## 🚀 Installation & Usage

**1. Build the package:**
Ensure you are in the root of your ROS 2 workspace.
```bash
colcon build --packages-select ros2_course --symlink-install
source install/setup.bash
```

2. Start the Simulation Environment:
In Terminal 1, launch the dVRK core and RViz.

Bash
ros2 launch ros2_course dvrk_env.launch.py
⚠️ Important: Once RViz loads, you MUST use the GUI buttons to HOME the PSM1 robot before proceeding.

3. Execute the Task:
In Terminal 2, launch the interactive marker and the autonomous control sequence.

Bash
source install/setup.bash
ros2 launch ros2_course dvrk_tasks.launch.py
✅ Grading Rubric Fulfillment
Completeness of the solution: Fully automated, multi-stage surgical routine handling edge-case synchronizations and continuous telemetry processing.

Proper ROS communication: Utilizes Custom Publishers/Subscribers, Timers, and Launch files over isolated DDS namespaces.

Usage of versioning: Full Git history maintained demonstrating incremental feature additions (Environment Setup -> Base Nodes -> Dynamic Logic -> Launch Packaging).

Proper structure: Clean separation of concerns between environment initialization (launch), visual architecture (interactive_marker), and kinematic control (psm_interactive_grasp).
