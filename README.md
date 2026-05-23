# dVRK Visual Servoing & Dynamic Tissue Excision Simulation

* **Author:** Gergely Dobovics
* **Context:** IT Engineering Faculty (Artificial Intelligence Specialization), Óbuda University (OE)
* **Project:** Mid-Term Robotics Assignment

## 📌 Project Overview
This project implements a dynamic, closed-loop kinematic control system for the simulated Patient Side Manipulator (PSM1) of the da Vinci Research Kit (dVRK). Utilizing ROS 2, the system introduces an interactive 6-DOF visualization marker that acts as a "tissue" target. The robot is programmed to autonomously track this user-manipulated target, perform a circumferential tissue excision, and extract the target by dynamically updating its Cartesian pose and jaw state.

## ⚙️ System Requirements & Dependencies
* **OS:** Ubuntu 22.04 or newer (WSL2 supported, see clock synchronization notes in code)
* **Middleware:** ROS 2 (Humble)
* **Robot Framework:** [dVRK 2.4.0 ROS 2 packages](https://dvrk.readthedocs.io/main/pages/software/compilation/ros2.html) (`dvrk_robot`, `dvrk_model`, `dvrk_config`)
* **Python Dependencies:**
   * `numpy`: For vector mathematics and positional offsets.
   * `scipy`: For Spherical Linear Interpolation (SLERP) of quaternions via `scipy.spatial.transform`.

## 🏗️ Architecture & ROS 2 Communication
The system is decoupled into two primary nodes communicating over the ROS 2 DDS network to ensure real-time responsiveness and scalability.

### 1. **Interactive Marker Server** (`interactive_marker.py`)
This node bridges human interaction in RViz with the robot’s spatial awareness. It generates a 6-DOF interactive target within RViz2, acting as the simulated "tissue" for the PSM.
* **ROS Concept:** Implements an `InteractiveMarkerServer`.
* **Topics Published:** `/dvrk_viz/interactive_target_pose` (Streams real-time coordinates to the robotic arm).
* **Topics Subscribed:** `/dvrk_viz/force_marker_pose` (Allows the robot to "drag" the marker during extraction).
* **Logic:** Uses `scipy.spatial.transform.Rotation` to prevent gimbal lock when initializing orientations.

### 2. **PSM Dynamic Controller** (`psm_interactive_grasp.py`)
The algorithmic "brain" of the project, driving the PSM1 hardware/simulation through a complex, multi-stage surgical procedure.
* **Topics Subscribed:** 
    * `/PSM1/measured_cp` (Live end-effector Cartesian pose)
    * `/PSM1/jaw/measured_js` (Live jaw angle)
    * `/dvrk_viz/interactive_target_pose` (Live target coordinates)
* **Topics Published:** `/PSM1/servo_cp` and `/PSM1/jaw/servo_jp`.
* **Logic:** Calculates the 3D vector difference between the robot's current position and the marker's live position at every tick to allow for "chasing" moving targets.

## 🧠 Algorithmic Concepts & Engineering

### Advanced Kinematic Control
* **Quaternion SLERP:** To ensure smooth rotation interpolation and prevent robotic "gimbal lock," the node uses scipy.spatial.transform.Slerp to transition the end-effector's angle proportionate to its speed.  
* **WSL2 Stability:** time.monotonic() is utilized for sleep and delta-time calculations to prevent the node from hanging if the ROS 2 simulation clock stutters in virtualized environments.
* **Data Synchronization:** Execution is paused until all telemetry streams are verified to prevent "blind" movements.

### The Surgical Excision State Machine
The execute_surgical_cutting_demo function orchestrates a five-stage procedure:  
1. **Approach:** Moves above the geometric center of the interactive marker.
2. **Circumferential Cut:** Dynamically calculates points along the edge of the cylinder using trigonometric offsets:
   * $x = r \cdot \cos(\theta)$
   * $y = r \cdot \sin(\theta)$
3. **Center & Plunge:** Positions the end-effector at the center and dives downward.
4. **Secure Grasp:** Clamps the jaw shut over the simulated tissue.
5. **Extraction:** Moves to a safe coordinate while simultaneously publishing to the forced marker topic to visually drag the cylinder out of the field.

## 🚀 Installation & Usage
After cloning the repository into your ROS 2 workspace, you have two options to get the system up and running:
### **Option A (Automated):** 
Use the provided bash script to build, launch the dVRK system, and open RViz then follow instructions.
```bash
./ros2_ws/src/ros2_course/start_dvrk.sh
```

### **Option B (Standard):**
**1. Build the package:**
Ensure you are in the root of your ROS 2 workspace (e.g., ~/ros2_ws).
```bash
cd ~/ros2_ws
colcon build --packages-select ros2_course --symlink-install
source install/setup.bash
```

**2. Launch the Environment:**
```bash
ros2 launch ros2_course dvrk_env.launch.py
```
⚠️ **Important:** Once RViz loads, you **MUST** use the GUI buttons to **HOME** the PSM1 robot before proceeding.
Then in Terminal 2, launch the interactive marker and the autonomous control sequence.
```bash
source install/setup.bash
ros2 launch ros2_course dvrk_tasks.launch.py
```

## ✅ Grading Rubric Fulfillment

- [x] **Completeness of the solution:** Fully automated routine handling edge-case synchronizations and continuous telemetry.

- [x] **Proper ROS communication:** Utilizes Custom Publishers/Subscribers, Timers, and Launch files over isolated DDS namespaces.

- [x] **Usage of versioning:** Full Git history maintained demonstrating incremental feature additions.

- [x] **Proper structure:** Clean separation between environment initialization and kinematic control logic.
