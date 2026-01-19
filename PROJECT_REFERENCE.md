# OTAGG Autonomous Vehicle Project - Reference Documentation

## 1. Project Overview

This project is a ROS 2 Jazzy/Humble based autonomous vehicle system designed for the Teknofest competition. It features a complete simulation-to-reality pipeline using Gazebo Harmonic, Nav2 for navigation, and YOLOv12 for perception.

**System Architecture:**

1. **Simulation Layer (`simulation_2025`)**: Provides the physics environment and robot sensor data.
2. **Navigation Layer (`navigation_2025`)**: Handles localization (EKF+AMCL), path planning (Nav2), and global coordination.
3. **Vision & Control Layer (`otagg_vision_2026`)**: Performs object detection and enforces traffic rules (red lights, stop signs) by overriding navigation commands.

---

## 2. Package: `simulation_2025`

**Path:** `src/simulation_2025/`

Responsible for the Gazebo Harmonic (Ignition) simulation environment.

### Key Files

#### `launch/teknofest_IGN.launch.py`

The entry point for the simulation.

- **Sets up Resources**: Exports `GZ_SIM_RESOURCE_PATH` so Gazebo can find models.
- **Launches Gazebo**: Starts `gz_sim` with the `TeknofestWORLDV2.sdf` world.
- **Launches Bridge**: Starts `ros_gz_bridge` using `config/ign_bridge.yaml`.
- **Spawns Robot**: Calls `robot_spawner.launch.py` after 5 seconds to insert the vehicle.
- **Traffic Lights**: Launches `traffic_light.py` controller.

#### `config/ign_bridge.yaml`

Maps Gazebo topics to ROS 2 topics.

- **Sensors**:
  - `lidar/points` -> `/lslidar_point_cloud` (PointCloud2)
  - `scan` -> `/scan` (LaserScan)
  - `camera` -> `/camera` (Image)
  - `gps/fix` -> `/gps/fix`
  - `imu/data_raw` -> `/imu/data_raw`
- **Control**:
  - `/cmd_vel` (ROS) -> `cmd_vel` (Gazebo)
- **State**:
  - `odom` -> `/odom` (Odometry)

#### `urdf/otagg_car.urdf.xacro`

The robot description file.

- **Chassis**: Box shape (2.51m x 1.3m x 1.32m).
- **Drive**: Rear-wheel drive, front-wheel steering (Ackermann-like configuration visually, though Gazebo physics often use differential drive plugins for simplicity or specialized plugins).
- **Sensors**: Includes modules for `lidar`, `camera`, `gps`, `imu`.

#### `models/teknofestWORLD.sdf`

The competition track definition.

- Includes the track mesh (`teknofestharitav2.dae`).
- Places static objects: Traffic lights, stop signs, bus stops (`DurakLevha`).
- Defines global physics (gravity, magnetic field).

---

## 3. Package: `navigation_2025`

**Path:** `src/navigation_2025/`

The "Driver" of the system. Handles bringing up the entire stack and navigating the map.

### Key Files

#### `launch/bringup.launch.py` (Master Launch)

This is the **Main** file you should run.
**Execution Order:**

1. **Sim**: Launches `teknofest_IGN.launch.py`.
2. **EKF (10s delay)**: Launches `ekf_localization.launch.py` (Robot Localization).
3. **AMCL (12.5s delay)**: Launches `localization.launch.py` (Map-based localization).
4. **Nav2 (20s delay)**: Launches `navigation.launch.py`.
5. **Vision Nodes**:
    - `yolo_detector` (15s)
    - `traffic_state_manager` (16s)
    - `velocity_override` (17s)
    - `bus_stop_loop` (25s)

#### `config/keepout_nav2_params.yaml`

Critical configuration for the Navigation Stack.

- **AMCL**:
  - `scan_topic`: `/scan`
  - `robot_model_type`: `nav2_amcl::OmniMotionModel`
- **Controller (RPP)**:
  - Uses `RegulatedPurePursuitController`.
  - `desired_linear_vel`: **4.0 m/s** (High speed).
  - `lookahead_dist`: 2.5m.
- **Costmaps**:
  - **Global & Local**: Both use `KeepoutFilter`.
  - **Keepout**: Uses `keepout_teknofestObstacle.yaml` map to define absolute no-go zones (likely lane boundaries or grass).
  - **Inflation**: Radius 1.5m.

#### `src/bt_custom_nodes_plugin.cpp` & `include/.../bt_custom_nodes.hpp`

Custom C++ nodes for the Behavior Tree.

- **DetectOscillation**: Checks if the robot is wiggling without moving.
- **Traffic Logic**: `IsRedLightDetected`, `IsStopSignDetected` (These exist in C++ but the system seems to rely on the Python `velocity_override` node for the actual stopping logic currently).

---

## 4. Package: `otagg_vision_2026`

**Path:** `src/otagg_vision_2026/`

The "Eyes" and "Safety" layer.

### Key Files

#### `otagg_vision/scripts/yolov12_node.py`

- **Model**: Loads a YOLOv12 model (`best.pt`).
- **Input**: Subscribes to `/camera_compressed`.
- **Output**:
  - Publishes annotated images to `/traffic/annotated/compressed`.
  - Publishes text alerts to `/traffic/alerts`.
- **Note**: This node seems to focus on visualization and string alerts.

#### `otagg_vision/scripts/traffic_state_manager_node.py`

- **Input**: Subscribes to `/traffic_signs` (Type: `TrafficSignArray`).
- **Logic**:
  - Filters detections (requires 3 consecutive detections).
  - Tracks "Light State" (Red/Yellow/Green).
- **Output**: Publishes `/traffic_state` (Type: `TrafficState`) containing distance and light color.

#### `otagg_vision/scripts/velocity_override_node.py` **(CRITICAL SAFETY NODE)**

Acts as a middleware between Nav2 and the Motors.

- **Input**:
  - `/cmd_vel_nav`: Velocity commands from Nav2.
  - `/traffic_state`: Filtered vision data.
- **Logic**:
  - **Red Light**: If dist < 5m, **STOP** (publishes 0.0). If < 20m, Decelerate.
  - **Stop Sign**: If dist < 3m, **STOP** for 3 seconds.
  - **Normal**: Pass `/cmd_vel_nav` -> `/cmd_vel`.
- **Output**: `/cmd_vel` (The actual command sent to the robot).

#### `otagg_vision/scripts/bus_stop_loop_node.py`

The Mission Planner.

- **Logic**: Contains a list of bus stop coordinates (`BUS_STOPS`).
- **Action**: Cyclically sends `NavigateToPose` actions to Nav2.
- **Interaction**: Relies on `velocity_override_node` to handle the actual stopping at signs/lights encountered along the way.

#### `otagg_vision_interfaces/msg/TrafficState.msg`

Custom message definition.

```
uint8 traffic_light_state (0=Unknown, 1=Green, 2=Yellow, 3=Red)
float32 traffic_light_distance
bool stop_sign_detected
float32 stop_sign_distance
uint8 nav_override_state (0=None, 1=Stop)
```
