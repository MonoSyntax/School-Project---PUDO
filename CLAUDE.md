# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS 2 Humble-based autonomous vehicle system for the Teknofest competition. The system integrates simulation (Gazebo Harmonic), navigation (Nav2), and computer vision (YOLOv12) for autonomous driving with traffic sign and light detection.

**Architecture:** Three-layer modular design with clear separation of concerns:

1. **Simulation Layer** (`simulation_2025`): Gazebo Harmonic physics and sensor simulation
2. **Navigation Layer** (`navigation_2025`): Localization (EKF+AMCL), path planning, and behavior trees
3. **Vision & Control Layer** (`otagg_vision_2026`): Object detection and safety-critical velocity override

## Critical Architectural Concepts

### Velocity Command Flow (Safety-Critical)

The system uses a **middleware override pattern** for safety:

```
Nav2 → /cmd_vel_nav → velocity_override_node → /cmd_vel → Robot
                              ↑
                        /traffic_state
```

- `velocity_override_node.py` acts as a safety gate between Nav2 and the robot
- It can **override** Nav2 commands to enforce red light stops (<5m) and stop sign stops (<3m, 3s duration)
- This pattern allows traffic rule compliance without modifying Nav2's core planning

### Traffic Detection Pipeline

```
Camera → yolov12_node.py → /traffic_signs → traffic_state_manager_node.py → /traffic_state
```

- `yolov12_node.py`: Raw YOLO detections with distance estimation from bounding box size
- `traffic_state_manager_node.py`: Filters noise (requires 3 consecutive detections), manages light states
- `velocity_override_node.py`: Consumes filtered state and applies safety logic

### Localization Fusion

The system fuses multiple sensor sources through a two-stage process:

1. **EKF** (`ekf.yaml`): Fuses IMU + Odometry → publishes `/odometry/filtered`
2. **AMCL**: Fuses laser scan + map → corrects pose drift on `/map` frame

### Keepout Zones

Both global and local costmaps use `KeepoutFilter` plugin with `keepout_teknofestObstacle.yaml`. This prevents the robot from planning paths outside lane boundaries or into grass areas, even if the global planner suggests it.

## Common Development Commands

### Build System

```bash
# Build all packages
colcon build --symlink-install

# Build specific package
colcon build --packages-select navigation_2025
colcon build --packages-select otagg_vision otagg_vision_interfaces

# Clean build (if CMake issues occur)
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Source Setup

```bash
# Required source order (critical!)
source /opt/ros/humble/setup.bash
source ~/pkgs_ws/install/setup.bash    # ros_gz bridge
source ~/ros2_ws/install/setup.bash    # this workspace
```

### Running the System

```bash
# Primary launch (brings up entire stack)
ros2 launch navigation_2025 bringup.launch.py

# Individual components (for debugging)
ros2 launch simulation_2025 teknofest_IGN.launch.py
ros2 launch navigation_2025 localization.launch.py
ros2 launch navigation_2025 navigation.launch.py
ros2 run otagg_vision yolov12_node.py
```

### Dependencies Management

```bash
# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Update rosdep database
rosdep update
```

### YOLO Model Training

```bash
cd "Training Folder"

# Fresh training
python3 train_detection.py

# Resume from checkpoint (if training interrupted)
python3 train_detection.py --resume

# Hyperparameter tuning
python3 train_detection.py --tune

# Deploy trained model
cp traffic_yolo12/<timestamp>/weights/best.pt \
   ~/ros2_ws/src/otagg_vision_2026/otagg_vision/models/yolo_traffic_best.pt
```

## Package Structure

### `simulation_2025`

- **Entry Point**: `launch/teknofest_IGN.launch.py`
- **Robot Definition**: `urdf/otagg_car.urdf.xacro` (chassis + sensors)
- **World**: `models/teknofestWORLD.sdf` (track with traffic lights and signs)
- **Bridge Config**: `config/ign_bridge.yaml` (Gazebo ↔ ROS 2 topic mappings)

Key topics bridged: `/camera`, `/scan`, `/lslidar_point_cloud`, `/imu/data_raw`, `/gps/fix`, `/cmd_vel`, `/odom`

### `navigation_2025`

- **Master Launch**: `launch/bringup.launch.py` (orchestrates all other launches with timing)
- **Nav2 Config**: `config/keepout_nav2_params.yaml` (all Nav2 parameters in one file)
  - Controller: `RegulatedPurePursuitController` (desired_linear_vel: 4.0 m/s)
  - Planner: Smac Planner
  - Costmaps: Uses `KeepoutFilter` for both global and local
- **Behavior Tree**: `Behavior_tree.xml` (defines recovery behaviors and navigation flow)
- **Custom BT Nodes**: `src/bt_custom_nodes_plugin.cpp` (DetectOscillation, Traffic detection nodes)
- **Maps**: `maps/teknofestObstacle.yaml` (main map), `maps/keepout_teknofestObstacle.yaml` (no-go zones)

### `otagg_vision_2026`

Contains two sub-packages:

#### `otagg_vision`

- **YOLO Detection**: `scripts/yolov12_node.py` (43 traffic sign classes + traffic lights)
  - Subscribes: `/camera_compressed`
  - Publishes: `/traffic/annotated/compressed`, `/traffic/alerts`, `/traffic_signs`
- **State Management**: `scripts/traffic_state_manager_node.py` (filters and tracks light states)
  - Subscribes: `/traffic_signs`
  - Publishes: `/traffic_state` (TrafficState message)
- **Safety Override**: `scripts/velocity_override_node.py` (CRITICAL NODE)
  - Subscribes: `/cmd_vel_nav`, `/traffic_state`
  - Publishes: `/cmd_vel`
  - Logic: Red light (<5m → STOP, <20m → slow), Stop sign (<3m → STOP 3s)
- **Mission Planner**: `scripts/bus_stop_loop_node.py` (sends NavigateToPose actions)
- **Models**: `models/` directory (YOLO .pt files)

#### `otagg_vision_interfaces`

Custom message definitions:

- `TrafficState.msg`: Contains traffic light state (0=Unknown, 1=Green, 2=Yellow, 3=Red), distances, stop sign detection, nav override state
- `TrafficSign.msg`: Individual detection with class, confidence, bbox, distance
- `TrafficSignArray.msg`: Array of detections from YOLO

## Key Configuration Files

### `navigation_2025/config/keepout_nav2_params.yaml`

Central Nav2 configuration. Contains all parameters for:

- AMCL (localization)
- Controller server (RPP parameters, speeds)
- Planner server (Smac planner)
- Global/Local costmaps (keepout zones, inflation)
- Behavior server (recovery behaviors)
- Velocity smoother

**Important Parameters:**

- `desired_linear_vel: 4.0` (high speed for competition)
- `keepout_filter.filter_info_topic: /costmap_filter_info` (critical for keepout zones)
- `inflation_layer.inflation_radius: 1.5` (obstacle padding)

### `navigation_2025/config/ekf.yaml`

EKF sensor fusion configuration. Defines which sensors contribute to which state estimates (position, velocity, orientation).

### `navigation_2025/Behavior_tree.xml`

Defines the navigation logic flow:

- Rate-controlled path planning (1 Hz)
- Path following with `FollowPath` controller
- Recovery behaviors on failure (backup, spin, wait)
- Traffic state integration points

## Environment Requirements

### System

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Harmonic (NOT Gazebo Classic)
- Python 3.10
- CUDA-capable NVIDIA GPU

### Critical Python Dependencies

```bash
pip install torch torchvision ultralytics opencv-python pillow
pip install 'numpy<2'  # CRITICAL: NumPy 2.x breaks cv_bridge
```

### ros_gz Bridge

**MUST** be built from source for Humble + Harmonic compatibility:

```bash
cd ~/pkgs_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b humble
cd ~/pkgs_ws
export GZ_VERSION=harmonic
rosdep install -r --from-paths src -i -y --rosdistro humble
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Common Issues & Solutions

### NumPy 2.x Incompatibility

**Symptom**: `AttributeError: _ARRAY_API not found` when running vision nodes
**Solution**:

```bash
pip uninstall numpy
pip install 'numpy<2'
```

### ros_gz Bridge Failure

**Symptom**: `[ERROR] [ros_gz_bridge]: Failed to create bridge`
**Solution**: Verify source order and `GZ_VERSION=harmonic` is set:

```bash
echo $GZ_VERSION  # Should output: harmonic
source /opt/ros/humble/setup.bash
source ~/pkgs_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Robot Not Moving / Costmap Empty

**Symptom**: Robot spawns but doesn't receive goals or costmap is empty in RViz
**Solution**: Restart simulation. Costmap loading timing issue. The `bringup.launch.py` has delays to prevent this, but occasionally fails.

### YOLO Model Not Found

**Symptom**: `FileNotFoundError: model path not found`
**Solution**: Ensure model file exists at `otagg_vision/models/best.pt` or update path in node

## Development Notes

### When Modifying Nav2 Parameters

- All Nav2 parameters are centralized in `keepout_nav2_params.yaml`
- After editing, no rebuild is needed (YAML is runtime-loaded)
- Test parameter changes: `ros2 param list` and `ros2 param get <node_name> <param_name>`

### When Adding New Traffic Sign Classes

1. Update training dataset in `Training Folder/dataset/`
2. Update `data.yaml` with new class names
3. Retrain model with `train_detection.py`
4. Update class mappings in `yolov12_node.py` if standardizing classes
5. Deploy new model to `otagg_vision/models/`

### When Modifying Behavior Tree

- Edit `Behavior_tree.xml`
- No rebuild needed (XML is runtime-loaded)
- Validate syntax before running
- Custom C++ BT nodes require rebuild of `navigation_2025` package

### When Adding Custom BT Nodes

1. Add node class to `include/navigation_2025/bt_custom_nodes.hpp`
2. Implement in `src/bt_custom_nodes_plugin.cpp`
3. Register in plugin macro at bottom of `.cpp`
4. Add to `bt_custom_nodes_plugin.xml`
5. Rebuild: `colcon build --packages-select navigation_2025`

## Testing & Debugging

### Verify System is Running

```bash
# Check active nodes
ros2 node list

# Check topics
ros2 topic list

# Monitor specific topics
ros2 topic echo /traffic_state
ros2 topic echo /cmd_vel
ros2 topic hz /camera_compressed

# Check transforms
ros2 run tf2_tools view_frames  # Generates PDF of TF tree
```

### Debug YOLO Detection

```bash
# View annotated camera output
ros2 run rqt_image_view rqt_image_view /traffic/annotated/compressed

# Monitor detection alerts
ros2 topic echo /traffic/alerts
```

### Debug Navigation

```bash
# View costmaps in RViz
rviz2

# Check Nav2 status
ros2 topic echo /plan  # Global plan
ros2 topic echo /local_plan  # Local plan

# Force goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 5.0}, orientation: {w: 1.0}}}}"
```

## Documentation References

- Main installation guide: `README.md` (Turkish, comprehensive setup)
- Project architecture: `PROJECT_REFERENCE.md` (English, system design)
- Nav2 parameters guide: `navigation_2025/TURKCE_PARAMETRE_KILAVUZU.md` (Turkish)
- ROS 2 Humble docs: <https://docs.ros.org/en/humble/>
- Nav2 docs: <https://docs.nav2.org/>
- Ultralytics YOLO: <https://docs.ultralytics.com/>
