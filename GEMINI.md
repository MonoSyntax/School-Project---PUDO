# OTAGG ROS 2 Workspace Context

This `GEMINI.md` provides context for the OTAGG autonomous vehicle project (Teknofest 2025/2026), built on ROS 2 Humble and Gazebo Harmonic.

## 1. Project Overview

This workspace contains the software stack for an autonomous vehicle designed for the Teknofest competition. It includes modules for simulation, navigation, and computer vision.

- **Target OS:** Ubuntu (Linux)
- **ROS Distribution:** ROS 2 Humble
- **Simulation Engine:** Gazebo Harmonic (requires custom `ros_gz` setup)

## 2. Package Architecture

### `navigation_2025`

**Purpose:** Advanced autonomous navigation using the Nav2 stack.

- **Key Features:**
  - **Keepout Zones:** Uses `KeepoutFilter` to strictly prohibit entry into specific map areas (enabled by default).
  - **Costmaps:** Optimized with large inflation radii for safety.
  - **Controller:** tuned `RegulatedPurePursuitController`.
  - **Planner:** REEDS_SHEPP motion model.
- **Key Files:**
  - Config: `config/keepout_nav2_params.yaml` (Central parameter file)
  - Launch: `launch/bringup.launch.py` (Main entry point: Sim + Loc + Nav)
  - Launch: `launch/localization.launch.py` (AMCL + Map Server + Keepout)
  - Launch: `launch/navigation.launch.py` (Nav2 Stack)
  - Map: `maps/teknofestObstacle.yaml`

### `otagg_vision_2026`

**Purpose:** Real-time object and traffic sign detection.

- **Key Features:**
  - **Models:** YOLO-based detection (YOLOv12 mentioned).
  - **Classes:** 43 traffic signs, vehicles, pedestrians.
  - **Interface:** Subscribes to `/camera_compressed` (`sensor_msgs/CompressedImage`).
- **Dependencies:** `torch`, `ultralytics`, `opencv-python`, `numpy<2` (Critical for `cv_bridge`).

### `simulation_2025`

**Purpose:** Simulation environment for the OTAGG vehicle.

- **Key Features:**
  - **Gazebo Harmonic:** Uses the latest Gazebo version (requires specific setup).
  - **Models:** Vehicle models (`otagg_car`), Teknofest world elements (`teknofest_zemin`, barriers, signs).
- **Key Files:**
  - Launch: `launch/teknofest_IGN.launch.py`, `launch/robot_spawner.launch.py`

## 3. Build & Setup Instructions

### Prerequisites

- ROS 2 Humble
- Gazebo Harmonic (`gz-harmonic`)
- Python Dependencies: `pip install torch torchvision ultralytics opencv-python "numpy<2"`
- **Critical:** `ros_gz` must be built from source for Humble + Harmonic compatibility (see `simulation_2025/README.md` for details).

### Building the Workspace

```bash
# Build all packages
cd ~/ros2_ws
colcon build

# Build specific package
colcon build --packages-select navigation_2025
```

### Sourcing

```bash
source install/setup.bash
```

## 4. Running the System

### Simulation

```bash
ros2 launch simulation_2025 teknofest_IGN.launch.py
```

### Navigation

```bash
# Full Nav2 stack with keepout zones and simulation
ros2 launch navigation_2025 bringup.launch.py
```

### Vision

```bash
# Camera Node
ros2 run otagg_vision camera_node.py

# YOLO Detection
ros2 run otagg_vision yolov12_node.py
```

## 5. Development Conventions

- **Code Style:** Follow standard ROS 2 Python/C++ guidelines.
- **Configuration:**
  - Navigation parameters are centralized in `navigation_2025/config/keepout_nav2_params.yaml`.
  - Vision models go in `otagg_vision/models`.
- **Known Issues:**
  - **NumPy:** Must be `< 2.0` to avoid `AttributeError: _ARRAY_API not found` with `cv_bridge`.
  - **Gazebo:** Requires `ros_gz` built from source in a separate workspace (e.g., `~/pkgs_ws`) to bridge Humble and Harmonic.