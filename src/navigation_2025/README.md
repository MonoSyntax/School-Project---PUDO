# navigation_2025: Advanced ROS2 Nav2 Navigation Package for TEKNOFEST

This package provides a highly optimized ROS2 Nav2-based autonomous navigation stack for TEKNOFEST competition robots. It ensures high performance and safe navigation in complex environments with advanced keepout zone management, robust costmap, and controller settings.

---

## Folder and File Structure

```
navigation_2025/
├── Behavior_tree.xml         # Behavior tree definition (BT)
├── CMakeLists.txt            # ROS2 CMake configuration
├── package.xml               # ROS2 package definition
├── config/
│   ├── keepout_nav2_params.yaml   # All Nav2 parameters (keepout, costmap, controller, etc.)
│   ├── ekf.yaml                   # EKF and sensor fusion parameters
│   └── ...
├── launch/
│   ├── bringup.launch.py          # Main launch file (Simulation + Localization + Navigation)
│   ├── localization.launch.py     # Localization launch (AMCL + Map + Keepout Filters)
│   ├── navigation.launch.py       # Navigation launch (Nav2 Stack)
│   └── ...
├── maps/
│   ├── teknofestObstacle.yaml     # Map definition
│   └── keepout_teknofestObstacle.yaml # Keepout zone definition
├── src/
│   └── nav_node.py                # (If any) custom node codes
└── README.md
```

---

## Key Scripts and Tasks
- `launch/bringup.launch.py`: Launches the entire navigation system (Nav2, costmap, keepout zones, etc.) in one command.
- `config/keepout_nav2_params.yaml`: All Nav2 parameters are centrally stored here.
- `Behavior_tree.xml`: Defines task flow and recovery strategies.

---

## Important Configuration Files
- `config/keepout_nav2_params.yaml`: Costmap, controller, planner, keepout zones, and all other Nav2 parameters.
- `config/ekf.yaml`: EKF settings for sensor fusion and localization.
- `maps/teknofestObstacle.yaml`: Map definition.
- `maps/keepout_teknofestObstacle.yaml`: Keepout zone definition.
- `Behavior_tree.xml`: Navigation behavior tree and recovery logic.

---

## Quick Start
1. **Dependencies:**
   - ROS2 (Tested with Humble/Foxy)
   - Nav2 package
   - Gazebo/IGN for simulation
2. **Build:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select navigation_2025
   source install/setup.bash
   ```
3. **Launch Example:**
   ```bash
   ros2 launch navigation_2025 bringup.launch.py
   ```
   - This launches Simulation + Localization (with Keepout) + Navigation.

---

## Troubleshooting & Best Practices
- **Robot Exiting Keepout Zone:**
  - Ensure `keepout_filter` is active in both costmaps and `allow_unknown: false` in the planner.
  - Increase `inflation_radius` and reduce `footprint`/`padding` for stricter boundary adherence.
- **CMake Errors:**
  - If you encounter missing script errors, check `CMakeLists.txt` and `package.xml`.

---

## Documentation & Support
- All parameters are documented in `config/keepout_nav2_params.yaml` (with Turkish and English descriptions).
