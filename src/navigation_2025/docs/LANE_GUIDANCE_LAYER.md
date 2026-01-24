# Lane Guidance Layer - Custom Costmap Plugin

## Overview

The **Lane Guidance Layer** is a custom Nav2 costmap plugin that treats lane detection point clouds as **soft guidance** rather than hard obstacles. This allows the autonomous vehicle to:

- **Prefer staying within lane boundaries** (by assigning medium cost to lane markings)
- **Cross lanes when necessary** (for turns, lane changes, or obstacle avoidance)
- **Maintain smooth navigation** without treating lanes as impassable barriers

## Architecture

### Plugin Design

The implementation follows Nav2's costmap layer architecture:

```
┌─────────────────────────────────────────────┐
│         LaneGuidanceLayer                   │
│  (inherits nav2_costmap_2d::Layer)         │
└─────────────────────────────────────────────┘
                    │
                    ├──> onInitialize()      : Setup ROS parameters, TF buffer, subscriptions
                    ├──> updateBounds()      : Calculate update region based on lane points
                    ├──> updateCosts()       : Apply costs to costmap cells
                    ├──> pointCloudCallback(): Receive and process lane detection data
                    └──> decayLanePoints()   : Remove old points based on temporal decay
```

### Data Flow

```
┌──────────────────┐
│  yolopv2_node.py │  Lane detection using YOLOPv2
└────────┬─────────┘
         │ /lane_detection/pointcloud (PointCloud2)
         ▼
┌────────────────────────────────┐
│   LaneGuidanceLayer Plugin     │
│  - Subscribe to point cloud    │
│  - Transform to global frame   │
│  - Filter by height            │
│  - Store with timestamps       │
└────────┬───────────────────────┘
         │
         ▼
┌──────────────────────────┐
│  Nav2 Costmap System     │
│  - updateBounds() called │
│  - updateCosts() called  │
│  - Apply cost value 100  │
└──────────────────────────┘
         │
         ▼
┌─────────────────────────┐
│  Path Planner           │
│  - Prefers lanes (cost  │
│    100) over free space │
│  - Can still cross      │
│    lanes if needed      │
└─────────────────────────┘
```

## Implementation Details

### Key Features

1. **Configurable Cost Value**
   - Default: 100 (medium cost)
   - Range: 0-254
   - Cost 100 encourages lane following but allows crossing
   - Lower than INSCRIBED_INFLATED_OBSTACLE (253), so obstacles still take priority

2. **Temporal Decay**
   - Stores points with timestamps
   - Automatically removes points older than `max_point_age`
   - Keeps costmap fresh with recent lane detections

3. **TF2 Transform Integration**
   - Transforms point clouds from `base_footprint` to global frame (`map` or `odom`)
   - Handles coordinate system conversions automatically
   - Respects transform tolerance settings

4. **Height Filtering**
   - Filters points by Z coordinate
   - Default range: -1.0 to 1.0 meters
   - Ensures only road-level lane markings are used

5. **Thread Safety**
   - Uses mutex locks for point storage
   - Safe concurrent access from callback and update threads

6. **Smart Cost Application**
   - Only updates costs lower than current value
   - Never overwrites actual obstacles with lane guidance
   - Uses `std::max()` to preserve higher costs

### Cost Behavior

```
Cost Values in Nav2 Costmap:
┌────────────────────────────────────────────────┐
│ 0              : FREE_SPACE (completely free)  │
│ 1-99           : Low cost (preferred paths)    │
│ 100            : LANE_GUIDANCE (our custom)    │
│ 101-252        : Medium-high costs             │
│ 253            : INSCRIBED_INFLATED_OBSTACLE   │
│ 254            : LETHAL_OBSTACLE (impassable)  │
│ 255            : NO_INFORMATION                │
└────────────────────────────────────────────────┘

Planning Behavior:
- Planner prefers costs 0-99 over 100
- Planner prefers cost 100 (lanes) over costs 101+
- Planner avoids costs 253-254 unless necessary
- Vehicle naturally follows lanes but can deviate
```

## Configuration

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enabled` | bool | `true` | Enable/disable the layer |
| `pointcloud_topic` | string | `/lane_detection/pointcloud` | Topic to subscribe for lane point clouds |
| `lane_cost` | int | `100` | Cost value to assign (0-254) |
| `min_height` | double | `-1.0` | Minimum point Z height (meters) |
| `max_height` | double | `1.0` | Maximum point Z height (meters) |
| `max_point_age` | double | `2.0` | Maximum age of points before removal (seconds) |
| `transform_tolerance` | double | `0.3` | TF transform lookup tolerance (seconds) |
| `use_temporal_decay` | bool | `true` | Enable automatic removal of old points |

### Configuration in Nav2 Params

Located in: `navigation_2025/config/keepout_nav2_params.yaml`

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "lane_guidance_layer", "inflation_layer"]

      lane_guidance_layer:
        plugin: "navigation_2025::LaneGuidanceLayer"
        enabled: True
        pointcloud_topic: "/lane_detection/pointcloud"
        lane_cost: 100
        min_height: -1.0
        max_height: 1.0
        max_point_age: 2.0
        transform_tolerance: 0.3
        use_temporal_decay: True

global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "lane_guidance_layer", "inflation_layer"]

      lane_guidance_layer:
        plugin: "navigation_2025::LaneGuidanceLayer"
        enabled: True
        pointcloud_topic: "/lane_detection/pointcloud"
        lane_cost: 100
        min_height: -1.0
        max_height: 1.0
        max_point_age: 2.0
        transform_tolerance: 0.3
        use_temporal_decay: True
```

### Plugin Order Matters

The plugin execution order is:
1. **static_layer**: Loads pre-built map
2. **obstacle_layer**: Marks dynamic obstacles (laser scans)
3. **lane_guidance_layer**: Adds lane guidance costs
4. **inflation_layer**: Inflates obstacles

This order ensures:
- Static obstacles are present first
- Dynamic obstacles (laser) are marked with lethal cost
- Lane guidance adds medium costs without overwriting obstacles
- Inflation expands safety zones around all obstacles

## Building and Installation

### 1. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select navigation_2025 --symlink-install
```

### 2. Source the Workspace

```bash
source ~/ros2_ws/install/setup.bash
```

### 3. Verify Plugin Registration

```bash
ros2 plugin list | grep LaneGuidanceLayer
```

Expected output:
```
navigation_2025::LaneGuidanceLayer (nav2_costmap_2d::Layer)
```

## Usage

### Launch the System

The plugin is automatically loaded when you launch the navigation stack:

```bash
ros2 launch navigation_2025 bringup.launch.py
```

### Monitor Lane Guidance

**Check if layer is active:**
```bash
ros2 node list | grep costmap
ros2 param list /local_costmap/local_costmap
```

**Monitor point cloud input:**
```bash
ros2 topic hz /lane_detection/pointcloud
ros2 topic echo /lane_detection/pointcloud --no-arr
```

**View costmap in RViz:**
```bash
rviz2
# Add display: By topic -> /local_costmap/costmap -> Map
# Lane guidance cells will appear with medium intensity (gray)
```

### Dynamic Parameter Tuning

Adjust cost value at runtime:
```bash
ros2 param set /local_costmap/local_costmap lane_guidance_layer.lane_cost 80
```

Adjust point decay time:
```bash
ros2 param set /local_costmap/local_costmap lane_guidance_layer.max_point_age 3.0
```

Disable/enable layer:
```bash
ros2 param set /local_costmap/local_costmap lane_guidance_layer.enabled false
```

## Tuning Guide

### Cost Value Selection

| lane_cost | Behavior |
|-----------|----------|
| **50-70** | Very weak guidance, robot may frequently deviate from lanes |
| **80-100** | **Recommended**: Balanced guidance, natural lane following with flexibility |
| **120-150** | Strong guidance, robot reluctant to cross lanes |
| **180-220** | Very strong guidance, may only cross lanes when absolutely necessary |

### Point Age Selection

| max_point_age | Use Case |
|---------------|----------|
| **0.5-1.0s** | High-speed driving, need fresh data only |
| **1.5-2.5s** | **Recommended**: Normal driving, balanced freshness |
| **3.0-5.0s** | Low-speed or sparse detection, longer memory |

## Troubleshooting

### Issue: Lane guidance not appearing in costmap

**Possible causes:**
1. Point cloud topic not publishing
   ```bash
   ros2 topic hz /lane_detection/pointcloud
   ```
2. Plugin not loaded
   ```bash
   ros2 plugin list | grep LaneGuidanceLayer
   ```
3. Layer disabled
   ```bash
   ros2 param get /local_costmap/local_costmap lane_guidance_layer.enabled
   ```

### Issue: Transform errors in logs

**Solution:** Increase transform tolerance
```bash
ros2 param set /local_costmap/local_costmap lane_guidance_layer.transform_tolerance 0.5
```

### Issue: Lane costs too weak/strong

**Solution:** Adjust lane_cost parameter
```bash
# Weaker guidance (easier to cross lanes)
ros2 param set /local_costmap/local_costmap lane_guidance_layer.lane_cost 70

# Stronger guidance (harder to cross lanes)
ros2 param set /local_costmap/local_costmap lane_guidance_layer.lane_cost 130
```

### Issue: Old lane markings persisting

**Solution:** Reduce max_point_age
```bash
ros2 param set /local_costmap/local_costmap lane_guidance_layer.max_point_age 1.0
```

## Comparison: Before vs After

### Before (Using ObstacleLayer)

```yaml
observation_sources: scan lane_cloud
lane_cloud:
  topic: /lane_detection/pointcloud
  marking: True  # Marks as LETHAL_OBSTACLE (cost 254)
  clearing: True
```

**Behavior:**
- Lane markings treated as solid obstacles
- Planner cannot cross lane lines
- May get stuck or fail to plan around curves
- Cannot perform lane changes

### After (Using LaneGuidanceLayer)

```yaml
plugins: ["static_layer", "obstacle_layer", "lane_guidance_layer", "inflation_layer"]
lane_guidance_layer:
  plugin: "navigation_2025::LaneGuidanceLayer"
  lane_cost: 100  # Medium cost, not lethal
```

**Behavior:**
- Lane markings encourage staying in lane
- Planner can cross lines when needed (turns, obstacles)
- Smooth navigation through curves
- Successful lane changes and maneuvers

## Performance Considerations

### Computational Cost

- **Very Low**: Only processes points within update bounds
- **Efficient**: Uses spatial culling and temporal decay
- **Optimized**: Direct costmap cell updates without ray tracing

### Memory Usage

- **Minimal**: Only stores point coordinates and timestamps
- **Bounded**: Automatic cleanup via temporal decay
- **Scalable**: Handles thousands of points efficiently

### Update Frequency

- Local costmap: 10 Hz (typical)
- Global costmap: 0.5 Hz (typical)
- Plugin adapts to costmap update rates

## Future Enhancements

Potential improvements:
1. **Distance-based decay**: Fade cost with distance from robot
2. **Confidence weighting**: Use detection confidence for cost scaling
3. **Lane type awareness**: Different costs for solid vs dashed lines
4. **Predictive lane projection**: Extend lanes forward based on trajectory

## References

- [Nav2 Costmap Concepts](https://docs.nav2.org/concepts/index.html)
- [Writing a Costmap Plugin](https://docs.nav2.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html)
- [Costmap Configuration](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)

## Authors

OTAGG Autonomous Vehicle Team - 2026
