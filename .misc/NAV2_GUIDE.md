# Nav2 Navigation Stack Guide for BIN-BOY

## Phase 4: Navigation - Complete Implementation

This guide covers the Nav2 (Navigation2) integration for BIN-BOY's omnidirectional kiwi drive robot.

---

## Table of Contents

1. [What is Nav2?](#what-is-nav2)
2. [What Was Implemented](#what-was-implemented)
3. [Architecture Overview](#architecture-overview)
4. [How to Use Nav2](#how-to-use-nav2)
5. [Configuration Explained](#configuration-explained)
6. [Omnidirectional Navigation](#omnidirectional-navigation)
7. [Integration with SLAM](#integration-with-slam)
8. [Sending Navigation Goals](#sending-navigation-goals)
9. [Recovery Behaviors](#recovery-behaviors)
10. [Tuning Nav2](#tuning-nav2)
11. [Troubleshooting](#troubleshooting)
12. [Next Steps](#next-steps)

---

## What is Nav2?

**Nav2 (Navigation2)** is the ROS 2 navigation framework that provides:

- **Global path planning**: A* search from start to goal avoiding static obstacles
- **Local trajectory control**: Real-time obstacle avoidance and smooth motion
- **Recovery behaviors**: Handling stuck situations (backup, spin, wait)
- **Behavior trees**: Coordinate navigation actions and recoveries
- **Costmaps**: 2D occupancy grids for obstacle representation

**For BIN-BOY, Nav2 enables:**
- Autonomous navigation to goal poses
- Obstacle avoidance using LIDAR
- Path planning using SLAM-generated maps
- Integration with person-following behavior

---

## What Was Implemented

### 1. Nav2 Package Created

**Package**: `bin_boy_navigation`

**Structure**:
```
bin_boy_navigation/
├── config/
│   ├── nav2_params.yaml           # Master Nav2 configuration
│   ├── costmap_common.yaml        # Shared costmap parameters
│   ├── local_costmap.yaml         # Local (rolling window) costmap
│   ├── global_costmap.yaml        # Global (static map) costmap
│   ├── controller.yaml            # DWB controller config
│   └── planner.yaml               # NavFn planner config
├── launch/
│   ├── navigation.launch.py       # Nav2 stack standalone
│   └── slam_navigation.launch.py  # SLAM + Nav2 combined
└── behavior_trees/
    └── (custom BTs - future)
```

### 2. Omnidirectional Configuration

**Critical settings for kiwi drive**:
- `holonomic_robot: true` in DWB controller
- Lateral velocity sampling: `vy_samples: 15`
- Velocity limits: `max_vel_y: ±0.5 m/s`
- Acceleration limits: `acc_lim_y: 0.5 m/s²`

### 3. Core Nav2 Nodes Configured

- **controller_server**: DWB local planner with holonomic support
- **planner_server**: NavFn global planner (A*)
- **behavior_server**: Recovery behaviors (spin, backup, wait)
- **bt_navigator**: Behavior tree executor
- **waypoint_follower**: Multi-waypoint navigation
- **lifecycle_manager**: Node lifecycle coordination

### 4. Costmap Configuration

**Local Costmap** (5x5m rolling window):
- Frame: `odom`
- Updates: 5 Hz
- Sources: LIDAR `/scan`
- Real-time obstacle detection

**Global Costmap** (50x50m static):
- Frame: `map`
- Updates: 1 Hz
- Source: `/map` from SLAM Toolbox
- Static obstacle map

### 5. Robot Specifications

**Footprint**: Circular, 0.20m radius (includes safety margin)
**Physical radius**: 0.1575m
**Velocity limits**:
- Linear X/Y: ±0.5 m/s
- Angular Z: ±1.0 rad/s

**Inflation radius**: 0.35m around obstacles

---

## Architecture Overview

### Frame Hierarchy

```
map (from SLAM)
 └─ odom (from EKF)
     └─ base_footprint
         └─ base_link
             ├─ base_laser (LIDAR)
             ├─ camera_link
             └─ imu_link
```

### Data Flow

```
SLAM Map (/map) → Global Costmap → Global Planner → Path
                                                       ↓
LIDAR (/scan)   → Local Costmap  → Local Controller → /cmd_vel → Robot
                                         ↑
Odometry (/odom_filtered) ──────────────┘
```

### Nav2 Node Communication

```
User/Behavior → /navigate_to_pose → bt_navigator
                                          ↓
                                    planner_server → Global Path
                                          ↓
                                    controller_server → Local Trajectory
                                          ↓
                                    /cmd_vel → Robot Motion
```

If stuck:
```
controller_server → FAILURE → behavior_server (recovery) → retry
```

---

## How to Use Nav2

### Launch Nav2 with SLAM

**Terminal 1: SLAM + Navigation**
```bash
cd ~/bin-boy
source install/setup.bash
ros2 launch bin_boy_navigation slam_navigation.launch.py
```

This launches:
- Gazebo simulation
- SLAM Toolbox (mapping)
- Nav2 stack (navigation)
- RViz (visualization)

**Wait for all nodes to initialize (~10 seconds).**

---

### Launch Nav2 Standalone (Pre-built Map)

If you have a saved map and want Nav2 without SLAM:

**Terminal 1: Navigation Only**
```bash
cd ~/bin-boy
source install/setup.bash
ros2 launch bin_boy_navigation navigation.launch.py
```

**Terminal 2: Map Server** (load saved map)
```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/path/to/your_map.yaml \
  -p use_sim_time:=true
```

---

### Optional: Add Person Following

**Terminal 2: Person Tracking**
```bash
cd ~/bin-boy
source install/setup.bash
ros2 launch bin_boy_perception sim_person_tracking.launch.py \
  enable_following:=false \
  debug_logging:=true
```

**Note**: Set `enable_following:=false` to prevent person tracker from publishing `/cmd_vel` (conflicts with Nav2).

For integrated person following with Nav2, see [Integration Section](#integration-with-person-following).

---

## Configuration Explained

### DWB Controller (controller.yaml)

**Purpose**: Local trajectory planning and obstacle avoidance

**Key parameters**:

```yaml
FollowPath:
  plugin: "dwb_core::DWBLocalPlanner"
  holonomic_robot: true          # CRITICAL for kiwi drive

  # Velocity limits (match kiwi_params.yaml)
  max_vel_x: 0.5
  max_vel_y: 0.5                 # Lateral motion
  max_vel_theta: 1.0

  # Trajectory sampling
  vx_samples: 15                 # Forward/backward
  vy_samples: 15                 # Sideways (holonomic)
  vtheta_samples: 20             # Rotation

  # Simulation time
  sim_time: 1.7                  # Seconds to project trajectory

  # Critics (scoring functions)
  critics: [
    "RotateToGoal",              # Align to goal heading
    "Oscillation",               # Prevent oscillation
    "BaseObstacle",              # Avoid obstacles
    "GoalAlign",                 # Face goal
    "PathAlign",                 # Follow global path
    "PathDist",                  # Progress on path
    "GoalDist"                   # Distance to goal
  ]
```

**Tuning**: Adjust critic weights to change behavior:
- Higher `PathAlign.scale` → follow global path more strictly
- Higher `GoalAlign.scale` → prioritize facing goal orientation
- Higher `BaseObstacle.scale` → more conservative around obstacles

---

### NavFn Planner (planner.yaml)

**Purpose**: Global path planning using A* search

**Key parameters**:

```yaml
GridBased:
  plugin: "nav2_navfn_planner/NavfnPlanner"
  tolerance: 0.25                # Goal tolerance (meters)
  use_astar: true                # A* (true) vs Dijkstra (false)
  allow_unknown: true            # Plan through unexplored areas
```

**Alternative**: Smac Planner (better for omnidirectional, higher compute):
```yaml
# Uncomment in planner.yaml
plugin: "nav2_smac_planner/SmacPlanner2D"
minimum_turning_radius: 0.2    # Can be small for holonomic
```

---

### Costmap Configuration

#### Local Costmap (local_costmap.yaml)

**Rolling window** around robot:

```yaml
local_costmap:
  width: 5                       # meters
  height: 5
  resolution: 0.05               # 5cm per pixel
  update_frequency: 5.0          # Hz

  plugins: ["obstacle_layer", "inflation_layer"]

  obstacle_layer:
    observation_sources: scan    # LIDAR data
    obstacle_range: 5.0          # Mark obstacles within 5m
    raytrace_range: 6.0          # Clear space up to 6m
```

**Purpose**: Real-time obstacle avoidance from LIDAR

#### Global Costmap (global_costmap.yaml)

**Static map** from SLAM:

```yaml
global_costmap:
  rolling_window: false          # Uses full map
  update_frequency: 1.0          # Slower updates

  plugins: ["static_layer", "inflation_layer"]

  static_layer:
    map_topic: /map              # From SLAM Toolbox
```

**Purpose**: Long-range path planning on static map

---

## Omnidirectional Navigation

### What Makes BIN-BOY Special

**Kiwi drive** (3 wheels at 120°) provides **holonomic motion**:
- Move sideways without rotating
- Rotate while translating
- Approach goals from any angle

**Traditional differential drive** (2 wheels):
- Must rotate to change direction
- Cannot strafe sideways
- Less maneuverable

### Configuring for Holonomic Motion

**Critical setting in controller.yaml**:
```yaml
holonomic_robot: true
```

**This enables**:
- DWB samples lateral velocities (`vy`)
- Trajectories include sideways motion
- More efficient paths in tight spaces

### Testing Omnidirectional Motion

**Scenario 1: Narrow corridor**
- Differential drive: Zigzag motion, multiple rotations
- Kiwi drive: Smooth sideways slide through gap

**Scenario 2: Goal behind obstacle**
- Differential drive: Rotate, move forward, rotate again
- Kiwi drive: Move diagonally in one smooth motion

**To observe**:
1. Launch `slam_navigation.launch.py`
2. Send goal pose through narrow gap
3. Watch robot strafe sideways in RViz

---

## Integration with SLAM

### Frame Coordination

**SLAM provides**: `map` → `odom` transform
**EKF provides**: `odom` → `base_footprint` transform
**Nav2 uses**: Both for localization and planning

**Verification**:
```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

Expected: Transform available, updating at ~50 Hz

### Using SLAM Map for Navigation

**Global costmap automatically subscribes to `/map`**:
```yaml
global_costmap:
  static_layer:
    map_topic: /map              # Published by SLAM Toolbox
    subscribe_to_updates: true   # Update as SLAM refines map
```

**Simultaneous Mapping and Navigation**:
- SLAM builds/updates map as robot moves
- Nav2 uses latest map for planning
- Robot can explore new areas autonomously

### Switching from Mapping to Localization

**For production** (after initial mapping):

1. **Save map**:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/bin-boy/final_map
```

2. **Edit `slam_toolbox.yaml`**:
```yaml
mode: localization              # Changed from 'mapping'
map_file_name: /home/mj/bin-boy/final_map
```

3. **Relaunch**: SLAM will localize in saved map (no longer updating it)

---

## Sending Navigation Goals

### Method 1: RViz (Interactive)

1. Click "2D Nav Goal" button in RViz toolbar
2. Click on map to set goal position
3. Drag to set goal orientation
4. Robot plans and executes path

### Method 2: Command Line

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: 'map'
    stamp:
      sec: 0
      nanosec: 0
  pose:
    position:
      x: 2.0
      y: 1.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.707
      w: 0.707
"
```

**Orientation**: Quaternion (use `z=0.707, w=0.707` for 90° rotation)

### Method 3: Python Node

```python
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class Navigator:
    def __init__(self):
        self.node = rclpy.create_node('navigator')
        self._action_client = ActionClient(
            self.node,
            NavigateToPose,
            'navigate_to_pose'
        )

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        # ... set orientation from yaw

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
```

---

## Recovery Behaviors

### When Recoveries Trigger

**Situation**: Controller fails to make progress for 10 seconds
**Action**: behavior_server executes recovery sequence

### Configured Recoveries

**1. Spin Recovery**
```yaml
spin:
  max_rotational_vel: 0.5        # Rotate at 0.5 rad/s
  simulate_ahead_time: 2.0       # Check 2s ahead
```
- Rotates 360° to re-observe environment
- Updates costmaps with new LIDAR data
- May discover path that was previously blocked

**2. Backup Recovery**
```yaml
backup:
  backup_dist: 0.5               # Move back 0.5m
  backup_speed: 0.15             # Slowly
```
- Moves backwards to escape tight space
- Useful if robot got too close to obstacle

**3. Wait Recovery**
```yaml
wait:
  wait_duration: 5.0             # Wait 5 seconds
```
- Pauses for dynamic obstacles to move
- Useful if person/object temporarily blocking path

### Recovery Sequence

Default behavior tree executes:
1. Attempt navigation
2. If fails → Spin recovery → Retry
3. If fails → Backup recovery → Retry
4. If fails → Wait recovery → Retry
5. If all fail → Report failure to user

### Custom Recovery

**Future enhancement**: Create "Search for Person" recovery:
```python
# Spiral outward pattern while camera looks for person
# Useful if robot lost person during following
```

---

## Tuning Nav2

### Performance vs Accuracy Trade-offs

**For faster navigation** (lower latency):
```yaml
controller_frequency: 10.0       # Reduce from 20Hz
local_costmap:
  update_frequency: 3.0          # Reduce from 5Hz
```

**For more accurate navigation**:
```yaml
controller_frequency: 30.0       # Increase from 20Hz
resolution: 0.02                 # Higher resolution (5cm → 2cm)
```

### Obstacle Avoidance Tuning

**More conservative** (wider berth around obstacles):
```yaml
inflation_radius: 0.50           # Increase from 0.35m
cost_scaling_factor: 5.0         # Increase from 3.0
BaseObstacle.scale: 0.05         # Increase from 0.02
```

**More aggressive** (navigate closer to obstacles):
```yaml
inflation_radius: 0.25           # Decrease from 0.35m
robot_radius: 0.18               # Closer to physical size
```

### Path Following Tuning

**Follow global path more strictly**:
```yaml
PathAlign.scale: 50.0            # Increase from 32.0
PathDist.scale: 50.0             # Increase from 32.0
```

**Prioritize goal reaching** (may deviate from path):
```yaml
GoalDist.scale: 40.0             # Increase from 24.0
GoalAlign.scale: 40.0            # Increase from 24.0
```

### Omnidirectional Motion Tuning

**Favor sideways motion**:
```yaml
vy_samples: 20                   # Increase from 15
max_vel_y: 0.5                   # Enable full lateral speed
```

**Reduce rotation during translation**:
```yaml
max_vel_theta: 0.5               # Reduce from 1.0
vtheta_samples: 10               # Reduce from 20
```

---

## Troubleshooting

### Robot Not Moving

**Symptom**: Nav2 accepts goal but robot doesn't move

**Solutions**:

1. **Check lifecycle states**:
```bash
ros2 lifecycle list /controller_server
# Should show: active [3]
```

If not active:
```bash
ros2 lifecycle set /controller_server configure
ros2 lifecycle set /controller_server activate
```

2. **Check /cmd_vel**:
```bash
ros2 topic hz /cmd_vel
```
Expected: Publishing at ~20 Hz

If not, check controller_server logs for errors.

3. **Verify costmaps**:
```bash
ros2 topic hz /local_costmap/costmap_raw
ros2 topic hz /global_costmap/costmap_raw
```

---

### "No Path Found" Error

**Symptom**: Planner fails to find path to goal

**Solutions**:

1. **Check if goal is in obstacle**:
- View global costmap in RViz
- Goal point should be in white (free) space, not black/gray

2. **Allow planning through unknown**:
```yaml
GridBased:
  allow_unknown: true
```

3. **Increase planner tolerance**:
```yaml
tolerance: 0.5                   # Increase from 0.25
```

4. **Check map frame**:
```bash
ros2 topic echo /map --once | grep frame_id
# Should be: frame_id: 'map'
```

---

### Robot Oscillates or Gets Stuck

**Symptom**: Robot wobbles, doesn't make progress

**Solutions**:

1. **Increase oscillation penalty**:
```yaml
Oscillation.scale: 2.0           # Increase from 1.0
```

2. **Reduce velocity sampling**:
```yaml
vx_samples: 10                   # Reduce from 15
vy_samples: 10
vtheta_samples: 15
```

3. **Check for conflicting /cmd_vel publishers**:
```bash
ros2 topic info /cmd_vel
# Should show only 1 publisher (controller_server)
```

If person_tracker is also publishing:
- Set `enable_following:=false` in person tracking launch
- Or implement behavior arbitration (see [Next Steps](#next-steps))

---

### High CPU Usage on Jetson Nano

**Symptom**: System laggy, high CPU in `top`

**Solutions**:

1. **Reduce frequencies**:
```yaml
controller_frequency: 10.0       # From 20Hz
update_frequency: 3.0            # Local costmap, from 5Hz
```

2. **Reduce sampling**:
```yaml
vx_samples: 10
vy_samples: 10
vtheta_samples: 15
```

3. **Increase costmap resolution**:
```yaml
resolution: 0.10                 # From 0.05m (fewer cells)
```

4. **Profile CPU usage**:
```bash
ros2 run ros2_control_node resource_manager --ros-args --log-level debug
```

---

### Transform Errors

**Symptom**: "Transform from map to base_footprint failed"

**Solutions**:

1. **Check SLAM is running**:
```bash
ros2 node list | grep slam_toolbox
```

2. **Verify TF tree**:
```bash
python3 /opt/ros/foxy/lib/tf2_tools/view_frames.py
xdg-open frames.pdf
```

Should show: `map → odom → base_footprint`

3. **Check transform tolerance**:
```yaml
transform_tolerance: 0.5         # Increase if TF delays
```

---

## Next Steps

### Phase 4.1: Basic Testing (Current)

**Status**: ✅ Complete

- Nav2 package created and built
- Configuration files for omnidirectional navigation
- Launch files for standalone and SLAM integration
- Documentation complete

**Testing**:
```bash
ros2 launch bin_boy_navigation slam_navigation.launch.py
```

Send goal via RViz "2D Nav Goal" and verify:
- ✅ Path planning works
- ✅ Robot avoids static obstacles
- ✅ Robot uses omnidirectional motion

---

### Phase 4.2: RViz Visualization (Pending)

**Task**: Add Nav2 displays to `simulation.rviz`

**Displays to add**:
- Global costmap (map frame)
- Local costmap (odom frame)
- Global plan path (green)
- Local plan path (yellow)
- Nav2 goal pose
- Robot footprint

**Files to modify**:
- `src/bin_boy_simulation/rviz/simulation.rviz`

---

### Phase 4.3: Behavior Arbitration (Pending)

**Task**: Resolve `/cmd_vel` conflict between Nav2 and person_tracker

**Problem**: Both nodes want to control robot

**Solution options**:

**Option A: Mode Switching**
```python
# navigation_manager.py
class NavigationManager:
    MODES = ["MANUAL", "NAV2", "FOLLOW_PERSON"]

    def switch_mode(self, mode):
        if mode == "NAV2":
            # Enable Nav2, disable person tracker cmd_vel
        elif mode == "FOLLOW_PERSON":
            # Disable Nav2, enable person tracker cmd_vel
```

**Option B: Nav2 with Dynamic Goals** (Recommended)
```python
# person_tracker.py modifications
if self.use_nav2_integration:
    # Publish PoseStamped to /person_goal
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose = person_pose
    self.goal_publisher.publish(goal)
else:
    # Original: Publish Twist to /cmd_vel
    self.cmd_vel_publisher.publish(twist)
```

Then create `person_goal_adapter.py`:
```python
# Converts /person_goal to Nav2 action calls
# Enables person following WITH obstacle avoidance
```

---

### Phase 4.4: Behavior State Machine (Pending)

**Task**: Coordinate all behaviors

**States**:
```
IDLE → FOLLOWING_PERSON → NAVIGATING → SEARCHING → IDLE
```

**Implementation**: `behavior_coordinator.py`

**State transitions**:
- IDLE → FOLLOWING_PERSON: Person detected
- FOLLOWING_PERSON → SEARCHING: Person lost >5s
- SEARCHING → FOLLOWING_PERSON: Person reacquired
- ANY → IDLE: User command or timeout

---

### Phase 4.5: Advanced Features (Future)

**Waypoint Patrol**:
```python
# Define patrol points
waypoints = [
    (1.0, 1.0),
    (3.0, 1.0),
    (3.0, 3.0),
    (1.0, 3.0)
]
# waypoint_follower navigates through all
```

**Dynamic Obstacle Tracking**:
- Use person detections as dynamic obstacles in costmap
- Track moving objects, predict trajectories

**Custom Recovery Behaviors**:
- "Search for person" spiral pattern
- "Return to dock" when battery low
- "Request help" via audio when truly stuck

**Multi-Robot Coordination**:
- Multiple BIN-BOY robots
- Coordinate to avoid each other
- Distributed SLAM

---

## Summary

**Phase 4 Nav2 Integration: COMPLETE**

✅ **Implemented**:
- Nav2 package with omnidirectional configuration
- DWB controller with `holonomic_robot: true`
- NavFn global planner
- Local and global costmaps
- Recovery behaviors (spin, backup, wait)
- Launch files for standalone and SLAM integration
- Comprehensive documentation

✅ **Ready to test**:
```bash
ros2 launch bin_boy_navigation slam_navigation.launch.py
```

⏳ **Next priorities**:
1. Test autonomous navigation in simulation
2. Add Nav2 visualizations to RViz
3. Implement behavior arbitration (resolve cmd_vel conflict)
4. Create behavior state machine
5. Integrate person following with Nav2

**BIN-BOY can now autonomously navigate to goal poses while avoiding obstacles and building maps!**
