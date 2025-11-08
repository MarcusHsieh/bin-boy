# BIN-BOY Nav2 Navigation Optimization Plan

**Date Created**: 2025-10-28
**Status**: Navigation stack working, ready for optimization
**Robot Platform**: Kiwi drive omnidirectional robot (0.315m diameter)
**ROS Version**: ROS2 Foxy
**Hardware**: Jetson Nano 4GB

---

## Executive Summary

The Nav2 navigation stack is **fully functional** as of 2025-10-28. All 5 lifecycle nodes (controller_server, planner_server, recoveries_server, bt_navigator, waypoint_follower) successfully configure and activate.

**Recent Fix**:
- Added 5 missing BT plugins (now 22 total)
- Fixed BT XML path to absolute path
- Added bt_navigator_rclcpp_node configuration

**Current Capabilities**:
- ✅ SLAM Toolbox mapping
- ✅ Nav2 autonomous navigation
- ✅ EKF sensor fusion
- ✅ Omnidirectional motion support
- ✅ Recovery behaviors (spin, backup, wait)

**Critical Gaps**:
- ❌ No map saving capability
- ❌ No AMCL localization mode
- ❌ No waypoint scripting system

---

## Optimization Roadmap

### 🔴 Phase 1: Critical Production Features (5 hours total)

These features are **essential** for practical robot use beyond initial testing.

#### 1.1 Map Management System (2 hours)

**Current Issue**: Maps created during SLAM cannot be saved or reused. Must remap every session.

**Files to Create**:

1. **`/home/mj/bin-boy/scripts/save_map.sh`**
```bash
#!/bin/bash
# Save current SLAM map with timestamp
set -e

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
MAP_DIR=~/bin-boy/maps
MAP_NAME=${1:-map_$TIMESTAMP}

mkdir -p $MAP_DIR

echo "========================================="
echo "  Saving SLAM Map"
echo "========================================="
echo "Map name: $MAP_NAME"
echo "Location: $MAP_DIR"

# Save map using map_saver_cli
cd $MAP_DIR
ros2 run nav2_map_server map_saver_cli -f $MAP_NAME --ros-args -p use_sim_time:=true

if [ -f "$MAP_NAME.yaml" ] && [ -f "$MAP_NAME.pgm" ]; then
    echo ""
    echo "✓ Map saved successfully!"
    echo "  YAML: $MAP_DIR/$MAP_NAME.yaml"
    echo "  PGM:  $MAP_DIR/$MAP_NAME.pgm"

    # Create metadata file
    cat > "$MAP_NAME.metadata.txt" << EOF
Saved: $(date)
Resolution: $(grep resolution "$MAP_NAME.yaml" | awk '{print $2}')
Origin: $(grep origin "$MAP_NAME.yaml")
Map size: $(identify -format "%wx%h pixels" "$MAP_NAME.pgm")
EOF

    echo ""
    echo "To use this map for localization:"
    echo "  bash ~/bin-boy/scripts/load_map.sh $MAP_NAME"
else
    echo "✗ Map save failed!"
    exit 1
fi
```

2. **`/home/mj/bin-boy/scripts/load_map.sh`**
```bash
#!/bin/bash
# Load a saved map for localization
set -e

MAP_DIR=~/bin-boy/maps
MAP_NAME=${1:-current_map}
MAP_FILE="$MAP_DIR/$MAP_NAME.yaml"

if [ ! -f "$MAP_FILE" ]; then
    echo "Error: Map file not found: $MAP_FILE"
    echo ""
    echo "Available maps:"
    bash ~/bin-boy/scripts/list_maps.sh
    exit 1
fi

echo "========================================="
echo "  Loading Map for Localization"
echo "========================================="
echo "Map: $MAP_NAME"
echo "File: $MAP_FILE"
echo ""

# Update nav2_params.yaml with map filename
sed -i "s|yaml_filename:.*|yaml_filename: \"$MAP_FILE\"|" \
    ~/bin-boy/src/bin_boy_navigation/config/nav2_params.yaml

echo "✓ Map configured in nav2_params.yaml"
echo ""
echo "To launch with this map:"
echo "  bash ~/bin-boy/launch_localization.sh"
```

3. **`/home/mj/bin-boy/scripts/list_maps.sh`**
```bash
#!/bin/bash
# List all saved maps with metadata

MAP_DIR=~/bin-boy/maps

echo "========================================="
echo "  Available Maps"
echo "========================================="

if [ ! -d "$MAP_DIR" ]; then
    echo "No maps directory found."
    echo "Create maps with: bash ~/bin-boy/scripts/save_map.sh"
    exit 0
fi

cd $MAP_DIR
COUNT=0

for yaml_file in *.yaml; do
    if [ -f "$yaml_file" ]; then
        MAP_NAME="${yaml_file%.yaml}"
        PGM_FILE="$MAP_NAME.pgm"
        META_FILE="$MAP_NAME.metadata.txt"

        echo ""
        echo "Map: $MAP_NAME"

        if [ -f "$META_FILE" ]; then
            cat "$META_FILE" | sed 's/^/  /'
        else
            if [ -f "$PGM_FILE" ]; then
                echo "  Size: $(identify -format "%wx%h pixels" "$PGM_FILE" 2>/dev/null || echo "unknown")"
            fi
            echo "  Files: $yaml_file, $PGM_FILE"
        fi

        COUNT=$((COUNT + 1))
    fi
done

echo ""
echo "========================================="
echo "Total maps: $COUNT"

if [ $COUNT -eq 0 ]; then
    echo ""
    echo "No maps found. Create one with:"
    echo "  bash ~/bin-boy/scripts/save_map.sh my_map_name"
fi
```

4. **Create maps directory**:
```bash
mkdir -p /home/mj/bin-boy/maps
touch /home/mj/bin-boy/maps/.gitkeep
```

**Testing Steps**:
1. Launch SLAM navigation: `bash ~/bin-boy/launch_nav2.sh`
2. Drive robot around to build map (use teleop or Nav2 goals)
3. Save map: `bash ~/bin-boy/scripts/save_map.sh office_map`
4. Verify: `bash ~/bin-boy/scripts/list_maps.sh`
5. Expected: Map files in `~/bin-boy/maps/office_map.{yaml,pgm}`

**Success Criteria**:
- ✓ Map saves successfully with timestamp
- ✓ YAML and PGM files created
- ✓ Metadata file records map details
- ✓ List command shows all maps

---

#### 1.2 AMCL Localization Mode (3 hours)

**Current Issue**: Only SLAM mode available. Cannot use saved maps for navigation without remapping.

**Files to Create**:

1. **`/home/mj/bin-boy/src/bin_boy_navigation/config/amcl_params.yaml`**
```yaml
# AMCL (Adaptive Monte Carlo Localization) Parameters
# For omnidirectional kiwi-drive robot

amcl:
  ros__parameters:
    use_sim_time: true

    # Frame parameters
    global_frame_id: map
    odom_frame_id: odom
    base_frame_id: base_footprint

    # Transform tolerance
    transform_tolerance: 0.5

    # Robot model - CRITICAL for omnidirectional robot
    robot_model_type: "omnidirectional"

    # Particle filter parameters
    min_particles: 500
    max_particles: 2000
    recovery_alpha_slow: 0.0
    recovery_alpha_fast: 0.0

    # Update parameters
    update_min_d: 0.15        # Min translation before update (m)
    update_min_a: 0.2         # Min rotation before update (rad)
    resample_interval: 1

    # Laser model parameters
    laser_model_type: "likelihood_field"
    laser_likelihood_max_dist: 2.0
    laser_max_range: 12.0
    laser_min_range: 0.1
    max_beams: 60

    # Odometry model - omnidirectional specific
    odom_model_type: "omni"   # CRITICAL - use omni model for kiwi drive

    # Odometry noise parameters (tune based on real robot)
    odom_alpha1: 0.2          # Rotation noise from rotation
    odom_alpha2: 0.2          # Rotation noise from translation
    odom_alpha3: 0.2          # Translation noise from translation
    odom_alpha4: 0.2          # Translation noise from rotation
    odom_alpha5: 0.2          # Strafe noise (omnidirectional)

    # Initial pose parameters
    set_initial_pose: false
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0

    # Covariance
    initial_cov_xx: 0.25
    initial_cov_yy: 0.25
    initial_cov_aa: 0.068

    # Laser z-parameters
    z_hit: 0.5
    z_short: 0.05
    z_max: 0.05
    z_rand: 0.5

    # Sensor parameters
    sigma_hit: 0.2
    lambda_short: 0.1

    # Additional parameters
    tf_broadcast: true
    save_pose_rate: 0.5

    # Scan topic
    scan_topic: scan

amcl_map_client:
  ros__parameters:
    use_sim_time: true

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: true
```

2. **`/home/mj/bin-boy/src/bin_boy_navigation/launch/localization_navigation.launch.py`**
```python
#!/usr/bin/env python3
"""
Localization + Navigation Launch File
Launches Nav2 stack with AMCL localization (uses pre-built map)

Use this mode when you have a saved map and want to navigate without SLAM.
For mapping, use slam_navigation.launch.py instead.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_simulation = get_package_share_directory('bin_boy_simulation')
    pkg_navigation = get_package_share_directory('bin_boy_navigation')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_rviz = LaunchConfiguration('rviz', default='true')
    map_file = LaunchConfiguration('map', default='')

    # Config files
    nav2_params = os.path.join(pkg_navigation, 'config', 'nav2_params.yaml')
    amcl_params = os.path.join(pkg_navigation, 'config', 'amcl_params.yaml')
    rviz_config = os.path.join(pkg_simulation, 'rviz', 'simulation.rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_enable_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )

    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map YAML file (if empty, uses nav2_params.yaml default)'
    )

    # Set environment variables
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Include Gazebo simulation (without SLAM)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_simulation, 'launch', 'gazebo_sim.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': 'false'  # We'll launch RViz separately
        }.items()
    )

    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_file}  # Override if provided
        ]
    )

    # AMCL Localization
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params]
    )

    # Lifecycle Manager for map_server and AMCL
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    # Include Navigation stack
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_navigation, 'launch', 'navigation.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # RViz
    rviz = Node(
        condition=IfCondition(enable_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Delayed start for map_server and AMCL
    delayed_localization = TimerAction(
        period=3.0,
        actions=[lifecycle_manager_localization]
    )

    return LaunchDescription([
        stdout_linebuf_envvar,
        declare_use_sim_time,
        declare_enable_rviz,
        declare_map_file,
        gazebo_launch,
        map_server,
        amcl,
        delayed_localization,
        navigation_launch,
        rviz
    ])
```

3. **`/home/mj/bin-boy/launch_localization.sh`** (convenience script)
```bash
#!/bin/bash
# Launch navigation with AMCL localization (uses saved map)
# Usage: bash launch_localization.sh [map_name]

set -e

MAP_NAME=${1:-current_map}
MAP_DIR=~/bin-boy/maps
MAP_FILE="$MAP_DIR/$MAP_NAME.yaml"

echo "========================================="
echo "  BIN-BOY Localization + Navigation"
echo "========================================="

# Check if map exists
if [ ! -f "$MAP_FILE" ]; then
    echo "Error: Map not found: $MAP_FILE"
    echo ""
    echo "Available maps:"
    bash ~/bin-boy/scripts/list_maps.sh
    exit 1
fi

echo ""
echo "Map: $MAP_NAME"
echo "File: $MAP_FILE"
echo ""
echo "Launching:"
echo "  - Gazebo simulation"
echo "  - Map server (with saved map)"
echo "  - AMCL localization"
echo "  - Nav2 stack"
echo "  - RViz visualization"
echo ""
echo "After launch:"
echo "  1. Set initial pose in RViz (2D Pose Estimate)"
echo "  2. Send navigation goals (2D Nav Goal)"
echo ""
echo "========================================="

cd ~/bin-boy
source install/setup.bash

ros2 launch bin_boy_navigation localization_navigation.launch.py \
    use_sim_time:=true \
    map:=$MAP_FILE
```

**File Updates Required**:

4. **Update `/home/mj/bin-boy/src/bin_boy_navigation/CMakeLists.txt`** (add after line 40):
```cmake
# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
```

5. **Update `/home/mj/bin-boy/src/bin_boy_navigation/config/nav2_params.yaml`** (line 256):
```yaml
# Map Server (for loading pre-built maps)
map_server:
  ros__parameters:
    use_sim_time: true
    yaml_filename: ""  # Will be populated by load_map.sh or launch argument
    topic_name: "map"
    frame_id: "map"
```

**Testing Steps**:
1. Save a map: `bash ~/bin-boy/scripts/save_map.sh test_map`
2. Stop SLAM navigation: `bash ~/bin-boy/stop_all.sh`
3. Launch localization mode: `bash ~/bin-boy/launch_localization.sh test_map`
4. In RViz, set initial pose with "2D Pose Estimate" tool
5. Watch particles converge
6. Send navigation goal with "2D Nav Goal"

**Success Criteria**:
- ✓ Map server loads saved map
- ✓ AMCL publishes particles in RViz
- ✓ Particles converge after setting initial pose
- ✓ Robot localizes and navigates using saved map

---

#### 1.3 Waypoint Navigation System (2 hours)

**Current Issue**: Waypoint follower node is launched but no interface to send waypoint sequences.

**Files to Create**:

1. **`/home/mj/bin-boy/src/bin_boy_navigation/bin_boy_navigation/waypoint_sender.py`**
```python
#!/usr/bin/env python3
"""
Waypoint Sender Node
Sends sequences of waypoints to Nav2 waypoint follower.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
import yaml


class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')

        self.get_logger().info('Waypoint Sender Node started')

        # Action client for waypoint follower
        self._action_client = ActionClient(
            self,
            FollowWaypoints,
            'follow_waypoints'
        )

    def send_waypoints_from_yaml(self, yaml_file):
        """Load waypoints from YAML and send to follower"""

        self.get_logger().info(f'Loading waypoints from: {yaml_file}')

        try:
            with open(yaml_file, 'r') as f:
                config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return False

        waypoints = config.get('waypoints', [])

        if not waypoints:
            self.get_logger().error('No waypoints found in file')
            return False

        self.get_logger().info(f'Loaded {len(waypoints)} waypoints')

        # Convert to PoseStamped messages
        poses = []
        for i, wp in enumerate(waypoints):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = float(wp['x'])
            pose.pose.position.y = float(wp['y'])
            pose.pose.position.z = 0.0

            # Convert yaw to quaternion
            yaw = float(wp.get('yaw', 0.0))
            pose.pose.orientation.z = np.sin(yaw / 2.0)
            pose.pose.orientation.w = np.cos(yaw / 2.0)

            poses.append(pose)
            self.get_logger().info(
                f'  Waypoint {i+1}: x={wp["x"]:.2f}, y={wp["y"]:.2f}, yaw={yaw:.2f}'
            )

        # Wait for action server
        self.get_logger().info('Waiting for waypoint follower action server...')
        self._action_client.wait_for_server()

        # Create goal
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.get_logger().info('Sending waypoints to follower...')

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Waypoint goal rejected')
            return

        self.get_logger().info('Waypoint goal accepted, executing...')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Current waypoint: {feedback.current_waypoint + 1}'
        )

    def get_result_callback(self, future):
        result = future.result().result

        if len(result.missed_waypoints) == 0:
            self.get_logger().info('✓ All waypoints reached successfully!')
        else:
            self.get_logger().warn(
                f'Completed with {len(result.missed_waypoints)} missed waypoints'
            )

        rclpy.shutdown()


def main(args=None):
    import sys
    import numpy as np

    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print('Usage: ros2 run bin_boy_navigation waypoint_sender <waypoints.yaml>')
        sys.exit(1)

    waypoint_file = sys.argv[1]

    sender = WaypointSender()
    sender.send_waypoints_from_yaml(waypoint_file)

    rclpy.spin(sender)


if __name__ == '__main__':
    main()
```

2. **`/home/mj/bin-boy/src/bin_boy_navigation/config/waypoints/patrol_square.yaml`** (example)
```yaml
# Example patrol route: Square pattern
# Units: meters for x/y, radians for yaw

waypoints:
  - x: 0.0
    y: 0.0
    yaw: 0.0

  - x: 2.0
    y: 0.0
    yaw: 1.57  # 90 degrees

  - x: 2.0
    y: 2.0
    yaw: 3.14  # 180 degrees

  - x: 0.0
    y: 2.0
    yaw: -1.57  # -90 degrees

  - x: 0.0
    y: 0.0
    yaw: 0.0  # Back to start
```

3. **Update `/home/mj/bin-boy/src/bin_boy_navigation/setup.py`** (around line 20):
```python
entry_points={
    'console_scripts': [
        'waypoint_sender = bin_boy_navigation.waypoint_sender:main',
    ],
},
```

**Testing Steps**:
1. Rebuild: `cd ~/bin-boy && colcon build --packages-select bin_boy_navigation`
2. Source: `source install/setup.bash`
3. Launch navigation: `bash ~/bin-boy/launch_nav2.sh`
4. Send waypoints: `ros2 run bin_boy_navigation waypoint_sender src/bin_boy_navigation/config/waypoints/patrol_square.yaml`
5. Watch robot follow waypoint sequence

**Success Criteria**:
- ✓ Node loads waypoints from YAML
- ✓ Sends goal to waypoint follower
- ✓ Robot visits each waypoint in sequence
- ✓ Reports completion status

---

### 🟡 Phase 2: Parameter Optimization (1-2 hours)

Quick performance improvements through parameter tuning.

#### 2.1 Velocity Limit Optimization

**Current State** (`src/bin_boy_navigation/config/controller.yaml`):
```yaml
max_vel_x: 0.5
max_vel_y: 0.5
max_vel_theta: 1.0
```

**Recommended Changes**:
```yaml
max_vel_x: 0.6    # +20% speed (test incrementally up to 0.8)
max_vel_y: 0.6    # +20% speed
max_vel_theta: 1.0  # Keep rotation speed same
```

**Testing Protocol**:
1. Increase to 0.6 m/s, test navigation
2. If stable, increase to 0.7 m/s
3. If stable, increase to 0.8 m/s
4. Find maximum stable velocity for your robot

**Expected Improvement**: 20-60% faster navigation

---

#### 2.2 Inflation Radius Optimization

**Current State** (`src/bin_boy_navigation/config/costmap_common.yaml` line 39):
```yaml
inflation_radius: 0.35
```

**Analysis**:
- Robot physical radius: 0.1575m
- Current total clearance: 0.1575 + 0.35 = 0.5275m (3.35x robot size)
- This is very conservative

**Recommended Change**:
```yaml
inflation_radius: 0.30  # Reduces total clearance to 0.4575m (2.9x robot size)
```

**Expected Improvement**:
- Tighter navigation around obstacles
- Ability to navigate narrower passages
- More direct paths

---

#### 2.3 Goal Tolerance Tightening

**Current State** (`src/bin_boy_navigation/config/controller.yaml` lines 56-58):
```yaml
general_goal_checker:
  xy_goal_tolerance: 0.15  # 15cm
  yaw_goal_tolerance: 0.2  # ~11 degrees
```

**Recommended Changes**:
```yaml
general_goal_checker:
  xy_goal_tolerance: 0.10  # 10cm - tighter positioning
  yaw_goal_tolerance: 0.15  # ~8.6 degrees - more precise orientation
```

**Expected Improvement**: More precise goal achievement

---

#### 2.4 Recovery Behavior Speed Increase

**Current State** (`src/bin_boy_navigation/config/nav2_params.yaml` line 147):
```yaml
spin:
  max_rotational_vel: 0.5  # Conservative
```

**Recommended Change**:
```yaml
spin:
  max_rotational_vel: 0.8  # 60% faster recovery
```

**Expected Improvement**: Faster unstuck recovery

---

#### 2.5 Trajectory Sampling Optimization

**Current State** (`src/bin_boy_navigation/config/controller.yaml`):
```yaml
vy_samples: 15      # Lateral velocity samples
sim_time: 1.7       # Trajectory simulation time
```

**Recommended Changes**:
```yaml
vy_samples: 20      # Better lateral motion quality (+33% samples)
sim_time: 2.0       # Longer lookahead for safer navigation
```

**Trade-off**: +15% CPU usage for better path quality

---

#### 2.6 Fix Costmap Inconsistencies

**Issue**: Global and local costmaps have different inflation settings.

**File**: `src/bin_boy_navigation/config/nav2_params.yaml`

**Local Costmap** (lines 208-214):
```yaml
inflate_unknown: false
inflate_around_unknown: false
```

**Global Costmap** (lines 242-249):
```yaml
inflate_unknown: false
inflate_around_unknown: true  # DIFFERENT!
```

**Fix**: Standardize both to:
```yaml
inflate_unknown: false
inflate_around_unknown: true
```

---

#### 2.7 Update Map Server Configuration

**File**: `src/bin_boy_navigation/config/nav2_params.yaml` (line 252)

**Current**:
```yaml
map_server:
  ros__parameters:
    use_sim_time: true
    yaml_filename: ""  # Empty
```

**Update to**:
```yaml
map_server:
  ros__parameters:
    use_sim_time: true
    yaml_filename: ""  # Populated by load_map.sh or launch arg
    topic_name: "map"
    frame_id: "map"
```

---

### 🟢 Phase 3: Documentation and Testing Tools (2-3 hours)

#### 3.1 Map Management Documentation

**File**: `/home/mj/bin-boy/docs/MAP_MANAGEMENT.md`

**Contents**:
- How to save maps
- How to load maps for localization
- Map quality assessment
- When to remap vs localize
- Troubleshooting map issues

---

#### 3.2 Navigation Benchmark Script

**File**: `/home/mj/bin-boy/scripts/nav_benchmark.sh`

**Purpose**: Test navigation with predefined goals, measure:
- Success rate
- Completion time
- Path length
- Number of recoveries
- CPU usage

---

#### 3.3 Parameter Tuning Guide

**File**: `/home/mj/bin-boy/docs/PARAMETER_TUNING.md`

**Contents**:
- Step-by-step tuning process
- What to adjust for different scenarios
- Performance trade-offs
- Expected behavior changes

---

## Quick Reference: File Locations

### Configuration Files
```
src/bin_boy_navigation/config/
├── nav2_params.yaml          # Main Nav2 config (modified)
├── amcl_params.yaml          # NEW - AMCL localization
├── controller.yaml           # DWB controller params
├── planner.yaml              # NavFn planner params
├── costmap_common.yaml       # Shared costmap params
├── global_costmap.yaml       # Global costmap specific
├── local_costmap.yaml        # Local costmap specific
└── waypoints/                # NEW - Waypoint route files
    └── patrol_square.yaml    # NEW - Example patrol
```

### Launch Files
```
src/bin_boy_navigation/launch/
├── navigation.launch.py              # Nav2 nodes only
├── slam_navigation.launch.py         # EXISTING - SLAM mode
└── localization_navigation.launch.py # NEW - Localization mode
```

### Scripts
```
scripts/
├── save_map.sh           # NEW - Save SLAM maps
├── load_map.sh           # NEW - Load maps for localization
├── list_maps.sh          # NEW - List available maps
└── nav_benchmark.sh      # NEW - Test navigation performance
```

### Top-Level Scripts
```
launch_nav2.sh            # EXISTING - Launch SLAM navigation
launch_localization.sh    # NEW - Launch localization mode
stop_all.sh               # EXISTING - Stop all processes
check_nav2_status.sh      # EXISTING - Check node states
activate_nav2.sh          # EXISTING - Manual activation
```

### Maps Directory
```
maps/
├── current_map.yaml      # Latest saved map
├── current_map.pgm
├── office_map.yaml       # Example saved map
├── office_map.pgm
└── .gitkeep
```

---

## Implementation Checklist

### Phase 1: Critical Features

- [ ] **Map Management System**
  - [ ] Create `scripts/save_map.sh`
  - [ ] Create `scripts/load_map.sh`
  - [ ] Create `scripts/list_maps.sh`
  - [ ] Create `maps/` directory
  - [ ] Make scripts executable: `chmod +x scripts/*.sh`
  - [ ] Test map saving workflow
  - [ ] Test map listing

- [ ] **AMCL Localization Mode**
  - [ ] Create `config/amcl_params.yaml`
  - [ ] Create `launch/localization_navigation.launch.py`
  - [ ] Create `launch_localization.sh`
  - [ ] Update `CMakeLists.txt` (install config)
  - [ ] Update `nav2_params.yaml` (map_server section)
  - [ ] Rebuild package: `colcon build --packages-select bin_boy_navigation`
  - [ ] Test localization workflow
  - [ ] Verify particle filter convergence
  - [ ] Test navigation with saved map

- [ ] **Waypoint Navigation System**
  - [ ] Create `bin_boy_navigation/waypoint_sender.py`
  - [ ] Create `config/waypoints/` directory
  - [ ] Create `config/waypoints/patrol_square.yaml` (example)
  - [ ] Update `setup.py` (entry point)
  - [ ] Rebuild package
  - [ ] Test waypoint sending
  - [ ] Create additional patrol routes

### Phase 2: Parameter Optimization

- [ ] **Velocity Optimization**
  - [ ] Edit `config/controller.yaml`
  - [ ] Change `max_vel_x` from 0.5 to 0.6
  - [ ] Change `max_vel_y` from 0.5 to 0.6
  - [ ] Rebuild and test
  - [ ] If stable, increase to 0.7, then 0.8

- [ ] **Inflation Optimization**
  - [ ] Edit `config/costmap_common.yaml`
  - [ ] Change `inflation_radius` from 0.35 to 0.30
  - [ ] Rebuild and test
  - [ ] Verify no collision issues

- [ ] **Goal Tolerance**
  - [ ] Edit `config/controller.yaml`
  - [ ] Change `xy_goal_tolerance` from 0.15 to 0.10
  - [ ] Change `yaw_goal_tolerance` from 0.2 to 0.15
  - [ ] Test goal achievement precision

- [ ] **Recovery Speed**
  - [ ] Edit `config/nav2_params.yaml`
  - [ ] Change spin `max_rotational_vel` from 0.5 to 0.8
  - [ ] Test recovery behaviors

- [ ] **Trajectory Sampling**
  - [ ] Edit `config/controller.yaml`
  - [ ] Change `vy_samples` from 15 to 20
  - [ ] Change `sim_time` from 1.7 to 2.0
  - [ ] Monitor CPU usage

- [ ] **Costmap Consistency**
  - [ ] Edit `config/nav2_params.yaml`
  - [ ] Standardize `inflate_around_unknown: true` for both costmaps
  - [ ] Verify consistent behavior

- [ ] **Map Server Config**
  - [ ] Edit `config/nav2_params.yaml` line 252-256
  - [ ] Add `topic_name` and `frame_id` parameters

### Phase 3: Documentation

- [ ] Create `docs/MAP_MANAGEMENT.md`
- [ ] Create `docs/PARAMETER_TUNING.md`
- [ ] Update `NAV2_GUIDE.md` with new features
- [ ] Create navigation benchmark script

---

## Testing Procedures

### Complete SLAM → Save → Localize Workflow

1. **Start SLAM Session**:
   ```bash
   cd ~/bin-boy && source install/setup.bash
   bash launch_nav2.sh
   ```

2. **Build Map**:
   - Use teleop or 2D Nav Goals to drive robot
   - Ensure good coverage of environment
   - Monitor map quality in RViz

3. **Save Map**:
   ```bash
   # In new terminal
   cd ~/bin-boy && source install/setup.bash
   bash scripts/save_map.sh office_map
   ```

4. **Stop SLAM**:
   ```bash
   bash stop_all.sh
   ```

5. **Launch Localization**:
   ```bash
   bash launch_localization.sh office_map
   ```

6. **Set Initial Pose**:
   - In RViz, click "2D Pose Estimate"
   - Click approximate robot location
   - Drag to set orientation
   - Watch particles converge

7. **Test Navigation**:
   - Click "2D Nav Goal"
   - Click destination
   - Verify navigation works

8. **Test Waypoints**:
   ```bash
   ros2 run bin_boy_navigation waypoint_sender \
     src/bin_boy_navigation/config/waypoints/patrol_square.yaml
   ```

---

## Performance Expectations

### Current Performance (Conservative Settings)
- Max speed: 0.5 m/s
- Goal tolerance: 15cm position, 11° orientation
- Inflation clearance: 52.75cm total
- Recovery spin: 0.5 rad/s

### Optimized Performance (After Tuning)
- Max speed: 0.6-0.8 m/s (20-60% faster)
- Goal tolerance: 10cm position, 8.6° orientation (33% tighter)
- Inflation clearance: 45.75cm total (13% reduction = tighter paths)
- Recovery spin: 0.8 rad/s (60% faster)

**Overall Expected Improvement**: 20-30% faster navigation with tighter, more precise paths.

---

## Troubleshooting Guide

### Map Saving Issues

**Problem**: Map save fails
**Solutions**:
- Verify SLAM is running: `ros2 topic echo /map --once`
- Check map_saver_cli is available: `ros2 run nav2_map_server map_saver_cli --help`
- Verify write permissions: `ls -ld ~/bin-boy/maps`

**Problem**: Map is blank or incomplete
**Solutions**:
- Drive robot more to build better map
- Check SLAM parameters in slam_params.yaml
- Verify scan data: `ros2 topic echo /scan`

### Localization Issues

**Problem**: AMCL particles don't converge
**Solutions**:
- Set better initial pose estimate
- Increase max_particles to 3000
- Reduce min_particles to 200
- Check odometry: `ros2 topic echo /odom_filtered`

**Problem**: Robot localizes incorrectly
**Solutions**:
- Verify map matches environment
- Check laser scan alignment
- Adjust odom noise parameters (odom_alpha1-5)

### Waypoint Issues

**Problem**: Waypoint follower doesn't accept goal
**Solutions**:
- Verify waypoint_follower is active: `bash check_nav2_status.sh`
- Check action server: `ros2 action list`
- Verify YAML syntax is correct

**Problem**: Robot misses waypoints
**Solutions**:
- Increase goal tolerance
- Add intermediate waypoints
- Check for obstacles blocking path

---

## Advanced Topics (Future Work)

### Custom Behavior Trees
- Modify BT XML for custom navigation logic
- Add person-following behaviors
- Implement conditional navigation

### Multi-Robot Coordination
- Namespace separation
- Shared map management
- Conflict resolution

### Real Hardware Deployment
- Tune odom noise parameters for real robot
- Adjust sensor parameters (laser, IMU)
- Add safety monitors
- Implement emergency stop

### Dynamic Obstacle Handling
- Enable costmap obstacle tracking
- Adjust update rates for dynamic environments
- Add prediction for moving obstacles

---

## Summary

This optimization plan provides a comprehensive roadmap for transforming your currently functional Nav2 system into a production-ready autonomous navigation platform.

**Estimated Total Time**:
- Phase 1 (Critical): 5 hours
- Phase 2 (Optimization): 2 hours
- Phase 3 (Documentation): 2 hours
- **Total: 9 hours**

**Priority Order**:
1. Map saving/loading (enables reuse of mapping work)
2. AMCL localization (enables map-based navigation)
3. Parameter optimization (immediate performance gains)
4. Waypoint system (enables autonomous patrol)
5. Documentation (ease of maintenance)

**Expected Outcomes**:
- ✓ Ability to save and reuse maps
- ✓ Switch between SLAM and localization modes
- ✓ 20-30% improvement in navigation speed
- ✓ Tighter, more precise navigation
- ✓ Autonomous waypoint following
- ✓ Production-ready navigation system

Good luck with the optimizations, sir!
