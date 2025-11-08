# BIN-BOY Nav2 Navigation - Current Status

**Last Updated**: 2025-10-28
**Status**: ✅ FULLY OPERATIONAL

---

## System Overview

**Robot Platform**: Custom kiwi-drive omnidirectional robot
- Physical diameter: 315mm (0.1575m radius)
- Drive type: 3-wheel omnidirectional (kiwi configuration)
- Sensors: Laser scanner, IMU (MPU6050), Camera
- Hardware: Jetson Nano 4GB
- ROS Version: ROS2 Foxy

---

## Current Capabilities

### ✅ Working Features

1. **SLAM Mapping**
   - SLAM Toolbox integration
   - Real-time map building
   - EKF sensor fusion (IMU + odometry)

2. **Autonomous Navigation**
   - All 5 Nav2 lifecycle nodes ACTIVE
   - Path planning (NavFn A*)
   - Local obstacle avoidance (DWB controller)
   - Recovery behaviors (spin, backup, wait)
   - Holonomic motion support

3. **Visualization**
   - RViz2 integration
   - Real-time costmap display
   - Path visualization

4. **Control Scripts**
   - `launch_nav2.sh` - Launch SLAM + Nav2
   - `stop_all.sh` - Stop all processes
   - `check_nav2_status.sh` - Verify node states
   - `activate_nav2.sh` - Manual node activation

---

## Recent Fixes (2025-10-28)

### bt_navigator Plugin Loading Issue - RESOLVED ✅

**Problem**: bt_navigator failed to configure with error:
```
[ERROR] Could not load library: libnav2_compute_path_through_poses_action_bt_node.so
```

**Root Cause**:
- Missing 5 BT plugins in configuration (had 17, needed 22)
- Relative path for BT XML file (needed absolute path)
- Missing `bt_navigator_rclcpp_node` configuration section

**Solution Applied**:
1. Enhanced `src/bin_boy_navigation/config/nav2_params.yaml`:
   - Added 5 missing plugins:
     - `nav2_goal_updated_condition_bt_node`
     - `nav2_speed_controller_bt_node`
     - `nav2_truncate_path_action_bt_node`
     - `nav2_time_expired_condition_bt_node`
     - `nav2_distance_traveled_condition_bt_node`

   2. Fixed BT XML path:
   - Before: `default_bt_xml_filename: navigate_w_replanning_and_recovery.xml`
   - After: `default_bt_xml_filename: /opt/ros/foxy/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml`

   3. Added Groot monitoring parameters:
   ```yaml
   enable_groot_monitoring: true
   groot_zmq_publisher_port: 1666
   groot_zmq_server_port: 1667
   ```

   4. Added `bt_navigator_rclcpp_node` section

**Result**: All Nav2 nodes successfully configure and activate. System fully operational.

**Backup**: Original config saved at `src/bin_boy_navigation/config/nav2_params.yaml.backup`

---

## Current Configuration

### Key Parameters

**Controller** (`config/controller.yaml`):
- Max velocity: 0.5 m/s (x, y)
- Max angular velocity: 1.0 rad/s
- Controller frequency: 20 Hz
- Holonomic mode: Enabled
- DWB samples: 15 (vx), 15 (vy), 20 (vtheta)

**Costmaps**:
- Resolution: 0.05m (5cm)
- Robot radius: 0.20m (includes safety margin)
- Inflation radius: 0.35m
- Update rate: 5 Hz (local), 1 Hz (global)

**Planner** (`config/planner.yaml`):
- Algorithm: NavFn (A*)
- Tolerance: 0.25m
- Unknown space: Allowed

**Recovery Behaviors**:
- Spin: max 0.5 rad/s
- Backup: 0.5m at 0.15 m/s
- Wait: 5 seconds

---

## Known Limitations / Missing Features

### ❌ Critical Gaps

1. **No Map Saving/Loading**
   - Can create maps but cannot save them
   - Must remap every session
   - No map management system

2. **No Localization Mode**
   - Only SLAM mode available
   - Cannot use pre-built maps for navigation
   - No AMCL configuration

3. **No Waypoint Interface**
   - Waypoint follower node runs but no way to send waypoints
   - Cannot program patrol routes
   - No autonomous waypoint navigation

### ⚠️ Optimization Opportunities

1. **Conservative Velocity Limits**
   - Current: 0.5 m/s
   - Robot likely capable of 0.6-0.8 m/s
   - Could increase for faster navigation

2. **Large Safety Margins**
   - Inflation radius: 0.35m (very conservative)
   - Could reduce to 0.30m for tighter navigation

3. **Loose Goal Tolerances**
   - Position: 15cm
   - Orientation: 11°
   - Could tighten for more precise navigation

---

## File Structure

### Configuration Files
```
src/bin_boy_navigation/config/
├── nav2_params.yaml          # Main Nav2 config ✅ Fixed
├── nav2_params.yaml.backup   # Backup before fix
├── controller.yaml           # DWB controller params
├── planner.yaml              # NavFn planner params
├── costmap_common.yaml       # Shared costmap params
├── global_costmap.yaml       # Global costmap
└── local_costmap.yaml        # Local costmap
```

### Launch Files
```
src/bin_boy_navigation/launch/
├── navigation.launch.py      # Nav2 nodes only
└── slam_navigation.launch.py # SLAM + Nav2 (full stack)
```

### Top-Level Scripts
```
launch_nav2.sh               # Launch full SLAM + Nav2 stack
stop_all.sh                  # Stop all ROS2/Gazebo processes
check_nav2_status.sh         # Check Nav2 node states
activate_nav2.sh             # Manually activate Nav2 nodes
```

---

## How to Use (Quick Reference)

### Launch Navigation
```bash
cd ~/bin-boy
source install/setup.bash
bash launch_nav2.sh
```

Wait 15 seconds for all nodes to activate.

### Verify System Status
```bash
# In new terminal
cd ~/bin-boy
source install/setup.bash
bash check_nav2_status.sh
```

Expected output:
```
✓ /controller_server: ACTIVE
✓ /planner_server: ACTIVE
✓ /recoveries_server: ACTIVE
✓ /bt_navigator: ACTIVE
✓ /waypoint_follower: ACTIVE

Nav2 READY FOR NAVIGATION
```

### Send Navigation Goal
1. Open RViz
2. Click "2D Nav Goal" button
3. Click destination on map
4. Drag to set orientation
5. Release to send goal

### Stop System
```bash
bash stop_all.sh
```

---

## Next Steps / Optimization Plan

See detailed plan in: `NAV2_OPTIMIZATION_PLAN.md`

**Priority 1** (Critical - 5 hours):
1. Map saving/loading system
2. AMCL localization mode
3. Waypoint navigation interface

**Priority 2** (Optimization - 2 hours):
1. Increase velocity limits (0.5 → 0.6-0.8 m/s)
2. Reduce inflation radius (0.35 → 0.30m)
3. Tighten goal tolerances
4. Standardize costmap settings

**Priority 3** (Enhancement - 2 hours):
1. Documentation improvements
2. Testing/benchmark tools
3. Advanced features

---

## Package Versions

All packages from official ROS2 Foxy repositories:

```
ros-foxy-nav2-* : 0.4.7-1focal.20230606
ros-foxy-slam-toolbox: Latest Foxy
ros-foxy-behaviortree-cpp-v3: 3.8.3
```

Build date: June 2023 (final Foxy release)

---

## Troubleshooting

### Navigation doesn't start
**Solution**:
1. Check node states: `bash check_nav2_status.sh`
2. Wait 15 seconds after launch
3. Manually activate: `bash activate_nav2.sh`

### Robot doesn't move
**Solution**:
1. Check /cmd_vel topic: `ros2 topic echo /cmd_vel`
2. Verify controller is active
3. Check for costmap obstacles

### Map quality is poor
**Solution**:
1. Drive robot slower during mapping
2. Ensure good sensor coverage
3. Check SLAM Toolbox parameters

---

## References

- **Main Documentation**: `NAV2_GUIDE.md` - Comprehensive guide
- **Quick Start**: `NAV2_QUICK_START.md` - Quick reference
- **Optimization Plan**: `NAV2_OPTIMIZATION_PLAN.md` - Future improvements
- **Nav2 Docs**: https://navigation.ros.org/

---

## Summary

The Nav2 navigation stack is **fully operational and ready for use**. All critical bugs have been resolved. The system can perform SLAM mapping and autonomous navigation in simulation.

**Current limitations** are related to missing production features (map management, localization mode) and conservative parameter settings that prioritize safety over performance.

**Recommended next step**: Implement map saving/loading system to enable reuse of created maps.

For detailed implementation steps, see `NAV2_OPTIMIZATION_PLAN.md`.
