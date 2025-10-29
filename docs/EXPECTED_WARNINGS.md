# Expected Warnings vs Actual Errors

This document explains which warnings are **EXPECTED BEHAVIOR** and which are **ACTUAL ERRORS** that need fixing.

---

## EXPECTED WARNINGS (Safe to Ignore)

### 1. "Invalid frame ID 'map' passed to canTransform"

**Warning Message:**
```
[rviz2-5] Warning: Invalid frame ID "map" passed to canTransform argument target_frame - frame does not exist
         at line 133 in /tmp/binarydeb/ros-foxy-tf2-0.13.14/src/buffer_core.cpp
```

**Why This Happens:**
- The `map` frame is published by AMCL
- AMCL requires an **initial pose** to be set before it publishes the map->odom transform
- With `set_initial_pose: false` (our optimization), you MUST manually set the pose

**This is EXPECTED and SAFE**

**How to Fix:**
1. Wait for RViz to open
2. Click "2D Pose Estimate" button in RViz
3. Click on the map where the robot is located
4. Drag to set orientation
5. Warning will disappear once pose is set

**Status:** ✓ **NOT AN ERROR** - System working as designed

---

### 2. "TF_OLD_DATA ignoring data from the past"

**Warning Message:**
```
[controller_server-10] Warning: TF_OLD_DATA ignoring data from the past for frame base_footprint at time 33.187000
```

**Why This Happens:**
- This happens during system startup when nodes start at different times
- TF buffer receives old transform data that's already been superseded
- Very common during the first 10-15 seconds of launch

**This is EXPECTED during startup**

**How to Fix:**
- Wait 15-20 seconds after launch
- Warnings should stop once all nodes are synchronized
- If warnings persist after 30 seconds, there may be a clock sync issue

**Status:** ✓ **NOT AN ERROR** - Transient startup behavior

---

### 3. "lifecycle node launched. Waiting on external lifecycle transitions"

**Info Message:**
```
[map_server-7] map_server lifecycle node launched.
               Waiting on external lifecycle transitions to activate
```

**Why This Happens:**
- Nav2 uses lifecycle nodes that need to be activated by lifecycle_manager
- The message appears during the brief window before lifecycle_manager activates the node

**This is INFORMATIONAL**

**What to Look For:**
- A few seconds later, you should see:
  ```
  [lifecycle_manager-9] Managed nodes are active
  ```
- This confirms nodes activated successfully

**Status:** ✓ **NOT AN ERROR** - Normal lifecycle behavior

---

### 4. "Be aware that nodes share an exact name"

**Warning Message:**
```
WARNING: Be aware that are nodes in the graph that share an exact name, this can have unintended side effects.
```

**Why This Happens:**
- Multiple lifecycle_manager nodes may have similar names
- ROS 2 namespacing creates node hierarchies that can look duplicated

**This is a ROS 2 warning**

**Impact:**
- Generally harmless in our setup
- Nav2 uses proper namespacing to avoid conflicts

**Status:** ✓ **NOT AN ERROR** - Can be ignored

---

## ACTUAL ERRORS (Need Fixing)

### 1. "Map file not found"

**Error Message:**
```
[ERROR] [map_server]: Failed to load map file: /path/to/map.yaml
```

**Why This Happens:**
- Map file path is incorrect
- Map file doesn't exist

**How to Fix:**
```bash
# List available maps
bash scripts/list_maps.sh

# Launch with correct map name
bash launch_localization.sh map_20251028_203407
```

**Status:** ✗ **THIS IS AN ERROR** - Fix the map path

---

### 2. "Failed to activate node"

**Error Message:**
```
[ERROR] [lifecycle_manager]: Failed to activate map_server
```

**Why This Happens:**
- Node configuration is invalid
- Required parameters missing
- Dependency not available

**How to Fix:**
- Check parameter files for syntax errors
- Verify all config files are installed
- Rebuild package: `colcon build --packages-select bin_boy_navigation`

**Status:** ✗ **THIS IS AN ERROR** - Check configuration

---

### 3. "Global Costmap: Map not received"

**Error Message:**
```
[ERROR] [global_costmap]: Timed out waiting for map
```

**Why This Happens:**
- map_server not publishing map
- QoS mismatch between publisher and subscriber

**How to Fix:**
- Verify map is being published: `ros2 topic echo /map --once`
- Check QoS settings (should be Transient Local for map topic)

**Status:** ✗ **THIS IS AN ERROR** - Check map_server

---

### 4. "AMCL: No laser scan received"

**Error Message:**
```
[ERROR] [amcl]: No laser scan received
```

**Why This Happens:**
- LIDAR not publishing to /scan topic
- Topic remapping incorrect

**How to Fix:**
- Verify scan topic: `ros2 topic echo /scan --once`
- Check simulation is running properly

**Status:** ✗ **THIS IS AN ERROR** - Check LIDAR

---

## How to Verify System is Working

### Quick Health Check

Run these commands after launch (wait 15 seconds):

```bash
# 1. Check all required nodes are running
ros2 node list | grep -E "(map_server|amcl|controller|planner)"

# Expected output:
# /map_server
# /amcl
# /controller_server
# /planner_server

# 2. Check map is published
ros2 topic echo /map --once

# Should see map data (header, info, data array)

# 3. Check TF tree is complete (AFTER setting initial pose)
ros2 run tf2_tools view_frames

# Should show: map -> odom -> base_footprint -> ...
```

---

## Launch Timeline

Understanding what happens when:

```
T+0s:   Launch starts
T+2s:   Gazebo starts, robot spawns
T+3s:   map_server and AMCL start
T+5s:   Map loads successfully
T+6s:   AMCL waits for initial pose (map frame warnings START)
T+8s:   Navigation stack starts
T+15s:  TF_OLD_DATA warnings stop
T+??s:  USER SETS INITIAL POSE IN RVIZ
T+??s+1: Map frame warnings STOP, system fully operational
```

**Key Point:** The system CANNOT be fully operational until you set the initial pose in RViz!

---

## Summary: Is My System Broken?

**NO** if you see:
- ✓ "Invalid frame ID 'map'" warnings (expected before initial pose set)
- ✓ "TF_OLD_DATA" warnings during first 15 seconds
- ✓ "Waiting on external lifecycle transitions" (brief message)
- ✓ Nodes all running (`ros2 node list`)
- ✓ Map published (`ros2 topic echo /map --once`)

**YES** if you see:
- ✗ "Failed to load map" errors
- ✗ "Failed to activate" errors
- ✗ Nodes missing from `ros2 node list`
- ✗ No map on `/map` topic
- ✗ System crashes/exits immediately

---

## Testing Your Launch

```bash
# 1. Launch system
bash launch_localization.sh map_20251028_203407

# 2. Wait 15 seconds (ignore warnings)

# 3. Verify nodes running
ros2 node list | wc -l
# Should show 40+ nodes

# 4. Verify map published
ros2 topic echo /map --once
# Should show map data

# 5. Set initial pose in RViz
# Click "2D Pose Estimate", click on map, drag

# 6. Verify map frame exists
ros2 run tf2_ros tf2_echo map base_footprint
# Should show transform, warnings stop

# 7. Test navigation
# Click "2D Nav Goal", click destination, drag
```

---

## Conclusion

**The warnings you're seeing are EXPECTED BEHAVIOR due to the optimization `set_initial_pose: false`.**

This is actually SAFER because:
- Prevents AMCL from assuming robot is at (0,0,0)
- Forces manual verification of robot position
- Reduces chance of localization errors

**To use the system:**
1. Launch and wait 15 seconds
2. **Set initial pose in RViz**
3. System becomes fully operational
4. Send navigation goals

**The system is NOT broken!**
