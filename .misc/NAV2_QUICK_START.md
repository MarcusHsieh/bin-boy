# Nav2 Quick Start Guide

## Clean Startup & Shutdown

### Start Nav2 System

**Option 1: Using helper script (recommended)**
```bash
bash ~/bin-boy/launch_nav2.sh
```

**Option 2: Manual launch**
```bash
cd ~/bin-boy
source install/setup.bash
ros2 launch bin_boy_navigation slam_navigation.launch.py
```

**What launches:**
- Gazebo simulation with robot
- SLAM Toolbox for mapping
- Nav2 navigation stack (5 nodes)
- RViz visualization

**Autoactivation:**
- Nav2 nodes automatically activate after **5 seconds**
- Wait ~15 seconds total before sending navigation commands

---

### Stop Everything Cleanly

**In the launch terminal:**
- Press **Ctrl+C** once
- Wait 10 seconds for clean shutdown

**If processes don't stop:**
```bash
bash ~/bin-boy/stop_all.sh
```

---

## Verify Nav2 is Ready

**Check node status:**
```bash
bash ~/bin-boy/check_nav2_status.sh
```

**Expected output:**
```
✓ /controller_server: ACTIVE
✓ /planner_server: ACTIVE
✓ /recoveries_server: ACTIVE
✓ /bt_navigator: ACTIVE
✓ /waypoint_follower: ACTIVE
✓ Local costmap publishing
✓ Global costmap publishing
✓ Single /cmd_vel publisher

✓ Nav2 READY FOR NAVIGATION
```

---

## Manual Activation (if autostart fails)

**If nodes show INACTIVE after 15 seconds:**
```bash
bash ~/bin-boy/activate_nav2.sh
```

Then verify:
```bash
bash ~/bin-boy/check_nav2_status.sh
```

---

## Send Navigation Commands

### Method 1: RViz (Interactive)

**In RViz window:**
1. Click **"2D Nav Goal"** button (top toolbar)
2. Click on map (white space = free, black = obstacle)
3. Drag to set orientation arrow
4. Release mouse

**Robot will:**
- Plan path (green line = global, yellow = local)
- Navigate to goal avoiding obstacles
- Stop when goal reached

### Method 2: Command Line

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: 'map'
  pose:
    position:
      x: 2.0
      y: 1.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
"
```

---

## Troubleshooting

### Nodes Stay INACTIVE

**Symptom:** check_nav2_status.sh shows INACTIVE after 15+ seconds

**Solution:**
```bash
bash ~/bin-boy/activate_nav2.sh
```

### Robot Doesn't Move

**Check 1: Nav2 active?**
```bash
ros2 lifecycle get /controller_server
# Should show: active [3]
```

**Check 2: /cmd_vel publishing?**
```bash
ros2 topic hz /cmd_vel
# Should show ~20 Hz when navigating
```

**Check 3: Multiple cmd_vel publishers?**
```bash
ros2 topic info /cmd_vel
# Should show Publisher count: 1
```

If Publisher count > 1: person_tracker is interfering
- Stop person_tracker or disable following

### "No Path Found"

**Check 1: Goal in free space?**
- In RViz, goal should be in white area (not black/gray)

**Check 2: Map available?**
```bash
ros2 topic hz /map
# Should show ~0.5 Hz
```

**Check 3: Global costmap working?**
```bash
ros2 topic hz /global_costmap/costmap_raw
# Should show ~1 Hz
```

---

## Common Commands

### Check all nodes
```bash
ros2 node list
```

### Check lifecycle state
```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

### Monitor navigation action
```bash
ros2 action list
ros2 action info /navigate_to_pose
```

### View costmaps
Already visible in RViz if displays configured

### Cancel navigation
```bash
# In RViz: Click "2D Nav Goal" on current position
# Or press Ctrl+C in goal terminal
```

---

## Workflow Summary

**Every time you start:**
1. Run: `bash ~/bin-boy/launch_nav2.sh`
2. Wait 15 seconds
3. Verify: `bash ~/bin-boy/check_nav2_status.sh`
4. If not active: `bash ~/bin-boy/activate_nav2.sh`
5. Send goals via RViz "2D Nav Goal"

**Every time you stop:**
1. Press Ctrl+C in launch terminal
2. Wait 10 seconds
3. If stuck: `bash ~/bin-boy/stop_all.sh`

---

## Files Reference

| Script | Purpose |
|--------|---------|
| `launch_nav2.sh` | Start SLAM + Nav2 system |
| `check_nav2_status.sh` | Verify Nav2 nodes are active |
| `activate_nav2.sh` | Manually activate nodes if autostart fails |
| `stop_all.sh` | Force kill all ROS/Gazebo processes |
| `kill_and_restart.sh` | Emergency restart (legacy) |

---

## Next Steps

**After confirming basic navigation works:**
- Add Nav2 visualizations to RViz (Phase 4.3)
- Implement behavior arbitration (Phase 4.4)
- Integrate person following with Nav2 (Phase 4.5)

**See full documentation:** `NAV2_GUIDE.md`
