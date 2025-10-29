# Map Management Guide

Complete guide for creating, saving, loading, and managing maps for BIN-BOY navigation.

---

## Table of Contents

1. [Overview](#overview)
2. [Map Creation (SLAM Mode)](#map-creation-slam-mode)
3. [Map Saving](#map-saving)
4. [Map Loading](#map-loading)
5. [Map Management](#map-management)
6. [Troubleshooting](#troubleshooting)

---

## Overview

BIN-BOY uses two navigation modes:

- **SLAM Mode**: Create new maps using real-time mapping
- **Localization Mode**: Navigate using pre-saved maps

Maps are stored in `~/bin-boy/maps/` with timestamp-based names.

---

## Map Creation (SLAM Mode)

### 1. Launch SLAM Mapping

```bash
cd ~/bin-boy
bash launch_nav2.sh
```

This launches:
- Gazebo simulation
- SLAM Toolbox (real-time mapping)
- Nav2 navigation stack
- RViz visualization

### 2. Drive the Robot

**Option A: Teleop (Manual)**
```bash
# In a new terminal
cd ~/bin-boy
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

**Option B: Navigation (Autonomous)**
- In RViz, click "2D Goal Pose"
- Click and drag to set goal location and orientation
- Robot autonomously navigates while mapping

### 3. Mapping Best Practices

**Coverage**:
- Drive through all areas you want mapped
- Ensure loop closure (return to start)
- Cover doorways, corners, and tight spaces

**Speed**:
- Move slowly for better map quality
- Pause briefly in new areas
- Avoid rapid rotations

**Loop Closure**:
- Return to previously mapped areas
- SLAM Toolbox will auto-detect and optimize loops
- Watch for map correction jumps

---

## Map Saving

### Automatic Save with Script

```bash
cd ~/bin-boy
bash scripts/save_map.sh
```

This will:
1. Create timestamped map file (e.g., `map_20251028_203407`)
2. Save to `~/bin-boy/maps/`
3. Generate `.yaml` and `.pgm` files
4. Display confirmation

**Output Example**:
```
Map saved to: /home/mj/bin-boy/maps/map_20251028_203407.yaml
Map saved to: /home/mj/bin-boy/maps/map_20251028_203407.pgm
```

### Manual Save (Alternative)

```bash
cd ~/bin-boy/maps
ros2 run nav2_map_server map_saver_cli -f my_custom_map_name
```

---

## Map Loading

### Launch Localization Mode

```bash
cd ~/bin-boy
bash launch_localization.sh <map_name>
```

**Example**:
```bash
bash launch_localization.sh map_20251028_203407
```

This will:
1. Launch Gazebo simulation
2. Load specified map
3. Start AMCL localization
4. Launch Nav2 navigation stack
5. Open RViz

### Set Initial Robot Pose

**IMPORTANT**: After launching localization mode, you MUST set the robot's initial pose:

1. In RViz, click **"2D Pose Estimate"** button
2. Click on the map where the robot is located
3. Drag to set the robot's orientation
4. AMCL will initialize particle filter around this pose

**Symptoms of missing initial pose**:
- Robot appears at (0,0) on map
- Navigation goals fail immediately
- AMCL particles not visible

---

## Map Management

### List All Saved Maps

```bash
cd ~/bin-boy
bash scripts/list_maps.sh
```

**Example Output**:
```
Available maps in /home/mj/bin-boy/maps:

  map_20251028_203407  (saved: 2025-10-28 20:34:07)
  map_20251028_203107  (saved: 2025-10-28 20:31:07)
  sim_room_1           (saved: 2025-10-27 15:22:33)

Usage: bash launch_localization.sh <map_name>
```

### Map File Structure

Each map consists of two files:

**`.yaml` - Map Metadata**
```yaml
image: map_20251028_203407.pgm
mode: trinary
resolution: 0.05
origin: [-9.850000, -9.850000, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

**`.pgm` - Occupancy Grid Image**
- Grayscale image of the environment
- White = free space
- Black = occupied/obstacles
- Gray = unknown

### Delete a Map

```bash
cd ~/bin-boy/maps
rm map_name.yaml map_name.pgm
```

### Rename a Map

```bash
cd ~/bin-boy/maps
mv old_name.yaml new_name.yaml
mv old_name.pgm new_name.pgm

# Update the .yaml file to reference the new .pgm name
sed -i 's/old_name.pgm/new_name.pgm/' new_name.yaml
```

---

## Troubleshooting

### Map Not Appearing in RViz

**Symptom**: RViz shows no map, just grid

**Causes & Solutions**:

1. **QoS Mismatch**
   - Solution: Map display in RViz should use "Transient Local" durability
   - Already fixed in `simulation.rviz` (lines 207, 214)

2. **Wrong Fixed Frame**
   - Solution: RViz Fixed Frame should be "map" for localization mode
   - Already fixed in `simulation.rviz` (line 315)

3. **Map Not Published**
   ```bash
   ros2 topic echo /map --once
   ```
   - If no output, map_server failed to load
   - Check map file path in launch arguments

### AMCL Not Localizing

**Symptom**: Robot position incorrect, particles scattered everywhere

**Solutions**:

1. **Set Initial Pose**
   - Use "2D Pose Estimate" in RViz
   - Click where robot actually is on the map

2. **Check TF Transforms**
   ```bash
   ros2 run tf2_ros tf2_echo map odom
   ```
   - Should show transform between map and odom
   - If error, AMCL failed to initialize

3. **Increase Particle Count**
   - Edit `amcl_params.yaml`
   - Increase `max_particles` from 2000 to 5000
   - Helps with difficult localization scenarios

### Map File Errors

**Symptom**: Launch fails with "could not load map" error

**Solutions**:

1. **Check File Exists**
   ```bash
   ls ~/bin-boy/maps/map_name.*
   ```

2. **Verify YAML Syntax**
   ```bash
   cat ~/bin-boy/maps/map_name.yaml
   ```
   - Ensure `image:` path is correct
   - Check no typos in parameters

3. **Check PGM File**
   ```bash
   file ~/bin-boy/maps/map_name.pgm
   ```
   - Should show: "PGM image"
   - If corrupted, re-save map

### Map Drift During Navigation

**Symptom**: Robot position slowly drifts from true location

**Causes**:
- Poor odometry calibration
- Lack of features for AMCL
- Incorrect AMCL parameters

**Solutions**:

1. **Improve Map Quality**
   - Re-map with slower, more thorough coverage
   - Ensure good loop closures

2. **Tune AMCL Parameters**
   - See `PARAMETER_TUNING.md` for AMCL tuning
   - Adjust odometry noise parameters

3. **Add More Features**
   - Maps with few features (empty rooms) localize poorly
   - Add obstacles/landmarks if possible

---

## Advanced: Map Editing

### Manually Edit PGM

1. Open `.pgm` file in image editor (GIMP, ImageMagick)
2. Paint obstacles (black) or free space (white)
3. Save as grayscale PGM
4. Reload map in localization mode

**Use Cases**:
- Remove noise/artifacts from SLAM
- Add "virtual walls" to restrict navigation
- Close small gaps in obstacles

### Merge Multiple Maps

**Not directly supported**, but can be done:

1. Convert all `.pgm` files to same resolution
2. Align maps using image editor
3. Merge layers
4. Export as single PGM
5. Create new `.yaml` with combined origin

---

## Quick Reference

### Common Commands

```bash
# Save current map
bash scripts/save_map.sh

# List all maps
bash scripts/list_maps.sh

# Load map and navigate
bash launch_localization.sh map_20251028_203407

# Create new map
bash launch_nav2.sh
# (drive around, then save)

# Check map topic
ros2 topic echo /map --once

# Verify map metadata
cat ~/bin-boy/maps/map_name.yaml
```

### Map Storage

- **Location**: `~/bin-boy/maps/`
- **Format**: `.yaml` + `.pgm` pair
- **Naming**: Timestamps or custom names
- **Resolution**: 0.05m (5cm per pixel)

### RViz Settings for Maps

- **Fixed Frame**: `map`
- **Map Topic**: `/map`
- **QoS Durability**: `Transient Local`
- **QoS History**: `Keep Last (1)`

---

## See Also

- `PARAMETER_TUNING.md` - Optimize AMCL for better localization
- `NAV2_OPTIMIZATION_PLAN.md` - Full navigation system tuning guide
- `launch_localization.sh` - Localization launch script
- `launch_nav2.sh` - SLAM mapping launch script
