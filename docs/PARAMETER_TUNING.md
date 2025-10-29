# Nav2 Parameter Tuning Guide

Comprehensive guide for tuning Nav2 navigation parameters for optimal performance on BIN-BOY omnidirectional robot.

---

## Table of Contents

1. [Overview](#overview)
2. [Optimizations Applied](#optimizations-applied)
3. [Parameter Categories](#parameter-categories)
4. [Incremental Tuning Process](#incremental-tuning-process)
5. [Advanced Tuning](#advanced-tuning)
6. [Performance Metrics](#performance-metrics)
7. [Troubleshooting](#troubleshooting)

---

## Overview

BIN-BOY navigation has been optimized for:
- **20-60% faster** navigation speeds
- **Tighter, more precise** paths
- **Better omnidirectional** motion quality
- **Faster recovery** from stuck situations

All parameter changes are documented with `# OPTIMIZED:` comments in configuration files.

---

## Optimizations Applied

### Summary of Changes

| Parameter | Old Value | New Value | Improvement |
|-----------|-----------|-----------|-------------|
| **Velocity Limits** |  |  |  |
| max_vel_x/y | 0.5 m/s | 0.6 m/s | +20% speed |
| max_speed_xy | 0.5 m/s | 0.6 m/s | +20% speed |
| **Goal Tolerance** |  |  |  |
| xy_goal_tolerance | 0.15m (15cm) | 0.10m (10cm) | +33% tighter |
| yaw_goal_tolerance | 0.2 rad (11°) | 0.15 rad (8.6°) | +25% tighter |
| **Trajectory Sampling** |  |  |  |
| vy_samples | 15 | 20 | +33% lateral quality |
| sim_time | 1.7s | 2.0s | +18% lookahead |
| **Inflation Radius** |  |  |  |
| inflation_radius | 0.35m | 0.30m | -13% clearance |
| Total clearance | 52.75cm | 45.75cm | Tighter paths |
| **Recovery Speed** |  |  |  |
| spin max_vel | 0.5 rad/s | 0.8 rad/s | +60% faster |
| **AMCL** |  |  |  |
| set_initial_pose | true | false | Manual init (safer) |
| **Costmap Consistency** |  |  |  |
| inflate_around_unknown | false (local) | true | Consistent behavior |

### Files Modified

1. `src/bin_boy_navigation/config/controller.yaml`
2. `src/bin_boy_navigation/config/costmap_common.yaml`
3. `src/bin_boy_navigation/config/local_costmap.yaml`
4. `src/bin_boy_navigation/config/global_costmap.yaml`
5. `src/bin_boy_navigation/config/nav2_params.yaml`
6. `src/bin_boy_navigation/config/amcl_params.yaml`

---

## Parameter Categories

### 1. Velocity Limits

**Location**: `controller.yaml` lines 33-40

**Parameters**:
```yaml
max_vel_x: 0.6         # Forward/backward speed (m/s)
max_vel_y: 0.6         # Lateral speed (m/s) - omnidirectional only
max_vel_theta: 1.0     # Rotation speed (rad/s)
max_speed_xy: 0.6      # Combined XY speed limit
```

**Tuning Guidelines**:
- **Conservative**: 0.3-0.5 m/s (safe, slow)
- **Moderate**: 0.6-0.7 m/s (balanced)
- **Aggressive**: 0.8-1.0 m/s (fast, requires careful tuning)

**Testing**:
1. Increase velocity in 0.1 m/s increments
2. Test navigation to goal
3. Watch for oscillations or overshoot
4. If unstable, reduce by 0.1 m/s

**Robot-Specific Factors**:
- Kiwi drive: Good omnidirectional control supports higher speeds
- Wheel slip: If wheels slip, reduce speed
- Payload: Heavier loads require slower speeds

---

### 2. Goal Tolerance

**Location**: `controller.yaml` lines 26-27

**Parameters**:
```yaml
xy_goal_tolerance: 0.10   # Position tolerance (meters)
yaw_goal_tolerance: 0.15  # Orientation tolerance (radians)
```

**Tuning Guidelines**:
- **Loose**: xy=0.20m, yaw=0.3rad (easier to achieve)
- **Moderate**: xy=0.10m, yaw=0.15rad (current)
- **Tight**: xy=0.05m, yaw=0.1rad (precise, may fail)

**Trade-offs**:
- Tighter = more precise, but longer convergence time
- Tighter = may fail if odometry is poor
- Looser = faster goal achievement, less precise

**Testing**:
```bash
# Send navigation goal
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{...}'

# Monitor goal achievement
ros2 topic echo /navigate_to_pose/_action/status
```

---

### 3. Trajectory Sampling

**Location**: `controller.yaml` lines 51-56

**Parameters**:
```yaml
vx_samples: 15         # Forward/backward velocity samples
vy_samples: 20         # Lateral velocity samples (omnidirectional)
vtheta_samples: 20     # Rotational velocity samples
sim_time: 2.0          # Trajectory lookahead time (seconds)
```

**Tuning Guidelines**:
- **More samples** = better path quality, higher CPU usage
- **Longer sim_time** = safer (longer lookahead), slower reaction

**Omnidirectional Importance**:
- `vy_samples` is CRITICAL for holonomic robots
- Increase to 20-30 for best lateral motion quality
- Lower values (10-15) may cause "crabbing" behavior

**CPU Monitoring**:
```bash
# Check controller CPU usage
ros2 run nav2_util cpu_usage
```

---

### 4. Inflation Radius

**Location**: Multiple files
- `costmap_common.yaml` line 39
- `local_costmap.yaml` line 54
- `global_costmap.yaml` line 46
- `nav2_params.yaml` lines 211, 245

**Parameter**:
```yaml
inflation_radius: 0.30  # Meters
```

**Total Safety Clearance**:
```
robot_radius (0.20m) + inflation_radius (0.30m) = 0.50m total
Physical robot: 0.1575m radius
Safety margin: 0.50m / 0.1575m = 3.17x robot size
```

**Tuning Guidelines**:
- **Conservative**: 0.40m (large safety margin)
- **Moderate**: 0.30m (current, balanced)
- **Aggressive**: 0.20m (tight spaces, collision risk)

**Testing**:
1. Navigate through narrow passages
2. Watch for collisions in RViz
3. Monitor `/local_costmap/costmap` topic
4. If collisions occur, increase radius by 0.05m

**Important**: All 5 locations must match for consistent behavior.

---

### 5. Recovery Behaviors

**Location**: `nav2_params.yaml` line 147

**Parameter**:
```yaml
spin:
  max_rotational_vel: 0.8  # rad/s
```

**Tuning Guidelines**:
- **Slow**: 0.3-0.5 rad/s (gentle recovery)
- **Moderate**: 0.8 rad/s (current)
- **Fast**: 1.0-1.5 rad/s (quick unstuck)

**When Recovery Triggers**:
- Robot stuck for >10 seconds
- Path blocked by unexpected obstacle
- Controller fails to make progress

**Testing**:
```bash
# Manually trigger recovery
ros2 action send_goal /spin nav2_msgs/action/Spin "{target_yaw: 3.14}"
```

---

### 6. AMCL Localization

**Location**: `amcl_params.yaml`

**Key Parameters**:
```yaml
set_initial_pose: false    # Require manual initialization
max_particles: 2000        # Particle filter count
min_particles: 500
recovery_alpha_slow: 0.001 # Particle reset thresholds
recovery_alpha_fast: 0.1
```

**Tuning for Poor Localization**:

If robot loses track of position:
```yaml
max_particles: 5000        # More particles (higher CPU)
min_particles: 1000
laser_max_range: 30.0      # Increase if large space
```

If robot drifts over time:
```yaml
odom_alpha1: 0.3  # Increase noise if odometry poor
odom_alpha2: 0.3
odom_alpha3: 0.3
odom_alpha4: 0.3
```

**Testing**:
```bash
# Monitor particle spread
ros2 topic echo /particle_cloud

# Check localization quality
ros2 topic echo /amcl_pose
```

---

## Incremental Tuning Process

### Step-by-Step Testing Protocol

**CRITICAL**: Change ONE parameter at a time and test thoroughly.

### 1. Velocity Tuning (30 minutes)

**Current**: max_vel = 0.6 m/s
**Goal**: Find maximum stable velocity

```bash
# 1. Launch navigation
bash launch_localization.sh map_20251028_203407

# 2. Edit controller.yaml
# Increase max_vel_x and max_vel_y to 0.7
# Also increase max_speed_xy to 0.7

# 3. Rebuild
colcon build --packages-select bin_boy_navigation
source install/setup.bash

# 4. Send navigation goal and observe
# - Smooth motion? → Try 0.8 m/s
# - Oscillating? → Reduce to 0.6 m/s
# - Overshooting goals? → Reduce speed
```

**Success Criteria**:
- Smooth acceleration/deceleration
- No oscillations near goal
- No wheel slippage
- Stable trajectory following

---

### 2. Inflation Radius Tuning (20 minutes)

**Current**: inflation_radius = 0.30m
**Goal**: Minimum safe clearance

```bash
# 1. Create test course with narrow passage
# 2. Reduce inflation_radius to 0.25m
# 3. Navigate through passage
# 4. If collision: increase by 0.05m
# 5. If too conservative: decrease by 0.05m
```

**IMPORTANT**: Update ALL 5 locations:
- costmap_common.yaml
- local_costmap.yaml
- global_costmap.yaml
- nav2_params.yaml (local costmap)
- nav2_params.yaml (global costmap)

---

### 3. Goal Tolerance Tuning (15 minutes)

**Current**: xy=0.10m, yaw=0.15rad
**Goal**: Tightest achievable precision

```bash
# 1. Send navigation goal to precise location
# 2. Measure final position error:
ros2 topic echo /amcl_pose

# 3. If consistently within 5cm:
#    → Try xy=0.05m
# 4. If often fails to converge:
#    → Increase to xy=0.15m
```

---

### 4. Trajectory Sampling Tuning (20 minutes)

**Current**: vy_samples=20, sim_time=2.0
**Goal**: Optimize quality vs CPU

```bash
# 1. Monitor CPU usage during navigation:
top -p $(pgrep controller_server)

# 2. If CPU < 50%:
#    → Try vy_samples=25 for even better quality
# 3. If CPU > 80%:
#    → Reduce vy_samples to 15

# 4. Test lateral motion quality:
#    - Send goal requiring sideways motion
#    - Watch for smooth lateral trajectories
```

---

### 5. Recovery Speed Tuning (10 minutes)

**Current**: max_rotational_vel = 0.8 rad/s
**Goal**: Fastest safe recovery

```bash
# 1. Manually trigger recovery:
ros2 action send_goal /spin nav2_msgs/action/Spin "{target_yaw: 6.28}"

# 2. If stable:
#    → Try 1.0 rad/s
# 3. If robot tips or wheels slip:
#    → Reduce to 0.6 rad/s
```

---

## Advanced Tuning

### DWB Critic Weights

**Location**: `controller.yaml` lines 77-86

Fine-tune trajectory scoring:

```yaml
PathAlign.scale: 32.0      # How much to follow global path
GoalAlign.scale: 24.0      # How much to face goal
PathDist.scale: 32.0       # Reward progress along path
GoalDist.scale: 24.0       # Reward progress toward goal
BaseObstacle.scale: 0.02   # Penalty for obstacle proximity
```

**Tuning**:
- Increase `PathAlign` if robot cuts corners
- Increase `GoalAlign` if robot approaches goal from wrong angle
- Increase `BaseObstacle` if robot gets too close to obstacles

---

### Costmap Update Frequencies

**Location**: `nav2_params.yaml`

```yaml
local_costmap:
  update_frequency: 5.0    # Hz - how often costmap updates
  publish_frequency: 2.0   # Hz - how often costmap published

global_costmap:
  update_frequency: 1.0    # Hz
  publish_frequency: 1.0   # Hz
```

**Trade-offs**:
- Higher frequency = better obstacle reactivity, higher CPU
- Lower frequency = less CPU, slower reaction to obstacles

---

### Controller Frequency

**Location**: `nav2_params.yaml`

```yaml
controller_server:
  controller_frequency: 20.0  # Hz - trajectory updates
```

**Tuning**:
- Higher (30-50 Hz) = smoother control, higher CPU
- Lower (10-15 Hz) = less CPU, jerkier motion
- 20 Hz is good balance for most robots

---

## Performance Metrics

### Measuring Navigation Quality

**1. Goal Achievement Rate**
```bash
# Run benchmark
bash scripts/nav_benchmark.sh

# Success rate should be >95%
```

**2. Time to Goal**
```bash
# Measure time for standard route
ros2 topic echo /navigate_to_pose/_action/feedback
```

**3. Path Smoothness**
```bash
# Visualize actual path vs planned path in RViz
# Enable "DWB Local Plan" display
```

**4. CPU Usage**
```bash
# Monitor during navigation
top -p $(pgrep controller_server)
# Should stay <80%
```

**5. Obstacle Clearance**
```bash
# Monitor costmap
ros2 topic echo /local_costmap/costmap | grep "min_distance"
# Should maintain >0.20m from obstacles
```

---

## Troubleshooting

### Robot Oscillates Near Goal

**Symptom**: Robot wobbles back and forth when approaching goal

**Causes**:
- Goal tolerance too tight
- Velocity too high
- Critic weights unbalanced

**Solutions**:
1. Increase `xy_goal_tolerance` to 0.15m
2. Reduce `max_vel_x/y` by 0.1 m/s
3. Increase `GoalDist.scale` to 30.0

---

### Robot Takes Inefficient Paths

**Symptom**: Robot makes wide detours or doesn't follow global plan

**Causes**:
- Inflation radius too large
- PathAlign weight too low
- Local costmap too small

**Solutions**:
1. Reduce `inflation_radius` by 0.05m
2. Increase `PathAlign.scale` to 40.0
3. Increase local costmap size in `local_costmap.yaml`

---

### Robot Gets Stuck Often

**Symptom**: Robot frequently triggers recovery behaviors

**Causes**:
- Inflation radius too large (can't fit through passages)
- Velocity too high (overshoots corrections)
- Recovery behaviors not aggressive enough

**Solutions**:
1. Reduce `inflation_radius` to 0.25m
2. Reduce `max_vel` to 0.5 m/s
3. Increase spin `max_rotational_vel` to 1.0 rad/s

---

### Poor Lateral Motion Quality

**Symptom**: Robot "crabbing" (diagonal motion when should move straight sideways)

**Causes**:
- vy_samples too low
- Holonomic mode disabled
- Controller frequency too low

**Solutions**:
1. Increase `vy_samples` to 25-30
2. Verify `holonomic_robot: true` in controller.yaml
3. Increase `controller_frequency` to 30.0

---

### AMCL Loses Localization

**Symptom**: Robot position jumps around on map

**Causes**:
- Too few particles
- Poor odometry
- Map lacks features

**Solutions**:
1. Increase `max_particles` to 5000
2. Tune odometry noise (`odom_alpha` parameters)
3. Re-map environment with better coverage
4. Increase `laser_max_range`

---

## Quick Reference

### Configuration File Locations

```
src/bin_boy_navigation/config/
├── controller.yaml          # DWB controller, velocities, tolerances
├── costmap_common.yaml      # Shared costmap parameters
├── local_costmap.yaml       # Rolling window costmap
├── global_costmap.yaml      # Static map costmap
├── nav2_params.yaml         # Full Nav2 stack configuration
└── amcl_params.yaml         # Localization parameters
```

### Testing Commands

```bash
# Launch navigation
bash launch_localization.sh map_20251028_203407

# Rebuild after parameter changes
colcon build --packages-select bin_boy_navigation
source install/setup.bash

# Send navigation goal
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{...}'

# Monitor controller
ros2 topic echo /cmd_vel

# Check costmap
ros2 topic echo /local_costmap/costmap

# Test recovery
ros2 action send_goal /spin nav2_msgs/action/Spin "{target_yaw: 3.14}"

# Run benchmark
bash scripts/nav_benchmark.sh
```

### Optimization Checklist

- [ ] Velocities tested at 0.6, 0.7, 0.8 m/s
- [ ] Goal tolerances validated (actual vs target)
- [ ] Inflation radius tested in narrow passages
- [ ] Trajectory sampling optimized for CPU vs quality
- [ ] Recovery behaviors tested (spin, backup)
- [ ] AMCL localization stable over 5+ minutes
- [ ] CPU usage <80% during navigation
- [ ] Goal achievement rate >95%
- [ ] All parameter files consistent

---

## See Also

- `MAP_MANAGEMENT.md` - Map creation and loading
- `NAV2_OPTIMIZATION_PLAN.md` - Full optimization roadmap
- [Nav2 Documentation](https://navigation.ros.org/)
- [DWB Controller Tuning Guide](https://navigation.ros.org/tuning/)
