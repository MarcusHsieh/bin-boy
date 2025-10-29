# Global Localization - Finding Robot from Unknown Position

This guide explains how to enable AMCL's global localization capability, which allows the robot to determine its position anywhere in the map without prior knowledge.

---

## 🎯 Problem: Kidnapped Robot

**Scenario**: Robot is placed at random location in environment
**Current Behavior**: AMCL assumes robot is at (0,0,0), localization fails
**Desired Behavior**: AMCL searches entire map and finds robot automatically

---

## 🧠 How Global Localization Works

### Standard AMCL (Current):
- 500-2000 particles clustered around initial pose
- Tracks robot from known starting position
- Fast and efficient (low particle count)

### Global Localization AMCL:
- 5000-10000 particles spread across ENTIRE map
- Each particle is a hypothesis: "robot might be here"
- Laser scans eliminate impossible locations
- Particles converge on actual robot position
- Once converged, reduces to normal tracking mode

---

## ⚙️ Configuration Changes Required

### Option 1: Temporary Global Localization (Recommended)

Use ROS 2 service to trigger global localization on demand:

```bash
# After launching navigation, if robot is in unknown location:
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

**What this does**:
- Spreads all particles uniformly across the map
- AMCL searches entire space
- Takes 10-30 seconds to converge
- Returns to normal tracking once localized

**Advantages**:
- No configuration changes needed
- Only uses high particle count when necessary
- Works with current setup

---

### Option 2: Permanent Global Localization Mode

Modify AMCL parameters for always-on global localization:

**File**: `src/bin_boy_navigation/config/amcl_params.yaml`

**Changes Required**:

```yaml
amcl:
  ros__parameters:
    # Increase particle count significantly
    min_particles: 1000          # was: 500
    max_particles: 10000         # was: 2000

    # Disable initial pose (spread particles everywhere)
    set_initial_pose: false

    # Enable aggressive resampling for convergence
    recovery_alpha_slow: 0.001   # was: 0.0
    recovery_alpha_fast: 0.1     # was: 0.0

    # Allow particles to spread across map
    pf_err: 0.05                 # Position error threshold
    pf_z: 0.99                   # Confidence threshold

    # More aggressive update to find position faster
    update_min_d: 0.1            # was: 0.15
    update_min_a: 0.1            # was: 0.2

    # Laser model parameters (help with matching)
    laser_model_type: "likelihood_field"  # already set
    laser_likelihood_max_dist: 2.0        # was: 2.0

    # Initial pose covariance (large = uncertain = spread particles)
    initial_cov_xx: 25.0         # was: 0.25 (100x larger!)
    initial_cov_yy: 25.0         # was: 0.25
    initial_cov_aa: 6.28         # was: 0.068 (full rotation uncertainty)
```

**Trade-offs**:
- ✅ Can find robot from any position
- ✅ Handles "kidnapped robot" problem
- ❌ Higher CPU usage (10x more particles)
- ❌ Slower initial convergence (10-30 seconds)
- ❌ May have false positives in symmetric environments

---

## 🧪 Testing Global Localization

### Test 1: Service-Based Global Localization (Easy)

```bash
# Terminal 1: Launch navigation
bash launch_localization.sh map_20251028_203407

# Wait for system to start (~20 seconds)

# Terminal 2: Teleport robot to random location in Gazebo
# Use Gazebo GUI to drag robot to different position

# Terminal 2: Trigger global localization
cd ~/bin-boy
source install/setup.bash
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty

# Watch in RViz:
# - Particle cloud spreads across entire map (green arrows everywhere)
# - After 10-30 seconds, particles converge on robot's actual location
# - Robot is now localized!
```

---

### Test 2: Permanent Global Localization (Advanced)

**Step 1**: Modify AMCL parameters as shown above

**Step 2**: Rebuild package
```bash
cd ~/bin-boy
colcon build --packages-select bin_boy_navigation
source install/setup.bash
```

**Step 3**: Launch and test
```bash
# Launch navigation
bash launch_localization.sh map_20251028_203407

# In Gazebo: Move robot to random location

# In RViz: Watch particle cloud
# - Should see particles spread across map
# - Wait 10-30 seconds
# - Particles converge on actual robot position
```

---

## 📊 Monitoring Global Localization

### Check Particle Spread (RViz)

In RViz, enable the **Particle Cloud** display:
1. Add → By topic → `/particle_cloud` → PoseArray
2. Color: Green with alpha 0.5
3. Arrow scale: 0.1m length, 0.02m width

**What to look for**:
- **Before convergence**: Particles spread across entire map
- **During convergence**: Particles cluster in 2-3 locations
- **After convergence**: All particles around robot's actual position

---

### Check Localization Quality (Terminal)

```bash
# Monitor AMCL pose covariance
ros2 topic echo /amcl_pose --field pose.covariance

# Good localization: [0.01, 0, 0, 0, 0, 0,
#                      0, 0.01, 0, 0, 0, 0,
#                      ...]
# (small values = confident)

# Bad localization:  [5.0, 0, 0, 0, 0, 0,
#                     0, 5.0, 0, 0, 0, 0,
#                     ...]
# (large values = uncertain)
```

---

## 🎯 When to Use Global Localization

### Use Global Localization When:
- ✅ Robot position is completely unknown
- ✅ Robot was manually moved (kidnapped)
- ✅ Localization has diverged/failed
- ✅ Starting navigation in new session without known position
- ✅ Testing robustness of localization system

### Use Standard Localization When:
- ✅ Robot always starts at known position (simulation)
- ✅ Position is known from previous session
- ✅ Computational resources are limited
- ✅ Fast startup is required

---

## ⚠️ Limitations and Caveats

### 1. Symmetric Environments
**Problem**: If multiple locations in map look identical (e.g., long hallway, repeating rooms)
**Effect**: Particles may converge to wrong location
**Solution**:
- Add unique landmarks
- Use larger particle count
- Verify convergence by moving robot and watching if tracking continues correctly

### 2. Computational Cost
**Current**: ~2000 particles = ~5% CPU
**Global**: ~10000 particles = ~25% CPU
**Impact**: May affect real-time performance on embedded systems

### 3. Convergence Time
**Standard localization**: Instant (known position)
**Global localization**: 10-30 seconds depending on:
- Map size
- Particle count
- Environment complexity
- Laser scan quality

### 4. Map Quality
**Requirement**: Map must have distinctive features
**Bad maps for global localization**:
- Empty rooms with no features
- Perfectly symmetric layouts
- Low resolution maps (large grid cells)
**Good maps**:
- Varied obstacles and features
- Asymmetric layout
- High resolution (5cm grid)

---

## 🔬 Advanced: Global Localization Algorithm

### How AMCL Finds Robot Position:

**Initialization Phase**:
```
1. Spread particles uniformly across FREE space in map
   - Skip obstacles (marked cells in occupancy grid)
   - Distribute across all valid (x,y) positions
   - Random orientations (theta)

2. Initial particle weight = 1/N (equal probability)
```

**Convergence Phase** (repeated every laser scan):
```
3. Predict: Move particles according to odometry
   - Add noise based on odom_alpha parameters

4. Update: Weight particles by laser scan matching
   For each particle:
     - Cast laser rays from particle position
     - Compare expected vs actual laser measurements
     - Compute likelihood (how well does scan match map?)
     - Assign weight to particle

5. Resample: Keep good particles, discard bad ones
   - Particles in wrong locations get low weights → eliminated
   - Particles near actual position get high weights → duplicated

6. Convergence detection:
   - If particles clustered in small area → converged
   - Reduce particle count back to normal
   - Switch to tracking mode
```

**Key Insight**:
- Wrong locations eliminated quickly (scan doesn't match map)
- Correct location reinforced (scan matches map perfectly)
- System "searches" entire map in parallel using particles

---

## 🧪 Experiment: Test Global Localization

### Experiment Setup

**Goal**: Prove AMCL can find robot from random locations

**Procedure**:
1. Enable global localization (Option 1 or 2 above)
2. Launch navigation system
3. Use Gazebo to teleport robot to 5 random locations
4. For each location:
   - Trigger global localization service
   - Time how long convergence takes
   - Verify robot localizes correctly
   - Test navigation goal after localization

**Success Criteria**:
- ✅ Particles spread across entire map initially
- ✅ Convergence within 30 seconds
- ✅ Final position matches actual robot location (within 10cm)
- ✅ Can navigate to goal after localization

---

## 🎓 Example: Manual Global Localization Test

```bash
# Terminal 1: Launch with standard config
cd ~/bin-boy
source install/setup.bash
bash launch_localization.sh map_20251028_203407

# Wait for launch to complete (~20 seconds)

# Terminal 2: Monitor particle cloud
ros2 topic echo /particle_cloud --field poses[0].position
# Should show particles clustered near (0,0,0)

# In Gazebo GUI:
# - Select robot
# - Press 'T' for translate mode
# - Drag robot to (3, 3, 0) - far from origin
# - Robot is now "kidnapped"

# Terminal 2: AMCL still thinks robot is at (0,0,0)
ros2 topic echo /amcl_pose --field pose.pose.position --once
# Will show position near (0,0,0) - WRONG!

# Terminal 2: Trigger global localization
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty

# Terminal 2: Watch particles spread
ros2 topic echo /particle_cloud --field poses[0].position
# Should see particles across entire map now

# Wait 20 seconds...

# Terminal 2: Check new position
ros2 topic echo /amcl_pose --field pose.pose.position --once
# Should now show position near (3,3,0) - CORRECT!

# In RViz: Send navigation goal
# Robot should navigate correctly from its actual position
```

---

## 📋 Quick Reference

### Commands

```bash
# Trigger global localization (one-time)
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty

# Check if localized
ros2 topic echo /amcl_pose --field pose.covariance[0] --once
# Value < 0.1 = good localization
# Value > 1.0 = poor/no localization

# Monitor particle spread
ros2 topic echo /particle_cloud --field poses | head -20

# Check AMCL parameters
ros2 param list /amcl
ros2 param get /amcl max_particles
ros2 param get /amcl min_particles
```

---

## 🎯 Recommendation for Your System

### Current Use Case: Simulation with Known Start
**Recommendation**: Keep current configuration
- `set_initial_pose: true` (auto-init at 0,0,0)
- Standard particle count (500-2000)
- Use global localization service **only when needed**

### If Testing Global Localization:
**Recommendation**: Use service-based approach (Option 1)
- No config changes needed
- Call service when robot is moved
- Normal performance rest of the time

### Future Real Robot Deployment:
**Recommendation**: Consider permanent global localization
- Real robots rarely start at perfect (0,0,0)
- More robust to position uncertainty
- Worth the CPU cost for reliability

---

## 📚 Further Reading

- [AMCL Package Documentation](http://wiki.ros.org/amcl)
- [Probabilistic Robotics Book](http://www.probabilistic-robotics.org/) - Chapter 8: Mobile Robot Localization
- [Particle Filters Explained](https://en.wikipedia.org/wiki/Particle_filter)
- [Nav2 AMCL Configuration](https://navigation.ros.org/configuration/packages/configuring-amcl.html)

---

**Summary**: AMCL CAN localize from unknown positions using global localization mode, but requires higher particle count and takes 10-30 seconds to converge. The service-based approach (`/reinitialize_global_localization`) is recommended for occasional use without performance penalty.
