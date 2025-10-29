# Global Localization Test - Complete Diagnosis & Fix

## Summary

**Status**: ✅ SYSTEM IS WORKING - Global localization is now ready to test

### What Was Fixed

**Issue #1: QoS Mismatch** ✅ FIXED
- **Problem**: Gazebo laser plugin was publishing with `RELIABLE` QoS, but AMCL subscribes with `BEST_EFFORT` QoS
- **Impact**: AMCL couldn't receive laser scans → No localization → No particles
- **Fix**: Modified `src/bin_boy_simulation/urdf/bin_boy_gazebo.urdf.xacro` to use `best_effort` QoS
- **File Changed**: `src/bin_boy_simulation/urdf/bin_boy_gazebo.urdf.xacro` (lines 84-98)
- **Verification**: Confirmed all laser publishers/subscribers now use BEST_EFFORT

## Current System Status

✅ Map server active and publishing map
✅ AMCL active and localized at (0,0,0)
✅ Laser scans publishing with correct QoS
✅ Robot model displaying in RViz
✅ TF tree complete
✅ Particles being published to `/particlecloud`

## How to Test Global Localization

### Step 1: Add Particle Cloud Visualization to RViz

**You need to add the ParticleCloud display to see the particles:**

1. In RViz, click **"Add"** button (bottom left of Displays panel)
2. Go to **"By topic"** tab
3. Find `/particlecloud` → **PoseArray**
4. Click **OK**

5. **Configure the display** (in left panel, expand PoseArray):
   - Set **Color**: `0; 255; 0` (green)
   - Set **Alpha**: `0.8`
   - Set **Arrow Length**: `0.3`
   - Set **Arrow Width**: `0.05`

**You should now see green arrows** clustered at your robot's position!

### Step 2: Test Global Localization

With the modified AMCL parameters (increased particles + recovery enabled):

1. **In Gazebo**: Use mouse to drag robot to a different location on the map

2. **Trigger global localization**:
   ```bash
   ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
   ```

3. **Watch in RViz**:
   - Green arrows should spread across the entire map
   - Within 10-30 seconds, particles converge on robot's actual location
   - All arrows cluster around where the robot actually is

### Step 3: Verify It Worked

**Success indicators:**
- Particle cloud spreads globally after service call
- Particles converge to robot's true position
- Robot can navigate correctly after convergence

## Modified AMCL Parameters (For Testing)

**Current settings** (in `src/bin_boy_navigation/config/amcl_params.yaml`):
```yaml
min_particles: 1000          # was: 500
max_particles: 5000          # was: 2000
recovery_alpha_slow: 0.001   # was: 0.0 (disabled)
recovery_alpha_fast: 0.1     # was: 0.0 (disabled)
```

**These settings enable global localization** but use more CPU.

## Reverting to Production Settings

**After testing**, restore the optimal simulation settings:

```bash
cd /home/mj/bin-boy
cp src/bin_boy_navigation/config/amcl_params.yaml.backup src/bin_boy_navigation/config/amcl_params.yaml
colcon build --packages-select bin_boy_navigation
source install/setup.bash
```

Then restart navigation. The QoS fix will remain, but recovery will be disabled for efficient simulation performance.

## Why This Happened

### Root Cause Analysis

**QoS Incompatibility:**
- ROS 2 requires matching QoS policies for publisher/subscriber communication
- Gazebo's default laser plugin uses `RELIABLE` reliability
- AMCL (from Nav2) expects `BEST_EFFORT` for sensor data
- Incompatible QoS = no data transfer = AMCL blind = no localization

**Why `BEST_EFFORT` for sensors?**
- Sensor data is time-sensitive
- Better to drop old data than wait for retransmission
- Localization algorithms handle occasional missing scans

### The Debugging Journey

1. **Initial symptoms**: No particles, TF errors, "multiple lidar maps"
2. **First suspect**: Clock not publishing (Gazebo paused) - but it was running
3. **Second suspect**: TF tree broken - but map frame existed
4. **Third suspect**: AMCL not activated - but lifecycle manager showed it was
5. **Root cause found**: QoS mismatch discovered via `ros2 topic info /scan --verbose`

**Diagnostic command that revealed it:**
```bash
ros2 topic info /scan --verbose
```

Showed:
- Publisher: RELIABLE
- Subscribers (AMCL, costmaps): BEST_EFFORT
- ❌ No data flowing

## Files Modified

### 1. `src/bin_boy_simulation/urdf/bin_boy_gazebo.urdf.xacro`
**Added QoS configuration to laser plugin:**
```xml
<plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_ray_sensor.so">
  <ros>
    <namespace>/</namespace>
    <remapping>~/out:=scan</remapping>
    <qos>
      <topic name="scan">
        <publisher>
          <reliability>best_effort</reliability>
        </publisher>
      </topic>
    </qos>
  </ros>
  <output_type>sensor_msgs/LaserScan</output_type>
  <frame_name>base_laser</frame_name>
</plugin>
```

### 2. `src/bin_boy_navigation/config/amcl_params.yaml`
**Temporarily modified for testing** (backup saved):
- Increased particle count: 500-2000 → 1000-5000
- Enabled recovery: 0.0 → 0.001/0.1

## Technical Details

### What is Global Localization?

**Normal operation:**
- Robot knows approximate position
- AMCL tracks from that position using particle filter
- Small particle cloud around robot

**Global localization:**
- Robot's position unknown (kidnapped robot problem)
- Particles spread across entire map
- Convergence takes 10-30 seconds as scan matching finds true position

### Recovery Parameters Explained

**`recovery_alpha_slow`** (Long-term filter)
- Tracks average localization quality over time
- When quality degrades gradually → adds random particles
- `0.001` = very conservative recovery

**`recovery_alpha_fast`** (Short-term filter)
- Tracks recent localization quality
- When quality drops suddenly → adds many random particles
- `0.1` = moderate response

**Both set to 0.0** = Recovery disabled = Global localization impossible

## Recommendations

### For Simulation (Your Current Use Case)
**Keep recovery DISABLED** (revert after testing):
- Robot always spawns at (0,0,0)
- Position is known
- Fast, efficient, low CPU usage
- Backup available at: `amcl_params.yaml.backup`

### For Physical Robot Testing
**Enable recovery**:
- Starting position unknown
- Need robustness to localization failures
- Higher CPU usage acceptable
- Use the current test configuration

## Next Steps

1. ✅ QoS fix is permanent - keep it
2. ⚠️ Test global localization with current settings
3. ✅ Revert AMCL parameters to production values after testing
4. ✅ Keep the QoS fix for future work

---

**Testing Status**: Ready for user validation
**Production Readiness**: Need to revert AMCL params after test
**Date**: 2025-10-29
