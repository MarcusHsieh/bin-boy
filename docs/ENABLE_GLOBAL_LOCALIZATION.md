# How to Enable Global Localization

**Current Status**: Global localization is DISABLED (recovery parameters = 0.0)

**Recommendation for Simulation**: Keep it disabled - not needed when robot always starts at known position (0,0,0)

---

## Why Service Call Didn't Spread Particles

When you ran:
```bash
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

The service call succeeded, but particles didn't spread because:
- `recovery_alpha_slow: 0.0` (disabled)
- `recovery_alpha_fast: 0.0` (disabled)

**These parameters must be > 0 for global localization to work.**

---

## Option 1: Keep Current Setup (RECOMMENDED)

**Advantages**:
- ✅ Fast startup (known position)
- ✅ Low CPU usage (500-2000 particles)
- ✅ Perfect for simulation (robot always at 0,0,0)
- ✅ Efficient tracking

**Use case**: Standard simulation testing

**What to do**: Nothing! Current config is optimal for your use case.

---

## Option 2: Enable Global Localization (For Testing)

**If you want to test "kidnapped robot" scenarios:**

### Step 1: Modify AMCL Parameters

Edit: `src/bin_boy_navigation/config/amcl_params.yaml`

Change lines 22-23 from:
```yaml
recovery_alpha_slow: 0.0
recovery_alpha_fast: 0.0
```

To:
```yaml
recovery_alpha_slow: 0.001  # Enable slow recovery
recovery_alpha_fast: 0.1    # Enable fast recovery
```

**Optionally increase particle count** (better global localization):
```yaml
min_particles: 1000   # was: 500
max_particles: 5000   # was: 2000
```

### Step 2: Rebuild Package

```bash
cd ~/bin-boy
colcon build --packages-select bin_boy_navigation
source install/setup.bash
```

### Step 3: Restart Navigation

```bash
# Kill current navigation
pkill -9 -f ros2

# Relaunch
bash launch_localization.sh map_20251028_203407
```

### Step 4: Test Global Localization

```bash
# In Gazebo: Move robot to different location (drag with mouse)

# Trigger global localization
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty

# Wait 20 seconds - particles should spread and converge
```

---

## How to Visualize Particles in RViz

To see particles spreading:

1. **Open RViz** (should already be open)

2. **Add Particle Display**:
   - Click **"Add"** button (bottom left)
   - Go to **"By topic"** tab
   - Find `/particlecloud` → **PoseArray**
   - Click **OK**

3. **Configure Display**:
   - In left panel, expand **PoseArray**
   - Set **Color**: 0;255;0 (green)
   - Set **Alpha**: 0.5
   - Set **Arrow Length**: 0.1
   - Set **Arrow Width**: 0.02

4. **What You'll See**:
   - **Before global localization**: Green arrows in one cluster
   - **After service call** (with recovery enabled): Arrows spread across map
   - **During convergence**: Arrows cluster around robot's actual position
   - **After convergence**: All arrows at robot location

---

## Comparison: Disabled vs Enabled

### Current Setup (Disabled - RECOMMENDED)
```yaml
recovery_alpha_slow: 0.0
recovery_alpha_fast: 0.0
min_particles: 500
max_particles: 2000
```

**Behavior**:
- Robot assumes it starts at (0,0,0)
- Particles stay clustered, track from initial position
- Service call does nothing (recovery disabled)
- **Perfect for simulation with known spawn point**

---

### Global Localization (Enabled - FOR TESTING)
```yaml
recovery_alpha_slow: 0.001
recovery_alpha_fast: 0.1
min_particles: 1000
max_particles: 5000
```

**Behavior**:
- Robot can find itself from any position
- Service call spreads particles across entire map
- Converges in 10-30 seconds
- **Useful for real robot or testing robustness**

**Trade-off**: Higher CPU usage, slower convergence

---

## Understanding Recovery Parameters

### `recovery_alpha_slow` (Long-term filter)
- Tracks average localization quality over long time
- When quality drops below threshold, adds random particles
- **0.0 = disabled**
- **0.001 = very conservative** (recommended)

### `recovery_alpha_fast` (Short-term filter)
- Tracks localization quality over recent measurements
- When quality drops suddenly, adds many random particles
- **0.0 = disabled**
- **0.1 = moderate** (recommended)

**How they work together**:
- If localization degrades gradually → slow filter triggers → adds few random particles
- If localization fails suddenly → fast filter triggers → adds many random particles
- If both = 0.0 → no recovery, no global localization possible

---

## Real-World Use Cases

### Simulation (Your Current Setup) ✅
**Config**: Recovery disabled, 500-2000 particles
**Why**: Robot always spawns at (0,0,0), position known
**Recommendation**: Keep current setup

### Lab Testing
**Config**: Recovery enabled, 2000-5000 particles
**Why**: Want to test robustness, may move robot manually
**Recommendation**: Enable if testing kidnapped robot scenarios

### Production Robot
**Config**: Recovery enabled, 5000-10000 particles
**Why**: Unknown starting position, need reliability
**Recommendation**: Enable for real deployments

---

## Quick Decision Guide

**Ask yourself**: "Does my robot always start at a known position?"

- **YES** (simulation, docking station, etc.)
  → Keep recovery **disabled** (current config)
  → Fast, efficient, no global localization needed

- **NO** (placed by human, multiple start points, etc.)
  → Enable recovery (modify config as shown above)
  → Slower but more robust

**Your situation**: Simulation with known spawn point
**My recommendation**: **Keep recovery disabled** (current is optimal)

---

## Testing Without Enabling Permanently

If you just want to test global localization **once** without changing config:

**Unfortunately**: Not possible with current parameter values
**Reason**: Recovery parameters are loaded at startup and can't be changed via service
**Solution**: Must modify config file and rebuild to test

**Alternative**: The service call works as a "reset" command with current config, but won't spread particles globally. It just resets the particle filter to initial state.

---

## Summary

### Current Status: ✅ Optimal for Your Use Case
- Recovery disabled (parameters = 0.0)
- Standard tracking mode
- Perfect for simulation

### To Enable Global Localization:
1. Set `recovery_alpha_slow: 0.001`
2. Set `recovery_alpha_fast: 0.1`
3. Optional: Increase `max_particles` to 5000
4. Rebuild and relaunch

### Recommendation: **Don't enable it** unless you need to test kidnapped robot scenarios

Your current setup is working perfectly for simulation! 🎯
