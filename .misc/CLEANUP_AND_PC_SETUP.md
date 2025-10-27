# PC Setup Guide

## Running Simulation on Your PC

### Package Requirements by System

| Package | Jetson (Hardware) | PC (Simulation Only) |
|---------|-------------------|----------------------|
| **bin_boy_description** | ✅ Required | ✅ Required |
| **bin_boy_simulation** | ✅ Required | ✅ Required |
| **bin_boy_perception** | ✅ Required | ⚠️ Optional* |
| **bin_boy_control** | ✅ Required | ❌ Not needed |
| **ldlidar_sl_ros2** | ✅ Required | ❌ Not needed |
| **csi_camera_cpp** | ✅ Required | ❌ Not needed |

*bin_boy_perception can be used with simulated camera for testing

### PC Setup Instructions

#### Step 1: Install ROS2

**Ubuntu 22.04 (Recommended):**
```bash
# Install ROS2 Humble (newer, better Gazebo support)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop-full
```

**Ubuntu 20.04 (If needed):**
```bash
# Install ROS2 Foxy (same as Jetson)
sudo apt update
sudo apt install ros-foxy-desktop
```

#### Step 2: Install Gazebo and ROS2 Integration

**For Humble:**
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-xacro
```

**For Foxy:**
```bash
sudo apt install ros-foxy-gazebo-ros-pkgs
sudo apt install ros-foxy-robot-state-publisher
sudo apt install ros-foxy-joint-state-publisher
sudo apt install ros-foxy-xacro
```

#### Step 3: Clone Your Repository

On your PC:

```bash
# Create workspace
mkdir -p ~/bin-boy-sim/src
cd ~/bin-boy-sim/src

# Option 1: Copy from Jetson (if you have git repo)
git clone <your-repo-url>

# Option 2: Manual copy from Jetson
# On Jetson, create a tar of needed packages:
cd ~/bin-boy/src
tar -czf ~/bin-boy-sim.tar.gz bin_boy_description bin_boy_simulation bin_boy_perception

# Transfer to PC (USB, scp, etc.)
# On PC:
cd ~/bin-boy-sim/src
tar -xzf ~/bin-boy-sim.tar.gz
```

**Minimal PC Setup (Simulation Only):**

If you only want simulation, copy just these:
```bash
# On Jetson
cd ~/bin-boy/src
tar -czf ~/bin-boy-minimal.tar.gz bin_boy_description bin_boy_simulation

# Transfer and extract on PC
```

#### Step 4: Install Dependencies (PC)

```bash
cd ~/bin-boy-sim
source /opt/ros/humble/setup.bash  # or foxy

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

#### Step 5: Build Workspace (PC)

```bash
cd ~/bin-boy-sim
colcon build --packages-select bin_boy_description bin_boy_simulation
source install/setup.bash
```

#### Step 6: Launch Simulation (PC)

```bash
source install/setup.bash
ros2 launch bin_boy_simulation simulation.launch.py
```

**Should launch much faster on PC with better GPU!**

---

## Performance Comparison

| System | Typical Performance |
|--------|---------------------|
| **Jetson Nano** | 5-15 FPS in Gazebo, high CPU usage |
| **PC (mid-range)** | 30-60 FPS, smooth |
| **PC (high-end)** | 60+ FPS, real-time |

---

## Synchronizing Jetson ↔ PC

### Quick Sync Script (Create on Jetson)

```bash
#!/bin/bash
# ~/bin-boy/sync_to_pc.sh

PACKAGES="bin_boy_description bin_boy_simulation"
PC_USER="your-username"
PC_IP="192.168.1.xxx"

cd ~/bin-boy/src
tar -czf /tmp/bin-boy-sync.tar.gz $PACKAGES

scp /tmp/bin-boy-sync.tar.gz $PC_USER@$PC_IP:/tmp/
ssh $PC_USER@$PC_IP "cd ~/bin-boy-sim/src && tar -xzf /tmp/bin-boy-sync.tar.gz"
```

Make executable:
```bash
chmod +x ~/bin-boy/sync_to_pc.sh
```

Usage:
```bash
# After making changes on Jetson
~/bin-boy/sync_to_pc.sh

# Then on PC
cd ~/bin-boy-sim
colcon build --packages-select bin_boy_simulation
source install/setup.bash
```

---

## Git Repository Structure (Recommended)

```
bin-boy/
├── .gitignore               # Ignore build/, install/, log/
├── README.md
├── SIMULATION_TESTING.md
├── FINAL_SOLUTION.md
│
└── src/
    ├── bin_boy_description/     # Commit - needed everywhere
    ├── bin_boy_simulation/      # Commit - simulation only
    ├── bin_boy_perception/      # Commit - optional for PC
    ├── bin_boy_control/         # Commit - Jetson only (hardware drivers)
    ├── ldlidar_sl_ros2/         # DON'T commit - external package
    └── csi_camera_cpp/          # DON'T commit - external package
```

### .gitignore

```
# Build artifacts
build/
install/
log/

# IDE
.vscode/
.idea/

# Temp files
*.pyc
__pycache__/
*~
*.swp

# OS
.DS_Store
Thumbs.db
```

---

## Testing on PC

### 1. Basic Simulation Launch
```bash
ros2 launch bin_boy_simulation simulation.launch.py
```

### 2. Motion Tests
```bash
# In new terminal
source ~/bin-boy-sim/install/setup.bash
python3 $(ros2 pkg prefix bin_boy_simulation)/share/bin_boy_simulation/scripts/test_motion.py
```

### 3. Sensor Validation
```bash
python3 $(ros2 pkg prefix bin_boy_simulation)/share/bin_boy_simulation/scripts/test_sensors.py
```

---

## Differences: Jetson vs PC

### On Jetson (Full System)
```bash
# Real hardware drivers + simulation
ros2 launch ldlidar_sl_ros2 ld14p.launch.py              # Real LIDAR
ros2 launch csi_camera_cpp csi_camera_ipc.launch.py      # Real camera
ros2 launch bin_boy_control kiwi_drive.launch.py         # Real motors
ros2 run bin_boy_control mpu6050_node                    # Real IMU
ros2 run bin_boy_perception person_tracker               # Person tracking
```

### On PC (Simulation Only)
```bash
# Single launch - everything simulated
ros2 launch bin_boy_simulation simulation.launch.py
# Includes: Gazebo LIDAR, camera, IMU, motion control
```

**Result:** Much simpler on PC, no hardware dependencies!

---

## Optional: Install gazebo-plugins (PC)

For better Gazebo functionality:

**Humble:**
```bash
sudo apt install ros-humble-gazebo-plugins
```

**Foxy:**
```bash
sudo apt install ros-foxy-gazebo-plugins
```

This adds more sensor and actuator plugins (though planar_move already works).

---

## Workflow Recommendation

### Development Cycle:
1. **Develop algorithms on PC** (fast simulation)
2. **Test in Gazebo** (validate logic)
3. **Sync to Jetson** (transfer code)
4. **Test on real hardware** (when motors arrive)
5. **Tune parameters** (based on real world)
6. **Update PC simulation** (with tuned parameters)

### What to Develop Where:

| Task | PC | Jetson |
|------|-----|--------|
| Navigation algorithms | ✅ Fast | ❌ Slow |
| Path planning | ✅ Fast | ❌ Slow |
| SLAM tuning (slam_toolbox) | ✅ Fast | ❌ Slow |
| Sensor fusion config | ✅ Fast | ⚠️ Can do |
| Person tracking logic | ✅ Fast | ⚠️ Can do |
| Motor PID tuning | ❌ No hardware | ✅ Real hardware |
| IMU calibration | ❌ No hardware | ✅ Real hardware |
| Real-world testing | ❌ No hardware | ✅ Real hardware |

---

## Troubleshooting PC Setup

### Gazebo Won't Launch
```bash
# Check Gazebo install
which gzserver
which gzclient

# If missing
sudo apt install gazebo11  # or gazebo (latest)
```

### Missing Plugins
```bash
# Check installed packages
ros2 pkg list | grep gazebo

# Install missing
sudo apt install ros-humble-gazebo-ros-pkgs
```

### RViz2 Errors
```bash
# Install if missing
sudo apt install ros-humble-rviz2
```

---

## Summary: What to Keep/Remove

### Keep on Jetson:
- ✅ All packages (you need real hardware drivers)
- ✅ SIMULATION_TESTING.md
- ✅ FINAL_SOLUTION.md
- ❌ Remove: Unused bridge nodes (gazebo_velocity_bridge.py, etc.)
- ❌ Remove: Debug docs (BUGFIX_SUMMARY.md, SIMULATION_FIX.md, etc.)

### For PC (Minimal):
- ✅ bin_boy_description
- ✅ bin_boy_simulation
- ⚠️ bin_boy_perception (optional)
- ❌ bin_boy_control (no hardware)
- ❌ ldlidar_sl_ros2 (no hardware)
- ❌ csi_camera_cpp (no hardware)

---

**Next Steps:**
1. Clean up unused files on Jetson
2. Set up PC workspace
3. Test simulation on PC
4. Enjoy faster development!

Sir, your PC will handle simulation **much** better than the Jetson Nano!
