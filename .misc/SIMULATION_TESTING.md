# Simulation Testing Guide for bin_boy Robot

This guide explains how to test your bin_boy robot in Gazebo simulation before deploying to real hardware.

## Overview

The simulation environment provides a complete virtual testbed with:
- Full robot model with kiwi drive kinematics
- Simulated sensors (LIDAR, camera, IMU)
- Indoor test environment with obstacles
- Person models for tracking tests
- Sensor fusion with robot_localization
- Comprehensive test scripts

## Quick Start

### 1. Build the Workspace

```bash
cd ~/bin-boy
colcon build --packages-select bin_boy_simulation bin_boy_control bin_boy_perception
source install/setup.bash
```

### 2. Launch Basic Simulation

```bash
# Launch Gazebo with robot and sensors
ros2 launch bin_boy_simulation simulation.launch.py
```

This opens:
- Gazebo simulation with test world
- RViz2 for visualization
- Robot with all sensors active

### 3. Test Robot Motion

In a new terminal:
```bash
source ~/bin-boy/install/setup.bash

# Manual control test
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}"

# Or run automated motion tests
python3 $(ros2 pkg prefix bin_boy_simulation)/share/bin_boy_simulation/scripts/test_motion.py
```

### 4. Verify Sensors

```bash
# Check all sensors are publishing
python3 $(ros2 pkg prefix bin_boy_simulation)/share/bin_boy_simulation/scripts/test_sensors.py
```

## Package Structure

```
bin-boy/
├── src/
│   ├── bin_boy_simulation/          # NEW: Simulation package
│   │   ├── launch/
│   │   │   ├── simulation.launch.py          # Basic Gazebo simulation
│   │   │   └── full_system_sim.launch.py     # Complete system with localization
│   │   ├── worlds/
│   │   │   └── test_indoor.world             # 10x10m indoor environment
│   │   ├── urdf/
│   │   │   └── bin_boy_gazebo.urdf.xacro     # Robot model with Gazebo plugins
│   │   └── scripts/
│   │       ├── test_motion.py                # Motion control tests
│   │       ├── test_sensors.py               # Sensor validation
│   │       └── run_integration_tests.sh      # Full test suite
│   │
│   ├── bin_boy_control/             # Motor control (existing)
│   │   ├── bin_boy_control/
│   │   │   ├── kiwi_drive_node.py            # Control node (hardware)
│   │   │   ├── sts3215_driver.py             # Servo driver (hardware)
│   │   │   └── mpu6050_node.py               # NEW: IMU driver
│   │   └── config/
│   │       └── ekf.yaml                      # NEW: Sensor fusion config
│   │
│   ├── bin_boy_perception/          # NEW: Perception package
│   │   └── bin_boy_perception/
│   │       └── person_tracker.py             # Person detection/tracking
│   │
│   ├── bin_boy_description/         # Robot URDF (existing)
│   └── ldlidar_sl_ros2/             # LIDAR driver (existing)
```

## What's New

### Simulation Infrastructure
1. **bin_boy_simulation** package with Gazebo integration
2. **Sensor plugins** for LIDAR, camera, IMU in simulation
3. **Kiwi drive controller** using planar_move plugin
4. **Test world** with walls, obstacles, and person models

### Real Hardware Drivers
1. **MPU-6050 IMU driver** (bin_boy_control/mpu6050_node.py)
2. **robot_localization config** for sensor fusion (odometry + IMU)

### Perception
1. **Person tracking node** (bin_boy_perception/person_tracker.py)
   - Integrates with YOLOv5 detection
   - Estimates person distance and angle
   - Optional following behavior

### Test Scripts
1. **test_motion.py** - Automated motion testing (8 test cases)
2. **test_sensors.py** - Sensor validation and monitoring
3. **run_integration_tests.sh** - Complete integration test suite

## Detailed Usage

### Launch Options

#### Basic Simulation
```bash
ros2 launch bin_boy_simulation simulation.launch.py
```
Launches: Gazebo, robot, sensors, RViz2

#### Without GUI (headless)
```bash
ros2 launch bin_boy_simulation simulation.launch.py gui:=false rviz:=false
```

#### Full System with Localization
```bash
ros2 launch bin_boy_simulation full_system_sim.launch.py
```
Launches: Gazebo + robot_localization EKF (fuses odometry + IMU)

### Motion Testing

The motion test script validates all kiwi drive capabilities:

```bash
python3 $(ros2 pkg prefix bin_boy_simulation)/share/bin_boy_simulation/scripts/test_motion.py
```

**Tests performed:**
1. Forward motion (0.3 m/s, 3s)
2. Backward motion (-0.3 m/s, 3s)
3. Strafe left (vy=0.3 m/s, 3s)  ← Kiwi drive unique capability
4. Strafe right (vy=-0.3 m/s, 3s)
5. Rotate CCW (0.5 rad/s, 3s)
6. Rotate CW (-0.5 rad/s, 3s)
7. Diagonal motion (vx=0.2, vy=0.2, 3s)
8. Combined motion (vx=0.2, omega=0.3, 3s)

### Sensor Validation

Monitor all sensors in real-time:

```bash
python3 $(ros2 pkg prefix bin_boy_simulation)/share/bin_boy_simulation/scripts/test_sensors.py
```

**Sensors checked:**
- LIDAR (/scan) - 450 points at 10Hz
- Camera (/camera/image_raw) - 640x480 at 30Hz
- Camera Info (/camera/camera_info)
- IMU (/imu/data) - 100Hz
- Odometry (/odom) - 50Hz

### Integration Testing

Run complete test suite:

```bash
cd ~/bin-boy/src/bin_boy_simulation/scripts
./run_integration_tests.sh
```

**Tests performed:**
- Package build verification
- Launch file existence
- URDF processing
- World file validation
- Configuration file checks
- Script availability

## Testing Workflows

### Workflow 1: Basic Motion Validation

1. Launch simulation:
   ```bash
   ros2 launch bin_boy_simulation simulation.launch.py
   ```

2. Run motion tests:
   ```bash
   python3 $(ros2 pkg prefix bin_boy_simulation)/share/bin_boy_simulation/scripts/test_motion.py
   ```

3. Observe in Gazebo and RViz:
   - Robot moves in all directions (including strafe)
   - Wheels rotate correctly
   - Odometry updates in RViz

### Workflow 2: Sensor Integration

1. Launch full system:
   ```bash
   ros2 launch bin_boy_simulation full_system_sim.launch.py
   ```

2. Verify sensors:
   ```bash
   python3 $(ros2 pkg prefix bin_boy_simulation)/share/bin_boy_simulation/scripts/test_sensors.py
   ```

3. Check topics:
   ```bash
   ros2 topic list
   ros2 topic echo /scan --once
   ros2 topic echo /imu/data --once
   ros2 topic echo /odom --once
   ```

### Workflow 3: Localization Testing

1. Launch with localization:
   ```bash
   ros2 launch bin_boy_simulation full_system_sim.launch.py
   ```

2. Drive robot around:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}" -r 10
   ```

3. Monitor fused odometry:
   ```bash
   ros2 topic echo /odometry/filtered
   ```

4. Compare raw vs filtered:
   - Raw odometry: /odom (from wheel encoders)
   - Filtered: /odometry/filtered (fused with IMU)

### Workflow 4: Person Tracking (Simulation Only)

Note: Person tracking in simulation requires manual trigger since YOLOv5 won't detect simple cylinder models. For real testing, use real hardware with camera.

1. Launch simulation
2. Launch person tracker:
   ```bash
   ros2 run bin_boy_perception person_tracker
   ```

3. Manually publish detection (simulating YOLOv5):
   ```bash
   ros2 topic pub /camera/detections std_msgs/String \
     "data: '[{\"class\":\"person\",\"confidence\":0.9,\"bbox\":[200,100,400,400]}]'"
   ```

## Sensor Specifications

### Simulated Sensors

| Sensor | Topic | Rate | Details |
|--------|-------|------|---------|
| LIDAR (LD14P) | /scan | 10Hz | 450 points, 360°, 0.02-12m range |
| Camera | /camera/image_raw | 30Hz | 640x480, 120° FOV |
| IMU (MPU-6050) | /imu/data | 100Hz | Accel + Gyro, no magnetometer |
| Odometry | /odom | 50Hz | Dead reckoning from wheel encoders |

### EKF Sensor Fusion

The Extended Kalman Filter (EKF) in robot_localization fuses:
- **Wheel odometry** (/odom): x, y, yaw position + velocities
- **IMU** (/imu/data): yaw rate, linear accelerations

Configuration: src/bin_boy_control/config/ekf.yaml

Output: /odometry/filtered

## Transitioning to Real Hardware

### Simulation vs Hardware Comparison

| Component | Simulation | Real Hardware |
|-----------|------------|---------------|
| Motion control | Planar move plugin | kiwi_drive_node + STS3215 servos |
| LIDAR | Gazebo ray sensor | ldlidar_sl_ros2 driver |
| Camera | Gazebo camera | csi_camera_cpp + YOLOv5 |
| IMU | Gazebo IMU sensor | mpu6050_node (I2C) |
| Odometry | Simulated perfect | Encoder-based with drift |

### Hardware Launch Commands

When motors arrive and hardware is connected:

```bash
# 1. LIDAR
ros2 launch ldlidar_sl_ros2 ld14p.launch.py

# 2. Motor control (in separate terminal)
ros2 launch bin_boy_control kiwi_drive.launch.py

# 3. IMU (in separate terminal)
ros2 run bin_boy_control mpu6050_node

# 4. Camera + YOLOv5 (in separate terminal)
ros2 launch csi_camera_cpp csi_camera_ipc.launch.py

# 5. Sensor fusion (in separate terminal)
ros2 run robot_localization ekf_node --ros-args --params-file src/bin_boy_control/config/ekf.yaml

# 6. Person tracking (in separate terminal)
ros2 run bin_boy_perception person_tracker --ros-args -p enable_following:=true
```

### Key Differences

1. **Odometry Quality**
   - Simulation: Perfect, no drift
   - Hardware: Encoder-based, accumulates error
   - Solution: EKF fusion with IMU improves accuracy

2. **Sensor Noise**
   - Simulation: Gaussian noise models
   - Hardware: Real-world noise + bias
   - Solution: Tuning EKF covariances (ekf.yaml)

3. **Control Response**
   - Simulation: Instantaneous
   - Hardware: Servo dynamics + mechanical lag
   - Solution: PID tuning in servo configuration

4. **Person Detection**
   - Simulation: Would need realistic human models
   - Hardware: YOLOv5 trained on real images
   - Solution: Test person tracking with real camera

## Troubleshooting

### YAML Parsing Error on Launch
**Error:** `TypeError: Unable to parse the value of parameter robot_description as yaml`

**Cause:** ROS2 Foxy tries to parse URDF content as YAML, fails on XML comments with colons.

**Solution:** Already fixed in simulation.launch.py using ParameterValue wrapper. If you see this error:
```bash
cd ~/bin-boy
colcon build --packages-select bin_boy_simulation
source install/setup.bash
```

See BUGFIX_SUMMARY.md for technical details.

### Simulation won't launch
```bash
# Check Gazebo installation
which gzserver

# Check package build
colcon build --packages-select bin_boy_simulation
source install/setup.bash
```

### Robot doesn't move
```bash
# Check cmd_vel is being published
ros2 topic echo /cmd_vel

# Check Gazebo plugin loaded
ros2 node list  # Should see /gazebo nodes

# Verify URDF processed correctly
ros2 run xacro xacro src/bin_boy_simulation/urdf/bin_boy_gazebo.urdf.xacro
```

### Sensors not publishing
```bash
# List all topics
ros2 topic list

# Check specific sensor
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
ros2 topic hz /imu/data

# Check Gazebo plugins in URDF
grep "plugin" src/bin_boy_simulation/urdf/bin_boy_gazebo.urdf.xacro
```

### EKF not fusing data
```bash
# Check EKF node is running
ros2 node list | grep ekf

# Monitor EKF diagnostics
ros2 topic echo /diagnostics

# Verify input topics
ros2 topic echo /odom --once
ros2 topic echo /imu/data --once

# Check EKF output
ros2 topic echo /odometry/filtered
```

## Next Steps

### Phase 2: Advanced Simulation Testing
1. Add dynamic obstacles
2. Implement SLAM (slam_toolbox)
3. Add Nav2 for autonomous navigation
4. Test person-following behavior

### Phase 3: Hardware Integration
1. Install IMU (MPU-6050) on robot
2. Mount motors and test servo control
3. Calibrate wheel odometry
4. Tune EKF parameters with real sensor data

### Phase 4: Autonomous Behavior
1. SLAM mapping of environment
2. Person detection and tracking
3. Obstacle avoidance
4. Autonomous following behavior

## Additional Resources

- ROS2 Humble Documentation: https://docs.ros.org/en/humble/
- Gazebo Documentation: http://gazebosim.org/docs
- robot_localization: http://docs.ros.org/en/humble/p/robot_localization/
- SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox

## Support

For issues or questions:
1. Check AUTONOMOUS_TRASH_CAN_SETUP.md for hardware setup
2. Check QUICK_START.md for quick commands
3. Run integration tests: `./src/bin_boy_simulation/scripts/run_integration_tests.sh`
4. Review logs: `ros2 run bin_boy_simulation <node_name>` with `--ros-args --log-level debug`
