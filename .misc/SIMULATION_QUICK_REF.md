# Simulation Quick Reference

## Essential Commands

### Launch Simulation
```bash
# Start here first
source ~/bin-boy/install/setup.bash
ros2 launch bin_boy_simulation simulation.launch.py
```

### Test Motion
```bash
# In new terminal (after simulation is running)
source ~/bin-boy/install/setup.bash
python3 $(ros2 pkg prefix bin_boy_simulation)/share/bin_boy_simulation/scripts/test_motion.py
```

### Test Sensors
```bash
python3 $(ros2 pkg prefix bin_boy_simulation)/share/bin_boy_simulation/scripts/test_sensors.py
```

### Manual Control
```bash
# Forward
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}" -r 10

# Strafe left (unique to kiwi drive!)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {y: 0.3}}" -r 10

# Rotate
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}" -r 10

# Stop (Ctrl+C the pub command)
```

## Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| /cmd_vel | Twist | Motion commands |
| /odom | Odometry | Raw wheel odometry |
| /odometry/filtered | Odometry | Fused (EKF) odometry |
| /scan | LaserScan | LIDAR data |
| /camera/image_raw | Image | Camera feed |
| /imu/data | Imu | IMU sensor data |
| /joint_states | JointState | Wheel positions |

## Monitoring

```bash
# List all topics
ros2 topic list

# Monitor a topic
ros2 topic echo /odom

# Check message rate
ros2 topic hz /scan

# View in RQT
rqt
```

## Troubleshooting

### Simulation won't start
```bash
cd ~/bin-boy
colcon build --packages-select bin_boy_simulation
source install/setup.bash
```

### Robot doesn't move
```bash
# Check cmd_vel is being received
ros2 topic echo /cmd_vel

# Manually test
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" -r 10
```

### Sensors not working
```bash
# Run sensor test
python3 $(ros2 pkg prefix bin_boy_simulation)/share/bin_boy_simulation/scripts/test_sensors.py
```

## Files to Know

- **Full guide:** SIMULATION_TESTING.md (400+ lines)
- **Summary:** IMPLEMENTATION_SUMMARY.md
- **Hardware setup:** AUTONOMOUS_TRASH_CAN_SETUP.md
- **This file:** SIMULATION_QUICK_REF.md

## When Motors Arrive

See SIMULATION_TESTING.md section "Transitioning to Real Hardware" for launch commands.
