#!/bin/bash
# Initialize AMCL Localization
# This script sends an initial pose estimate to AMCL to start localization

echo "========================================"
echo "  Initialize AMCL Localization"
echo "========================================"
echo ""

cd ~/bin-boy
source install/setup.bash

# Check if AMCL is running
if ! ros2 node list | grep -q "/amcl"; then
    echo "ERROR: AMCL node is not running!"
    echo "Please launch navigation first:"
    echo "  bash launch_localization.sh map_20251028_203407"
    exit 1
fi

echo "Sending initial pose estimate to AMCL..."
echo "Position: (0, 0, 0)"
echo "Orientation: 0 degrees"
echo ""

# Send initial pose to AMCL
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'map'
  },
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.068]
  }
}" >/dev/null 2>&1

echo "✓ Initial pose sent"
echo ""
echo "Waiting for AMCL to initialize (5 seconds)..."
sleep 5

# Check if AMCL is publishing
if timeout 2 ros2 topic hz /amcl_pose >/dev/null 2>&1; then
    echo "✓ AMCL is now publishing pose estimates"
    echo ""
    echo "System is ready for navigation!"
    echo "You can now:"
    echo "  1. Send navigation goals in RViz (2D Nav Goal)"
    echo "  2. Run waypoint navigation:"
    echo "     ros2 run bin_boy_navigation waypoint_sender config/waypoints/patrol_square.yaml"
    echo "  3. Run benchmark:"
    echo "     bash scripts/nav_benchmark.sh"
else
    echo "⚠ AMCL may not be publishing yet"
    echo "This could be normal if laser scans aren't available yet"
    echo ""
    echo "Try setting initial pose manually in RViz:"
    echo "  1. Click '2D Pose Estimate' button"
    echo "  2. Click on map at robot location"
    echo "  3. Drag to set orientation"
fi
