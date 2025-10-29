#!/bin/bash
# Navigation Benchmark Script
#
# Tests Nav2 navigation performance by sending a series of goals
# and measuring success rate, time to goal, and path efficiency.
#
# Usage:
#   bash scripts/nav_benchmark.sh
#
# Prerequisites:
#   - Navigation stack must be running (launch_localization.sh)
#   - Robot must be localized on the map
#   - ROS 2 workspace sourced

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
BENCHMARK_GOALS_FILE="${1:-config/waypoints/patrol_square.yaml}"
TIMEOUT_PER_GOAL=60  # seconds
RESULTS_DIR="benchmark_results"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RESULTS_FILE="$RESULTS_DIR/benchmark_$TIMESTAMP.log"

# Counters
TOTAL_GOALS=0
SUCCESSFUL_GOALS=0
FAILED_GOALS=0
TOTAL_TIME=0

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  BIN-BOY Navigation Benchmark${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Create results directory
mkdir -p "$RESULTS_DIR"

# Check if ROS is running
echo -e "${YELLOW}[CHECK]${NC} Verifying ROS 2 is running..."
if ! ros2 node list &>/dev/null; then
    echo -e "${RED}[ERROR]${NC} ROS 2 is not running!"
    echo "Please launch navigation first:"
    echo "  bash launch_localization.sh <map_name>"
    exit 1
fi
echo -e "${GREEN}[OK]${NC} ROS 2 is running"

# Check if controller_server is running
echo -e "${YELLOW}[CHECK]${NC} Verifying controller_server is running..."
if ! ros2 node list | grep -q "controller_server"; then
    echo -e "${RED}[ERROR]${NC} controller_server is not running!"
    echo "Please launch navigation first:"
    echo "  bash launch_localization.sh <map_name>"
    exit 1
fi
echo -e "${GREEN}[OK]${NC} controller_server is running"

# Check if planner_server is running
echo -e "${YELLOW}[CHECK]${NC} Verifying planner_server is running..."
if ! ros2 node list | grep -q "planner_server"; then
    echo -e "${RED}[ERROR]${NC} planner_server is not running!"
    exit 1
fi
echo -e "${GREEN}[OK]${NC} planner_server is running"

# Check if AMCL is running and localized
echo -e "${YELLOW}[CHECK]${NC} Verifying AMCL localization..."
if ! ros2 node list | grep -q "amcl"; then
    echo -e "${RED}[ERROR]${NC} AMCL is not running!"
    exit 1
fi

# Check if robot has valid pose
if ! ros2 topic echo /amcl_pose --once --spin-time 2 &>/dev/null; then
    echo -e "${RED}[ERROR]${NC} Robot is not localized!"
    echo "Please set initial pose using '2D Pose Estimate' in RViz"
    exit 1
fi
echo -e "${GREEN}[OK]${NC} Robot is localized"

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Benchmark Configuration${NC}"
echo -e "${BLUE}========================================${NC}"
echo "Goals file: $BENCHMARK_GOALS_FILE"
echo "Timeout per goal: ${TIMEOUT_PER_GOAL}s"
echo "Results file: $RESULTS_FILE"
echo ""

# Initialize results file
cat > "$RESULTS_FILE" <<EOF
BIN-BOY Navigation Benchmark Results
Timestamp: $TIMESTAMP
Goals file: $BENCHMARK_GOALS_FILE
Timeout: ${TIMEOUT_PER_GOAL}s

========================================
Goal Results:
========================================
EOF

# Read waypoints from YAML file
echo -e "${YELLOW}[INFO]${NC} Loading waypoints from $BENCHMARK_GOALS_FILE..."

# Extract waypoints using Python
WAYPOINTS=$(python3 -c "
import yaml
import sys

with open('$BENCHMARK_GOALS_FILE', 'r') as f:
    data = yaml.safe_load(f)

for i, wp in enumerate(data['waypoints']):
    print(f\"{i},{wp['x']},{wp['y']},{wp['theta']}\")
")

if [ -z "$WAYPOINTS" ]; then
    echo -e "${RED}[ERROR]${NC} Failed to load waypoints from file"
    exit 1
fi

TOTAL_GOALS=$(echo "$WAYPOINTS" | wc -l)
echo -e "${GREEN}[OK]${NC} Loaded $TOTAL_GOALS waypoints"
echo ""

# Function to send navigation goal
send_nav_goal() {
    local x=$1
    local y=$2
    local theta=$3
    local goal_num=$4

    # Calculate quaternion from theta (rotation around Z axis)
    local qz=$(python3 -c "import math; print(math.sin($theta / 2.0))")
    local qw=$(python3 -c "import math; print(math.cos($theta / 2.0))")

    echo -e "${YELLOW}[GOAL $goal_num/$TOTAL_GOALS]${NC} Navigating to ($x, $y, theta=$theta)"

    # Record start time
    local start_time=$(date +%s)

    # Send goal via action
    local result=$(timeout $TIMEOUT_PER_GOAL ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: 'map'
    stamp:
      sec: 0
      nanosec: 0
  pose:
    position:
      x: $x
      y: $y
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: $qz
      w: $qw
behavior_tree: ''
" 2>&1)

    # Record end time
    local end_time=$(date +%s)
    local duration=$((end_time - start_time))

    # Check if goal succeeded
    if echo "$result" | grep -q "Goal reached"; then
        echo -e "${GREEN}[SUCCESS]${NC} Goal $goal_num reached in ${duration}s"
        SUCCESSFUL_GOALS=$((SUCCESSFUL_GOALS + 1))
        TOTAL_TIME=$((TOTAL_TIME + duration))
        echo "Goal $goal_num: SUCCESS ($x, $y, $theta) - ${duration}s" >> "$RESULTS_FILE"
        return 0
    else
        echo -e "${RED}[FAILED]${NC} Goal $goal_num failed or timed out"
        FAILED_GOALS=$((FAILED_GOALS + 1))
        echo "Goal $goal_num: FAILED ($x, $y, $theta) - Timeout or error" >> "$RESULTS_FILE"
        return 1
    fi
}

# Run benchmark
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Running Benchmark${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

GOAL_NUM=1
while IFS=',' read -r idx x y theta; do
    send_nav_goal "$x" "$y" "$theta" "$GOAL_NUM"
    echo ""
    GOAL_NUM=$((GOAL_NUM + 1))

    # Brief pause between goals
    sleep 2
done <<< "$WAYPOINTS"

# Calculate statistics
SUCCESS_RATE=$(python3 -c "print(f'{($SUCCESSFUL_GOALS / $TOTAL_GOALS * 100):.1f}')")
AVG_TIME=$(python3 -c "print(f'{($TOTAL_TIME / max($SUCCESSFUL_GOALS, 1)):.1f}')")

# Display summary
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Benchmark Summary${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "Total goals:      ${TOTAL_GOALS}"
echo -e "${GREEN}Successful:       ${SUCCESSFUL_GOALS}${NC}"
echo -e "${RED}Failed:           ${FAILED_GOALS}${NC}"
echo -e "Success rate:     ${SUCCESS_RATE}%"
echo -e "Total time:       ${TOTAL_TIME}s"
echo -e "Avg time/goal:    ${AVG_TIME}s"
echo ""

# Append summary to results file
cat >> "$RESULTS_FILE" <<EOF

========================================
Summary:
========================================
Total goals:      $TOTAL_GOALS
Successful:       $SUCCESSFUL_GOALS
Failed:           $FAILED_GOALS
Success rate:     ${SUCCESS_RATE}%
Total time:       ${TOTAL_TIME}s
Avg time/goal:    ${AVG_TIME}s
EOF

echo -e "${BLUE}Results saved to: ${RESULTS_FILE}${NC}"

# Performance assessment
echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Performance Assessment${NC}"
echo -e "${BLUE}========================================${NC}"

if (( $(echo "$SUCCESS_RATE >= 95" | bc -l) )); then
    echo -e "${GREEN}[EXCELLENT]${NC} Success rate >= 95%"
elif (( $(echo "$SUCCESS_RATE >= 80" | bc -l) )); then
    echo -e "${YELLOW}[GOOD]${NC} Success rate >= 80%"
else
    echo -e "${RED}[NEEDS TUNING]${NC} Success rate < 80%"
    echo "Consider reviewing parameter tuning (see docs/PARAMETER_TUNING.md)"
fi

if (( $(echo "$AVG_TIME <= 20" | bc -l) )); then
    echo -e "${GREEN}[FAST]${NC} Average time <= 20s per goal"
elif (( $(echo "$AVG_TIME <= 40" | bc -l) )); then
    echo -e "${YELLOW}[MODERATE]${NC} Average time <= 40s per goal"
else
    echo -e "${RED}[SLOW]${NC} Average time > 40s per goal"
    echo "Consider increasing max_vel_x/y for faster navigation"
fi

echo ""
echo -e "${BLUE}Benchmark complete!${NC}"

exit 0
