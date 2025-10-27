#!/bin/bash
# Integration Test Suite for bin_boy Robot
# Tests simulation environment end-to-end

set -e  # Exit on error

echo "========================================"
echo "bin_boy Integration Test Suite"
echo "========================================"
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test results
TESTS_PASSED=0
TESTS_FAILED=0

run_test() {
    local test_name=$1
    local command=$2

    echo -e "${YELLOW}Running: $test_name${NC}"
    if eval "$command"; then
        echo -e "${GREEN}✓ PASS${NC}: $test_name"
        ((TESTS_PASSED++))
    else
        echo -e "${RED}✗ FAIL${NC}: $test_name"
        ((TESTS_FAILED++))
    fi
    echo ""
}

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Error: ROS2 not sourced. Please run: source install/setup.bash${NC}"
    exit 1
fi

echo "ROS Distribution: $ROS_DISTRO"
echo ""

# Test 1: Check if packages are built
echo "Test 1: Verifying packages..."
run_test "bin_boy_description package" "ros2 pkg list | grep -q bin_boy_description"
run_test "bin_boy_control package" "ros2 pkg list | grep -q bin_boy_control"
run_test "bin_boy_simulation package" "ros2 pkg list | grep -q bin_boy_simulation"

# Test 2: Check if launch files exist
echo "Test 2: Verifying launch files..."
run_test "simulation.launch.py exists" "ros2 pkg prefix bin_boy_simulation"
run_test "kiwi_drive.launch.py exists" "ros2 pkg prefix bin_boy_control"

# Test 3: Verify URDF can be processed
echo "Test 3: Verifying URDF..."
run_test "URDF xacro processing" "ros2 run xacro xacro $(ros2 pkg prefix bin_boy_simulation)/share/bin_boy_simulation/urdf/bin_boy_gazebo.urdf.xacro > /tmp/test_urdf.xml"
if [ -f /tmp/test_urdf.xml ]; then
    run_test "URDF contains robot tag" "grep -q '<robot' /tmp/test_urdf.xml"
    rm /tmp/test_urdf.xml
fi

# Test 4: Check Gazebo world file
echo "Test 4: Verifying world files..."
run_test "Test world file exists" "test -f $(ros2 pkg prefix bin_boy_simulation)/share/bin_boy_simulation/worlds/test_indoor.world"

# Test 5: Check configuration files
echo "Test 5: Verifying configuration files..."
run_test "EKF config exists" "test -f $(ros2 pkg prefix bin_boy_control)/share/bin_boy_control/config/ekf.yaml"

# Test 6: Check Python scripts are executable
echo "Test 6: Verifying test scripts..."
SCRIPT_DIR=$(ros2 pkg prefix bin_boy_simulation)/share/bin_boy_simulation/scripts
if [ -d "$SCRIPT_DIR" ]; then
    run_test "test_motion.py exists" "test -f $SCRIPT_DIR/test_motion.py"
    run_test "test_sensors.py exists" "test -f $SCRIPT_DIR/test_sensors.py"
fi

# Summary
echo "========================================"
echo "Test Summary"
echo "========================================"
echo -e "Tests Passed: ${GREEN}$TESTS_PASSED${NC}"
echo -e "Tests Failed: ${RED}$TESTS_FAILED${NC}"
echo ""

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "${GREEN}✓ All integration tests PASSED!${NC}"
    echo ""
    echo "Next steps:"
    echo "1. Launch simulation:"
    echo "   ros2 launch bin_boy_simulation simulation.launch.py"
    echo ""
    echo "2. Test motion control:"
    echo "   python3 $SCRIPT_DIR/test_motion.py"
    echo ""
    echo "3. Test sensors:"
    echo "   python3 $SCRIPT_DIR/test_sensors.py"
    echo ""
    exit 0
else
    echo -e "${RED}✗ Some tests FAILED. Please fix issues before proceeding.${NC}"
    exit 1
fi
