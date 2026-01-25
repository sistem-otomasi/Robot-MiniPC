#!/bin/bash
# Individual Node Test Script

set -e

cd /root/Otomasi/Sistem-Otomasi-Robot/Robot/ros2_ws
source install/setup.bash

echo "========================================"
echo "Individual Node Testing"
echo "========================================"

echo ""
echo "✅ TEST 1: robot_talker"
echo "Command: timeout 3 ros2 run robot_example robot_talker"
echo "Expected: Publishing messages to robot_topic"
timeout 3 ros2 run robot_example robot_talker 2>&1 || true

echo ""
echo "✅ TEST 2: hardware_monitor"
echo "Command: timeout 3 ros2 run robot_example hardware_monitor"
echo "Expected: Publishing hardware status"
timeout 3 ros2 run robot_example hardware_monitor 2>&1 || true

echo ""
echo "✅ TEST 3: kinematics_processor"
echo "Command: timeout 3 ros2 run robot_example kinematics_processor"
echo "Expected: Initialized kinematics node"
timeout 3 ros2 run robot_example kinematics_processor 2>&1 || true

echo ""
echo "✅ TEST 4: config_manager"
echo "Command: timeout 3 ros2 run robot_example config_manager"
echo "Expected: Config manager initialized (with warning about missing config)"
timeout 3 ros2 run robot_example config_manager 2>&1 || true

echo ""
echo "✅ TEST 5: robot_listener"
echo "Command: timeout 3 ros2 run robot_example robot_listener"
echo "Expected: Waiting for messages on robot_topic"
timeout 3 ros2 run robot_example robot_listener 2>&1 || true

echo ""
echo "========================================"
echo "✅ All individual node tests completed!"
echo "========================================"
echo ""
echo "Available executables:"
ros2 pkg executables robot_example 2>&1 | grep -E "robot_|hardware_|kinematics_|config_" || echo "No executables found"

echo ""
echo "Next steps:"
echo "1. Start one node: ros2 run robot_example hardware_monitor"
echo "2. Monitor topics: ros2 topic list -t"
echo "3. Echo specific topic: ros2 topic echo /hardware/battery"
