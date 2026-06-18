#!/bin/bash
cd /mnt/c/Users/22240/rc2026_snapshot/rc
source install/setup.bash
export ROS_DOMAIN_ID=0

# Kill any existing ROS nodes
pkill -f "ros2 run uphill" 2>/dev/null
pkill -f "ros2 run fence_locator" 2>/dev/null
pkill -f "ros2 bag play" 2>/dev/null
sleep 1

# Start bag
ros2 bag play /mnt/c/Users/22240/rc2026_snapshot/bag/cha1nav2_20260612_154209 -r 0.5 --clock 100 2>/dev/null &
BAG_PID=$!
sleep 2

# Start uphill
ros2 run uphill uphill_state_node --ros-args -p use_sim_time:=true 2>&1 | grep -E "baseline|flat.*ground|ground.*lock|transition|state" &
UPHILL_PID=$!
sleep 1

# Start fence_locator
ros2 run fence_locator fence_locator --ros-args -p use_sim_time:=true -p publish_low_confidence:=true 2>&1 | grep -E "ground|locked|flat|cloud|frame|center" &
FENCE_PID=$!

# Wait for bag to finish
wait $BAG_PID 2>/dev/null
sleep 2

# Kill nodes
kill $UPHILL_PID $FENCE_PID 2>/dev/null
pkill -f "ros2 run uphill" 2>/dev/null
pkill -f "ros2 run fence_locator" 2>/dev/null
