#!/bin/bash
cd /mnt/c/Users/22240/rc2026_snapshot/rc
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting uphill_state_node..."
ros2 run uphill uphill_state_node &
UPHILL_PID=$!

sleep 2

echo "Starting fence_locator..."
ros2 run fence_locator fence_locator &
FENCE_PID=$!

sleep 3

echo "Starting bag play..."
ros2 bag play /mnt/c/Users/22240/rc2026_snapshot/bag/cha1nav2_20260523_133237 --clock --read-ahead-queue-size 1000 &
BAG_PID=$!

wait $BAG_PID
echo "Bag finished, killing nodes..."
kill $UPHILL_PID $FENCE_PID 2>/dev/null
wait $UPHILL_PID $FENCE_PID 2>/dev/null
echo "=== Done ==="
