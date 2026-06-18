#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

cores="$(nproc 2>/dev/null || echo 1)"
mem_gb="$(awk '/MemAvailable/ { printf "%d", $2 / 1024 / 1024 }' /proc/meminfo 2>/dev/null || echo 0)"
workers="$cores"
if [ "$mem_gb" -gt 0 ] && [ "$mem_gb" -lt "$workers" ]; then
  workers="$mem_gb"
fi
if [ "$workers" -lt 1 ]; then
  workers=1
fi

echo "==> CPU cores: $cores, Available memory: ${mem_gb}GB"
echo "==> Building with $workers parallel workers"
colcon build --symlink-install --parallel-workers "$workers"
source install/setup.bash
echo "==> Build completed!"
