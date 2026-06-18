#!/usr/bin/env python3
"""同时跑 launch 和对比分析，一步到位。"""

import subprocess
import time
import sys
import os

# 启动 launch
proc = subprocess.Popen(
    ["ros2", "launch", "fence_locator", "launch_fence.launch.py"],
    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    cwd="/mnt/c/Users/22240/rc2026_snapshot/rc"
)

# 等待启动
time.sleep(5)

# 启动对比脚本
comp = subprocess.Popen(
    [sys.executable, "/mnt/c/Users/22240/rc2026_snapshot/compare_marker_cloud.py"],
    stdout=subprocess.PIPE, stderr=subprocess.STDOUT
)

# 等待 bag 播放完
time.sleep(50)

# 收集输出
comp.terminate()
proc.terminate()
out = comp.stdout.read().decode()
print(out)
