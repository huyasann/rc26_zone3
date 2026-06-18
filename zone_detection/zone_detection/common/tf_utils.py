"""TF / 角度工具函数。

四元数 ↔ RPY, 角度归一化, 平均 yaw, yaw → 四元数。
纯数学函数，不依赖 ROS2 运行时。
"""

import math
from typing import List, Optional, Tuple

import numpy as np
from geometry_msgs.msg import Quaternion


def quat_to_rpy(q) -> Tuple[float, float, float]:
    """四元数 → roll/pitch/yaw."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                      1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quat_yaw(q) -> float:
    """从四元数提取 yaw (等价 quat_to_rpy[2])."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """yaw → 四元数 (x, y, z, w). 仅绕 Z 旋转."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def norm_angle(angle: float) -> float:
    """归一化到 [-π, π)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def mean_yaw(yaws: List[float]) -> float:
    """多个 yaw 的平均值 (处理 ±π 跳变)."""
    s = float(np.mean([math.sin(y) for y in yaws]))
    c = float(np.mean([math.cos(y) for y in yaws]))
    return math.atan2(s, c)


def blend_yaw(old: float, new: float, alpha: float) -> float:
    """平滑融合两个 yaw: old + alpha * (new - old)."""
    delta = norm_angle(new - old)
    return norm_angle(old + alpha * delta)
