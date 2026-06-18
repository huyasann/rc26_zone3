#!/usr/bin/env python3
"""Zone3 上坡后重试区 marker + 九宫格 ROI 点云定位节点。

核心用途：
1. 根据机器人上坡过程中的 odom 轨迹拟合当前队伍的 zone3_root。
2. 在拟合出的 zone3_root 下发布重试区、坡道、九宫格 ROI 等 marker。
3. 从 /odin1/cloud_slam 中裁剪九宫格附近点云，统一发布到 /rc26/zone3/debugcloud。

九宫格 ROI 的真实坐标链：
1. /odin1/odometry_highfreq 提供机器人在 odom 下的上坡轨迹。
2. 脚本用坡道低点、坡顶点和坡道方向，在 odom 下锁定 zone3_root。
3. /odin1/cloud_slam 的点云先通过 TF 变换到 odom。
4. 每个点的 odom XY 再反算到已锁定的 zone3_root 局部坐标。
5. GRID_ROI_X/Y 在 zone3_root 局部坐标下裁剪，裁出来的点再以 odom 发布。

结论：
九宫格 ROI 不是 odin1_base_link 下的随车 ROI，而是锁在 odom 中的拟合场地 root 上。
车继续运动后，ROI 仍停在预测的九宫格区域，不会跟着车体移动。
"""

from __future__ import annotations

import math
import csv
import os
import time
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


# ---------- ROS 话题与日志路径 ----------
ODOM_TOPIC = "/odin1/odometry_highfreq"
# 输入：高频里程计，用来检测上坡事件并拟合坡道方向。
CLOUD_TOPIC = "/odin1/cloud_slam"
# 输入：cloud_slam 点云，锁定 root 后从这里裁剪九宫格 ROI。
MARKER_TOPIC = "/rc26/zone3/retry_probe_markers"
# 输出：所有调试 marker 统一发布到这一个 MarkerArray。
CLOUD_DEBUG_TOPIC = "/rc26/zone3/debugcloud"
# 输出：所有 ROI 调试点云统一发布到这一个 PointCloud2。
LOG_DIR = "/home/inkc/inkc/Rc2026/src/tmp/tmp/logs"
# 输出：CSV 详细日志目录，便于回看拟合过程。
DEFAULT_DETAIL_LOG_ROOT_DIR = "/home/inkc/inkc/Rc2026/files/record/logs"
# launch 传入 detail_log_root_dir 时优先使用；默认写入 files/record/logs。
DETAIL_LOG_HZ = 2.0
# 详细日志最高写入频率。未初锁阶段始终允许；初锁后只有位置+角度门控都通过才允许。
HEARTBEAT_LOG_HZ = 1.0
# 心跳日志写入频率。

# ---------- 队伍与场地几何参数：BLUE 数值为基准，RED 运行时按 root 局部 X 镜像 ----------
IS_BLUE_TEAM = True  # 默认蓝方；launch 可用 is_blue_team:=false 切红方。
BLUE_RAMP_X = 2.275  # 蓝方坡道中心线在 root 局部 X 方向的位置。
RAMP_LOW_Y = 1.30  # 坡道低端 root 局部 Y，用上坡起点反推 root。
RAMP_TOP_Y = -0.20  # 坡道高端 root 局部 Y，用坡顶点反推 root。
RAMP_LEFT_X = 1.50  # 坡道左边界 root 局部 X，只用于画坡道轮廓。
RAMP_RIGHT_X = 3.05  # 坡道右边界 root 局部 X，只用于画坡道轮廓。
Z3_PLATFORM_Z_REL = 0.40  # Z3 平台相对场地地面的理论高度。
BLUE_RETRY_REL = (2.50, -0.90, 0.4510)
# 蓝方重试区中心，root 局部坐标 (x, y, z)。
Z3_RETRY_SIZE = (1.0, 1.0, 0.001)
# 重试区理论尺寸 (长, 宽, 厚)，marker 显示时会把厚度加粗。
BLUE_GRID_CENTER_REL = (-3.025, -0.15, 0.40)
# 九宫格理论中心，root 局部坐标，仅作为参考 marker。
GRID_BASE_SIZE = (0.32, 1.62, 0.40)
# 九宫格基座尺寸，参考 field_publisher/rc26_field.py。
GRID_BASE_CENTER_Z_REL = Z3_PLATFORM_Z_REL + GRID_BASE_SIZE[2] * 0.5
# 九宫格基座中心相对 zone3_root 的 Z；rc26_field.py 中 surface=0.4, base_center=+0.2。
GRID_BLOCK_SIZE = (0.30, 0.54, 0.54)
# 九宫格单格尺寸，参考 field_publisher/rc26_field.py。
GRID_COL_YS_REL = (0.54, 0.0, -0.54)
# 九宫格三列相对模型中心的 Y 偏移。
GRID_LAYER_ZS_REL = (0.47, 1.01, 1.55)
# 九宫格三层相对基座中心的 Z 偏移。

# ---------- 九宫格点云 ROI：用于从 cloud_slam 中裁剪九宫格附近点云 ----------
GRID_ROI_X = (-3.70, -2.2)
# BLUE root 局部 X 范围；RED 会自动镜像，围绕镜像后的 grid_center_rel[0] 调。
GRID_ROI_Y = (-1.30, 1.00)
# root 局部 Y 范围；红蓝共用，围绕 grid_center_rel[1] 调。
GRID_ROI_Z = (0.00, 2.50)
# odom 全局 Z 范围；不是 base_link 高度，也不是平台地面归一化高度。

# ---------- 坡道 root 锁定阈值 ----------
RAMP_STRONG_MAX_RESIDUAL = 0.35
# 坡底/坡顶各自反推 root 的最大允许差值；强锁定阈值，低于它直接信任坡道。
RAMP_WEAK_MAX_RESIDUAL = 0.50
# 弱锁定阈值；低于它只认为“位置大概对”，仅放大 ROI 方便观察点云。

# ---------- 弱锁定时的九宫格 ROI 扩展 ----------
GRID_WEAK_ROI_MARGIN_X = 0.2
# 弱锁定时额外放大的 root 局部 X 边距，避免坡道粗 root 偏一点就裁掉九宫格。
GRID_WEAK_ROI_MARGIN_Y = 0.2
# 弱锁定时额外放大的 root 局部 Y 边距。

# ---------- 九宫格点云拟合：基础阈值 ----------
GRID_DETECT_MIN_H = 0.75
# ROI 点云中用于拟合九宫格模型的相对高度下限。
GRID_DETECT_MAX_H = 2.60
# ROI 点云中用于拟合九宫格模型的相对高度上限。
GRID_DETECT_MIN_POINTS = 80
# 高位点数量低于该值时不更新拟合模型。
GRID_DETECT_MIN_VERTICAL_SPAN = 0.80
# ROI 内实际点云垂直跨度低于该值时，不认为看到了九宫格结构。

# ---------- 九宫格点云拟合：模型局部范围 ----------
GRID_MODEL_FOOTPRINT_X = 0.60
# 拟合出中心/yaw 后，回到实际点云里找基座高度时使用的模型局部 X 半宽。
GRID_MODEL_FOOTPRINT_Y = 1.05
# 拟合出中心/yaw 后，回到实际点云里找基座高度时使用的模型局部 Y 半宽。
GRID_BASE_Z_CORE_X = 0.24
# 估计基座底面高度时使用的模型局部 X 半宽；比 FOOTPRINT_X 小，避开前后边缘离群点。
GRID_BASE_Z_CORE_Y = 0.72
# 估计基座底面高度时使用的模型局部 Y 半宽；只取基座主体中部。
GRID_BASE_Z_MIN_POINTS = 20
# 基座核心区域低位点少于该数量时，回退到原 footprint 估计。

# ---------- 九宫格点云拟合：前后方向离群点过滤 ----------
GRID_DEPTH_SUPPORT_FILTER_ENABLE = True
# 是否用横向支撑过滤九宫格前后方向离群点。
GRID_DEPTH_SUPPORT_BIN_SIZE = 0.06
# 前后方向分 bin 尺寸，单位 m。
GRID_DEPTH_SUPPORT_MIN_POINTS = 8
# 一个前后 bin 至少有多少点才可能参与模型深度估计。
GRID_DEPTH_SUPPORT_MIN_Y_SPAN = 0.55
# 一个前后 bin 内点云左右跨度至少多大，低于它通常是前方离群线/散点。
GRID_DEPTH_SUPPORT_MIN_BINS = 3
# 至少需要多少个有效前后 bin，避免只用极少点强行拟合。

# ---------- 九宫格点云拟合：进入条件与多帧叠加 ----------
GRID_FIT_YAW_GATE_ENABLE = True
# 是否启用九宫格点云拟合前的车头朝向门控。
GRID_FIT_YAW_GATE_DEG = 15.0
# odin1_base_link yaw 与重试区指向九宫格箭头 yaw 的最大允许误差。
GRID_FIT_POSITION_GATE_ENABLE = True
# 是否启用九宫格点云拟合前的位置门控。
GRID_FIT_POSITION_GATE_SIZE_X = 1.40
# 位置门控框 root 局部 X 尺寸，中心是当前队伍 retry_rel，可比重试区略大。
GRID_FIT_POSITION_GATE_SIZE_Y = 1.40
# 位置门控框 root 局部 Y 尺寸，中心是当前队伍 retry_rel，可比重试区略大。
GRID_FIT_ACCUMULATE_FRAMES_ENABLE = False
# 是否启用多帧 ROI 点云叠加拟合；默认关闭，保持单帧拟合行为。
GRID_FIT_ACCUMULATE_FRAME_COUNT = 5
# 多帧叠加时最多保留最近多少帧通过门控的 ROI 点云。

# ---------- 九宫格点云拟合：前景和箭头走廊过滤 ----------
GRID_FOREGROUND_RAY_FILTER_ENABLE = True
# 九宫格拟合前是否按车体到点云的光线保留前景点。
GRID_FOREGROUND_RAY_BIN_DEG = 1.0
# 光线角度 bin 宽度，越小越接近真实激光束，但点太稀时会不稳定。
GRID_FOREGROUND_RAY_KEEP_DEPTH = 0.22
# 每条光线只保留最近距离之后这段厚度内的点，过滤后方杂物。
GRID_MODEL_ALPHA = 0.50
# 拟合九宫格模型 marker 透明度。
GRID_MODEL_FREEZE_AFTER_FIRST = True
# 九宫格模型 marker 首次成功拟合后是否冻结，避免调试 marker 随后续点云跳动。
GRID_ARROW_CORRIDOR_ENABLE = True
# 九宫格拟合前是否启用重试区箭头走廊过滤。
GRID_ARROW_CORRIDOR_FORWARD_MARGIN = 1.0
# 箭头方向上，以理论九宫格中心距离为基准前后保留的距离。
GRID_ARROW_CORRIDOR_HALF_WIDTH = 1.1
# 箭头左右方向保留半宽。

# ---------- Marker 命名、编号与显示参数：只影响 RViz 调试显示，不参与拟合计算 ----------
MARKER_FRAME = "odom"
# 所有调试 marker 都发布在 odom 下，和 debugcloud 坐标系一致。
MARKER_ID_RETRY_CUBE = 1
MARKER_ID_RETRY_OUTLINE = 2
MARKER_ID_RETRY_ARROW = 3
MARKER_ID_RAMP_OUTLINE = 4
MARKER_ID_RAMP_PATH = 5
MARKER_ID_GRID_ROI = 10
MARKER_ID_GRID_REFERENCE = 11
MARKER_ID_POSITION_GATE = 12
MARKER_ID_GRID_FIT_BASE = 20
MARKER_ID_GRID_FIT_BLOCK_START = 21

RETRY_CUBE_THICKNESS = 0.025
# 重试区面 marker 的显示厚度。
RETRY_OUTLINE_WIDTH = 0.035
RETRY_OUTLINE_Z_OFFSET = 0.035
RETRY_ARROW_SHAFT_DIAMETER = 0.04
RETRY_ARROW_HEAD_DIAMETER = 0.10
RETRY_ARROW_HEAD_LENGTH = 0.10
RETRY_ARROW_Z_OFFSET = 0.10
RAMP_OUTLINE_WIDTH = 0.025
RAMP_PATH_WIDTH = 0.035
RAMP_LOW_Z_REL = 0.08
GRID_ROI_ALPHA = 0.12
GRID_REFERENCE_WIDTH = 0.03
GRID_REFERENCE_Z_OFFSET = 0.04
POSITION_GATE_Z_OFFSET = 0.055
POSITION_GATE_THICKNESS = 0.035
POSITION_GATE_ALPHA = 0.25
POSITION_GATE_DISABLED_ALPHA = 0.08

COLOR_RETRY = (0.05, 0.35, 1.0, 0.35)
COLOR_RETRY_OUTLINE = (0.05, 0.35, 1.0, 1.0)
COLOR_RETRY_ARROW = (1.0, 0.85, 0.05, 1.0)
COLOR_RAMP_OUTLINE = (0.9, 0.9, 0.9, 0.9)
COLOR_RAMP_PATH = (0.0, 0.9, 0.2, 1.0)
COLOR_GRID_ROI = (0.1, 1.0, 0.2, GRID_ROI_ALPHA)
COLOR_GRID_REFERENCE = (1.0, 0.2, 0.8, 1.0)
COLOR_POSITION_GATE = (0.0, 0.95, 1.0, POSITION_GATE_ALPHA)
COLOR_POSITION_GATE_DISABLED = (0.0, 0.95, 1.0, POSITION_GATE_DISABLED_ALPHA)
COLOR_GRID_FIT_BASE = (1.0, 1.0, 1.0)
COLOR_GRID_FIT_BLOCK = (0.0, 0.0, 0.0)

# ---------- 拟合 zone3_root TF 输出 ----------
ZONE3_ROOT_TF_PUBLISH_ENABLE = True
# 是否根据点云拟合出的九宫格模型反推并发布 zone3_root。
ZONE3_ROOT_TF_PARENT_FRAME = "odom"
# zone3_root TF 父帧，和 zone3/localizer_node.py 的 SOURCE_FIXED_FRAME 保持一致。
ZONE3_ROOT_TF_CHILD_FRAME = ""
# 为空时自动使用 blue_zone3_root 或 red_zone3_root。
ZONE3_ROOT_TF_USE_GRID_Z = True
# True 时用拟合出的九宫格基座中心高度反推 zone3_root.z，避免场地模型整体偏高/偏低。
ZONE3_ROOT_TF_Z = 0.0
# USE_GRID_Z=False 时使用的固定 zone3_root.z，和 zone3/config.py 的 GROUND_Z 语义一致。


def quat_to_rpy(q) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def rot(yaw: float, x: float, y: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


class Zone3GridLocalizer(Node):
    def __init__(self):
        super().__init__("zone3_grid_localizer")

        # 话题参数：launch 里可以重映射输入/输出，不需要改脚本主体。
        self.declare_parameter("is_blue_team", IS_BLUE_TEAM)  # true=蓝方，false=红方。
        self.declare_parameter("odom_topic", ODOM_TOPIC)  # 输入 odom 话题。
        self.declare_parameter("cloud_topic", CLOUD_TOPIC)  # 输入 cloud_slam 点云话题。
        self.declare_parameter("marker_topic", MARKER_TOPIC)  # 输出 MarkerArray 话题。
        self.declare_parameter("debugcloud_topic", CLOUD_DEBUG_TOPIC)  # 输出 ROI 调试点云话题。

        # 上坡锁定参数：决定什么时候认为已经完成 Z3 上坡并允许锁定 root。
        self.declare_parameter("min_z_gain", 0.34)  # 坡底到坡顶的最小 odom Z 增量，过小会误触发。
        self.declare_parameter("max_z_gain", 0.70)  # 坡底到坡顶的最大 odom Z 增量，过大通常说明轨迹段选错。
        self.declare_parameter("min_xy_dist", 0.70)  # 上坡段最小 XY 位移，过滤原地抖动或短片段。
        self.declare_parameter("min_pitch_abs_deg", 18.0)  # 上坡段最大 pitch 绝对值阈值，确认车体确实在爬坡。
        self.declare_parameter("window_sec", 35.0)  # odom 缓存窗口长度，只在最近这段轨迹内找坡道。

        # 上坡事件参数：先用较弱条件找到“开始上坡”的时间，再向前回溯找坡底。
        self.declare_parameter("ramp_event_min_z_gain", 0.035)  # 触发上坡事件的最小局部 Z 增量。
        self.declare_parameter("ramp_event_min_pitch_abs_deg", 6.0)  # 触发上坡事件的最小 pitch 绝对值。
        self.declare_parameter("ramp_event_backtrack_sec", 1.0)  # 事件发生后向前回溯多少秒找坡底低点。

        # 发布与日志参数。
        self.declare_parameter("lock_delay_sec", 2.0)  # 候选拟合至少保持多久才正式锁定，降低瞬时误拟合。
        self.declare_parameter("publish_rate_hz", 10.0)  # marker 发布频率。
        self.declare_parameter("log_dir", LOG_DIR)  # 兼容旧参数；未传 detail_log_root_dir 时作为日志目录。
        self.declare_parameter("detailed_file_log", False)  # launch 传入；是否写文件日志。
        self.declare_parameter("log_enabled", False)  # launch 传入；兼容通用日志开关。
        self.declare_parameter("detail_log_root_dir", DEFAULT_DETAIL_LOG_ROOT_DIR)  # 日志根目录。
        self.declare_parameter("detail_log_name", "zone3")  # 日志文件名前缀。
        self.declare_parameter("detail_log_timestamp", "")  # launch 统一时间戳。

        # 九宫格拟合门控参数。
        self.declare_parameter("grid_fit_yaw_gate_enable", GRID_FIT_YAW_GATE_ENABLE)
        # 是否启用九宫格点云拟合前的角度位控。
        self.declare_parameter("grid_fit_yaw_gate_deg", GRID_FIT_YAW_GATE_DEG)
        # 角度位控阈值，单位 deg。
        self.declare_parameter("grid_fit_position_gate_enable", GRID_FIT_POSITION_GATE_ENABLE)
        # 是否启用九宫格点云拟合前的位置门控。
        self.declare_parameter("grid_fit_position_gate_size_x", GRID_FIT_POSITION_GATE_SIZE_X)
        # 位置门控框 X 尺寸，root 局部坐标，中心为当前队伍 retry_rel。
        self.declare_parameter("grid_fit_position_gate_size_y", GRID_FIT_POSITION_GATE_SIZE_Y)
        # 位置门控框 Y 尺寸，root 局部坐标，中心为当前队伍 retry_rel。
        self.declare_parameter("grid_fit_accumulate_frames_enable", GRID_FIT_ACCUMULATE_FRAMES_ENABLE)
        # 是否启用多帧叠加拟合。
        self.declare_parameter("grid_fit_accumulate_frame_count", GRID_FIT_ACCUMULATE_FRAME_COUNT)
        # 多帧叠加保留帧数。

        # 九宫格拟合离群点过滤参数。
        self.declare_parameter("grid_depth_support_filter_enable", GRID_DEPTH_SUPPORT_FILTER_ENABLE)
        # 是否启用前后方向横向支撑滤波，抑制前表面离群点。
        self.declare_parameter("grid_depth_support_bin_size", GRID_DEPTH_SUPPORT_BIN_SIZE)
        # 前后方向分 bin 尺寸，单位 m。
        self.declare_parameter("grid_depth_support_min_points", GRID_DEPTH_SUPPORT_MIN_POINTS)
        # 每个有效前后 bin 的最少点数。
        self.declare_parameter("grid_depth_support_min_y_span", GRID_DEPTH_SUPPORT_MIN_Y_SPAN)
        # 每个有效前后 bin 的最小左右跨度。
        self.declare_parameter("grid_depth_support_min_bins", GRID_DEPTH_SUPPORT_MIN_BINS)
        # 最少有效前后 bin 数。

        # 九宫格拟合前景/走廊过滤参数。
        self.declare_parameter("grid_foreground_ray_filter_enable", GRID_FOREGROUND_RAY_FILTER_ENABLE)
        # 是否启用前景光线滤波，过滤九宫格后方杂物。
        self.declare_parameter("grid_foreground_ray_bin_deg", GRID_FOREGROUND_RAY_BIN_DEG)
        # 前景光线滤波角度 bin，单位 deg。
        self.declare_parameter("grid_foreground_ray_keep_depth", GRID_FOREGROUND_RAY_KEEP_DEPTH)
        # 每条光线保留最近点后的深度厚度，单位 m。
        self.declare_parameter("grid_model_alpha", GRID_MODEL_ALPHA)
        # 拟合九宫格模型透明度。
        self.declare_parameter("grid_model_freeze_after_first", GRID_MODEL_FREEZE_AFTER_FIRST)
        # 首次成功拟合九宫格模型后是否冻结 marker。
        self.declare_parameter("grid_arrow_corridor_enable", GRID_ARROW_CORRIDOR_ENABLE)
        # 是否启用重试区箭头走廊过滤。
        self.declare_parameter(
            "grid_arrow_corridor_forward_margin",
            GRID_ARROW_CORRIDOR_FORWARD_MARGIN,
        )
        # 箭头方向前后保留距离。
        self.declare_parameter("grid_arrow_corridor_half_width", GRID_ARROW_CORRIDOR_HALF_WIDTH)
        # 箭头左右方向保留半宽。

        # 拟合 zone3_root TF 输出参数。
        self.declare_parameter("zone3_root_tf_publish_enable", ZONE3_ROOT_TF_PUBLISH_ENABLE)
        # 是否根据九宫格拟合发布 zone3_root。
        self.declare_parameter("zone3_root_tf_parent_frame", ZONE3_ROOT_TF_PARENT_FRAME)
        # zone3_root TF 父帧。
        self.declare_parameter("zone3_root_tf_child_frame", ZONE3_ROOT_TF_CHILD_FRAME)
        # zone3_root TF 子帧。
        self.declare_parameter("zone3_root_tf_use_grid_z", ZONE3_ROOT_TF_USE_GRID_Z)
        # 是否用拟合九宫格高度反推 zone3_root.z。
        self.declare_parameter("zone3_root_tf_z", ZONE3_ROOT_TF_Z)
        # 固定 zone3_root TF 高度。

        # 话题与上坡锁定参数读取。
        self._is_blue_team = bool(self.get_parameter("is_blue_team").value)
        self._team_label = "blue" if self._is_blue_team else "red"
        self._team_sign = 1.0 if self._is_blue_team else -1.0
        self._zone3_root_frame = f"{self._team_label}_zone3_root"
        self._retry_rel = self._mirror_x(BLUE_RETRY_REL)
        self._grid_center_rel = self._mirror_x(BLUE_GRID_CENTER_REL)
        self._ramp_x = self._team_sign * BLUE_RAMP_X
        ramp_edges = sorted((self._team_sign * RAMP_LEFT_X, self._team_sign * RAMP_RIGHT_X))
        self._ramp_left_x, self._ramp_right_x = ramp_edges
        roi_x = sorted((self._team_sign * GRID_ROI_X[0], self._team_sign * GRID_ROI_X[1]))
        self._grid_roi_x = (float(roi_x[0]), float(roi_x[1]))
        self._grid_roi_y = GRID_ROI_Y
        self._marker_ns_retry = f"{self._team_label}_z3_retry_probe"
        self._marker_ns_position_gate = f"{self._team_label}_z3_fit_position_gate"
        self._marker_ns_grid_fit = f"{self._team_label}_z3_grid_fit_model"

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._cloud_topic = str(self.get_parameter("cloud_topic").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        debugcloud_topic = str(self.get_parameter("debugcloud_topic").value)
        self._min_z_gain = float(self.get_parameter("min_z_gain").value)
        self._max_z_gain = float(self.get_parameter("max_z_gain").value)
        self._min_xy_dist = float(self.get_parameter("min_xy_dist").value)
        self._min_pitch_abs = math.radians(float(self.get_parameter("min_pitch_abs_deg").value))
        self._window_sec = float(self.get_parameter("window_sec").value)
        self._event_min_z_gain = float(self.get_parameter("ramp_event_min_z_gain").value)
        self._event_min_pitch_abs = math.radians(
            float(self.get_parameter("ramp_event_min_pitch_abs_deg").value)
        )
        self._event_backtrack_sec = float(self.get_parameter("ramp_event_backtrack_sec").value)
        self._lock_delay_sec = float(self.get_parameter("lock_delay_sec").value)
        self._file_log_enabled = bool(self.get_parameter("detailed_file_log").value) or bool(
            self.get_parameter("log_enabled").value
        )
        self._detail_log_name = str(self.get_parameter("detail_log_name").value).strip() or "zone3"
        self._detail_log_timestamp = str(self.get_parameter("detail_log_timestamp").value).strip()
        detail_log_root_dir = str(self.get_parameter("detail_log_root_dir").value).strip()
        if detail_log_root_dir:
            self._log_dir = os.path.join(detail_log_root_dir, "zone_detection", "zone3")
        else:
            self._log_dir = str(self.get_parameter("log_dir").value)

        # 九宫格拟合参数读取。
        self._grid_fit_yaw_gate_enable = bool(
            self.get_parameter("grid_fit_yaw_gate_enable").value
        )
        self._grid_fit_yaw_gate = math.radians(
            float(self.get_parameter("grid_fit_yaw_gate_deg").value)
        )
        self._grid_fit_position_gate_enable = bool(
            self.get_parameter("grid_fit_position_gate_enable").value
        )
        self._grid_fit_position_gate_size_x = max(
            0.05,
            float(self.get_parameter("grid_fit_position_gate_size_x").value),
        )
        self._grid_fit_position_gate_size_y = max(
            0.05,
            float(self.get_parameter("grid_fit_position_gate_size_y").value),
        )
        self._grid_fit_accumulate_frames_enable = bool(
            self.get_parameter("grid_fit_accumulate_frames_enable").value
        )
        self._grid_fit_accumulate_frame_count = max(
            1,
            int(self.get_parameter("grid_fit_accumulate_frame_count").value),
        )
        self._grid_depth_support_filter_enable = bool(
            self.get_parameter("grid_depth_support_filter_enable").value
        )
        self._grid_depth_support_bin_size = max(
            0.02,
            float(self.get_parameter("grid_depth_support_bin_size").value),
        )
        self._grid_depth_support_min_points = max(
            1,
            int(self.get_parameter("grid_depth_support_min_points").value),
        )
        self._grid_depth_support_min_y_span = max(
            0.05,
            float(self.get_parameter("grid_depth_support_min_y_span").value),
        )
        self._grid_depth_support_min_bins = max(
            1,
            int(self.get_parameter("grid_depth_support_min_bins").value),
        )
        self._grid_foreground_ray_filter_enable = bool(
            self.get_parameter("grid_foreground_ray_filter_enable").value
        )
        self._grid_foreground_ray_bin = math.radians(
            float(self.get_parameter("grid_foreground_ray_bin_deg").value)
        )
        self._grid_foreground_ray_keep_depth = float(
            self.get_parameter("grid_foreground_ray_keep_depth").value
        )
        self._grid_model_alpha = float(self.get_parameter("grid_model_alpha").value)
        self._grid_model_freeze_after_first = bool(
            self.get_parameter("grid_model_freeze_after_first").value
        )
        self._grid_arrow_corridor_enable = bool(
            self.get_parameter("grid_arrow_corridor_enable").value
        )
        self._grid_arrow_corridor_forward_margin = float(
            self.get_parameter("grid_arrow_corridor_forward_margin").value
        )
        self._grid_arrow_corridor_half_width = float(
            self.get_parameter("grid_arrow_corridor_half_width").value
        )
        self._zone3_root_tf_publish_enable = bool(
            self.get_parameter("zone3_root_tf_publish_enable").value
        )
        self._zone3_root_tf_parent_frame = str(
            self.get_parameter("zone3_root_tf_parent_frame").value
        ).strip()
        if not self._zone3_root_tf_parent_frame:
            self._zone3_root_tf_parent_frame = ZONE3_ROOT_TF_PARENT_FRAME
        self._zone3_root_tf_child_frame = str(
            self.get_parameter("zone3_root_tf_child_frame").value
        ).strip()
        if not self._zone3_root_tf_child_frame:
            self._zone3_root_tf_child_frame = self._zone3_root_frame
        self._zone3_root_tf_use_grid_z = bool(
            self.get_parameter("zone3_root_tf_use_grid_z").value
        )
        self._zone3_root_tf_z = float(self.get_parameter("zone3_root_tf_z").value)

        self._buf: deque[dict] = deque()
        self._ramp_event_since = None
        self._candidate_fit = None
        self._candidate_since = None
        self._fit = None
        self._latest_odom = None
        self._grid_model_pose = None
        self._grid_model_locked = False
        self._grid_accum_frames = deque(maxlen=self._grid_fit_accumulate_frame_count)
        self._last_yaw_gate_state = None
        self._last_position_gate_state = None
        self._last_roi_points = 0
        self._last_log_t = 0.0
        self._last_cloud_warn_t = 0.0
        self._last_yaw_gate_log_t = 0.0
        self._last_position_gate_log_t = 0.0
        self._last_grid_model_log_t = 0.0
        self._last_zone3_root_tf_log_t = 0.0
        self._last_arrow_filter_log_t = 0.0
        self._last_detail_log_t = 0.0
        self._heartbeat_seq = 0
        self._detail_log_path, self._heartbeat_log_path = self._open_logs()
        self._log_path = self._detail_log_path
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, sensor_qos)
        self.create_subscription(PointCloud2, self._cloud_topic, self._cloud_cb, sensor_qos)
        self._marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self._debugcloud_pub = self.create_publisher(PointCloud2, debugcloud_topic, 10)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(1.0, rate), self._publish_markers)
        self.create_timer(1.0 / HEARTBEAT_LOG_HZ, self._write_heartbeat_log)

    def _mirror_x(self, rel: tuple[float, float, float]) -> tuple[float, float, float]:
        return self._team_sign * rel[0], rel[1], rel[2]

    def _odom_cb(self, msg: Odometry):
        stamp = msg.header.stamp
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        p = msg.pose.pose.position
        roll, pitch, yaw = quat_to_rpy(msg.pose.pose.orientation)
        self._buf.append({
            "t": t,
            "x": p.x,
            "y": p.y,
            "z": p.z,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "stamp": stamp,
        })
        self._latest_odom = self._buf[-1]
        while self._buf and t - self._buf[0]["t"] > self._window_sec:
            self._buf.popleft()
        if self._fit is None:
            self._try_fit()

    def _try_fit(self):
        if len(self._buf) < 80:
            return
        pts = list(self._buf)
        span = self._online_ramp_span(pts)
        if span is None:
            z_gain = pts[-1]["z"] - min(p["z"] for p in pts)
            xy_dist = math.hypot(pts[-1]["x"] - pts[0]["x"], pts[-1]["y"] - pts[0]["y"])
            self._log_row("wait_span", pts[-1], z_gain=z_gain, xy_dist=xy_dist)
            self._status(f"wait z_gain={z_gain:.3f} xy={xy_dist:.3f}")
            return

        start, end = span
        low = pts[start]
        top = pts[end]
        z_gain = top["z"] - low["z"]
        xy_dist = math.hypot(top["x"] - low["x"], top["y"] - low["y"])
        pitch_abs = max(abs(p["pitch"]) for p in pts[start:end + 1])
        if z_gain < self._min_z_gain or z_gain > self._max_z_gain or xy_dist < self._min_xy_dist:
            self._log_row("wait_z_xy", top, low=low, top=top, z_gain=z_gain, xy_dist=xy_dist, pitch_abs=pitch_abs)
            self._status(f"wait z_gain={z_gain:.3f} xy={xy_dist:.3f}")
            return
        if pitch_abs < self._min_pitch_abs:
            self._log_row("wait_pitch", top, low=low, top=top, z_gain=z_gain, xy_dist=xy_dist, pitch_abs=pitch_abs)
            self._status(f"wait pitch_abs={math.degrees(pitch_abs):.1f}")
            return

        fit = self._make_ramp_fit(pts, start, end)
        if fit is None:
            self._log_row("reject_fit", top, low=low, top=top, z_gain=z_gain, xy_dist=xy_dist, pitch_abs=pitch_abs)
            self._status("wait ramp_axis")
            return
        if self._candidate_fit is None:
            self._candidate_fit = fit
            self._candidate_since = pts[-1]["t"]
        elif self._ramp_score(fit) > self._ramp_score(self._candidate_fit):
            self._candidate_fit = fit
        age = pts[-1]["t"] - float(self._candidate_since)
        if age < self._lock_delay_sec:
            self._log_row("hold_candidate", top, low=fit["low"], top=fit["top"], fit=fit, age=age, pitch_abs=pitch_abs)
            self._status(
                f"hold ramp age={age:.2f}s z_gain={fit['z_gain']:.3f} residual={fit['residual']:.3f}"
            )
            return

        self._fit = self._candidate_fit
        retry_x, retry_y, retry_z = self._local_point(*self._retry_rel)
        self._log_row(
            "lock_retry_marker",
            self._fit["top"],
            low=self._fit["low"],
            top=self._fit["top"],
            fit=self._fit,
            retry=(retry_x, retry_y, retry_z),
            age=age,
            pitch_abs=pitch_abs,
        )
        self.get_logger().info(
            "[LOCK_RETRY_MARKER] "
            f"root=({self._fit['root_x']:.3f},{self._fit['root_y']:.3f},"
            f"{math.degrees(self._fit['root_yaw']):.2f}deg) "
            f"retry=({retry_x:.3f},{retry_y:.3f},{retry_z:.3f}) "
            f"ramp_yaw={math.degrees(self._fit['ramp_yaw']):.2f}deg "
            f"z_gain={self._fit['z_gain']:.3f} residual={self._fit['residual']:.3f}"
        )

    def _online_ramp_span(self, pts):
        if self._ramp_event_since is None:
            self._detect_ramp_event(pts)
        if self._ramp_event_since is None:
            return None
        event_i = next((i for i, p in enumerate(pts) if p["t"] >= self._ramp_event_since), None)
        if event_i is None:
            return None
        back_t = self._ramp_event_since - self._event_backtrack_sec
        back_i = next((i for i, p in enumerate(pts) if p["t"] >= back_t), 0)
        start_hi = min(len(pts), event_i + 12)
        if start_hi <= back_i:
            return None
        start = min(range(back_i, start_hi), key=lambda i: pts[i]["z"])
        end = max(range(start + 1, len(pts)), key=lambda i: pts[i]["z"])
        if end <= start + 8:
            return None
        return start, end

    def _detect_ramp_event(self, pts):
        for i in range(10, len(pts)):
            base_i0 = max(0, i - 80)
            base = min(pts[base_i0:i], key=lambda p: p["z"])
            z_gain = pts[i]["z"] - base["z"]
            if z_gain < self._event_min_z_gain:
                continue
            local_pitch = max(abs(p["pitch"]) for p in pts[max(0, i - 20):i + 1])
            if local_pitch < self._event_min_pitch_abs:
                continue
            xy_dist = math.hypot(pts[i]["x"] - base["x"], pts[i]["y"] - base["y"])
            self._ramp_event_since = base["t"]
            self._log_row(
                "ramp_event",
                pts[i],
                low=base,
                z_gain=z_gain,
                xy_dist=xy_dist,
                pitch_abs=local_pitch,
            )
            self.get_logger().info(
                "[RAMP_EVENT] "
                f"t={self._ramp_event_since:.3f} trigger_t={pts[i]['t']:.3f} "
                f"z_gain={z_gain:.3f} xy={xy_dist:.3f} "
                f"pitch_abs={math.degrees(local_pitch):.1f}deg"
            )
            return

    def _make_ramp_fit(self, pts, start: int, end: int):
        low = pts[start]
        top = pts[end]
        seg = pts[start:end + 1]
        xy = np.asarray([[p["x"], p["y"]] for p in seg], dtype=np.float64)
        z = np.asarray([p["z"] for p in seg], dtype=np.float64)
        pitch = np.asarray([abs(p["pitch"]) for p in seg], dtype=np.float64)
        low_z = float(low["z"])
        top_z = float(top["z"])
        mask = (z >= low_z + 0.04) & (z <= top_z - 0.015) & (pitch >= math.radians(5.0))
        if int(mask.sum()) < 12:
            mask = (z >= low_z + 0.02) & (z <= top_z)
        if int(mask.sum()) < 8:
            return None
        ramp_xy = xy[mask]
        center = ramp_xy.mean(axis=0)
        _, _, vh = np.linalg.svd(ramp_xy - center, full_matrices=False)
        axis = vh[0]
        low_to_top = np.array([top["x"] - low["x"], top["y"] - low["y"]], dtype=np.float64)
        if float(np.dot(axis, low_to_top)) < 0.0:
            axis = -axis
        ramp_yaw = math.atan2(float(axis[1]), float(axis[0]))
        root_yaw = ramp_yaw + math.pi / 2.0
        proj = (ramp_xy - center) @ axis
        lo_p, hi_p = np.percentile(proj, [4.0, 96.0])
        low_xy = center + axis * lo_p
        top_xy = center + axis * hi_p
        low = {**low, "x": float(low_xy[0]), "y": float(low_xy[1])}
        top = {**top, "x": float(top_xy[0]), "y": float(top_xy[1])}
        top_dx, top_dy = rot(root_yaw, self._ramp_x, RAMP_TOP_Y)
        low_dx, low_dy = rot(root_yaw, self._ramp_x, RAMP_LOW_Y)
        root_top = (top["x"] - top_dx, top["y"] - top_dy)
        root_low = (low["x"] - low_dx, low["y"] - low_dy)
        root_x = 0.5 * (root_top[0] + root_low[0])
        root_y = 0.5 * (root_top[1] + root_low[1])
        residual = math.hypot(root_top[0] - root_low[0], root_top[1] - root_low[1])
        if residual > RAMP_WEAK_MAX_RESIDUAL:
            self._log_row(
                "reject_residual",
                top,
                low=low,
                top=top,
                z_gain=top_z - low_z,
                xy_dist=float(hi_p - lo_p),
                residual=residual,
                ramp_yaw=ramp_yaw,
                root_yaw=root_yaw,
            )
            self._status(f"reject residual={residual:.3f}")
            return None
        source = "ramp_strong" if residual <= RAMP_STRONG_MAX_RESIDUAL else "ramp_weak"
        return {
            "root_x": root_x,
            "root_y": root_y,
            "root_yaw": root_yaw,
            "low": low,
            "top": top,
            "z_gain": top_z - low_z,
            "xy_dist": float(hi_p - lo_p),
            "residual": residual,
            "ramp_yaw": ramp_yaw,
            "ground_z_odom": low_z,
            "source": source,
        }

    @staticmethod
    def _ramp_score(fit):
        return fit["z_gain"] - 2.0 * fit["residual"]

    def _local_point(self, local_x: float, local_y: float, local_z: float) -> tuple[float, float, float]:
        # marker 的几何点都先按当前队伍 zone3_root 局部坐标书写，
        # 再用已锁定的 root 位姿转换到 odom 中显示。
        dx, dy = rot(self._fit["root_yaw"], local_x, local_y)
        return self._fit["root_x"] + dx, self._fit["root_y"] + dy, local_z

    def _point(self, local_x: float, local_y: float, local_z: float) -> Point:
        x, y, z = self._local_point(local_x, local_y, local_z)
        p = Point()
        p.x = x
        p.y = y
        p.z = z
        return p

    def _cloud_cb(self, msg: PointCloud2):
        if self._fit is None:
            return
        try:
            x, y, z = self._parse_xyz(msg)
            # debugcloud 和 marker 统一使用 odom frame。
            # 如果 cloud_slam 本来就是 odom，这一步不会改变点；
            # 如果 cloud_slam 是其他 frame，则先用 TF 转到 odom，再做 ROI 裁剪。
            ox, oy, oz = self._to_odom_xyz(msg.header, x, y, z)
        except Exception as e:
            self._cloud_warn(f"debugcloud skip: {e}")
            return
        if ox is None:
            return
        finite = np.isfinite(ox) & np.isfinite(oy) & np.isfinite(oz)
        if not finite.any():
            self._debugcloud_pub.publish(self._empty_cloud(msg.header.stamp))
            return
        ox = ox[finite]
        oy = oy[finite]
        oz = oz[finite]

        # 关键点：ROI 不是 odin1_base_link 下的随车框。
        # 这里先把 odom XY 反算到已锁定的 zone3_root 局部坐标，
        # 所以 GRID_ROI_X/Y 会固定在拟合出的场地位置，车移动后 ROI 不跟车走。
        lx, ly = self._odom_to_root_xy(ox, oy)
        x_min, x_max, y_min, y_max = self._active_grid_roi_bounds()
        roi = (
            (lx >= x_min) & (lx <= x_max)
            & (ly >= y_min) & (ly <= y_max)
            # Z 目前直接使用 odom 高度筛选。
            # 如果后面发现坡面/地面漂移影响很大，再改成锁定平台 ground_z 后筛相对高度。
            & (oz >= GRID_ROI_Z[0]) & (oz <= GRID_ROI_Z[1])
        )
        if not roi.any():
            self._debugcloud_pub.publish(self._empty_cloud(msg.header.stamp))
            return
        x_pub = ox[roi]
        y_pub = oy[roi]
        z_pub = oz[roi]
        yaw_gate = self._grid_fit_yaw_gate_state()
        position_gate = self._grid_fit_position_gate_state()
        self._last_yaw_gate_state = yaw_gate
        self._last_position_gate_state = position_gate
        self._last_roi_points = len(z_pub)
        self._log_yaw_gate(yaw_gate, len(z_pub))
        self._log_position_gate(position_gate, len(z_pub))
        if yaw_gate["pass"] and position_gate["pass"]:
            fit_x, fit_y, fit_z = self._grid_fit_points(x_pub, y_pub, z_pub)
            self._update_grid_model_fit(fit_x, fit_y, fit_z, yaw_gate, position_gate)
        else:
            self._grid_accum_frames.clear()
        r, g, b = self._height_colors(z_pub)
        self._debugcloud_pub.publish(
            self._make_rgb_cloud(msg.header.stamp, x_pub, y_pub, z_pub, r, g, b)
        )

    def _publish_markers(self):
        if self._fit is None:
            return
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        retry = self._retry_cube(stamp)
        outline = self._retry_outline(stamp)
        arrow = self._retry_face_arrow(stamp)
        ramp = self._ramp_outline(stamp)
        path = self._ramp_path(stamp)
        roi = self._grid_roi_marker(stamp)
        grid = self._grid_reference_marker(stamp)
        position_gate = self._position_gate_marker(stamp)
        arr.markers.extend([retry, outline, arrow, ramp, path, roi, grid, position_gate])
        arr.markers.extend(self._grid_fitted_model_markers(stamp))
        self._marker_pub.publish(arr)
        self._publish_zone3_root_tf(stamp)

    def _base_marker(self, stamp, marker_id: int, ns: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = MARKER_FRAME
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.lifetime.sec = 0
        return marker

    @staticmethod
    def _set_color(marker: Marker, color):
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])

    def _retry_cube(self, stamp) -> Marker:
        marker = self._base_marker(
            stamp,
            MARKER_ID_RETRY_CUBE,
            self._marker_ns_retry,
            Marker.CUBE,
        )
        marker.pose.position = self._point(*self._retry_rel)
        qx, qy, qz, qw = quat_from_yaw(self._fit["root_yaw"])
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x, marker.scale.y, marker.scale.z = Z3_RETRY_SIZE
        marker.scale.z = RETRY_CUBE_THICKNESS
        self._set_color(marker, COLOR_RETRY)
        return marker

    def _retry_outline(self, stamp) -> Marker:
        marker = self._base_marker(
            stamp,
            MARKER_ID_RETRY_OUTLINE,
            self._marker_ns_retry,
            Marker.LINE_STRIP,
        )
        marker.scale.x = RETRY_OUTLINE_WIDTH
        self._set_color(marker, COLOR_RETRY_OUTLINE)
        cx, cy, cz = self._retry_rel
        hx = Z3_RETRY_SIZE[0] * 0.5
        hy = Z3_RETRY_SIZE[1] * 0.5
        corners = [
            (cx - hx, cy - hy, cz + RETRY_OUTLINE_Z_OFFSET),
            (cx + hx, cy - hy, cz + RETRY_OUTLINE_Z_OFFSET),
            (cx + hx, cy + hy, cz + RETRY_OUTLINE_Z_OFFSET),
            (cx - hx, cy + hy, cz + RETRY_OUTLINE_Z_OFFSET),
            (cx - hx, cy - hy, cz + RETRY_OUTLINE_Z_OFFSET),
        ]
        marker.points = [self._point(*p) for p in corners]
        return marker

    def _retry_face_arrow(self, stamp) -> Marker:
        marker = self._base_marker(
            stamp,
            MARKER_ID_RETRY_ARROW,
            self._marker_ns_retry,
            Marker.ARROW,
        )
        marker.scale.x = RETRY_ARROW_SHAFT_DIAMETER
        marker.scale.y = RETRY_ARROW_HEAD_DIAMETER
        marker.scale.z = RETRY_ARROW_HEAD_LENGTH
        self._set_color(marker, COLOR_RETRY_ARROW)
        sx, sy, sz = self._retry_rel
        gx, gy, gz = self._grid_center_rel
        marker.points = [
            self._point(sx, sy, sz + RETRY_ARROW_Z_OFFSET),
            self._point(gx, gy, gz + RETRY_ARROW_Z_OFFSET),
        ]
        return marker

    def _ramp_outline(self, stamp) -> Marker:
        marker = self._base_marker(
            stamp,
            MARKER_ID_RAMP_OUTLINE,
            self._marker_ns_retry,
            Marker.LINE_STRIP,
        )
        marker.scale.x = RAMP_OUTLINE_WIDTH
        self._set_color(marker, COLOR_RAMP_OUTLINE)
        corners = [
            (self._ramp_left_x, RAMP_LOW_Y, RAMP_LOW_Z_REL),
            (self._ramp_right_x, RAMP_LOW_Y, RAMP_LOW_Z_REL),
            (self._ramp_right_x, RAMP_TOP_Y, Z3_PLATFORM_Z_REL),
            (self._ramp_left_x, RAMP_TOP_Y, Z3_PLATFORM_Z_REL),
            (self._ramp_left_x, RAMP_LOW_Y, RAMP_LOW_Z_REL),
        ]
        marker.points = [self._point(*p) for p in corners]
        return marker

    def _ramp_path(self, stamp) -> Marker:
        marker = self._base_marker(
            stamp,
            MARKER_ID_RAMP_PATH,
            self._marker_ns_retry,
            Marker.LINE_STRIP,
        )
        marker.scale.x = RAMP_PATH_WIDTH
        self._set_color(marker, COLOR_RAMP_PATH)
        for key in ("low", "top"):
            p = Point()
            p.x = self._fit[key]["x"]
            p.y = self._fit[key]["y"]
            p.z = self._fit[key]["z"]
            marker.points.append(p)
        return marker

    def _grid_roi_marker(self, stamp) -> Marker:
        marker = self._base_marker(
            stamp,
            MARKER_ID_GRID_ROI,
            self._marker_ns_retry,
            Marker.CUBE,
        )
        # 这个 cube 画的就是 _cloud_cb 使用的同一个 ROI 体积。
        # 中心点用 root 局部坐标计算，再通过 _point() 转到 odom，
        # 因此 RViz 里看到的绿色框就是 debugcloud 实际裁剪的位置。
        x0, x1, y0, y1 = self._active_grid_roi_bounds()
        cx = 0.5 * (x0 + x1)
        cy = 0.5 * (y0 + y1)
        cz = 0.5 * (GRID_ROI_Z[0] + GRID_ROI_Z[1])
        marker.pose.position = self._point(cx, cy, cz)
        qx, qy, qz, qw = quat_from_yaw(self._fit["root_yaw"])
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = x1 - x0
        marker.scale.y = y1 - y0
        marker.scale.z = GRID_ROI_Z[1] - GRID_ROI_Z[0]
        self._set_color(marker, COLOR_GRID_ROI)
        return marker

    def _grid_reference_marker(self, stamp) -> Marker:
        marker = self._base_marker(
            stamp,
            MARKER_ID_GRID_REFERENCE,
            self._marker_ns_retry,
            Marker.LINE_STRIP,
        )
        marker.scale.x = GRID_REFERENCE_WIDTH
        self._set_color(marker, COLOR_GRID_REFERENCE)
        cx, cy, cz = self._grid_center_rel
        hx = GRID_BASE_SIZE[0] * 0.5
        hy = GRID_BASE_SIZE[1] * 0.5
        corners = [
            (cx - hx, cy - hy, cz + GRID_REFERENCE_Z_OFFSET),
            (cx + hx, cy - hy, cz + GRID_REFERENCE_Z_OFFSET),
            (cx + hx, cy + hy, cz + GRID_REFERENCE_Z_OFFSET),
            (cx - hx, cy + hy, cz + GRID_REFERENCE_Z_OFFSET),
            (cx - hx, cy - hy, cz + GRID_REFERENCE_Z_OFFSET),
        ]
        marker.points = [self._point(*p) for p in corners]
        return marker

    def _position_gate_marker(self, stamp) -> Marker:
        marker = self._base_marker(
            stamp,
            MARKER_ID_POSITION_GATE,
            self._marker_ns_position_gate,
            Marker.CUBE,
        )
        cx, cy, cz = self._retry_rel
        marker.pose.position = self._point(cx, cy, cz + POSITION_GATE_Z_OFFSET)
        qx, qy, qz, qw = quat_from_yaw(self._fit["root_yaw"])
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = self._grid_fit_position_gate_size_x
        marker.scale.y = self._grid_fit_position_gate_size_y
        marker.scale.z = POSITION_GATE_THICKNESS
        color = (
            COLOR_POSITION_GATE
            if self._grid_fit_position_gate_enable
            else COLOR_POSITION_GATE_DISABLED
        )
        self._set_color(marker, color)
        return marker

    def _grid_fitted_model_markers(self, stamp) -> list[Marker]:
        if self._grid_model_pose is None:
            return []
        pose = self._grid_model_pose
        markers = []
        qx, qy, qz, qw = quat_from_yaw(pose["yaw"])

        base = self._base_marker(
            stamp,
            MARKER_ID_GRID_FIT_BASE,
            self._marker_ns_grid_fit,
            Marker.CUBE,
        )
        base.pose.position.x = pose["x"]
        base.pose.position.y = pose["y"]
        base.pose.position.z = pose["base_z"]
        base.pose.orientation.x = qx
        base.pose.orientation.y = qy
        base.pose.orientation.z = qz
        base.pose.orientation.w = qw
        base.scale.x, base.scale.y, base.scale.z = GRID_BASE_SIZE
        base.color.r, base.color.g, base.color.b = COLOR_GRID_FIT_BASE
        base.color.a = self._grid_model_alpha
        markers.append(base)

        marker_id = MARKER_ID_GRID_FIT_BLOCK_START
        for lz in GRID_LAYER_ZS_REL:
            for cy in GRID_COL_YS_REL:
                bx, by = rot(pose["yaw"], 0.0, cy)
                block = self._base_marker(
                    stamp,
                    marker_id,
                    self._marker_ns_grid_fit,
                    Marker.CUBE,
                )
                block.pose.position.x = pose["x"] + bx
                block.pose.position.y = pose["y"] + by
                block.pose.position.z = pose["base_z"] + lz
                block.pose.orientation.x = qx
                block.pose.orientation.y = qy
                block.pose.orientation.z = qz
                block.pose.orientation.w = qw
                block.scale.x, block.scale.y, block.scale.z = GRID_BLOCK_SIZE
                block.color.r, block.color.g, block.color.b = COLOR_GRID_FIT_BLOCK
                block.color.a = self._grid_model_alpha
                markers.append(block)
                marker_id += 1
        return markers

    def _publish_zone3_root_tf(self, stamp):
        if not self._zone3_root_tf_publish_enable or self._grid_model_pose is None:
            return

        root_x, root_y, root_z, root_yaw = self._zone3_root_pose_from_grid()
        qx, qy, qz, qw = quat_from_yaw(root_yaw)
        tf_msg = self._tf_msg(
            stamp,
            self._zone3_root_tf_parent_frame,
            self._zone3_root_tf_child_frame,
            root_x,
            root_y,
            root_z,
            qx,
            qy,
            qz,
            qw,
        )
        self._tf_broadcaster.sendTransform(tf_msg)
        self._log_zone3_root_tf(root_x, root_y, root_z, root_yaw)

    def _zone3_root_pose_from_grid(self):
        pose = self._grid_model_pose
        local_grid_x, local_grid_y, _ = self._grid_center_rel
        root_yaw = self._grid_root_yaw(pose["yaw"])
        dx, dy = rot(root_yaw, local_grid_x, local_grid_y)
        root_x = float(pose["x"] - dx)
        root_y = float(pose["y"] - dy)
        if self._zone3_root_tf_use_grid_z:
            root_z = float(pose["base_z"] - GRID_BASE_CENTER_Z_REL)
        else:
            root_z = float(self._zone3_root_tf_z)
        return root_x, root_y, root_z, root_yaw

    def _grid_root_yaw(self, grid_yaw: float) -> float:
        if self._fit is None:
            return float(grid_yaw)
        candidate_a = self._norm_angle(grid_yaw)
        candidate_b = self._norm_angle(grid_yaw + math.pi)
        ref = self._fit["root_yaw"]
        if abs(self._norm_angle(candidate_b - ref)) < abs(self._norm_angle(candidate_a - ref)):
            return candidate_b
        return candidate_a

    @staticmethod
    def _tf_msg(stamp, parent, child, x, y, z, qx, qy, qz, qw) -> TransformStamped:
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(x)
        msg.transform.translation.y = float(y)
        msg.transform.translation.z = float(z)
        msg.transform.rotation.x = float(qx)
        msg.transform.rotation.y = float(qy)
        msg.transform.rotation.z = float(qz)
        msg.transform.rotation.w = float(qw)
        return msg

    def _log_zone3_root_tf(
        self,
        root_x: float,
        root_y: float,
        root_z: float,
        root_yaw: float,
    ):
        now = time.monotonic()
        if now - self._last_zone3_root_tf_log_t < 1.0:
            return
        self._last_zone3_root_tf_log_t = now
        pose = self._grid_model_pose
        self._log_row(
            "zone3_root_tf",
            self._latest_odom or {},
            fit=self._fit,
            gate=self._last_yaw_gate_state,
            position_gate=self._last_position_gate_state,
            roi_points=self._last_roi_points,
            root_tf=(root_x, root_y, root_z, root_yaw),
            grid_pose=pose,
        )

    def _status(self, text: str):
        _ = text

    def _cloud_warn(self, text: str):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_cloud_warn_t > 1.0:
            self._last_cloud_warn_t = now
            self._log_row("cloud_warn", self._latest_odom or {}, message=text, force=True)

    def _grid_fit_yaw_gate_state(self):
        target_yaw = self._retry_to_grid_yaw()
        odom = self._latest_odom
        if not self._grid_fit_yaw_gate_enable:
            return {
                "enabled": False,
                "pass": True,
                "reason": "disabled",
                "robot_yaw": odom["yaw"] if odom else None,
                "target_yaw": target_yaw,
                "error": 0.0,
                "tol": self._grid_fit_yaw_gate,
            }
        if odom is None:
            return {
                "enabled": True,
                "pass": False,
                "reason": "no_odom",
                "robot_yaw": None,
                "target_yaw": target_yaw,
                "error": None,
                "tol": self._grid_fit_yaw_gate,
            }
        err = self._norm_angle(odom["yaw"] - target_yaw)
        return {
            "enabled": True,
            "pass": abs(err) <= self._grid_fit_yaw_gate,
            "reason": "ok" if abs(err) <= self._grid_fit_yaw_gate else "yaw_error",
            "robot_yaw": odom["yaw"],
            "target_yaw": target_yaw,
            "error": err,
            "tol": self._grid_fit_yaw_gate,
        }

    def _grid_fit_position_gate_state(self):
        odom = self._latest_odom
        cx, cy, _ = self._retry_rel
        if odom is None:
            return {
                "enabled": self._grid_fit_position_gate_enable,
                "pass": not self._grid_fit_position_gate_enable,
                "reason": "no_odom",
                "local_x": None,
                "local_y": None,
                "dx": None,
                "dy": None,
                "size_x": self._grid_fit_position_gate_size_x,
                "size_y": self._grid_fit_position_gate_size_y,
            }

        lx, ly = self._odom_to_root_xy(
            np.asarray([odom["x"]], dtype=np.float64),
            np.asarray([odom["y"]], dtype=np.float64),
        )
        local_x = float(lx[0])
        local_y = float(ly[0])
        dx = local_x - cx
        dy = local_y - cy
        inside = (
            abs(dx) <= self._grid_fit_position_gate_size_x * 0.5
            and abs(dy) <= self._grid_fit_position_gate_size_y * 0.5
        )
        if not self._grid_fit_position_gate_enable:
            inside = True
        return {
            "enabled": self._grid_fit_position_gate_enable,
            "pass": inside,
            "reason": "ok" if inside else "outside_retry_gate",
            "local_x": local_x,
            "local_y": local_y,
            "dx": dx,
            "dy": dy,
            "size_x": self._grid_fit_position_gate_size_x,
            "size_y": self._grid_fit_position_gate_size_y,
        }

    def _retry_to_grid_yaw(self):
        sx, sy, _ = self._retry_rel
        gx, gy, _ = self._grid_center_rel
        local_yaw = math.atan2(gy - sy, gx - sx)
        return self._norm_angle(self._fit["root_yaw"] + local_yaw)

    def _log_yaw_gate(self, gate, roi_points: int):
        now = time.monotonic()
        if now - self._last_yaw_gate_log_t < 1.0:
            return
        self._last_yaw_gate_log_t = now
        self._log_row(
            "grid_fit_yaw_gate",
            self._latest_odom or {},
            fit=self._fit,
            gate=gate,
            roi_points=roi_points,
        )

    def _log_position_gate(self, gate, roi_points: int):
        now = time.monotonic()
        if now - self._last_position_gate_log_t < 1.0:
            return
        self._last_position_gate_log_t = now
        self._log_row(
            "grid_fit_position_gate",
            self._latest_odom or {},
            fit=self._fit,
            position_gate=gate,
            roi_points=roi_points,
            accum_frames=len(self._grid_accum_frames),
        )

    def _grid_fit_points(self, x, y, z):
        if not self._grid_fit_accumulate_frames_enable:
            return x, y, z
        self._grid_accum_frames.append((
            np.asarray(x, dtype=np.float64).copy(),
            np.asarray(y, dtype=np.float64).copy(),
            np.asarray(z, dtype=np.float64).copy(),
        ))
        return (
            np.concatenate([frame[0] for frame in self._grid_accum_frames]),
            np.concatenate([frame[1] for frame in self._grid_accum_frames]),
            np.concatenate([frame[2] for frame in self._grid_accum_frames]),
        )

    @staticmethod
    def _deg_text(value):
        if value is None:
            return "nan"
        return f"{math.degrees(float(value)):.1f}deg"

    @staticmethod
    def _num_text(value):
        if value is None:
            return "nan"
        return f"{float(value):.3f}"

    def _update_grid_model_fit(self, x, y, z, yaw_gate, position_gate):
        if self._grid_model_locked:
            return
        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        if int(finite.sum()) < GRID_DETECT_MIN_POINTS:
            return

        fx = np.asarray(x[finite], dtype=np.float64)
        fy = np.asarray(y[finite], dtype=np.float64)
        fz = np.asarray(z[finite], dtype=np.float64)
        raw_points = len(fx)
        fx, fy, fz = self._arrow_corridor_points(fx, fy, fz)
        corridor_points = len(fx)
        if len(fx) < GRID_DETECT_MIN_POINTS:
            return
        fx, fy, fz = self._foreground_ray_points(fx, fy, fz)
        if len(fx) < GRID_DETECT_MIN_POINTS:
            return

        visible_bottom_z = float(np.percentile(fz, 2.0))
        visible_top_z = float(np.percentile(fz, 98.0))
        if visible_top_z - visible_bottom_z < GRID_DETECT_MIN_VERTICAL_SPAN:
            return

        high = (
            (fz >= visible_bottom_z + GRID_DETECT_MIN_H)
            & (fz <= visible_bottom_z + GRID_DETECT_MAX_H)
        )
        if int(high.sum()) < GRID_DETECT_MIN_POINTS:
            return

        gx = fx[high]
        gy = fy[high]
        pts = np.column_stack((gx, gy))
        center0 = pts.mean(axis=0)
        demean = pts - center0
        if len(pts) < 3:
            return

        cov = np.cov(demean.T)
        if not np.all(np.isfinite(cov)):
            return
        vals, vecs = np.linalg.eigh(cov)
        long_axis = vecs[:, int(np.argmax(vals))]
        long_axis /= max(1e-9, float(np.hypot(long_axis[0], long_axis[1])))

        # rc26_field.py 里九宫格模型的长边在模型局部 Y 方向，
        # 所以 PCA 长轴对应 yaw + 90deg，模型 X 方向 yaw 需要减去 90deg。
        yaw = self._normalize_half_turn(math.atan2(long_axis[1], long_axis[0]) - math.pi / 2.0)
        c = math.cos(yaw)
        s = math.sin(yaw)
        lx = c * (gx - center0[0]) + s * (gy - center0[1])
        ly = -s * (gx - center0[0]) + c * (gy - center0[1])

        support = self._supported_depth_points(lx, ly)
        if support is not None:
            support_lx, support_ly, support_bins = support
            width = self._robust_span(support_ly)
            depth = self._robust_span(support_lx)
            local_center_x = 0.5 * (
                np.percentile(support_lx, 3.0) + np.percentile(support_lx, 97.0)
            )
            local_center_y = 0.5 * (
                np.percentile(support_ly, 3.0) + np.percentile(support_ly, 97.0)
            )
        else:
            support_lx = lx
            support_bins = 0
            width = self._robust_span(ly)
            depth = self._robust_span(lx)
            local_center_x = 0.5 * (np.percentile(lx, 5.0) + np.percentile(lx, 95.0))
            local_center_y = 0.5 * (np.percentile(ly, 3.0) + np.percentile(ly, 97.0))
        if width < 0.65 or width > 2.35 or depth > 0.95:
            return

        center_x = center0[0] + c * local_center_x - s * local_center_y
        center_y = center0[1] + s * local_center_x + c * local_center_y

        all_lx = c * (fx - center_x) + s * (fy - center_y)
        all_ly = -s * (fx - center_x) + c * (fy - center_y)
        footprint = (
            (np.abs(all_lx) <= GRID_MODEL_FOOTPRINT_X)
            & (np.abs(all_ly) <= GRID_MODEL_FOOTPRINT_Y)
        )
        if int(footprint.sum()) < 30:
            return
        base_core = (
            footprint
            & (np.abs(all_lx) <= GRID_BASE_Z_CORE_X)
            & (np.abs(all_ly) <= GRID_BASE_Z_CORE_Y)
        )
        base_z_source = "core" if int(base_core.sum()) >= GRID_BASE_Z_MIN_POINTS else "footprint"
        model_z = fz[base_core if base_z_source == "core" else footprint]
        low_band_max = float(np.percentile(model_z, 35.0))
        low_band = model_z[model_z <= low_band_max]
        if len(low_band) < 10:
            return
        base_bottom_z = float(np.percentile(low_band, 5.0))
        base_center_z = base_bottom_z + GRID_BASE_SIZE[2] * 0.5

        self._grid_model_pose = {
            "x": float(center_x),
            "y": float(center_y),
            "base_z": float(base_center_z),
            "yaw": float(yaw),
            "points": int(high.sum()),
            "width": float(width),
            "depth": float(depth),
            "visible_bottom_z": visible_bottom_z,
            "visible_top_z": visible_top_z,
            "base_bottom_z": base_bottom_z,
            "footprint_points": int(footprint.sum()),
            "base_z_source": base_z_source,
            "base_z_points": int(len(model_z)),
            "raw_points": int(raw_points),
            "corridor_points": int(corridor_points),
            "foreground_points": int(len(fx)),
            "gate_error": yaw_gate.get("error"),
            "gate_pass": yaw_gate.get("pass"),
            "position_gate_pass": position_gate.get("pass"),
            "position_gate_dx": position_gate.get("dx"),
            "position_gate_dy": position_gate.get("dy"),
            "accum_enabled": self._grid_fit_accumulate_frames_enable,
            "accum_frames": len(self._grid_accum_frames),
            "accum_points": int(len(x)),
            "depth_support_enabled": self._grid_depth_support_filter_enable,
            "depth_support_bins": int(support_bins),
            "depth_support_points": int(len(support_lx)),
        }
        if self._grid_model_freeze_after_first:
            self._grid_model_locked = True
        self._log_grid_model_fit()

    def _supported_depth_points(self, lx, ly):
        if not self._grid_depth_support_filter_enable:
            return None
        if len(lx) < GRID_DETECT_MIN_POINTS:
            return None

        bin_size = self._grid_depth_support_bin_size
        bins = np.floor(lx / bin_size).astype(np.int32)
        keep_bins = []
        for bin_id in np.unique(bins):
            in_bin = bins == bin_id
            if int(in_bin.sum()) < self._grid_depth_support_min_points:
                continue
            y_span = self._robust_span(ly[in_bin])
            if y_span < self._grid_depth_support_min_y_span:
                continue
            keep_bins.append(int(bin_id))
        if len(keep_bins) < self._grid_depth_support_min_bins:
            return None

        keep = np.isin(bins, np.asarray(keep_bins, dtype=np.int32))
        if int(keep.sum()) < GRID_DETECT_MIN_POINTS:
            return None
        return lx[keep], ly[keep], len(keep_bins)

    def _arrow_corridor_points(self, x, y, z):
        if not self._grid_arrow_corridor_enable:
            return x, y, z
        sx, sy, _ = self._retry_rel
        gx, gy, _ = self._grid_center_rel
        arrow_len = math.hypot(gx - sx, gy - sy)
        if arrow_len <= 1e-6:
            return x, y, z

        sx_odom, sy_odom, _ = self._local_point(sx, sy, 0.0)
        yaw = self._retry_to_grid_yaw()
        dx = x - sx_odom
        dy = y - sy_odom
        c = math.cos(yaw)
        s = math.sin(yaw)
        forward = c * dx + s * dy
        lateral = -s * dx + c * dy
        keep = (
            (forward >= arrow_len - self._grid_arrow_corridor_forward_margin)
            & (forward <= arrow_len + self._grid_arrow_corridor_forward_margin)
            & (np.abs(lateral) <= self._grid_arrow_corridor_half_width)
        )
        self._log_arrow_filter(len(x), int(keep.sum()), arrow_len)
        if int(keep.sum()) < GRID_DETECT_MIN_POINTS:
            return x, y, z
        return x[keep], y[keep], z[keep]

    def _log_arrow_filter(self, raw_points: int, kept_points: int, arrow_len: float):
        now = time.monotonic()
        if now - self._last_arrow_filter_log_t < 1.0:
            return
        self._last_arrow_filter_log_t = now
        self._log_row(
            "grid_arrow_filter",
            self._latest_odom or {},
            fit=self._fit,
            gate=self._last_yaw_gate_state,
            position_gate=self._last_position_gate_state,
            roi_points=self._last_roi_points,
            message=(
                f"raw={raw_points} kept={kept_points} arrow_len={arrow_len:.3f} "
                f"half_width={self._grid_arrow_corridor_half_width:.3f}"
            ),
        )

    def _foreground_ray_points(self, x, y, z):
        if not self._grid_foreground_ray_filter_enable:
            return x, y, z
        odom = self._latest_odom
        if odom is None or self._grid_foreground_ray_bin <= 0.0:
            return x, y, z

        dx = x - odom["x"]
        dy = y - odom["y"]
        ranges = np.hypot(dx, dy)
        finite = np.isfinite(ranges) & (ranges > 0.05)
        if int(finite.sum()) < GRID_DETECT_MIN_POINTS:
            return x, y, z

        angles = np.arctan2(dy[finite], dx[finite])
        ray_bins = np.floor((angles + math.pi) / self._grid_foreground_ray_bin).astype(np.int32)
        valid_ranges = ranges[finite]
        order = np.argsort(valid_ranges)
        sorted_bins = ray_bins[order]
        sorted_ranges = valid_ranges[order]
        unique_bins, first_idx = np.unique(sorted_bins, return_index=True)
        min_by_bin = dict(zip(unique_bins.tolist(), sorted_ranges[first_idx].tolist()))

        keep_finite = np.zeros(int(finite.sum()), dtype=bool)
        for i, (bin_id, rng) in enumerate(zip(ray_bins, valid_ranges)):
            if rng <= min_by_bin[int(bin_id)] + self._grid_foreground_ray_keep_depth:
                keep_finite[i] = True
        keep = np.zeros(len(x), dtype=bool)
        keep[np.flatnonzero(finite)] = keep_finite
        if int(keep.sum()) < GRID_DETECT_MIN_POINTS:
            return x, y, z
        return x[keep], y[keep], z[keep]

    def _log_grid_model_fit(self):
        now = time.monotonic()
        if now - self._last_grid_model_log_t < 1.0:
            return
        self._last_grid_model_log_t = now
        p = self._grid_model_pose
        self._log_row(
            "grid_model_fit",
            self._latest_odom or {},
            fit=self._fit,
            gate=self._last_yaw_gate_state,
            position_gate=self._last_position_gate_state,
            accum_frames=len(self._grid_accum_frames),
            accum_points=p.get("accum_points"),
            roi_points=self._last_roi_points,
            grid_pose=p,
        )

    @staticmethod
    def _robust_span(values):
        return float(np.percentile(values, 97.0) - np.percentile(values, 3.0))

    @staticmethod
    def _normalize_half_turn(angle):
        while angle <= -math.pi / 2.0:
            angle += math.pi
        while angle > math.pi / 2.0:
            angle -= math.pi
        return angle

    @staticmethod
    def _norm_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _active_grid_roi_bounds(self):
        x0, x1 = self._grid_roi_x
        y0, y1 = self._grid_roi_y
        if self._fit and self._fit.get("source") == "ramp_weak":
            x0 -= GRID_WEAK_ROI_MARGIN_X
            x1 += GRID_WEAK_ROI_MARGIN_X
            y0 -= GRID_WEAK_ROI_MARGIN_Y
            y1 += GRID_WEAK_ROI_MARGIN_Y
        return x0, x1, y0, y1

    def _odom_to_root_xy(self, x, y):
        # _local_point() 的 XY 反变换：
        # odom 点 -> 减去已锁定 root 平移 -> 旋转 -root_yaw。
        # 输出的 lx/ly 可以直接和当前队伍镜像后的 GRID_ROI_X/Y 比较。
        dx = x - self._fit["root_x"]
        dy = y - self._fit["root_y"]
        c = math.cos(self._fit["root_yaw"])
        s = math.sin(self._fit["root_yaw"])
        return c * dx + s * dy, -s * dx + c * dy

    @staticmethod
    def _height_colors(z):
        r = np.full(len(z), 120, dtype=np.uint8)
        g = np.full(len(z), 120, dtype=np.uint8)
        b = np.full(len(z), 120, dtype=np.uint8)
        ground = z < 0.08
        base = (z >= 0.08) & (z < 0.55)
        mid = (z >= 0.55) & (z < 0.80)
        high1 = (z >= 0.80) & (z < 1.34)
        high2 = (z >= 1.34) & (z < 1.88)
        high3 = z >= 1.88
        r[ground], g[ground], b[ground] = 210, 210, 210
        r[base], g[base], b[base] = 255, 230, 0
        r[mid], g[mid], b[mid] = 0, 200, 200
        r[high1], g[high1], b[high1] = 255, 80, 20
        r[high2], g[high2], b[high2] = 80, 220, 80
        r[high3], g[high3], b[high3] = 200, 80, 255
        return r, g, b

    def _to_odom_xyz(self, header, x, y, z):
        src = header.frame_id
        if not src or src == "odom":
            return (
                x.astype(np.float64, copy=False),
                y.astype(np.float64, copy=False),
                z.astype(np.float64, copy=False),
            )
        try:
            t = self._tf_buffer.lookup_transform("odom", src, Time())
        except Exception as e:
            self._cloud_warn(f"TF {src}->odom unavailable: {e}")
            return None, None, None
        q = t.transform.rotation
        tx = float(t.transform.translation.x)
        ty = float(t.transform.translation.y)
        tz = float(t.transform.translation.z)
        xx, yy, zz = q.x * q.x, q.y * q.y, q.z * q.z
        xy, xz, yz = q.x * q.y, q.x * q.z, q.y * q.z
        wx, wy, wz = q.w * q.x, q.w * q.y, q.w * q.z
        r00 = 1.0 - 2.0 * (yy + zz)
        r01 = 2.0 * (xy - wz)
        r02 = 2.0 * (xz + wy)
        r10 = 2.0 * (xy + wz)
        r11 = 1.0 - 2.0 * (xx + zz)
        r12 = 2.0 * (yz - wx)
        r20 = 2.0 * (xz - wy)
        r21 = 2.0 * (yz + wx)
        r22 = 1.0 - 2.0 * (xx + yy)
        xf = x.astype(np.float64, copy=False)
        yf = y.astype(np.float64, copy=False)
        zf = z.astype(np.float64, copy=False)
        return (
            r00 * xf + r01 * yf + r02 * zf + tx,
            r10 * xf + r11 * yf + r12 * zf + ty,
            r20 * xf + r21 * yf + r22 * zf + tz,
        )

    @staticmethod
    def _parse_xyz(cloud: PointCloud2):
        offsets = {}
        for field in cloud.fields:
            if field.name in ("x", "y", "z"):
                offsets[field.name] = field.offset
        if len(offsets) != 3:
            raise ValueError("PointCloud2 missing x/y/z")
        n = cloud.width * cloud.height if cloud.height > 1 else cloud.width
        dt = np.dtype({
            "names": ["x", "y", "z"],
            "formats": [np.float32, np.float32, np.float32],
            "offsets": [offsets["x"], offsets["y"], offsets["z"]],
            "itemsize": cloud.point_step,
        })
        pts = np.frombuffer(cloud.data, dtype=dt, count=n)
        return pts["x"], pts["y"], pts["z"]

    @classmethod
    def _empty_cloud(cls, stamp) -> PointCloud2:
        empty_f = np.empty(0, dtype=np.float32)
        empty_u = np.empty(0, dtype=np.uint8)
        return cls._make_rgb_cloud(stamp, empty_f, empty_f, empty_f, empty_u, empty_u, empty_u)

    @staticmethod
    def _make_rgb_cloud(stamp, x, y, z, r, g, b) -> PointCloud2:
        n = len(x)
        header = Header()
        header.stamp = stamp
        header.frame_id = "odom"
        a = np.full(n, 255, dtype=np.uint8)
        pts = np.zeros(n, dtype=[
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("rgb", np.uint32),
        ])
        pts["x"] = np.asarray(x, dtype=np.float32)
        pts["y"] = np.asarray(y, dtype=np.float32)
        pts["z"] = np.asarray(z, dtype=np.float32)
        pts["rgb"] = (
            (a.astype(np.uint32) << 24)
            | (np.asarray(r, dtype=np.uint8).astype(np.uint32) << 16)
            | (np.asarray(g, dtype=np.uint8).astype(np.uint32) << 8)
            | np.asarray(b, dtype=np.uint8).astype(np.uint32)
        )
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.data = pts.tobytes()
        msg.is_dense = True
        return msg

    def _log_stamp(self) -> str:
        if self._detail_log_timestamp:
            return self._detail_log_timestamp
        return datetime.now().strftime("%Y%m%d_%H%M%S")

    def _open_logs(self) -> tuple[str | None, str | None]:
        if not self._file_log_enabled:
            return None, None
        os.makedirs(self._log_dir, exist_ok=True)
        prefix = f"{self._detail_log_name}_{self._team_label}_{self._log_stamp()}"
        detail_path = os.path.join(self._log_dir, f"{prefix}_detail.csv")
        heartbeat_path = os.path.join(self._log_dir, f"{prefix}_heartbeat.csv")

        with open(detail_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "event",
                "stamp_t",
                "odom_x",
                "odom_y",
                "odom_z",
                "odom_roll_deg",
                "odom_pitch_deg",
                "odom_yaw_deg",
                "low_x",
                "low_y",
                "low_z",
                "top_x",
                "top_y",
                "top_z",
                "z_gain",
                "xy_dist",
                "pitch_abs_deg",
                "age",
                "residual",
                "ramp_yaw_deg",
                "root_x",
                "root_y",
                "root_yaw_deg",
                "retry_x",
                "retry_y",
                "retry_z",
                "yaw_gate_enabled",
                "yaw_gate_pass",
                "yaw_gate_reason",
                "yaw_gate_robot_yaw_deg",
                "yaw_gate_target_yaw_deg",
                "yaw_gate_error_deg",
                "yaw_gate_tol_deg",
                "position_gate_enabled",
                "position_gate_pass",
                "position_gate_reason",
                "position_gate_local_x",
                "position_gate_local_y",
                "position_gate_dx",
                "position_gate_dy",
                "position_gate_size_x",
                "position_gate_size_y",
                "accum_frames",
                "accum_points",
                "roi_points",
                "root_tf_x",
                "root_tf_y",
                "root_tf_z",
                "root_tf_yaw_deg",
                "grid_x",
                "grid_y",
                "grid_base_z",
                "grid_yaw_deg",
                "grid_width",
                "grid_depth",
                "grid_raw_points",
                "grid_corridor_points",
                "grid_foreground_points",
                "grid_high_points",
                "grid_footprint_points",
                "message",
            ])

        with open(heartbeat_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "seq",
                "wall_time",
                "ros_time",
                "team",
                "ramp_event",
                "ramp_locked",
                "grid_locked",
                "latest_odom_x",
                "latest_odom_y",
                "latest_odom_z",
                "latest_odom_yaw_deg",
                "roi_points",
                "yaw_gate_pass",
                "yaw_gate_error_deg",
                "position_gate_pass",
                "position_gate_dx",
                "position_gate_dy",
                "tf_child",
            ])
        return detail_path, heartbeat_path

    def _write_heartbeat_log(self):
        if not self._file_log_enabled or self._heartbeat_log_path is None:
            return
        odom = self._latest_odom or {}
        yaw_gate = self._last_yaw_gate_state or {}
        position_gate = self._last_position_gate_state or {}
        self._heartbeat_seq += 1
        with open(self._heartbeat_log_path, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                self._heartbeat_seq,
                f"{time.time():.6f}",
                f"{self.get_clock().now().nanoseconds * 1e-9:.6f}",
                self._team_label,
                int(self._ramp_event_since is not None),
                int(self._fit is not None),
                int(self._grid_model_locked),
                self._fmt(odom.get("x")),
                self._fmt(odom.get("y")),
                self._fmt(odom.get("z")),
                self._fmt_deg(odom.get("yaw")),
                self._last_roi_points,
                int(yaw_gate["pass"]) if "pass" in yaw_gate else "",
                self._fmt_deg(yaw_gate.get("error")),
                int(position_gate["pass"]) if "pass" in position_gate else "",
                self._fmt(position_gate.get("dx")),
                self._fmt(position_gate.get("dy")),
                self._zone3_root_tf_child_frame,
            ])

    def _detail_gate_open(self, gate: dict | None, position_gate: dict | None) -> bool:
        if self._fit is None:
            return True
        gate = gate if gate is not None else self._last_yaw_gate_state
        position_gate = position_gate if position_gate is not None else self._last_position_gate_state
        return bool(gate and gate.get("pass") and position_gate and position_gate.get("pass"))

    def _should_write_detail(
        self,
        event: str,
        gate: dict | None,
        position_gate: dict | None,
        force: bool,
    ) -> bool:
        if not self._file_log_enabled or self._detail_log_path is None:
            return False
        if force or event in ("ramp_event", "lock_retry_marker"):
            return True
        if not self._detail_gate_open(gate, position_gate):
            return False
        now = time.monotonic()
        if now - self._last_detail_log_t < 1.0 / DETAIL_LOG_HZ:
            return False
        self._last_detail_log_t = now
        return True

    def _log_row(
        self,
        event: str,
        odom: dict,
        *,
        low: dict | None = None,
        top: dict | None = None,
        fit: dict | None = None,
        retry: tuple[float, float, float] | None = None,
        z_gain: float | None = None,
        xy_dist: float | None = None,
        pitch_abs: float | None = None,
        age: float | None = None,
        residual: float | None = None,
        ramp_yaw: float | None = None,
        root_yaw: float | None = None,
        gate: dict | None = None,
        position_gate: dict | None = None,
        accum_frames: int | None = None,
        accum_points: int | None = None,
        roi_points: int | None = None,
        root_tf: tuple[float, float, float, float] | None = None,
        grid_pose: dict | None = None,
        message: str = "",
        force: bool = False,
    ):
        if not self._should_write_detail(event, gate, position_gate, force):
            return
        if fit is not None:
            z_gain = fit.get("z_gain", z_gain)
            xy_dist = fit.get("xy_dist", xy_dist)
            residual = fit.get("residual", residual)
            ramp_yaw = fit.get("ramp_yaw", ramp_yaw)
            root_yaw = fit.get("root_yaw", root_yaw)
        root_x = fit.get("root_x") if fit is not None else None
        root_y = fit.get("root_y") if fit is not None else None
        retry = retry or (None, None, None)
        gate = gate or {}
        position_gate = position_gate or {}
        root_tf = root_tf or (None, None, None, None)
        grid_pose = grid_pose or {}
        with open(self._detail_log_path, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                event,
                self._fmt(odom.get("t")),
                self._fmt(odom.get("x")),
                self._fmt(odom.get("y")),
                self._fmt(odom.get("z")),
                self._fmt_deg(odom.get("roll")),
                self._fmt_deg(odom.get("pitch")),
                self._fmt_deg(odom.get("yaw")),
                self._fmt(low.get("x") if low else None),
                self._fmt(low.get("y") if low else None),
                self._fmt(low.get("z") if low else None),
                self._fmt(top.get("x") if top else None),
                self._fmt(top.get("y") if top else None),
                self._fmt(top.get("z") if top else None),
                self._fmt(z_gain),
                self._fmt(xy_dist),
                self._fmt_deg(pitch_abs),
                self._fmt(age),
                self._fmt(residual),
                self._fmt_deg(ramp_yaw),
                self._fmt(root_x),
                self._fmt(root_y),
                self._fmt_deg(root_yaw),
                self._fmt(retry[0]),
                self._fmt(retry[1]),
                self._fmt(retry[2]),
                int(gate["enabled"]) if "enabled" in gate else "",
                int(gate["pass"]) if "pass" in gate else "",
                gate.get("reason", ""),
                self._fmt_deg(gate.get("robot_yaw")),
                self._fmt_deg(gate.get("target_yaw")),
                self._fmt_deg(gate.get("error")),
                self._fmt_deg(gate.get("tol")),
                int(position_gate["enabled"]) if "enabled" in position_gate else "",
                int(position_gate["pass"]) if "pass" in position_gate else "",
                position_gate.get("reason", ""),
                self._fmt(position_gate.get("local_x")),
                self._fmt(position_gate.get("local_y")),
                self._fmt(position_gate.get("dx")),
                self._fmt(position_gate.get("dy")),
                self._fmt(position_gate.get("size_x")),
                self._fmt(position_gate.get("size_y")),
                accum_frames if accum_frames is not None else "",
                accum_points if accum_points is not None else "",
                roi_points if roi_points is not None else "",
                self._fmt(root_tf[0]),
                self._fmt(root_tf[1]),
                self._fmt(root_tf[2]),
                self._fmt_deg(root_tf[3]),
                self._fmt(grid_pose.get("x")),
                self._fmt(grid_pose.get("y")),
                self._fmt(grid_pose.get("base_z")),
                self._fmt_deg(grid_pose.get("yaw")),
                self._fmt(grid_pose.get("width")),
                self._fmt(grid_pose.get("depth")),
                grid_pose.get("raw_points", ""),
                grid_pose.get("corridor_points", ""),
                grid_pose.get("foreground_points", ""),
                grid_pose.get("points", ""),
                grid_pose.get("footprint_points", ""),
                message,
            ])

    @staticmethod
    def _fmt(value):
        if value is None:
            return ""
        return f"{float(value):.6f}"

    @staticmethod
    def _fmt_deg(value):
        if value is None:
            return ""
        return f"{math.degrees(float(value)):.6f}"


def main():
    rclpy.init()
    node = Zone3GridLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
