#!/usr/bin/env python3
"""Zone3 grid localization node.

Detects the high 3x3 grid from /odin1/cloud_slam and publishes a dynamic
odom -> {team}_zone3_root transform. Debug clouds are published for RViz.
"""

import csv
import math
import os
import time
from dataclasses import dataclass
from datetime import datetime

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker


# ============================== 可调参数区 ==============================
#
# 检测流程：
#   1. 在输入点云 frame 中做粗 ROI；
#   2. 把高度统一投影到 odom，得到离地高度 h；
#   3. 只保留九宫格可能出现的高位结构点；
#   4. 在 XY 平面做连通域和九宫格几何评分；
#   5. 连续稳定后锁定并持续发布 odom -> {team}_zone3_root。
#
# 调参顺序：
#   - 检测不到候选：先放宽 ROI 或高位高度范围；
#   - 候选太多误检：提高 MIN_LOCK_CONFIDENCE，或收紧候选宽度/深度判断；
#   - 锁定抖动：增大 STABLE_LOCK_COUNT，或减小 STABLE_CENTER_TOL；
#   - TF 对不上场地模型：先检查 IS_BLUE_TEAM 和 GRID_CENTER_REL_*。

# -------------------------------------------------------------------------
# 1) 输入点云 ROI 裁剪 (odin1_base_link 系)
# -------------------------------------------------------------------------
# X/Y 范围取决于点云 source frame。
#   odin1_base_link 系: X=车前, Y=车左 (默认)
#   odom 系:            X=odom X, Y=odom Y
# 在 _roi_for_frame() 中根据 header.frame_id 自动切换。
X_MIN, X_MAX = 0.15, 5.50
Y_MIN, Y_MAX = -2.20, 2.20
Z_MIN, Z_MAX = -1.50, 2.80
# cloud_slam (odom 系) 下的 ROI: Z3 重试区附近必须覆盖到九宫格
ODOM_X_MIN, ODOM_X_MAX = 0.0, 16.0
ODOM_Y_MIN, ODOM_Y_MAX = -4.0, 4.0

# -------------------------------------------------------------------------
# 2) 坐标系与高度归一
# -------------------------------------------------------------------------
# 离地高度 h 投影到这个 frame 的 Z 轴；当前用 odom 是为了匹配 Odin TF。
HEIGHT_FRAME = "odom"
# 动态 TF 的父 frame。本节点按需求维护 odom -> zone3_root。
SOURCE_FIXED_FRAME = "odom"

# -------------------------------------------------------------------------
# 3) 地面估计
# -------------------------------------------------------------------------
# 1 使用手动 GROUND_Z；0 用高度直方图自动估计。Z3 平台/坡道地面回波不稳，
# 比赛调试建议保持手动模式。
GROUND_Z_KNOWN = 1
# 手动地面高度，单位 m，坐标系为 HEIGHT_FRAME。
GROUND_Z = -0.270
# 仅用于调试点云中把地面染白，不参与九宫格拟合。
GROUND_TOLERANCE = 0.035
# 自动地面估计直方图 bin 宽度。
HISTOGRAM_BIN_WIDTH = 0.02
# 自动估计时，从低到高找第一个达到主峰该比例的 bin，减少高结构误当地面。
GROUND_PEAK_RATIO = 0.15
# 自动地面更新的 EMA 系数；越大跟随越快，越小越稳。
GROUND_UPDATE_ALPHA = 0.18
# 自动更新允许的最大单帧地面跳变。
GROUND_MAX_UPDATE_STEP = 0.06

# -------------------------------------------------------------------------
# 4) 队伍与 TF 命名
# -------------------------------------------------------------------------
# 1/True  -> blue_zone3_root
# 0/False -> red_zone3_root
# zone3_root 在 __init__ 中根据 IS_BLUE_TEAM 运行时参数动态计算。
IS_BLUE_TEAM = 0

# -------------------------------------------------------------------------
# 5) 九宫格场地先验 (来自 rc26_field.py)
# -------------------------------------------------------------------------
# zone3_root 在场地模型中的绝对坐标 (rc26_field.py ZONE3_ROOT)
BLUE_ZONE3_ROOT_X = 3.025
RED_ZONE3_ROOT_X = -3.025
ZONE3_ROOT_FIELD_Y = -4.60
# 九宫格中心在场地模型中的坐标, 用于反推 zone3_root 位姿
GRID_FIELD_X = 0.0
GRID_FIELD_Y = -4.75
# 九宫格几何尺寸
GRID_WIDTH_Y = 1.62
GRID_DEPTH_X = 0.32
# 高位点筛选高度范围 (离地 h)
GRID_MIN_H = 0.75
GRID_MAX_H = 2.60

# -------------------------------------------------------------------------
# 6) 重试区门控 + Z2 先验
# -------------------------------------------------------------------------
# 机器人必须位于 Z3 重试区域内才检测九宫格, 避免提前误检。
REQUIRE_ZONE3_ODOM_GATE = 1

# 重试区位置 (odom 系, 红蓝独立)
BLUE_RETRY_CENTER_X = 11.100 - 0.4
BLUE_RETRY_CENTER_Y = 4.100
RED_RETRY_CENTER_X = 11.100
RED_RETRY_CENTER_Y = -4.100

# 重试区尺寸
RETRY_AREA_HALF_SIZE = 0.8

# 重试区 TF/Marker 发布高度 (odom 系 Z)
RETRY_AREA_Z = 0.3

# 重试区 TF 朝向九宫格的 yaw (odom 系, 红蓝独立, 基于场地模型推算)
# BLUE: atan2(-5.525, -0.75) ≈ -1.708 rad, RED: atan2(5.525, -0.75) ≈ 1.708 rad
BLUE_RETRY_FACE_YAW = -1.708
RED_RETRY_FACE_YAW = 1.708

# 朝向门控容差: 机器人朝向与 RETRY_FACE_YAW 偏差超过此值则不开始拟合
RETRY_FACE_YAW_TOLERANCE = math.radians(45)

# 机器人 TF frame
ROBOT_TF_FRAME = "odin1_base_link"

# Z2 先验: 第一次进入重试区时, 读取 Z2 TF 做 Z3 初始锁定
ENABLE_Z2_PRIOR = 1

# Z2→Z3 在场地模型中的固定平移 (zone3_root - zone2_root)
# Z2_ROOT = (±3.025, 0.55), Z3_ROOT = (±3.025, -4.60) → 偏移 (0, -5.15)
Z2_TO_Z3_OFFSET_X = 0.0
Z2_TO_Z3_OFFSET_Y = -5.15

# Z3 精修参数 (到达重试区后, 在 Z2 初锁基础上小范围修正)
Z3_REFINE_MAX_TRANSLATION = 0.40   # 最大平移修正 (米)
Z3_REFINE_MAX_YAW_DELTA = math.radians(8.0)  # 最大 yaw 修正
Z3_REFINE_ALPHA = 0.65             # EMA 平滑系数
Z3_REFINE_STABLE_COUNT = 3         # 精修稳定帧数

# -------------------------------------------------------------------------
# 7) 锁定策略
# -------------------------------------------------------------------------
# 单帧几何评分阈值。
MIN_LOCK_CONFIDENCE = 0.76
# 连续多少个合格候选稳定后才锁定 TF。
STABLE_LOCK_COUNT = 4
# 连续候选 root 平移/yaw 允许的最大离散。
STABLE_CENTER_TOL = 0.35
STABLE_YAW_TOL = math.radians(12.0)
# 锁定后动态 TF 重发频率。
DYNAMIC_TF_RATE = 10.0
# INFO 日志节流间隔 (CSV 仍逐帧写入)
LOG_INTERVAL = 0.5
# 点云降采样步长 (1=不降采样)
DOWNSAMPLE_STEP = 1
# 累计最近 N 帧高位候选点再拟合, 提升稀疏点云下的连通性
ACCUMULATE_FRAMES = 20

# -------------------------------------------------------------------------
# 8) 调试输出
# -------------------------------------------------------------------------
# 详细检测日志目录。默认不写文件，可通过
#   -p detailed_file_log:=true
# 打开逐帧 CSV 记录。
DETAILED_FILE_LOG = False
DEBUG_DIR = "/home/inkc/inkc/Rc2026/files/record/logs/zone_detection"
# 逐帧记录检测分数、九宫格中心、root 位姿、宽度/深度/层数，便于离线复盘。
DEBUG_CSV_PREFIX = "zone3_debug"

# 调试可视化总开关: 0=关闭 (不创建/发布调试点云/TF/Marker), 1=开启
ENABLE_DEBUG_VIS = 1

# 高度诊断日志: 逐帧记录各高度区间的点数分布 / ground_z / drift
# 用于排查层2/层3误染为层1的问题, 输出到 DEBUG_DIR 下的 zone3_height_diag_*.csv
ENABLE_HEIGHT_DIAG_LOG = 0
HEIGHT_DIAG_LOG_DIR = "/home/inkc/inkc/Rc2026/files/record/logs/zone_detection"

# 高度区间调色板: (h_min, h_max, r, g, b) — 用于调试标定 ground_z 和层高
# 每个点根据离地高度 h 落入哪个区间就染对应颜色, 超出所有区间的点不显示
HEIGHT_BANDS = [
    (-0.10, 0.07, 180, 180, 180),    # 地面附近: 灰色
    (0.07, 0.50, 255, 255, 0),       # 基座/平台顶: 黄色
    (0.50, 0.80, 0, 200, 200),       # 基座~层1间隙: 青色
    (0.80, 1.34, 255, 80, 20),       # 层1块: 橙色
    (1.34, 1.88, 80, 200, 80),       # 层2块: 绿色
    (1.88, 2.50, 200, 80, 255),      # 层3块: 紫色
]


def _make_detail_log_stamp():
    return datetime.now().strftime("%m%d_%H%M")


@dataclass
class GridDetection:
    center_x: float
    center_y: float
    yaw: float
    confidence: float
    point_count: int
    width: float
    depth: float
    layer_count: int
    mask: np.ndarray


def choose_team_root_pose(grid_x, grid_y, grid_yaw,
                          grid_center_rel_x, grid_center_rel_y,
                          is_blue_team):
    candidates = []
    for yaw in (grid_yaw, _norm_angle(grid_yaw + math.pi)):
        c = math.cos(yaw)
        s = math.sin(yaw)
        root_x = grid_x - (c * grid_center_rel_x - s * grid_center_rel_y)
        root_y = grid_y - (s * grid_center_rel_x + c * grid_center_rel_y)
        expected_side = 1.0 if is_blue_team else -1.0
        candidates.append((expected_side * root_y, root_x, root_x, root_y, yaw))
    candidates.sort(key=lambda item: (item[0], item[1]), reverse=True)
    _side_score, _forward_score, root_x, root_y, yaw = candidates[0]
    return float(root_x), float(root_y), float(yaw)




def detect_grid_pose(x, y, h):
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    h = np.asarray(h, dtype=np.float64)
    n = min(len(x), len(y), len(h))
    if n == 0:
        return None
    x = x[:n]
    y = y[:n]
    h = h[:n]

    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(h)
    high = finite & (h >= GRID_MIN_H) & (h <= GRID_MAX_H)
    if int(high.sum()) < 40:
        return None

    hx = x[high]
    hy = y[high]
    hh = h[high]
    comps = _connected_components(hx, hy, cell=0.08, min_cell_points=2)
    if not comps:
        return None

    best = None
    best_score = -1.0
    for comp in comps:
        if len(comp) < 40:
            continue
        result = _fit_component(hx[comp], hy[comp], hh[comp], len(x), np.flatnonzero(high)[comp])
        if result is None:
            continue
        score = result.confidence * math.log1p(result.point_count)
        if score > best_score:
            best = result
            best_score = score
    return best


def _fit_component(x, y, h, total_count, original_indices):
    pts = np.column_stack((x, y))
    center0 = pts.mean(axis=0)
    demean = pts - center0
    if len(pts) < 3:
        return None
    cov = np.cov(demean.T)
    if not np.all(np.isfinite(cov)):
        return None
    vals, vecs = np.linalg.eigh(cov)
    long_axis = vecs[:, int(np.argmax(vals))]
    long_axis /= max(1e-9, float(np.hypot(long_axis[0], long_axis[1])))

    yaw = _normalize_half_turn(math.atan2(long_axis[1], long_axis[0]) - math.pi / 2.0)
    c = math.cos(yaw)
    s = math.sin(yaw)
    lx = c * (x - center0[0]) + s * (y - center0[1])
    ly = -s * (x - center0[0]) + c * (y - center0[1])

    width = _robust_span(ly)
    depth = _robust_span(lx)
    if width < 0.65 or width > 2.35:
        return None
    if depth > 0.80:
        return None

    local_center_x = 0.5 * (_percentile(lx, 5.0) + _percentile(lx, 95.0))
    local_center_y = 0.5 * (_percentile(ly, 3.0) + _percentile(ly, 97.0))
    center_x = center0[0] + c * local_center_x - s * local_center_y
    center_y = center0[1] + s * local_center_x + c * local_center_y

    layer_count = _count_height_layers(h)
    width_score = math.exp(-abs(width - GRID_WIDTH_Y) / 0.42)
    depth_score = math.exp(-max(0.0, depth - 0.55) / 0.35)
    layer_score = min(1.0, layer_count / 3.0)
    density_score = min(1.0, len(x) / 420.0)
    confidence = 0.40 * width_score + 0.10 * depth_score + 0.35 * layer_score + 0.15 * density_score
    if confidence < 0.25:
        return None

    mask = np.zeros(total_count, dtype=bool)
    mask[original_indices] = True
    return GridDetection(
        center_x=float(center_x),
        center_y=float(center_y),
        yaw=float(yaw),
        confidence=float(confidence),
        point_count=int(len(x)),
        width=float(width),
        depth=float(depth),
        layer_count=int(layer_count),
        mask=mask,
    )


def _connected_components(x, y, cell=0.08, min_cell_points=2):
    ix = np.floor(x / cell).astype(np.int32)
    iy = np.floor(y / cell).astype(np.int32)
    keys = np.column_stack((ix, iy))
    unique, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    active_cell = counts >= min_cell_points
    active_points = active_cell[inv]
    if not active_points.any():
        return []

    cell_to_points = {}
    for pi, ci in enumerate(inv):
        if active_cell[ci]:
            cell_to_points.setdefault(int(ci), []).append(pi)

    coord_to_ci = {tuple(coord): int(i) for i, coord in enumerate(unique) if active_cell[i]}
    visited = set()
    comps = []
    for ci in list(cell_to_points):
        if ci in visited:
            continue
        stack = [ci]
        visited.add(ci)
        point_ids = []
        while stack:
            cur = stack.pop()
            point_ids.extend(cell_to_points[cur])
            cx, cy = unique[cur]
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nb = coord_to_ci.get((int(cx + dx), int(cy + dy)))
                    if nb is not None and nb not in visited:
                        visited.add(nb)
                        stack.append(nb)
        comps.append(np.asarray(point_ids, dtype=np.int32))
    comps.sort(key=len, reverse=True)
    return comps


def _count_height_layers(h):
    layer_ranges = ((0.80, 1.34), (1.34, 1.88), (1.88, 2.42))
    return sum(int(((h >= lo) & (h < hi)).sum()) >= 30 for lo, hi in layer_ranges)


def _robust_span(values):
    return float(_percentile(values, 97.0) - _percentile(values, 3.0))


def _percentile(values, p):
    return float(np.percentile(values, p))


def _normalize_half_turn(angle):
    while angle <= -math.pi / 2.0:
        angle += math.pi
    while angle > math.pi / 2.0:
        angle -= math.pi
    return angle


def _norm_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Zone3GridLocalizer(Node):
    def __init__(self):
        super().__init__("zone3_grid_localizer")

        self.sub = self.create_subscription(
            PointCloud2, "/odin1/cloud_slam", self.cloud_cb, 10)
        self._enable_debug_vis = bool(
            self.declare_parameter("enable_debug_vis", bool(ENABLE_DEBUG_VIS)).value)
        if self._enable_debug_vis:
            self._pub_candidates = self.create_publisher(
                PointCloud2, "/rc26/zone3/cloud_grid_candidates", 10)
            self._pub_model = self.create_publisher(
                PointCloud2, "/rc26/zone3/cloud_grid_model", 10)
            self._pub_retry_marker = self.create_publisher(
                Marker, "/rc26/zone3/retry_area_marker", 10)
            self._pub_height_bands = self.create_publisher(
                PointCloud2, "/rc26/zone3/cloud_height_bands", 10)
        else:
            self._pub_candidates = None
            self._pub_model = None
            self._pub_retry_marker = None
            self._pub_height_bands = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self._ground_z = GROUND_Z if GROUND_Z_KNOWN else None
        self._last_log = 0.0
        self._last_tf_warn = 0.0
        self._last_gate_log = 0.0
        self._frames = []
        self._locked = False
        self._is_blue_team = bool(
            self.declare_parameter("is_blue_team", bool(IS_BLUE_TEAM)).value)
        self._detailed_file_log = bool(
            self.declare_parameter("detailed_file_log", bool(DETAILED_FILE_LOG)).value)
        self._debug_csv = ""
        self._require_odom_gate = bool(
            self.declare_parameter("require_zone3_odom_gate", bool(REQUIRE_ZONE3_ODOM_GATE)).value)
        if self._is_blue_team:
            def_ret_x = BLUE_RETRY_CENTER_X
            def_ret_y = BLUE_RETRY_CENTER_Y
        else:
            def_ret_x = RED_RETRY_CENTER_X
            def_ret_y = RED_RETRY_CENTER_Y
        self._retry_center_x = float(
            self.declare_parameter("retry_center_x", def_ret_x).value)
        self._retry_center_y = float(
            self.declare_parameter("retry_center_y", def_ret_y).value)
        self._retry_half_size = float(
            self.declare_parameter("retry_half_size", RETRY_AREA_HALF_SIZE).value)
        self._retry_z = float(
            self.declare_parameter("retry_z", RETRY_AREA_Z).value)
        self._retry_face_yaw = float(
            self.declare_parameter(
                "retry_face_yaw",
                BLUE_RETRY_FACE_YAW if self._is_blue_team else RED_RETRY_FACE_YAW).value)
        self._retry_face_yaw_tol = float(
            self.declare_parameter("retry_face_yaw_tol", float(RETRY_FACE_YAW_TOLERANCE)).value)
        self._enable_z2_prior = bool(
            self.declare_parameter("enable_z2_prior", bool(ENABLE_Z2_PRIOR)).value)
        self._z3_refine_max_translation = float(
            self.declare_parameter("z3_refine_max_translation", Z3_REFINE_MAX_TRANSLATION).value)
        self._z3_refine_max_yaw_delta = float(
            self.declare_parameter("z3_refine_max_yaw_delta", float(Z3_REFINE_MAX_YAW_DELTA)).value)
        self._z3_refine_alpha = float(
            self.declare_parameter("z3_refine_alpha", Z3_REFINE_ALPHA).value)
        self._z3_refine_stable_count = int(
            self.declare_parameter("z3_refine_stable_count", Z3_REFINE_STABLE_COUNT).value)
        self._z2_frame = ("blue_" if self._is_blue_team else "red_") + "zone2_root"
        self._refined = False
        self._pending_refines = []
        self._robot_tf_frame = str(
            self.declare_parameter("robot_tf_frame", ROBOT_TF_FRAME).value)
        self._min_lock_confidence = float(
            self.declare_parameter("min_lock_confidence", MIN_LOCK_CONFIDENCE).value)
        self._stable_lock_count = int(
            self.declare_parameter("stable_lock_count", STABLE_LOCK_COUNT).value)
        self._stable_center_tol = float(
            self.declare_parameter("stable_center_tol", STABLE_CENTER_TOL).value)
        self._stable_yaw_tol = float(
            self.declare_parameter("stable_yaw_tol", STABLE_YAW_TOL).value)
        self._accumulate_frames = int(
            self.declare_parameter("accumulate_frames", ACCUMULATE_FRAMES).value)
        self._zone3_root_frame = ("blue_" if self._is_blue_team else "red_") + "zone3_root"
        self._grid_center_rel_x = GRID_FIELD_X - (
            BLUE_ZONE3_ROOT_X if self._is_blue_team else RED_ZONE3_ROOT_X)
        self._grid_center_rel_y = GRID_FIELD_Y - ZONE3_ROOT_FIELD_Y
        self._tf_x = 0.0
        self._tf_y = 0.0
        self._tf_yaw = 0.0
        self._last_detection = None
        self._last_header = None
        self._pending_locks = []
        self._gate_logged = False  # [重试区到达] 仅输出一次

        self._tf_timer = self.create_timer(1.0 / DYNAMIC_TF_RATE, self._publish_tf_timer)
        self._init_debug_csv()
        self._init_height_diag_log()
        self._write_detail_event(
            "start",
            child_frame=self._zone3_root_frame,
            grid_rel_x=self._grid_center_rel_x,
            grid_rel_y=self._grid_center_rel_y,
            odom_gate=int(self._require_odom_gate),
        )

    def cloud_cb(self, msg):
        if msg.width == 0 or msg.height == 0:
            return

        x, y, z = self._parse_xyz(msg)
        if len(x) == 0:
            return
        x = x[::DOWNSAMPLE_STEP]
        y = y[::DOWNSAMPLE_STEP]
        z = z[::DOWNSAMPLE_STEP]

        # 根据 source frame 选择 ROI
        src = msg.header.frame_id
        if src == "odom":
            rx_min, rx_max = ODOM_X_MIN, ODOM_X_MAX
            ry_min, ry_max = ODOM_Y_MIN, ODOM_Y_MAX
        else:
            rx_min, rx_max = X_MIN, X_MAX
            ry_min, ry_max = Y_MIN, Y_MAX
        roi = ((x >= rx_min) & (x <= rx_max) &
               (y >= ry_min) & (y <= ry_max) &
               (z >= Z_MIN) & (z <= Z_MAX))
        if not roi.any():
            if self._pub_candidates:
                self._pub_candidates.publish(self._empty_cloud(msg.header))
            return
        x, y, z = x[roi], y[roi], z[roi]

        z_h = self._to_height_frame(msg.header, x, y, z)
        if z_h is None or len(z_h) < 40:
            if self._pub_candidates:
                self._pub_candidates.publish(self._empty_cloud(msg.header))
            return

        if self._ground_z is None or not GROUND_Z_KNOWN:
            mg, _mp = self._detect_ground(z_h)
            if self._ground_z is None:
                self._ground_z = mg
            elif abs(mg - self._ground_z) <= GROUND_MAX_UPDATE_STEP:
                self._ground_z += GROUND_UPDATE_ALPHA * (mg - self._ground_z)
        if self._ground_z is None:
            return

        h = z_h - self._ground_z
        if self._height_diag_log:
            stamp_f = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            self._write_height_diag(stamp_f, msg.header.frame_id, x, y, z_h, h)
        if self._pub_height_bands:
            self._publish_height_bands(msg.header, x, y, z, h)
        high = (h >= GRID_MIN_H) & (h <= GRID_MAX_H)
        if self._pub_candidates:
            self._publish_candidates(msg.header, x, y, z, h, high)
        if int(high.sum()) < 40:
            if self._pub_model:
                self._pub_model.publish(self._empty_cloud(msg.header))
            return
        # === 重试区门控 (分位置 / 朝向 两阶段) ===
        if self._require_odom_gate:
            pose = self._get_robot_pose_from_tf()
            if pose is None:
                self._log_gate_wait(f"no tf {self._robot_tf_frame}")
                self._frames.clear()
                self._pending_locks.clear()
                if self._pub_model:
                    self._pub_model.publish(self._empty_cloud(msg.header))
                return
            rx, ry, ryaw = pose

            # 阶段1: 位置门控 (在重试区内 → 立即 Z2 初锁, 仅第一次)
            in_pos = (abs(rx - self._retry_center_x) <= self._retry_half_size and
                      abs(ry - self._retry_center_y) <= self._retry_half_size)
            if not in_pos:
                if self._gate_logged:
                    self._gate_logged = False
                    self.get_logger().info(f"[离开重试区]")
                self._log_gate_wait(
                    f"retry_pos robot=({rx:.2f},{ry:.2f}) "
                    f"center=({self._retry_center_x:.2f},{self._retry_center_y:.2f})")
                self._frames.clear()
                self._pending_locks.clear()
                if self._pub_model:
                    self._pub_model.publish(self._empty_cloud(msg.header))
                return

            # 在重试区内 → Z2 初锁 (仅第一次进入时)
            if not self._locked and self._enable_z2_prior:
                self._try_z2_prior_lock()

            # 阶段2: 朝向门控 (对准了才检测精修)
            yaw_err = abs(self._norm_angle(ryaw - self._retry_face_yaw))
            if yaw_err > self._retry_face_yaw_tol:
                self._log_gate_wait(
                    f"retry_yaw robot_yaw={math.degrees(ryaw):.1f} "
                    f"face_yaw={math.degrees(self._retry_face_yaw):.1f} "
                    f"err={math.degrees(yaw_err):.1f} > tol={math.degrees(self._retry_face_yaw_tol):.1f}")
                self._frames.clear()
                self._pending_locks.clear()
                if self._pub_model:
                    self._pub_model.publish(self._empty_cloud(msg.header))
                return

            # 门控全通: 位置+朝向都对 (仅首次输出)
            if not self._gate_logged:
                self._gate_logged = True
                self.get_logger().info(
                    f"[重试区到达] robot=({rx:.2f},{ry:.2f}) "
                    f"yaw={math.degrees(ryaw):.1f}")

        # 门控全部通过 (或关闭) → 继续检测
        self._frames.append((x[high].astype(np.float64), y[high].astype(np.float64), h[high].astype(np.float64)))
        if len(self._frames) > self._accumulate_frames:
            self._frames.pop(0)
        ax = np.concatenate([f[0] for f in self._frames])
        ay = np.concatenate([f[1] for f in self._frames])
        ah = np.concatenate([f[2] for f in self._frames])

        det = detect_grid_pose(ax, ay, ah)
        if det is None:
            if self._pub_model:
                self._pub_model.publish(self._empty_cloud(msg.header))
            self._log_detection(msg.header, len(x), int(high.sum()), None)
            return

        odom_pose = self._grid_pose_to_odom(msg.header.frame_id, det.center_x, det.center_y, det.yaw)
        if odom_pose is None:
            return
        grid_odom_x, grid_odom_y, grid_odom_yaw = odom_pose
        root_x, root_y, root_yaw = self._root_from_grid_pose(grid_odom_x, grid_odom_y, grid_odom_yaw)

        self._last_detection = det
        self._last_header = msg.header
        if not self._locked and self._lock_candidate_is_stable(det, root_x, root_y, root_yaw):
            self._locked = True
            stable = self._pending_locks[-self._stable_lock_count:]
            self._tf_x = float(np.mean([p[0] for p in stable]))
            self._tf_y = float(np.mean([p[1] for p in stable]))
            self._tf_yaw = self._mean_yaw([p[2] for p in stable])
            self.get_logger().info(
                f"[zone3] LOCK {self._zone3_root_frame}: "
                f"x={self._tf_x:.3f} y={self._tf_y:.3f} yaw={math.degrees(self._tf_yaw):.1f}deg")
        elif self._locked and not self._refined and self._zone3_odom_gate_open():
            # 已由 Z2 先验锁定 → 到重试区后小范围精修
            self._try_refine(root_x, root_y, root_yaw)
        if self._locked:
            self._publish_tf(msg.header.stamp)

        if self._pub_model:
            self._publish_model(msg.header, det)
        self._log_detection(msg.header, len(x), int(high.sum()), det, root_x, root_y, root_yaw)

    def _grid_pose_to_odom(self, source_frame, gx, gy, gyaw):
        if not source_frame or source_frame == SOURCE_FIXED_FRAME:
            return gx, gy, gyaw
        try:
            t = self._tf_buffer.lookup_transform(SOURCE_FIXED_FRAME, source_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            if time.monotonic() - self._last_tf_warn >= 1.0:
                self._last_tf_warn = time.monotonic()
                self.get_logger().warn(f"TF {source_frame}->{SOURCE_FIXED_FRAME} unavailable: {e}")
            return None

        syaw = self._quat_yaw(t.transform.rotation)
        c = math.cos(syaw)
        s = math.sin(syaw)
        ox = c * gx - s * gy + t.transform.translation.x
        oy = s * gx + c * gy + t.transform.translation.y
        return ox, oy, self._norm_angle(gyaw + syaw)

    def _root_from_grid_pose(self, gx, gy, yaw):
        return choose_team_root_pose(
            gx, gy, yaw,
            self._grid_center_rel_x,
            self._grid_center_rel_y,
            self._is_blue_team,
        )

    def _get_robot_pose_from_tf(self):
        try:
            t = self._tf_buffer.lookup_transform("odom", self._robot_tf_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            if time.monotonic() - self._last_tf_warn >= 1.0:
                self._last_tf_warn = time.monotonic()
                self.get_logger().warn(f"TF odom->{self._robot_tf_frame} 不可用: {e}")
            return None
        return (t.transform.translation.x,
                t.transform.translation.y,
                self._quat_yaw(t.transform.rotation))

    def _try_z2_prior_lock(self):
        """只锁一次: 第一次进入重试区时从 Z2 TF 锁定 Z3 (后续不再跟随)."""
        if self._locked or self._refined:
            return True  # 已锁过, 不再重新读取 Z2
        if not self._enable_z2_prior:
            return False
        try:
            t = self._tf_buffer.lookup_transform("odom", self._z2_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return False
        syaw = self._quat_yaw(t.transform.rotation)
        c = math.cos(syaw)
        s = math.sin(syaw)
        ox = c * Z2_TO_Z3_OFFSET_X - s * Z2_TO_Z3_OFFSET_Y
        oy = s * Z2_TO_Z3_OFFSET_X + c * Z2_TO_Z3_OFFSET_Y
        self._tf_x = t.transform.translation.x + ox
        self._tf_y = t.transform.translation.y + oy
        self._tf_yaw = self._norm_angle(syaw)
        self._locked = True
        self.get_logger().info(
            f"[zone3] Z2 prior LOCK {self._zone3_root_frame}: "
            f"x={self._tf_x:.3f} y={self._tf_y:.3f} yaw={math.degrees(self._tf_yaw):.1f}")
        return True

    def _zone3_odom_gate_open(self):
        pose = self._get_robot_pose_from_tf()
        if pose is None:
            self._log_gate_wait(f"no tf {self._robot_tf_frame}")
            return False
        rx, ry, ryaw = pose

        # 重试区位置门控 (中心 ±HALF_SIZE)
        if abs(rx - self._retry_center_x) > self._retry_half_size:
            self._log_gate_wait(
                f"retry_pos robot=({rx:.2f},{ry:.2f}) "
                f"center=({self._retry_center_x:.2f},{self._retry_center_y:.2f})")
            return False
        if abs(ry - self._retry_center_y) > self._retry_half_size:
            self._log_gate_wait(
                f"retry_pos robot=({rx:.2f},{ry:.2f}) "
                f"center=({self._retry_center_x:.2f},{self._retry_center_y:.2f})")
            return False

        # 朝向门控: 机器人 yaw 必须在朝向九宫格的容差范围内
        yaw_err = abs(self._norm_angle(ryaw - self._retry_face_yaw))
        if yaw_err > self._retry_face_yaw_tol:
            self._log_gate_wait(
                f"retry_yaw robot_yaw={math.degrees(ryaw):.1f} "
                f"face_yaw={math.degrees(self._retry_face_yaw):.1f} "
                f"err={math.degrees(yaw_err):.1f} > tol={math.degrees(self._retry_face_yaw_tol):.1f}")
            return False

        return True

    def _log_gate_wait(self, reason):
        now = time.monotonic()
        if now - self._last_gate_log >= 1.0:
            self._last_gate_log = now
            self._write_detail_event("odom_gate_wait", reason=reason)

    def _lock_candidate_is_stable(self, det, root_x, root_y, root_yaw):
        if not self._is_good_grid_candidate(det):
            self._pending_locks.clear()
            return False
        self._pending_locks.append((float(root_x), float(root_y), float(root_yaw), det))
        if len(self._pending_locks) > self._stable_lock_count:
            self._pending_locks = self._pending_locks[-self._stable_lock_count:]
        if len(self._pending_locks) < self._stable_lock_count:
            return False
        xs = np.asarray([p[0] for p in self._pending_locks], dtype=np.float64)
        ys = np.asarray([p[1] for p in self._pending_locks], dtype=np.float64)
        yaws = [p[2] for p in self._pending_locks]
        center_spread = float(np.max(np.hypot(xs - xs.mean(), ys - ys.mean())))
        yaw0 = yaws[0]
        yaw_spread = max(abs(self._norm_angle(y - yaw0)) for y in yaws)
        return center_spread <= self._stable_center_tol and yaw_spread <= self._stable_yaw_tol

    def _is_good_grid_candidate(self, det):
        return (
            det.confidence >= self._min_lock_confidence and
            det.layer_count >= 3 and
            det.point_count >= 700 and
            1.30 <= det.width <= 1.95 and
            0.15 <= det.depth <= 0.60
        )

    def _try_refine(self, root_x, root_y, root_yaw):
        """在 Z2 先验锁定的基础上做小范围精修."""
        dx = root_x - self._tf_x
        dy = root_y - self._tf_y
        dyaw = abs(self._norm_angle(root_yaw - self._tf_yaw))
        if math.hypot(dx, dy) > self._z3_refine_max_translation or dyaw > self._z3_refine_max_yaw_delta:
            self._pending_refines.clear()
            return
        self._pending_refines.append((root_x, root_y, root_yaw))
        if len(self._pending_refines) > self._z3_refine_stable_count:
            self._pending_refines = self._pending_refines[-self._z3_refine_stable_count:]
        if len(self._pending_refines) < self._z3_refine_stable_count:
            return
        xs = [p[0] for p in self._pending_refines]
        ys = [p[1] for p in self._pending_refines]
        spread = float(np.max(np.hypot(np.asarray(xs) - np.mean(xs),
                                       np.asarray(ys) - np.mean(ys))))
        if spread > self._stable_center_tol:
            self._pending_refines.clear()
            return
        mean_x = float(np.mean(xs))
        mean_y = float(np.mean(ys))
        mean_yaw = self._mean_yaw([p[2] for p in self._pending_refines])
        self._tf_x += self._z3_refine_alpha * (mean_x - self._tf_x)
        self._tf_y += self._z3_refine_alpha * (mean_y - self._tf_y)
        self._tf_yaw = self._blend_yaw(self._tf_yaw, mean_yaw, self._z3_refine_alpha)
        self._refined = True
        self._pending_refines.clear()
        self.get_logger().info(
            f"[zone3] REFINE {self._zone3_root_frame}: "
            f"x={self._tf_x:.3f} y={self._tf_y:.3f} yaw={math.degrees(self._tf_yaw):.1f}")

    def _publish_retry_test_tf_msg(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "zone3_retry_test"
        t.transform.translation.x = self._retry_center_x
        t.transform.translation.y = self._retry_center_y
        t.transform.translation.z = self._retry_z
        qx, qy, qz, qw = self._quat_from_yaw(self._retry_face_yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)

    def _publish_retry_area_marker(self):
        if not self._pub_retry_marker:
            return
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "odom"
        marker.ns = "zone3_retry_area"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self._retry_center_x
        marker.pose.position.y = self._retry_center_y
        marker.pose.position.z = self._retry_z
        qx, qy, qz, qw = self._quat_from_yaw(self._retry_face_yaw)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        half = self._retry_half_size
        marker.scale.x = half * 2.0
        marker.scale.y = half * 2.0
        marker.scale.z = 0.002
        marker.color.a = 0.4
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime.sec = 0
        self._pub_retry_marker.publish(marker)

    def _publish_tf_timer(self):
        if self._locked:
            self._publish_tf(self.get_clock().now().to_msg())
        if self._enable_debug_vis:
            self._publish_retry_test_tf_msg()
            self._publish_retry_area_marker()

    def _publish_tf(self, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = SOURCE_FIXED_FRAME
        tf_msg.child_frame_id = self._zone3_root_frame
        tf_msg.transform.translation.x = float(self._tf_x)
        tf_msg.transform.translation.y = float(self._tf_y)
        tf_msg.transform.translation.z = float(GROUND_Z)
        qx, qy, qz, qw = self._quat_from_yaw(self._tf_yaw)
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf_msg)

    def _publish_height_bands(self, header, x, y, z, h):
        """按 HEIGHT_BANDS 区间染色发布调试点云."""
        n = len(x)
        if n == 0 or not HEIGHT_BANDS:
            self._pub_height_bands.publish(self._empty_cloud(header))
            return
        r = np.full(n, 0, dtype=np.uint8)
        g = np.full(n, 0, dtype=np.uint8)
        b = np.full(n, 0, dtype=np.uint8)
        mask = np.zeros(n, dtype=bool)
        for lo, hi, cr, cg, cb in HEIGHT_BANDS:
            in_band = (h >= lo) & (h < hi)
            r[in_band] = cr
            g[in_band] = cg
            b[in_band] = cb
            mask |= in_band
        if not mask.any():
            self._pub_height_bands.publish(self._empty_cloud(header))
            return
        self._pub_height_bands.publish(
            self._make_cloud(header, x[mask], y[mask], z[mask], r[mask], g[mask], b[mask]))

    def _publish_candidates(self, header, x, y, z, h, high):
        publish = high | (np.abs(h) <= GROUND_TOLERANCE)
        if not publish.any():
            self._pub_candidates.publish(self._empty_cloud(header))
            return
        n = int(publish.sum())
        r = np.full(n, 80, dtype=np.uint8)
        g = np.full(n, 80, dtype=np.uint8)
        b = np.full(n, 80, dtype=np.uint8)
        hp = high[publish]
        r[hp], g[hp], b[hp] = 255, 80, 20
        gp = np.abs(h[publish]) <= GROUND_TOLERANCE
        r[gp], g[gp], b[gp] = 255, 255, 255
        self._pub_candidates.publish(self._make_cloud(header, x[publish], y[publish], z[publish], r, g, b))

    def _publish_model(self, header, det):
        mx, my, mz = self._model_points(det)
        r = np.full(len(mx), 255, dtype=np.uint8)
        g = np.full(len(mx), 255, dtype=np.uint8)
        b = np.zeros(len(mx), dtype=np.uint8)
        self._pub_model.publish(self._make_cloud(header, mx, my, mz, r, g, b))

    def _model_points(self, det):
        c = math.cos(det.yaw)
        s = math.sin(det.yaw)
        local = []
        ys = np.array([-GRID_WIDTH_Y / 2.0, 0.0, GRID_WIDTH_Y / 2.0])
        zs = np.array([1.07, 1.61, 2.15])
        for zc in zs:
            for yc in ys:
                for xx in np.linspace(-GRID_DEPTH_X / 2.0, GRID_DEPTH_X / 2.0, 8):
                    local.append((xx, yc - 0.25, zc))
                    local.append((xx, yc + 0.25, zc))
                for yy in np.linspace(yc - 0.25, yc + 0.25, 12):
                    local.append((-GRID_DEPTH_X / 2.0, yy, zc))
                    local.append((GRID_DEPTH_X / 2.0, yy, zc))
        arr = np.asarray(local, dtype=np.float64)
        x = det.center_x + c * arr[:, 0] - s * arr[:, 1]
        y = det.center_y + s * arr[:, 0] + c * arr[:, 1]
        z = arr[:, 2] + (self._ground_z if self._ground_z is not None else GROUND_Z)
        return x.astype(np.float32), y.astype(np.float32), z.astype(np.float32)

    def _to_height_frame(self, header, x, y, z):
        src = header.frame_id
        if not src or src == HEIGHT_FRAME:
            return z
        try:
            t = self._tf_buffer.lookup_transform(HEIGHT_FRAME, src, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            if time.monotonic() - self._last_tf_warn >= 1.0:
                self._last_tf_warn = time.monotonic()
                self.get_logger().warn(
                    f"TF {src}->{HEIGHT_FRAME} 不可用, 跳过此帧: {e}")
            return None

        rot = t.transform.rotation
        xx, yy = rot.x**2, rot.y**2
        xz, yz = rot.x * rot.z, rot.y * rot.z
        wx, wy = rot.w * rot.x, rot.w * rot.y
        r20, r21, r22 = (2 * (xz - wy),
                         2 * (yz + wx),
                         1 - 2 * (xx + yy))
        return (r20 * x.astype(np.float32, copy=False)
                + r21 * y.astype(np.float32, copy=False)
                + r22 * z.astype(np.float32, copy=False)
                + np.float32(t.transform.translation.z))

    def _detect_ground(self, z):
        zf = z[np.isfinite(z)]
        if len(zf) == 0:
            return 0.0, 0.0
        lo = np.floor(zf.min() / HISTOGRAM_BIN_WIDTH) * HISTOGRAM_BIN_WIDTH
        hi = np.ceil(zf.max() / HISTOGRAM_BIN_WIDTH) * HISTOGRAM_BIN_WIDTH
        if hi <= lo:
            hi = lo + HISTOGRAM_BIN_WIDTH
        bins = np.arange(lo, hi + HISTOGRAM_BIN_WIDTH, HISTOGRAM_BIN_WIDTH)
        hist, edges = np.histogram(zf, bins=bins)
        peak = hist.max()
        idx = next((i for i, v in enumerate(hist) if v >= peak * GROUND_PEAK_RATIO), int(np.argmax(hist)))
        return float((edges[idx] + edges[idx + 1]) / 2.0), float(100.0 * hist[idx] / len(zf))

    def _parse_xyz(self, cloud):
        n = cloud.width * cloud.height if cloud.height > 1 else cloud.width
        dt = np.dtype({"names": ["x", "y", "z"],
                       "formats": [np.float32] * 3,
                       "offsets": [0, 4, 8],
                       "itemsize": cloud.point_step})
        pts = np.frombuffer(cloud.data, dtype=dt, count=n)
        return pts["x"], pts["y"], pts["z"]

    def _empty_cloud(self, header):
        return self._make_cloud(
            header,
            np.empty(0, dtype=np.float32),
            np.empty(0, dtype=np.float32),
            np.empty(0, dtype=np.float32),
            np.empty(0, dtype=np.uint8),
            np.empty(0, dtype=np.uint8),
            np.empty(0, dtype=np.uint8),
        )

    def _make_cloud(self, header, x, y, z, r, g, b):
        n = len(x)
        a = np.full(n, 255, dtype=np.uint8)
        pts = np.zeros(n, dtype=[("x", np.float32), ("y", np.float32),
                                 ("z", np.float32), ("rgb", np.uint32)])
        pts["x"], pts["y"], pts["z"] = x, y, z
        pts["rgb"] = (a.astype(np.uint32) << 24) \
                     | (r.astype(np.uint32) << 16) \
                     | (g.astype(np.uint32) << 8) \
                     | b.astype(np.uint32)

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
            for name, offset in [("x", 0), ("y", 4), ("z", 8), ("rgb", 12)]]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.data = pts.tobytes()
        msg.is_dense = True
        return msg

    def _init_debug_csv(self):
        if not self._detailed_file_log:
            return
        os.makedirs(DEBUG_DIR, exist_ok=True)
        self._debug_csv = os.path.join(DEBUG_DIR, f"{DEBUG_CSV_PREFIX}_{_make_detail_log_stamp()}.csv")
        with open(self._debug_csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "stamp_sec", "event", "source_points", "high_points", "confidence",
                "grid_x", "grid_y", "grid_yaw", "root_x", "root_y", "root_yaw",
                "component_points", "width", "depth", "layers", "extra"])

    def _log_detection(self, header, source_points, high_points, det, root_x=None, root_y=None, root_yaw=None):
        now = time.monotonic()
        if now - self._last_log >= LOG_INTERVAL:
            self._last_log = now
            if det is None:
                self._write_detail_event(
                    "detection_none",
                    source_points=source_points,
                    high_points=high_points,
                )
            else:
                self._write_detail_event(
                    "detection",
                    source_points=source_points,
                    high_points=high_points,
                    confidence=det.confidence,
                    grid_x=det.center_x,
                    grid_y=det.center_y,
                    root_x=root_x,
                    root_y=root_y,
                    yaw_deg=math.degrees(det.yaw),
                    root_yaw_deg=math.degrees(root_yaw),
                    points=det.point_count,
                    width=det.width,
                    depth=det.depth,
                    layers=det.layer_count,
                )
        if not self._detailed_file_log:
            return
        stamp_sec = float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9
        with open(self._debug_csv, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            if det is None:
                writer.writerow([stamp_sec, "detection_none", source_points, high_points, 0, "", "", "", "", "", "", 0, "", "", 0, ""])
            else:
                writer.writerow([
                    stamp_sec, "detection", source_points, high_points, det.confidence,
                    det.center_x, det.center_y, det.yaw,
                    root_x, root_y, root_yaw,
                    det.point_count, det.width, det.depth, det.layer_count, ""])

    def _write_detail_event(self, event, **data):
        if not self._detailed_file_log or not self._debug_csv:
            return
        payload = " ".join(f"{k}={v}" for k, v in data.items())
        with open(self._debug_csv, "a", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow([f"{time.time():.3f}", event, "", "", "", "", "", "", "", "", "", "", "", "", "", payload])

    def _init_height_diag_log(self):
        self._height_diag_log = bool(
            self.declare_parameter("enable_height_diag_log", bool(ENABLE_HEIGHT_DIAG_LOG)).value)
        self._height_diag_path = ""
        if not self._height_diag_log:
            return
        os.makedirs(HEIGHT_DIAG_LOG_DIR, exist_ok=True)
        self._height_diag_path = os.path.join(
            HEIGHT_DIAG_LOG_DIR, f"zone3_height_diag_{_make_detail_log_stamp()}.csv")
        with open(self._height_diag_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow([
                "stamp_sec", "frame_id", "n_total",
                "n_gray", "n_yellow", "n_cyan", "n_orange", "n_green", "n_purple", "n_unbanded",
                "orange_green_purple_medianZ_and_count",
                "sample_points_xyz_h",
            ])

    def _write_height_diag(self, stamp_sec, src, x, y, z_h, h):
        if not self._height_diag_log or not self._height_diag_path:
            return
        n_total = len(h)
        # band index: 0=gray 1=yellow 2=cyan 3=orange 4=green 5=purple 6=unbanded
        cnt = [0]*7
        z_all = {i: [] for i in (3, 4, 5)}  # orange/green/purple Z values
        samp = {i: [] for i in (3, 4, 5)}   # sample points
        for pi in range(n_total):
            hi, zi = h[pi], z_h[pi]
            for bi, (lo, hi_r, _, _, _) in enumerate(HEIGHT_BANDS):
                if lo <= hi < hi_r:
                    cnt[bi] += 1
                    if bi in (3, 4, 5):
                        z_all[bi].append(float(zi))
                        if len(samp[bi]) < 5:
                            samp[bi].append(
                                f"({float(x[pi]):.2f},{float(y[pi]):.2f},{float(zi):.3f},{float(hi):.3f})")
                    break
            else:
                cnt[6] += 1
        # 各 band 的 odom_z 中位数 → 看高度是否稳定
        def med(arr):
            return f"{float(np.median(arr)):.3f}" if arr else "none"
        band_info = ";".join(
            f"{i}:med_z={med(z_all[i])} n={cnt[i]}"
            for i in (3, 4, 5))
        # 采样点详情
        samp_info = ";".join(
            f"b{i}:{','.join(samp[i])}" if samp[i] else f"b{i}:none"
            for i in (3, 4, 5))
        with open(self._height_diag_path, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow([
                f"{stamp_sec:.3f}", src, n_total,
                cnt[0], cnt[1], cnt[2], cnt[3], cnt[4], cnt[5], cnt[6],
                band_info, samp_info,
            ])

    @staticmethod
    def _quat_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def _quat_from_yaw(yaw):
        return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)

    @staticmethod
    def _norm_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _blend_yaw(self, old, new, alpha):
        delta = self._norm_angle(new - old)
        return self._norm_angle(old + alpha * delta)

    def _mean_yaw(self, yaws):
        s = float(np.mean([math.sin(y) for y in yaws]))
        c = float(np.mean([math.cos(y) for y in yaws]))
        return self._norm_angle(math.atan2(s, c))


def main():
    rclpy.init()
    node = Zone3GridLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
