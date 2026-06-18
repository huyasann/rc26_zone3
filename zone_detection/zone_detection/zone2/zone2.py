#!/usr/bin/env python3
"""Zone2 点云检测与动态 TF 发布节点.

从 nav2step.py 中提取:
  - /odin1/cloud_slam 点云订阅
  - /rc26/zone2/cloud_height_bands 颜色带点云发布
  - /rc26/zone2/cloud_facade_recon 墙皮/重建点云发布
  - 前立面拟合、白/黄柱判决
  - odom -> zone2_root 动态 TF 发布

不包含:
  - Nav2 NavigateToPose
  - 导航目标快照 / action 发送
  - Trigger 导航服务
"""

import math
import csv
import os
import select
import sys
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from scipy.ndimage import binary_dilation
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


# ============================== 可配置参数 ==============================

# -------------------------------------------------------------------------
# 1) 输入与裁剪
# -------------------------------------------------------------------------
# 原始点云输入来自 Odin，先做空间门控，避免退缩区和武馆区干扰。
X_MIN, X_MAX = 0.0, 3.5
Y_MIN, Y_MAX = -1.7, 1.7
Z_MIN, Z_MAX = -0.50, 0.50

# 高度计算参考系。当前直接使用 odom 做 Z 轴物理高度归一。
HEIGHT_FRAME = 'odom'

# -------------------------------------------------------------------------
# 2) 地面估计
# -------------------------------------------------------------------------
# 1: 使用手动地面高度；0: 首帧自动估计。
GROUND_Z_KNOWN = 1
# 手动地面高度，单位 m，坐标系为 HEIGHT_FRAME。
GROUND_Z = -0.270
# 认为点属于地面的高度容差，单位 m。
GROUND_TOLERANCE = 0.035
# 地面直方图中，从低到高扫描时的峰值比例阈值。
GROUND_PEAK_RATIO = 0.15
# 地面直方图 bin 宽度，单位 m。
HISTOGRAM_BIN_WIDTH = 0.02
# 自动地面更新的 EMA 系数。
GROUND_UPDATE_ALPHA = 0.18
# 自动地面更新时，允许和当前地面高度偏差的最大步长。
GROUND_MAX_UPDATE_STEP = 0.06

# -------------------------------------------------------------------------
# 3) 高度颜色带输出
# -------------------------------------------------------------------------
# 颜色带输出仅用于调试或上游可视化，不参与后续拟合决策。
HEIGHT_BAND_1_MAX = 0.20
HEIGHT_BAND_2_MAX = 0.40

# -------------------------------------------------------------------------
# 4) 第二区域先验与动态 TF
# -------------------------------------------------------------------------
# 队伍开关:
#   True  -> 蓝方
#   False -> 红方
IS_BLUE_TEAM = True

# 本节点只维护 odom -> {team}_zone2_root，zone2_root 下的具体场地模型由 rc26_field.py 发布。
ZONE2_ROOT_FRAME = ("blue_" if IS_BLUE_TEAM else "red_") + "zone2_root"

# 检测红点对应的 zone2_root 局部先验:
# 第一排中间 200mm 梅林块前表面中心。
# rc26_field.py 中第一排中心为 y=1.65，方块边长 1.2，前表面 y=1.65+0.6=2.25。
ZONE2_TARGET_X = 0.0
ZONE2_TARGET_Y = 2.25

# 黄线拟合得到的是前表面直线 yaw；该偏置把检测线 yaw 转换成 zone2_root yaw。
# 保持 nav2step.py 旧逻辑的 pi/2，使本脚本和 rc26_field.py 的 Z2 坐标轴一致。
ZONE2_ROOT_YAW_OFFSET = math.pi / 2

# zone2_root 的 Z 偏移手动配置；检测只约束 x/y/yaw。
ZONE2_ROOT_Z = -0.270

# 锁定后持续重发 TF 的频率。
DYNAMIC_TF_RATE = 10.0

# 自动锁定策略。检测到的 zone2_root 连续多帧稳定后自动锁，避免依赖键盘 Enter。
AUTO_LOCK_ZONE2_ROOT = 1
STABLE_LOCK_COUNT = 4
STABLE_CENTER_TOL = 0.22
STABLE_YAW_TOL = math.radians(10.0)

# 二阶段小范围修正:
# 先用远距离观测粗锁；车走到 Z2 R2 入口区、正对梅林时，点云质量最好，
# 此时只允许在粗锁附近做小幅修正，避免后续误检把 TF 拉飞。
ODOM_TOPIC = "/odin1/odometry_highfreq"
ENABLE_ENTRY_REFINEMENT = 1
REFINE_STABLE_COUNT = 3
REFINE_CENTER_TOL = 0.16
REFINE_YAW_TOL = math.radians(8.0)
REFINE_MAX_TRANSLATION = 0.65
REFINE_MAX_YAW_DELTA = math.radians(20.0)
REFINE_ENTRY_X_ABS_MAX = 2.70
REFINE_ENTRY_Y_MIN = 1.45
REFINE_ENTRY_Y_MAX = 3.70
REFINE_FACING_YAW = -math.pi / 2.0
REFINE_FACING_YAW_TOL = math.radians(75.0)
REFINE_ALPHA = 0.65
REFINE_ONCE = 1

# -------------------------------------------------------------------------
# 9) 观测位置门控 (odom 系)
# -------------------------------------------------------------------------
# 仅在机器人位于 Z2 最佳观测区域内时才开始点云检测,
# 避免在 Z1 武馆阶段误检或空转 CPU。
#
# 参考场地模型 (rc26_field.py):
#   Z2_BEST_OBS_POINT_REL  = (0.0, 3.70)     ← zone2_root 系下的观测中心
#   Z2_BEST_OBS_POINT_SIZE = [1.5, 1.5, ...]  ← 观测区域大小
#
# GATE_CENTER_X/Y 默认值按场地模型计算（见下方文档说明），
# 在 init 中按红蓝队修正，不做硬编码常量。
ENABLE_ODOM_GATE = 1               # 0=关闭 (和现在一样不限制), 1=启用 (默认开)
GATE_CENTER_X_DEFAULT = 1.600      # BLUE 队默认观测中心 X (RED 队取反为 -1.600)
GATE_CENTER_Y_DEFAULT = 1.350     # BLUE 队默认观测中心 Y (RED 队取反为 +1.350)
GATE_HALF_SIZE_X = 0.75            # 观测区半宽 (= Z2_BEST_OBS_POINT_SIZE[0]/2)
GATE_HALF_SIZE_Y = 0.75            # 观测区半高 (= Z2_BEST_OBS_POINT_SIZE[1]/2)
GATE_STRENGTH_ATTENUATION = 0.5    # 边缘检测强度衰减系数: 1.0=完全衰减, 0.0=不衰减
GATE_YAW_TOLERANCE_DEG = 30        # 朝向容差 (度), 相对 odom 正前方 ±60°

# -------------------------------------------------------------------------
# 5) 前立面分层与拟合
# -------------------------------------------------------------------------
# 前立面有效高度窗口，覆盖 200mm 白线和 400mm 黄线。
FACADE_SLICE_Z_MIN = 0.005
FACADE_SLICE_Z_MAX = 0.400
# 200mm 白线的高度带范围。
BAND_200_Z_MIN = 0.050
BAND_200_Z_MAX = 0.200
# 400mm 黄线的高度带范围。
BAND_400_Z_MIN = 0.250
BAND_400_Z_MAX = 0.400

# 2D 直方图的分辨率。X/Y 越小越敏感，但噪声也更高。
Y_BIN_SIZE = 0.05
X_BIN_WIDTH = 0.05
# 单个 (Y, X) bin 至少需要的点数，低于该值视为稀疏噪点。
MIN_DENSITY_PEAK = 5
# RANSAC 拟合残差阈值，单位 m。
RANSAC_RESIDUAL_THRESHOLD = 0.05
# RANSAC 最少内点数门槛。
MIN_EDGE_POINTS = 5
# RANSAC 迭代次数。
RANSAC_N_ITER = 200
# 限制拟合斜率范围，避免把深度方向的线当成前立面。
MAX_ALLOWED_SLOPE = 0.6

# -------------------------------------------------------------------------
# 6) 墙面空间约束与体素判决
# -------------------------------------------------------------------------
# 只允许离拟合基准线左右一定宽度内的点参与后续判决。
WALL_HALF_WIDTH = 0.10
# 柱状评估的 y 方向分箱宽度。
COL_WIDTH = 0.05
# 柱状统计的 y 范围。
Y_COLUMN_MIN = -2.5
Y_COLUMN_MAX = 2.5
Y_COLUMN_NUM = int((Y_COLUMN_MAX - Y_COLUMN_MIN) / COL_WIDTH)
# 黄色柱的左右膨胀格数，用于屏蔽白色候选。
DILATE_BINS = 2

# -------------------------------------------------------------------------
# 7) 视觉几何重构
# -------------------------------------------------------------------------
# 合成白线采样点数。
SYNTHETIC_LINE_N = 500
# 合成红心簇点数。
SYNTHETIC_CLUSTER_N = 100
# 合成红心的高斯散布标准差，单位 m。
SYNTHETIC_CLUSTER_SPREAD = 0.015

# -------------------------------------------------------------------------
# 8) 降采样与日志
# -------------------------------------------------------------------------
# 每 5 个点取 1 个，减少计算量。
DOWNSAMPLE_STEP = 5
# 是否打印调试日志。
LOG_ENABLED = False
# 日志最小间隔，单位 s。
LOG_INTERVAL = 1.0
# 是否把详细检测过程写入文件。默认关闭，可通过
#   -p detailed_file_log:=true
# 打开。终端只保留锁定/修正等关键事件。
DETAILED_FILE_LOG = False
DETAIL_LOG_DIR = "/home/inkc/inkc/Rc2026/files/record/logs/zone_detection"


def _make_detail_log_stamp():
    return datetime.now().strftime("%m%d_%H%M")


class Zone2DetectorNode(Node):
    def __init__(self):
        super().__init__('zone2_detector_node')

        self.sub = self.create_subscription(
            PointCloud2, '/odin1/cloud_slam', self.cloud_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_cb, 20)
        self._pub_filtered = self.create_publisher(
            PointCloud2, '/rc26/zone2/cloud_height_bands', 10)
        self._pub_facade = self.create_publisher(
            PointCloud2, '/rc26/zone2/cloud_facade_recon', 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self._last_log = time.monotonic()
        self._last_tf_warn = 0.0
        self._last_refine_log = 0.0
        self._rng = np.random.default_rng()
        self._is_blue_team = bool(
            self.declare_parameter("is_blue_team", bool(IS_BLUE_TEAM)).value)
        self._detailed_file_log = bool(
            self.declare_parameter("detailed_file_log", bool(DETAILED_FILE_LOG)).value)
        self._detail_log_path = ""
        self._detail_log_ready = False
        self._zone2_root_frame = ("blue_" if self._is_blue_team else "red_") + "zone2_root"
        self._auto_lock = bool(
            self.declare_parameter("auto_lock_zone2_root", bool(AUTO_LOCK_ZONE2_ROOT)).value)
        self._stable_lock_count = int(
            self.declare_parameter("stable_lock_count", STABLE_LOCK_COUNT).value)
        self._stable_center_tol = float(
            self.declare_parameter("stable_center_tol", STABLE_CENTER_TOL).value)
        self._stable_yaw_tol = float(
            self.declare_parameter("stable_yaw_tol", STABLE_YAW_TOL).value)
        self._enable_refinement = bool(
            self.declare_parameter("enable_entry_refinement", bool(ENABLE_ENTRY_REFINEMENT)).value)
        self._refine_stable_count = int(
            self.declare_parameter("refine_stable_count", REFINE_STABLE_COUNT).value)
        self._refine_center_tol = float(
            self.declare_parameter("refine_center_tol", REFINE_CENTER_TOL).value)
        self._refine_yaw_tol = float(
            self.declare_parameter("refine_yaw_tol", REFINE_YAW_TOL).value)
        self._refine_max_translation = float(
            self.declare_parameter("refine_max_translation", REFINE_MAX_TRANSLATION).value)
        self._refine_max_yaw_delta = float(
            self.declare_parameter("refine_max_yaw_delta", REFINE_MAX_YAW_DELTA).value)
        self._refine_entry_x_abs_max = float(
            self.declare_parameter("refine_entry_x_abs_max", REFINE_ENTRY_X_ABS_MAX).value)
        self._refine_entry_y_min = float(
            self.declare_parameter("refine_entry_y_min", REFINE_ENTRY_Y_MIN).value)
        self._refine_entry_y_max = float(
            self.declare_parameter("refine_entry_y_max", REFINE_ENTRY_Y_MAX).value)
        self._refine_facing_yaw = float(
            self.declare_parameter("refine_facing_yaw", REFINE_FACING_YAW).value)
        self._refine_facing_yaw_tol = float(
            self.declare_parameter("refine_facing_yaw_tol", REFINE_FACING_YAW_TOL).value)
        self._refine_alpha = float(
            self.declare_parameter("refine_alpha", REFINE_ALPHA).value)
        self._refine_once = bool(
            self.declare_parameter("refine_once", bool(REFINE_ONCE)).value)
        self._pending_locks = []
        self._pending_refines = []
        self._last_candidate = None
        self._robot_odom = None
        self._refine_done = False
        self._init_detail_log()

        # --- 观测位置门控 (红蓝 X/Y 镜像) ---
        if self._is_blue_team:
            default_gate_x = GATE_CENTER_X_DEFAULT
            default_gate_y = GATE_CENTER_Y_DEFAULT
        else:
            default_gate_x = -GATE_CENTER_X_DEFAULT
            default_gate_y = -GATE_CENTER_Y_DEFAULT
        self._enable_odom_gate = bool(
            self.declare_parameter("enable_odom_gate", bool(ENABLE_ODOM_GATE)).value)
        self._gate_center_x = float(
            self.declare_parameter("gate_center_x", default_gate_x).value)
        self._gate_center_y = float(
            self.declare_parameter("gate_center_y", default_gate_y).value)
        self._gate_half_size_x = float(
            self.declare_parameter("gate_half_size_x", GATE_HALF_SIZE_X).value)
        self._gate_half_size_y = float(
            self.declare_parameter("gate_half_size_y", GATE_HALF_SIZE_Y).value)
        self._gate_strength_attenuation = float(
            self.declare_parameter("gate_strength_attenuation", GATE_STRENGTH_ATTENUATION).value)
        self._gate_yaw_tolerance_rad = math.radians(
            self.declare_parameter("gate_yaw_tolerance_deg", GATE_YAW_TOLERANCE_DEG).value)
        self._gate_strength = 1.0      # 当前帧的检测强度 (0~1), 未启用时为 1.0

        if self._enable_odom_gate:
            team_label = "BLUE" if self._is_blue_team else "RED"
            self.get_logger().info(
                f"[观测门控] {team_label} 中心"
                f"=({self._gate_center_x:.2f},{self._gate_center_y:.2f}) "
                f"范围={self._gate_half_size_x*2:.1f}m×{self._gate_half_size_y*2:.1f}m, "
                f"衰减={self._gate_strength_attenuation:.1f}, "
                f"航向=odom前方±{GATE_YAW_TOLERANCE_DEG}°")

        if GROUND_Z_KNOWN:
            self._ground_z = GROUND_Z
            self._ground_pct = 100.0
            if LOG_ENABLED:
                self.get_logger().info(f"手动 ground_z={self._ground_z:.3f}")
        else:
            self._ground_z = None
            self._ground_pct = 0.0
            if LOG_ENABLED:
                self.get_logger().info("自动检测模式，等待首帧...")

        self._zone2_root_locked = False
        self._locked_tf_x = 0.0
        self._locked_tf_y = 0.0
        self._locked_tf_yaw = 0.0

        self._detected_facade_x = 0.0
        self._detected_facade_y = 0.0
        self._detected_yaw = 0.0

        self._tf_timer = self.create_timer(1.0 / DYNAMIC_TF_RATE, self._tf_timer_cb)
        self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._keyboard_thread.start()
        self._write_detail_log(
            "start",
            child_frame=self._zone2_root_frame,
            auto_lock=int(self._auto_lock),
            entry_refine=int(self._enable_refinement),
        )

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        _, _, yaw = self._quat_to_rpy(msg.pose.pose.orientation)
        self._robot_odom = (float(p.x), float(p.y), float(yaw))

    def cloud_cb(self, msg):
        # ---- 观测位置门控 (未锁定时也生效) ----
        if not self._odom_gate_check():
            return

        if self._zone2_root_locked and (not self._enable_refinement or self._refine_done):
            return
        if self._zone2_root_locked and not self._refine_gate_open():
            return

        if msg.width == 0 or msg.height == 0:
            return

        x, y, z = self._parse_xyz(msg)
        if len(x) == 0:
            return

        x = x[::DOWNSAMPLE_STEP]
        y = y[::DOWNSAMPLE_STEP]
        z = z[::DOWNSAMPLE_STEP]

        roi = ((x >= X_MIN) & (x <= X_MAX) &
               (y >= Y_MIN) & (y <= Y_MAX) &
               (z >= Z_MIN) & (z <= Z_MAX))
        if not roi.any():
            self._pub_filtered.publish(self._empty_cloud(msg.header))
            self._pub_facade.publish(self._empty_cloud(msg.header))
            return

        x, y, z = x[roi], y[roi], z[roi]

        z_h = self._to_height_frame(msg.header, x, y, z)
        if len(z_h) < 10:
            self._pub_filtered.publish(self._empty_cloud(msg.header))
            return

        if self._ground_z is None or not GROUND_Z_KNOWN:
            mg, mp = self._detect_ground(z_h)
            if self._ground_z is None:
                self._ground_z, self._ground_pct = mg, mp
            elif abs(mg - self._ground_z) <= GROUND_MAX_UPDATE_STEP:
                self._ground_z += GROUND_UPDATE_ALPHA * (mg - self._ground_z)
                self._ground_pct = mp

        if self._ground_z is None:
            self._pub_filtered.publish(self._empty_cloud(msg.header))
            return

        h = z_h - self._ground_z

        is_ground = np.abs(h) <= GROUND_TOLERANCE
        bands = [
            (h >= 0.0) & (h < HEIGHT_BAND_1_MAX),
            (h >= HEIGHT_BAND_1_MAX) & (h < HEIGHT_BAND_2_MAX),
        ]
        publish = is_ground | bands[0] | bands[1]
        n_a = int(publish.sum())
        if n_a > 0:
            r = np.zeros(n_a, dtype=np.uint8)
            g = np.zeros(n_a, dtype=np.uint8)
            b = np.zeros(n_a, dtype=np.uint8)
            for bi, (rc, gc, bc) in enumerate([(255, 0, 0), (0, 255, 0)]):
                m = bands[bi][publish]
                r[m], g[m], b[m] = rc, gc, bc
            gm = is_ground[publish]
            r[gm], g[gm], b[gm] = 255, 255, 255
            self._pub_filtered.publish(
                self._make_cloud(msg.header, x[publish], y[publish], z[publish], r, g, b))
        else:
            self._pub_filtered.publish(self._empty_cloud(msg.header))

        mask_200_band = (h >= BAND_200_Z_MIN) & (h <= BAND_200_Z_MAX)
        mask_400_band = (h >= BAND_400_Z_MIN) & (h <= BAND_400_Z_MAX)
        mask_facade_slice = (h >= FACADE_SLICE_Z_MIN) & (h <= FACADE_SLICE_Z_MAX)
        mask_facade_slice &= mask_200_band | mask_400_band

        if not mask_facade_slice.any():
            return

        x_f, y_f = x[mask_facade_slice], y[mask_facade_slice]

        num_y_bins = int(np.round((Y_MAX - Y_MIN) / Y_BIN_SIZE))
        y_bin_edges = np.linspace(Y_MIN, Y_MAX, num_y_bins + 1)
        y_bin_idx = np.digitize(y_f, y_bin_edges) - 1
        valid_y = (y_bin_idx >= 0) & (y_bin_idx < num_y_bins)

        num_x_bins = int(np.ceil((X_MAX - X_MIN) / X_BIN_WIDTH))
        x_bin_edges = np.linspace(X_MIN, X_MIN + num_x_bins * X_BIN_WIDTH, num_x_bins + 1)
        x_bin_idx = np.digitize(x_f, x_bin_edges) - 1
        valid_x = (x_bin_idx >= 0) & (x_bin_idx < num_x_bins)

        valid = valid_y & valid_x
        if not valid.any():
            return

        flat_idx = y_bin_idx[valid] * num_x_bins + x_bin_idx[valid]
        counts = np.bincount(flat_idx, minlength=num_y_bins * num_x_bins)
        counts_2d = counts.reshape(num_y_bins, num_x_bins)

        dense_mask_2d = counts_2d >= MIN_DENSITY_PEAK
        row_has_edge = dense_mask_2d.any(axis=1)
        if not row_has_edge.any():
            return

        nearest_x_bin = np.argmax(dense_mask_2d, axis=1)
        dense_rows = np.where(row_has_edge)[0]
        if len(dense_rows) < MIN_EDGE_POINTS:
            return

        edge_x = X_MIN + (nearest_x_bin[dense_rows].astype(np.float64) + 0.5) * X_BIN_WIDTH
        edge_y = Y_MIN + (dense_rows.astype(np.float64) + 0.5) * Y_BIN_SIZE

        try:
            k, b, inlier_mask = self._fit_ransac(edge_x, edge_y)
        except (ValueError, AttributeError):
            return

        if inlier_mask is None or inlier_mask.sum() < MIN_EDGE_POINTS:
            return

        yaw = np.arctan(float(k))
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        x_rot = cos_yaw * x - sin_yaw * y
        y_rot = sin_yaw * x + cos_yaw * y
        x_facade_rot = cos_yaw * float(b)

        on_wall_mask = np.abs(x_rot - x_facade_rot) <= WALL_HALF_WIDTH
        if not on_wall_mask.any():
            return

        y_idx_wall = np.floor((y_rot - Y_COLUMN_MIN) / COL_WIDTH).astype(np.int32)
        valid_wall = (y_idx_wall >= 0) & (y_idx_wall < Y_COLUMN_NUM)
        if not (valid_wall & on_wall_mask).any():
            return

        in_200_wall = mask_200_band & on_wall_mask & valid_wall
        in_400_wall = mask_400_band & on_wall_mask & valid_wall

        cnt_200 = np.bincount(y_idx_wall[in_200_wall], minlength=Y_COLUMN_NUM)
        cnt_400 = np.bincount(y_idx_wall[in_400_wall], minlength=Y_COLUMN_NUM)

        is_yellow_col = (cnt_200 >= 3) & (cnt_400 >= 3)
        is_white_candidate = (cnt_200 >= 3) & (cnt_400 == 0)

        dilated = binary_dilation(is_yellow_col, iterations=DILATE_BINS)
        is_white_col = is_white_candidate & ~dilated

        n_white = int(is_white_col.sum())
        n_yellow = int(is_yellow_col.sum())

        is_white_pt = np.zeros(len(x), dtype=bool)
        is_yellow_pt = np.zeros(len(x), dtype=bool)
        m = valid_wall & on_wall_mask & (h <= 1.0)
        if m.any():
            ci = y_idx_wall[m]
            is_white_pt[m] = is_white_col[ci]
            is_yellow_pt[m] = is_yellow_col[ci]

        have_target = False
        y_center = y_white_min = y_white_max = 0.0
        xs = ys = zs_line = np.empty(0, dtype=np.float64)
        xc = yc = zc_clust = np.empty(0, dtype=np.float64)

        if n_white > 0:
            y_white_vals = y_rot[is_white_pt]
            if len(y_white_vals) > 0:
                have_target = True
                y_white_min = float(y_white_vals.min())
                y_white_max = float(y_white_vals.max())
                y_center = (y_white_min + y_white_max) / 2.0

                if n_yellow > 0:
                    y_yellow_vals = y_rot[is_yellow_pt]
                    if len(y_yellow_vals) > 0:
                        left_yellows = y_yellow_vals[y_yellow_vals < y_center]
                        right_yellows = y_yellow_vals[y_yellow_vals > y_center]

                        has_left_wall = len(left_yellows) > 0
                        has_right_wall = len(right_yellows) > 0

                        if has_left_wall and has_right_wall:
                            left_wall_edge = float(left_yellows.max())
                            right_wall_edge = float(right_yellows.min())
                            y_center = (left_wall_edge + right_wall_edge) / 2.0
                        elif has_right_wall and not has_left_wall:
                            right_wall_edge = float(right_yellows.min())
                            y_center = right_wall_edge - 0.60
                        elif has_left_wall and not has_right_wall:
                            left_wall_edge = float(left_yellows.max())
                            y_center = left_wall_edge + 0.60

                zw = z[is_white_pt & mask_200_band]
                z_synth = float(zw.mean()) if len(zw) > 10 else 0.0

                y_line = np.linspace(y_white_min, y_white_max, SYNTHETIC_LINE_N)
                x_line = np.full_like(y_line, x_facade_rot)
                z_line = np.full_like(y_line, z_synth)

                xs = cos_yaw * x_line + sin_yaw * y_line
                ys = -sin_yaw * x_line + cos_yaw * y_line
                zs_line = z_line

                x_red = x_facade_rot + self._rng.normal(0, SYNTHETIC_CLUSTER_SPREAD, SYNTHETIC_CLUSTER_N)
                y_red = y_center + self._rng.normal(0, SYNTHETIC_CLUSTER_SPREAD, SYNTHETIC_CLUSTER_N)
                z_red = z_synth + self._rng.normal(0, SYNTHETIC_CLUSTER_SPREAD, SYNTHETIC_CLUSTER_N)

                xc = cos_yaw * x_red + sin_yaw * y_red
                yc = -sin_yaw * x_red + cos_yaw * y_red
                zc_clust = z_red

        if is_yellow_pt.any():
            x_out = x[is_yellow_pt]
            y_out = y[is_yellow_pt]
            z_out = z[is_yellow_pt]
            r = np.full(len(x_out), 255, dtype=np.uint8)
            g = np.full(len(x_out), 255, dtype=np.uint8)
            b = np.zeros(len(x_out), dtype=np.uint8)
        else:
            x_out = np.empty(0, dtype=np.float32)
            y_out = np.empty(0, dtype=np.float32)
            z_out = np.empty(0, dtype=np.float32)
            r = np.empty(0, dtype=np.uint8)
            g = np.empty(0, dtype=np.uint8)
            b = np.empty(0, dtype=np.uint8)

        if have_target:
            n_ws = len(xs)
            x_out = np.concatenate([x_out, xs.astype(np.float32)])
            y_out = np.concatenate([y_out, ys.astype(np.float32)])
            z_out = np.concatenate([z_out, zs_line.astype(np.float32)])
            r = np.concatenate([r, np.full(n_ws, 255, dtype=np.uint8)])
            g = np.concatenate([g, np.full(n_ws, 255, dtype=np.uint8)])
            b = np.concatenate([b, np.full(n_ws, 255, dtype=np.uint8)])

            n_rc = len(xc)
            x_out = np.concatenate([x_out, xc.astype(np.float32)])
            y_out = np.concatenate([y_out, yc.astype(np.float32)])
            z_out = np.concatenate([z_out, zc_clust.astype(np.float32)])
            r = np.concatenate([r, np.full(n_rc, 255, dtype=np.uint8)])
            g = np.concatenate([g, np.zeros(n_rc, dtype=np.uint8)])
            b = np.concatenate([b, np.zeros(n_rc, dtype=np.uint8)])

        if len(x_out) == 0:
            self._pub_facade.publish(self._empty_cloud(msg.header))
            return

        self._pub_facade.publish(
            self._make_cloud(msg.header, x_out, y_out, z_out, r, g, b))

        if have_target:
            self._detected_facade_x = float(x_facade_rot)
            self._detected_facade_y = float(y_center)
            self._detected_yaw = float(yaw)
            self._write_detail_log(
                "candidate",
                facade_x=self._detected_facade_x,
                facade_y=self._detected_facade_y,
                yaw_deg=math.degrees(self._detected_yaw),
                white_cols=n_white,
                yellow_cols=n_yellow,
                white_min=y_white_min,
                white_max=y_white_max,
            )

            self._publish_zone2_root_tf(
                self._detected_facade_x,
                self._detected_facade_y,
                self._detected_yaw,
                msg.header.frame_id,
            )

        if LOG_ENABLED and time.monotonic() - self._last_log >= LOG_INTERVAL:
            self._last_log = time.monotonic()
            yaw_deg = float(np.degrees(yaw))
            if have_target:
                self.get_logger().info(
                    f"[靶心] Y坐标: {y_center:.3f} m | "
                    f"[位姿] 相对车头 Yaw: {yaw_deg:.1f} 度 | "
                    f"白线Y_range=[{y_white_min:.2f},{y_white_max:.2f}]m | "
                    f"黄柱={n_yellow}")
            else:
                self.get_logger().info(
                    f"[靶心] Y坐标: nan m | "
                    f"[位姿] 相对车头 Yaw: {yaw_deg:.1f} 度 | "
                    f"黄柱={n_yellow}")

    def _fit_ransac(self, edge_x, edge_y):
        n = len(edge_x)
        if n < 2:
            raise ValueError("Too few points for RANSAC")

        best_k, best_b = 0.0, 0.0
        best_inliers = np.zeros(n, dtype=bool)
        best_inlier_count = 0
        rng = np.random.default_rng()

        for _ in range(RANSAC_N_ITER):
            idx = rng.choice(n, 2, replace=False)
            y1, y2 = edge_y[idx[0]], edge_y[idx[1]]
            x1, x2 = edge_x[idx[0]], edge_x[idx[1]]

            if abs(y2 - y1) < 1e-10:
                continue
            k = (x2 - x1) / (y2 - y1)
            if abs(k) > MAX_ALLOWED_SLOPE:
                continue

            b_i = x1 - k * y1
            residuals = np.abs(edge_x - (k * edge_y + b_i))
            inliers = residuals <= RANSAC_RESIDUAL_THRESHOLD
            inlier_count = int(inliers.sum())

            if inlier_count > best_inlier_count:
                best_inlier_count = inlier_count
                best_k, best_b = k, b_i
                best_inliers = inliers

        if best_inlier_count < MIN_EDGE_POINTS:
            raise ValueError(
                f"Insufficient inliers under |k|<={MAX_ALLOWED_SLOPE}: "
                f"{best_inlier_count} < {MIN_EDGE_POINTS}")

        y_in = edge_y[best_inliers]
        x_in = edge_x[best_inliers]
        A = np.vstack([y_in, np.ones_like(y_in)]).T
        k_opt, b_opt = np.linalg.lstsq(A, x_in, rcond=None)[0]

        if abs(float(k_opt)) > MAX_ALLOWED_SLOPE:
            raise ValueError(
                f"Refined slope out of bound: |{float(k_opt):.3f}| > {MAX_ALLOWED_SLOPE}")

        return float(k_opt), float(b_opt), best_inliers

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
                self.get_logger().warn(f"TF {src}->{HEIGHT_FRAME} 不可用: {e}")
            return z

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
        if len(z) == 0:
            return 0.0, 0.0
        bins = self._hist_bins(z)
        hist, edges = np.histogram(z, bins=bins)
        peak = hist.max()
        idx = next((i for i, v in enumerate(hist)
                    if v >= peak * GROUND_PEAK_RATIO), None)
        if idx is None:
            idx = int(np.argmax(hist))
        z_mid = (edges[idx] + edges[idx + 1]) / 2.0
        return z_mid, 100.0 * hist[idx] / len(z)

    def _hist_bins(self, z):
        zf = z[np.isfinite(z)]
        if len(zf) == 0:
            return np.array([0.0, HISTOGRAM_BIN_WIDTH], dtype=np.float32)
        lo = np.floor(zf.min() / HISTOGRAM_BIN_WIDTH) * HISTOGRAM_BIN_WIDTH
        hi = np.ceil(zf.max() / HISTOGRAM_BIN_WIDTH) * HISTOGRAM_BIN_WIDTH
        if hi <= lo:
            hi = lo + HISTOGRAM_BIN_WIDTH
        return np.arange(lo, hi + HISTOGRAM_BIN_WIDTH, HISTOGRAM_BIN_WIDTH)

    def _keyboard_loop(self):
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], 0.2)
            if readable:
                line = sys.stdin.readline()
                if not line:
                    break
                cmd = line.strip().lower()
                if cmd == 'q':
                    self._write_detail_log("user_quit")
                    rclpy.shutdown()
                    break
                elif cmd == '' or cmd == ' ':
                    if not self._zone2_root_locked:
                        if self._last_candidate is None:
                            self.get_logger().warn('[锁定] 暂无有效 zone2_root 候选，忽略 Enter')
                            continue
                        self._zone2_root_locked = True
                        self._locked_tf_x, self._locked_tf_y, self._locked_tf_yaw = self._last_candidate
                        self.get_logger().info(
                            f"[锁定] {self._zone2_root_frame}: "
                            f"x={self._locked_tf_x:.3f} y={self._locked_tf_y:.3f} "
                            f"yaw={math.degrees(self._locked_tf_yaw):.1f}deg")

    def _publish_zone2_root_tf(self, facade_x, facade_y, yaw, source_frame):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        lx = cos_yaw * facade_x + sin_yaw * facade_y
        ly = -sin_yaw * facade_x + cos_yaw * facade_y

        ox, oy = lx, ly
        syaw = 0.0
        dx, dy = 0.0, 0.0
        if source_frame != 'odom':
            try:
                t = self._tf_buffer.lookup_transform('odom', source_frame, Time())
                dx = t.transform.translation.x
                dy = t.transform.translation.y
                _, _, syaw = self._quat_to_rpy(t.transform.rotation)
                rx = math.cos(syaw) * lx - math.sin(syaw) * ly + dx
                ry = math.sin(syaw) * lx + math.cos(syaw) * ly + dy
                ox, oy = rx, ry
            except Exception as e:
                now = time.monotonic()
                if now - self._last_tf_warn > 2.0:
                    self._last_tf_warn = now
                    self.get_logger().warn(f'[TF] 传感器→odom 查询失败: {e}')
                return

        tf_yaw = ZONE2_ROOT_YAW_OFFSET - yaw + syaw

        vec_sight_x = ox - dx
        vec_sight_y = oy - dy
        v_deep_x = math.sin(tf_yaw)
        v_deep_y = -math.cos(tf_yaw)
        dot_product = v_deep_x * vec_sight_x + v_deep_y * vec_sight_y
        if dot_product < 0:
            tf_yaw += math.pi

        ct = math.cos(tf_yaw)
        st = math.sin(tf_yaw)
        tf_x = ox - (ct * ZONE2_TARGET_X - st * ZONE2_TARGET_Y)
        tf_y = oy - (st * ZONE2_TARGET_X + ct * ZONE2_TARGET_Y)

        self._last_candidate = (float(tf_x), float(tf_y), float(tf_yaw))
        self._write_detail_log(
            "root_candidate",
            x=tf_x,
            y=tf_y,
            yaw_deg=math.degrees(tf_yaw),
            locked=int(self._zone2_root_locked),
        )
        if not self._zone2_root_locked:
            self._locked_tf_x = tf_x
            self._locked_tf_y = tf_y
            self._locked_tf_yaw = tf_yaw
            self._send_tf(tf_x, tf_y, ZONE2_ROOT_Z, tf_yaw)
        if (not self._zone2_root_locked and self._auto_lock and
                self._lock_candidate_is_stable(tf_x, tf_y, tf_yaw)):
            self._zone2_root_locked = True
            req = max(self._stable_lock_count,
                      int(self._stable_lock_count / max(self._gate_strength, 0.1)))
            stable = self._pending_locks[-req:]
            self._locked_tf_x = float(np.mean([p[0] for p in stable]))
            self._locked_tf_y = float(np.mean([p[1] for p in stable]))
            self._locked_tf_yaw = self._mean_yaw([p[2] for p in stable])
            self.get_logger().info(
                f"[自动锁定] {self._zone2_root_frame}: "
                f"x={self._locked_tf_x:.3f} y={self._locked_tf_y:.3f} "
                f"yaw={math.degrees(self._locked_tf_yaw):.1f}deg")
            self._pending_refines.clear()
        elif self._zone2_root_locked:
            self._try_refine(tf_x, tf_y, tf_yaw)

    def _lock_candidate_is_stable(self, tf_x, tf_y, tf_yaw):
        # 根据 gate_strength 调整稳锁所需帧数:
        #   中心 (strength=1.0): 需要 _stable_lock_count 帧
        #   边缘 (strength=0.1): 需要更多帧, 保守锁定
        required = max(self._stable_lock_count,
                       int(self._stable_lock_count / max(self._gate_strength, 0.1)))

        self._pending_locks.append((float(tf_x), float(tf_y), float(tf_yaw)))
        if len(self._pending_locks) > required:
            self._pending_locks = self._pending_locks[-required:]
        if len(self._pending_locks) < required:
            return False
        xs = np.asarray([p[0] for p in self._pending_locks], dtype=np.float64)
        ys = np.asarray([p[1] for p in self._pending_locks], dtype=np.float64)
        yaws = [p[2] for p in self._pending_locks]
        center_spread = float(np.max(np.hypot(xs - xs.mean(), ys - ys.mean())))
        yaw0 = yaws[0]
        yaw_spread = max(abs(self._norm_angle(y - yaw0)) for y in yaws)
        return center_spread <= self._stable_center_tol and yaw_spread <= self._stable_yaw_tol

    def _try_refine(self, tf_x, tf_y, tf_yaw):
        if self._distance_to_locked(tf_x, tf_y) > self._refine_max_translation:
            self._pending_refines.clear()
            return
        if abs(self._norm_angle(tf_yaw - self._locked_tf_yaw)) > self._refine_max_yaw_delta:
            self._pending_refines.clear()
            return

        self._pending_refines.append((float(tf_x), float(tf_y), float(tf_yaw)))
        if len(self._pending_refines) > self._refine_stable_count:
            self._pending_refines = self._pending_refines[-self._refine_stable_count:]
        if len(self._pending_refines) < self._refine_stable_count:
            return

        xs = np.asarray([p[0] for p in self._pending_refines], dtype=np.float64)
        ys = np.asarray([p[1] for p in self._pending_refines], dtype=np.float64)
        yaws = [p[2] for p in self._pending_refines]
        center_spread = float(np.max(np.hypot(xs - xs.mean(), ys - ys.mean())))
        yaw0 = yaws[0]
        yaw_spread = max(abs(self._norm_angle(y - yaw0)) for y in yaws)
        if center_spread > self._refine_center_tol or yaw_spread > self._refine_yaw_tol:
            return

        new_x = float(np.mean(xs))
        new_y = float(np.mean(ys))
        new_yaw = self._mean_yaw(yaws)
        old_x, old_y, old_yaw = self._locked_tf_x, self._locked_tf_y, self._locked_tf_yaw
        # 用 gate_strength 缩放 alpha: 中心快修, 边缘慢修
        a = max(0.0, min(1.0, self._refine_alpha * self._gate_strength))
        self._locked_tf_x = old_x + a * (new_x - old_x)
        self._locked_tf_y = old_y + a * (new_y - old_y)
        self._locked_tf_yaw = self._norm_angle(old_yaw + a * self._norm_angle(new_yaw - old_yaw))
        self._pending_refines.clear()
        if self._refine_once:
            self._refine_done = True
        self._write_detail_log(
            "entry_refine",
            old_x=old_x,
            old_y=old_y,
            old_yaw_deg=math.degrees(old_yaw),
            new_x=self._locked_tf_x,
            new_y=self._locked_tf_y,
            new_yaw_deg=math.degrees(self._locked_tf_yaw),
        )
        self.get_logger().info(
            f"[入口修正] {self._zone2_root_frame}: "
            f"x={old_x:.3f}->{self._locked_tf_x:.3f} "
            f"y={old_y:.3f}->{self._locked_tf_y:.3f} "
            f"yaw={math.degrees(old_yaw):.1f}->{math.degrees(self._locked_tf_yaw):.1f}deg")

    def _odom_gate_check(self) -> bool:
        """检查机器人位置+朝向是否在 Z2 最佳观测区域内.

        Returns:
            True  = 位置和朝向都符合 (或 gate 未启用), 允许检测
            False = 不符合, 跳过本帧
        """
        if not self._enable_odom_gate:
            self._gate_strength = 1.0
            return True

        if self._robot_odom is None:
            return False

        rx, ry, ryaw = self._robot_odom

        # ---- 位置检查 ----
        dx = abs(rx - self._gate_center_x)
        dy = abs(ry - self._gate_center_y)
        max_dx = max(dx / self._gate_half_size_x, 0.0)
        max_dy = max(dy / self._gate_half_size_y, 0.0)
        ratio = max(max_dx, max_dy)
        if ratio > 1.0:
            self._gate_strength = 0.0
            return False

        # ---- 朝向检查: 车头相对 odom 正前方不超过 ±tol ----
        if abs(self._norm_angle(ryaw)) > self._gate_yaw_tolerance_rad:
            self._gate_strength = 0.0
            return False

        # 0=正中心, 1=边缘; strength 0.1~1.0
        self._gate_strength = max(0.1, 1.0 - ratio * self._gate_strength_attenuation)
        return True

    def _refine_gate_open(self):
        if not self._zone2_root_locked or self._robot_odom is None:
            return False
        lx, ly, lyaw = self._robot_in_zone2_frame()
        in_entry = (
            abs(lx) <= self._refine_entry_x_abs_max and
            self._refine_entry_y_min <= ly <= self._refine_entry_y_max
        )
        facing_merlin = abs(self._norm_angle(lyaw - self._refine_facing_yaw)) <= self._refine_facing_yaw_tol
        ok = in_entry and facing_merlin
        if not ok:
            self._log_refine_gate(lx, ly, lyaw)
        return ok

    def _robot_in_zone2_frame(self):
        rx, ry, ryaw = self._robot_odom
        dx = rx - self._locked_tf_x
        dy = ry - self._locked_tf_y
        c = math.cos(self._locked_tf_yaw)
        s = math.sin(self._locked_tf_yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        local_yaw = self._norm_angle(ryaw - self._locked_tf_yaw)
        return local_x, local_y, local_yaw

    def _log_refine_gate(self, lx, ly, lyaw):
        now = time.monotonic()
        if now - self._last_refine_log >= 2.0:
            self._last_refine_log = now
            self._write_detail_log(
                "entry_refine_wait",
                local_x=lx,
                local_y=ly,
                local_yaw_deg=math.degrees(lyaw),
            )

    def _init_detail_log(self):
        if not self._detailed_file_log:
            return
        os.makedirs(DETAIL_LOG_DIR, exist_ok=True)
        self._detail_log_path = os.path.join(
            DETAIL_LOG_DIR, f"zone2_detail_{_make_detail_log_stamp()}.csv")
        with open(self._detail_log_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["time", "event", "data"])
        self._detail_log_ready = True

    def _write_detail_log(self, event, **data):
        if not self._detail_log_ready:
            return
        payload = " ".join(f"{k}={v}" for k, v in data.items())
        with open(self._detail_log_path, "a", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow([f"{time.time():.3f}", event, payload])

    def _distance_to_locked(self, x, y):
        return float(math.hypot(float(x) - self._locked_tf_x, float(y) - self._locked_tf_y))

    def _send_tf(self, x, y, z, yaw):
        t = TransformStamped()
        t.header.frame_id = 'odom'
        t.header.stamp = self.get_clock().now().to_msg()
        t.child_frame_id = self._zone2_root_frame
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)

    def _tf_timer_cb(self):
        if self._zone2_root_locked:
            self._send_tf(self._locked_tf_x, self._locked_tf_y, ZONE2_ROOT_Z,
                          self._locked_tf_yaw)

    @staticmethod
    def _norm_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _mean_yaw(self, yaws):
        s = float(np.mean([math.sin(y) for y in yaws]))
        c = float(np.mean([math.cos(y) for y in yaws]))
        return self._norm_angle(math.atan2(s, c))

    @staticmethod
    def _quat_to_rpy(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                          1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0,
                          2.0 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def _parse_xyz(self, cloud):
        n = cloud.width * cloud.height if cloud.height > 1 else cloud.width
        dt = np.dtype({'names': ['x', 'y', 'z'],
                       'formats': [np.float32] * 3,
                       'offsets': [0, 4, 8],
                       'itemsize': cloud.point_step})
        pts = np.frombuffer(cloud.data, dtype=dt, count=n)
        return pts['x'], pts['y'], pts['z']

    def _empty_cloud(self, header):
        return self._make_cloud(header,
                                np.empty(0, dtype=np.float32),
                                np.empty(0, dtype=np.float32),
                                np.empty(0, dtype=np.float32),
                                np.empty(0, dtype=np.uint8),
                                np.empty(0, dtype=np.uint8),
                                np.empty(0, dtype=np.uint8))

    def _make_cloud(self, header, x, y, z, r, g, b):
        n = len(x)
        a = np.full(n, 255, dtype=np.uint8)
        pts = np.zeros(n, dtype=[('x', np.float32), ('y', np.float32),
                                 ('z', np.float32), ('rgb', np.uint32)])
        pts['x'], pts['y'], pts['z'] = x, y, z
        pts['rgb'] = (a.astype(np.uint32) << 24) \
                     | (r.astype(np.uint32) << 16) \
                     | (g.astype(np.uint32) << 8) \
                     | b.astype(np.uint32)

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
            for n, o in [('x', 0), ('y', 4), ('z', 8), ('rgb', 12)]]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.data = pts.tobytes()
        msg.is_dense = True
        return msg


def main():
    rclpy.init()
    node = Zone2DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
