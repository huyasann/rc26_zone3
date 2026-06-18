"""Zone2DetectorNode — 梅林前立面点云检测节点.

订阅 /odin1/cloud_slam, 检测梅林台阶前立面 (白线/黄柱), 输出 zone2_root TF.

全自动运行: 通过 odom 观测位姿门控自动启动检测,
多帧稳定后自动锁定 zone2_root TF, 入口区自动触发精修.

提交流程:
  点云 → ROI → 高度归一 → 地面检测 → 离地高度 h
  → 前立面切片 (200/400 带) → 2D 直方图边缘 → RANSAC 拟合
  → 墙面点筛选 → 白/黄柱判决 → 靶心定位 → zone2_root TF
"""

import math
import os
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from scipy.ndimage import binary_dilation
from sensor_msgs.msg import PointCloud2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from zone_detection.common.point_cloud_utils import parse_xyz, make_rgb_cloud, empty_cloud
from zone_detection.common.tf_utils import quat_to_rpy, norm_angle, mean_yaw
from zone_detection.common.ground_estimator import GroundEstimator
from zone_detection.common.debug_logger import DebugLogger
from zone_detection.zone2 import config as C


class Zone2DetectorNode(Node):
    """Z2 梅林前立面点云检测 + 动态 TF 发布."""

    def __init__(self):
        super().__init__("zone2_detector_node")

        # ── 参数声明 ──────────────────────────────────────
        self._is_blue_team = bool(
            self.declare_parameter("is_blue_team", bool(C.IS_BLUE_TEAM)).value)
        self._detailed_file_log = bool(
            self.declare_parameter("detailed_file_log", bool(C.DETAILED_FILE_LOG)).value)
        self._detail_log_timestamp = str(self.declare_parameter("detail_log_timestamp", "").value).strip()
        self._detail_log_name = str(self.declare_parameter("detail_log_name", "zone2_detail").value).strip() or "zone2_detail"
        detail_log_root_dir = str(self.declare_parameter("detail_log_root_dir", C.DETAIL_LOG_DIR).value).strip() or C.DETAIL_LOG_DIR
        self._detail_log_dir = os.path.join(detail_log_root_dir, "zone_detection")
        self._auto_lock = bool(
            self.declare_parameter("auto_lock_zone2_root", bool(C.AUTO_LOCK_ZONE2_ROOT)).value)
        self._stable_lock_count = int(
            self.declare_parameter("stable_lock_count", C.STABLE_LOCK_COUNT).value)
        self._stable_center_tol = float(
            self.declare_parameter("stable_center_tol", C.STABLE_CENTER_TOL).value)
        self._stable_yaw_tol = float(
            self.declare_parameter("stable_yaw_tol", C.STABLE_YAW_TOL).value)
        self._enable_refinement = bool(
            self.declare_parameter("enable_entry_refinement", bool(C.ENABLE_ENTRY_REFINEMENT)).value)
        self._refine_stable_count = int(
            self.declare_parameter("refine_stable_count", C.REFINE_STABLE_COUNT).value)
        self._refine_center_tol = float(
            self.declare_parameter("refine_center_tol", C.REFINE_CENTER_TOL).value)
        self._refine_yaw_tol = float(
            self.declare_parameter("refine_yaw_tol", C.REFINE_YAW_TOL).value)
        self._refine_max_translation = float(
            self.declare_parameter("refine_max_translation", C.REFINE_MAX_TRANSLATION).value)
        self._refine_max_yaw_delta = float(
            self.declare_parameter("refine_max_yaw_delta", C.REFINE_MAX_YAW_DELTA).value)
        self._refine_entry_x_abs_max = float(
            self.declare_parameter("refine_entry_x_abs_max", C.REFINE_ENTRY_X_ABS_MAX).value)
        self._refine_entry_y_min = float(
            self.declare_parameter("refine_entry_y_min", C.REFINE_ENTRY_Y_MIN).value)
        self._refine_entry_y_max = float(
            self.declare_parameter("refine_entry_y_max", C.REFINE_ENTRY_Y_MAX).value)
        self._refine_facing_yaw = float(
            self.declare_parameter("refine_facing_yaw", C.REFINE_FACING_YAW).value)
        self._refine_facing_yaw_tol = float(
            self.declare_parameter("refine_facing_yaw_tol", C.REFINE_FACING_YAW_TOL).value)
        self._refine_alpha = float(
            self.declare_parameter("refine_alpha", C.REFINE_ALPHA).value)
        self._refine_once = bool(
            self.declare_parameter("refine_once", bool(C.REFINE_ONCE)).value)
        self._enable_odom_gate = bool(
            self.declare_parameter("enable_odom_gate", bool(C.ENABLE_ODOM_GATE)).value)
        self._gate_center_x = float(self.declare_parameter(
            "gate_center_x",
            C.GATE_CENTER_X_DEFAULT if self._is_blue_team else -C.GATE_CENTER_X_DEFAULT).value)
        self._gate_center_y = float(self.declare_parameter(
            "gate_center_y",
            C.GATE_CENTER_Y_DEFAULT if self._is_blue_team else -C.GATE_CENTER_Y_DEFAULT).value)
        self._gate_half_size_x = float(
            self.declare_parameter("gate_half_size_x", C.GATE_HALF_SIZE_X).value)
        self._gate_half_size_y = float(
            self.declare_parameter("gate_half_size_y", C.GATE_HALF_SIZE_Y).value)
        self._gate_strength_attenuation = float(
            self.declare_parameter("gate_strength_attenuation", C.GATE_STRENGTH_ATTENUATION).value)
        self._gate_yaw_tolerance_rad = math.radians(
            self.declare_parameter("gate_yaw_tolerance_deg", C.GATE_YAW_TOLERANCE_DEG).value)
        self._publish_gate_debug = bool(
            self.declare_parameter("publish_gate_debug_tf", bool(C.PUBLISH_GATE_DEBUG_TF)).value)

        self._zone2_root_frame = (
            "blue_" if self._is_blue_team else "red_") + "zone2_root"

        # ── 话题订阅/发布 ─────────────────────────────────
        self.sub = self.create_subscription(
            PointCloud2, "/odin1/cloud_slam", self.cloud_cb, 10)
        self._odom_sub = self.create_subscription(
            Odometry, "/odin1/odometry_highfreq", self._odom_cb, 20)
        self._pub_filtered = self.create_publisher(
            PointCloud2, "/rc26/zone2/cloud_height_bands", 10)
        self._pub_facade = self.create_publisher(
            PointCloud2, "/rc26/zone2/cloud_facade_recon", 10)

        # ── TF ─────────────────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── 地面估计 ───────────────────────────────────────
        known_z = C.GROUND_Z if C.GROUND_Z_KNOWN else None
        self._ground_est = GroundEstimator(self, known_z)

        # ── 状态 ───────────────────────────────────────────
        self._rng = np.random.default_rng()
        self._robot_odom: Optional[Tuple[float, float, float]] = None
        self._zone2_root_locked = False
        self._locked_tf_x = 0.0
        self._locked_tf_y = 0.0
        self._locked_tf_yaw = 0.0
        self._detected_facade_x = 0.0
        self._detected_facade_y = 0.0
        self._detected_yaw = 0.0
        self._pending_locks: list = []
        self._pending_refines: list = []
        self._refine_done = False
        self._gate_strength = 1.0

        # ── 日志 ───────────────────────────────────────────
        self._debug_log = DebugLogger(
            self._detailed_file_log, self._detail_log_dir, self._detail_log_name,
            self._detail_log_timestamp)
        self._last_log = 0.0
        self._last_tf_warn = 0.0
        self._last_refine_log = 0.0

        # ── 定时器 ─────────────────────────────────────────
        self._tf_timer = self.create_timer(1.0 / C.DYNAMIC_TF_RATE, self._tf_timer_cb)

        self._debug_log.write_event(
            "start",
            child_frame=self._zone2_root_frame,
            auto_lock=int(self._auto_lock),
            entry_refine=int(self._enable_refinement),
        )

    # ════════════════════════════════════════════════════════
    # 回调
    # ════════════════════════════════════════════════════════

    def _odom_cb(self, msg):
        """缓存最新 odometry 位姿, 用于门控判断."""
        p = msg.pose.pose.position
        _, _, yaw = quat_to_rpy(msg.pose.pose.orientation)
        self._robot_odom = (float(p.x), float(p.y), float(yaw))

    def cloud_cb(self, msg):
        """主处理管线: 门控 → 地面 → 立面拟合 → 判决 → TF."""
        if not self._odom_gate_check():
            return
        if self._zone2_root_locked and (not self._enable_refinement or self._refine_done):
            return
        if self._zone2_root_locked and not self._refine_gate_open():
            return
        if msg.width == 0 or msg.height == 0:
            return

        x, y, z = parse_xyz(msg)
        if len(x) == 0:
            return

        x = x[::C.DOWNSAMPLE_STEP]
        y = y[::C.DOWNSAMPLE_STEP]
        z = z[::C.DOWNSAMPLE_STEP]

        # ROI 裁剪
        roi = ((x >= C.X_MIN) & (x <= C.X_MAX) &
               (y >= C.Y_MIN) & (y <= C.Y_MAX) &
               (z >= C.Z_MIN) & (z <= C.Z_MAX))
        if not roi.any():
            self._pub_filtered.publish(empty_cloud(msg.header))
            self._pub_facade.publish(empty_cloud(msg.header))
            return
        x, y, z = x[roi], y[roi], z[roi]

        # 高度归一
        z_h = self._ground_est.to_height_frame(msg.header, x, y, z)
        if z_h is None or len(z_h) < 10:
            self._pub_filtered.publish(empty_cloud(msg.header))
            return

        # 地面检测
        if not C.GROUND_Z_KNOWN:
            was_none = self._ground_est.ground_z is None
            self._ground_est.update_ground(z_h)
            if was_none and self._ground_est.ground_z is not None:
                self.get_logger().info(
                    f"[地面] 首帧自动估计完成: ground_z={self._ground_est.ground_z:.4f}m")
                self._debug_log.write_event(
                    "ground_first_estimate",
                    ground_z=self._ground_est.ground_z)
        if self._ground_est.ground_z is None:
            self._pub_filtered.publish(empty_cloud(msg.header))
            return

        h = z_h - self._ground_est.ground_z

        # ── 染色发布 (调试用) ──
        self._publish_height_bands(msg.header, x, y, z, h)

        # ── 立面检测 ──
        result = self._detect_facade(x, y, z, h, msg.header)
        if result is None:
            return
        facade_x, facade_y, yaw, n_white, n_yellow, y_white_min, y_white_max = result

        self._detected_facade_x = facade_x
        self._detected_facade_y = facade_y
        self._detected_yaw = yaw

        # ── 发布 zone2_root TF ──
        self._publish_zone2_root_tf(facade_x, facade_y, yaw, msg.header.frame_id)

        # ── 日志 ──
        if C.LOG_ENABLED and time.monotonic() - self._last_log >= C.LOG_INTERVAL:
            self._last_log = time.monotonic()
            self.get_logger().info(
                f"[靶心] Y={facade_y:.3f}m | Yaw={math.degrees(yaw):.1f}° | "
                f"白Y=[{y_white_min:.2f},{y_white_max:.2f}] | 黄柱={n_yellow}")

    # ════════════════════════════════════════════════════════
    # 立面检测管线
    # ════════════════════════════════════════════════════════

    def _detect_facade(self, x, y, z, h, header):
        """立面拟合 + 白/黄柱判决 + 重建点云发布."""
        # ── 立面切片 ──
        mask_200 = (h >= C.BAND_200_Z_MIN) & (h <= C.BAND_200_Z_MAX)
        mask_400 = (h >= C.BAND_400_Z_MIN) & (h <= C.BAND_400_Z_MAX)
        mask_slice = ((h >= C.FACADE_SLICE_Z_MIN) & (h <= C.FACADE_SLICE_Z_MAX)
                      & (mask_200 | mask_400))
        if not mask_slice.any():
            return None
        x_f, y_f = x[mask_slice], y[mask_slice]

        # ── 2D 直方图 → 边缘点集 ──
        num_y_bins = int(round((C.Y_MAX - C.Y_MIN) / C.Y_BIN_SIZE))
        y_bin_edges = np.linspace(C.Y_MIN, C.Y_MAX, num_y_bins + 1)
        y_bin_idx = np.digitize(y_f, y_bin_edges) - 1
        valid_y = (y_bin_idx >= 0) & (y_bin_idx < num_y_bins)

        num_x_bins = int(np.ceil((C.X_MAX - C.X_MIN) / C.X_BIN_WIDTH))
        x_bin_edges = np.linspace(C.X_MIN, C.X_MIN + num_x_bins * C.X_BIN_WIDTH,
                                  num_x_bins + 1)
        x_bin_idx = np.digitize(x_f, x_bin_edges) - 1
        valid_x = (x_bin_idx >= 0) & (x_bin_idx < num_x_bins)

        valid = valid_y & valid_x
        if not valid.any():
            return None

        flat_idx = y_bin_idx[valid] * num_x_bins + x_bin_idx[valid]
        counts = np.bincount(flat_idx, minlength=num_y_bins * num_x_bins)
        counts_2d = counts.reshape(num_y_bins, num_x_bins)
        dense = counts_2d >= C.MIN_DENSITY_PEAK
        row_has_edge = dense.any(axis=1)
        if not row_has_edge.any():
            return None

        nearest_x_bin = np.argmax(dense, axis=1)
        dense_rows = np.where(row_has_edge)[0]
        if len(dense_rows) < C.MIN_EDGE_POINTS:
            return None

        edge_x = C.X_MIN + (nearest_x_bin[dense_rows].astype(np.float64) + 0.5) * C.X_BIN_WIDTH
        edge_y = C.Y_MIN + (dense_rows.astype(np.float64) + 0.5) * C.Y_BIN_SIZE

        # ── RANSAC 拟合 ──
        try:
            k, b, inlier_mask = self._fit_ransac(edge_x, edge_y)
        except (ValueError, AttributeError):
            return None
        if inlier_mask is None or inlier_mask.sum() < C.MIN_EDGE_POINTS:
            return None

        yaw = math.atan(float(k))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x_rot = cos_yaw * x - sin_yaw * y
        y_rot = sin_yaw * x + cos_yaw * y
        x_facade_rot = cos_yaw * float(b)

        # ── 墙面点筛选 ──
        on_wall = np.abs(x_rot - x_facade_rot) <= C.WALL_HALF_WIDTH
        if not on_wall.any():
            return None

        y_idx = np.floor((y_rot - C.Y_COLUMN_MIN) / C.COL_WIDTH).astype(np.int32)
        valid_wall = (y_idx >= 0) & (y_idx < C.Y_COLUMN_NUM) & on_wall

        cnt_200 = np.bincount(y_idx[mask_200 & valid_wall], minlength=C.Y_COLUMN_NUM)
        cnt_400 = np.bincount(y_idx[mask_400 & valid_wall], minlength=C.Y_COLUMN_NUM)

        is_yellow = (cnt_200 >= 3) & (cnt_400 >= 3)
        is_white_cand = (cnt_200 >= 3) & (cnt_400 == 0)
        is_white = is_white_cand & ~binary_dilation(is_yellow, iterations=C.DILATE_BINS)
        n_white = int(is_white.sum())
        n_yellow = int(is_yellow.sum())

        # 柱级 → 点级映射 (原 zone2.py L502-508)
        is_white_pt = np.zeros(len(x), dtype=bool)
        is_yellow_pt = np.zeros(len(x), dtype=bool)
        m = valid_wall & on_wall & (h <= 1.0)
        if m.any():
            ci = y_idx[m]
            is_white_pt[m] = is_white[ci]
            is_yellow_pt[m] = is_yellow[ci]

        # ── 靶心定位 + 重建点云 ──
        return self._locate_target_and_publish(
            x, y, z, h, yaw, x_facade_rot, y_rot,
            is_white_pt, is_yellow_pt, n_white, n_yellow,
            header,
        )

    def _locate_target_and_publish(self, x, y, z, h, yaw, x_facade_rot, y_rot,
                                     is_white, is_yellow, n_white, n_yellow, header):
        """靶心定位: 白线 Y 中心 ± 黄柱边界约束 + 合成重建点云并发布."""
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        have_target = False
        y_center = y_white_min = y_white_max = 0.0
        xs = ys = zs_line = np.empty(0, dtype=np.float64)
        xc = yc = zc_clust = np.empty(0, dtype=np.float64)

        if n_white > 0:
            y_white_vals = y_rot[is_white]
            if len(y_white_vals) > 0:
                have_target = True
                y_white_min = float(y_white_vals.min())
                y_white_max = float(y_white_vals.max())
                y_center = (y_white_min + y_white_max) / 2.0

                # 黄柱边界约束
                if n_yellow > 0:
                    y_yellow_vals = y_rot[is_yellow]
                    if len(y_yellow_vals) > 0:
                        left = y_yellow_vals[y_yellow_vals < y_center]
                        right = y_yellow_vals[y_yellow_vals > y_center]
                        has_left = len(left) > 0
                        has_right = len(right) > 0
                        if has_left and has_right:
                            y_center = (float(left.max()) + float(right.min())) / 2.0
                        elif has_right:
                            y_center = float(right.min()) - 0.60
                        elif has_left:
                            y_center = float(left.max()) + 0.60

                # 合成白线/红心 (调试可视化)
                zw = z[is_white & (h >= C.BAND_200_Z_MIN) & (h <= C.BAND_200_Z_MAX)]
                z_synth = float(zw.mean()) if len(zw) > 10 else 0.0
                y_line = np.linspace(y_white_min, y_white_max, C.SYNTHETIC_LINE_N)
                x_line = np.full_like(y_line, x_facade_rot)
                z_line_arr = np.full_like(y_line, z_synth)
                xs = cos_yaw * x_line + sin_yaw * y_line
                ys = -sin_yaw * x_line + cos_yaw * y_line
                zs_line = z_line_arr

                x_red = x_facade_rot + self._rng.normal(0, C.SYNTHETIC_CLUSTER_SPREAD, C.SYNTHETIC_CLUSTER_N)
                y_red = y_center + self._rng.normal(0, C.SYNTHETIC_CLUSTER_SPREAD, C.SYNTHETIC_CLUSTER_N)
                z_red = z_synth + self._rng.normal(0, C.SYNTHETIC_CLUSTER_SPREAD, C.SYNTHETIC_CLUSTER_N)
                xc = cos_yaw * x_red + sin_yaw * y_red
                yc = -sin_yaw * x_red + cos_yaw * y_red
                zc_clust = z_red

        # 组装重建点云
        x_out, y_out, z_out, r_out, g_out, b_out = self._build_recon_cloud(
            x, y, z, is_yellow, xs, ys, zs_line, xc, yc, zc_clust, have_target)
        if len(x_out) > 0:
            self._pub_facade.publish(
                make_rgb_cloud(header, x_out, y_out, z_out, r_out, g_out, b_out))

        if have_target:
            self._debug_log.write_event(
                "candidate",
                facade_x=x_facade_rot,
                facade_y=y_center,
                yaw_deg=math.degrees(yaw),
                white_cols=n_white,
                yellow_cols=n_yellow,
                white_min=y_white_min,
                white_max=y_white_max,
            )

        return (x_facade_rot, y_center, yaw, n_white, n_yellow,
                y_white_min, y_white_max) if have_target else None

    def _build_recon_cloud(self, x, y, z, is_yellow,
                            xs, ys, zs, xc, yc, zc, have_target):
        """组装立面重建点云: 黄柱(黄255,255,0) + 白线(白255,255,255) + 红心(红255,0,0)."""
        parts_x, parts_y, parts_z, parts_r, parts_g, parts_b = [], [], [], [], [], []

        # 黄柱
        if is_yellow.any():
            parts_x.append(x[is_yellow].astype(np.float32))
            parts_y.append(y[is_yellow].astype(np.float32))
            parts_z.append(z[is_yellow].astype(np.float32))
            n = int(is_yellow.sum())
            parts_r.append(np.full(n, 255, dtype=np.uint8))
            parts_g.append(np.full(n, 255, dtype=np.uint8))
            parts_b.append(np.zeros(n, dtype=np.uint8))

        if have_target:
            # 白线
            n_ws = len(xs)
            parts_x.append(xs.astype(np.float32))
            parts_y.append(ys.astype(np.float32))
            parts_z.append(zs.astype(np.float32))
            parts_r.append(np.full(n_ws, 255, dtype=np.uint8))
            parts_g.append(np.full(n_ws, 255, dtype=np.uint8))
            parts_b.append(np.full(n_ws, 255, dtype=np.uint8))
            # 红心
            n_rc = len(xc)
            parts_x.append(xc.astype(np.float32))
            parts_y.append(yc.astype(np.float32))
            parts_z.append(zc.astype(np.float32))
            parts_r.append(np.full(n_rc, 255, dtype=np.uint8))
            parts_g.append(np.zeros(n_rc, dtype=np.uint8))
            parts_b.append(np.zeros(n_rc, dtype=np.uint8))

        if not parts_x:
            return (np.empty(0, dtype=np.float32),) * 6

        return (np.concatenate(parts_x), np.concatenate(parts_y),
                np.concatenate(parts_z), np.concatenate(parts_r),
                np.concatenate(parts_g), np.concatenate(parts_b))

    # ════════════════════════════════════════════════════════
    # RANSAC
    # ════════════════════════════════════════════════════════

    @staticmethod
    def _fit_ransac(edge_x, edge_y):
        """RANSAC 拟合直线 x = k*y + b, 限制斜率防误检."""
        n = len(edge_x)
        if n < 2:
            raise ValueError("Too few points for RANSAC")

        best_k, best_b = 0.0, 0.0
        best_inliers = np.zeros(n, dtype=bool)
        best_count = 0
        rng = np.random.default_rng()

        for _ in range(C.RANSAC_N_ITER):
            idx = rng.choice(n, 2, replace=False)
            y1, y2 = edge_y[idx[0]], edge_y[idx[1]]
            x1, x2 = edge_x[idx[0]], edge_x[idx[1]]
            if abs(y2 - y1) < 1e-10:
                continue
            k = (x2 - x1) / (y2 - y1)
            if abs(k) > C.MAX_ALLOWED_SLOPE:
                continue
            b_i = x1 - k * y1
            residuals = np.abs(edge_x - (k * edge_y + b_i))
            inliers = residuals <= C.RANSAC_RESIDUAL_THRESHOLD
            count = int(inliers.sum())
            if count > best_count:
                best_count = count
                best_k, best_b = k, b_i
                best_inliers = inliers

        if best_count < C.MIN_EDGE_POINTS:
            raise ValueError(f"Inliers {best_count} < {C.MIN_EDGE_POINTS}")

        # 最小二乘精化
        y_in = edge_y[best_inliers]
        x_in = edge_x[best_inliers]
        A = np.vstack([y_in, np.ones_like(y_in)]).T
        k_opt, b_opt = np.linalg.lstsq(A, x_in, rcond=None)[0]
        if abs(float(k_opt)) > C.MAX_ALLOWED_SLOPE:
            raise ValueError(f"Refined slope |{float(k_opt):.3f}| > {C.MAX_ALLOWED_SLOPE}")
        return float(k_opt), float(b_opt), best_inliers

    # ════════════════════════════════════════════════════════
    # TF 发布 / 锁定 / 精修
    # ════════════════════════════════════════════════════════

    def _publish_zone2_root_tf(self, facade_x, facade_y, yaw, source_frame):
        """计算并发布 zone2_root TF."""
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        lx = cos_yaw * facade_x + sin_yaw * facade_y
        ly = -sin_yaw * facade_x + cos_yaw * facade_y

        ox, oy = lx, ly
        syaw = 0.0
        dx, dy = 0.0, 0.0
        if source_frame != "odom":
            try:
                t = self._tf_buffer.lookup_transform("odom", source_frame, Time())
                dx = t.transform.translation.x
                dy = t.transform.translation.y
                _, _, syaw = quat_to_rpy(t.transform.rotation)
                ox = math.cos(syaw) * lx - math.sin(syaw) * ly + dx
                oy = math.sin(syaw) * lx + math.cos(syaw) * ly + dy
            except Exception as e:
                now = time.monotonic()
                if now - self._last_tf_warn > 2.0:
                    self._last_tf_warn = now
                    self.get_logger().warn(f"[TF] sensor→odom 查询失败: {e}")
                return

        tf_yaw = C.ZONE2_ROOT_YAW_OFFSET - yaw + syaw

        # 方向判断: 确保 zone2_root 朝向梅林
        vec_sight_x = ox - dx
        vec_sight_y = oy - dy
        v_deep_x = math.sin(tf_yaw)
        v_deep_y = -math.cos(tf_yaw)
        if v_deep_x * vec_sight_x + v_deep_y * vec_sight_y < 0:
            tf_yaw += math.pi

        ct = math.cos(tf_yaw)
        st = math.sin(tf_yaw)
        tf_x = ox - (ct * C.ZONE2_TARGET_X - st * C.ZONE2_TARGET_Y)
        tf_y = oy - (st * C.ZONE2_TARGET_X + ct * C.ZONE2_TARGET_Y)

        self._debug_log.write_event(
            "root_candidate",
            x=tf_x, y=tf_y,
            yaw_deg=math.degrees(tf_yaw),
            locked=int(self._zone2_root_locked),
        )

        if not self._zone2_root_locked:
            self._locked_tf_x, self._locked_tf_y, self._locked_tf_yaw = tf_x, tf_y, tf_yaw
            self._send_tf(tf_x, tf_y, C.ZONE2_ROOT_Z, tf_yaw)

        # 自动锁定
        if not self._zone2_root_locked and self._auto_lock:
            self._try_auto_lock(tf_x, tf_y, tf_yaw)
        elif self._zone2_root_locked:
            self._try_refine(tf_x, tf_y, tf_yaw)

    def _try_auto_lock(self, tf_x, tf_y, tf_yaw):
        """多帧稳定后自动锁定."""
        required = max(self._stable_lock_count,
                       int(self._stable_lock_count / max(self._gate_strength, 0.1)))
        self._pending_locks.append((tf_x, tf_y, tf_yaw))
        if len(self._pending_locks) > required:
            self._pending_locks = self._pending_locks[-required:]
        if len(self._pending_locks) < required:
            return

        xs = np.asarray([p[0] for p in self._pending_locks])
        ys = np.asarray([p[1] for p in self._pending_locks])
        yaws = [p[2] for p in self._pending_locks]
        spread = float(np.max(np.hypot(xs - xs.mean(), ys - ys.mean())))
        yaw_spread = max(abs(norm_angle(y - yaws[0])) for y in yaws)
        if spread > self._stable_center_tol or yaw_spread > self._stable_yaw_tol:
            return

        stable = self._pending_locks[-required:]
        self._locked_tf_x = float(np.mean([p[0] for p in stable]))
        self._locked_tf_y = float(np.mean([p[1] for p in stable]))
        self._locked_tf_yaw = mean_yaw([p[2] for p in stable])
        self._zone2_root_locked = True
        self.get_logger().info(
            f"[自动锁定] {self._zone2_root_frame}: "
            f"x={self._locked_tf_x:.3f} y={self._locked_tf_y:.3f} "
            f"yaw={math.degrees(self._locked_tf_yaw):.1f}deg")
        self._pending_refines.clear()

    def _try_refine(self, tf_x, tf_y, tf_yaw):
        """入口区精修: 在粗锁附近小范围 EMA 修正."""
        if self._distance_to_locked(tf_x, tf_y) > self._refine_max_translation:
            self._pending_refines.clear()
            return
        if abs(norm_angle(tf_yaw - self._locked_tf_yaw)) > self._refine_max_yaw_delta:
            self._pending_refines.clear()
            return

        self._pending_refines.append((tf_x, tf_y, tf_yaw))
        if len(self._pending_refines) > self._refine_stable_count:
            self._pending_refines = self._pending_refines[-self._refine_stable_count:]
        if len(self._pending_refines) < self._refine_stable_count:
            return

        xs = np.asarray([p[0] for p in self._pending_refines])
        ys = np.asarray([p[1] for p in self._pending_refines])
        yaws = [p[2] for p in self._pending_refines]
        spread = float(np.max(np.hypot(xs - xs.mean(), ys - ys.mean())))
        yaw_spread = max(abs(norm_angle(y - yaws[0])) for y in yaws)
        if spread > self._refine_center_tol or yaw_spread > self._refine_yaw_tol:
            return

        new_x, new_y = float(np.mean(xs)), float(np.mean(ys))
        new_yaw = mean_yaw(yaws)
        old = (self._locked_tf_x, self._locked_tf_y, self._locked_tf_yaw)
        a = max(0.0, min(1.0, self._refine_alpha * self._gate_strength))
        self._locked_tf_x = old[0] + a * (new_x - old[0])
        self._locked_tf_y = old[1] + a * (new_y - old[1])
        self._locked_tf_yaw = norm_angle(old[2] + a * norm_angle(new_yaw - old[2]))
        self._pending_refines.clear()
        if self._refine_once:
            self._refine_done = True

        self.get_logger().info(
            f"[入口修正] {self._zone2_root_frame}: "
            f"x={old[0]:.3f}->{self._locked_tf_x:.3f} "
            f"y={old[1]:.3f}->{self._locked_tf_y:.3f} "
            f"yaw={math.degrees(old[2]):.1f}->{math.degrees(self._locked_tf_yaw):.1f}deg")

    # ════════════════════════════════════════════════════════
    # 门控
    # ════════════════════════════════════════════════════════

    def _odom_gate_check(self) -> bool:
        """机器人位置+朝向门控, 同时计算 gate_strength."""
        if not self._enable_odom_gate:
            self._gate_strength = 1.0
            return True
        if self._robot_odom is None:
            return False

        rx, ry, ryaw = self._robot_odom
        dx = abs(rx - self._gate_center_x)
        dy = abs(ry - self._gate_center_y)
        ratio = max(dx / self._gate_half_size_x, dy / self._gate_half_size_y, 0.0)
        if ratio > 1.0:
            self._gate_strength = 0.0
            return False
        if abs(norm_angle(ryaw)) > self._gate_yaw_tolerance_rad:
            self._gate_strength = 0.0
            return False

        self._gate_strength = max(0.1, 1.0 - ratio * self._gate_strength_attenuation)
        return True

    def _refine_gate_open(self) -> bool:
        """检查机器人是否在入口精修区内."""
        if not self._zone2_root_locked or self._robot_odom is None:
            return False
        lx, ly, lyaw = self._robot_in_zone2_frame()
        in_entry = (abs(lx) <= self._refine_entry_x_abs_max
                    and self._refine_entry_y_min <= ly <= self._refine_entry_y_max)
        facing = abs(norm_angle(lyaw - self._refine_facing_yaw)) <= self._refine_facing_yaw_tol
        if not (in_entry and facing):
            now = time.monotonic()
            if now - self._last_refine_log >= 2.0:
                self._last_refine_log = now
                self._debug_log.write_event(
                    "entry_refine_wait", local_x=lx, local_y=ly,
                    local_yaw_deg=math.degrees(lyaw))
        return in_entry and facing

    def _robot_in_zone2_frame(self) -> Tuple[float, float, float]:
        """返回机器人在 zone2_root 系下的 (x, y, yaw)."""
        rx, ry, ryaw = self._robot_odom
        dx = rx - self._locked_tf_x
        dy = ry - self._locked_tf_y
        c = math.cos(self._locked_tf_yaw)
        s = math.sin(self._locked_tf_yaw)
        return (c * dx + s * dy,
                -s * dx + c * dy,
                norm_angle(ryaw - self._locked_tf_yaw))

    # ════════════════════════════════════════════════════════
    # 辅助
    # ════════════════════════════════════════════════════════

    def _publish_height_bands(self, header, x, y, z, h):
        """地面(白) + 低带(绿) + 高带(红) 染色发布."""
        is_ground = np.abs(h) <= C.GROUND_TOLERANCE
        band1 = (h >= 0.0) & (h < C.HEIGHT_BAND_1_MAX)
        band2 = (h >= C.HEIGHT_BAND_1_MAX) & (h < C.HEIGHT_BAND_2_MAX)
        publish = is_ground | band1 | band2
        n = int(publish.sum())
        if n == 0:
            self._pub_filtered.publish(empty_cloud(header))
            return

        r = np.zeros(n, dtype=np.uint8)
        g = np.zeros(n, dtype=np.uint8)
        b = np.zeros(n, dtype=np.uint8)
        for bi, (rc, gc, bc) in enumerate([(255, 0, 0), (0, 255, 0)]):
            m = [band1, band2][bi][publish]
            r[m], g[m], b[m] = rc, gc, bc
        gm = is_ground[publish]
        r[gm], g[gm], b[gm] = 255, 255, 255
        self._pub_filtered.publish(
            make_rgb_cloud(header, x[publish], y[publish], z[publish], r, g, b))

    def _send_tf(self, x, y, z, yaw):
        """发布 odom → zone2_root 单帧 TransformStamped."""
        t = TransformStamped()
        t.header.frame_id = "odom"
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
        """定时器: 锁定后 10Hz 重复发布 TF + 调试 TF."""
        if self._zone2_root_locked:
            self._send_tf(self._locked_tf_x, self._locked_tf_y,
                          C.ZONE2_ROOT_Z, self._locked_tf_yaw)
        if self._publish_gate_debug:
            self._publish_gate_debug_tf()

    def _publish_gate_debug_tf(self):
        """发布观测区域中心调试 TF (zone2_gate_debug), 参考 Z3 _publish_retry_test_tf."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "zone2_gate_debug"
        t.transform.translation.x = float(self._gate_center_x)
        t.transform.translation.y = float(self._gate_center_y)
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t)

    def _distance_to_locked(self, x, y):
        """当前候选到锁定点的欧式距离 (m)."""
        return math.hypot(x - self._locked_tf_x, y - self._locked_tf_y)

    def __del__(self):
        """清理日志文件句柄 (安全: 防止 init 失败时未定义)."""
        if hasattr(self, '_debug_log'):
            self._debug_log.close()


def main():
    """节点入口: 初始化 → spin → 清理."""
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


if __name__ == "__main__":
    main()
