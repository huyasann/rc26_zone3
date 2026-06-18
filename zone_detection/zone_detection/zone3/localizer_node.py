"""Zone3GridLocalizer — 九宫格点云检测节点.

订阅 /odin1/cloud_slam, 检测高台 3×3 九宫格, 输出 zone3_root TF.

流程:
  点云 → ROI → 地面检测 → 高位筛选 → 多帧累积
  → 连通域分析 → PCA 方向 → 几何评分 → 多帧稳定锁
  → Z2 先验锁 (首次进入重试区) → 精修

依赖: 重试区位置门控 + 朝向门控, Z2 TF (先验)
"""

import math
import os
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker

from zone_detection.common.point_cloud_utils import parse_xyz, make_rgb_cloud, empty_cloud
from zone_detection.common.tf_utils import quat_yaw, norm_angle, mean_yaw, quat_from_yaw
from zone_detection.common.ground_estimator import GroundEstimator
from zone_detection.common.debug_logger import Zone3CsvLogger, HeightDiagLogger, DebugLogger
from zone_detection.zone3 import config as C
from zone_detection.zone3.grid_detect import (
    detect_grid_pose, choose_team_root_pose, GridDetection,
)


class Zone3GridLocalizer(Node):
    """Z3 九宫格点云检测 + 动态 TF 发布."""

    def __init__(self):
        super().__init__("zone3_grid_localizer")

        # ── 参数声明 ──────────────────────────────────────
        self._is_blue_team = bool(
            self.declare_parameter("is_blue_team", bool(C.IS_BLUE_TEAM)).value)
        self._enable_debug_vis = bool(
            self.declare_parameter("enable_debug_vis", bool(C.ENABLE_DEBUG_VIS)).value)
        self._detailed_file_log = bool(
            self.declare_parameter("detailed_file_log", bool(C.DETAILED_FILE_LOG)).value)
        self._detail_log_timestamp = str(self.declare_parameter("detail_log_timestamp", "").value).strip()
        self._detail_log_name = str(self.declare_parameter("detail_log_name", "zone3").value).strip() or "zone3"
        detail_log_root_dir = str(self.declare_parameter("detail_log_root_dir", C.DEBUG_DIR).value).strip() or C.DEBUG_DIR
        self._detail_log_dir = os.path.join(detail_log_root_dir, "zone_detection")
        self._require_odom_gate = bool(
            self.declare_parameter("require_zone3_odom_gate", bool(C.REQUIRE_ZONE3_ODOM_GATE)).value)
        self._enable_z2_prior = bool(
            self.declare_parameter("enable_z2_prior", bool(C.ENABLE_Z2_PRIOR)).value)
        self._robot_tf_frame = str(
            self.declare_parameter("robot_tf_frame", C.ROBOT_TF_FRAME).value)
        self._min_lock_confidence = float(
            self.declare_parameter("min_lock_confidence", C.MIN_LOCK_CONFIDENCE).value)
        self._stable_lock_count = int(
            self.declare_parameter("stable_lock_count", C.STABLE_LOCK_COUNT).value)
        self._stable_center_tol = float(
            self.declare_parameter("stable_center_tol", C.STABLE_CENTER_TOL).value)
        self._stable_yaw_tol = float(
            self.declare_parameter("stable_yaw_tol", C.STABLE_YAW_TOL).value)
        self._accumulate_frames = int(
            self.declare_parameter("accumulate_frames", C.ACCUMULATE_FRAMES).value)
        self._z3_refine_max_translation = float(
            self.declare_parameter("z3_refine_max_translation", C.Z3_REFINE_MAX_TRANSLATION).value)
        self._z3_refine_max_yaw_delta = float(
            self.declare_parameter("z3_refine_max_yaw_delta", float(C.Z3_REFINE_MAX_YAW_DELTA)).value)
        self._z3_refine_alpha = float(
            self.declare_parameter("z3_refine_alpha", C.Z3_REFINE_ALPHA).value)
        self._z3_refine_stable_count = int(
            self.declare_parameter("z3_refine_stable_count", C.Z3_REFINE_STABLE_COUNT).value)

        # 重试区参数
        if self._is_blue_team:
            def_ret_x, def_ret_y = C.BLUE_RETRY_CENTER_X, C.BLUE_RETRY_CENTER_Y
            def_face_yaw = C.BLUE_RETRY_FACE_YAW
        else:
            def_ret_x, def_ret_y = C.RED_RETRY_CENTER_X, C.RED_RETRY_CENTER_Y
            def_face_yaw = C.RED_RETRY_FACE_YAW
        self._retry_center_x = float(
            self.declare_parameter("retry_center_x", def_ret_x).value)
        self._retry_center_y = float(
            self.declare_parameter("retry_center_y", def_ret_y).value)
        self._retry_half_size = float(
            self.declare_parameter("retry_half_size", C.RETRY_AREA_HALF_SIZE).value)
        self._retry_z = float(
            self.declare_parameter("retry_z", C.RETRY_AREA_Z).value)
        self._retry_face_yaw = float(
            self.declare_parameter("retry_face_yaw", def_face_yaw).value)
        self._retry_face_yaw_tol = float(
            self.declare_parameter("retry_face_yaw_tol", float(C.RETRY_FACE_YAW_TOLERANCE)).value)

        self._zone3_root_frame = ("blue_" if self._is_blue_team else "red_") + "zone3_root"
        self._z2_frame = ("blue_" if self._is_blue_team else "red_") + "zone2_root"
        self._grid_center_rel_x = C.GRID_FIELD_X - (
            C.BLUE_ZONE3_ROOT_X if self._is_blue_team else C.RED_ZONE3_ROOT_X)
        self._grid_center_rel_y = C.GRID_FIELD_Y - C.ZONE3_ROOT_FIELD_Y

        # ── 话题 ──────────────────────────────────────────
        self.sub = self.create_subscription(
            PointCloud2, "/odin1/cloud_slam", self.cloud_cb, 10)
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
            for attr in ("_pub_candidates", "_pub_model", "_pub_retry_marker", "_pub_height_bands"):
                setattr(self, attr, None)

        # ── TF ────────────────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── 地面估计 ──────────────────────────────────────
        known_z = C.GROUND_Z if C.GROUND_Z_KNOWN else None
        self._ground_est = GroundEstimator(self, known_z)

        # ── 状态 ──────────────────────────────────────────
        self._locked = False
        self._refined = False
        self._tf_x = 0.0
        self._tf_y = 0.0
        self._tf_yaw = 0.0
        self._frames: list = []
        self._pending_locks: list = []
        self._pending_refines: list = []
        self._last_detection: Optional[GridDetection] = None
        self._last_header = None
        self._gate_logged = False

        # ── 日志 ──────────────────────────────────────────
        self._last_log = 0.0
        self._last_tf_warn = 0.0
        self._last_gate_log = 0.0
        self._debug_csv = Zone3CsvLogger(
            self._detailed_file_log, self._detail_log_dir, f"{self._detail_log_name}_debug",
            self._detail_log_timestamp)
        self._event_log = DebugLogger(
            self._detailed_file_log, self._detail_log_dir, f"{self._detail_log_name}_event",
            self._detail_log_timestamp)
        self._height_diag_log = HeightDiagLogger(
            bool(self.declare_parameter("enable_height_diag_log", bool(C.ENABLE_HEIGHT_DIAG_LOG)).value),
            self._detail_log_dir,
            self._detail_log_timestamp)

        # ── 定时器 ────────────────────────────────────────
        self._tf_timer = self.create_timer(1.0 / C.DYNAMIC_TF_RATE, self._publish_tf_timer)

        self._event_log.write_event(
            "start",
            child_frame=self._zone3_root_frame,
            grid_rel_x=self._grid_center_rel_x,
            grid_rel_y=self._grid_center_rel_y,
            odom_gate=int(self._require_odom_gate),
        )

    # ════════════════════════════════════════════════════════
    # 主回调
    # ════════════════════════════════════════════════════════

    def cloud_cb(self, msg):
        """主处理管线: 门控 → 地面 → 高位累积 → 九宫格检测 → 锁定/精修."""
        if msg.width == 0 or msg.height == 0:
            return

        x, y, z = parse_xyz(msg)
        if len(x) == 0:
            return
        x = x[::C.DOWNSAMPLE_STEP]
        y = y[::C.DOWNSAMPLE_STEP]
        z = z[::C.DOWNSAMPLE_STEP]

        # ROI 裁剪 (按帧类型)
        src = msg.header.frame_id
        if src == "odom":
            rx_min, rx_max = C.ODOM_X_MIN, C.ODOM_X_MAX
            ry_min, ry_max = C.ODOM_Y_MIN, C.ODOM_Y_MAX
        else:
            rx_min, rx_max = C.X_MIN, C.X_MAX
            ry_min, ry_max = C.Y_MIN, C.Y_MAX
        roi = ((x >= rx_min) & (x <= rx_max) &
               (y >= ry_min) & (y <= ry_max) &
               (z >= C.Z_MIN) & (z <= C.Z_MAX))
        if not roi.any():
            self._publish_empty_vis(msg.header)
            return
        x, y, z = x[roi], y[roi], z[roi]

        # 高度归一
        z_h = self._ground_est.to_height_frame(msg.header, x, y, z)
        if z_h is None or len(z_h) < 40:
            self._publish_empty_vis(msg.header)
            return

        # 地面更新
        if not C.GROUND_Z_KNOWN:
            self._ground_est.update_ground(z_h)
        if self._ground_est.ground_z is None:
            return
        h = z_h - self._ground_est.ground_z

        # 高度诊断 + 染色发布
        if self._height_diag_log:
            stamp_f = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            self._height_diag_log.write(stamp_f, src, x, y, z_h, h, C.HEIGHT_BANDS)
        self._publish_height_bands(msg.header, x, y, z, h)

        # ── 重试区门控 ──
        if self._require_odom_gate:
            pose = self._get_robot_pose()
            if pose is None:
                self._log_gate(f"no tf {self._robot_tf_frame}")
                self._frames.clear()
                self._pending_locks.clear()
                self._publish_empty_vis(msg.header)
                return
            rx, ry, ryaw = pose

            # 位置门控
            in_pos = (abs(rx - self._retry_center_x) <= self._retry_half_size and
                      abs(ry - self._retry_center_y) <= self._retry_half_size)
            if not in_pos:
                self._gate_logged = False
                self._log_gate(
                    f"retry_pos robot=({rx:.2f},{ry:.2f}) "
                    f"center=({self._retry_center_x:.2f},{self._retry_center_y:.2f})")
                self._frames.clear()
                self._pending_locks.clear()
                self._publish_empty_vis(msg.header)
                return

            # Z2 先验锁 (仅首次进入)
            if not self._locked and self._enable_z2_prior:
                self._try_z2_prior_lock()

            # 朝向门控
            yaw_err = abs(norm_angle(ryaw - self._retry_face_yaw))
            if yaw_err > self._retry_face_yaw_tol:
                self._log_gate(
                    f"retry_yaw robot_yaw={math.degrees(ryaw):.1f} "
                    f"face_yaw={math.degrees(self._retry_face_yaw):.1f} "
                    f"err={math.degrees(yaw_err):.1f}")
                self._frames.clear()
                self._pending_locks.clear()
                self._publish_empty_vis(msg.header)
                return

            if not self._gate_logged:
                self._gate_logged = True
                self.get_logger().info(
                    f"[重试区到达] robot=({rx:.2f},{ry:.2f}) yaw={math.degrees(ryaw):.1f}")

        # ── 累计高位点 + 九宫格检测 ──
        high = (h >= C.GRID_MIN_H) & (h <= C.GRID_MAX_H)
        self._publish_candidates(msg.header, x, y, z, h, high)
        if int(high.sum()) < 40:
            self._publish_empty_vis(msg.header)
            return

        self._frames.append(
            (x[high].astype(np.float64), y[high].astype(np.float64), h[high].astype(np.float64)))
        while len(self._frames) > self._accumulate_frames:
            self._frames.pop(0)
        ax = np.concatenate([f[0] for f in self._frames])
        ay = np.concatenate([f[1] for f in self._frames])
        ah = np.concatenate([f[2] for f in self._frames])

        det = detect_grid_pose(ax, ay, ah, C.GRID_MIN_H, C.GRID_MAX_H)
        if det is None:
            self._publish_empty_vis(msg.header)
            self._log_detection(msg.header, len(x), int(high.sum()), None)
            return

        # 转换到 odom 系
        odom_pose = self._grid_pose_to_odom(msg.header.frame_id,
                                              det.center_x, det.center_y, det.yaw)
        if odom_pose is None:
            return
        grid_ox, grid_oy, grid_oyaw = odom_pose

        # 计算 zone3_root 位姿
        root_x, root_y, root_yaw = choose_team_root_pose(
            grid_ox, grid_oy, grid_oyaw,
            self._grid_center_rel_x, self._grid_center_rel_y,
            self._is_blue_team,
        )

        self._last_detection = det
        self._last_header = msg.header

        # 锁定或精修
        if not self._locked and self._lock_candidate_is_stable(det, root_x, root_y, root_yaw):
            self._do_lock()
        elif self._locked and not self._refined and self._zone3_odom_gate_open():
            self._try_refine(root_x, root_y, root_yaw)

        if self._locked:
            self._publish_tf(msg.header.stamp)
        if self._pub_model:
            self._publish_model(msg.header, det)

        self._log_detection(msg.header, len(x), int(high.sum()), det,
                            root_x, root_y, root_yaw)

    # ════════════════════════════════════════════════════════
    # 锁定
    # ════════════════════════════════════════════════════════

    def _is_good_grid_candidate(self, det: GridDetection) -> bool:
        """单帧候选质量检查: confidence / layer / point_count / width / depth."""
        return (
            det.confidence >= self._min_lock_confidence
            and det.layer_count >= 3
            and det.point_count >= 700
            and 1.30 <= det.width <= 1.95
            and 0.15 <= det.depth <= 0.60
        )

    def _lock_candidate_is_stable(self, det, root_x, root_y, root_yaw) -> bool:
        """多帧稳定性检查: 候选质量 + 中心/yaw 离散度."""
        if not self._is_good_grid_candidate(det):
            self._pending_locks.clear()
            return False
        self._pending_locks.append((root_x, root_y, root_yaw, det))
        if len(self._pending_locks) > self._stable_lock_count:
            self._pending_locks = self._pending_locks[-self._stable_lock_count:]
        if len(self._pending_locks) < self._stable_lock_count:
            return False
        xs = np.asarray([p[0] for p in self._pending_locks])
        ys = np.asarray([p[1] for p in self._pending_locks])
        yaws = [p[2] for p in self._pending_locks]
        spread = float(np.max(np.hypot(xs - xs.mean(), ys - ys.mean())))
        yaw_spread = max(abs(norm_angle(y - yaws[0])) for y in yaws)
        return spread <= self._stable_center_tol and yaw_spread <= self._stable_yaw_tol

    def _do_lock(self):
        """多帧稳定后锁定: 取平均位姿作为最终 zone3_root."""
        stable = self._pending_locks[-self._stable_lock_count:]
        self._tf_x = float(np.mean([p[0] for p in stable]))
        self._tf_y = float(np.mean([p[1] for p in stable]))
        self._tf_yaw = mean_yaw([p[2] for p in stable])
        self._locked = True
        self.get_logger().info(
            f"[zone3] LOCK {self._zone3_root_frame}: "
            f"x={self._tf_x:.3f} y={self._tf_y:.3f} yaw={math.degrees(self._tf_yaw):.1f}deg")

    # ════════════════════════════════════════════════════════
    # Z2 先验锁 + 精修
    # ════════════════════════════════════════════════════════

    def _try_z2_prior_lock(self) -> bool:
        """从 zone2 TF + 固定偏移推算 zone3 初锁 (仅一次, 带 1s TF 冷却)."""
        if self._locked or self._refined or not self._enable_z2_prior:
            return self._locked
        now = time.monotonic()
        if now - getattr(self, '_last_z2_attempt', 0.0) < 1.0:
            return False
        self._last_z2_attempt = now
        try:
            t = self._tf_buffer.lookup_transform("odom", self._z2_frame, Time())
        except Exception:
            return False

        syaw = quat_yaw(t.transform.rotation)
        c, s = math.cos(syaw), math.sin(syaw)
        ox = c * C.Z2_TO_Z3_OFFSET_X - s * C.Z2_TO_Z3_OFFSET_Y
        oy = s * C.Z2_TO_Z3_OFFSET_X + c * C.Z2_TO_Z3_OFFSET_Y
        self._tf_x = t.transform.translation.x + ox
        self._tf_y = t.transform.translation.y + oy
        self._tf_yaw = norm_angle(syaw)
        self._locked = True
        self.get_logger().info(
            f"[zone3] Z2 prior LOCK {self._zone3_root_frame}: "
            f"x={self._tf_x:.3f} y={self._tf_y:.3f} yaw={math.degrees(self._tf_yaw):.1f}")
        return True

    def _try_refine(self, root_x, root_y, root_yaw):
        """在 Z2 先验锁基础上做小范围精修."""
        dx, dy = root_x - self._tf_x, root_y - self._tf_y
        dyaw = abs(norm_angle(root_yaw - self._tf_yaw))
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
        mean_yaw_v = mean_yaw([p[2] for p in self._pending_refines])
        self._tf_x += self._z3_refine_alpha * (mean_x - self._tf_x)
        self._tf_y += self._z3_refine_alpha * (mean_y - self._tf_y)
        self._tf_yaw = self._blend_yaw(self._tf_yaw, mean_yaw_v, self._z3_refine_alpha)
        self._refined = True
        self._pending_refines.clear()
        self.get_logger().info(
            f"[zone3] REFINE {self._zone3_root_frame}: "
            f"x={self._tf_x:.3f} y={self._tf_y:.3f} yaw={math.degrees(self._tf_yaw):.1f}")

    # ════════════════════════════════════════════════════════
    # 位姿转换
    # ════════════════════════════════════════════════════════

    def _grid_pose_to_odom(self, source_frame, gx, gy, gyaw) -> Optional[Tuple[float, float, float]]:
        """将检测到的九宫格位姿从 source_frame 转换到 odom 系."""
        if not source_frame or source_frame == C.SOURCE_FIXED_FRAME:
            return gx, gy, gyaw
        try:
            t = self._tf_buffer.lookup_transform(C.SOURCE_FIXED_FRAME, source_frame, Time())
        except Exception as e:
            now = time.monotonic()
            if now - self._last_tf_warn >= 1.0:
                self._last_tf_warn = now
                self.get_logger().warn(f"TF {source_frame}->{C.SOURCE_FIXED_FRAME}: {e}")
            return None

        syaw = quat_yaw(t.transform.rotation)
        c, s = math.cos(syaw), math.sin(syaw)
        ox = c * gx - s * gy + t.transform.translation.x
        oy = s * gx + c * gy + t.transform.translation.y
        return ox, oy, norm_angle(gyaw + syaw)

    def _get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """从 TF 获取机器人 odom 位姿."""
        try:
            t = self._tf_buffer.lookup_transform("odom", self._robot_tf_frame, Time())
        except Exception as e:
            now = time.monotonic()
            if now - self._last_tf_warn >= 1.0:
                self._last_tf_warn = now
                self.get_logger().warn(f"TF odom->{self._robot_tf_frame}: {e}")
            return None
        return (t.transform.translation.x,
                t.transform.translation.y,
                quat_yaw(t.transform.rotation))

    def _zone3_odom_gate_open(self) -> bool:
        """精修阶段重新检查门控."""
        pose = self._get_robot_pose()
        if pose is None:
            return False
        rx, ry, ryaw = pose
        if abs(rx - self._retry_center_x) > self._retry_half_size:
            return False
        if abs(ry - self._retry_center_y) > self._retry_half_size:
            return False
        return abs(norm_angle(ryaw - self._retry_face_yaw)) <= self._retry_face_yaw_tol

    # ════════════════════════════════════════════════════════
    # 可视化发布
    # ════════════════════════════════════════════════════════

    def _publish_empty_vis(self, header):
        """发布空点云清空 RViz 显示."""
        if self._pub_candidates:
            self._pub_candidates.publish(empty_cloud(header))
        if self._pub_model:
            self._pub_model.publish(empty_cloud(header))

    def _publish_height_bands(self, header, x, y, z, h):
        """按 HEIGHT_BANDS 调色板染色发布点云, 调试地面和层高."""
        if not self._pub_height_bands:
            return
        n = len(x)
        if n == 0 or not C.HEIGHT_BANDS:
            self._pub_height_bands.publish(empty_cloud(header))
            return
        r = np.full(n, 0, dtype=np.uint8)
        g = np.full(n, 0, dtype=np.uint8)
        b = np.full(n, 0, dtype=np.uint8)
        mask = np.zeros(n, dtype=bool)
        for lo, hi, cr, cg, cb in C.HEIGHT_BANDS:
            in_band = (h >= lo) & (h < hi)
            r[in_band] = cr
            g[in_band] = cg
            b[in_band] = cb
            mask |= in_band
        if not mask.any():
            self._pub_height_bands.publish(empty_cloud(header))
            return
        self._pub_height_bands.publish(
            make_rgb_cloud(header, x[mask], y[mask], z[mask], r[mask], g[mask], b[mask]))

    def _publish_candidates(self, header, x, y, z, h, high):
        """发布高位候选(橙) + 地面(白) 染色点云."""
        if not self._pub_candidates:
            return
        publish = high | (np.abs(h) <= C.GROUND_TOLERANCE)
        if not publish.any():
            self._pub_candidates.publish(empty_cloud(header))
            return
        n = int(publish.sum())
        r = np.full(n, 80, dtype=np.uint8)
        g = np.full(n, 80, dtype=np.uint8)
        b = np.full(n, 80, dtype=np.uint8)
        hp = high[publish]
        r[hp], g[hp], b[hp] = 255, 80, 20
        gp = np.abs(h[publish]) <= C.GROUND_TOLERANCE
        r[gp], g[gp], b[gp] = 255, 255, 255
        self._pub_candidates.publish(
            make_rgb_cloud(header, x[publish], y[publish], z[publish], r, g, b))

    def _publish_model(self, header, det: GridDetection):
        """发布九宫格骨架线框点云 (黄色), 与实测点云叠对比."""
        if not self._pub_model:
            return
        mx, my, mz = self._model_points(det)
        r = np.full(len(mx), 255, dtype=np.uint8)
        g = np.full(len(mx), 255, dtype=np.uint8)
        b = np.zeros(len(mx), dtype=np.uint8)
        self._pub_model.publish(make_rgb_cloud(header, mx, my, mz, r, g, b))

    def _model_points(self, det: GridDetection):
        """构建九宫格 3×3×3 层骨架线框."""
        c = math.cos(det.yaw)
        s = math.sin(det.yaw)
        ys = np.array([-C.GRID_WIDTH_Y / 2.0, 0.0, C.GRID_WIDTH_Y / 2.0])
        zs = np.array([1.07, 1.61, 2.15])
        local = []
        for zc in zs:
            for yc in ys:
                for xx in np.linspace(-C.GRID_DEPTH_X / 2.0, C.GRID_DEPTH_X / 2.0, 8):
                    local.append((xx, yc - 0.25, zc))
                    local.append((xx, yc + 0.25, zc))
                for yy in np.linspace(yc - 0.25, yc + 0.25, 12):
                    local.append((-C.GRID_DEPTH_X / 2.0, yy, zc))
                    local.append((C.GRID_DEPTH_X / 2.0, yy, zc))
        arr = np.asarray(local, dtype=np.float64)
        gz = self._ground_est.ground_z if self._ground_est.ground_z is not None else C.GROUND_Z
        x = det.center_x + c * arr[:, 0] - s * arr[:, 1]
        y = det.center_y + s * arr[:, 0] + c * arr[:, 1]
        z = arr[:, 2] + gz
        return x.astype(np.float32), y.astype(np.float32), z.astype(np.float32)

    def _publish_retry_marker(self):
        """发布重试区绿色半透明方块 (RViz 调试)."""
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
        qx, qy, qz, qw = quat_from_yaw(self._retry_face_yaw)
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

    # ════════════════════════════════════════════════════════
    # TF 发布
    # ════════════════════════════════════════════════════════

    def _publish_tf(self, stamp):
        """发布 odom → zone3_root TransformStamped."""
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = C.SOURCE_FIXED_FRAME
        tf_msg.child_frame_id = self._zone3_root_frame
        tf_msg.transform.translation.x = float(self._tf_x)
        tf_msg.transform.translation.y = float(self._tf_y)
        tf_msg.transform.translation.z = float(C.GROUND_Z)
        qx, qy, qz, qw = quat_from_yaw(self._tf_yaw)
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf_msg)

    def _publish_tf_timer(self):
        """定时器: 锁定后 10Hz 发 TF, 同时发布调试 Marker."""
        if self._locked:
            self._publish_tf(self.get_clock().now().to_msg())
        if self._enable_debug_vis:
            self._publish_retry_test_tf()
            self._publish_retry_marker()

    def _publish_retry_test_tf(self):
        """发布重试区中心 TF (调试用)."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "zone3_retry_test"
        t.transform.translation.x = self._retry_center_x
        t.transform.translation.y = self._retry_center_y
        t.transform.translation.z = self._retry_z
        qx, qy, qz, qw = quat_from_yaw(self._retry_face_yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)

    # ════════════════════════════════════════════════════════
    # 日志
    # ════════════════════════════════════════════════════════

    def _log_gate(self, reason):
        """门控等待日志 (节流到 1Hz)."""
        now = time.monotonic()
        if now - self._last_gate_log >= 1.0:
            self._last_gate_log = now
            self._event_log.write_event("odom_gate_wait", reason=reason)

    def _log_detection(self, header, source_points, high_points, det,
                        root_x=None, root_y=None, root_yaw=None):
        """终端事件日志 + CSV 逐帧记录."""
        now = time.monotonic()
        if now - self._last_log >= C.LOG_INTERVAL:
            self._last_log = now
            if det is None:
                self._event_log.write_event(
                    "detection_none", source_points=source_points, high_points=high_points)
            else:
                self._event_log.write_event(
                    "detection", source_points=source_points, high_points=high_points,
                    confidence=det.confidence, grid_x=det.center_x, grid_y=det.center_y,
                    root_x=root_x, root_y=root_y, yaw_deg=math.degrees(det.yaw),
                    root_yaw_deg=math.degrees(root_yaw) if root_yaw else "",
                    points=det.point_count, width=det.width, depth=det.depth,
                    layers=det.layer_count)

        if not self._detailed_file_log:
            return
        stamp_sec = float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9
        data = {
            "source_points": source_points,
            "high_points": high_points,
            "confidence": det.confidence if det else 0,
            "grid_x": det.center_x if det else "",
            "grid_y": det.center_y if det else "",
            "grid_yaw": det.yaw if det else "",
            "root_x": root_x or "",
            "root_y": root_y or "",
            "root_yaw": root_yaw or "",
            "component_points": det.point_count if det else 0,
            "width": det.width if det else "",
            "depth": det.depth if det else "",
            "layers": det.layer_count if det else 0,
            "extra": "",
        }
        self._debug_csv.write(stamp_sec, "detection" if det else "detection_none", data)

    @staticmethod
    def _blend_yaw(old, new, alpha):
        """用 EMA 系数融合两个 yaw."""
        delta = norm_angle(new - old)
        return norm_angle(old + alpha * delta)

    def __del__(self):
        """清理所有日志文件句柄 (安全: 防止 init 失败时未定义)."""
        for attr in ('_debug_csv', '_event_log', '_height_diag_log'):
            if hasattr(self, attr):
                getattr(self, attr).close()


def main():
    """节点入口: 初始化 → spin → 清理."""
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
