#!/usr/bin/env python3
"""Live BLUE Z3 ramp-based field fit.

Subscribes to /odin1/odometry_highfreq, detects the Z3 ramp while a bag plays,
and publishes odom -> blue_zone3_root after a stable uphill segment is found.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


ODOM_TOPIC = "/odin1/odometry_highfreq"
CLOUD_TOPIC = "/odin1/cloud_slam"
BLUE_ROOT_FRAME = "blue_zone3_root"

# rc26_field.py BLUE Z3 ramp geometry, local to blue_zone3_root.
BLUE_RAMP_X = 2.275
RAMP_LOW_Y = 1.30
RAMP_TOP_Y = -0.20
RAMP_PLATFORM_GAP = 0.08
INNER_PLATFORM_Y_MAX = -0.42
INNER_PLATFORM_X_MAX = 3.15
BLUE_ROOT_YAW_PRIOR = math.pi / 2.0
CLOUD_REFINE_MAX_YAW_PRIOR_ERR = math.radians(7.0)
TOP_TRANSITION_REF_X = 2.275
RAMP_LEFT_REF_X = 1.50
RAMP_RIGHT_REF_X = 3.05
RAMP_WIDTH_REF = RAMP_RIGHT_REF_X - RAMP_LEFT_REF_X
Z3_PLATFORM_Z_REL = 0.40
HISTOGRAM_BIN_WIDTH = 0.02
RIGHT_EDGE_REF_Y = -0.85
RIGHT_EDGE_REF_X = 3.05
INNER_SEAM_REF_X = 1.50


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


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


class BlueZ3LiveRampFit(Node):
    def __init__(self):
        super().__init__("blue_z3_live_ramp_fit")
        self.declare_parameter("odom_topic", ODOM_TOPIC)
        self.declare_parameter("cloud_topic", CLOUD_TOPIC)
        self.declare_parameter("min_z_gain", 0.34)
        self.declare_parameter("max_z_gain", 0.70)
        self.declare_parameter("min_xy_dist", 0.70)
        self.declare_parameter("min_pitch_abs_deg", 18.0)
        self.declare_parameter("window_sec", 35.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("ramp_lock_delay_sec", 2.0)
        self.declare_parameter("ground_min_points", 60)
        self.declare_parameter("ground_roi_min_points", 180)
        self.declare_parameter("ground_prior_max_error", 0.25)
        self.declare_parameter("z_lock_stable_frames", 4)
        self.declare_parameter("cloud_cache_sec", 200.0)
        self.declare_parameter("cloud_cache_max_points", 4000000)
        self.declare_parameter("cloud_refine_min_inliers", 350)
        self.declare_parameter("cloud_refine_min_coverage", 0.65)
        self.declare_parameter("cloud_refine_max_rmse", 0.035)
        self.declare_parameter("ramp_event_min_z_gain", 0.035)
        self.declare_parameter("ramp_event_min_pitch_abs_deg", 6.0)
        self.declare_parameter("provisional_min_z_gain", 0.08)
        self.declare_parameter("provisional_min_xy_dist", 0.35)
        self.declare_parameter("provisional_min_pitch_abs_deg", 8.0)
        self.declare_parameter("debug_cloud_window_sec", 1.4)
        self.declare_parameter("ramp_event_backtrack_sec", 1.0)
        self.declare_parameter("refine_cloud_window_sec", 6.0)
        self.declare_parameter("ground_backtrack_sec", 2.5)
        self.declare_parameter("ground_forward_sec", 0.8)

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._cloud_topic = str(self.get_parameter("cloud_topic").value)
        self._min_z_gain = float(self.get_parameter("min_z_gain").value)
        self._max_z_gain = float(self.get_parameter("max_z_gain").value)
        self._min_xy_dist = float(self.get_parameter("min_xy_dist").value)
        self._min_pitch_abs = math.radians(float(self.get_parameter("min_pitch_abs_deg").value))
        self._window_sec = float(self.get_parameter("window_sec").value)
        self._ramp_lock_delay_sec = float(self.get_parameter("ramp_lock_delay_sec").value)
        self._ground_min_points = int(self.get_parameter("ground_min_points").value)
        self._ground_roi_min_points = int(self.get_parameter("ground_roi_min_points").value)
        self._ground_prior_max_error = float(self.get_parameter("ground_prior_max_error").value)
        self._z_lock_stable_frames = int(self.get_parameter("z_lock_stable_frames").value)
        self._cloud_cache_sec = float(self.get_parameter("cloud_cache_sec").value)
        self._cloud_cache_max_points = int(self.get_parameter("cloud_cache_max_points").value)
        self._cloud_refine_min_inliers = int(self.get_parameter("cloud_refine_min_inliers").value)
        self._cloud_refine_min_coverage = float(self.get_parameter("cloud_refine_min_coverage").value)
        self._cloud_refine_max_rmse = float(self.get_parameter("cloud_refine_max_rmse").value)
        self._ramp_event_min_z_gain = float(self.get_parameter("ramp_event_min_z_gain").value)
        self._ramp_event_min_pitch_abs = math.radians(
            float(self.get_parameter("ramp_event_min_pitch_abs_deg").value)
        )
        self._provisional_min_z_gain = float(self.get_parameter("provisional_min_z_gain").value)
        self._provisional_min_xy_dist = float(self.get_parameter("provisional_min_xy_dist").value)
        self._provisional_min_pitch_abs = math.radians(
            float(self.get_parameter("provisional_min_pitch_abs_deg").value)
        )
        self._debug_cloud_window_sec = float(self.get_parameter("debug_cloud_window_sec").value)
        self._ramp_event_backtrack_sec = float(self.get_parameter("ramp_event_backtrack_sec").value)
        self._refine_cloud_window_sec = float(self.get_parameter("refine_cloud_window_sec").value)
        self._ground_backtrack_sec = float(self.get_parameter("ground_backtrack_sec").value)
        self._ground_forward_sec = float(self.get_parameter("ground_forward_sec").value)

        self._buf: deque[dict] = deque()
        self._cloud_buf: deque[dict] = deque()
        self._locked = False
        self._fit = None
        self._provisional_fit = None
        self._ramp_event_since = None
        self._candidate_fit = None
        self._candidate_since = None
        self._ground_z = None
        self._platform_z = None
        self._cloud_refined = False
        self._ground_pending: deque[float] = deque(maxlen=self._z_lock_stable_frames)
        self._platform_pending: deque[float] = deque(maxlen=self._z_lock_stable_frames)
        self._last_log_t = 0.0
        self._last_cloud_warn_t = 0.0

        self._tf = TransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._marker_pub = self.create_publisher(MarkerArray, "/rc26/zone3/blue_live_ramp_fit_markers", 10)
        self._debug_cloud_pub = self.create_publisher(
            PointCloud2, "/rc26/zone3/blue_live_ramp_fit/cloud_debug", 10
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, sensor_qos)
        self.create_subscription(PointCloud2, self._cloud_topic, self._cloud_cb, sensor_qos)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(1.0, rate), self._publish)
        self.get_logger().info(f"blue_z3_live_ramp_fit listening {self._odom_topic} and {self._cloud_topic}")

    def _odom_cb(self, msg: Odometry):
        stamp = msg.header.stamp
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        p = msg.pose.pose.position
        roll, pitch, yaw = quat_to_rpy(msg.pose.pose.orientation)
        item = {"t": t, "x": p.x, "y": p.y, "z": p.z, "roll": roll, "pitch": pitch, "yaw": yaw, "stamp": stamp}
        self._buf.append(item)
        while self._buf and t - self._buf[0]["t"] > self._window_sec:
            self._buf.popleft()
        if not self._locked:
            self._try_fit()

    def _try_fit(self):
        if len(self._buf) < 80:
            return
        pts = list(self._buf)
        span = self._online_ramp_span(pts)
        if span is None:
            z_gain = pts[-1]["z"] - min(p["z"] for p in pts)
            xy_dist = math.hypot(pts[-1]["x"] - pts[0]["x"], pts[-1]["y"] - pts[0]["y"])
            self._throttled_status(f"wait z_gain={z_gain:.3f} xy={xy_dist:.3f}")
            return
        start, end = span
        low = pts[start]
        top = pts[end]
        z_gain = top["z"] - low["z"]
        xy_dist = math.hypot(top["x"] - low["x"], top["y"] - low["y"])
        pitch_abs = max(abs(p["pitch"]) for p in pts[start : end + 1])
        if (
            self._ramp_event_since is not None
            and not self._locked
            and z_gain >= self._provisional_min_z_gain
            and xy_dist >= self._provisional_min_xy_dist
            and pitch_abs >= self._provisional_min_pitch_abs
        ):
            provisional = self._make_ramp_fit(pts, start, end, partial=True)
            if provisional is not None and (
                self._provisional_fit is None
                or self._ramp_score(provisional) > self._ramp_score(self._provisional_fit)
            ):
                self._provisional_fit = provisional
                self._throttled_status(
                    "provisional "
                    f"z_gain={provisional['z_gain']:.3f} "
                    f"progress={provisional['progress']:.2f} residual={provisional['residual']:.3f}"
                )
        if z_gain < self._min_z_gain or z_gain > self._max_z_gain or xy_dist < self._min_xy_dist:
            self._throttled_status(f"wait z_gain={z_gain:.3f} xy={xy_dist:.3f}")
            return
        if pitch_abs < self._min_pitch_abs:
            self._throttled_status(f"wait pitch_abs={math.degrees(pitch_abs):.1f}")
            return

        fit = self._make_ramp_fit(pts, start, end, partial=False)
        if fit is None:
            self._throttled_status("wait ramp_axis")
            return
        if self._candidate_fit is None:
            self._candidate_fit = fit
            self._candidate_since = pts[-1]["t"]
        elif self._ramp_score(fit) > self._ramp_score(self._candidate_fit):
            self._candidate_fit = fit
        age = pts[-1]["t"] - float(self._candidate_since)
        if age < self._ramp_lock_delay_sec:
            self._throttled_status(
                f"hold ramp age={age:.2f}s z_gain={fit['z_gain']:.3f} residual={fit['residual']:.3f}"
            )
            return

        self._fit = self._candidate_fit
        self._locked = True
        self.get_logger().info(
            "[LOCK] blue_zone3_root "
            f"x={self._fit['root_x']:.3f} y={self._fit['root_y']:.3f} "
            f"yaw={math.degrees(self._fit['root_yaw']):.2f}deg "
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
        back_t = self._ramp_event_since - self._ramp_event_backtrack_sec
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
            if z_gain < self._ramp_event_min_z_gain:
                continue
            local_pitch = max(abs(p["pitch"]) for p in pts[max(0, i - 20) : i + 1])
            if local_pitch < self._ramp_event_min_pitch_abs:
                continue
            xy_dist = math.hypot(pts[i]["x"] - base["x"], pts[i]["y"] - base["y"])
            self._ramp_event_since = base["t"]
            self.get_logger().info(
                "[RAMP_EVENT] uphill started "
                f"t={self._ramp_event_since:.3f} trigger_t={pts[i]['t']:.3f} z_gain={z_gain:.3f} "
                f"xy={xy_dist:.3f} pitch_abs={math.degrees(local_pitch):.1f}deg"
            )
            return

    def _make_ramp_fit(self, pts, start: int, end: int, *, partial: bool):
        low = pts[start]
        top = pts[end]
        seg = pts[start : end + 1]
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
        progress = np.clip((top_z - low_z) / Z3_PLATFORM_Z_REL, 0.0, 1.0)
        top_local_y = (
            RAMP_LOW_Y + float(progress) * (RAMP_TOP_Y - RAMP_LOW_Y)
            if partial
            else RAMP_TOP_Y
        )
        top_dx, top_dy = rot(root_yaw, BLUE_RAMP_X, top_local_y)
        low_dx, low_dy = rot(root_yaw, BLUE_RAMP_X, RAMP_LOW_Y)
        root_top = (top["x"] - top_dx, top["y"] - top_dy)
        root_low = (low["x"] - low_dx, low["y"] - low_dy)
        root_x = 0.5 * (root_top[0] + root_low[0])
        root_y = 0.5 * (root_top[1] + root_low[1])
        residual = math.hypot(root_top[0] - root_low[0], root_top[1] - root_low[1])
        max_residual = 0.45 if partial else 0.35
        if residual > max_residual:
            self._throttled_status(f"reject residual={residual:.3f}")
            return None
        return {
            "stamp": top["stamp"],
            "root_x": root_x,
            "root_y": root_y,
            "root_yaw": root_yaw,
            "low": low,
            "top": top,
            "z_gain": top_z - low_z,
            "xy_dist": float(hi_p - lo_p),
            "residual": residual,
            "ramp_yaw": ramp_yaw,
            "partial": partial,
            "progress": float(progress),
            "top_local_y": top_local_y,
            "ground_z_odom": low_z,
            "platform_z_odom": low_z + Z3_PLATFORM_Z_REL,
        }

    @staticmethod
    def _ramp_score(fit):
        return fit["z_gain"] - 2.0 * fit["residual"]

    def _cloud_cb(self, msg: PointCloud2):
        if self._cloud_refined and self._ground_z is not None and self._platform_z is not None:
            return
        try:
            x, y, z = self._parse_xyz(msg)
            ox, oy, oz = self._to_odom_xyz(msg.header, x, y, z)
        except Exception as e:
            self._throttled_cloud_warn(f"cloud skip: {e}")
            return
        if ox is None:
            return
        finite = np.isfinite(ox) & np.isfinite(oy) & np.isfinite(oz)
        if not finite.any():
            return
        ox = ox[finite]
        oy = oy[finite]
        oz = oz[finite]
        step = max(1, len(oz) // 3000)
        stamp = msg.header.stamp
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        self._cloud_buf.append({"t": t, "x": ox[::step], "y": oy[::step], "z": oz[::step]})
        while self._cloud_buf and t - self._cloud_buf[0]["t"] > self._cloud_cache_sec:
            self._cloud_buf.popleft()
        active_fit = self._active_cloud_fit()
        if active_fit is None:
            return

        if self._fit is None:
            cx, cy, cz = self._merged_cloud_cache(max_age=self._debug_cloud_window_sec, now=t)
        elif not self._cloud_refined:
            cx, cy, cz = self._merged_cloud_cache(max_age=self._refine_cloud_window_sec, now=t)
        else:
            cx, cy, cz = self._merged_cloud_cache()
        if len(cz) == 0:
            return
        lx, ly = self._odom_to_root_xy(cx, cy, active_fit)
        lz = cz
        ground_x, ground_y, ground_z_arr = cx, cy, cz
        if self._ramp_event_since is not None:
            gx, gy, gz = self._merged_cloud_interval(
                self._ramp_event_since - self._ground_backtrack_sec,
                self._ramp_event_since + self._ground_forward_sec,
            )
            if len(gz) > 0:
                ground_x, ground_y, ground_z_arr = gx, gy, gz
        ground_lx, ground_ly = self._odom_to_root_xy(ground_x, ground_y, active_fit)

        low_roi = (
            (ground_lx >= 1.10) & (ground_lx <= 3.40)
            & (ground_ly >= RAMP_LOW_Y + 0.12) & (ground_ly <= 2.40)
        )
        if int(low_roi.sum()) < self._ground_min_points:
            low_roi = (
                (ground_lx >= 1.00) & (ground_lx <= 3.45)
                & (ground_ly >= RAMP_LOW_Y - 0.05) & (ground_ly <= 2.50)
            )
        platform_roi = self._platform_surface_roi(lx, ly)

        ground_prior = self._ground_z
        low_z = None
        if int(low_roi.sum()) >= self._ground_roi_min_points:
            low_z = self._detect_z_peak(ground_z_arr[low_roi], lowest=True)
            if (
                low_z is not None
                and ground_prior is not None
                and abs(low_z - ground_prior) > self._ground_prior_max_error
            ):
                low_z = None
        platform_z = None
        base_low_z = low_z
        if base_low_z is None:
            base_low_z = self._ground_z
        if base_low_z is not None:
            pz = lz[platform_roi]
            pz = pz[(pz >= base_low_z + 0.25) & (pz <= base_low_z + 0.65)]
            platform_z = self._detect_z_peak(pz, lowest=False)
        if platform_z is None and int(platform_roi.sum()) >= self._ground_min_points:
            platform_z = self._detect_z_peak(lz[platform_roi], lowest=False)

        if self._fit is not None and low_z is not None:
            self._ground_pending.append(low_z)
            self._ground_z = self._stable_z(self._ground_pending)
        if self._fit is not None and platform_z is not None:
            self._platform_pending.append(platform_z)
            self._platform_z = self._stable_z(self._platform_pending)
        if self._fit is not None and self._ground_z is None and self._platform_z is not None:
            self._ground_z = self._platform_z - Z3_PLATFORM_Z_REL

        self._throttled_status(
            "zlock "
            f"low={low_z if low_z is not None else float('nan'):.3f} "
            f"platform={platform_z if platform_z is not None else float('nan'):.3f} "
            f"low_n={int(low_roi.sum())} platform_n={int(platform_roi.sum())}"
        )
        if self._fit is not None and self._ground_z is not None and self._platform_z is not None:
            dz = self._platform_z - self._ground_z
            effective_dz = Z3_PLATFORM_Z_REL if dz < 0.39 or dz > 0.52 else dz
            err = dz - Z3_PLATFORM_Z_REL
            self.get_logger().info(
                "[ZLOCK] cloud_slam ground "
                f"ground_z={self._ground_z:.4f} platform_z={self._platform_z:.4f} "
                f"raw_delta={dz:.4f} effective_delta={effective_dz:.4f} "
                f"expected={Z3_PLATFORM_Z_REL:.3f} err={err:+.4f}"
            )
            if not self._cloud_refined:
                self._refine_fit_from_cloud(cx, cy, cz)
                active_fit = self._fit
                lx, ly = self._odom_to_root_xy(cx, cy, active_fit)
        debug_x, debug_y, debug_z = cx, cy, cz
        if len(ground_z_arr) > 0 and ground_z_arr is not cz:
            debug_x = np.concatenate((cx, ground_x))
            debug_y = np.concatenate((cy, ground_y))
            debug_z = np.concatenate((cz, ground_z_arr))
        debug_lx, debug_ly = self._odom_to_root_xy(debug_x, debug_y, active_fit)
        debug_low_roi = (
            (debug_lx >= 1.10) & (debug_lx <= 3.40)
            & (debug_ly >= RAMP_LOW_Y + 0.12) & (debug_ly <= 2.40)
        )
        if int(debug_low_roi.sum()) < self._ground_min_points:
            debug_low_roi = (
                (debug_lx >= 1.00) & (debug_lx <= 3.45)
                & (debug_ly >= RAMP_LOW_Y - 0.05) & (debug_ly <= 2.50)
            )
        debug_platform_roi = self._platform_surface_roi(debug_lx, debug_ly)
        self._publish_debug_cloud(
            msg.header.stamp,
            active_fit,
            debug_x,
            debug_y,
            debug_z,
            debug_lx,
            debug_ly,
            debug_low_roi,
            debug_platform_roi,
            self._ground_z if self._ground_z is not None else active_fit.get("ground_z_odom", low_z),
            self._platform_z if self._platform_z is not None else active_fit.get("platform_z_odom", platform_z),
        )

    def _throttled_status(self, text: str):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_log_t > 1.0:
            self._last_log_t = now
            self.get_logger().info(f"[search] {text}")

    def _throttled_cloud_warn(self, text: str):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_cloud_warn_t > 1.0:
            self._last_cloud_warn_t = now
            self.get_logger().warn(text)

    def _publish_debug_cloud(self, stamp, fit, x, y, z, lx, ly, low_roi, platform_roi, low_z, platform_z):
        header = Header()
        header.stamp = stamp
        header.frame_id = "odom"
        masks, guides = self._debug_cloud_masks(z, lx, ly, low_roi, platform_roi, low_z, platform_z)
        keep = np.zeros(len(z), dtype=bool)
        for name in ("low_ground", "platform", "ramp", "top_cross", "bottom_cross"):
            keep |= masks[name]
        keep = self._thin_debug_cloud(lx, ly, z, keep)
        line_x, line_y, line_z, line_r, line_g, line_b = self._make_debug_line_points(fit, guides)
        if not keep.any() and len(line_x) == 0:
            self._debug_cloud_pub.publish(self._empty_cloud(header))
            return
        r = np.full(len(z), 120, dtype=np.uint8)
        g = np.full(len(z), 120, dtype=np.uint8)
        b = np.full(len(z), 120, dtype=np.uint8)
        self._paint(r, g, b, masks["low_ground"], (0, 255, 0))
        self._paint(r, g, b, masks["platform"], (0, 220, 255))
        self._paint(r, g, b, masks["ramp"], (255, 255, 255))
        self._paint(r, g, b, masks["top_cross"], (255, 40, 40))
        self._paint(r, g, b, masks["bottom_cross"], (255, 40, 40))
        out_x = x[keep]
        out_y = y[keep]
        out_z = z[keep]
        out_r = r[keep]
        out_g = g[keep]
        out_b = b[keep]
        if len(line_x) > 0:
            self._throttled_status(
                "debug_line "
                f"top=({guides['top_x0']:.2f},{guides['top_x1']:.2f})@{guides['top_y']:.2f} "
                f"bottom=({guides['bottom_x0']:.2f},{guides['bottom_x1']:.2f})@{guides['bottom_y']:.2f}"
            )
            out_x = np.concatenate((out_x, line_x))
            out_y = np.concatenate((out_y, line_y))
            out_z = np.concatenate((out_z, line_z))
            out_r = np.concatenate((out_r, line_r))
            out_g = np.concatenate((out_g, line_g))
            out_b = np.concatenate((out_b, line_b))
        self._debug_cloud_pub.publish(self._make_rgb_cloud(header, out_x, out_y, out_z, out_r, out_g, out_b))

    def _debug_cloud_masks(self, z, lx, ly, low_roi, platform_roi, low_z, platform_z):
        ground_z = self._ground_z if self._ground_z is not None else low_z
        high_z = self._platform_z if self._platform_z is not None else platform_z
        empty = np.zeros(len(z), dtype=bool)
        guides = {
            "top_y": float("nan"),
            "bottom_y": float("nan"),
            "left_x": float("nan"),
            "right_x": float("nan"),
            "top_x0": float("nan"),
            "top_x1": float("nan"),
            "bottom_x0": float("nan"),
            "bottom_x1": float("nan"),
            "ground_z": ground_z if ground_z is not None else float("nan"),
            "platform_z": high_z if high_z is not None else float("nan"),
        }
        if ground_z is None:
            return {
                "low_ground": low_roi,
                "platform": empty,
                "ramp": empty,
                "top_cross": empty,
                "bottom_cross": empty,
                "seam": empty,
                "right_side": empty,
            }, guides
        low_ground = low_roi & (np.abs(z - ground_z) <= 0.045) & (ly >= RAMP_LOW_Y + 0.08)
        platform = empty
        ramp = empty
        bottom_cross = empty
        seam = empty
        right_side = empty
        if high_z is not None:
            measured_delta = high_z - ground_z
            effective_platform_z = high_z
            if measured_delta < 0.39 or measured_delta > 0.52:
                effective_platform_z = ground_z + Z3_PLATFORM_Z_REL
            z_delta = effective_platform_z - ground_z
            if 0.38 <= z_delta <= 0.55:
                platform = platform_roi & (np.abs(z - effective_platform_z) <= 0.055)
                model_z = ground_z + (RAMP_LOW_Y - ly) / (RAMP_LOW_Y - RAMP_TOP_Y) * z_delta
                ramp_seed = (
                    (lx >= 1.50) & (lx <= 3.05)
                    & (ly >= RAMP_TOP_Y + RAMP_PLATFORM_GAP) & (ly <= RAMP_LOW_Y)
                    & (z >= ground_z + 0.03) & (z <= effective_platform_z - 0.025)
                    & (np.abs(z - model_z) <= 0.060)
                )
                seam = (
                    (lx >= 1.10) & (lx <= 1.90)
                    & (ly >= -1.55) & (ly <= INNER_PLATFORM_Y_MAX)
                    & (z >= ground_z + 0.20) & (z <= effective_platform_z + 0.18)
                )
                right_side = (
                    (lx >= 2.30) & (lx <= INNER_PLATFORM_X_MAX)
                    & (ly >= -1.45) & (ly <= INNER_PLATFORM_Y_MAX)
                    & (z >= ground_z + 0.34) & (z <= ground_z + 0.60)
                )
                top_cross = empty
                top_y = float("nan")
                bottom_y = float("nan")
                ramp_left_x = float("nan")
                ramp_right_x = float("nan")
                top_x0 = float("nan")
                top_x1 = float("nan")
                bottom_x0 = float("nan")
                bottom_x1 = float("nan")
                if int(ramp_seed.sum()) >= 80 and int(low_ground.sum()) >= 30:
                    bottom = self._fit_bottom_transition(
                        lx[ramp_seed], ly[ramp_seed], z[ramp_seed], lx, ly, z, ground_z
                    )
                    bottom_y = bottom["y_at_ref"]
                    if np.isfinite(bottom_y):
                        bottom_y = float(np.clip(bottom_y, RAMP_LOW_Y - 0.10, RAMP_LOW_Y + 0.10))
                if int(ramp_seed.sum()) >= 80 and int(platform.sum()) >= 40:
                    transition = self._fit_top_transition(
                        lx[ramp_seed], ly[ramp_seed], z[ramp_seed], lx, ly, z, effective_platform_z
                    )
                    top_y = transition["y_at_ref"]
                    if np.isfinite(top_y):
                        top_y = float(np.clip(top_y, RAMP_TOP_Y - 0.10, RAMP_TOP_Y + 0.10))
                if int(ramp_seed.sum()) >= 80:
                    side = self._estimate_debug_ramp_sides(lx[ramp_seed], ly[ramp_seed])
                    ramp_left_x = side["left_x"]
                    ramp_right_x = side["right_x"]
                    if np.isfinite(ramp_left_x) and np.isfinite(ramp_right_x):
                        ramp = (
                            ramp_seed
                            & (lx >= ramp_left_x - 0.04) & (lx <= ramp_right_x + 0.04)
                            & (ly >= (top_y if np.isfinite(top_y) else (RAMP_TOP_Y + RAMP_PLATFORM_GAP)) - 0.03)
                            & (ly <= (bottom_y if np.isfinite(bottom_y) else RAMP_LOW_Y) + 0.03)
                        )
                    else:
                        ramp = ramp_seed
                else:
                    ramp = ramp_seed
                if np.isfinite(bottom_y):
                    x0 = ramp_left_x - 0.05 if np.isfinite(ramp_left_x) else 1.20
                    x1 = ramp_right_x + 0.05 if np.isfinite(ramp_right_x) else 3.20
                    bottom_cross = (
                        (lx >= x0) & (lx <= x1)
                            & (np.abs(ly - bottom_y) <= 0.075)
                        & (
                            (np.abs(z - ground_z) <= 0.085)
                            | (
                                (z >= ground_z - 0.01) & (z <= ground_z + 0.18)
                                & ramp
                            )
                        )
                    )
                    bottom_band = (
                        (np.abs(ly - bottom_y) <= 0.11)
                        & (
                            low_ground
                            | (
                                ramp
                                & (z >= ground_z - 0.01) & (z <= ground_z + 0.18)
                            )
                        )
                    )
                    if int(bottom_band.sum()) >= 10:
                        x0 = float(np.percentile(lx[bottom_band], 4))
                        x1 = float(np.percentile(lx[bottom_band], 96))
                    bottom_x0 = x0
                    bottom_x1 = x1
                if np.isfinite(top_y):
                    x0 = ramp_left_x - 0.05 if np.isfinite(ramp_left_x) else 1.45
                    x1 = ramp_right_x + 0.05 if np.isfinite(ramp_right_x) else 3.10
                    top_cross = (
                        (lx >= x0) & (lx <= x1)
                        & (np.abs(ly - top_y) <= 0.075)
                        & (z >= effective_platform_z - 0.16) & (z <= effective_platform_z + 0.05)
                        & (ramp | platform)
                    )
                    top_band = (
                        (np.abs(ly - top_y) <= 0.11)
                        & (z >= effective_platform_z - 0.16) & (z <= effective_platform_z + 0.05)
                        & (ramp | platform)
                    )
                    if int(top_band.sum()) >= 10:
                        x0 = float(np.percentile(lx[top_band], 4))
                        x1 = float(np.percentile(lx[top_band], 96))
                    top_x0 = x0
                    top_x1 = x1
                guides = {
                    "top_y": top_y,
                    "bottom_y": bottom_y,
                    "left_x": ramp_left_x,
                    "right_x": ramp_right_x,
                    "top_x0": top_x0,
                    "top_x1": top_x1,
                    "bottom_x0": bottom_x0,
                    "bottom_x1": bottom_x1,
                    "ground_z": ground_z,
                    "platform_z": effective_platform_z,
                }
                seam &= ~(low_ground | platform | ramp)
                right_side &= ~(low_ground | platform | ramp | top_cross | bottom_cross | seam)
                top_cross &= ~(low_ground | bottom_cross | seam | right_side)
                bottom_cross &= ~(top_cross | seam | right_side)
            else:
                top_cross = empty
                bottom_cross = empty
        else:
            top_cross = empty
            bottom_cross = empty
        return {
            "low_ground": low_ground,
            "platform": platform,
            "ramp": ramp,
            "top_cross": top_cross,
            "bottom_cross": bottom_cross,
            "seam": seam,
            "right_side": right_side,
        }, guides

    def _make_debug_line_points(self, fit, guides):
        top_y = guides["top_y"]
        bottom_y = guides["bottom_y"]
        left_x = guides["left_x"]
        right_x = guides["right_x"]
        top_x0 = guides["top_x0"]
        top_x1 = guides["top_x1"]
        bottom_x0 = guides["bottom_x0"]
        bottom_x1 = guides["bottom_x1"]
        ground_z = guides["ground_z"]
        platform_z = guides["platform_z"]
        if (
            fit is None
            or not np.isfinite(left_x)
            or not np.isfinite(right_x)
        ):
            empty_f = np.empty(0, dtype=np.float32)
            empty_u = np.empty(0, dtype=np.uint8)
            return empty_f, empty_f, empty_f, empty_u, empty_u, empty_u
        line_local_x = []
        line_local_y = []
        line_z = []
        if np.isfinite(top_y) and np.isfinite(platform_z):
            tx0 = top_x0 if np.isfinite(top_x0) else (left_x - 0.02)
            tx1 = top_x1 if np.isfinite(top_x1) else (right_x + 0.02)
            xs = np.linspace(tx0, tx1, 96, dtype=np.float64)
            line_local_x.append(xs)
            line_local_y.append(np.full_like(xs, top_y))
            line_z.append(np.full_like(xs, platform_z - 0.015))
        if np.isfinite(bottom_y) and np.isfinite(ground_z):
            bx0 = bottom_x0 if np.isfinite(bottom_x0) else (left_x - 0.02)
            bx1 = bottom_x1 if np.isfinite(bottom_x1) else (right_x + 0.02)
            xs = np.linspace(bx0, bx1, 96, dtype=np.float64)
            line_local_x.append(xs)
            line_local_y.append(np.full_like(xs, bottom_y))
            line_z.append(np.full_like(xs, ground_z + 0.015))
        if not line_local_x:
            empty_f = np.empty(0, dtype=np.float32)
            empty_u = np.empty(0, dtype=np.uint8)
            return empty_f, empty_f, empty_f, empty_u, empty_u, empty_u
        local_x = np.concatenate(line_local_x)
        local_y = np.concatenate(line_local_y)
        z = np.concatenate(line_z)
        world_xy = np.array([rot(fit["root_yaw"], float(px), float(py)) for px, py in zip(local_x, local_y)])
        x = fit["root_x"] + world_xy[:, 0]
        y = fit["root_y"] + world_xy[:, 1]
        r = np.full(len(x), 255, dtype=np.uint8)
        g = np.full(len(x), 40, dtype=np.uint8)
        b = np.full(len(x), 40, dtype=np.uint8)
        return (
            x.astype(np.float32),
            y.astype(np.float32),
            z.astype(np.float32),
            r,
            g,
            b,
        )

    @staticmethod
    def _paint(r, g, b, mask, color):
        r[mask], g[mask], b[mask] = color

    @staticmethod
    def _thin_debug_cloud(lx, ly, z, keep):
        idx = np.flatnonzero(keep)
        if len(idx) <= 25000:
            return keep
        gx = np.floor(lx[idx] / 0.025).astype(np.int32)
        gy = np.floor(ly[idx] / 0.025).astype(np.int32)
        gz = np.floor(z[idx] / 0.025).astype(np.int32)
        _, first = np.unique(gx * 73856093 ^ gy * 19349663 ^ gz * 83492791, return_index=True)
        thinned = np.zeros_like(keep)
        thinned[idx[first]] = True
        return thinned

    @staticmethod
    def _make_rgb_cloud(header: Header, x, y, z, r, g, b) -> PointCloud2:
        n = len(x)
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

    @classmethod
    def _empty_cloud(cls, header: Header) -> PointCloud2:
        empty_f = np.empty(0, dtype=np.float32)
        empty_u = np.empty(0, dtype=np.uint8)
        return cls._make_rgb_cloud(header, empty_f, empty_f, empty_f, empty_u, empty_u, empty_u)

    def _publish(self):
        if not self._fit:
            return
        now_msg = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = "odom"
        t.child_frame_id = BLUE_ROOT_FRAME
        t.transform.translation.x = self._fit["root_x"]
        t.transform.translation.y = self._fit["root_y"]
        t.transform.translation.z = float(self._ground_z if self._ground_z is not None else 0.0)
        qx, qy, qz, qw = quat_from_yaw(self._fit["root_yaw"])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf.sendTransform(t)
        self._publish_markers(now_msg)

    def _refine_fit_from_cloud(self, x, y, z):
        if len(z) > 25000:
            step = max(1, len(z) // 25000)
            x = x[::step]
            y = y[::step]
            z = z[::step]
        base = self._fit
        best = self._cloud_refine_search(
            x, y, z, base["root_x"], base["root_y"], base["root_yaw"],
            yaw_range_deg=8.0, yaw_step_deg=2.0, x_range=0.80, y_range=0.35,
            xy_step=0.20, edge_weight=0.0, edge_required=False,
        )
        if best is not None:
            best = self._cloud_refine_search(
                x, y, z, best["root_x"], best["root_y"], best["root_yaw"],
                yaw_range_deg=2.0, yaw_step_deg=0.5, x_range=0.20, y_range=0.20,
                xy_step=0.05, edge_weight=0.0, edge_required=False,
            ) or best
            best = self._cloud_refine_search(
                x, y, z, best["root_x"], best["root_y"], best["root_yaw"],
                yaw_range_deg=0.6, yaw_step_deg=0.2, x_range=0.06, y_range=0.10,
                xy_step=0.02, edge_weight=0.0, edge_required=False,
            ) or best
        if best is None:
            self.get_logger().warn("[CLOUD_REFINE] rejected: no ramp boundary candidate")
            return
        ok = (
            best["inliers"] >= self._cloud_refine_min_inliers
            and best["coverage"] >= self._cloud_refine_min_coverage
            and best["rmse"] <= self._cloud_refine_max_rmse
        )
        if not ok:
            self._cloud_refined = True
            self.get_logger().warn(
                "[CLOUD_REFINE] rejected: "
                f"inliers={best['inliers']} coverage={best['coverage']:.2f} rmse={best['rmse']:.3f} "
                f"right_cells={best.get('right_cells', 0)} "
                f"right=({best.get('right_x_med', float('nan')):.2f},{best.get('right_y_med', float('nan')):.2f}) "
                f"edge_lx={best.get('edge_x_at_ref', float('nan')):.2f} "
                f"edge_lx_err={best.get('edge_x_error', float('nan')):+.2f} "
                f"edge_score={best.get('edge_score', 0.0):.2f} "
                f"right_score={best.get('right_score', 0.0):.2f} plane_dz={best.get('plane_dz', 0.0):.3f}"
                f" top_y={best.get('transition_y_at_ref', float('nan')):.2f}"
                f" top_y_err={best.get('transition_y_error', float('nan')):+.2f}"
                f" top_rmse={best.get('transition_rmse', float('nan')):.3f}"
                f" top_score={best.get('transition_score', 0.0):.2f}"
                f" ramp_lx=({best.get('ramp_left_x_at_ref', float('nan')):.2f},{best.get('ramp_right_x_at_ref', float('nan')):.2f})"
                f" ramp_lx_err=({best.get('ramp_left_x_error', float('nan')):+.2f},{best.get('ramp_right_x_error', float('nan')):+.2f})"
                f" ramp_w={best.get('ramp_width', float('nan')):.2f}"
                f" ramp_w_err={best.get('ramp_width_error', float('nan')):+.2f}"
                f" yaw_prior_err={math.degrees(best.get('yaw_prior_err', 0.0)):.2f}deg"
            )
            return
        old = self._fit
        seam_hint = 0.0
        fixed_root_x = best["root_x"]
        fixed_root_y = best["root_y"]
        self._fit = {
            **old,
            "root_x": fixed_root_x,
            "root_y": fixed_root_y,
            "root_yaw": best["root_yaw"],
            "ramp_yaw": best["root_yaw"] - math.pi / 2.0,
            "cloud_inliers": best["inliers"],
            "cloud_rmse": best["rmse"],
            "cloud_coverage": best["coverage"],
        }
        self._cloud_refined = True
        refine_dx = fixed_root_x - old["root_x"]
        refine_dy = fixed_root_y - old["root_y"]
        refine_dyaw = math.degrees(best["root_yaw"] - old["root_yaw"])
        self.get_logger().info(
            "[CLOUD_REFINE] blue_zone3_root "
            f"x={fixed_root_x:.3f} y={fixed_root_y:.3f} "
            f"yaw={math.degrees(best['root_yaw']):.2f}deg "
            f"delta=({refine_dx:+.3f},{refine_dy:+.3f},{refine_dyaw:+.2f}deg) "
            f"seam_hint={seam_hint:+.3f} "
            f"ramp_yaw={math.degrees(best['root_yaw'] - math.pi / 2.0):.2f}deg "
            f"inliers={best['inliers']} coverage={best['coverage']:.2f} rmse={best['rmse']:.3f} "
            f"seam_cells={best.get('seam_cells', 0)} "
            f"seam_lx={best.get('seam_x_at_ref', float('nan')):.2f} "
            f"seam_err={best.get('seam_x_error', float('nan')):+.2f} "
            f"seam_rmse={best.get('seam_rmse', float('nan')):.3f} "
            f"seam_score={best.get('seam_score', 0.0):.2f} "
            f"right_cells={best.get('right_cells', 0)} "
            f"right=({best.get('right_x_med', float('nan')):.2f},{best.get('right_y_med', float('nan')):.2f}) "
            f"edge_cells={best.get('edge_cells', 0)} "
            f"edge_lx={best.get('edge_x_at_ref', float('nan')):.2f} "
            f"edge_lx_err={best.get('edge_x_error', float('nan')):+.2f} "
            f"edge_rmse={best.get('edge_rmse', float('nan')):.3f} "
            f"edge_score={best.get('edge_score', 0.0):.2f} "
            f"right_score={best.get('right_score', 0.0):.2f} outside={best.get('outside_score', 0.0):.2f} "
            f"plane_dz={best.get('plane_dz', 0.0):.3f} "
            f"top_y={best.get('transition_y_at_ref', float('nan')):.2f} "
            f"top_y_err={best.get('transition_y_error', float('nan')):+.2f} "
            f"top_rmse={best.get('transition_rmse', float('nan')):.3f} "
            f"top_score={best.get('transition_score', 0.0):.2f} "
            f"ramp_lx=({best.get('ramp_left_x_at_ref', float('nan')):.2f},{best.get('ramp_right_x_at_ref', float('nan')):.2f}) "
            f"ramp_lx_err=({best.get('ramp_left_x_error', float('nan')):+.2f},{best.get('ramp_right_x_error', float('nan')):+.2f}) "
            f"ramp_w={best.get('ramp_width', float('nan')):.2f} "
            f"ramp_w_err={best.get('ramp_width_error', float('nan')):+.2f} "
            f"yaw_prior_err={math.degrees(best.get('yaw_prior_err', 0.0)):.2f}deg"
        )

    def _cloud_refine_search(self, x, y, z, cx, cy, cyaw, *, yaw_range_deg, yaw_step_deg, x_range, y_range, xy_step, edge_weight, edge_required):
        best = None
        for dyaw_deg in np.arange(-yaw_range_deg, yaw_range_deg + 1e-9, yaw_step_deg):
            yaw = cyaw + math.radians(float(dyaw_deg))
            for dx in np.arange(-x_range, x_range + 1e-9, xy_step):
                for dy in np.arange(-y_range, y_range + 1e-9, xy_step):
                    cand = {
                        "root_x": cx + float(dx),
                        "root_y": cy + float(dy),
                        "root_yaw": yaw,
                    }
                    score = self._cloud_boundary_score(x, y, z, cand, edge_weight, edge_required)
                    if score is None:
                        continue
                    if best is None or score["score"] > best["score"]:
                        best = {**cand, **score}
        return best

    def _cloud_boundary_score(self, x, y, z, fit, edge_weight, edge_required):
        yaw_prior_err = abs(angle_diff(fit["root_yaw"], BLUE_ROOT_YAW_PRIOR))
        if yaw_prior_err > CLOUD_REFINE_MAX_YAW_PRIOR_ERR:
            return None
        measured_delta = self._platform_z - self._ground_z
        effective_platform_z = self._platform_z
        if measured_delta < 0.39 or measured_delta > 0.52:
            effective_platform_z = self._ground_z + Z3_PLATFORM_Z_REL
        z_delta = effective_platform_z - self._ground_z
        if z_delta < 0.38 or z_delta > 0.55:
            return None
        dx = x - fit["root_x"]
        dy = y - fit["root_y"]
        c = math.cos(fit["root_yaw"])
        s = math.sin(fit["root_yaw"])
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        ramp = (
            (lx >= 1.50) & (lx <= 3.05)
            & (ly >= RAMP_TOP_Y + RAMP_PLATFORM_GAP) & (ly <= RAMP_LOW_Y)
            & (z >= self._ground_z + 0.03) & (z <= effective_platform_z - 0.025)
        )
        if int(ramp.sum()) < 80:
            return None
        rx = lx[ramp]
        ry = ly[ramp]
        rz = z[ramp]
        cx = np.clip(((rx - 1.50) / 0.05).astype(int), 0, 30)
        cy = np.clip(((ry - RAMP_TOP_Y) / 0.05).astype(int), 0, 29)
        cells = np.unique(cx * 64 + cy)
        plane_cells = int(cells.size)
        if plane_cells < 180:
            return None
        a = np.column_stack((rx, ry, np.ones_like(rx)))
        coeff, *_ = np.linalg.lstsq(a, rz, rcond=None)
        plane_z = a @ coeff
        err = rz - plane_z
        inlier = np.abs(err) <= 0.055
        n_inliers = int(inlier.sum())
        if n_inliers < 80:
            return None
        plane_rmse = float(np.sqrt(np.mean(err[inlier] ** 2)))
        plane_dx = float(coeff[0])
        plane_dy = float(coeff[1])
        plane_dz = -plane_dy * (RAMP_LOW_Y - RAMP_TOP_Y)
        if plane_rmse > 0.035 or abs(plane_dx) > 0.06 or plane_dy >= -0.22:
            return None
        if plane_dz < 0.40 or plane_dz > 0.52 or abs(plane_dz - z_delta) > 0.08:
            return None
        model_z = self._ground_z + (RAMP_LOW_Y - ry) / (RAMP_LOW_Y - RAMP_TOP_Y) * z_delta
        model_err = rz - model_z
        model_inlier = np.abs(model_err) <= 0.055
        if int(model_inlier.sum()) < 80:
            return None
        model_rmse = float(np.sqrt(np.mean(model_err[model_inlier] ** 2)))
        if model_rmse > 0.035:
            return None
        ramp_left = self._fit_ramp_side_edge(rx[model_inlier], ry[model_inlier], RAMP_LEFT_REF_X, percentile=5.0)
        ramp_right = self._fit_ramp_side_edge(
            rx[model_inlier], ry[model_inlier], RAMP_RIGHT_REF_X, percentile=95.0
        )
        ramp_left_score = ramp_left["score"]
        ramp_left_cells = ramp_left["cells"]
        ramp_left_rmse = ramp_left["rmse"]
        ramp_left_x_at_ref = ramp_left["x_at_ref"]
        ramp_left_x_error = ramp_left["x_error"]
        ramp_left_slope = ramp_left["slope"]
        ramp_right_score = ramp_right["score"]
        ramp_right_cells = ramp_right["cells"]
        ramp_right_rmse = ramp_right["rmse"]
        ramp_right_x_at_ref = ramp_right["x_at_ref"]
        ramp_right_x_error = ramp_right["x_error"]
        ramp_right_slope = ramp_right["slope"]
        if np.isfinite(ramp_left_x_at_ref) and np.isfinite(ramp_right_x_at_ref):
            ramp_width = ramp_right_x_at_ref - ramp_left_x_at_ref
            ramp_width_error = ramp_width - RAMP_WIDTH_REF
            ramp_width_score = math.exp(-abs(ramp_width_error) / 0.12)
        x_bins = np.unique(np.clip(((rx[model_inlier] - 1.50) / 0.20).astype(int), 0, 7)).size
        y_bins = np.unique(np.clip(((ry[model_inlier] - RAMP_TOP_Y) / 0.20).astype(int), 0, 7)).size
        if x_bins < 6 or y_bins < 3:
            return None
        right = (
            (lx >= 2.30) & (lx <= INNER_PLATFORM_X_MAX)
            & (ly >= -1.45) & (ly <= INNER_PLATFORM_Y_MAX)
            & (z >= self._ground_z + 0.34) & (z <= self._ground_z + 0.60)
        )
        right_cells = 0
        right_score = 0.0
        outside_score = 0.0
        right_y_med = float("nan")
        right_x_med = float("nan")
        transition_score = 0.0
        transition_cells = 0
        transition_rmse = float("nan")
        transition_y_at_ref = float("nan")
        transition_y_error = float("nan")
        transition_slope = float("nan")
        bottom_score = 0.0
        bottom_cells = 0
        bottom_rmse = float("nan")
        bottom_y_at_ref = float("nan")
        bottom_y_error = float("nan")
        bottom_slope = float("nan")
        edge_score = 0.0
        edge_cells = 0
        edge_rmse = float("nan")
        edge_x_at_ref = float("nan")
        edge_x_error = float("nan")
        edge_slope = float("nan")
        seam_score = 0.0
        seam_cells = 0
        seam_rmse = float("nan")
        seam_x_at_ref = float("nan")
        seam_x_error = float("nan")
        seam_slope = float("nan")
        seam = (
            (lx >= 1.10) & (lx <= 1.90)
            & (ly >= -1.55) & (ly <= INNER_PLATFORM_Y_MAX)
            & (z >= self._ground_z + 0.20) & (z <= effective_platform_z + 0.18)
        )
        if int(seam.sum()) >= 40:
            seam_edge = self._fit_vertical_edge_x(lx[seam], ly[seam], INNER_SEAM_REF_X, percentile=35.0)
            seam_score = seam_edge["score"]
            seam_cells = seam_edge["cells"]
            seam_rmse = seam_edge["rmse"]
            seam_x_at_ref = seam_edge["x_at_ref"]
            seam_x_error = seam_edge["x_error"]
            seam_slope = seam_edge["slope"]
        if int(right.sum()) >= 20:
            tx = lx[right]
            ty = ly[right]
            tz = z[right]
            right_x_med = float(np.median(tx))
            right_y_med = float(np.median(ty))
            tcx = np.clip(((tx - 2.30) / 0.05).astype(int), 0, 32)
            tcy = np.clip(((ty + 1.45) / 0.05).astype(int), 0, 24)
            right_cells = int(np.unique(tcx * 64 + tcy).size)
            if right_cells >= 45 and float(np.std(tz)) <= 0.09:
                x_span = float(np.percentile(tx, 95) - np.percentile(tx, 5))
                y_span = float(np.percentile(ty, 95) - np.percentile(ty, 5))
                if x_span >= 0.15 and y_span >= 0.40:
                    pos_score = math.exp(-abs(right_y_med + 0.825) / 0.35) * math.exp(-abs(right_x_med - 2.72) / 0.55)
                    right_score = min(1.0, right_cells / 100.0) * pos_score
                    outside_score = min(1.0, float(np.mean(tx > 3.05)) / 0.25)
                    if outside_score > 0.20:
                        right_score = 0.0
                    if edge_required and outside_score > 0.35:
                        return None
                    edge = self._fit_right_triangle_edge(tx, ty)
                    edge_score = edge["score"]
                    edge_cells = edge["cells"]
                    edge_rmse = edge["rmse"]
                    edge_x_at_ref = edge["x_at_ref"]
                    edge_x_error = edge["x_error"]
                    edge_slope = edge["slope"]
        transition = self._fit_top_transition(
            rx[model_inlier], ry[model_inlier], rz[model_inlier], lx, ly, z, effective_platform_z
        )
        bottom = self._fit_bottom_transition(
            rx[model_inlier], ry[model_inlier], rz[model_inlier],
            lx, ly, z, self._ground_z
        )
        transition_score = transition["score"]
        transition_cells = transition["cells"]
        transition_rmse = transition["rmse"]
        transition_y_at_ref = transition["y_at_ref"]
        transition_y_error = transition["y_error"]
        transition_slope = transition["slope"]
        bottom_score = bottom["score"]
        bottom_cells = bottom["cells"]
        bottom_rmse = bottom["rmse"]
        bottom_y_at_ref = bottom["y_at_ref"]
        bottom_y_error = bottom["y_error"]
        bottom_slope = bottom["slope"]
        seam_ok = (
            seam_cells >= 8
            and seam_score > 0.05
            and np.isfinite(seam_x_error)
            and abs(seam_x_error) <= 0.18
            and np.isfinite(seam_rmse)
            and seam_rmse <= 0.08
        )
        if edge_required and not seam_ok:
            return None
        if y_bins < 5:
            return None
        coverage = float((x_bins / 8.0) * (y_bins / 8.0))
        z_consistency = max(0.0, 1.0 - abs(plane_dz - z_delta) / 0.08)
        plane_score = max(0.0, 1.0 - model_rmse / 0.035) * min(1.0, plane_cells / 350.0)
        yaw_score = max(0.0, 1.0 - yaw_prior_err / CLOUD_REFINE_MAX_YAW_PRIOR_ERR)
        wall = (
            (lx >= 1.20) & (lx <= 3.60)
            & (ly >= -0.40) & (ly <= 1.50)
            & (z > effective_platform_z + 0.10)
        )
        wall_penalty = min(0.25, int(wall.sum()) / max(1, int(ramp.sum())) * 0.10)
        score = (
            0.34 * plane_score
            + 0.14 * coverage
            + 0.08 * z_consistency
            + 0.08 * yaw_score
            + 0.14 * ramp_left_score
            + 0.14 * ramp_right_score
            + 0.08 * ramp_width_score
            + 0.10 * transition_score
            + 0.10 * bottom_score
            + edge_weight * seam_score
            + edge_weight * edge_score
            - wall_penalty
        )
        return {
            "score": float(score),
            "inliers": plane_cells,
            "coverage": coverage,
            "rmse": model_rmse,
            "right_cells": right_cells,
            "right_x_med": right_x_med,
            "right_y_med": right_y_med,
            "right_score": right_score,
            "outside_score": outside_score,
            "transition_score": transition_score,
            "transition_cells": transition_cells,
            "transition_rmse": transition_rmse,
            "transition_y_at_ref": transition_y_at_ref,
            "transition_y_error": transition_y_error,
            "transition_slope": transition_slope,
            "bottom_score": bottom_score,
            "bottom_cells": bottom_cells,
            "bottom_rmse": bottom_rmse,
            "bottom_y_at_ref": bottom_y_at_ref,
            "bottom_y_error": bottom_y_error,
            "bottom_slope": bottom_slope,
            "ramp_left_score": ramp_left_score,
            "ramp_left_cells": ramp_left_cells,
            "ramp_left_rmse": ramp_left_rmse,
            "ramp_left_x_at_ref": ramp_left_x_at_ref,
            "ramp_left_x_error": ramp_left_x_error,
            "ramp_left_slope": ramp_left_slope,
            "ramp_right_score": ramp_right_score,
            "ramp_right_cells": ramp_right_cells,
            "ramp_right_rmse": ramp_right_rmse,
            "ramp_right_x_at_ref": ramp_right_x_at_ref,
            "ramp_right_x_error": ramp_right_x_error,
            "ramp_right_slope": ramp_right_slope,
            "ramp_width": ramp_width,
            "ramp_width_error": ramp_width_error,
            "ramp_width_score": ramp_width_score,
            "seam_score": seam_score,
            "seam_cells": seam_cells,
            "seam_rmse": seam_rmse,
            "seam_x_at_ref": seam_x_at_ref,
            "seam_x_error": seam_x_error,
            "seam_slope": seam_slope,
            "edge_score": edge_score,
            "edge_cells": edge_cells,
            "edge_rmse": edge_rmse,
            "edge_x_at_ref": edge_x_at_ref,
            "edge_x_error": edge_x_error,
            "edge_slope": edge_slope,
            "plane_dz": plane_dz,
            "yaw_prior_err": yaw_prior_err,
        }

    def _fit_right_triangle_edge(self, tx, ty):
        return self._fit_vertical_edge_x(tx, ty, RIGHT_EDGE_REF_X, percentile=75.0)

    def _estimate_debug_ramp_sides(self, tx, ty):
        if len(tx) < 30:
            return {"left_x": float("nan"), "right_x": float("nan")}
        core = (ty >= RAMP_TOP_Y + 0.18) & (ty <= RAMP_LOW_Y - 0.12)
        core_x = tx[core] if int(core.sum()) >= 20 else tx
        center_x = float(np.median(core_x))
        left = self._fit_ramp_side_edge(tx, ty, RAMP_LEFT_REF_X, percentile=7.0)
        right = self._fit_ramp_side_edge(tx, ty, RAMP_RIGHT_REF_X, percentile=93.0)
        left_x = left["x_at_ref"]
        right_x = right["x_at_ref"]
        if not np.isfinite(left_x):
            left_x = float(np.percentile(tx, 8))
        if not np.isfinite(right_x):
            right_x = float(np.percentile(tx, 92))
        if not np.isfinite(left_x) or not np.isfinite(right_x):
            return {"left_x": float("nan"), "right_x": float("nan")}
        width = right_x - left_x
        if width < 1.25 or width > 1.85:
            left_x = center_x - 0.5 * RAMP_WIDTH_REF
            right_x = center_x + 0.5 * RAMP_WIDTH_REF
        else:
            left_x = center_x - 0.5 * width
            right_x = center_x + 0.5 * width
        return {"left_x": left_x, "right_x": right_x}

    def _fit_ramp_side_edge(self, tx, ty, ref_x: float, *, percentile: float):
        if len(tx) < 15:
            return self._empty_right_edge()
        y_bin = np.floor((ty - (RAMP_TOP_Y + 0.05)) / 0.08).astype(int)
        edge_x = []
        edge_y = []
        for b in np.unique(y_bin):
            m = y_bin == b
            if int(m.sum()) < 2:
                continue
            bx = tx[m]
            by = ty[m]
            edge_x.append(float(np.percentile(bx, percentile)))
            edge_y.append(float(np.median(by)))
        if len(edge_x) < 3:
            return {
                **self._empty_right_edge(),
                "cells": int(len(edge_x)),
            }
        ex = np.asarray(edge_x, dtype=np.float64)
        ey = np.asarray(edge_y, dtype=np.float64)
        if float(np.max(ey) - np.min(ey)) < 0.25:
            return {
                **self._empty_right_edge(),
                "cells": int(len(edge_x)),
            }
        a = np.column_stack((ey, np.ones_like(ey)))
        coeff, *_ = np.linalg.lstsq(a, ex, rcond=None)
        pred = a @ coeff
        err = ex - pred
        keep = np.abs(err) <= 0.20
        if int(keep.sum()) >= 3 and int(keep.sum()) < len(ey):
            ex = ex[keep]
            ey = ey[keep]
            a = np.column_stack((ey, np.ones_like(ey)))
            coeff, *_ = np.linalg.lstsq(a, ex, rcond=None)
            pred = a @ coeff
            err = ex - pred
        rmse = float(np.sqrt(np.mean(err ** 2)))
        slope = float(coeff[0])
        x_at_ref = float(coeff[0] * 0.55 + coeff[1])
        x_error = x_at_ref - ref_x
        if rmse > 0.18 or abs(slope) > 0.35:
            return {
                **self._empty_right_edge(),
                "cells": int(len(ey)),
                "rmse": rmse,
                "x_at_ref": x_at_ref,
                "x_error": x_error,
                "slope": slope,
            }
        shape_score = min(1.0, len(ey) / 6.0) * max(0.0, 1.0 - rmse / 0.18)
        x_score = math.exp(-abs(x_error) / 0.22)
        slope_score = math.exp(-abs(slope) / 0.25)
        return {
            "score": float(shape_score * x_score * slope_score),
            "cells": int(len(ey)),
            "rmse": rmse,
            "x_at_ref": x_at_ref,
            "x_error": x_error,
            "slope": slope,
        }

    def _fit_top_transition(self, ramp_lx, ramp_ly, ramp_z, lx, ly, z, platform_z):
        ramp_profile = (
            (ramp_ly >= RAMP_TOP_Y - 0.05) & (ramp_ly <= RAMP_LOW_Y)
        )
        platform_profile = (
            (lx >= 1.45) & (lx <= 3.10)
            & (ly >= -0.90) & (ly <= RAMP_TOP_Y + 0.15)
            & (np.abs(z - platform_z) <= 0.05)
        )
        ramp_y, ramp_z_fit = self._compress_profile_axis(
            ramp_ly[ramp_profile], ramp_z[ramp_profile], bin_size=0.08
        )
        plat_y, plat_z = self._compress_profile_axis(ly[platform_profile], z[platform_profile], bin_size=0.10)
        if len(ramp_y) < 6 or len(plat_y) < 3:
            return self._empty_top_transition()
        ramp_coeff, ramp_rmse = self._fit_line_1d(ramp_y, ramp_z_fit, trim=0.045)
        plat_coeff, plat_rmse = self._fit_line_1d(plat_y, plat_z, trim=0.025)
        if ramp_coeff is None or plat_coeff is None:
            return self._empty_top_transition()
        ramp_slope = float(ramp_coeff[0])
        plat_slope = float(plat_coeff[0])
        denom = ramp_slope - plat_slope
        if abs(denom) < 0.05:
            return self._empty_top_transition()
        y_at_ref = float((plat_coeff[1] - ramp_coeff[1]) / denom)
        z_cross = float(ramp_slope * y_at_ref + ramp_coeff[1])
        y_error = y_at_ref - RAMP_TOP_Y
        z_error = z_cross - platform_z
        rmse = max(ramp_rmse, plat_rmse)
        slope = plat_slope
        if (
            ramp_rmse > 0.05
            or plat_rmse > 0.03
            or abs(ramp_slope) < 0.18
            or abs(ramp_slope) > 0.42
            or abs(plat_slope) > 0.05
        ):
            return {
                **self._empty_top_transition(),
                "cells": int(len(ramp_y) + len(plat_y)),
                "rmse": rmse,
                "y_at_ref": y_at_ref,
                "y_error": y_error,
                "slope": slope,
            }
        shape_score = min(1.0, len(ramp_y) / 10.0) * min(1.0, len(plat_y) / 4.0)
        fit_score = max(0.0, 1.0 - ramp_rmse / 0.05) * max(0.0, 1.0 - plat_rmse / 0.03)
        y_score = math.exp(-abs(y_error) / 0.10)
        z_score = math.exp(-abs(z_error) / 0.04)
        slope_score = math.exp(-abs(plat_slope) / 0.04)
        return {
            "score": float(shape_score * fit_score * y_score * z_score * slope_score),
            "cells": int(len(ramp_y) + len(plat_y)),
            "rmse": rmse,
            "y_at_ref": y_at_ref,
            "y_error": y_error,
            "slope": slope,
        }

    def _fit_bottom_transition(self, ramp_lx, ramp_ly, ramp_z, lx, ly, z, ground_z):
        ramp_profile = (
            (ramp_ly >= RAMP_TOP_Y) & (ramp_ly <= RAMP_LOW_Y + 0.18)
        )
        ground_profile = (
            (lx >= 1.20) & (lx <= 3.20)
            & (ly >= RAMP_LOW_Y - 0.10) & (ly <= RAMP_LOW_Y + 1.00)
            & (np.abs(z - ground_z) <= 0.05)
        )
        ramp_y, ramp_z_fit = self._compress_profile_axis(
            ramp_ly[ramp_profile], ramp_z[ramp_profile], bin_size=0.08
        )
        ground_y, ground_z_fit = self._compress_profile_axis(
            ly[ground_profile], z[ground_profile], bin_size=0.10
        )
        if len(ramp_y) < 5 or len(ground_y) < 3:
            return self._empty_top_transition()
        ramp_coeff, ramp_rmse = self._fit_line_1d(ramp_y, ramp_z_fit, trim=0.045)
        ground_coeff, ground_rmse = self._fit_line_1d(ground_y, ground_z_fit, trim=0.025)
        if ramp_coeff is None or ground_coeff is None:
            return self._empty_top_transition()
        ramp_slope = float(ramp_coeff[0])
        ground_slope = float(ground_coeff[0])
        denom = ramp_slope - ground_slope
        if abs(denom) < 0.05:
            return self._empty_top_transition()
        y_at_ref = float((ground_coeff[1] - ramp_coeff[1]) / denom)
        z_cross = float(ramp_slope * y_at_ref + ramp_coeff[1])
        y_error = y_at_ref - RAMP_LOW_Y
        z_error = z_cross - ground_z
        rmse = max(ramp_rmse, ground_rmse)
        slope = ground_slope
        if (
            ramp_rmse > 0.05
            or ground_rmse > 0.03
            or abs(ramp_slope) < 0.18
            or abs(ramp_slope) > 0.42
            or abs(ground_slope) > 0.05
        ):
            return {
                **self._empty_top_transition(),
                "cells": int(len(ramp_y) + len(ground_y)),
                "rmse": rmse,
                "y_at_ref": y_at_ref,
                "y_error": y_error,
                "slope": slope,
            }
        shape_score = min(1.0, len(ramp_y) / 8.0) * min(1.0, len(ground_y) / 4.0)
        fit_score = max(0.0, 1.0 - ramp_rmse / 0.05) * max(0.0, 1.0 - ground_rmse / 0.03)
        y_score = math.exp(-abs(y_error) / 0.10)
        z_score = math.exp(-abs(z_error) / 0.04)
        slope_score = math.exp(-abs(ground_slope) / 0.04)
        return {
            "score": float(shape_score * fit_score * y_score * z_score * slope_score),
            "cells": int(len(ramp_y) + len(ground_y)),
            "rmse": rmse,
            "y_at_ref": y_at_ref,
            "y_error": y_error,
            "slope": slope,
        }

    def _fit_vertical_edge_x(self, tx, ty, ref_x: float, *, percentile: float):
        if len(tx) < 30:
            return self._empty_right_edge()
        y_bin = np.floor((ty + 1.55) / 0.08).astype(int)
        edge_x = []
        edge_y = []
        for b in np.unique(y_bin):
            m = y_bin == b
            if int(m.sum()) < 4:
                continue
            bx = tx[m]
            by = ty[m]
            edge_x.append(float(np.percentile(bx, percentile)))
            edge_y.append(float(np.median(by)))
        if len(edge_x) < 5:
            return self._empty_right_edge()
        ex = np.asarray(edge_x, dtype=np.float64)
        ey = np.asarray(edge_y, dtype=np.float64)
        if float(np.max(ey) - np.min(ey)) < 0.35:
            return self._empty_right_edge()
        a = np.column_stack((ey, np.ones_like(ey)))
        coeff, *_ = np.linalg.lstsq(a, ex, rcond=None)
        pred = a @ coeff
        err = ex - pred
        keep = np.abs(err) <= 0.18
        if int(keep.sum()) >= 5 and int(keep.sum()) < len(ey):
            ex = ex[keep]
            ey = ey[keep]
            a = np.column_stack((ey, np.ones_like(ey)))
            coeff, *_ = np.linalg.lstsq(a, ex, rcond=None)
            pred = a @ coeff
            err = ex - pred
        rmse = float(np.sqrt(np.mean(err ** 2)))
        slope = float(coeff[0])
        x_at_ref = float(slope * RIGHT_EDGE_REF_Y + coeff[1])
        x_error = x_at_ref - ref_x
        if rmse > 0.10 or abs(slope) > 0.45:
            return {
                **self._empty_right_edge(),
                "cells": int(len(ey)),
                "rmse": rmse,
                "x_at_ref": x_at_ref,
                "x_error": x_error,
                "slope": slope,
            }
        shape_score = min(1.0, len(ey) / 9.0) * max(0.0, 1.0 - rmse / 0.10)
        x_score = math.exp(-abs(x_error) / 0.18)
        slope_score = math.exp(-abs(slope) / 0.35)
        return {
            "score": float(shape_score * x_score * slope_score),
            "cells": int(len(ey)),
            "rmse": rmse,
            "x_at_ref": x_at_ref,
            "x_error": x_error,
            "slope": slope,
        }

    @staticmethod
    def _compress_profile_axis(axis, values, *, bin_size: float):
        if len(axis) == 0:
            return np.empty(0, dtype=np.float64), np.empty(0, dtype=np.float64)
        bins = np.floor(axis / bin_size).astype(int)
        out_axis = []
        out_values = []
        for b in np.unique(bins):
            m = bins == b
            if int(m.sum()) < 4:
                continue
            out_axis.append(float(np.median(axis[m])))
            out_values.append(float(np.median(values[m])))
        return np.asarray(out_axis, dtype=np.float64), np.asarray(out_values, dtype=np.float64)

    @staticmethod
    def _fit_line_1d(x, y, *, trim: float):
        if len(x) < 3:
            return None, float("nan")
        a = np.column_stack((x, np.ones_like(x)))
        coeff, *_ = np.linalg.lstsq(a, y, rcond=None)
        pred = a @ coeff
        err = y - pred
        keep = np.abs(err) <= trim
        if int(keep.sum()) >= 3 and int(keep.sum()) < len(x):
            x = x[keep]
            y = y[keep]
            a = np.column_stack((x, np.ones_like(x)))
            coeff, *_ = np.linalg.lstsq(a, y, rcond=None)
            pred = a @ coeff
            err = y - pred
        rmse = float(np.sqrt(np.mean(err ** 2)))
        return coeff, rmse

    @staticmethod
    def _empty_right_edge():
        return {
            "score": 0.0,
            "cells": 0,
            "rmse": float("nan"),
            "x_at_ref": float("nan"),
            "x_error": float("nan"),
            "slope": float("nan"),
        }

    @staticmethod
    def _empty_top_transition():
        return {
            "score": 0.0,
            "cells": 0,
            "rmse": float("nan"),
            "y_at_ref": float("nan"),
            "y_error": float("nan"),
            "slope": float("nan"),
        }

    def _publish_markers(self, stamp):
        fit = self._fit
        arr = MarkerArray()
        root_x = fit["root_x"]
        root_y = fit["root_y"]
        yaw = fit["root_yaw"]

        def point(local_x, local_y, z=0.0):
            x, y = rot(yaw, local_x, local_y)
            p = Marker().pose.position
            p.x = root_x + x
            p.y = root_y + y
            p.z = z
            return p

        line = Marker()
        line.header.frame_id = "odom"
        line.header.stamp = stamp
        line.ns = "blue_z3_live_fit"
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.035
        line.color.r = 0.1
        line.color.g = 0.3
        line.color.b = 1.0
        line.color.a = 1.0
        corners = [(1.50, 1.30), (3.05, 1.30), (3.05, -0.20), (1.50, -0.20), (1.50, 1.30)]
        line.points = [point(x, y, 0.08) for x, y in corners]
        arr.markers.append(line)

        side = Marker()
        side.header.frame_id = "odom"
        side.header.stamp = stamp
        side.ns = "blue_z3_live_fit"
        side.id = 5
        side.type = Marker.LINE_STRIP
        side.action = Marker.ADD
        side.scale.x = 0.03
        side.color.r = 0.0
        side.color.g = 0.9
        side.color.b = 1.0
        side.color.a = 1.0
        side_corners = [(1.50, -0.20), (3.05, -0.20), (3.05, -1.45), (1.50, -1.45), (1.50, -0.20)]
        side.points = [point(x, y, 0.45) for x, y in side_corners]
        arr.markers.append(side)

        path = Marker()
        path.header.frame_id = "odom"
        path.header.stamp = stamp
        path.ns = "blue_z3_live_fit"
        path.id = 2
        path.type = Marker.LINE_STRIP
        path.action = Marker.ADD
        path.scale.x = 0.025
        path.color.r = 0.0
        path.color.g = 0.8
        path.color.b = 0.2
        path.color.a = 1.0
        for item in (fit["low"], fit["top"]):
            p = Marker().pose.position
            p.x = item["x"]
            p.y = item["y"]
            p.z = item["z"]
            path.points.append(p)
        arr.markers.append(path)

        if self._ground_z is not None:
            ground = Marker()
            ground.header.frame_id = "odom"
            ground.header.stamp = stamp
            ground.ns = "blue_z3_live_fit"
            ground.id = 3
            ground.type = Marker.CUBE
            ground.action = Marker.ADD
            gx, gy = point(2.275, 1.90, self._ground_z).x, point(2.275, 1.90, self._ground_z).y
            ground.pose.position.x = gx
            ground.pose.position.y = gy
            ground.pose.position.z = self._ground_z
            qx, qy, qz, qw = quat_from_yaw(yaw)
            ground.pose.orientation.x = qx
            ground.pose.orientation.y = qy
            ground.pose.orientation.z = qz
            ground.pose.orientation.w = qw
            ground.scale.x = 1.45
            ground.scale.y = 1.10
            ground.scale.z = 0.015
            ground.color.r = 1.0
            ground.color.g = 1.0
            ground.color.b = 1.0
            ground.color.a = 0.45
            arr.markers.append(ground)
        if self._platform_z is not None:
            platform = Marker()
            platform.header.frame_id = "odom"
            platform.header.stamp = stamp
            platform.ns = "blue_z3_live_fit"
            platform.id = 4
            platform.type = Marker.CUBE
            platform.action = Marker.ADD
            px, py = point(2.275, -0.85, self._platform_z).x, point(2.275, -0.85, self._platform_z).y
            platform.pose.position.x = px
            platform.pose.position.y = py
            platform.pose.position.z = self._platform_z
            qx, qy, qz, qw = quat_from_yaw(yaw)
            platform.pose.orientation.x = qx
            platform.pose.orientation.y = qy
            platform.pose.orientation.z = qz
            platform.pose.orientation.w = qw
            platform.scale.x = 1.45
            platform.scale.y = 1.00
            platform.scale.z = 0.015
            platform.color.r = 0.2
            platform.color.g = 0.8
            platform.color.b = 1.0
            platform.color.a = 0.45
            arr.markers.append(platform)
        self._marker_pub.publish(arr)

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
            self._throttled_cloud_warn(f"TF {src}->odom unavailable: {e}")
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

    def _active_cloud_fit(self):
        return self._fit or self._candidate_fit or self._provisional_fit

    def _odom_to_root_xy(self, x, y, fit=None):
        fit = fit or self._fit
        dx = x - fit["root_x"]
        dy = y - fit["root_y"]
        c = math.cos(fit["root_yaw"])
        s = math.sin(fit["root_yaw"])
        return c * dx + s * dy, -s * dx + c * dy

    def _platform_surface_roi(self, lx, ly):
        top_y = RAMP_TOP_Y
        if self._fit is not None:
            top_y = float(self._fit.get("top_local_y", RAMP_TOP_Y))
        x_min = 1.35
        x_max = 3.20
        y_min = top_y - 0.70
        y_max = top_y - 0.02
        roi = (lx >= x_min) & (lx <= x_max) & (ly >= y_min) & (ly <= y_max)
        if int(roi.sum()) < self._ground_min_points:
            roi = (lx >= 1.20) & (lx <= 3.25) & (ly >= top_y - 0.95) & (ly <= top_y + 0.02)
        return roi

    def _merged_cloud_cache(self, *, max_age=None, now=None):
        if not self._cloud_buf:
            empty = np.array([], dtype=np.float64)
            return empty, empty, empty
        xs, ys, zs = [], [], []
        total = 0
        for item in reversed(self._cloud_buf):
            if max_age is not None and now is not None and now - item["t"] > max_age:
                break
            xs.append(item["x"])
            ys.append(item["y"])
            zs.append(item["z"])
            total += len(item["z"])
            if total >= self._cloud_cache_max_points:
                break
        return np.concatenate(xs), np.concatenate(ys), np.concatenate(zs)

    def _merged_cloud_interval(self, start_t: float, end_t: float):
        if not self._cloud_buf or end_t < start_t:
            empty = np.array([], dtype=np.float64)
            return empty, empty, empty
        xs, ys, zs = [], [], []
        total = 0
        for item in self._cloud_buf:
            if item["t"] < start_t or item["t"] > end_t:
                continue
            xs.append(item["x"])
            ys.append(item["y"])
            zs.append(item["z"])
            total += len(item["z"])
            if total >= self._cloud_cache_max_points:
                break
        if not xs:
            empty = np.array([], dtype=np.float64)
            return empty, empty, empty
        return np.concatenate(xs), np.concatenate(ys), np.concatenate(zs)

    def _detect_z_peak(self, z, *, lowest: bool):
        z = z[np.isfinite(z)]
        if len(z) < self._ground_min_points:
            return None
        lo = np.floor(float(z.min()) / HISTOGRAM_BIN_WIDTH) * HISTOGRAM_BIN_WIDTH
        hi = np.ceil(float(z.max()) / HISTOGRAM_BIN_WIDTH) * HISTOGRAM_BIN_WIDTH
        if hi <= lo:
            return float(np.median(z))
        edges = np.arange(lo, hi + HISTOGRAM_BIN_WIDTH, HISTOGRAM_BIN_WIDTH)
        hist, edges = np.histogram(z, bins=edges)
        if hist.size == 0 or hist.max() <= 0:
            return None
        if lowest:
            threshold = max(10, int(hist.max() * 0.15))
            idx = next((i for i, count in enumerate(hist) if count >= threshold), int(np.argmax(hist)))
        else:
            idx = int(np.argmax(hist))
        in_bin = (z >= edges[idx]) & (z < edges[idx + 1])
        if lowest and int(in_bin.sum()) > 0:
            return float(np.median(z[in_bin]))
        if int(in_bin.sum()) < self._ground_min_points:
            return None
        return float(np.median(z[in_bin]))

    @staticmethod
    def _stable_z(values: deque[float]):
        if not values:
            return None
        if len(values) < values.maxlen:
            return None
        arr = np.asarray(values, dtype=np.float64)
        if float(arr.max() - arr.min()) > 0.05:
            return None
        return float(np.median(arr))


def main():
    rclpy.init()
    node = BlueZ3LiveRampFit()
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
