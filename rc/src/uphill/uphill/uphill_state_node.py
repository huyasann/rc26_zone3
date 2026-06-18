from __future__ import annotations

import json
import math
import copy
import select
import sys
import termios
import threading
import tty
from collections import deque
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


STATE_FLAT = "flat"
STATE_FLAT_TO_UPHILL = "flat_to_uphill"
STATE_UPHILL = "uphill"
STATE_UPHILL_TO_PLATFORM = "uphill_to_platform"
STATE_PLATFORM = "platform"


@dataclass
class Baseline:
    z: float
    pitch: float
    yaw: float


@dataclass
class OdomPoseSample:
    rel_s: float
    pose: PoseStamped
    pitch_abs_ema: float
    z_rise_ema: float
    ground_z: float | None


def quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = 2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def circular_mean(vals: list[float]) -> float:
    return math.atan2(sum(math.sin(v) for v in vals), sum(math.cos(v) for v in vals))


class UphillStateNode(Node):
    def __init__(self) -> None:
        super().__init__("uphill_state_node")

        self.odom_topic = self.declare_parameter("odom_topic", "/odin1/odometry_highfreq").value
        self.cloud_topic = self.declare_parameter("cloud_topic", "/odin1/cloud_slam").value
        self.height_frame = self.declare_parameter("height_frame", "odom").value
        self.state_topic = self.declare_parameter("state_topic", "/uphill/state").value
        self.debug_topic = self.declare_parameter("debug_topic", "/uphill/debug").value

        self.baseline_frames = int(self.declare_parameter("baseline_frames", 20).value)
        self.velocity_window_s = float(self.declare_parameter("velocity_window_s", 0.40).value)
        self.vx_gate_mps = float(self.declare_parameter("vx_gate_mps", 0.08).value)
        self.pitch_start_deg = float(self.declare_parameter("pitch_start_deg", 0.60).value)
        self.pitch_contact_deg = float(self.declare_parameter("pitch_contact_deg", 7.47).value)
        self.pitch_stable_deg = float(self.declare_parameter("pitch_stable_deg", 14.00).value)
        self.pitch_platform_deg = float(self.declare_parameter("pitch_platform_deg", 9.00).value)
        self.z_start_m = float(self.declare_parameter("z_start_m", 0.015).value)
        self.z_contact_m = float(self.declare_parameter("z_contact_m", 0.06).value)
        self.z_high_m = float(self.declare_parameter("z_high_m", 0.36).value)
        self.hold_s = float(self.declare_parameter("hold_s", 0.15).value)
        self.start_hold_s = float(self.declare_parameter("start_hold_s", 0.04).value)
        self.ema_alpha = float(self.declare_parameter("ema_alpha", 0.35).value)
        self.growth_check_window_s = float(self.declare_parameter("growth_check_window_s", 0.50).value)
        self.min_growth_rate_deg_s = float(self.declare_parameter("min_growth_rate_deg_s", 1.20).value)
        self.min_total_growth_deg = float(self.declare_parameter("min_total_growth_deg", 4.00).value)
        self.candidate_timeout_s = float(self.declare_parameter("candidate_timeout_s", 5.50).value)
        self.candidate_drop_reject_deg = float(self.declare_parameter("candidate_drop_reject_deg", 2.00).value)
        self.publish_trajectory = bool(self.declare_parameter("publish_trajectory", False).value)
        self.endpoint_marker_size = float(self.declare_parameter("endpoint_marker_size", 0.06).value)
        self.endpoint_history_s = float(self.declare_parameter("endpoint_history_s", 4.00).value)
        self.endpoint_ground_offset_m = float(self.declare_parameter("endpoint_ground_offset_m", 0.05).value)
        self.ground_histogram_bin_width = float(self.declare_parameter("ground_histogram_bin_width", 0.02).value)
        self.ground_peak_ratio = float(self.declare_parameter("ground_peak_ratio", 0.15).value)
        self.ground_offset_alpha = float(self.declare_parameter("ground_offset_alpha", 0.20).value)
        self.enable_endpoint_ground_detection = bool(
            self.declare_parameter("enable_endpoint_ground_detection", False).value
        )

        self.baseline_samples: list[tuple[float, float]] = []
        self.baseline: Baseline | None = None
        self.state = STATE_FLAT
        self.state_since_s: float | None = None
        self.t0_s: float | None = None
        self.history: deque[tuple[float, float, float]] = deque()
        self.pitch_abs_ema: float | None = None
        self.z_rise_ema: float | None = None
        self.pitch_candidate_history: deque[tuple[float, float]] = deque()
        self.candidate_start_pitch: float | None = None
        self.candidate_peak_pitch: float = 0.0
        self.candidate_since: dict[str, float | None] = {}
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_z = 0.0
        self.last_odom_yaw = 0.0
        self.stop_keyboard = False
        self.shutdown_requested = False
        self.record_trajectory = False
        self.trajectory_points: list[Point] = []
        self.trajectory_poses: list[PoseStamped] = []
        self.transition_marker_id = 100
        self.pending_uphill_start_pose: PoseStamped | None = None
        self.pending_uphill_start_z_rise: float = 0.0
        self.pending_uphill_end_sample: OdomPoseSample | None = None
        self.pose_history: deque[OdomPoseSample] = deque()
        self.endpoint_markers: dict[int, Marker] = {}
        self.cloud_ground_z: float | None = None
        self.cloud_ground_peak_pct = 0.0
        self.measured_ground_offset_m: float | None = None
        self.last_tf_warn_s = 0.0

        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)
        self.transition_pose_pub = self.create_publisher(PoseStamped, "/uphill/transition_pose", 10)
        self.path_pub = self.create_publisher(Path, "/uphill/trajectory_path", 10)
        self.traj_marker_pub = self.create_publisher(Marker, "/uphill/trajectory_marker", 10)
        self.transition_marker_pub = self.create_publisher(Marker, "/uphill/transition_marker", 10)
        endpoint_qos = QoSProfile(depth=10)
        endpoint_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.endpoint_marker_pub = self.create_publisher(Marker, "/uphill/endpoint_marker", endpoint_qos)
        self.create_timer(0.50, self._republish_endpoint_markers)
        self.tf_buffer = Buffer() if self.enable_endpoint_ground_detection else None
        self.tf_listener = TransformListener(self.tf_buffer, self) if self.tf_buffer is not None else None
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        if self.enable_endpoint_ground_detection:
            self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, 10)
        self.keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self.keyboard_thread.start()

        self.get_logger().info(f"subscribe: {self.odom_topic}")
        if self.enable_endpoint_ground_detection:
            self.get_logger().info(f"ground detect cloud: {self.cloud_topic} -> {self.height_frame}")
        self.get_logger().info(f"publish: {self.state_topic}, {self.debug_topic}")
        self.get_logger().info("press R to reset state machine and re-collect baseline")

    def on_odom(self, msg: Odometry) -> None:
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp_s <= 0.0:
            stamp_s = self.get_clock().now().nanoseconds * 1e-9
        if self.t0_s is None:
            self.t0_s = stamp_s
        rel_s = stamp_s - self.t0_s

        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        _, pitch, yaw = quat_to_rpy(quat.x, quat.y, quat.z, quat.w)

        self.last_odom_x = float(pos.x)
        self.last_odom_y = float(pos.y)
        self.last_odom_z = float(pos.z)
        self.last_odom_yaw = yaw
        if self.enable_endpoint_ground_detection:
            self._update_ground_offset_from_cloud()

        if self.baseline is None:
            self.baseline_samples.append((float(pos.z), pitch))
            if len(self.baseline_samples) >= self.baseline_frames:
                z0 = sum(v[0] for v in self.baseline_samples) / len(self.baseline_samples)
                pitch0 = sum(v[1] for v in self.baseline_samples) / len(self.baseline_samples)
                yaw0 = circular_mean([yaw])
                self.baseline = Baseline(z=z0, pitch=pitch0, yaw=yaw0)
                self.state_since_s = rel_s
                self.get_logger().info(
                    f"baseline done: z={z0:.3f}, pitch={math.degrees(pitch0):.2f}deg"
                )
            return

        pitch_delta_deg = math.degrees(angle_diff(pitch, self.baseline.pitch))
        pitch_abs = abs(pitch_delta_deg)
        z_rise = float(pos.z) - self.baseline.z

        self.pitch_abs_ema = self._ema(self.pitch_abs_ema, pitch_abs)
        self.z_rise_ema = self._ema(self.z_rise_ema, z_rise)

        current_pose = PoseStamped()
        current_pose.header.stamp = msg.header.stamp
        current_pose.header.frame_id = "odom"
        current_pose.pose.position.x = self.last_odom_x
        current_pose.pose.position.y = self.last_odom_y
        current_pose.pose.position.z = self.last_odom_z
        current_pose.pose.orientation = msg.pose.pose.orientation
        self._record_pose_history(
            rel_s,
            current_pose,
            float(self.pitch_abs_ema or 0.0),
            float(self.z_rise_ema or 0.0),
            self.cloud_ground_z,
        )
        current_sample = self.pose_history[-1]

        self.history.append((rel_s, float(pos.x), float(pos.y)))
        while self.history and rel_s - self.history[0][0] > self.velocity_window_s:
            self.history.popleft()
        vx = self._vx()

        moving_forward = vx > self.vx_gate_mps
        pitch_start = self.pitch_abs_ema >= self.pitch_start_deg
        pitch_contact = self.pitch_abs_ema >= self.pitch_contact_deg
        pitch_stable = self.pitch_abs_ema >= self.pitch_stable_deg
        pitch_platform = self.pitch_abs_ema <= self.pitch_platform_deg
        z_start = self.z_rise_ema >= self.z_start_m
        z_contact = self.z_rise_ema >= self.z_contact_m
        z_high = self.z_rise_ema >= self.z_high_m
        growth_ok = self._candidate_growth_ok(rel_s, pitch_abs)
        candidate_age = 0.0
        if self.state == STATE_FLAT_TO_UPHILL and self.state_since_s is not None:
            candidate_age = rel_s - self.state_since_s
        candidate_drop = self.candidate_peak_pitch - pitch_abs

        old_state = self.state
        if self.state == STATE_UPHILL and pitch_stable:
            if (
                self.pending_uphill_end_sample is None
                or current_sample.pitch_abs_ema >= self.pending_uphill_end_sample.pitch_abs_ema
            ):
                self.pending_uphill_end_sample = copy.deepcopy(current_sample)

        if self.state == STATE_FLAT:
            if self._held_for("flat_to_uphill", rel_s, moving_forward and pitch_start, self.start_hold_s):
                self._set_state(STATE_FLAT_TO_UPHILL, rel_s)

        elif self.state == STATE_FLAT_TO_UPHILL:
            if self._held("to_uphill", rel_s, moving_forward and pitch_stable and growth_ok):
                self._set_state(STATE_UPHILL, rel_s)
            elif self._held(
                "back_flat",
                rel_s,
                (not pitch_start)
                or (candidate_age >= self.candidate_timeout_s and not pitch_stable)
                or (candidate_age >= self.growth_check_window_s and not growth_ok and pitch_abs < self.pitch_contact_deg)
                or (candidate_drop >= self.candidate_drop_reject_deg and not pitch_stable),
            ):
                self._set_state(STATE_FLAT, rel_s)

        elif self.state == STATE_UPHILL:
            if self._held("to_platform_transition", rel_s, z_high and moving_forward and not pitch_stable):
                self._set_state(STATE_UPHILL_TO_PLATFORM, rel_s)

        elif self.state == STATE_UPHILL_TO_PLATFORM:
            if self._held("to_platform", rel_s, z_high and pitch_platform):
                self._set_state(STATE_PLATFORM, rel_s)
            elif self._held("back_uphill", rel_s, moving_forward and pitch_stable):
                self._set_state(STATE_UPHILL, rel_s)

        elif self.state == STATE_PLATFORM:
            if self._held("reset_flat", rel_s, not z_contact and not pitch_contact):
                self._set_state(STATE_FLAT, rel_s)

        if self.state != old_state:
            transition_text = (
                f"transition: {old_state} -> {self.state}, t={rel_s:.2f}s, "
                f"pitch={self.pitch_abs_ema:.2f}deg, z={self.z_rise_ema:.3f}m, "
                f"vx={vx:.3f}m/s, growth_ok={growth_ok}"
            )
            if old_state == STATE_FLAT and self.state == STATE_FLAT_TO_UPHILL:
                self.get_logger().debug(transition_text)
            elif old_state == STATE_FLAT_TO_UPHILL and self.state == STATE_FLAT:
                self.get_logger().debug(transition_text)
            else:
                self.get_logger().info(transition_text)
            # 发布状态切换时的 odom 位姿
            transition_pose = current_pose
            if self.state == STATE_UPHILL and self.pending_uphill_start_pose is not None:
                transition_pose = self.pending_uphill_start_pose
            self.transition_pose_pub.publish(transition_pose)
            self._record_transition_pose(current_pose, self.state)

        self._record_trajectory_point(msg)
        self._publish(rel_s, pitch_delta_deg, vx)

    def on_cloud(self, msg: PointCloud2) -> None:
        try:
            x, y, z = self._parse_xyz(msg)
        except Exception as exc:
            self.get_logger().warn(f"PointCloud2 parse failed: {exc}")
            return
        if len(z) == 0:
            return
        z_odom = self._to_height_frame_z(msg.header, x, y, z)
        if z_odom is None:
            return
        ground_z, peak_pct = self._detect_ground_z(z_odom)
        if ground_z is None:
            return
        self.cloud_ground_z = ground_z
        self.cloud_ground_peak_pct = peak_pct

    @staticmethod
    def _parse_xyz(cloud: PointCloud2) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        offsets = {field.name: field.offset for field in cloud.fields if field.name in ("x", "y", "z")}
        if len(offsets) < 3:
            raise ValueError("PointCloud2 missing x/y/z fields")
        count = cloud.width * cloud.height if cloud.height > 1 else cloud.width
        dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": [np.float32, np.float32, np.float32],
                "offsets": [offsets["x"], offsets["y"], offsets["z"]],
                "itemsize": cloud.point_step,
            }
        )
        pts = np.frombuffer(cloud.data, dtype=dtype, count=count)
        return pts["x"], pts["y"], pts["z"]

    def _to_height_frame_z(
        self,
        header,
        x: np.ndarray,
        y: np.ndarray,
        z: np.ndarray,
    ) -> np.ndarray | None:
        src = header.frame_id
        if not src or src == self.height_frame:
            return z.astype(np.float64, copy=False)
        if self.tf_buffer is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(self.height_frame, src, Time())
        except Exception as exc:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if now_s - self.last_tf_warn_s >= 1.0:
                self.last_tf_warn_s = now_s
                self.get_logger().warn(f"TF {src}->{self.height_frame} failed: {str(exc)[:80]}")
            return None

        rot = transform.transform.rotation
        xx = rot.x * rot.x
        yy = rot.y * rot.y
        xz = rot.x * rot.z
        yz = rot.y * rot.z
        wx = rot.w * rot.x
        wy = rot.w * rot.y
        r20 = 2.0 * (xz - wy)
        r21 = 2.0 * (yz + wx)
        r22 = 1.0 - 2.0 * (xx + yy)
        return (
            r20 * x.astype(np.float64, copy=False)
            + r21 * y.astype(np.float64, copy=False)
            + r22 * z.astype(np.float64, copy=False)
            + float(transform.transform.translation.z)
        )

    def _detect_ground_z(self, z_values: np.ndarray) -> tuple[float | None, float]:
        finite = z_values[np.isfinite(z_values)]
        if len(finite) < 20:
            return None, 0.0
        lo = math.floor(float(finite.min()) / self.ground_histogram_bin_width) * self.ground_histogram_bin_width
        hi = math.ceil(float(finite.max()) / self.ground_histogram_bin_width) * self.ground_histogram_bin_width
        if hi <= lo:
            hi = lo + self.ground_histogram_bin_width
        bins = np.arange(lo, hi + self.ground_histogram_bin_width, self.ground_histogram_bin_width)
        hist, edges = np.histogram(finite, bins=bins)
        peak = int(hist.max()) if len(hist) else 0
        if peak <= 0:
            return None, 0.0
        threshold = peak * self.ground_peak_ratio
        idx = next((i for i, count in enumerate(hist) if count >= threshold), int(np.argmax(hist)))
        ground_z = float((edges[idx] + edges[idx + 1]) * 0.5)
        peak_pct = 100.0 * float(hist[idx]) / float(len(finite))
        return ground_z, peak_pct

    def _update_ground_offset_from_cloud(self) -> None:
        if self.cloud_ground_z is None:
            return
        if self.state not in (STATE_FLAT, STATE_FLAT_TO_UPHILL):
            return
        detected_offset = self.last_odom_z - self.cloud_ground_z
        if not math.isfinite(detected_offset):
            return
        if detected_offset < -0.2 or detected_offset > 0.8:
            return
        if self.measured_ground_offset_m is None:
            self.measured_ground_offset_m = detected_offset
            self.get_logger().info(
                f"ground offset locked from cloud: {self.measured_ground_offset_m:.3f}m "
                f"(ground_z={self.cloud_ground_z:.3f}, peak={self.cloud_ground_peak_pct:.1f}%)"
            )
        else:
            self.measured_ground_offset_m += self.ground_offset_alpha * (
                detected_offset - self.measured_ground_offset_m
            )

    def _ema(self, old: float | None, new: float) -> float:
        if old is None:
            return new
        return old * (1.0 - self.ema_alpha) + new * self.ema_alpha

    def _vx(self) -> float:
        if len(self.history) < 2:
            return 0.0
        t0, x0, _ = self.history[0]
        t1, x1, _ = self.history[-1]
        dt = t1 - t0
        if dt <= 1e-6:
            return 0.0
        return (x1 - x0) / dt

    def _held(self, key: str, rel_s: float, cond: bool) -> bool:
        return self._held_for(key, rel_s, cond, self.hold_s)

    def _held_for(self, key: str, rel_s: float, cond: bool, hold_s: float) -> bool:
        if not cond:
            self.candidate_since[key] = None
            return False
        start = self.candidate_since.get(key)
        if start is None:
            self.candidate_since[key] = rel_s
            return False
        return rel_s - start >= hold_s

    def _set_state(self, state: str, rel_s: float) -> None:
        if state == self.state:
            return
        self.state = state
        self.state_since_s = rel_s
        self.candidate_since.clear()
        self.pitch_candidate_history.clear()
        if state == STATE_FLAT_TO_UPHILL:
            self.candidate_start_pitch = self.pitch_abs_ema
            self.candidate_peak_pitch = self.pitch_abs_ema or 0.0
        else:
            self.candidate_start_pitch = None
            self.candidate_peak_pitch = 0.0

    def _candidate_growth_ok(self, rel_s: float, pitch_abs: float) -> bool:
        if self.state != STATE_FLAT_TO_UPHILL:
            return False
        if self.candidate_start_pitch is None:
            self.candidate_start_pitch = pitch_abs
        self.candidate_peak_pitch = max(self.candidate_peak_pitch, pitch_abs)
        self.pitch_candidate_history.append((rel_s, pitch_abs))
        while (
            self.pitch_candidate_history
            and rel_s - self.pitch_candidate_history[0][0] > self.growth_check_window_s
        ):
            self.pitch_candidate_history.popleft()
        total_growth = pitch_abs - self.candidate_start_pitch
        if len(self.pitch_candidate_history) < 2:
            return False
        t0, p0 = self.pitch_candidate_history[0]
        t1, p1 = self.pitch_candidate_history[-1]
        dt = max(1e-6, t1 - t0)
        growth_rate = (p1 - p0) / dt
        return total_growth >= self.min_total_growth_deg and growth_rate >= self.min_growth_rate_deg_s

    def _reset_state_machine(self) -> None:
        self.baseline_samples.clear()
        self.baseline = None
        self.state = STATE_FLAT
        self.state_since_s = None
        self.t0_s = None
        self.history.clear()
        self.pitch_abs_ema = None
        self.z_rise_ema = None
        self.pitch_candidate_history.clear()
        self.pose_history.clear()
        self.candidate_start_pitch = None
        self.candidate_peak_pitch = 0.0
        self.candidate_since.clear()
        self.record_trajectory = False
        self.trajectory_points.clear()
        self.trajectory_poses.clear()
        self.transition_marker_id = 100
        self.pending_uphill_start_pose = None
        self.pending_uphill_start_z_rise = 0.0
        self.pending_uphill_end_sample = None
        self.endpoint_markers.clear()
        self._clear_trajectory_markers()
        self._clear_endpoint_markers()
        self.get_logger().warn("已按 R 重置：重新采集平地基准")

    def _record_transition_pose(self, pose: PoseStamped, new_state: str) -> None:
        if new_state == STATE_FLAT_TO_UPHILL:
            self.pending_uphill_start_pose = copy.deepcopy(pose)
            self.pending_uphill_start_z_rise = float(self.z_rise_ema or 0.0)
            self.pending_uphill_end_sample = None
        elif new_state == STATE_UPHILL and self.pending_uphill_start_pose is not None:
            self._clear_endpoint_markers()
            self._publish_endpoint_marker(self.pending_uphill_start_pose, 0, self.pending_uphill_start_z_rise)
        elif new_state == STATE_UPHILL_TO_PLATFORM:
            sample = self.pending_uphill_end_sample or self._find_uphill_end_sample()
            if sample is not None:
                self._publish_endpoint_marker(sample.pose, 1, sample.z_rise_ema)
        elif new_state == STATE_PLATFORM:
            sample = self.pending_uphill_end_sample or self._find_uphill_end_sample()
            if sample is None:
                self._publish_endpoint_marker(pose, 1)
            else:
                self._publish_endpoint_marker(sample.pose, 1, sample.z_rise_ema)
        elif new_state == STATE_FLAT:
            self.pending_uphill_start_pose = None
            self.pending_uphill_start_z_rise = 0.0
            self.pending_uphill_end_sample = None

        if not self.publish_trajectory:
            return

        if not self.record_trajectory:
            self.record_trajectory = True
            self.trajectory_points.clear()
            self.trajectory_poses.clear()
            self.transition_marker_id = 100
            self._clear_trajectory_markers()
            self.get_logger().info(
                "trajectory recording started: /uphill/trajectory_path, /uphill/trajectory_marker"
            )

        marker = Marker()
        marker.header = pose.header
        marker.ns = "uphill_transition"
        marker.id = self.transition_marker_id
        self.transition_marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.a = 0.95
        marker.color.r, marker.color.g, marker.color.b = self._state_color(new_state)
        self.transition_marker_pub.publish(marker)

    def _record_trajectory_point(self, msg: Odometry) -> None:
        if not self.publish_trajectory or not self.record_trajectory:
            return
        point = Point()
        point.x = float(msg.pose.pose.position.x)
        point.y = float(msg.pose.pose.position.y)
        point.z = float(msg.pose.pose.position.z)
        if self.trajectory_points:
            last = self.trajectory_points[-1]
            if math.hypot(point.x - last.x, point.y - last.y) < 0.01 and abs(point.z - last.z) < 0.005:
                return
        self.trajectory_points.append(point)

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "odom"
        pose.pose = msg.pose.pose
        self.trajectory_poses.append(pose)
        if len(self.trajectory_points) > 3000:
            self.trajectory_points = self.trajectory_points[-3000:]
            self.trajectory_poses = self.trajectory_poses[-3000:]
        self._publish_trajectory_markers(msg.header.stamp)

    def _record_pose_history(
        self,
        rel_s: float,
        pose: PoseStamped,
        pitch_abs_ema: float,
        z_rise_ema: float,
        ground_z: float | None,
    ) -> None:
        self.pose_history.append(
            OdomPoseSample(
                rel_s=rel_s,
                pose=copy.deepcopy(pose),
                pitch_abs_ema=pitch_abs_ema,
                z_rise_ema=z_rise_ema,
                ground_z=ground_z,
            )
        )
        while self.pose_history and rel_s - self.pose_history[0].rel_s > self.endpoint_history_s:
            self.pose_history.popleft()

    def _find_uphill_start_sample(self) -> OdomPoseSample | None:
        if not self.pose_history:
            return None
        samples = list(self.pose_history)
        threshold = max(1.0, self.pitch_start_deg + 0.25)
        candidates = [s for s in samples if s.pitch_abs_ema <= threshold]
        if candidates:
            return candidates[-1]
        return min(samples, key=lambda s: s.pitch_abs_ema)

    def _find_uphill_end_sample(self) -> OdomPoseSample | None:
        if not self.pose_history:
            return None
        samples = list(self.pose_history)
        high_samples = [s for s in samples if s.pitch_abs_ema >= self.pitch_stable_deg - 1.0]
        if high_samples:
            return max(high_samples, key=lambda s: s.rel_s)
        return max(samples, key=lambda s: s.pitch_abs_ema)

    def _publish_trajectory_markers(self, stamp) -> None:
        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = stamp
        path.poses = list(self.trajectory_poses)
        self.path_pub.publish(path)

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = stamp
        marker.ns = "uphill_trajectory"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.color.r = 1.0
        marker.color.g = 0.55
        marker.color.b = 0.0
        marker.color.a = 0.95
        marker.points = list(self.trajectory_points)
        self.traj_marker_pub.publish(marker)

    def _clear_trajectory_markers(self) -> None:
        for pub, ns in (
            (self.traj_marker_pub, "uphill_trajectory"),
            (self.transition_marker_pub, "uphill_transition"),
        ):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ns
            marker.id = 0
            marker.action = Marker.DELETEALL
            pub.publish(marker)

    def _publish_endpoint_marker(
        self,
        pose: PoseStamped,
        marker_id: int,
        z_rise_override: float | None = None,
    ) -> None:
        marker = Marker()
        marker.header = pose.header
        marker.ns = "uphill_endpoints"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = copy.deepcopy(pose.pose)
        raw_z = float(pose.pose.position.z)
        z_rise = float(self.z_rise_ema or 0.0) if z_rise_override is None else z_rise_override
        ground_offset = self._endpoint_ground_offset()
        marker.pose.position.z = raw_z - ground_offset
        marker.scale.x = self.endpoint_marker_size
        marker.scale.y = self.endpoint_marker_size
        marker.scale.z = self.endpoint_marker_size
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.endpoint_markers[marker_id] = copy.deepcopy(marker)
        self.endpoint_marker_pub.publish(marker)

        name = "start" if marker_id == 0 else "end"
        self.get_logger().info(
            f"endpoint {name}: x={marker.pose.position.x:.3f}, "
            f"y={marker.pose.position.y:.3f}, z={marker.pose.position.z:.3f}, "
            f"odom_z={raw_z:.3f}, z_rise={z_rise:.3f}, "
            f"ground_offset={ground_offset:.3f}, cloud_ground={self.cloud_ground_z}"
        )

    def _endpoint_ground_offset(self) -> float:
        if self.measured_ground_offset_m is not None:
            return float(self.measured_ground_offset_m)
        return float(self.endpoint_ground_offset_m)

    def _clear_endpoint_markers(self) -> None:
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "uphill_endpoints"
        marker.id = 0
        marker.action = Marker.DELETEALL
        self.endpoint_marker_pub.publish(marker)

    def _republish_endpoint_markers(self) -> None:
        for marker in self.endpoint_markers.values():
            marker.header.stamp = self.get_clock().now().to_msg()
            self.endpoint_marker_pub.publish(marker)

    @staticmethod
    def _state_color(state: str) -> tuple[float, float, float]:
        if state == STATE_FLAT_TO_UPHILL:
            return 1.0, 0.8, 0.0
        if state == STATE_UPHILL:
            return 1.0, 0.2, 0.0
        if state == STATE_UPHILL_TO_PLATFORM:
            return 0.2, 0.7, 1.0
        if state == STATE_PLATFORM:
            return 0.2, 1.0, 0.2
        return 0.8, 0.8, 0.8

    def _keyboard_loop(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().warn("当前终端不支持按键读取，R 重置不可用")
            return
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok() and not self.stop_keyboard:
                readable, _, _ = select.select([sys.stdin], [], [], 0.10)
                if not readable:
                    continue
                key = sys.stdin.read(1)
                if key in ("r", "R"):
                    self._reset_state_machine()
                elif key == "\x03":
                    self.request_shutdown("收到 Ctrl+C")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def request_shutdown(self, reason: str = "请求退出") -> None:
        if self.shutdown_requested:
            return
        self.shutdown_requested = True
        self.stop_keyboard = True
        self.get_logger().info(f"{reason}，准备安全退出")
        if rclpy.ok():
            rclpy.shutdown()

    def _publish(self, rel_s: float, pitch_delta_deg: float, vx: float) -> None:
        public_state = self._public_state()
        state_msg = String()
        state_msg.data = public_state
        self.state_pub.publish(state_msg)

        debug = {
            "state": public_state,
            "internal_state": self.state,
            "rel_s": round(rel_s, 3),
            "state_duration_s": round(rel_s - (self.state_since_s or rel_s), 3),
            "pitch_delta_deg": round(pitch_delta_deg, 3),
            "pitch_abs_ema_deg": round(self.pitch_abs_ema or 0.0, 3),
            "pitch_start_deg": round(self.pitch_start_deg, 3),
            "pitch_stable_deg": round(self.pitch_stable_deg, 3),
            "start_hold_s": round(self.start_hold_s, 3),
            "growth_window_s": round(self.growth_check_window_s, 3),
            "candidate_start_pitch": round(self.candidate_start_pitch or 0.0, 3),
            "candidate_peak_pitch": round(self.candidate_peak_pitch, 3),
            "z_rise_ema_m": round(self.z_rise_ema or 0.0, 4),
            "z_start_m": round(self.z_start_m, 4),
            "vx_mps": round(vx, 4),
        }
        debug_msg = String()
        debug_msg.data = json.dumps(debug, ensure_ascii=False)
        self.debug_pub.publish(debug_msg)

    def _public_state(self) -> str:
        if self.state == STATE_FLAT_TO_UPHILL:
            return STATE_FLAT
        return self.state


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = UphillStateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if "Unable to convert call argument" not in str(exc):
            raise
    finally:
        node.stop_keyboard = True
        try:
            if node.keyboard_thread.is_alive():
                node.keyboard_thread.join(timeout=0.5)
        except KeyboardInterrupt:
            pass
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
