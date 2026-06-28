#!/usr/bin/env python3
"""Fuse Zone3 corner TF with nine-grid point-cloud PCA as an independent node."""

from __future__ import annotations

import json
import math
import os
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def parse_cloud_xyz(cloud: PointCloud2) -> np.ndarray:
    count = cloud.width * cloud.height if cloud.height > 1 else cloud.width
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [np.float32, np.float32, np.float32],
            "offsets": [0, 4, 8],
            "itemsize": cloud.point_step,
        }
    )
    raw = np.frombuffer(cloud.data, dtype=dtype, count=count)
    pts = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(np.float64, copy=False)
    keep = np.isfinite(pts[:, 0]) & np.isfinite(pts[:, 1]) & np.isfinite(pts[:, 2])
    return pts[keep]


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def angle_abs_half_turn(angle: float) -> float:
    wrapped = math.atan2(math.sin(angle), math.cos(angle))
    return min(abs(wrapped), abs(math.pi - abs(wrapped)))


def signed_half_turn(angle: float) -> float:
    wrapped = math.atan2(math.sin(angle), math.cos(angle))
    if wrapped > math.pi / 2.0:
        wrapped -= math.pi
    elif wrapped < -math.pi / 2.0:
        wrapped += math.pi
    return wrapped


def connected_components(x: np.ndarray, y: np.ndarray, cell: float = 0.08) -> list[np.ndarray]:
    if len(x) == 0:
        return []
    ix = np.floor(x / cell).astype(np.int32)
    iy = np.floor(y / cell).astype(np.int32)
    keys = np.column_stack((ix, iy))
    unique, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    active = counts >= 2
    if not active.any():
        return []

    cell_points: dict[int, list[int]] = {}
    for point_i, cell_i in enumerate(inv):
        if active[cell_i]:
            cell_points.setdefault(int(cell_i), []).append(point_i)
    coord_to_cell = {tuple(coord): int(i) for i, coord in enumerate(unique) if active[i]}

    visited: set[int] = set()
    comps: list[np.ndarray] = []
    for start in list(cell_points):
        if start in visited:
            continue
        stack = [start]
        visited.add(start)
        points: list[int] = []
        while stack:
            current = stack.pop()
            points.extend(cell_points[current])
            cx, cy = unique[current]
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nb = coord_to_cell.get((int(cx + dx), int(cy + dy)))
                    if nb is not None and nb not in visited:
                        visited.add(nb)
                        stack.append(nb)
        comps.append(np.asarray(points, dtype=np.int32))
    comps.sort(key=len, reverse=True)
    return comps


def robust_span(values: np.ndarray, lo: float = 3.0, hi: float = 97.0) -> float:
    if len(values) == 0:
        return 0.0
    return float(np.percentile(values, hi) - np.percentile(values, lo))


def count_grid_layers(h: np.ndarray) -> int:
    layers = ((0.80, 1.34), (1.34, 1.88), (1.88, 2.50))
    return sum(int(((h >= lo) & (h < hi)).sum()) >= 18 for lo, hi in layers)


def count_grid_columns(local_y: np.ndarray) -> int:
    if len(local_y) == 0:
        return 0
    center = 0.5 * (float(np.percentile(local_y, 3.0)) + float(np.percentile(local_y, 97.0)))
    return sum(int((np.abs((local_y - center) - cy) <= 0.25).sum()) >= 10 for cy in (-0.54, 0.0, 0.54))


class Zone3GridFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("zone3_grid_fusion")

        self.parent_frame = str(self.declare_parameter("parent_frame", "odom").value)
        self.corner_frame = str(self.declare_parameter("corner_frame", "blue_zone3_root_auto").value)
        self.output_frame = str(self.declare_parameter("output_frame", "blue_zone3_root_grid").value)
        self.cloud_topic = str(self.declare_parameter("cloud_topic", "/odin1/cloud_slam").value)
        self.state_topic = str(self.declare_parameter("state_topic", "/uphill/state").value)
        self.result_topic = str(self.declare_parameter("result_topic", "/zone3/grid_fusion/result").value)
        self.marker_topic = str(self.declare_parameter("marker_topic", "/zone3/grid_fusion/markers").value)
        self.grid_center_x = float(self.declare_parameter("grid_center_x_m", -3.025).value)
        self.grid_center_y = float(self.declare_parameter("grid_center_y_m", -0.150).value)
        self.grid_min_h = float(self.declare_parameter("grid_min_h_m", 0.72).value)
        self.grid_max_h = float(self.declare_parameter("grid_max_h_m", 2.60).value)
        self.grid_roi_x = float(self.declare_parameter("grid_roi_x_m", 1.25).value)
        self.grid_roi_y = float(self.declare_parameter("grid_roi_y_m", 1.40).value)
        self.grid_expected_width = float(self.declare_parameter("grid_expected_width_m", 1.62).value)
        self.grid_min_width = float(self.declare_parameter("grid_min_width_m", 1.35).value)
        self.grid_max_width = float(self.declare_parameter("grid_max_width_m", 1.95).value)
        self.grid_max_depth = float(self.declare_parameter("grid_max_depth_m", 0.80).value)
        self.grid_max_center_err = float(self.declare_parameter("grid_max_center_err_m", 0.65).value)
        self.grid_max_yaw_err = math.radians(float(self.declare_parameter("grid_max_yaw_err_deg", 20.0).value))
        self.min_grid_points = int(self.declare_parameter("min_grid_points", 35).value)
        self.max_cloud_points = int(self.declare_parameter("max_cloud_points", 180000).value)
        self.sample_per_frame = int(self.declare_parameter("sample_per_frame", 3500).value)
        self.xy_gain = float(self.declare_parameter("xy_gain", 0.35).value)
        self.yaw_gain = float(self.declare_parameter("yaw_gain", 0.25).value)
        self.max_xy_correction = float(self.declare_parameter("max_xy_correction_m", 0.22).value)
        self.max_yaw_correction = math.radians(float(self.declare_parameter("max_yaw_correction_deg", 2.5).value))
        self.publish_without_grid = bool(self.declare_parameter("publish_without_grid", True).value)
        self.hold_last_grid_sec = float(self.declare_parameter("hold_last_grid_sec", 8.0).value)
        self.min_hold_score = float(self.declare_parameter("min_hold_score", 120.0).value)
        self.save_debug_log = bool(self.declare_parameter("save_debug_log", False).value)
        self.console_log_interval_sec = float(self.declare_parameter("console_log_interval_sec", 5.0).value)
        self.console_log_only_changes = bool(self.declare_parameter("console_log_only_changes", True).value)
        self.debug_log_path = str(
            self.declare_parameter(
                "debug_log_path",
                "/mnt/c/Users/22240/rc2026_snapshot/outputs/zone3_grid_fusion_debug.jsonl",
            ).value
        )
        self.timer_hz = float(self.declare_parameter("timer_hz", 8.0).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_br = TransformBroadcaster(self)
        self.cloud_samples: deque[np.ndarray] = deque()
        self.cloud_points = 0
        self.state = "unknown"
        self.last_result: dict | None = None
        self.last_good_grid: dict | None = None
        self.last_good_grid_sec = 0.0
        self._last_log_sec = 0.0
        self._last_log_used_grid: bool | None = None
        self._last_log_score: float | None = None
        self._debug_log_ready = False

        self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, 10)
        self.create_subscription(String, self.state_topic, self.on_state, 10)
        self.pub_result = self.create_publisher(String, self.result_topic, 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/zone3/grid_fusion/root_pose", 10)
        self.pub_marker = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.timer = self.create_timer(1.0 / max(0.5, self.timer_hz), self.on_timer)
        self.get_logger().info(
            f"zone3_grid_fusion ready: {self.corner_frame} + {self.cloud_topic} -> {self.output_frame}"
        )
        if self.save_debug_log:
            self.prepare_debug_log()

    def on_state(self, msg: String) -> None:
        self.state = msg.data.strip()

    def on_cloud(self, msg: PointCloud2) -> None:
        pts = parse_cloud_xyz(msg)
        if len(pts) == 0:
            return
        step = max(1, len(pts) // max(1, self.sample_per_frame))
        sample = pts[::step]
        self.cloud_samples.append(sample)
        self.cloud_points += len(sample)
        while self.cloud_samples and self.cloud_points > self.max_cloud_points:
            old = self.cloud_samples.popleft()
            self.cloud_points -= len(old)

    def lookup_corner_pose(self) -> tuple[float, float, float, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform(self.parent_frame, self.corner_frame, rclpy.time.Time())
        except TransformException:
            return None
        t = tf.transform.translation
        yaw = yaw_from_quat(tf.transform.rotation)
        return float(t.x), float(t.y), float(t.z), yaw

    def detect_grid(self, root_pose: tuple[float, float, float, float]) -> dict | None:
        if not self.cloud_samples:
            return None
        pts = np.concatenate(list(self.cloud_samples), axis=0)
        if len(pts) < 300:
            return None
        root_x, root_y, root_z, root_yaw = root_pose
        dx = pts[:, 0] - root_x
        dy = pts[:, 1] - root_y
        lx = dx * math.cos(root_yaw) + dy * math.sin(root_yaw)
        ly = -dx * math.sin(root_yaw) + dy * math.cos(root_yaw)
        h = pts[:, 2] - root_z
        xy_roi = (
            (np.abs(lx - self.grid_center_x) <= self.grid_roi_x)
            & (np.abs(ly - self.grid_center_y) <= self.grid_roi_y)
            & np.isfinite(h)
        )
        if int(xy_roi.sum()) < self.min_grid_points:
            return None
        bottom_h = self.visible_bottom_height(h[xy_roi])
        roi = (
            xy_roi
            & (h >= self.grid_min_h)
            & (h <= self.grid_max_h)
        )
        bottom_roi = (
            xy_roi
            & (h >= bottom_h + self.grid_min_h)
            & (h <= bottom_h + self.grid_max_h)
        )
        if int(bottom_roi.sum()) >= self.min_grid_points:
            roi = bottom_roi
        if int(roi.sum()) < self.min_grid_points:
            return None
        rx = lx[roi]
        ry = ly[roi]
        rh = h[roi]
        comps = connected_components(rx, ry, cell=0.08)
        best = None
        for comp in comps[:8]:
            if len(comp) < self.min_grid_points:
                continue
            cx = rx[comp]
            cy = ry[comp]
            ch = rh[comp]
            cloud = np.column_stack((cx, cy))
            center = cloud.mean(axis=0)
            try:
                _, _, vh = np.linalg.svd(cloud - center, full_matrices=False)
            except np.linalg.LinAlgError:
                continue
            long_axis = vh[0]
            long_axis /= max(1e-9, float(np.linalg.norm(long_axis)))
            yaw = math.atan2(float(long_axis[1]), float(long_axis[0])) - math.pi / 2.0
            cyaw = math.cos(yaw)
            syaw = math.sin(yaw)
            local_x = cyaw * (cx - center[0]) + syaw * (cy - center[1])
            local_y = -syaw * (cx - center[0]) + cyaw * (cy - center[1])
            fit_x, fit_y, fit_h, support_bins = self.supported_grid_slice(local_x, local_y, ch)
            width = robust_span(fit_y)
            depth = robust_span(fit_x, 5.0, 95.0)
            if width < self.grid_min_width or width > self.grid_max_width or depth > self.grid_max_depth:
                continue
            center_local_x = 0.5 * (float(np.percentile(fit_x, 5.0)) + float(np.percentile(fit_x, 95.0)))
            center_local_y = 0.5 * (float(np.percentile(fit_y, 3.0)) + float(np.percentile(fit_y, 97.0)))
            fit_center_x = float(center[0] + cyaw * center_local_x - syaw * center_local_y)
            fit_center_y = float(center[1] + syaw * center_local_x + cyaw * center_local_y)
            layer_count = count_grid_layers(fit_h)
            column_count = count_grid_columns(fit_y)
            center_err = math.hypot(fit_center_x - self.grid_center_x, fit_center_y - self.grid_center_y)
            yaw_err = angle_abs_half_turn(yaw)
            if center_err > self.grid_max_center_err or yaw_err > self.grid_max_yaw_err:
                continue
            score = (
                100.0 * math.exp(-center_err / 0.50)
                + 80.0 * math.exp(-yaw_err / math.radians(12.0))
                + 85.0 * math.exp(-abs(width - self.grid_expected_width) / 0.22)
                + 40.0 * math.exp(-max(0.0, depth - 0.35) / 0.25)
                + 35.0 * min(1.0, layer_count / 3.0)
                + 30.0 * min(1.0, column_count / 3.0)
                + 20.0 * min(1.0, len(fit_x) / 420.0)
            )
            item = {
                "score": float(score),
                "points": int(len(fit_x)),
                "raw_points": int(len(comp)),
                "center_x": float(fit_center_x),
                "center_y": float(fit_center_y),
                "center_err": float(center_err),
                "yaw": float(yaw),
                "yaw_err": float(yaw_err),
                "width": float(width),
                "depth": float(depth),
                "layers": int(layer_count),
                "columns": int(column_count),
                "support_bins": int(support_bins),
            }
            if best is None or item["score"] > best["score"]:
                best = item
        return best

    @staticmethod
    def visible_bottom_height(h: np.ndarray) -> float:
        finite = h[np.isfinite(h)]
        if len(finite) < 20:
            return 0.0
        low = finite[(finite >= -0.80) & (finite <= 1.00)]
        source = low if len(low) >= 20 else finite
        return float(np.percentile(source, 2.0))

    def supported_grid_slice(self, local_x: np.ndarray, local_y: np.ndarray, h: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
        bin_size = 0.06
        bins = np.floor(local_x / bin_size).astype(np.int32)
        keep_bins: list[int] = []
        for bin_id in np.unique(bins):
            in_bin = bins == bin_id
            if int(in_bin.sum()) < 8:
                continue
            if robust_span(local_y[in_bin]) < 0.55:
                continue
            keep_bins.append(int(bin_id))
        if len(keep_bins) < 3:
            return local_x, local_y, h, 0
        keep = np.isin(bins, np.asarray(keep_bins, dtype=np.int32))
        if int(keep.sum()) < self.min_grid_points:
            return local_x, local_y, h, 0
        return local_x[keep], local_y[keep], h[keep], len(keep_bins)

    def fuse_pose(self, root_pose: tuple[float, float, float, float], grid: dict | None) -> tuple[float, float, float, float, dict]:
        root_x, root_y, root_z, root_yaw = root_pose
        if grid is None:
            return root_x, root_y, root_z, root_yaw, {"used_grid": False, "dx": 0.0, "dy": 0.0, "dyaw": 0.0}
        local_dx = float(grid["center_x"]) - self.grid_center_x
        local_dy = float(grid["center_y"]) - self.grid_center_y
        local_norm = math.hypot(local_dx, local_dy)
        if local_norm > self.max_xy_correction:
            scale = self.max_xy_correction / max(1e-9, local_norm)
            local_dx *= scale
            local_dy *= scale
        dyaw_source = signed_half_turn(float(grid["yaw"]))
        dyaw = max(-self.max_yaw_correction, min(self.max_yaw_correction, dyaw_source))
        local_dx *= self.xy_gain
        local_dy *= self.xy_gain
        dyaw *= self.yaw_gain
        fused_yaw = root_yaw + dyaw
        fused_x = root_x + math.cos(root_yaw) * local_dx - math.sin(root_yaw) * local_dy
        fused_y = root_y + math.sin(root_yaw) * local_dx + math.cos(root_yaw) * local_dy
        return fused_x, fused_y, root_z, fused_yaw, {
            "used_grid": True,
            "dx": float(local_dx),
            "dy": float(local_dy),
            "dyaw": float(dyaw),
        }

    def publish_tf(self, pose: tuple[float, float, float, float]) -> None:
        x, y, z, yaw = pose
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.parent_frame
        tf.child_frame_id = self.output_frame
        tf.transform.translation.x = float(x)
        tf.transform.translation.y = float(y)
        tf.transform.translation.z = float(z)
        qx, qy, qz, qw = quat_from_yaw(yaw)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_br.sendTransform(tf)

    def publish_pose(self, pose: tuple[float, float, float, float]) -> None:
        x, y, z, yaw = pose
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.parent_frame
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        qx, qy, qz, qw = quat_from_yaw(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub_pose.publish(msg)

    def publish_markers(self, pose: tuple[float, float, float, float], grid: dict | None) -> None:
        x, y, z, yaw = pose
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        center = Marker()
        center.header.frame_id = self.parent_frame
        center.header.stamp = stamp
        center.ns = "zone3_grid_fusion"
        center.id = 1
        center.type = Marker.SPHERE
        center.action = Marker.ADD
        center.scale.x = 0.10
        center.scale.y = 0.10
        center.scale.z = 0.10
        center.color.r = 1.0
        center.color.g = 0.9
        center.color.b = 0.0
        center.color.a = 0.95
        center.pose.position.x = float(x)
        center.pose.position.y = float(y)
        center.pose.position.z = float(z + 0.08)
        center.pose.orientation.w = 1.0
        ma.markers.append(center)

        if grid is not None:
            gx = x + math.cos(yaw) * self.grid_center_x - math.sin(yaw) * self.grid_center_y
            gy = y + math.sin(yaw) * self.grid_center_x + math.cos(yaw) * self.grid_center_y
            grid_marker = Marker()
            grid_marker.header.frame_id = self.parent_frame
            grid_marker.header.stamp = stamp
            grid_marker.ns = "zone3_grid_fusion"
            grid_marker.id = 2
            grid_marker.type = Marker.CUBE
            grid_marker.action = Marker.ADD
            grid_marker.scale.x = 0.30
            grid_marker.scale.y = 1.62
            grid_marker.scale.z = 0.05
            grid_marker.color.r = 1.0
            grid_marker.color.g = 1.0
            grid_marker.color.b = 1.0
            grid_marker.color.a = 0.55
            grid_marker.pose.position.x = float(gx)
            grid_marker.pose.position.y = float(gy)
            grid_marker.pose.position.z = float(z + 0.85)
            qx, qy, qz, qw = quat_from_yaw(yaw + float(grid["yaw"]))
            grid_marker.pose.orientation.x = qx
            grid_marker.pose.orientation.y = qy
            grid_marker.pose.orientation.z = qz
            grid_marker.pose.orientation.w = qw
            ma.markers.append(grid_marker)
        self.pub_marker.publish(ma)

    def on_timer(self) -> None:
        root_pose = self.lookup_corner_pose()
        if root_pose is None:
            return
        grid = self.detect_grid(root_pose)
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if grid is not None:
            if float(grid["score"]) >= self.min_hold_score:
                self.last_good_grid = dict(grid)
                self.last_good_grid_sec = now_sec
            else:
                grid = None
        elif self.last_good_grid is not None and now_sec - self.last_good_grid_sec <= self.hold_last_grid_sec:
            grid = dict(self.last_good_grid)
            grid["held"] = True
        if grid is None and not self.publish_without_grid:
            return
        fused = self.fuse_pose(root_pose, grid)
        pose = fused[:4]
        meta = fused[4]
        self.publish_tf(pose)
        self.publish_pose(pose)
        self.publish_markers(pose, grid)
        result = {
            "stamp_sec": float(now_sec),
            "state": self.state,
            "corner_frame": self.corner_frame,
            "output_frame": self.output_frame,
            "cloud_points": int(self.cloud_points),
            "used_grid": bool(meta["used_grid"]),
            "grid": grid,
            "correction": meta,
            "corner_pose": {
                "x": root_pose[0],
                "y": root_pose[1],
                "z": root_pose[2],
                "yaw_deg": math.degrees(root_pose[3]),
            },
            "pose": {"x": pose[0], "y": pose[1], "z": pose[2], "yaw_deg": math.degrees(pose[3])},
            "params": {
                "grid_center_x": self.grid_center_x,
                "grid_center_y": self.grid_center_y,
                "grid_roi_x": self.grid_roi_x,
                "grid_roi_y": self.grid_roi_y,
                "grid_expected_width": self.grid_expected_width,
                "grid_min_width": self.grid_min_width,
                "grid_max_width": self.grid_max_width,
                "grid_max_depth": self.grid_max_depth,
                "grid_max_center_err": self.grid_max_center_err,
                "grid_max_yaw_err_deg": math.degrees(self.grid_max_yaw_err),
                "xy_gain": self.xy_gain,
                "yaw_gain": self.yaw_gain,
                "max_xy_correction": self.max_xy_correction,
                "max_yaw_correction_deg": math.degrees(self.max_yaw_correction),
                "min_hold_score": self.min_hold_score,
                "hold_last_grid_sec": self.hold_last_grid_sec,
            },
        }
        self.last_result = result
        msg = String()
        msg.data = json.dumps(result, ensure_ascii=False)
        self.pub_result.publish(msg)
        self.append_debug_log(result)
        should_log = now_sec - self._last_log_sec >= self.console_log_interval_sec
        if self.console_log_only_changes:
            score = float(grid["score"]) if grid is not None else None
            score_changed = (
                score is not None
                and (self._last_log_score is None or abs(score - self._last_log_score) >= 20.0)
            )
            should_log = should_log or bool(meta["used_grid"]) != self._last_log_used_grid or score_changed
        if should_log:
            self._last_log_sec = now_sec
            self._last_log_used_grid = bool(meta["used_grid"])
            self._last_log_score = float(grid["score"]) if grid is not None else None
            if grid is None:
                self.get_logger().info(
                    f"fusion: used_grid=false, cloud={self.cloud_points}, pose=({pose[0]:.3f},{pose[1]:.3f},{math.degrees(pose[3]):.2f}deg)"
                )
            else:
                self.get_logger().info(
                    "fusion: "
                    f"used_grid=true, score={grid['score']:.1f}, n={grid['points']}, "
                    f"center_err={grid['center_err']:.3f}, yaw_err={math.degrees(grid['yaw_err']):.2f}deg, "
                    f"corr=({meta['dx']:.3f},{meta['dy']:.3f},{math.degrees(meta['dyaw']):.2f}deg), "
                    f"pose=({pose[0]:.3f},{pose[1]:.3f},{math.degrees(pose[3]):.2f}deg)"
                )

    def prepare_debug_log(self) -> None:
        try:
            parent = os.path.dirname(self.debug_log_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(self.debug_log_path, "w", encoding="utf-8") as f:
                f.write("")
            self._debug_log_ready = True
            self.get_logger().info(f"grid fusion debug log: {self.debug_log_path}")
        except OSError as exc:
            self._debug_log_ready = False
            self.get_logger().error(f"failed to prepare debug log {self.debug_log_path}: {exc}")

    def append_debug_log(self, result: dict) -> None:
        if not self.save_debug_log or not self._debug_log_ready:
            return
        try:
            with open(self.debug_log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(result, ensure_ascii=False, sort_keys=True) + "\n")
        except OSError as exc:
            self._debug_log_ready = False
            self.get_logger().error(f"failed to append debug log {self.debug_log_path}: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Zone3GridFusionNode()
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
