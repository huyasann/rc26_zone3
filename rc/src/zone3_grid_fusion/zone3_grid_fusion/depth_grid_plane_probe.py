#!/usr/bin/env python3
"""Headless prototype for forward-depth cutting and Zone3 grid cloud candidates."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class Candidate:
    center_x: float
    center_y: float
    center_z: float
    local_x: float
    local_y: float
    yaw: float
    local_yaw: float
    points: int
    confidence: float
    width: float
    depth: float
    layer_count: int
    column_count: int
    plane_points: int
    plane_rmse: float
    depth_support_bins: int
    center_err: float


def parse_cloud_xyz(cloud: PointCloud2) -> np.ndarray:
    offsets: dict[str, int] = {}
    for field in cloud.fields:
        if field.name in ("x", "y", "z"):
            offsets[field.name] = int(field.offset)
    if len(offsets) != 3:
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
    raw = np.frombuffer(cloud.data, dtype=dtype, count=count)
    pts = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(np.float64, copy=False)
    keep = np.isfinite(pts[:, 0]) & np.isfinite(pts[:, 1]) & np.isfinite(pts[:, 2])
    return pts[keep]


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quat_to_matrix(q) -> np.ndarray:
    xx, yy, zz = q.x * q.x, q.y * q.y, q.z * q.z
    xy, xz, yz = q.x * q.y, q.x * q.z, q.y * q.z
    wx, wy, wz = q.w * q.x, q.w * q.y, q.w * q.z
    return np.asarray(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def transform_points(points: np.ndarray, tf) -> np.ndarray:
    q = tf.transform.rotation
    t = tf.transform.translation
    rot = quat_to_matrix(q)
    out = points @ rot.T
    out[:, 0] += float(t.x)
    out[:, 1] += float(t.y)
    out[:, 2] += float(t.z)
    return out


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def normalize_half_turn(angle: float) -> float:
    while angle <= -math.pi / 2.0:
        angle += math.pi
    while angle > math.pi / 2.0:
        angle -= math.pi
    return angle


def robust_span(values: np.ndarray, lo: float = 3.0, hi: float = 97.0) -> float:
    if len(values) == 0:
        return 0.0
    return float(np.percentile(values, hi) - np.percentile(values, lo))


def connected_components(x: np.ndarray, y: np.ndarray, cell: float, min_cell_points: int) -> list[np.ndarray]:
    if len(x) == 0:
        return []
    ix = np.floor(x / cell).astype(np.int32)
    iy = np.floor(y / cell).astype(np.int32)
    keys = np.column_stack((ix, iy))
    unique, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    active = counts >= min_cell_points
    if not active.any():
        return []

    cell_points: dict[int, list[int]] = {}
    for point_i, cell_i in enumerate(inv):
        if active[cell_i]:
            cell_points.setdefault(int(cell_i), []).append(point_i)
    coord_to_cell = {tuple(coord): int(i) for i, coord in enumerate(unique) if active[i]}

    comps: list[np.ndarray] = []
    visited: set[int] = set()
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


def count_height_layers(h: np.ndarray, bottom_h: float) -> int:
    layers = (
        (bottom_h + 0.80, bottom_h + 1.34),
        (bottom_h + 1.34, bottom_h + 1.88),
        (bottom_h + 1.88, bottom_h + 2.42),
    )
    return sum(int(((h >= lo) & (h < hi)).sum()) >= 25 for lo, hi in layers)


class DepthGridPlaneProbe(Node):
    def __init__(self) -> None:
        super().__init__("zone3_depth_grid_plane_probe")

        self.parent_frame = str(self.declare_parameter("parent_frame", "odom").value)
        self.base_frame = str(self.declare_parameter("base_frame", "odin1_base_link").value)
        self.corner_frame = str(self.declare_parameter("corner_frame", "blue_zone3_root_auto").value)
        self.cloud_topic = str(self.declare_parameter("cloud_topic", "/odin1/cloud_slam").value)
        self.result_topic = str(self.declare_parameter("result_topic", "/zone3/depth_grid_probe/result").value)
        self.marker_topic = str(self.declare_parameter("marker_topic", "/zone3/depth_grid_probe/markers").value)

        self.forward_x_min = float(self.declare_parameter("forward_x_min_m", 0.25).value)
        self.forward_x_max = float(self.declare_parameter("forward_x_max_m", 5.20).value)
        self.forward_y_min = float(self.declare_parameter("forward_y_min_m", -1.35).value)
        self.forward_y_max = float(self.declare_parameter("forward_y_max_m", 1.35).value)
        self.forward_z_min = float(self.declare_parameter("forward_z_min_m", -0.55).value)
        self.forward_z_max = float(self.declare_parameter("forward_z_max_m", 2.80).value)
        self.foreground_enable = bool(self.declare_parameter("foreground_ray_filter_enable", True).value)
        self.foreground_bin_deg = float(self.declare_parameter("foreground_ray_bin_deg", 1.0).value)
        self.foreground_keep_depth = float(self.declare_parameter("foreground_keep_depth_m", 0.35).value)
        self.auto_cut_axis = bool(self.declare_parameter("auto_cut_axis", True).value)
        self.root_roi_fallback = bool(self.declare_parameter("root_roi_fallback", True).value)

        self.grid_center_x = float(self.declare_parameter("grid_center_x_m", -3.025).value)
        self.grid_center_y = float(self.declare_parameter("grid_center_y_m", -0.150).value)
        self.root_roi_x = float(self.declare_parameter("root_roi_x_m", 1.80).value)
        self.root_roi_y = float(self.declare_parameter("root_roi_y_m", 1.80).value)
        self.grid_min_h = float(self.declare_parameter("grid_min_h_m", 0.75).value)
        self.grid_max_h = float(self.declare_parameter("grid_max_h_m", 2.60).value)
        self.min_high_points = int(self.declare_parameter("min_high_points", 45).value)
        self.component_cell = float(self.declare_parameter("component_cell_m", 0.08).value)
        self.max_candidates = int(self.declare_parameter("max_candidates", 5).value)
        self.log_interval_sec = float(self.declare_parameter("log_interval_sec", 1.0).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub_result = self.create_publisher(String, self.result_topic, 10)
        self.pub_marker = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, 10)
        self.last_log = 0.0
        self.get_logger().info(
            f"depth_grid_plane_probe ready: {self.cloud_topic}, {self.base_frame}, {self.corner_frame}"
        )

    def on_cloud(self, msg: PointCloud2) -> None:
        try:
            pts_src = parse_cloud_xyz(msg)
        except ValueError as exc:
            self.publish_status("bad_cloud", {"error": str(exc)})
            return
        if len(pts_src) < 50:
            self.publish_status("few_raw_points", {"raw_points": int(len(pts_src))})
            return

        try:
            base_pts = self.to_frame(pts_src, msg.header.frame_id, self.base_frame)
            odom_pts = self.to_frame(pts_src, msg.header.frame_id, self.parent_frame)
            root = self.lookup_root()
        except TransformException as exc:
            self.publish_status("tf_unavailable", {"error": str(exc), "raw_points": int(len(pts_src))})
            return

        selected = self.select_depth_cut(base_pts, odom_pts, root)
        counts = selected["counts"]
        raw_local = self.odom_to_root_local(odom_pts, root)
        raw_roi = self.root_roi_mask(raw_local)
        counts["raw_root_roi"] = int(raw_roi.sum())
        if counts["root_roi"] < self.min_high_points and self.root_roi_fallback and counts["raw_root_roi"] >= self.min_high_points:
            selected = {
                **selected,
                "axis": "root_roi_fallback",
                "odom_cut": odom_pts,
                "local": raw_local,
                "roi": raw_roi,
            }
            counts = {
                **counts,
                "forward_rect": int(len(odom_pts)),
                "foreground": int(raw_roi.sum()),
                "root_roi": int(raw_roi.sum()),
            }
            selected["counts"] = counts
        if counts["forward_rect"] < self.min_high_points:
            self.publish_status("few_forward_rect_points", self.public_cut_debug(selected))
            return
        if counts["foreground"] < self.min_high_points:
            self.publish_status("few_foreground_points", self.public_cut_debug(selected))
            return
        local = selected["local"]
        odom_cut = selected["odom_cut"]
        roi = selected["roi"]
        if counts["root_roi"] < self.min_high_points:
            self.publish_status("few_root_roi_points", self.public_cut_debug(selected))
            return

        local_roi = local[roi]
        odom_roi = odom_cut[roi]
        bottom_h = self.visible_bottom_height(local_roi[:, 2])
        high = (
            (local_roi[:, 2] >= bottom_h + self.grid_min_h)
            & (local_roi[:, 2] <= bottom_h + self.grid_max_h)
        )
        counts["high"] = int(high.sum())
        if counts["high"] < self.min_high_points:
            self.publish_status("few_high_points", {"counts": counts, "bottom_h": bottom_h})
            return

        candidates = self.detect_candidates(local_roi, odom_roi, high, root, bottom_h)
        result = {
            "status": "ok" if candidates else "no_candidate",
            "cloud_frame": msg.header.frame_id,
            "parent_frame": self.parent_frame,
            "base_frame": self.base_frame,
            "corner_frame": self.corner_frame,
            "counts": counts,
            "cut_axis": selected["axis"],
            "cut_profiles": selected["profiles"],
            "bottom_h": bottom_h,
            "best": self.candidate_to_dict(candidates[0]) if candidates else None,
            "candidates": [self.candidate_to_dict(c) for c in candidates],
        }
        self.publish_result(result)
        self.publish_markers(candidates, msg.header.stamp)
        self.log_result(result)

    def to_frame(self, points: np.ndarray, source_frame: str, target_frame: str) -> np.ndarray:
        if not source_frame or source_frame == target_frame:
            return points.astype(np.float64, copy=True)
        tf = self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
        return transform_points(points.astype(np.float64, copy=True), tf)

    def lookup_root(self) -> tuple[float, float, float, float]:
        tf = self.tf_buffer.lookup_transform(self.parent_frame, self.corner_frame, Time())
        t = tf.transform.translation
        yaw = yaw_from_quat(tf.transform.rotation)
        return float(t.x), float(t.y), float(t.z), yaw

    def forward_rect_mask(self, pts: np.ndarray) -> np.ndarray:
        return (
            (pts[:, 0] >= self.forward_x_min)
            & (pts[:, 0] <= self.forward_x_max)
            & (pts[:, 1] >= self.forward_y_min)
            & (pts[:, 1] <= self.forward_y_max)
            & (pts[:, 2] >= self.forward_z_min)
            & (pts[:, 2] <= self.forward_z_max)
        )

    def select_depth_cut(self, base_pts: np.ndarray, odom_pts: np.ndarray, root: tuple[float, float, float, float]) -> dict:
        axes = [("+x", 1.0, 0.0)]
        if self.auto_cut_axis:
            axes = [("+x", 1.0, 0.0), ("-x", -1.0, 0.0), ("+y", 0.0, 1.0), ("-y", 0.0, -1.0)]

        best: dict | None = None
        profiles: list[dict] = []
        for name, ax, ay in axes:
            cut_frame = self.project_base_cut_frame(base_pts, ax, ay)
            rect = self.forward_rect_mask(cut_frame)
            rect_count = int(rect.sum())
            if rect_count <= 0:
                profile = {"axis": name, "forward_rect": 0, "foreground": 0, "root_roi": 0}
                profiles.append(profile)
                continue
            fg = self.foreground_mask(cut_frame[rect])
            odom_cut = odom_pts[rect][fg]
            local = self.odom_to_root_local(odom_cut, root) if len(odom_cut) else np.empty((0, 3), dtype=np.float64)
            roi = self.root_roi_mask(local)
            high_count = 0
            if int(roi.sum()) >= self.min_high_points:
                bottom_h = self.visible_bottom_height(local[roi, 2])
                high = (local[roi, 2] >= bottom_h + self.grid_min_h) & (local[roi, 2] <= bottom_h + self.grid_max_h)
                high_count = int(high.sum())
            profile = {
                "axis": name,
                "forward_rect": rect_count,
                "foreground": int(fg.sum()),
                "root_roi": int(roi.sum()),
                "high": high_count,
            }
            profiles.append(profile)
            item = {
                "axis": name,
                "counts": {
                    "raw": int(len(base_pts)),
                    "forward_rect": rect_count,
                    "foreground": int(fg.sum()),
                    "root_roi": int(roi.sum()),
                },
                "profiles": profiles,
                "odom_cut": odom_cut,
                "local": local,
                "roi": roi,
            }
            key = (profile["root_roi"], profile["high"], profile["foreground"])
            if best is None or key > best["key"]:
                best = {"key": key, **item}

        if best is None:
            empty = np.empty((0, 3), dtype=np.float64)
            return {
                "axis": "none",
                "counts": {"raw": int(len(base_pts)), "forward_rect": 0, "foreground": 0, "root_roi": 0},
                "profiles": profiles,
                "odom_cut": empty,
                "local": empty,
                "roi": np.zeros(0, dtype=bool),
            }
        best["profiles"] = profiles
        best.pop("key", None)
        return best

    @staticmethod
    def project_base_cut_frame(base_pts: np.ndarray, axis_x: float, axis_y: float) -> np.ndarray:
        forward = axis_x * base_pts[:, 0] + axis_y * base_pts[:, 1]
        lateral = -axis_y * base_pts[:, 0] + axis_x * base_pts[:, 1]
        return np.column_stack((forward, lateral, base_pts[:, 2]))

    def root_roi_mask(self, local: np.ndarray) -> np.ndarray:
        if len(local) == 0:
            return np.zeros(0, dtype=bool)
        return (
            (np.abs(local[:, 0] - self.grid_center_x) <= self.root_roi_x)
            & (np.abs(local[:, 1] - self.grid_center_y) <= self.root_roi_y)
            & np.isfinite(local[:, 2])
        )

    @staticmethod
    def public_cut_debug(selected: dict) -> dict:
        return {
            "axis": selected.get("axis", "unknown"),
            "counts": selected.get("counts", {}),
            "profiles": selected.get("profiles", []),
        }

    def foreground_mask(self, pts: np.ndarray) -> np.ndarray:
        if not self.foreground_enable or self.foreground_bin_deg <= 0.0:
            return np.ones(len(pts), dtype=bool)
        ranges = np.hypot(pts[:, 0], pts[:, 1])
        finite = np.isfinite(ranges) & (ranges > 0.05)
        if int(finite.sum()) < self.min_high_points:
            return np.ones(len(pts), dtype=bool)

        angles = np.arctan2(pts[finite, 1], pts[finite, 0])
        bin_size = math.radians(max(0.1, self.foreground_bin_deg))
        ray_bins = np.floor((angles + math.pi) / bin_size).astype(np.int32)
        valid_ranges = ranges[finite]
        order = np.argsort(valid_ranges)
        sorted_bins = ray_bins[order]
        sorted_ranges = valid_ranges[order]
        unique_bins, first_idx = np.unique(sorted_bins, return_index=True)
        min_by_bin = dict(zip(unique_bins.tolist(), sorted_ranges[first_idx].tolist()))

        keep_valid = np.zeros(int(finite.sum()), dtype=bool)
        for i, (bin_id, rng) in enumerate(zip(ray_bins, valid_ranges)):
            if rng <= min_by_bin[int(bin_id)] + self.foreground_keep_depth:
                keep_valid[i] = True
        keep = np.zeros(len(pts), dtype=bool)
        keep[np.flatnonzero(finite)] = keep_valid
        if int(keep.sum()) < self.min_high_points:
            return np.ones(len(pts), dtype=bool)
        return keep

    def odom_to_root_local(self, odom_pts: np.ndarray, root: tuple[float, float, float, float]) -> np.ndarray:
        rx, ry, rz, yaw = root
        dx = odom_pts[:, 0] - rx
        dy = odom_pts[:, 1] - ry
        c = math.cos(yaw)
        s = math.sin(yaw)
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        h = odom_pts[:, 2] - rz
        return np.column_stack((lx, ly, h))

    def root_local_to_odom(self, x: float, y: float, z: float, root: tuple[float, float, float, float]) -> tuple[float, float, float]:
        rx, ry, rz, yaw = root
        c = math.cos(yaw)
        s = math.sin(yaw)
        return rx + c * x - s * y, ry + s * x + c * y, rz + z

    @staticmethod
    def visible_bottom_height(h: np.ndarray) -> float:
        finite = h[np.isfinite(h)]
        if len(finite) < 20:
            return 0.0
        low = finite[(finite >= -0.50) & (finite <= 1.00)]
        source = low if len(low) >= 20 else finite
        return float(np.percentile(source, 2.0))

    def detect_candidates(
        self,
        local_roi: np.ndarray,
        odom_roi: np.ndarray,
        high_mask: np.ndarray,
        root: tuple[float, float, float, float],
        bottom_h: float,
    ) -> list[Candidate]:
        high_local = local_roi[high_mask]
        comps = connected_components(
            high_local[:, 0],
            high_local[:, 1],
            cell=self.component_cell,
            min_cell_points=2,
        )
        candidates: list[Candidate] = []
        high_indices = np.flatnonzero(high_mask)
        for comp in comps[:12]:
            if len(comp) < self.min_high_points:
                continue
            idx = high_indices[comp]
            cand = self.fit_component(local_roi, odom_roi, idx, root, bottom_h)
            if cand is not None:
                candidates.append(cand)
        candidates.sort(key=lambda item: item.confidence, reverse=True)
        return candidates[: max(1, self.max_candidates)]

    def fit_component(
        self,
        local_roi: np.ndarray,
        odom_roi: np.ndarray,
        indices: np.ndarray,
        root: tuple[float, float, float, float],
        bottom_h: float,
    ) -> Candidate | None:
        pts = local_roi[indices]
        xy = pts[:, :2]
        center0 = xy.mean(axis=0)
        demean = xy - center0
        if len(xy) < 3:
            return None
        try:
            _, _, vh = np.linalg.svd(demean, full_matrices=False)
        except np.linalg.LinAlgError:
            return None
        long_axis = vh[0]
        long_axis /= max(1e-9, float(np.linalg.norm(long_axis)))
        local_yaw = normalize_half_turn(math.atan2(float(long_axis[1]), float(long_axis[0])) - math.pi / 2.0)
        c = math.cos(local_yaw)
        s = math.sin(local_yaw)
        lx = c * (pts[:, 0] - center0[0]) + s * (pts[:, 1] - center0[1])
        ly = -s * (pts[:, 0] - center0[0]) + c * (pts[:, 1] - center0[1])

        support = self.supported_depth_points(lx, ly)
        if support is not None:
            fit_lx, fit_ly, depth_bins = support
        else:
            fit_lx, fit_ly, depth_bins = lx, ly, 0
        width = robust_span(fit_ly)
        depth = robust_span(fit_lx)
        if width < 0.55 or width > 2.50 or depth > 1.15:
            return None

        local_center_x = 0.5 * (float(np.percentile(fit_lx, 5.0)) + float(np.percentile(fit_lx, 95.0)))
        local_center_y = 0.5 * (float(np.percentile(fit_ly, 3.0)) + float(np.percentile(fit_ly, 97.0)))
        center_x = float(center0[0] + c * local_center_x - s * local_center_y)
        center_y = float(center0[1] + s * local_center_x + c * local_center_y)

        all_lx = c * (local_roi[:, 0] - center_x) + s * (local_roi[:, 1] - center_y)
        all_ly = -s * (local_roi[:, 0] - center_x) + c * (local_roi[:, 1] - center_y)
        footprint = (np.abs(all_lx) <= 0.62) & (np.abs(all_ly) <= 1.08)
        if int(footprint.sum()) < 25:
            return None

        plane_band = footprint & (local_roi[:, 2] >= bottom_h - 0.08) & (local_roi[:, 2] <= bottom_h + 0.58)
        plane_h = local_roi[plane_band, 2]
        plane_points = int(len(plane_h))
        if plane_points > 0:
            plane_z = float(np.median(plane_h))
            plane_rmse = float(np.sqrt(np.mean((plane_h - plane_z) ** 2)))
        else:
            plane_z = float(bottom_h)
            plane_rmse = 0.50

        comp_h = pts[:, 2]
        layer_count = count_height_layers(comp_h, bottom_h)
        column_count = self.count_columns(fit_ly)
        center_err = math.hypot(center_x - self.grid_center_x, center_y - self.grid_center_y)

        width_score = math.exp(-abs(width - 1.62) / 0.38)
        depth_score = math.exp(-max(0.0, depth - 0.62) / 0.42)
        layer_score = min(1.0, layer_count / 3.0)
        column_score = min(1.0, column_count / 3.0)
        point_score = min(1.0, len(indices) / 700.0)
        plane_score = min(1.0, plane_points / 150.0) * math.exp(-min(0.25, plane_rmse) / 0.10)
        prior_score = math.exp(-center_err / 0.90)
        confidence = (
            0.18 * point_score
            + 0.20 * width_score
            + 0.10 * depth_score
            + 0.24 * layer_score
            + 0.13 * column_score
            + 0.10 * plane_score
            + 0.05 * prior_score
        )
        odom_x, odom_y, odom_z = self.root_local_to_odom(center_x, center_y, plane_z, root)
        yaw = normalize_angle(root[3] + local_yaw)
        return Candidate(
            center_x=float(odom_x),
            center_y=float(odom_y),
            center_z=float(odom_z),
            local_x=center_x,
            local_y=center_y,
            yaw=float(yaw),
            local_yaw=float(local_yaw),
            points=int(len(indices)),
            confidence=float(max(0.0, min(1.0, confidence))),
            width=float(width),
            depth=float(depth),
            layer_count=int(layer_count),
            column_count=int(column_count),
            plane_points=plane_points,
            plane_rmse=float(plane_rmse),
            depth_support_bins=int(depth_bins),
            center_err=float(center_err),
        )

    def supported_depth_points(self, lx: np.ndarray, ly: np.ndarray) -> tuple[np.ndarray, np.ndarray, int] | None:
        bin_size = 0.06
        bins = np.floor(lx / bin_size).astype(np.int32)
        keep_bins: list[int] = []
        for bin_id in np.unique(bins):
            in_bin = bins == bin_id
            if int(in_bin.sum()) < 8:
                continue
            if robust_span(ly[in_bin]) < 0.55:
                continue
            keep_bins.append(int(bin_id))
        if len(keep_bins) < 3:
            return None
        keep = np.isin(bins, np.asarray(keep_bins, dtype=np.int32))
        if int(keep.sum()) < self.min_high_points:
            return None
        return lx[keep], ly[keep], len(keep_bins)

    @staticmethod
    def count_columns(local_y: np.ndarray) -> int:
        if len(local_y) == 0:
            return 0
        centers = (-0.54, 0.0, 0.54)
        y0 = 0.5 * (float(np.percentile(local_y, 3.0)) + float(np.percentile(local_y, 97.0)))
        return sum(int((np.abs((local_y - y0) - cy) <= 0.24).sum()) >= 12 for cy in centers)

    @staticmethod
    def candidate_to_dict(c: Candidate) -> dict:
        return {
            "center": {"x": c.center_x, "y": c.center_y, "z": c.center_z},
            "local_center": {"x": c.local_x, "y": c.local_y},
            "yaw_deg": math.degrees(c.yaw),
            "local_yaw_deg": math.degrees(c.local_yaw),
            "points": c.points,
            "confidence": c.confidence,
            "width": c.width,
            "depth": c.depth,
            "layers": c.layer_count,
            "columns": c.column_count,
            "plane_points": c.plane_points,
            "plane_rmse": c.plane_rmse,
            "depth_support_bins": c.depth_support_bins,
            "center_err": c.center_err,
        }

    def publish_status(self, status: str, extra: dict) -> None:
        result = {"status": status, **extra}
        self.publish_result(result)
        self.log_result(result)

    def publish_result(self, result: dict) -> None:
        msg = String()
        msg.data = json.dumps(result, ensure_ascii=False, sort_keys=True)
        self.pub_result.publish(msg)

    def publish_markers(self, candidates: list[Candidate], stamp) -> None:
        ma = MarkerArray()
        for i, cand in enumerate(candidates[: self.max_candidates]):
            marker = Marker()
            marker.header.frame_id = self.parent_frame
            marker.header.stamp = stamp
            marker.ns = "depth_grid_plane_probe"
            marker.id = i + 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = cand.center_x
            marker.pose.position.y = cand.center_y
            marker.pose.position.z = cand.center_z + 0.80
            half = 0.5 * cand.yaw
            marker.pose.orientation.z = math.sin(half)
            marker.pose.orientation.w = math.cos(half)
            marker.scale.x = max(0.18, cand.depth)
            marker.scale.y = max(0.18, cand.width)
            marker.scale.z = 0.08
            marker.color.r = 1.0
            marker.color.g = 0.25 + 0.65 * cand.confidence
            marker.color.b = 0.05
            marker.color.a = 0.45 + 0.35 * cand.confidence
            marker.lifetime.sec = 1
            ma.markers.append(marker)

            text = Marker()
            text.header.frame_id = self.parent_frame
            text.header.stamp = stamp
            text.ns = "depth_grid_plane_probe_text"
            text.id = i + 101
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(x=cand.center_x, y=cand.center_y, z=cand.center_z + 1.25)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.18
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 0.95
            text.text = f"{cand.confidence:.2f} n={cand.points} L{cand.layer_count} C{cand.column_count}"
            text.lifetime.sec = 1
            ma.markers.append(text)
        self.pub_marker.publish(ma)

    def log_result(self, result: dict) -> None:
        now = time.monotonic()
        if now - self.last_log < self.log_interval_sec:
            return
        self.last_log = now
        status = result.get("status")
        best = result.get("best")
        counts = result.get("counts", {})
        if best:
            self.get_logger().info(
                "probe "
                f"axis={result.get('cut_axis', '-')} "
                f"conf={best['confidence']:.2f} n={best['points']} "
                f"yaw={best['yaw_deg']:.1f}deg "
                f"w={best['width']:.2f} d={best['depth']:.2f} "
                f"layers={best['layers']} cols={best['columns']} "
                f"counts={counts}"
            )
        else:
            axis = result.get("axis", "-")
            profiles = result.get("profiles", [])
            self.get_logger().info(f"probe status={status} axis={axis} counts={counts} profiles={profiles}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthGridPlaneProbe()
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
