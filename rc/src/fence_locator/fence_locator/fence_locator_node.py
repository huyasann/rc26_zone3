#!/usr/bin/env python3
"""上坡围栏定位节点。"""

from __future__ import annotations

import math
import select
import sys
import termios
import threading
import time
import tty
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker

from .geometry import local_to_odom
from .pointcloud import parse_cloud
from .zone3_corner_detector import Zone3CornerConfig, fit_zone3_corner_from_samples

class FenceLocatorNode(Node):
    def __init__(self) -> None:
        super().__init__("fence_locator")

        # ══ 参数 ══
        self.corridor_near = 0.3
        self.corridor_far = 5.0
        self.corridor_half = 1.5  # 放宽走廊看两侧
        self.y_bin_size = 0.02
        self.x_slice_width = 0.5
        self.sharpness_thr = 0.3  # 降低门槛
        self.ramp_width = 1.55
        self.min_trusted = 1
        self.conf_thr = 0.3  # 降低发布门槛
        # ══ 地面检测参数（借鉴 GroundEstimator）
        self.ground_update_alpha = 0.18
        self.ground_max_step = 0.06
        self.ground_z = None  # 实时地面 Z（EMA 平滑）
        self.flat_ground_z = None  # 平坦阶段锁定的地面 Z（固定基准）
        self.marker_ground_z_locked = None  # marker 专用 Z，只锁一次，避免显示上下跳
        self.field_side = str(self.declare_parameter("field_side", "blue").value).lower()
        if self.field_side not in ("blue", "red", "auto"):
            self.get_logger().warn(f"field_side={self.field_side} 无效，改为 auto")
            self.field_side = "auto"
        self.publish_hz = float(self.declare_parameter("publish_hz", 10.0).value)
        self.publish_low_confidence = bool(self.declare_parameter("publish_low_confidence", True).value)
        self.ramp_pitch_rad = math.radians(float(self.declare_parameter("ramp_pitch_deg", 15.0).value))
        self.ramp_model_length = float(self.declare_parameter("ramp_model_length_m", 1.50).value)
        self.use_odom_pca_yaw = bool(self.declare_parameter("use_odom_pca_yaw", False).value)
        self.min_odom_pca_points = int(self.declare_parameter("min_odom_pca_points", 8).value)
        self.ramp_event_backtrack_sec = float(self.declare_parameter("ramp_event_backtrack_sec", 0.8).value)
        self.enable_cloud_ransac_line = bool(
            self.declare_parameter("enable_cloud_ransac_line", True).value
        )
        self.apply_cloud_ransac_yaw = bool(
            self.declare_parameter("apply_cloud_ransac_yaw", True).value
        )
        self.apply_cloud_ransac_lateral_offset = bool(
            self.declare_parameter("apply_cloud_ransac_lateral_offset", False).value
        )
        self.cloud_ransac_max_lateral_correction = float(
            self.declare_parameter("cloud_ransac_max_lateral_correction_m", 0.45).value
        )
        self.cloud_ransac_max_points = int(
            self.declare_parameter("cloud_ransac_max_points", 60000).value
        )
        self.cloud_ransac_min_edge_points = int(
            self.declare_parameter("cloud_ransac_min_edge_points", 6).value
        )
        self.cloud_ransac_min_inliers = int(
            self.declare_parameter("cloud_ransac_min_inliers", 5).value
        )
        self.cloud_ransac_z_offset = float(
            self.declare_parameter("cloud_ransac_z_offset_m", 0.0).value
        )
        self.update_ramp_yaw_from_odom = bool(
            self.declare_parameter("update_ramp_yaw_from_odom", False).value
        )
        self.update_ramp_length_from_odom = bool(
            self.declare_parameter("update_ramp_length_from_odom", False).value
        )
        self.initial_center_lateral = float(self.declare_parameter("initial_center_lateral_m", 0.0).value)
        self.marker_z_offset = float(self.declare_parameter("marker_z_offset_m", 0.0).value)
        self.anchor_fence_entry_to_start = bool(
            self.declare_parameter("anchor_fence_entry_to_start", True).value
        )
        self.apply_lateral_detection_to_model = bool(
            self.declare_parameter("apply_lateral_detection_to_model", False).value
        )
        self.auto_field_side = bool(self.declare_parameter("auto_field_side", True).value)
        self.publish_fence_top_marker = bool(
            self.declare_parameter("publish_fence_top_marker", False).value
        )
        self.publish_ramp_template = bool(
            self.declare_parameter("publish_ramp_template", False).value
        )
        self.publish_legacy_ramp_markers = bool(
            self.declare_parameter("publish_legacy_ramp_markers", False).value
        )
        self.publish_zone3_debug_markers = bool(
            self.declare_parameter("publish_zone3_debug_markers", True).value
        )
        self.publish_zone3_root_tf = bool(
            self.declare_parameter("publish_zone3_root_tf", True).value
        )
        self.zone3_root_frame = str(
            self.declare_parameter("zone3_root_frame", "").value
        ).strip()
        self.enable_zone3_field_marker = bool(
            self.declare_parameter("publish_zone3_field_marker", False).value
        )
        self.enable_zone3_ransac_refine = bool(
            self.declare_parameter("enable_zone3_ransac_refine", True).value
        )
        self.zone3_ransac_radius = float(
            self.declare_parameter("zone3_ransac_radius_m", 0.85).value
        )
        self.zone3_ransac_dist_thr = float(
            self.declare_parameter("zone3_ransac_dist_thr_m", 0.045).value
        )
        self.zone3_ransac_min_inliers = int(
            self.declare_parameter("zone3_ransac_min_inliers", 35).value
        )
        self.zone3_model_key_x = float(
            self.declare_parameter("zone3_model_key_x_m", 3.000).value
        )
        self.zone3_model_key_y = float(
            self.declare_parameter("zone3_model_key_y_m", -1.400).value
        )
        self.zone3_model_key_z = float(
            self.declare_parameter("zone3_model_key_z_m", 0.45).value
        )
        self.zone3_edge_normal_offset = float(
            self.declare_parameter("zone3_edge_normal_offset_m", 0.0).value
        )
        self.zone3_inner_corner_offset = float(
            self.declare_parameter("zone3_inner_corner_offset_m", 0.0).value
        )
        self.zone3_root_calib_forward = float(
            self.declare_parameter("zone3_root_calib_forward_m", 0.0).value
        )
        self.zone3_root_calib_lateral = float(
            self.declare_parameter("zone3_root_calib_lateral_m", 0.0).value
        )
        self.zone3_root_calib_z = float(
            self.declare_parameter("zone3_root_calib_z_m", 0.0).value
        )
        self.zone3_root_calib_yaw = math.radians(
            float(self.declare_parameter("zone3_root_calib_yaw_deg", 0.0).value)
        )
        self.zone3_field_outer_len = float(
            self.declare_parameter("zone3_field_outer_len_m", 6.05).value
        )
        self.zone3_field_depth_len = float(
            self.declare_parameter("zone3_field_depth_len_m", 2.75).value
        )
        # 来自 zone_detection/field_publisher/rc26_field.py:
        # Z3_MAIN_SIZE=[4.5,2.60], Z3_SIDE_SIZE=[1.55,1.25],
        # Z3_CARPET_EXT_SIZE=[1.55,2.75]。当前先用角点坐标系画三区外轮廓。
        self.zone3_model_side_len = float(
            self.declare_parameter("zone3_model_side_len_m", 1.55).value
        )
        self.zone3_model_main_len = float(
            self.declare_parameter("zone3_model_main_len_m", 4.5).value
        )
        self.zone3_model_main_depth = float(
            self.declare_parameter("zone3_model_main_depth_m", 2.60).value
        )
        self.zone3_model_side_depth = float(
            self.declare_parameter("zone3_model_side_depth_m", 2.75).value
        )
        self.enable_zone3_yaw_refine = bool(
            self.declare_parameter("enable_zone3_yaw_refine", False).value
        )
        self.zone3_yaw_refine_range_deg = float(
            self.declare_parameter("zone3_yaw_refine_range_deg", 3.0).value
        )
        self.zone3_yaw_refine_step_deg = float(
            self.declare_parameter("zone3_yaw_refine_step_deg", 0.15).value
        )
        self.zone3_post_platform_cloud_frames = int(
            self.declare_parameter("zone3_post_platform_cloud_frames", 12).value
        )
        self.enable_zone3_inside_refine = bool(
            self.declare_parameter("enable_zone3_inside_refine", True).value
        )
        self.zone3_inside_refine_xy_range = float(
            self.declare_parameter("zone3_inside_refine_xy_range_m", 0.08).value
        )
        self.zone3_inside_refine_xy_step = float(
            self.declare_parameter("zone3_inside_refine_xy_step_m", 0.02).value
        )
        self.zone3_inside_refine_yaw_range = math.radians(
            float(self.declare_parameter("zone3_inside_refine_yaw_range_deg", 0.6).value)
        )
        self.zone3_inside_refine_yaw_step = math.radians(
            float(self.declare_parameter("zone3_inside_refine_yaw_step_deg", 0.2).value)
        )
        self.zone3_inside_refine_min_outside_ratio = float(
            self.declare_parameter("zone3_inside_refine_min_outside_ratio", 0.08).value
        )
        self.zone3_inside_refine_min_outside_points = int(
            self.declare_parameter("zone3_inside_refine_min_outside_points", 60).value
        )
        self.entry_forward_source = str(
            self.declare_parameter("entry_forward_source", "odom_start").value
        ).lower()

        self.state = "flat"
        self.last_state_pub: str | None = None
        self.enabled = False
        self.lateral_detection_enabled = False
        self.finalized = False

        self.lateral_ests: list[float] = []
        self.trusted_count = 0
        self.two_side_count = 0
        self.positive_side_count = 0
        self.negative_side_count = 0
        self.inferred_field_side = self.field_side if self.field_side in ("blue", "red") else "unknown"
        self.lateral_locked: float | None = None
        self.confidence = 0.0

        self.cloud_seen = 0
        self.cloud_analyzed = 0
        self.last_pose: tuple[float, float, float, float] | None = None
        self.locked_pose: tuple[float, float, float, float] | None = None
        self.ramp_start_pose: tuple[float, float, float, float] | None = None
        self.ramp_end_pose: tuple[float, float, float, float] | None = None
        self.pending_transition_pose: tuple[float, float, float, float, float] | None = None
        self.last_corridor_points = 0
        self.last_candidate_peaks = 0
        self.last_update_time = None
        self.odom_history: deque[tuple[float, float, float, float, float]] = deque(maxlen=1200)
        self.ramp_odom_points: list[tuple[float, float, float, float, float]] = []
        self.collect_ramp_odom = False
        self.ramp_start_time: float | None = None
        self.ramp_axis_yaw: float | None = None
        self.ramp_pca_residual: float | None = None
        self.ramp_odom_length: float | None = None
        self.cloud_samples: list[np.ndarray] = []
        self.cloud_sample_points = 0
        self.cloud_ransac_line: dict | None = None
        self.cloud_ransac_reason = "not_run"
        self.cloud_lateral_correction = 0.0
        self.bottom_corner: dict | None = None
        self.bottom_corner_reason = "not_run"
        self.zone3_corner: dict | None = None
        self.zone3_corner_reason = "not_run"
        self.zone3_yaw_refine_delta = 0.0
        self.zone3_yaw_refine_score = 0.0
        self.zone3_inside_score = 0.0
        self.zone3_inside_points = 0
        self.zone3_outside_points = 0
        self.zone3_outside_ratio = 1.0
        self.zone3_inside_refine_dx = 0.0
        self.zone3_inside_refine_dy = 0.0
        self.zone3_inside_refine_dyaw = 0.0

        self.stop_keyboard = False
        self.shutdown_requested = False
        self.shutting_down = False
        self.marker_log_once = False
        self.marker_clear_once = False
        self.top_marker_delete_once = False
        self.template_marker_delete_once = False
        self.collection_log_once = False
        self.collection_reset_done = False
        self.zone3_root_tf_log_once = False
        self.zone3_collect_after_platform = False
        self.zone3_platform_cloud_frames = 0

        self.create_subscription(String, "/uphill/state", self.on_state, 10)
        self.create_subscription(PoseStamped, "/uphill/transition_pose", self.on_transition_pose, 10)
        self.create_subscription(Odometry, "/odin1/odometry_highfreq", self.on_odom, 50)
        self.create_subscription(PointCloud2, "/odin1/cloud_slam", self.on_cloud, 10)

        self.pub_offset = self.create_publisher(Float32, "/ramp/lateral_offset", 10)
        self.pub_conf = self.create_publisher(Float32, "/ramp/confidence", 10)
        self.pub_side = self.create_publisher(String, "/ramp/inferred_field_side", 10)
        marker_qos = QoSProfile(depth=10)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_marker = self.create_publisher(Marker, "/ramp/model_marker", marker_qos)
        self.tf_br = TransformBroadcaster(self)
        self.static_tf_br = StaticTransformBroadcaster(self)
        self.publish_timer = self.create_timer(1.0 / max(0.1, self.publish_hz), self.publish)
        self.startup_clear_timer = self.create_timer(0.5, self.clear_startup_markers)

        self.keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self.keyboard_thread.start()

        self.get_logger().info(
            f"fence_locator ready, field_side={self.field_side}, "
            f"use_odom_pca_yaw={self.use_odom_pca_yaw}, "
            f"update_ramp_yaw_from_odom={self.update_ramp_yaw_from_odom}"
        )

    def clear_startup_markers(self) -> None:
        if self.marker_clear_once:
            try:
                self.startup_clear_timer.cancel()
            except Exception:
                pass
            return
        self.marker_clear_once = True
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ramp_model"
        marker.id = 0
        marker.action = Marker.DELETEALL
        self.pub_marker.publish(marker)
        try:
            self.startup_clear_timer.cancel()
        except Exception:
            pass
        self.get_logger().info("cleared stale /ramp/model_marker markers")

    def on_odom(self, msg: Odometry) -> None:
        stamp = msg.header.stamp
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.last_pose = (float(pos.x), float(pos.y), float(pos.z), yaw)
        sample = (t, float(pos.x), float(pos.y), float(pos.z), yaw)
        self.odom_history.append(sample)
        if self.collect_ramp_odom:
            self.ramp_odom_points.append(sample)

    def on_transition_pose(self, msg: PoseStamped) -> None:
        stamp = msg.header.stamp
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        pos = msg.pose.position
        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.pending_transition_pose = (t, float(pos.x), float(pos.y), float(pos.z), yaw)

    def _reset_zone3_cloud_cache(self, reason: str) -> None:
        self.cloud_samples.clear()
        self.cloud_sample_points = 0
        self.cloud_ransac_line = None
        self.cloud_ransac_reason = reason
        self.bottom_corner = None
        self.bottom_corner_reason = reason
        self.zone3_corner = None
        self.zone3_corner_reason = reason
        self.cloud_lateral_correction = 0.0
        self.zone3_root_tf_log_once = False
        self.zone3_yaw_refine_delta = 0.0
        self.zone3_yaw_refine_score = 0.0
        self.zone3_inside_score = 0.0
        self.zone3_inside_points = 0
        self.zone3_outside_points = 0
        self.zone3_outside_ratio = 1.0
        self.zone3_inside_refine_dx = 0.0
        self.zone3_inside_refine_dy = 0.0
        self.zone3_inside_refine_dyaw = 0.0

    def _finish_platform_cloud_collection(self) -> None:
        if self.finalized:
            return
        self.zone3_collect_after_platform = False
        self.enabled = False
        self.lateral_detection_enabled = False
        self.finalized = True
        self.collection_reset_done = False
        self.get_logger().info(
            "stop post-platform point cloud collection, "
            f"frames={self.zone3_platform_cloud_frames}, "
            f"samples={self.cloud_sample_points}, "
            f"cloud={self.cloud_seen}, analyzed={self.cloud_analyzed}"
        )
        self.fuse_and_publish()

    def on_state(self, msg: String) -> None:
        new_state = msg.data.strip()
        if new_state == self.last_state_pub:
            return
        self.last_state_pub = new_state
        self.state = new_state

        if self.state == "flat_to_uphill":
            if self.collection_reset_done:
                return
            # 锁定地面 Z（只用点云直方图检测，不用硬编码偏移）
            if self.ground_z is not None:
                self.flat_ground_z = self.ground_z
                self.marker_ground_z_locked = self.flat_ground_z
                self.get_logger().info(f"flat ground Z locked from point cloud: {self.flat_ground_z:.3f}m")
            else:
                self.get_logger().warn("flat_to_uphill but ground_z not yet available, will lock on next cloud frame")
            start_pose = self._take_transition_start_pose() or self.last_pose
            if start_pose is not None:
                self.ramp_start_pose = start_pose
                self.ramp_end_pose = None
                self.locked_pose = self.ramp_start_pose
                self.lateral_locked = None
                self.confidence = 0.0
                sx, sy, sz, syaw = self.ramp_start_pose
                self.get_logger().info(
                    f"ramp start fixed: x={sx:.3f}, y={sy:.3f}, z={sz:.3f}, yaw={math.degrees(syaw):.2f}deg"
                )
            self.ramp_start_time = (
                self.pending_transition_pose[0]
                if self.pending_transition_pose is not None
                else (self.odom_history[-1][0] if self.odom_history else None)
            )
            self.ramp_odom_points = self._backtracked_odom_points()
            self.collect_ramp_odom = True
            self._reset_zone3_cloud_cache("reset")
            self.zone3_collect_after_platform = False
            self.zone3_platform_cloud_frames = 0
            self.collection_reset_done = True
            self.lateral_ests.clear()
            self.trusted_count = 0
            self.two_side_count = 0
            self.positive_side_count = 0
            self.negative_side_count = 0
            self.inferred_field_side = self.field_side if self.field_side in ("blue", "red") else "unknown"
            self.cloud_seen = 0
            self.cloud_analyzed = 0
            self.last_corridor_points = 0
            self.last_candidate_peaks = 0
            self.enabled = True
            self.lateral_detection_enabled = False
            self.finalized = False
            self.collection_log_once = False
            self.get_logger().info("waiting for platform before zone3 point cloud collection")
        elif self.state == "uphill":
            if not self.collection_reset_done:
                self._start_collection_from_transition("confirmed ramp")
                self._reset_zone3_cloud_cache("reset_before_platform")
            self.enabled = True
            self.lateral_detection_enabled = False
            if not self.collection_log_once:
                self.collection_log_once = True
                self.get_logger().info("ramp confirmed, zone3 point cloud collection still waiting for platform")
        elif self.state == "uphill_to_platform":
            self.enabled = True
            self.lateral_detection_enabled = False
        elif self.state == "platform":
            if self.finalized:
                return
            self.collect_ramp_odom = False
            if self.last_pose is not None:
                self.ramp_end_pose = self.last_pose
                self._update_fixed_ramp_from_end()
            self.finalized = True
            self.collection_reset_done = False
            self.enabled = False
            self.lateral_detection_enabled = False
            self._reset_zone3_cloud_cache("post_platform_reset")
            self.zone3_collect_after_platform = True
            self.zone3_platform_cloud_frames = 0
            self.enabled = True
            self.lateral_detection_enabled = False
            self.finalized = False
            self.get_logger().info(
                f"start post-platform point cloud collection, target_frames={self.zone3_post_platform_cloud_frames}"
            )

    def _start_collection_from_transition(self, reason: str) -> None:
        if self.ground_z is not None:
            self.flat_ground_z = self.ground_z
            self.marker_ground_z_locked = self.flat_ground_z
            self.get_logger().info(f"flat ground Z locked from point cloud: {self.flat_ground_z:.3f}m")
        else:
            self.get_logger().warn(f"{reason}: ground_z not yet available, will lock on next cloud frame")

        start_pose = self._take_transition_start_pose() or self.last_pose
        if start_pose is not None:
            self.ramp_start_pose = start_pose
            self.ramp_end_pose = None
            self.locked_pose = self.ramp_start_pose
            self.lateral_locked = None
            self.confidence = 0.0
            sx, sy, sz, syaw = self.ramp_start_pose
            self.get_logger().info(
                f"ramp start fixed: x={sx:.3f}, y={sy:.3f}, z={sz:.3f}, yaw={math.degrees(syaw):.2f}deg"
            )

        self.ramp_start_time = (
            self.pending_transition_pose[0]
            if self.pending_transition_pose is not None
            else (self.odom_history[-1][0] if self.odom_history else None)
        )
        self.ramp_odom_points = self._backtracked_odom_points()
        self.collect_ramp_odom = True
        self.cloud_samples.clear()
        self.cloud_sample_points = 0
        self.cloud_ransac_line = None
        self.cloud_ransac_reason = "reset"
        self.bottom_corner = None
        self.bottom_corner_reason = "reset"
        self.zone3_corner = None
        self.zone3_corner_reason = "reset"
        self.cloud_lateral_correction = 0.0
        self.zone3_root_tf_log_once = False
        self.collection_reset_done = True
        self.lateral_ests.clear()
        self.trusted_count = 0
        self.two_side_count = 0
        self.positive_side_count = 0
        self.negative_side_count = 0
        self.inferred_field_side = self.field_side if self.field_side in ("blue", "red") else "unknown"
        self.cloud_seen = 0
        self.cloud_analyzed = 0
        self.last_corridor_points = 0
        self.last_candidate_peaks = 0
        self.enabled = True
        self.lateral_detection_enabled = False
        self.finalized = False
        self.collection_log_once = False
        self.get_logger().info(f"prepared ramp pose from {reason}, waiting for platform before zone3 cloud collection")

    def _take_transition_start_pose(self) -> tuple[float, float, float, float] | None:
        if self.pending_transition_pose is None:
            return None
        _, x, y, z, yaw = self.pending_transition_pose
        return (x, y, z, yaw)

    def on_cloud(self, msg: PointCloud2) -> None:
        if self.shutting_down or not self.enabled:
            return
        self.cloud_seen += 1

        pts = parse_cloud(msg)
        if len(pts) == 0:
            return

        x, y, z = pts["x"], pts["y"], pts["z"]
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        if valid.sum() == 0:
            return

        self.cloud_analyzed += 1
        xv = x[valid]
        yv = y[valid]
        zv = z[valid]
        if self.zone3_collect_after_platform:
            self._cache_cloud_points(xv, yv, zv)
            self.zone3_platform_cloud_frames += 1
            if self.zone3_platform_cloud_frames >= max(1, self.zone3_post_platform_cloud_frames):
                self._finish_platform_cloud_collection()
            return
        if not self.lateral_detection_enabled:
            return
        result = self.detect_fence_single(xv, yv, zv)
        if result is None:
            return

        est, visible_side = result
        self.vote_field_side(visible_side)
        if est is not None:
            raw_est = float(est)
            self.lateral_ests.append(raw_est)
            self.update_model(raw_est, confidence=0.30)

    def vote_field_side(self, visible_side: str) -> None:
        if visible_side == "positive":
            self.positive_side_count += 1
        elif visible_side == "negative":
            self.negative_side_count += 1
        else:
            return

        if not self.auto_field_side:
            return
        if self.field_side in ("blue", "red"):
            self.inferred_field_side = self.field_side
            return

        old_side = self.inferred_field_side
        if self.positive_side_count >= self.negative_side_count + 3:
            self.inferred_field_side = "blue"
        elif self.negative_side_count >= self.positive_side_count + 3:
            self.inferred_field_side = "red"

        if self.inferred_field_side != old_side:
            self.get_logger().info(
                f"auto field side: {self.inferred_field_side} "
                f"(positive={self.positive_side_count}, negative={self.negative_side_count})"
            )

    def publish_inferred_side(self) -> None:
        msg = String()
        msg.data = self.inferred_field_side
        self.pub_side.publish(msg)

    def update_model(self, lateral: float, confidence: float) -> None:
        self.lateral_locked = lateral if self.apply_lateral_detection_to_model else self.initial_center_lateral
        self.confidence = max(self.confidence, confidence)
        self.locked_pose = self.ramp_start_pose or self.last_pose
        self.last_update_time = self.get_clock().now()

    def _update_fixed_ramp_from_end(self) -> None:
        if self.ramp_start_pose is None or self.ramp_end_pose is None:
            return
        sx, sy, sz, syaw = self.ramp_start_pose
        ex, ey, ez, _ = self.ramp_end_pose
        dx = ex - sx
        dy = ey - sy
        length = math.hypot(dx, dy)
        if length < 0.20:
            return
        self.ramp_odom_length = length
        odom_yaw = math.atan2(dy, dx)
        model_yaw = syaw
        yaw_source = "start_pose"
        pca = self._fit_ramp_yaw_from_odom()
        if self.use_odom_pca_yaw and pca is not None:
            odom_yaw, residual = pca
            self.ramp_axis_yaw = odom_yaw
            self.ramp_pca_residual = residual
            yaw_source = "pca"
        if self.update_ramp_yaw_from_odom:
            model_yaw = odom_yaw
            self.ramp_start_pose = (sx, sy, sz, model_yaw)
            if self.lateral_locked is not None:
                self.locked_pose = self.ramp_start_pose
        if self.update_ramp_length_from_odom:
            self.ramp_model_length = length
            if self.lateral_locked is not None:
                self.locked_pose = self.ramp_start_pose
        self.get_logger().info(
            f"ramp end fixed: x={ex:.3f}, y={ey:.3f}, z={ez:.3f}, "
            f"model_yaw={math.degrees(model_yaw):.2f}deg, odom_yaw={math.degrees(odom_yaw):.2f}deg, "
            f"odom_length={length:.3f}m, "
            f"model_length={self.ramp_model_length:.3f}m, "
            f"pitch={math.degrees(self.ramp_pitch_rad):.1f}deg, "
            f"update_yaw={self.update_ramp_yaw_from_odom}, "
            f"update_from_odom={self.update_ramp_length_from_odom}, "
            f"yaw_source={yaw_source}, pca_residual={self.ramp_pca_residual if self.ramp_pca_residual is not None else float('nan'):.3f}"
        )

    def _backtracked_odom_points(self) -> list[tuple[float, float, float, float, float]]:
        if not self.odom_history:
            return []
        now = self.odom_history[-1][0]
        start = now - self.ramp_event_backtrack_sec
        return [item for item in self.odom_history if item[0] >= start]

    def _fit_ramp_yaw_from_odom(self) -> tuple[float, float] | None:
        if len(self.ramp_odom_points) < self.min_odom_pca_points:
            return None
        pts = np.asarray([[p[1], p[2], p[3]] for p in self.ramp_odom_points], dtype=np.float64)
        z = pts[:, 2]
        z_lo = float(np.percentile(z, 8.0))
        z_hi = float(np.percentile(z, 92.0))
        if z_hi - z_lo > 0.10:
            mask = (z >= z_lo + 0.03) & (z <= z_hi - 0.01)
            if int(mask.sum()) >= self.min_odom_pca_points:
                pts = pts[mask]
        xy = pts[:, :2]
        center = xy.mean(axis=0)
        _, _, vh = np.linalg.svd(xy - center, full_matrices=False)
        axis = vh[0]
        start = np.asarray([self.ramp_odom_points[0][1], self.ramp_odom_points[0][2]], dtype=np.float64)
        end = np.asarray([self.ramp_odom_points[-1][1], self.ramp_odom_points[-1][2]], dtype=np.float64)
        if float(np.dot(axis, end - start)) < 0.0:
            axis = -axis
        proj = (xy - center) @ axis
        perp = (xy - center) @ np.asarray([-axis[1], axis[0]])
        if float(proj.max() - proj.min()) < 0.25:
            return None
        yaw = math.atan2(float(axis[1]), float(axis[0]))
        residual = float(np.sqrt(np.mean(perp**2)))
        return yaw, residual

    def detect_fence_single(self, x: np.ndarray, y: np.ndarray, z: np.ndarray) -> tuple[float, str] | None:
        if self.last_pose is None:
            return None

        robot_x, robot_y, _, robot_yaw = self.last_pose
        dx = x - robot_x
        dy = y - robot_y

        forward = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
        lateral = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)

        in_corridor = (
            (forward >= self.corridor_near)
            & (forward <= self.corridor_far)
            & (np.abs(lateral) <= self.corridor_half)
        )
        self.last_corridor_points = int(in_corridor.sum())
        if self.last_corridor_points < 50:
            self.last_candidate_peaks = 0
            return None

        cx = forward[in_corridor]
        cy = lateral[in_corridor]
        cz = z[in_corridor]

        # 更新全局地面高度（直方图 + EMA，借鉴 GroundEstimator）
        self._update_ground_z(cz)

        x_lo = float(cx.min())
        x_hi = float(cx.max())
        n_slices = max(3, int((x_hi - x_lo) / self.x_slice_width))
        fence_lines: list[tuple[float, int, float]] = []

        for i in range(n_slices):
            xl = x_lo + i * (x_hi - x_lo) / n_slices
            xr = x_lo + (i + 1) * (x_hi - x_lo) / n_slices
            mask = (cx >= xl) & (cx < xr)
            if mask.sum() < 30:
                continue

            # 使用 EMA 平滑的全局地面 Z
            local_ground = self.ground_z if self.ground_z is not None else np.percentile(cz[mask], 5)
            fence_mask = mask & (cz >= local_ground) & (cz <= local_ground + 0.15)
            if fence_mask.sum() < 10:
                continue

            sy = cy[fence_mask]
            width = max(0.1, float(sy.max() - sy.min()))
            bins = max(30, int(width / self.y_bin_size))
            hist, edges = np.histogram(sy, bins=bins)
            centers = (edges[:-1] + edges[1:]) / 2.0

            for peak_i in np.argsort(hist)[-2:][::-1]:
                peak_y = float(centers[peak_i])
                near_peak = np.abs(sy - peak_y) < 0.1
                sharpness = float(hist[peak_i]) / max(1, int(near_peak.sum()))
                if sharpness >= self.sharpness_thr and hist[peak_i] >= 5:
                    fence_lines.append((peak_y, int(hist[peak_i]), sharpness))

        self.last_candidate_peaks = len(fence_lines)
        if not fence_lines:
            return None

        lefts = [f for f in fence_lines if f[0] < 0.0]
        rights = [f for f in fence_lines if f[0] > 0.0]
        best_l = max(lefts, key=lambda f: f[1]) if lefts else None
        best_r = max(rights, key=lambda f: f[1]) if rights else None

        if best_l and best_r:
            self.trusted_count += 1
            self.two_side_count += 1
            return (best_l[0] + best_r[0]) / 2.0, "both"
        if best_l:
            self.trusted_count += 1
            return best_l[0] + self.ramp_width / 2.0, "negative"
        if best_r:
            self.trusted_count += 1
            return best_r[0] - self.ramp_width / 2.0, "positive"
        return None

    def _update_ground_z(self, z: np.ndarray) -> None:
        """直方图检测地面 + EMA 平滑（借鉴 GroundEstimator）。
        
        使用最低峰检测：找到直方图中第一个达到峰值 15% 的 bin。
        围栏比地面高 10cm，不会影响最低峰的检测。
        """
        if len(z) == 0:
            return
        hist, edges = np.histogram(z, bins=50)
        peak = hist.max()
        if peak <= 0:
            return
        # 找第一个达到峰值 15% 的 bin（最低地面峰）
        threshold = peak * 0.15
        for i, count in enumerate(hist):
            if count >= threshold:
                detected = float((edges[i] + edges[i + 1]) / 2.0)
                break
        else:
            detected = float(edges[np.argmax(hist)])

        # 如果 flat_ground_z 还没设置，用第一个估计值作为基准
        if self.flat_ground_z is None:
            self.flat_ground_z = detected
            self.get_logger().info(f"flat ground Z initialized: {self.flat_ground_z:.3f}m")

        if self.ground_z is None:
            self.ground_z = detected
        else:
            step = detected - self.ground_z
            if abs(step) <= self.ground_max_step:
                self.ground_z += self.ground_update_alpha * step

    def _marker_ground_z(self, fallback_z: float) -> float:
        if self.marker_ground_z_locked is not None:
            return float(self.marker_ground_z_locked)
        if self.flat_ground_z is not None:
            self.marker_ground_z_locked = float(self.flat_ground_z)
            return float(self.marker_ground_z_locked)
        self.marker_ground_z_locked = float(fallback_z)
        return float(self.marker_ground_z_locked)

    def fuse_and_publish(self) -> None:
        self._fit_zone3_corner_from_cloud()
        if self.zone3_corner is not None:
            self.publish_zone3_root_transform()
            if self.locked_pose is not None:
                base_x, base_y, base_z, model_yaw = self.locked_pose
                self.publish_markers(base_x, base_y, base_z, model_yaw)
            corner = self.zone3_corner
            self.get_logger().info(
                f"zone3 root fit: corner=({corner['corner_x']:.3f},{corner['corner_y']:.3f}), "
                f"angle={corner['angle_deg']:.1f}deg, "
                f"vertical_n={corner['vertical_count']}, vertical_span={corner['vertical_span']:.3f}m, "
                f"outer_rmse={corner['outer_rmse']:.3f}, far_rmse={corner['far_rmse']:.3f}, "
                f"refined={corner.get('refined', False)}, source={corner.get('ransac_source', '-')}, "
                f"shift={corner.get('refine_shift', 0.0):.3f}m, "
                f"yaw_weight far/outer={self._zone3_line_weight('far'):.1f}/{self._zone3_line_weight('outer'):.1f}, "
                f"yaw_refine={math.degrees(self.zone3_yaw_refine_delta):+.2f}deg, "
                f"edge_offset={self.zone3_edge_normal_offset:+.3f}m, "
                f"root_calib=({self.zone3_root_calib_forward:+.3f},"
                f"{self.zone3_root_calib_lateral:+.3f},"
                f"{self.zone3_root_calib_z:+.3f},"
                f"{math.degrees(self.zone3_root_calib_yaw):+.2f}deg), "
                f"inside={self.zone3_inside_points}, outside={self.zone3_outside_points}, "
                f"out_ratio={self.zone3_outside_ratio:.2f}, "
                f"inside_refine=({self.zone3_inside_refine_dx:+.3f},"
                f"{self.zone3_inside_refine_dy:+.3f},"
                f"{math.degrees(self.zone3_inside_refine_dyaw):+.2f}deg), "
                f"outer_inliers={corner.get('outer_bins', 0)}, far_inliers={corner.get('far_bins', 0)}, "
                f"side={corner['side']}"
            )
        else:
            self.get_logger().info(f"zone3 root fit: none, reason={self.zone3_corner_reason}")

        if len(self.lateral_ests) < self.min_trusted:
            self.get_logger().warn(f"可信帧不足 ({len(self.lateral_ests)}/{self.min_trusted})")
            return

        vals = np.array(self.lateral_ests)
        spread = float(np.std(vals)) if len(vals) > 1 else 0.0
        v_min = float(vals.min())
        v_med = float(np.median(vals))
        v_max = float(vals.max())
        if len(vals) == 1 or abs(float(vals.max() - vals.min())) < 1e-6:
            center = float(vals.mean())
            in_peak = len(vals)
        else:
            bins = max(20, int((vals.max() - vals.min()) / 0.02))
            hist, edges = np.histogram(vals, bins=bins)
            peak_i = int(np.argmax(hist))
            center = float((edges[peak_i] + edges[peak_i + 1]) / 2.0)
            in_peak = int((np.abs(vals - center) < 0.05).sum())

        two_side_ratio = self.two_side_count / max(1, len(vals))
        side_factor = 0.45 + 0.55 * two_side_ratio
        self.confidence = min(1.0, in_peak / len(vals) * side_factor)
        self.update_model(center, self.confidence)
        self._fit_bottom_corner_from_cloud()
        self._fit_cloud_ransac_line()
        if (
            self.apply_cloud_ransac_lateral_offset
            and self.cloud_ransac_line is not None
            and self.cloud_ransac_line["inliers"] >= self.cloud_ransac_min_inliers
            and abs(float(self.cloud_ransac_line["center_delta"])) <= self.cloud_ransac_max_lateral_correction
        ):
            self.cloud_lateral_correction = float(self.cloud_ransac_line["center_delta"])
            self.update_model(center + self.cloud_lateral_correction, self.confidence)
        if self.apply_cloud_ransac_yaw and self._cloud_ransac_line_is_trusted() and self.ramp_start_pose is not None:
            sx, sy, sz, _ = self.ramp_start_pose
            self.ramp_start_pose = (sx, sy, sz, float(self.cloud_ransac_line["yaw"]))
            self.locked_pose = self.ramp_start_pose

        self.get_logger().info(
            f"locked: center_lateral={center:.3f}m, confidence={self.confidence:.2f}, "
            f"frames={len(vals)}, two_side={self.two_side_count}, "
            f"min/median/max={v_min:.3f}/{v_med:.3f}/{v_max:.3f}, std={spread:.3f}, "
            f"side={self.inferred_field_side}, positive/negative={self.positive_side_count}/{self.negative_side_count}"
        )
        if self.cloud_ransac_line is not None:
            line = self.cloud_ransac_line
            self.get_logger().info(
                f"cloud RANSAC edge: yaw={math.degrees(line['yaw']):.2f}deg, "
                f"local_slope={line['slope']:.3f}, inliers={line['inliers']}/{line['edge_points']}, "
                f"rmse={line['rmse']:.3f}, z_rmse={line['z_rmse']:.3f}, side={line['side']}, "
                f"center_delta={line['center_delta']:+.3f}, "
                f"corner=({line['corner_x']:.3f},{line['corner_y']:.3f})"
            )
        else:
            self.get_logger().info(f"cloud RANSAC edge: none, reason={self.cloud_ransac_reason}")
        if self.bottom_corner is not None:
            corner = self.bottom_corner
            self.get_logger().info(
                f"bottom corner: forward={corner['forward']:.3f}m, "
                f"fence_lateral={corner['fence_lateral']:.3f}m, "
                f"fence_lateral_end={corner.get('fence_lateral_end', corner['fence_lateral']):.3f}m, "
                f"ground_z={corner['ground_z']:.3f}m, "
                f"mode={corner.get('mode', 'profile_intersection')}, "
                f"side_k={corner.get('side_k', float('nan')):.3f}, "
                f"side_b={corner.get('side_b', float('nan')):.3f}, "
                f"side_rmse={corner.get('side_rmse', float('nan')):.3f}, "
                f"ramp_slope={corner['ramp_slope']:.3f}, "
                f"ramp_rmse={corner['ramp_rmse']:.3f}, ground_rmse={corner['ground_rmse']:.3f}"
            )
        else:
            self.get_logger().info(f"bottom corner: none, reason={self.bottom_corner_reason}")
        if self.zone3_corner is not None:
            corner = self.zone3_corner
            self.get_logger().info(
                f"zone3 XYZ corner: xy=({corner['corner_x']:.3f},{corner['corner_y']:.3f}), "
                f"angle={corner['angle_deg']:.1f}deg, "
                f"vertical_n={corner['vertical_count']}, vertical_span={corner['vertical_span']:.3f}m, "
                f"outer_rmse={corner['outer_rmse']:.3f}, far_rmse={corner['far_rmse']:.3f}, "
                f"refined={corner.get('refined', False)}, "
                f"outer_inliers={corner.get('outer_bins', 0)}, far_inliers={corner.get('far_bins', 0)}, "
                f"side={corner['side']}"
            )
        else:
            self.get_logger().info(f"zone3 XYZ corner: none, reason={self.zone3_corner_reason}")
        self.publish()

    def _cache_cloud_points(self, x: np.ndarray, y: np.ndarray, z: np.ndarray) -> None:
        if not self.enable_cloud_ransac_line:
            return
        n = len(z)
        if n == 0:
            return
        step = max(1, n // 2500)
        sample = np.column_stack((x[::step], y[::step], z[::step])).astype(np.float64, copy=False)
        self.cloud_samples.append(sample)
        self.cloud_sample_points += len(sample)
        while self.cloud_samples and self.cloud_sample_points > self.cloud_ransac_max_points:
            removed = self.cloud_samples.pop(0)
            self.cloud_sample_points -= len(removed)

    def _cloud_ransac_line_is_trusted(self) -> bool:
        if self.cloud_ransac_line is None:
            return False
        line = self.cloud_ransac_line
        return (
            int(line["edge_points"]) >= self.cloud_ransac_min_edge_points
            and int(line["inliers"]) >= self.cloud_ransac_min_inliers
            and float(line["rmse"]) <= 0.08
            and float(line["span"]) >= 0.65
            and abs(float(line["yaw_delta"])) <= math.radians(12.0)
        )

    def _fit_bottom_corner_from_cloud(self) -> None:
        if not self.cloud_samples:
            self.bottom_corner_reason = "no_samples"
            return
        pose = self.locked_pose or self.ramp_start_pose
        if pose is None:
            self.bottom_corner_reason = "no_pose"
            return
        base_x, base_y, base_z, model_yaw = pose
        pts = np.concatenate(self.cloud_samples, axis=0)
        if len(pts) < 300:
            self.bottom_corner_reason = f"few_cloud_samples:{len(pts)}"
            return
        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]
        dx = x - base_x
        dy = y - base_y
        forward = dx * math.cos(model_yaw) + dy * math.sin(model_yaw)
        lateral = -dx * math.sin(model_yaw) + dy * math.cos(model_yaw)

        roi = (
            (forward >= -0.55)
            & (forward <= self.ramp_model_length + 0.45)
            & (np.abs(lateral) <= self.ramp_width * 0.75)
            & (z >= base_z - 0.35)
            & (z <= base_z + 0.65)
        )
        if int(roi.sum()) < 120:
            self.bottom_corner_reason = f"few_roi:{int(roi.sum())}"
            return
        f = forward[roi]
        l = lateral[roi]
        zr = z[roi]
        ground_z0 = self._detect_low_z_peak(zr)
        if ground_z0 is None:
            self.bottom_corner_reason = "ground_peak_failed"
            return

        ground_mask = (zr >= ground_z0 - 0.045) & (zr <= ground_z0 + 0.075) & (f <= 0.70)
        ramp_mask = (zr >= ground_z0 + 0.055) & (zr <= ground_z0 + 0.50) & (f >= -0.10)
        if int(ground_mask.sum()) < 25 and int(ramp_mask.sum()) >= 80:
            if self.ramp_odom_length is not None:
                ramp_fallback_forward = max(0.0, float(self.ramp_odom_length - self.ramp_model_length))
                fallback_mode = "odom_end_minus_ramp_length"
            else:
                ramp_fallback_forward = max(0.0, float(np.percentile(f[ramp_mask], 2.0) - 0.06))
                fallback_mode = "ramp_front_fallback"
            side_fit_forward = ramp_fallback_forward
            if self.entry_forward_source == "odom_start":
                ramp_fallback_forward = 0.0
                fallback_mode = "odom_start_forward"
            side = self._active_side()
            side_fit = self._fit_side_edge_from_cloud(f, l, zr, side_fit_forward, side)
            cloud_forward = self._estimate_entry_forward_from_cloud(f, l, zr, side_fit_forward, side)
            corner_forward = cloud_forward if cloud_forward is not None else ramp_fallback_forward
            if cloud_forward is not None:
                fallback_mode = "cloud_entry_forward"
            visible_forward = None
            visible_lateral = None
            if side_fit is None:
                fence_lateral = (
                    self.initial_center_lateral - self.ramp_width * 0.5
                    if side == "red"
                    else self.initial_center_lateral + self.ramp_width * 0.5
                )
                fence_lateral_end = fence_lateral
                side_k = 0.0
                side_b = fence_lateral
                side_rmse = float("nan")
            else:
                side_k, side_b, side_rmse = side_fit
                fence_lateral = float(side_k * corner_forward + side_b)
                fence_lateral_end = float(side_k * (corner_forward + self.ramp_model_length) + side_b)
                visible_forward = self._first_visible_side_forward(f, l, zr, corner_forward, side, side_k, side_b)
                if visible_forward is not None:
                    visible_lateral = float(side_k * visible_forward + side_b)
            self.bottom_corner = {
                "forward": corner_forward,
                "fence_lateral": fence_lateral,
                "fence_lateral_end": fence_lateral_end,
                "ground_z": float(ground_z0),
                "ramp_slope": float("nan"),
                "ground_slope": float("nan"),
                "ramp_rmse": float("nan"),
                "ground_rmse": float("nan"),
                "ground_bins": 0,
                "ramp_bins": 0,
                "side_k": float(side_k),
                "side_b": float(side_b),
                "side_rmse": float(side_rmse),
                "odom_forward": float(ramp_fallback_forward),
                "cloud_forward": float(cloud_forward) if cloud_forward is not None else float("nan"),
                "visible_forward": float(visible_forward) if visible_forward is not None else float("nan"),
                "visible_lateral": float(visible_lateral) if visible_lateral is not None else float("nan"),
                "mode": fallback_mode,
            }
            self.bottom_corner_reason = "ok_fallback_no_ground"
            return
        if int(ground_mask.sum()) < 25 or int(ramp_mask.sum()) < 60:
            z_p = np.percentile(zr, [2.0, 10.0, 30.0, 50.0, 70.0])
            f_p = np.percentile(f, [2.0, 10.0, 50.0, 90.0, 98.0])
            self.bottom_corner_reason = (
                f"few_ground_or_ramp:{int(ground_mask.sum())}/{int(ramp_mask.sum())},"
                f"ground_z={ground_z0:.3f},"
                f"z_pct={z_p[0]:.3f}/{z_p[1]:.3f}/{z_p[2]:.3f}/{z_p[3]:.3f}/{z_p[4]:.3f},"
                f"f_pct={f_p[0]:.3f}/{f_p[1]:.3f}/{f_p[2]:.3f}/{f_p[3]:.3f}/{f_p[4]:.3f}"
            )
            return

        ground_f, ground_z = self._compress_profile_axis(f[ground_mask], zr[ground_mask], bin_size=0.10)
        ramp_f, ramp_z = self._compress_profile_axis(f[ramp_mask], zr[ramp_mask], bin_size=0.10)
        if len(ground_f) < 3 or len(ramp_f) < 5:
            self.bottom_corner_reason = f"few_profile_bins:{len(ground_f)}/{len(ramp_f)}"
            return
        ground_coeff, ground_rmse = self._fit_line_1d(ground_f, ground_z, trim=0.035)
        ramp_coeff, ramp_rmse = self._fit_line_1d(ramp_f, ramp_z, trim=0.055)
        if ground_coeff is None or ramp_coeff is None:
            self.bottom_corner_reason = "line_fit_failed"
            return
        ground_slope = float(ground_coeff[0])
        ramp_slope = float(ramp_coeff[0])
        denom = ramp_slope - ground_slope
        if abs(denom) < 0.08:
            self.bottom_corner_reason = f"parallel_profiles:{ramp_slope:.3f}/{ground_slope:.3f}"
            return
        bottom_forward = float((ground_coeff[1] - ramp_coeff[1]) / denom)
        if not (-0.45 <= bottom_forward <= 0.85):
            self.bottom_corner_reason = f"bottom_forward_out:{bottom_forward:.3f}"
            return
        if ramp_rmse > 0.075 or ground_rmse > 0.055 or ramp_slope < 0.12:
            self.bottom_corner_reason = (
                f"bad_fit:ramp_slope={ramp_slope:.3f},"
                f"ramp_rmse={ramp_rmse:.3f},ground_rmse={ground_rmse:.3f}"
            )
            return

        side = self._active_side()
        side_fit = self._fit_side_edge_from_cloud(f, l, zr, bottom_forward, side)
        if side_fit is None:
            fence_lateral = (
                self.initial_center_lateral - self.ramp_width * 0.5
                if side == "red"
                else self.initial_center_lateral + self.ramp_width * 0.5
            )
            fence_lateral_end = fence_lateral
            side_k = 0.0
            side_b = fence_lateral
            side_rmse = float("nan")
        else:
            side_k, side_b, side_rmse = side_fit
            fence_lateral = float(side_k * bottom_forward + side_b)
            fence_lateral_end = float(side_k * (bottom_forward + self.ramp_model_length) + side_b)

        self.bottom_corner = {
            "forward": bottom_forward,
            "fence_lateral": fence_lateral,
            "fence_lateral_end": fence_lateral_end,
            "ground_z": float(ground_coeff[0] * bottom_forward + ground_coeff[1]),
            "ramp_slope": ramp_slope,
            "ground_slope": ground_slope,
            "ramp_rmse": float(ramp_rmse),
            "ground_rmse": float(ground_rmse),
            "ground_bins": int(len(ground_f)),
            "ramp_bins": int(len(ramp_f)),
            "side_k": float(side_k),
            "side_b": float(side_b),
            "side_rmse": float(side_rmse),
        }
        self.marker_ground_z_locked = float(self.bottom_corner["ground_z"])
        self.flat_ground_z = float(self.bottom_corner["ground_z"])
        self.bottom_corner_reason = "ok"

    def _fit_side_edge_from_cloud(
        self,
        forward: np.ndarray,
        lateral: np.ndarray,
        z: np.ndarray,
        bottom_forward: float,
        side: str,
    ) -> tuple[float, float, float] | None:
        edge_f = []
        edge_l = []
        # 坡脚附近常被平台边、坡面缺口、车体姿态变化污染；用稳定坡段反推坡脚侧边。
        fit_start = bottom_forward + 0.20
        for lo in np.arange(fit_start, bottom_forward + self.ramp_model_length + 0.12, 0.12):
            m = (
                (forward >= lo)
                & (forward < lo + 0.12)
                & np.isfinite(lateral)
                & np.isfinite(z)
            )
            if int(m.sum()) < 10:
                continue
            edge_f.append(float(np.median(forward[m])))
            if side == "red":
                edge_l.append(float(np.percentile(lateral[m], 3.0)))
            else:
                edge_l.append(float(np.percentile(lateral[m], 97.0)))
        if len(edge_f) < 5:
            return None
        ef = np.asarray(edge_f, dtype=np.float64)
        el = np.asarray(edge_l, dtype=np.float64)
        med = float(np.median(el))
        if side == "red":
            keep = el <= med + 0.12
        else:
            keep = el >= med - 0.12
        if int(keep.sum()) >= 5:
            ef = ef[keep]
            el = el[keep]
        if len(ef) < 5 or float(ef.max() - ef.min()) < 0.45:
            return None
        a = np.column_stack((ef, np.ones_like(ef)))
        k, b = np.linalg.lstsq(a, el, rcond=None)[0]
        err = el - (k * ef + b)
        keep = np.abs(err) <= 0.10
        if int(keep.sum()) >= 5 and int(keep.sum()) < len(ef):
            ef = ef[keep]
            el = el[keep]
            a = np.column_stack((ef, np.ones_like(ef)))
            k, b = np.linalg.lstsq(a, el, rcond=None)[0]
            err = el - (k * ef + b)
        rmse = float(np.sqrt(np.mean(err**2))) if len(err) else float("nan")
        if not np.isfinite(rmse) or rmse > 0.09 or abs(float(k)) > 0.35:
            return None
        return float(k), float(b), rmse

    def _first_visible_side_forward(
        self,
        forward: np.ndarray,
        lateral: np.ndarray,
        z: np.ndarray,
        prior_forward: float,
        side: str,
        side_k: float,
        side_b: float,
    ) -> float | None:
        best_forward = None
        for lo in np.arange(max(0.0, prior_forward - 0.35), prior_forward + 0.90, 0.08):
            m = (
                (forward >= lo)
                & (forward < lo + 0.08)
                & np.isfinite(lateral)
                & np.isfinite(z)
            )
            if int(m.sum()) < 10:
                continue
            edge_l = float(np.percentile(lateral[m], 3.0 if side == "red" else 97.0))
            mid_f = float(np.median(forward[m]))
            expected_l = float(side_k * mid_f + side_b)
            if abs(edge_l - expected_l) > 0.16:
                continue
            best_forward = mid_f
            break
        return best_forward

    def _estimate_entry_forward_from_cloud(
        self,
        forward: np.ndarray,
        lateral: np.ndarray,
        z: np.ndarray,
        prior_forward: float,
        side: str,
    ) -> float | None:
        if len(forward) < 200:
            return None
        if side == "red":
            side_mask = lateral <= np.percentile(lateral, 12.0)
        else:
            side_mask = lateral >= np.percentile(lateral, 88.0)
        near = (
            side_mask
            & (forward >= prior_forward - 0.35)
            & (forward <= prior_forward + 0.65)
            & np.isfinite(forward)
            & np.isfinite(z)
        )
        if int(near.sum()) < 35:
            return None
        fn = forward[near]
        zn = z[near]
        z_lo, z_hi = np.percentile(zn, [5.0, 70.0])
        height_mask = (zn >= z_lo - 0.03) & (zn <= z_hi + 0.05)
        fn = fn[height_mask]
        if len(fn) < 25:
            return None
        bins = np.arange(prior_forward - 0.35, prior_forward + 0.65, 0.06)
        if len(bins) < 3:
            return None
        hist, edges = np.histogram(fn, bins=bins)
        threshold = max(4, int(hist.max() * 0.22))
        good = np.flatnonzero(hist >= threshold)
        if len(good) == 0:
            return None
        entry = float((edges[int(good[0])] + edges[int(good[0]) + 1]) * 0.5)
        if not (prior_forward - 0.35 <= entry <= prior_forward + 0.45):
            return None
        return max(0.0, entry)

    @staticmethod
    def _detect_low_z_peak(z: np.ndarray) -> float | None:
        if len(z) < 20:
            return None
        z = z[np.isfinite(z)]
        if len(z) < 20:
            return None
        lo, hi = np.percentile(z, [2.0, 70.0])
        if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
            return None
        hist, edges = np.histogram(z[(z >= lo) & (z <= hi)], bins=48)
        if len(hist) == 0 or int(hist.max()) <= 0:
            return None
        threshold = max(3, int(hist.max() * 0.18))
        for i, count in enumerate(hist):
            if int(count) >= threshold:
                return float((edges[i] + edges[i + 1]) * 0.5)
        i = int(np.argmax(hist))
        return float((edges[i] + edges[i + 1]) * 0.5)

    @staticmethod
    def _compress_profile_axis(axis: np.ndarray, z: np.ndarray, bin_size: float) -> tuple[np.ndarray, np.ndarray]:
        if len(axis) == 0:
            return np.empty(0), np.empty(0)
        bins = np.floor(axis / bin_size).astype(int)
        out_axis = []
        out_z = []
        for b in np.unique(bins):
            m = bins == b
            if int(m.sum()) < 3:
                continue
            out_axis.append(float(np.median(axis[m])))
            out_z.append(float(np.median(z[m])))
        return np.asarray(out_axis, dtype=np.float64), np.asarray(out_z, dtype=np.float64)

    @staticmethod
    def _fit_line_1d(axis: np.ndarray, z: np.ndarray, trim: float) -> tuple[np.ndarray | None, float]:
        if len(axis) < 2:
            return None, float("nan")
        a = np.column_stack((axis, np.ones_like(axis)))
        coeff, *_ = np.linalg.lstsq(a, z, rcond=None)
        pred = a @ coeff
        err = z - pred
        keep = np.abs(err) <= trim
        if int(keep.sum()) >= 2 and int(keep.sum()) < len(axis):
            axis = axis[keep]
            z = z[keep]
            a = np.column_stack((axis, np.ones_like(axis)))
            coeff, *_ = np.linalg.lstsq(a, z, rcond=None)
            pred = a @ coeff
            err = z - pred
        rmse = float(np.sqrt(np.mean(err**2))) if len(err) else float("nan")
        return coeff, rmse

    def _fit_cloud_ransac_line(self) -> None:
        if not self.enable_cloud_ransac_line or not self.cloud_samples:
            self.cloud_ransac_reason = "disabled_or_no_samples"
            return
        pose = self.locked_pose or self.ramp_start_pose
        if pose is None:
            self.cloud_ransac_reason = "no_pose"
            return
        base_x, base_y, base_z, model_yaw = pose
        pts = np.concatenate(self.cloud_samples, axis=0)
        if len(pts) < 200:
            self.cloud_ransac_reason = f"few_cloud_samples:{len(pts)}"
            return
        ground_z = self._marker_ground_z(base_z)
        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]
        dx = x - base_x
        dy = y - base_y
        forward = dx * math.cos(model_yaw) + dy * math.sin(model_yaw)
        lateral = -dx * math.sin(model_yaw) + dy * math.cos(model_yaw)
        model_z = ground_z + forward * math.tan(self.ramp_pitch_rad)
        roi = (
            (forward >= -0.10)
            & (forward <= self.ramp_model_length + 0.25)
            & (np.abs(lateral) <= self.ramp_width * 0.85)
            & (z >= ground_z - 0.04)
            & (z <= ground_z + 0.55)
            & (np.abs(z - model_z) <= 0.16)
        )
        if int(roi.sum()) < 80:
            self.cloud_ransac_reason = f"few_roi:{int(roi.sum())}"
            return
        fx = forward[roi]
        fy = lateral[roi]
        side = self._active_side()
        edge_x = []
        edge_y = []
        edge_z = []
        bin_width = 0.12
        bins = np.floor(fx / bin_width).astype(int)
        for b in np.unique(bins):
            m = bins == b
            if int(m.sum()) < 8:
                continue
            bx = fx[m]
            by = fy[m]
            edge_x.append(float(np.median(bx)))
            if side == "red":
                ey = float(np.percentile(by, 8.0))
            else:
                ey = float(np.percentile(by, 92.0))
            edge_y.append(ey)
            near_edge = np.abs(by - ey) <= 0.08
            bz = z[roi][m]
            if int(near_edge.sum()) >= 3:
                edge_z.append(float(np.median(bz[near_edge])))
            else:
                edge_z.append(float(np.median(bz)))
        if len(edge_x) < self.cloud_ransac_min_edge_points:
            self.cloud_ransac_reason = f"few_edge_bins:{len(edge_x)}"
            return
        edge_x_arr = np.asarray(edge_x)
        edge_y_arr = np.asarray(edge_y)
        edge_z_arr = np.asarray(edge_z)
        line = self._fit_ransac_y_of_x(edge_x_arr, edge_y_arr)
        if line is None:
            self.cloud_ransac_reason = f"ransac_failed:edge_bins={len(edge_x)}"
            return
        slope, intercept, inliers, rmse = line
        if int(inliers.sum()) < self.cloud_ransac_min_inliers:
            self.cloud_ransac_reason = f"few_inliers:{int(inliers.sum())}/{len(edge_x)}"
            return
        z_slope, z_intercept, z_rmse = self._fit_z_of_x(edge_x_arr[inliers], edge_z_arr[inliers])
        yaw = model_yaw + math.atan(float(slope))
        yaw_delta = math.atan2(math.sin(yaw - model_yaw), math.cos(yaw - model_yaw))
        span = float(edge_x_arr[inliers].max() - edge_x_arr[inliers].min())
        center_lateral = float(self.lateral_locked) if self.lateral_locked is not None else 0.0
        expected_fence_lateral = (
            center_lateral - self.ramp_width * 0.5
            if side == "red"
            else center_lateral + self.ramp_width * 0.5
        )
        # 只用点云边缘修正侧向偏移，不默认接管角度。
        corrected_fence_lateral = float(intercept)
        center_delta = corrected_fence_lateral - expected_fence_lateral
        x0 = 0.0
        x1 = self.ramp_model_length
        y0 = corrected_fence_lateral
        y1 = float(slope) * x1 + float(intercept)
        p0x, p0y = self._local_to_odom(base_x, base_y, model_yaw, x0, y0)
        p1x, p1y = self._local_to_odom(base_x, base_y, model_yaw, x1, y1)
        z0 = float(z_intercept + self.cloud_ransac_z_offset)
        z1 = float(z_slope * x1 + z_intercept + self.cloud_ransac_z_offset)
        corner_x, corner_y = p0x, p0y
        self.cloud_ransac_line = {
            "yaw": yaw,
            "yaw_delta": float(yaw_delta),
            "slope": float(slope),
            "intercept": float(intercept),
            "span": span,
            "expected_fence_lateral": float(expected_fence_lateral),
            "corrected_fence_lateral": float(corrected_fence_lateral),
            "center_delta": float(center_delta),
            "z_slope": float(z_slope),
            "z_intercept": float(z_intercept),
            "z_rmse": float(z_rmse),
            "inliers": int(inliers.sum()),
            "edge_points": int(len(edge_x)),
            "rmse": float(rmse),
            "side": side,
            "p0": (p0x, p0y, z0),
            "p1": (p1x, p1y, z1),
            "corner_x": corner_x,
            "corner_y": corner_y,
        }
        self.cloud_ransac_reason = "ok"

    @staticmethod
    def _fit_ransac_y_of_x(edge_x: np.ndarray, edge_y: np.ndarray) -> tuple[float, float, np.ndarray, float] | None:
        n = len(edge_x)
        if n < 2:
            return None
        rng = np.random.default_rng(20260615)
        best_inliers = np.zeros(n, dtype=bool)
        best_count = 0
        best_k = 0.0
        best_b = 0.0
        for _ in range(160):
            i, j = rng.choice(n, 2, replace=False)
            x1, x2 = edge_x[i], edge_x[j]
            if abs(x2 - x1) < 1e-6:
                continue
            k = (edge_y[j] - edge_y[i]) / (x2 - x1)
            if abs(k) > 0.45:
                continue
            b = edge_y[i] - k * x1
            residual = np.abs(edge_y - (k * edge_x + b))
            inliers = residual <= 0.06
            count = int(inliers.sum())
            if count > best_count:
                best_count = count
                best_inliers = inliers
                best_k = float(k)
                best_b = float(b)
        if best_count < 4:
            return None
        xi = edge_x[best_inliers]
        yi = edge_y[best_inliers]
        a = np.column_stack((xi, np.ones_like(xi)))
        k_opt, b_opt = np.linalg.lstsq(a, yi, rcond=None)[0]
        pred = k_opt * xi + b_opt
        rmse = float(np.sqrt(np.mean((yi - pred) ** 2)))
        if abs(float(k_opt)) > 0.45 or rmse > 0.08:
            return None
        return float(k_opt), float(b_opt), best_inliers, rmse

    @staticmethod
    def _fit_z_of_x(edge_x: np.ndarray, edge_z: np.ndarray) -> tuple[float, float, float]:
        if len(edge_x) < 2:
            z0 = float(edge_z[0]) if len(edge_z) else 0.0
            return 0.0, z0, 0.0
        a = np.column_stack((edge_x, np.ones_like(edge_x)))
        k, b = np.linalg.lstsq(a, edge_z, rcond=None)[0]
        pred = k * edge_x + b
        err = edge_z - pred
        keep = np.abs(err) <= 0.08
        if int(keep.sum()) >= 3 and int(keep.sum()) < len(edge_x):
            edge_x = edge_x[keep]
            edge_z = edge_z[keep]
            a = np.column_stack((edge_x, np.ones_like(edge_x)))
            k, b = np.linalg.lstsq(a, edge_z, rcond=None)[0]
            pred = k * edge_x + b
            err = edge_z - pred
        rmse = float(np.sqrt(np.mean(err**2))) if len(err) else 0.0
        return float(k), float(b), rmse

    def _fit_zone3_corner_from_cloud(self) -> None:
        forced_corner_side = None
        if self.field_side == "blue":
            forced_corner_side = "positive"
        elif self.field_side == "red":
            forced_corner_side = "negative"
        config = Zone3CornerConfig(
            enable_ransac_refine=self.enable_zone3_ransac_refine,
            ransac_radius=self.zone3_ransac_radius,
            ransac_dist_thr=self.zone3_ransac_dist_thr,
            ransac_min_inliers=self.zone3_ransac_min_inliers,
            forced_side=forced_corner_side,
        )
        self.zone3_corner, self.zone3_corner_reason = fit_zone3_corner_from_samples(
            self.cloud_samples,
            self.locked_pose or self.ramp_start_pose,
            config,
        )

    @staticmethod
    def _local_to_odom(base_x: float, base_y: float, yaw: float, forward: float, lateral: float) -> tuple[float, float]:
        return local_to_odom(base_x, base_y, yaw, forward, lateral)

    def _active_side(self) -> str:
        if self.field_side in ("blue", "red"):
            return self.field_side
        if self.inferred_field_side in ("blue", "red"):
            return self.inferred_field_side
        return "blue"

    @staticmethod
    def _angle_mean(a: float, b: float) -> float:
        return math.atan2(math.sin(a) + math.sin(b), math.cos(a) + math.cos(b))

    @staticmethod
    def _angle_weighted_mean(a: float, wa: float, b: float, wb: float) -> float:
        x = math.cos(a) * wa + math.cos(b) * wb
        y = math.sin(a) * wa + math.sin(b) * wb
        if abs(x) < 1e-9 and abs(y) < 1e-9:
            return math.atan2(math.sin(a) + math.sin(b), math.cos(a) + math.cos(b))
        return math.atan2(y, x)

    def _zone3_line_weight(self, prefix: str) -> float:
        if self.zone3_corner is None:
            return 1.0
        span = float(self.zone3_corner.get(f"ransac_{prefix}_span", 0.0))
        points = float(self.zone3_corner.get(f"ransac_{prefix}_points", 0.0))
        inliers = float(self.zone3_corner.get(f"{prefix}_bins", 0.0))
        rmse = float(self.zone3_corner.get(f"{prefix}_rmse", 0.08))
        support = max(points, inliers, 1.0)
        clean = 1.0 / max(0.015, rmse)
        weight = max(0.15, span) * math.sqrt(support) * clean
        return max(0.05, min(1000.0, weight))

    def _zone3_axes_from_corner(self) -> tuple[np.ndarray, np.ndarray, float] | None:
        if self.zone3_corner is None or self.locked_pose is None:
            return None
        base_x, base_y, _base_z, model_yaw = self.locked_pose
        cu = float(self.zone3_corner["corner_u"])
        cv = float(self.zone3_corner["corner_v"])
        outer_k = float(self.zone3_corner["outer_k"])
        far_k = float(self.zone3_corner["far_k"])

        axis_outer = np.asarray([1.0, outer_k], dtype=np.float64)
        axis_outer /= max(1e-6, float(np.linalg.norm(axis_outer)))
        axis_far = np.asarray([far_k, 1.0], dtype=np.float64)
        axis_far /= max(1e-6, float(np.linalg.norm(axis_far)))

        # 红线方向对应 far edge，蓝线方向对应 outer edge。RANSAC 可以各自有小角度误差，
        # 但场地坐标轴必须正交：用可信度更高的一条作为主轴，另一条强制取 90 度垂线。
        far_weight = self._zone3_line_weight("far")
        outer_weight = self._zone3_line_weight("outer")
        if far_weight >= outer_weight:
            axis_x_seed = axis_far
            axis_y_seed = np.asarray([-axis_x_seed[1], axis_x_seed[0]], dtype=np.float64)
            if float(np.dot(axis_y_seed, axis_outer)) < 0.0:
                axis_y_seed = -axis_y_seed
        else:
            axis_y_seed = axis_outer
            axis_x_seed = np.asarray([axis_y_seed[1], -axis_y_seed[0]], dtype=np.float64)
            if float(np.dot(axis_x_seed, axis_far)) < 0.0:
                axis_x_seed = -axis_x_seed
        axis_x_seed /= max(1e-6, float(np.linalg.norm(axis_x_seed)))
        axis_y_seed /= max(1e-6, float(np.linalg.norm(axis_y_seed)))

        points_local: np.ndarray | None = None
        if self.cloud_samples:
            pts = np.concatenate(self.cloud_samples, axis=0)
            if len(pts) > 0:
                dx = pts[:, 0] - base_x
                dy = pts[:, 1] - base_y
                u = dx * math.cos(model_yaw) + dy * math.sin(model_yaw)
                v = -dx * math.sin(model_yaw) + dy * math.cos(model_yaw)
                z = pts[:, 2]
                cz = float(self.zone3_corner.get("corner_z", np.nan))
                keep = np.isfinite(u) & np.isfinite(v) & (np.abs(u - cu) <= 7.0) & (np.abs(v - cv) <= 4.0)
                if np.isfinite(cz):
                    keep = keep & np.isfinite(z) & (z >= cz - 0.45) & (z <= cz + 0.45)
                if int(keep.sum()) > 0:
                    points_local = np.column_stack((u[keep], v[keep]))

        total_len = self.zone3_model_side_len + self.zone3_model_main_len
        main_depth = self.zone3_model_main_depth
        side_depth = self.zone3_model_side_depth

        def score_axes(axis_x: np.ndarray, axis_y: np.ndarray) -> float:
            if points_local is None:
                centroid_u = float(self.zone3_corner.get("centroid_u", cu))
                centroid_v = float(self.zone3_corner.get("centroid_v", cv))
                rel = np.asarray([centroid_u - cu, centroid_v - cv], dtype=np.float64)
                return float(np.dot(rel, axis_x) + np.dot(rel, axis_y))
            rel = points_local - np.asarray([cu, cv], dtype=np.float64)
            px = rel @ axis_x
            py = rel @ axis_y
            max_depth = np.where(px <= self.zone3_model_side_len, side_depth, main_depth)
            inside = (px >= -0.08) & (px <= total_len + 0.08) & (py >= -0.08) & (py <= max_depth + 0.08)
            near_x_edge = (np.abs(py) <= 0.16) & (px >= -0.10) & (px <= total_len + 0.10)
            near_y_edge = (np.abs(px) <= 0.16) & (py >= -0.10) & (py <= side_depth + 0.10)
            opposite = (px < -0.25) | (py < -0.25)
            return float(inside.sum()) + 1.5 * float(near_x_edge.sum()) + 1.5 * float(near_y_edge.sum()) - 0.25 * float(opposite.sum())

        best: tuple[float, np.ndarray, np.ndarray] | None = None
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                axis_x = axis_x_seed * sx
                axis_y = axis_y_seed * sy
                score = score_axes(axis_x, axis_y)
                if best is None or score > best[0]:
                    best = (score, axis_x, axis_y)
        if best is None:
            return None
        return best[1], best[2], best[0]

    def _zone3_inner_anchor_local(self, axis_x: np.ndarray, axis_y: np.ndarray) -> np.ndarray | None:
        if self.zone3_corner is None:
            return None
        cu = float(self.zone3_corner["corner_u"])
        cv = float(self.zone3_corner["corner_v"])
        offset = max(0.0, float(self.zone3_inner_corner_offset))
        if offset <= 1e-6:
            return np.asarray([cu, cv], dtype=np.float64)
        inward = axis_x + axis_y
        norm = float(np.linalg.norm(inward))
        if norm < 1e-6:
            return np.asarray([cu, cv], dtype=np.float64)
        inward /= norm
        return np.asarray([cu, cv], dtype=np.float64) + inward * offset

    def _refine_zone3_root_yaw(self, root_yaw: float, key_x: float, key_y: float) -> float:
        self.zone3_yaw_refine_delta = 0.0
        self.zone3_yaw_refine_score = 0.0
        if (not self.enable_zone3_yaw_refine) or self.zone3_corner is None or not self.cloud_samples:
            return root_yaw
        points = np.concatenate(self.cloud_samples, axis=0)
        if len(points) < 300:
            return root_yaw

        cloud_x = float(self.zone3_corner["corner_x"])
        cloud_y = float(self.zone3_corner["corner_y"])
        cloud_z = float(self.zone3_corner.get("corner_z", np.nan))
        total_len = self.zone3_model_side_len + self.zone3_model_main_len
        main_depth = self.zone3_model_main_depth
        side_depth = self.zone3_model_side_depth
        range_rad = math.radians(max(0.0, self.zone3_yaw_refine_range_deg))
        step_rad = math.radians(max(0.03, self.zone3_yaw_refine_step_deg))
        if range_rad <= 1e-6:
            return root_yaw

        z = points[:, 2]
        keep = np.isfinite(points[:, 0]) & np.isfinite(points[:, 1])
        if np.isfinite(cloud_z):
            keep = keep & np.isfinite(z) & (z >= cloud_z - 0.45) & (z <= cloud_z + 0.45)
        pts = points[keep]
        if len(pts) < 300:
            return root_yaw
        if len(pts) > 8000:
            pts = pts[:: max(1, len(pts) // 8000)]

        offsets = np.arange(-range_rad, range_rad + step_rad * 0.5, step_rad)
        best_yaw = root_yaw
        best_score = -1.0e18
        for delta in offsets:
            yaw = root_yaw + float(delta)
            root_x = cloud_x - (math.cos(yaw) * key_x - math.sin(yaw) * key_y)
            root_y = cloud_y - (math.sin(yaw) * key_x + math.cos(yaw) * key_y)
            dx = pts[:, 0] - root_x
            dy = pts[:, 1] - root_y
            local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
            local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
            a = key_x - local_x
            b = local_y - key_y
            max_depth = np.where(a <= self.zone3_model_side_len, side_depth, main_depth)
            inside = (a >= -0.15) & (a <= total_len + 0.15) & (b >= -0.15) & (b <= max_depth + 0.15)
            edge_x = inside & (np.abs(b) <= 0.18)
            edge_y = inside & (np.abs(a) <= 0.18) & (b <= side_depth + 0.20)
            if int(edge_x.sum() + edge_y.sum()) < 20:
                continue
            dist = np.minimum(np.abs(b), np.abs(a))
            edge_mask = edge_x | edge_y
            clipped = np.minimum(dist[edge_mask], 0.30)
            score = (
                2.0 * float(edge_x.sum())
                + 2.0 * float(edge_y.sum())
                + 0.10 * float(inside.sum())
                - 35.0 * float(np.mean(clipped))
            )
            if score > best_score:
                best_score = score
                best_yaw = yaw

        if best_score > -1.0e17:
            self.zone3_yaw_refine_delta = best_yaw - root_yaw
            self.zone3_yaw_refine_score = best_score
            return best_yaw
        return root_yaw

    def _zone3_field_mask(self, local_x: np.ndarray, local_y: np.ndarray, margin: float) -> np.ndarray:
        if self._active_side() == "red":
            local_x = -local_x
        main_x0 = -3.0 - margin
        main_x1 = 1.5 + margin
        main_y0 = -1.45 - margin
        main_y1 = 1.15 + margin
        side_x0 = 1.5 - margin
        side_x1 = 3.05 + margin
        side_y0 = -1.45 - margin
        side_y1 = 1.30 + margin
        main = (local_x >= main_x0) & (local_x <= main_x1) & (local_y >= main_y0) & (local_y <= main_y1)
        side = (local_x >= side_x0) & (local_x <= side_x1) & (local_y >= side_y0) & (local_y <= side_y1)
        return main | side

    def _zone3_field_fit_score(
        self,
        root_x: float,
        root_y: float,
        root_yaw: float,
        cloud_z: float,
    ) -> dict:
        if not self.cloud_samples:
            return {"score": -1.0e18, "inside": 0, "outside": 0, "outside_ratio": 1.0}
        pts = np.concatenate(self.cloud_samples, axis=0)
        if len(pts) < 200:
            return {"score": -1.0e18, "inside": 0, "outside": 0, "outside_ratio": 1.0}
        keep = np.isfinite(pts[:, 0]) & np.isfinite(pts[:, 1]) & np.isfinite(pts[:, 2])
        if np.isfinite(cloud_z):
            keep = keep & (pts[:, 2] >= cloud_z - 0.35) & (pts[:, 2] <= cloud_z + 0.45)
        pts = pts[keep]
        if len(pts) < 200:
            return {"score": -1.0e18, "inside": 0, "outside": 0, "outside_ratio": 1.0}
        if len(pts) > 9000:
            pts = pts[:: max(1, len(pts) // 9000)]

        dx = pts[:, 0] - root_x
        dy = pts[:, 1] - root_y
        local_x = dx * math.cos(root_yaw) + dy * math.sin(root_yaw)
        local_y = -dx * math.sin(root_yaw) + dy * math.cos(root_yaw)

        # 只评价场地附近的点，避免远处墙面/人群影响评分。
        near = self._zone3_field_mask(local_x, local_y, margin=0.45)
        if int(near.sum()) < 80:
            near = self._zone3_field_mask(local_x, local_y, margin=0.75)
        if int(near.sum()) < 80:
            return {"score": -1.0e18, "inside": 0, "outside": 0, "outside_ratio": 1.0}

        inside = self._zone3_field_mask(local_x, local_y, margin=0.06) & near
        relaxed_inside = self._zone3_field_mask(local_x, local_y, margin=0.18) & near
        hard_inside = self._zone3_field_mask(local_x, local_y, margin=0.30) & near
        outside = near & (~relaxed_inside)
        hard_outside = near & (~hard_inside)

        lx = local_x[near]
        ly = local_y[near]
        # 鼓励点云靠近可见边界，同时强惩罚跑到场地外侧。
        edge_y_min = np.abs(ly + 1.45) <= 0.16
        edge_x_pos = np.abs(lx - 3.00) <= 0.16
        edge_x_joint = np.abs(lx - 1.50) <= 0.16
        edge_bonus = int(edge_y_min.sum()) + int(edge_x_pos.sum()) + 0.5 * int(edge_x_joint.sum())

        inside_count = int(inside.sum())
        outside_count = int(outside.sum())
        hard_outside_count = int(hard_outside.sum())
        outside_ratio = outside_count / max(1, inside_count + outside_count)
        score = (
            3.0 * inside_count
            + 1.2 * float(edge_bonus)
            - 8.0 * outside_count
            - 14.0 * hard_outside_count
            - 250.0 * outside_ratio
        )
        return {
            "score": float(score),
            "inside": inside_count,
            "outside": outside_count,
            "outside_ratio": float(outside_ratio),
        }

    def _refine_zone3_root_by_inside_constraint(
        self,
        root_x: float,
        root_y: float,
        root_z: float,
        root_yaw: float,
        cloud_z: float,
    ) -> tuple[float, float, float]:
        self.zone3_inside_refine_dx = 0.0
        self.zone3_inside_refine_dy = 0.0
        self.zone3_inside_refine_dyaw = 0.0
        current = self._zone3_field_fit_score(root_x, root_y, root_yaw, cloud_z)
        best_score = float(current["score"])
        best = (root_x, root_y, root_yaw, 0.0, 0.0, 0.0, current)
        self.zone3_inside_score = best_score
        self.zone3_inside_points = int(current["inside"])
        self.zone3_outside_points = int(current["outside"])
        self.zone3_outside_ratio = float(current["outside_ratio"])
        if (
            (not self.enable_zone3_inside_refine)
            or best_score <= -1.0e17
            or (
                self.zone3_outside_ratio <= self.zone3_inside_refine_min_outside_ratio
                and self.zone3_outside_points <= self.zone3_inside_refine_min_outside_points
            )
        ):
            return root_x, root_y, root_yaw

        xy_range = max(0.0, float(self.zone3_inside_refine_xy_range))
        xy_step = max(0.01, float(self.zone3_inside_refine_xy_step))
        yaw_range = max(0.0, float(self.zone3_inside_refine_yaw_range))
        yaw_step = max(math.radians(0.05), float(self.zone3_inside_refine_yaw_step))
        dx_values = np.arange(-xy_range, xy_range + xy_step * 0.5, xy_step)
        dy_values = np.arange(-xy_range, xy_range + xy_step * 0.5, xy_step)
        yaw_values = np.arange(-yaw_range, yaw_range + yaw_step * 0.5, yaw_step)

        for dyaw in yaw_values:
            yaw = root_yaw + float(dyaw)
            cy = math.cos(yaw)
            sy = math.sin(yaw)
            for local_dx in dx_values:
                for local_dy in dy_values:
                    cand_x = root_x + cy * float(local_dx) - sy * float(local_dy)
                    cand_y = root_y + sy * float(local_dx) + cy * float(local_dy)
                    stats = self._zone3_field_fit_score(cand_x, cand_y, yaw, cloud_z)
                    move_cost = 35.0 * math.hypot(float(local_dx), float(local_dy)) + 6.0 * abs(math.degrees(float(dyaw)))
                    score = float(stats["score"]) - move_cost
                    if score > best_score:
                        best_score = score
                        best = (cand_x, cand_y, yaw, float(local_dx), float(local_dy), float(dyaw), stats)

        best_x, best_y, best_yaw, best_dx, best_dy, best_dyaw, best_stats = best
        self.zone3_inside_score = float(best_stats["score"])
        self.zone3_inside_points = int(best_stats["inside"])
        self.zone3_outside_points = int(best_stats["outside"])
        self.zone3_outside_ratio = float(best_stats["outside_ratio"])
        self.zone3_inside_refine_dx = best_dx
        self.zone3_inside_refine_dy = best_dy
        self.zone3_inside_refine_dyaw = best_dyaw
        return best_x, best_y, best_yaw

    def _zone3_root_pose_from_corner(self) -> tuple[float, float, float, float] | None:
        if self.zone3_corner is None or self.locked_pose is None:
            return None
        _base_x, _base_y, _base_z, model_yaw = self.locked_pose
        axes = self._zone3_axes_from_corner()
        if axes is None:
            return None
        axis_x, axis_y, _axis_score = axes
        anchor_local = self._zone3_inner_anchor_local(axis_x, axis_y)
        if anchor_local is None:
            return None

        def local_vec_to_odom(vec: np.ndarray) -> np.ndarray:
            return np.asarray(
                [
                    math.cos(model_yaw) * float(vec[0]) - math.sin(model_yaw) * float(vec[1]),
                    math.sin(model_yaw) * float(vec[0]) + math.cos(model_yaw) * float(vec[1]),
                ],
                dtype=np.float64,
            )

        # 对蓝区当前确认的模型点：
        # root 下关键点为 (3.000, -1.400, 0.450)。
        # 从该点进入场地的两条边，近似对应 root 的 -X 与 +Y。
        root_x_axis = -local_vec_to_odom(axis_x)
        root_y_axis = local_vec_to_odom(axis_y)
        yaw_from_x = math.atan2(float(root_x_axis[1]), float(root_x_axis[0]))
        yaw_from_y = math.atan2(float(root_y_axis[1]), float(root_y_axis[0])) - math.pi / 2.0
        weight_x = self._zone3_line_weight("far")
        weight_y = self._zone3_line_weight("outer")
        root_yaw = self._angle_weighted_mean(yaw_from_x, weight_x, yaw_from_y, weight_y)

        cloud_x = float(self.zone3_corner["corner_x"])
        cloud_y = float(self.zone3_corner["corner_y"])
        cloud_z = float(self.zone3_corner["corner_z"])
        raw_local = np.asarray(
            [float(self.zone3_corner["corner_u"]), float(self.zone3_corner["corner_v"])],
            dtype=np.float64,
        )
        inner_shift_local = anchor_local - raw_local
        inner_shift_odom = local_vec_to_odom(inner_shift_local)
        cloud_x += float(inner_shift_odom[0])
        cloud_y += float(inner_shift_odom[1])
        if abs(self.zone3_edge_normal_offset) > 1e-6:
            anchor_shift = local_vec_to_odom((axis_x + axis_y) * self.zone3_edge_normal_offset)
            cloud_x += float(anchor_shift[0])
            cloud_y += float(anchor_shift[1])

        side = self._active_side()
        key_x = self.zone3_model_key_x if side == "blue" else -self.zone3_model_key_x
        key_y = self.zone3_model_key_y
        key_z = self.zone3_model_key_z
        root_yaw = self._refine_zone3_root_yaw(root_yaw, key_x, key_y)
        root_x = cloud_x - (math.cos(root_yaw) * key_x - math.sin(root_yaw) * key_y)
        root_y = cloud_y - (math.sin(root_yaw) * key_x + math.cos(root_yaw) * key_y)
        root_z = cloud_z - key_z
        if (
            abs(self.zone3_root_calib_forward) > 1e-6
            or abs(self.zone3_root_calib_lateral) > 1e-6
            or abs(self.zone3_root_calib_z) > 1e-6
            or abs(self.zone3_root_calib_yaw) > 1e-6
        ):
            corrected_yaw = root_yaw + self.zone3_root_calib_yaw
            root_x += (
                math.cos(corrected_yaw) * self.zone3_root_calib_forward
                - math.sin(corrected_yaw) * self.zone3_root_calib_lateral
            )
            root_y += (
                math.sin(corrected_yaw) * self.zone3_root_calib_forward
                + math.cos(corrected_yaw) * self.zone3_root_calib_lateral
            )
            root_z += self.zone3_root_calib_z
            root_yaw = corrected_yaw
        root_x, root_y, root_yaw = self._refine_zone3_root_by_inside_constraint(
            root_x,
            root_y,
            root_z,
            root_yaw,
            cloud_z,
        )
        return root_x, root_y, root_z, root_yaw

    def publish_zone3_root_transform(self) -> None:
        if not self.publish_zone3_root_tf:
            return
        pose = self._zone3_root_pose_from_corner()
        if pose is None:
            return
        root_x, root_y, root_z, root_yaw = pose
        side = self._active_side()
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = self.zone3_root_frame or f"{side}_zone3_root"
        tf.transform.translation.x = float(root_x)
        tf.transform.translation.y = float(root_y)
        tf.transform.translation.z = float(root_z)
        qx, qy, qz, qw = self.rpy_to_quat(0.0, 0.0, root_yaw)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_br.sendTransform(tf)
        if not self.zone3_root_tf_log_once:
            self.static_tf_br.sendTransform(tf)
            self.zone3_root_tf_log_once = True
            self.get_logger().info(
                f"zone3 root tf: odom -> {tf.child_frame_id}, "
                f"xyz=({root_x:.3f},{root_y:.3f},{root_z:.3f}), "
                f"yaw={math.degrees(root_yaw):.2f}deg"
            )

    def publish(self) -> None:
        if self.shutting_down:
            return
        if self.zone3_corner is not None and self.locked_pose is not None:
            self.publish_zone3_root_transform()
            base_x, base_y, base_z, model_yaw = self.locked_pose
            self.publish_markers(base_x, base_y, base_z, model_yaw)
        if self.lateral_locked is None or self.locked_pose is None:
            return
        if (not self.publish_low_confidence) and self.confidence < self.conf_thr:
            return

        off = Float32()
        off.data = float(self.lateral_locked)
        self.pub_offset.publish(off)

        conf = Float32()
        conf.data = float(self.confidence)
        self.pub_conf.publish(conf)
        self.publish_inferred_side()

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "ramp_centerline"
        base_x, base_y, base_z, model_yaw = self.locked_pose
        # 使用平坦阶段锁定的地面 Z（固定基准），而非爬坡时波动的 ground_z
        ground_z = self._marker_ground_z(base_z)
        tf.transform.translation.x = base_x - math.sin(model_yaw) * float(self.lateral_locked)
        tf.transform.translation.y = base_y + math.cos(model_yaw) * float(self.lateral_locked)
        tf.transform.translation.z = ground_z  # 使用固定地面基准
        qx, qy, qz, qw = self.rpy_to_quat(0.0, self.ramp_pitch_rad, model_yaw)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_br.sendTransform(tf)
        self.publish_zone3_root_transform()
        self.publish_markers(base_x, base_y, base_z, model_yaw)

    @staticmethod
    def rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def publish_markers(self, base_x: float, base_y: float, base_z: float, model_yaw: float) -> None:
        if not self.marker_clear_once:
            self.marker_clear_once = True
            clear_marker = Marker()
            clear_marker.header.frame_id = "odom"
            clear_marker.header.stamp = self.get_clock().now().to_msg()
            clear_marker.ns = "ramp_model"
            clear_marker.id = 0
            clear_marker.action = Marker.DELETEALL
            self.pub_marker.publish(clear_marker)

        if self.publish_zone3_debug_markers:
            self.publish_zone3_corner_marker()
        self.publish_zone3_field_marker()

        if not self.publish_legacy_ramp_markers:
            return
        if self.bottom_corner is None:
            return
        center_lateral = float(self.lateral_locked)
        detected_center_lateral = float(self.lateral_ests[-1]) if self.lateral_ests else center_lateral
        fence_height = 0.10  # 围栏高度 10cm
        ground_z = self._marker_ground_z(base_z)
        entry_forward = float(self.bottom_corner["forward"]) if self.bottom_corner is not None else 0.0

        # 根据推断的场地侧选择围栏位置
        if self.inferred_field_side == "blue":
            entry_fence_lateral = self.initial_center_lateral + self.ramp_width * 0.5
            detected_fence_lateral = detected_center_lateral + self.ramp_width * 0.5
        elif self.inferred_field_side == "red":
            entry_fence_lateral = self.initial_center_lateral - self.ramp_width * 0.5
            detected_fence_lateral = detected_center_lateral - self.ramp_width * 0.5
        else:
            entry_fence_lateral = self.initial_center_lateral + self.ramp_width * 0.5
            detected_fence_lateral = detected_center_lateral + self.ramp_width * 0.5
        if self.bottom_corner is not None:
            entry_fence_lateral = float(self.bottom_corner["fence_lateral"])
            detected_fence_lateral = float(self.bottom_corner.get("fence_lateral_end", entry_fence_lateral))
        model_fence_lateral = entry_fence_lateral
        end_fence_lateral = detected_fence_lateral
        start_fence_lateral = entry_fence_lateral if self.anchor_fence_entry_to_start else end_fence_lateral

        if self.publish_fence_top_marker:
            # 围栏顶部（+10cm）
            self.publish_line_marker(
                marker_id=1,
                name="fence_top",
                lateral=start_fence_lateral,
                end_lateral=end_fence_lateral,
                start_forward=entry_forward,
                end_forward=entry_forward + self.ramp_model_length,
                base_x=base_x,
                base_y=base_y,
                base_z=base_z,
                model_yaw=model_yaw,
                height_offset=fence_height,
                rgba=(0.0, 0.6, 1.0, 0.7),
                width=0.025,
            )
        else:
            if not self.top_marker_delete_once:
                self.top_marker_delete_once = True
                self.delete_marker(marker_id=1)
        if self.publish_ramp_template:
            self.publish_ramp_template_marker(
                base_x,
                base_y,
                base_z,
                model_yaw,
                start_fence_lateral,
                end_fence_lateral,
                entry_forward,
            )
        elif not self.template_marker_delete_once:
            self.template_marker_delete_once = True
            self.delete_marker(marker_id=10)
        self.publish_corner_marker(base_x, base_y, base_z, model_yaw, start_fence_lateral, entry_forward)
        self.publish_visible_corner_candidate(base_x, base_y, base_z, model_yaw)
        self.publish_cloud_ransac_marker()
        # 主结果最后发布，避免被辅助模板覆盖。
        self.publish_line_marker(
            marker_id=0,
            name="fence_bottom",
            lateral=start_fence_lateral,
            end_lateral=end_fence_lateral,
            start_forward=entry_forward,
            end_forward=entry_forward + self.ramp_model_length,
            base_x=base_x,
            base_y=base_y,
            base_z=base_z,
            model_yaw=model_yaw,
            height_offset=0.0,
            rgba=(0.0, 0.9, 1.0, 1.0),
            width=0.055,
        )

        if not self.marker_log_once:
            self.marker_log_once = True
            self.get_logger().info(
                f"published ramp model markers: side={self.inferred_field_side}, "
                f"entry_lateral={start_fence_lateral:.3f}m, end_lateral={end_fence_lateral:.3f}m, "
                f"entry_forward={entry_forward:.3f}m, "
                f"detected_end_lateral={detected_fence_lateral:.3f}m, "
                f"z_bottom={ground_z:.3f}m, z_top={ground_z + fence_height:.3f}m"
            )

    def publish_line_marker(
        self,
        marker_id: int,
        name: str,
        lateral: float,
        end_lateral: float | None,
        start_forward: float,
        end_forward: float,
        base_x: float,
        base_y: float,
        base_z: float,
        model_yaw: float,
        height_offset: float,
        rgba: tuple[float, float, float, float],
        width: float,
    ) -> None:
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ramp_model"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = width
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        marker.pose.orientation.w = 1.0

        for forward, point_lateral in (
            (start_forward, lateral),
            (end_forward, end_lateral if end_lateral is not None else lateral),
        ):
            point = Point()
            point.x = base_x + math.cos(model_yaw) * forward - math.sin(model_yaw) * point_lateral
            point.y = base_y + math.sin(model_yaw) * forward + math.cos(model_yaw) * point_lateral
            # 使用平坦阶段锁定的地面 Z（固定基准）
            ground_z = self._marker_ground_z(base_z)
            slope_forward = max(0.0, forward - start_forward)
            point.z = ground_z + self.marker_z_offset + height_offset + slope_forward * math.tan(self.ramp_pitch_rad)
            marker.points.append(point)

        self.pub_marker.publish(marker)

    def publish_ramp_template_marker(
        self,
        base_x: float,
        base_y: float,
        base_z: float,
        model_yaw: float,
        start_fence_lateral: float,
        end_fence_lateral: float,
        entry_forward: float,
    ) -> None:
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ramp_model"
        marker.id = 10
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.018
        marker.color.r = 0.2
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 0.75
        marker.pose.orientation.w = 1.0
        ground_z = self._marker_ground_z(base_z)
        end_forward = entry_forward + self.ramp_model_length
        fence_start = self._ramp_marker_point(
            base_x,
            base_y,
            ground_z,
            model_yaw,
            entry_forward,
            start_fence_lateral,
            entry_forward,
            height_offset=0.04,
        )
        fence_end = self._ramp_marker_point(
            base_x,
            base_y,
            ground_z,
            model_yaw,
            end_forward,
            end_fence_lateral,
            entry_forward,
            height_offset=0.04,
        )

        axis_x = fence_end.x - fence_start.x
        axis_y = fence_end.y - fence_start.y
        axis_len = math.hypot(axis_x, axis_y)
        if axis_len < 1e-6:
            return
        axis_x /= axis_len
        axis_y /= axis_len
        left_x = -axis_y
        left_y = axis_x
        model_lateral_x = -math.sin(model_yaw)
        model_lateral_y = math.cos(model_yaw)
        if self.inferred_field_side == "red":
            desired_x = model_lateral_x
            desired_y = model_lateral_y
        else:
            desired_x = -model_lateral_x
            desired_y = -model_lateral_y
        if left_x * desired_x + left_y * desired_y >= 0.0:
            inward_x = left_x
            inward_y = left_y
        else:
            inward_x = -left_x
            inward_y = -left_y

        inner_start = Point()
        inner_start.x = fence_start.x + inward_x * self.ramp_width
        inner_start.y = fence_start.y + inward_y * self.ramp_width
        inner_start.z = fence_start.z
        inner_end = Point()
        inner_end.x = fence_end.x + inward_x * self.ramp_width
        inner_end.y = fence_end.y + inward_y * self.ramp_width
        inner_end.z = fence_end.z

        marker.points = [inner_start, inner_end, fence_end, fence_start, inner_start]
        self.pub_marker.publish(marker)

    def _ramp_marker_point(
        self,
        base_x: float,
        base_y: float,
        ground_z: float,
        model_yaw: float,
        forward: float,
        lateral: float,
        entry_forward: float,
        height_offset: float,
    ) -> Point:
        point = Point()
        point.x = base_x + math.cos(model_yaw) * forward - math.sin(model_yaw) * lateral
        point.y = base_y + math.sin(model_yaw) * forward + math.cos(model_yaw) * lateral
        slope_forward = max(0.0, forward - entry_forward)
        point.z = ground_z + self.marker_z_offset + height_offset + slope_forward * math.tan(self.ramp_pitch_rad)
        return point

    def delete_marker(self, marker_id: int) -> None:
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ramp_model"
        marker.id = marker_id
        marker.action = Marker.DELETE
        self.pub_marker.publish(marker)

    def publish_corner_marker(
        self,
        base_x: float,
        base_y: float,
        base_z: float,
        model_yaw: float,
        fence_lateral: float,
        entry_forward: float,
    ) -> None:
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ramp_model"
        marker.id = 20
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.035
        marker.scale.y = 0.035
        marker.scale.z = 0.035
        marker.color.r = 1.0
        marker.color.g = 0.15
        marker.color.b = 0.15
        marker.color.a = 0.9
        marker.pose.orientation.w = 1.0
        ground_z = self._marker_ground_z(base_z)
        marker.pose.position.x = base_x + math.cos(model_yaw) * entry_forward - math.sin(model_yaw) * fence_lateral
        marker.pose.position.y = base_y + math.sin(model_yaw) * entry_forward + math.cos(model_yaw) * fence_lateral
        marker.pose.position.z = ground_z + self.marker_z_offset + 0.015
        self.pub_marker.publish(marker)

    def publish_visible_corner_candidate(
        self,
        base_x: float,
        base_y: float,
        base_z: float,
        model_yaw: float,
    ) -> None:
        if self.bottom_corner is None:
            return
        forward = self.bottom_corner.get("visible_forward")
        lateral = self.bottom_corner.get("visible_lateral")
        if forward is None or lateral is None:
            return
        if not np.isfinite(float(forward)) or not np.isfinite(float(lateral)):
            self.delete_marker(marker_id=21)
            return
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ramp_model"
        marker.id = 21
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.045
        marker.scale.y = 0.045
        marker.scale.z = 0.045
        marker.color.r = 0.7
        marker.color.g = 0.1
        marker.color.b = 1.0
        marker.color.a = 0.9
        marker.pose.orientation.w = 1.0
        ground_z = self._marker_ground_z(base_z)
        marker.pose.position.x = base_x + math.cos(model_yaw) * float(forward) - math.sin(model_yaw) * float(lateral)
        marker.pose.position.y = base_y + math.sin(model_yaw) * float(forward) + math.cos(model_yaw) * float(lateral)
        marker.pose.position.z = ground_z + self.marker_z_offset + 0.025
        self.pub_marker.publish(marker)

    def publish_cloud_ransac_marker(self) -> None:
        if self.cloud_ransac_line is None:
            return
        line = self.cloud_ransac_line
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ramp_model"
        marker.id = 30
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.color.r = 1.0
        marker.color.g = 0.55
        marker.color.b = 0.0
        marker.color.a = 0.95
        marker.pose.orientation.w = 1.0
        for item in ("p0", "p1"):
            point = Point()
            point.x, point.y, point.z = line[item]
            marker.points.append(point)
        self.pub_marker.publish(marker)

        corner = Marker()
        corner.header.frame_id = "odom"
        corner.header.stamp = marker.header.stamp
        corner.ns = "ramp_model"
        corner.id = 31
        corner.type = Marker.SPHERE
        corner.action = Marker.ADD
        corner.scale.x = 0.16
        corner.scale.y = 0.16
        corner.scale.z = 0.16
        corner.color.r = 0.7
        corner.color.g = 0.1
        corner.color.b = 1.0
        corner.color.a = 0.9
        corner.pose.orientation.w = 1.0
        corner.pose.position.x = float(line["corner_x"])
        corner.pose.position.y = float(line["corner_y"])
        corner.pose.position.z = float(line["p0"][2])
        self.pub_marker.publish(corner)

    def publish_zone3_corner_marker(self) -> None:
        if self.zone3_corner is None or self.locked_pose is None:
            return
        axes = self._zone3_axes_from_corner()
        if axes is None:
            return
        axis_x, axis_y, _axis_score = axes
        base_x, base_y, base_z, model_yaw = self.locked_pose
        line_z = float(self.zone3_corner["corner_z"]) + self.marker_z_offset
        ground_z = self._marker_ground_z(base_z)
        if not np.isfinite(line_z):
            line_z = ground_z + 0.10

        def local_point(forward: float, lateral: float, z_value: float) -> Point:
            point = Point()
            point.x = base_x + math.cos(model_yaw) * forward - math.sin(model_yaw) * lateral
            point.y = base_y + math.sin(model_yaw) * forward + math.cos(model_yaw) * lateral
            point.z = z_value
            return point

        def local_xy_point(local_xy: np.ndarray, z_value: float) -> Point:
            return local_point(float(local_xy[0]), float(local_xy[1]), z_value)

        cu = float(self.zone3_corner["corner_u"])
        cv = float(self.zone3_corner["corner_v"])
        corner_local = np.asarray([cu, cv], dtype=np.float64)
        anchor_local = self._zone3_inner_anchor_local(axis_x, axis_y)
        if anchor_local is None:
            anchor_local = corner_local

        # 蓝线：水平边 1
        outer = Marker()
        outer.header.frame_id = "odom"
        outer.header.stamp = self.get_clock().now().to_msg()
        outer.ns = "ramp_model"
        outer.id = 41
        outer.type = Marker.LINE_STRIP
        outer.action = Marker.ADD
        outer.scale.x = 0.055
        outer.color.r = 0.0
        outer.color.g = 0.25
        outer.color.b = 1.0
        outer.color.a = 0.95
        outer.pose.orientation.w = 1.0
        for dist in (-0.20, 2.50):
            outer.points.append(local_xy_point(corner_local + axis_y * dist, line_z))
        self.pub_marker.publish(outer)

        # 红线：水平边 2，和蓝线接近 90 度
        far = Marker()
        far.header.frame_id = "odom"
        far.header.stamp = outer.header.stamp
        far.ns = "ramp_model"
        far.id = 42
        far.type = Marker.LINE_STRIP
        far.action = Marker.ADD
        far.scale.x = 0.055
        far.color.r = 1.0
        far.color.g = 0.0
        far.color.b = 0.0
        far.color.a = 0.95
        far.pose.orientation.w = 1.0
        for dist in (-1.35, 1.35):
            far.points.append(local_xy_point(corner_local + axis_x * dist, line_z))
        self.pub_marker.publish(far)

        raw_corner = Marker()
        raw_corner.header.frame_id = "odom"
        raw_corner.header.stamp = outer.header.stamp
        raw_corner.ns = "ramp_model"
        raw_corner.id = 45
        raw_corner.type = Marker.SPHERE
        raw_corner.action = Marker.ADD
        raw_corner.scale.x = 0.045
        raw_corner.scale.y = 0.045
        raw_corner.scale.z = 0.045
        raw_corner.color.r = 1.0
        raw_corner.color.g = 0.55
        raw_corner.color.b = 0.0
        raw_corner.color.a = 0.95
        raw_corner.pose.orientation.w = 1.0
        raw_point = local_xy_point(corner_local, line_z)
        raw_corner.pose.position.x = raw_point.x
        raw_corner.pose.position.y = raw_point.y
        raw_corner.pose.position.z = raw_point.z
        self.pub_marker.publish(raw_corner)

        # 红点：蓝红交点
        corner = Marker()
        corner.header.frame_id = "odom"
        corner.header.stamp = outer.header.stamp
        corner.ns = "ramp_model"
        corner.id = 40
        corner.type = Marker.SPHERE
        corner.action = Marker.ADD
        corner.scale.x = 0.08
        corner.scale.y = 0.08
        corner.scale.z = 0.08
        corner.color.r = 1.0
        corner.color.g = 0.0
        corner.color.b = 0.0
        corner.color.a = 1.0
        corner.pose.orientation.w = 1.0
        anchor_point = local_xy_point(anchor_local, line_z)
        corner.pose.position.x = anchor_point.x
        corner.pose.position.y = anchor_point.y
        corner.pose.position.z = anchor_point.z
        self.pub_marker.publish(corner)

        # 紫线：竖向支撑，表示这个 XY 附近确实有 Z 方向点列。
        vertical = Marker()
        vertical.header.frame_id = "odom"
        vertical.header.stamp = outer.header.stamp
        vertical.ns = "ramp_model"
        vertical.id = 43
        vertical.type = Marker.LINE_STRIP
        vertical.action = Marker.ADD
        vertical.scale.x = 0.045
        vertical.color.r = 0.55
        vertical.color.g = 0.0
        vertical.color.b = 1.0
        vertical.color.a = 0.95
        vertical.pose.orientation.w = 1.0
        low = float(self.zone3_corner.get("vertical_z_low", line_z - 0.05))
        high = float(self.zone3_corner.get("vertical_z_high", line_z + 0.05))
        if not np.isfinite(low) or not np.isfinite(high) or high <= low:
            low, high = line_z - 0.05, line_z + 0.05
        vertical.points.append(local_point(cu, cv, low))
        vertical.points.append(local_point(cu, cv, high))
        self.pub_marker.publish(vertical)

    def publish_zone3_field_marker(self) -> None:
        if (not self.enable_zone3_field_marker) or self.zone3_corner is None or self.locked_pose is None:
            return
        root_pose = self._zone3_root_pose_from_corner()
        if root_pose is None:
            return
        selected_axes = self._zone3_axes_from_corner()
        if selected_axes is None:
            return
        selected_axis_x, selected_axis_y, _axis_score = selected_axes
        base_x, base_y, base_z, model_yaw = self.locked_pose
        ground_z = self._marker_ground_z(base_z)
        z_value = float(self.zone3_corner.get("corner_z", ground_z)) + self.marker_z_offset + 0.025
        if not np.isfinite(z_value):
            z_value = ground_z + 0.06

        cu = float(self.zone3_corner["corner_u"])
        cv = float(self.zone3_corner["corner_v"])
        anchor_local = self._zone3_inner_anchor_local(selected_axis_x, selected_axis_y)
        if anchor_local is None:
            anchor_local = np.asarray([cu, cv], dtype=np.float64)
        outer_k = float(self.zone3_corner["outer_k"])
        far_k = float(self.zone3_corner["far_k"])
        centroid_u = float(self.zone3_corner.get("centroid_u", cu))
        centroid_v = float(self.zone3_corner.get("centroid_v", cv))
        to_cloud = np.asarray([centroid_u - cu, centroid_v - cv], dtype=np.float64)

        axis_outer = np.asarray([1.0, outer_k], dtype=np.float64)
        axis_outer /= max(1e-6, float(np.linalg.norm(axis_outer)))
        axis_far = np.asarray([far_k, 1.0], dtype=np.float64)
        axis_far /= max(1e-6, float(np.linalg.norm(axis_far)))

        # 从角点往点云主体方向扩，而不是固定正负方向。
        if float(np.dot(to_cloud, axis_outer)) < 0.0:
            axis_outer = -axis_outer
        if float(np.dot(to_cloud, axis_far)) < 0.0:
            axis_far = -axis_far

        p0 = anchor_local
        total_len = self.zone3_model_side_len + self.zone3_model_main_len
        main_depth = self.zone3_model_main_depth
        side_depth = self.zone3_model_side_depth
        outline_local = [
            (0.0, 0.0),
            (total_len, 0.0),
            (total_len, main_depth),
            (self.zone3_model_side_len, main_depth),
            (self.zone3_model_side_len, side_depth),
            (0.0, side_depth),
            (0.0, 0.0),
        ]
        axis_x = selected_axis_x
        axis_y = selected_axis_y
        outline_points = [
            p0 + axis_x * float(a) + axis_y * float(b)
            for a, b in outline_local
        ]

        def local_point(local_xy: np.ndarray) -> Point:
            point = Point()
            forward = float(local_xy[0])
            lateral = float(local_xy[1])
            point.x = base_x + math.cos(model_yaw) * forward - math.sin(model_yaw) * lateral
            point.y = base_y + math.sin(model_yaw) * forward + math.cos(model_yaw) * lateral
            point.z = z_value
            return point

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ramp_model"
        marker.id = 44
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.035
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.0
        marker.color.a = 0.95
        marker.pose.orientation.w = 1.0
        for item in outline_points:
            marker.points.append(local_point(item))
        self.pub_marker.publish(marker)
        self.delete_marker(marker_id=46)

    def stop_marker_publish(self) -> None:
        if self.shutting_down:
            return
        self.shutting_down = True
        self.enabled = False
        self.finalized = True
        try:
            self.publish_timer.cancel()
        except Exception:
            pass

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ramp_model"
        marker.id = 0
        marker.action = Marker.DELETEALL
        try:
            self.pub_marker.publish(marker)
            time.sleep(0.05)
            self.get_logger().info("stop marker publish and clear /ramp/model_marker")
        except Exception as exc:
            self.get_logger().warn(f"marker clear skipped during shutdown: {exc}")

    def _keyboard_loop(self) -> None:
        if not sys.stdin.isatty():
            return
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok() and not self.stop_keyboard:
                readable, _, _ = select.select([sys.stdin], [], [], 0.10)
                if not readable:
                    continue
                key = sys.stdin.read(1)
                if key in ("q", "Q"):
                    self.get_logger().info("Q pressed, shutting down")
                    self.stop_keyboard = True
                    self.stop_marker_publish()
                    if rclpy.ok():
                        rclpy.shutdown()
                    return
                if key == "\x03":
                    self.get_logger().info("Ctrl+C, shutting down")
                    self.stop_keyboard = True
                    self.stop_marker_publish()
                    if rclpy.ok():
                        rclpy.shutdown()
                    return
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FenceLocatorNode()
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
            node.stop_marker_publish()
        except (KeyboardInterrupt, Exception):
            pass
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
