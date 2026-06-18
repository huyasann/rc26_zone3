"""KfsGridDetectorNode — 九宫格颜色检测节点 (纯 ROS2, 无 Qt).

订阅 /odin1/cloud_slam, 等待 zone3 TF 锁定后,
统计每个九宫格内红色/蓝色点云数量,
发布 Marker 可视化 + 终端日志 + CSV 记录.

原脚本: script/kfs_grid_qt.py (Qt 依赖已剥离)
"""

from __future__ import annotations

import math
import os
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from zone_detection.common.point_cloud_utils import parse_xyz
from zone_detection.common.tf_utils import quat_yaw, quat_from_yaw
from zone_detection.common.ground_estimator import GroundEstimator
from zone_detection.common.debug_logger import DebugLogger
from zone_detection.kfs_grid import config as C
from zone_detection.kfs_grid.color_scorer import (
    compute_red_blue_scores,
    compute_cell_stats,
    classify_cells,
    summarize_results,
    CellResult,
)


class KfsGridDetectorNode(Node):
    """九宫格颜色检测 ROS2 节点.

    自动等待 zone3 TF → 点云坐标系变换 → 格子统计 → 发布.
    """

    def __init__(self):
        super().__init__("kfs_grid_detector")

        # ── 参数声明 ──────────────────────────────────────
        self._accumulate_frames = int(
            self.declare_parameter("accumulate_frames", C.ACCUMULATE_FRAMES).value)
        self._expand_x = float(
            self.declare_parameter("expand_x", C.EXPAND_X).value)
        self._expand_y = float(
            self.declare_parameter("expand_y", C.EXPAND_Y).value)
        self._expand_z = float(
            self.declare_parameter("expand_z", C.EXPAND_Z).value)
        self._empty_threshold = int(
            self.declare_parameter("empty_threshold", C.EMPTY_THRESHOLD).value)
        self._detailed_log = bool(
            self.declare_parameter("detailed_file_log", bool(C.DETAILED_FILE_LOG)).value)
        self._detail_log_timestamp = str(self.declare_parameter("detail_log_timestamp", "").value).strip()
        self._detail_log_name = str(self.declare_parameter("detail_log_name", "kfs_grid").value).strip() or "kfs_grid"
        detail_log_root_dir = str(self.declare_parameter("detail_log_root_dir", C.DEBUG_DIR).value).strip() or C.DEBUG_DIR
        self._detail_log_dir = os.path.join(detail_log_root_dir, "zone_detection")
        self._log_enabled = bool(
            self.declare_parameter("log_enabled", bool(C.LOG_ENABLED)).value)
        self._ground_z_known = bool(
            self.declare_parameter("ground_z_known", bool(C.GROUND_Z_KNOWN)).value)
        self._ground_z = float(
            self.declare_parameter("ground_z", C.GROUND_Z).value)

        # ── 话题 ──────────────────────────────────────────
        self._sub = self.create_subscription(
            PointCloud2, C.CLOUD_TOPIC, self._cloud_cb, 10)
        self._pub_markers = self.create_publisher(
            MarkerArray, "/rc26/zone3/cell_markers", 10)

        # ── TF ────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── 地面估计 ───────────────────────────────────────
        known_z = self._ground_z if self._ground_z_known else None
        self._ground_est = GroundEstimator(self, known_z)

        # ── 状态 ──────────────────────────────────────────
        self._lock = None  # 用简单的单线程替代 (spin 是单线程的)
        self._cell_stats: List[Tuple[int, int, int]] = [(0, 0, 0)] * 9
        self._cell_results: List[CellResult] = []  # 最新分类结果 (9 格)
        self._locked = False
        self._zone3_frame = ""
        self._tf_x = 0.0
        self._tf_y = 0.0
        self._tf_yaw = 0.0
        self._stamp_sec = 0.0
        self._frame_total = 0
        self._roi_total = 0
        self._last_marker_stamp = 0.0

        # 存储最新 TF 状态供定时器持续发布 Marker
        self._last_tf_x = 0.0
        self._last_tf_y = 0.0
        self._last_tf_yaw = 0.0
        self._last_is_blue = True

        # 多帧累积
        self._acc_buf: list = []
        self._acc_count = 0

        # ── 日志 ──────────────────────────────────────────
        self._debug_log = DebugLogger(
            self._detailed_log, self._detail_log_dir, self._detail_log_name,
            self._detail_log_timestamp)
        self._last_log = 0.0
        self._last_tf_warn = 0.0

        # ── 定时器 ────────────────────────────────────────
        self._result_timer = self.create_timer(
            1.0 / C.CELL_RESULT_RATE, self._result_timer_cb)
        self._marker_timer = self.create_timer(
            C.MARKER_PUBLISH_INTERVAL, self._marker_timer_cb)  # 独立 Marker 发布, 脱离累积周期

        self._debug_log.write_event(
            "start",
            accum_frames=self._accumulate_frames,
            expand=f"{self._expand_x}/{self._expand_y}/{self._expand_z}",
            ground_z_known=int(self._ground_z_known),
            ground_z=self._ground_z,
        )

        self.get_logger().info(
            f"KfsGridDetectorNode started | accum={self._accumulate_frames}帧"
            f" | expand X{self._expand_x:.2f} Y{self._expand_y:.2f} Z{self._expand_z:.2f}"
            f" | ground_z={'fixed='+str(self._ground_z) if self._ground_z_known else 'auto'}")

    # ════════════════════════════════════════════════════════
    # 点云回调 (主处理管线)
    # ════════════════════════════════════════════════════════

    def _cloud_cb(self, msg: PointCloud2):
        """每帧: 解析点云 → 查 zone3 TF → 投影 → 累积 → 统计 → 发布 Marker."""
        try:
            x, y, z = parse_xyz(msg)
        except Exception:
            return
        n_total = len(x)
        if n_total == 0:
            return

        # 解析 RGB (必须支持 rgb 字段)
        rgb_data = self._parse_rgb(msg, n_total)
        if rgb_data is None:
            return
        r, g, b = rgb_data

        stamp_sec = (float(msg.header.stamp.sec)
                     + float(msg.header.stamp.nanosec) * 1e-9)

        # 1. 查 zone3 TF
        z3 = self._resolve_zone3_tf()
        if z3 is None:
            self._locked = False
            return
        tf_x, tf_y, tf_yaw, frame_name = z3
        # 存储 TF 状态供 Marker 定时器持续发布
        self._last_tf_x = tf_x
        self._last_tf_y = tf_y
        self._last_tf_yaw = tf_yaw
        self._last_is_blue = "blue" in frame_name
        c, s = math.cos(tf_yaw), math.sin(tf_yaw)

        # 2. 点云 → odom 系
        if C.CLOUD_IN_ODOM_FRAME:
            px, py, pz = x.copy(), y.copy(), z.copy()
        else:
            try:
                t = self._tf_buffer.lookup_transform(
                    "odom", msg.header.frame_id, RclpyTime(),
                    timeout=rclpy.duration.Duration(seconds=C.TF_TIMEOUT_SEC))
            except Exception:
                return
            px, py, pz = self._transform_points(x, y, z, t)

        # 2b. 高度归一 (投影到 odom Z 轴, 与 zone2/zone3 一致)
        z_h = self._ground_est.to_height_frame(msg.header, px, py, pz)
        if z_h is None or len(z_h) < 10:
            return
        if not self._ground_z_known:
            self._ground_est.update_ground(z_h)
        if self._ground_est.ground_z is None:
            return
        current_ground_z = self._ground_est.ground_z

        # 3. 计算网格中心在 odom 中的位姿
        is_blue = "blue" in frame_name
        gdx = C.BLUE_GRID_CENTER_X if is_blue else -C.BLUE_GRID_CENTER_X
        gdy = C.GRID_CENTER_Y
        gx = tf_x + c * gdx - s * gdy
        gy = tf_y + s * gdx + c * gdy

        # 4. 投影到 grid-local 系
        h = z_h - current_ground_z
        lx = c * (px - gx) + s * (py - gy)
        ly = -s * (px - gx) + c * (py - gy)

        # 5. ROI 预筛
        roi = (
            (np.abs(lx) <= C.ROI_LX_HALF)
            & (np.abs(ly) <= C.ROI_LY_HALF)
            & (h >= C.ROI_H_MIN) & (h <= C.ROI_H_MAX)
        )
        idx_roi = np.where(roi)[0]
        n_roi = len(idx_roi)
        if n_roi == 0:
            return

        # 6. 累积到缓存
        self._acc_buf.append((
            h[idx_roi], lx[idx_roi], ly[idx_roi],
            r[idx_roi], g[idx_roi], b[idx_roi],
        ))
        self._acc_count += 1
        if self._acc_count < self._accumulate_frames:
            return

        # 7. 合并累积帧
        acc_h = np.concatenate([a[0] for a in self._acc_buf])
        acc_lx = np.concatenate([a[1] for a in self._acc_buf])
        acc_ly = np.concatenate([a[2] for a in self._acc_buf])
        acc_r = np.concatenate([a[3] for a in self._acc_buf]).astype(np.int16)
        acc_g = np.concatenate([a[4] for a in self._acc_buf]).astype(np.int16)
        acc_b = np.concatenate([a[5] for a in self._acc_buf]).astype(np.int16)

        self._acc_buf.clear()
        self._acc_count = 0

        # 8. 计算色彩权重
        w_red, w_blue = compute_red_blue_scores(acc_r, acc_g, acc_b)

        # 9. 逐格统计
        stats = compute_cell_stats(
            acc_lx, acc_ly, acc_h, w_red, w_blue,
            C.COL_CENTERS, C.LAYER_CENTERS,
            self._expand_x, self._expand_y, self._expand_z,
        )

        # 10. 分类
        results = classify_cells(
            stats, C.CELL_NAMES,
            self._empty_threshold,
            C.MIN_VALID_SCORE_ABS, C.MIN_VALID_SCORE_RATIO, C.DOMINANT_RATIO,
        )
        summary = summarize_results(results)

        # 11. 更新状态
        self._cell_stats = stats
        self._cell_results = results  # 供 Marker 发布和 Qt 窗口使用
        self._locked = True
        self._zone3_frame = frame_name
        self._tf_x = tf_x
        self._tf_y = tf_y
        self._tf_yaw = tf_yaw
        self._stamp_sec = stamp_sec
        self._frame_total = n_total
        self._roi_total = n_roi

        # 12. 终端日志
        if self._log_enabled and time.monotonic() - self._last_log >= C.LOG_INTERVAL:
            self._last_log = time.monotonic()
            total_cell = sum(s[2] for s in stats)
            self.get_logger().info(
                f"帧={n_total//1000}k ROI={n_roi} 格={total_cell} | "
                f"R={summary['total_red_score']} B={summary['total_blue_score']} "
                f"{'🔴' if summary['dominant']=='RED' else '🔵' if summary['dominant']=='BLUE' else '⚪'} "
                f"[{summary['dominant']}] | "
                f"红{summary['red_cells']}格 蓝{summary['blue_cells']}格 "
                f"空{summary['empty_cells']}格 ?{summary['unknown_cells']}格")

        # 13. 事件日志
        self._debug_log.write_event(
            "detection",
            frame=frame_name,
            total_pts=n_total,
            roi_pts=n_roi,
            red_score=summary["total_red_score"],
            blue_score=summary["total_blue_score"],
            dominant=summary["dominant"],
            red_cells=summary["red_cells"],
            blue_cells=summary["blue_cells"],
            empty_cells=summary["empty_cells"],
        )

    # ════════════════════════════════════════════════════════
    # TF 查找
    # ════════════════════════════════════════════════════════

    def _resolve_zone3_tf(self) -> Optional[Tuple[float, float, float, str]]:
        """查找 odom→{blue|red}_zone3_root TF."""
        for prefix in ("blue_", "red_"):
            frame = prefix + "zone3_root"
            try:
                t = self._tf_buffer.lookup_transform(
                    "odom", frame, RclpyTime(),
                    timeout=rclpy.duration.Duration(seconds=C.Z3_TF_TIMEOUT_SEC))
            except Exception:
                continue
            return (float(t.transform.translation.x),
                    float(t.transform.translation.y),
                    quat_yaw(t.transform.rotation),
                    frame)
        return None

    # ════════════════════════════════════════════════════════
    # 点云坐标系变换
    # ════════════════════════════════════════════════════════

    @staticmethod
    def _transform_points(
        x: np.ndarray, y: np.ndarray, z: np.ndarray, t,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """点云 (x,y,z) 经 TransformStamped t 从源系变换到目标系."""
        rot = t.transform.rotation
        qx, qy, qz, qw = rot.x, rot.y, rot.z, rot.w
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r02 = 2.0 * (qx * qz + qy * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qx * qw)
        r20 = 2.0 * (qx * qz - qy * qw)
        r21 = 2.0 * (qy * qz + qx * qw)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
        tx = float(t.transform.translation.x)
        ty = float(t.transform.translation.y)
        tz = float(t.transform.translation.z)
        return (r00 * x + r01 * y + r02 * z + tx,
                r10 * x + r11 * y + r12 * z + ty,
                r20 * x + r21 * y + r22 * z + tz)

    # ════════════════════════════════════════════════════════
    # RGB 解析
    # ════════════════════════════════════════════════════════

    @staticmethod
    def _parse_rgb(cloud: PointCloud2, n: int
                   ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """从 PointCloud2 提取 r/g/b 通道 (uint8)."""
        dt = np.dtype({
            "names": ["x", "y", "z", "rgb"],
            "formats": [np.float32, np.float32, np.float32, np.uint32],
            "offsets": [0, 4, 8, 12],
            "itemsize": cloud.point_step,
        })
        pts = np.frombuffer(cloud.data, dtype=dt, count=n)
        packed = pts["rgb"]
        r = ((packed >> 16) & 0xFF).astype(np.uint8)
        g = ((packed >> 8) & 0xFF).astype(np.uint8)
        b = (packed & 0xFF).astype(np.uint8)
        return r, g, b

    # ════════════════════════════════════════════════════════
    # Marker 发布
    # ════════════════════════════════════════════════════════

    def _publish_cell_markers(self, stamp, tf_x, tf_y, tf_yaw, is_blue):
        """发布 9 格 MarkerArray (CUBE, 颜色随检测结果变化).

        ENABLE_RESULT_MARKERS=True:  RED→红, BLUE→蓝, EMPTY→深灰, UNKNOWN→浅灰
        ENABLE_RESULT_MARKERS=False: 使用旧版层默认颜色 (底蓝/中绿/顶红)
        """
        ma = MarkerArray()
        c, s = math.cos(tf_yaw), math.sin(tf_yaw)
        gdx = C.BLUE_GRID_CENTER_X if is_blue else -C.BLUE_GRID_CENTER_X
        gdy = C.GRID_CENTER_Y

        # 构建结果查找表 (cell_id → label)
        result_by_id: dict = {}
        if C.ENABLE_RESULT_MARKERS and self._cell_results:
            for r in self._cell_results:
                result_by_id[r.layer * 3 + r.col] = r.label

        for lidx in range(3):
            for cidx in range(3):
                cell_id = lidx * 3 + cidx
                yc = C.COL_CENTERS[cidx]
                zc = C.LAYER_CENTERS[lidx]
                ox = tf_x + c * gdx - s * (gdy + yc)
                oy = tf_y + s * gdx + c * (gdy + yc)
                # 使用当前 ground_z (可能来自 GroundEstimator 动态更新)
                oz = (self._ground_est.ground_z if self._ground_est.ground_z is not None else C.GROUND_Z) + zc

                mk = Marker()
                mk.header.stamp = stamp
                mk.header.frame_id = "odom"
                mk.ns = "kfs_cell"
                mk.id = cell_id
                mk.type = Marker.CUBE
                mk.action = Marker.ADD
                mk.pose.position.x = ox
                mk.pose.position.y = oy
                mk.pose.position.z = oz
                qx, qy, qz, qw = quat_from_yaw(tf_yaw)
                mk.pose.orientation.x = qx
                mk.pose.orientation.y = qy
                mk.pose.orientation.z = qz
                mk.pose.orientation.w = qw
                mk.scale.x = C.MARKER_SCALE_X
                mk.scale.y = C.MARKER_SCALE_Y
                mk.scale.z = C.MARKER_SCALE_Z
                mk.color.a = C.MARKER_ALPHA

                # 根据检测结果选择颜色
                label = result_by_id.get(cell_id, "UNKNOWN")
                if label == "RED":
                    cr, cg, cb = C.MARKER_COLOR_RED
                elif label == "BLUE":
                    cr, cg, cb = C.MARKER_COLOR_BLUE
                elif label == "EMPTY":
                    cr, cg, cb = C.MARKER_COLOR_EMPTY
                elif C.ENABLE_RESULT_MARKERS:
                    cr, cg, cb = C.MARKER_COLOR_UNKNOWN  # UNKNOWN + 启用结果着色
                else:
                    cr, cg, cb = C.LAYER_MARKER_COLORS[lidx]  # 旧版层默认颜色
                mk.color.r = cr
                mk.color.g = cg
                mk.color.b = cb
                mk.lifetime.sec = C.MARKER_LIFETIME_SEC
                ma.markers.append(mk)
        self._pub_markers.publish(ma)

    # ════════════════════════════════════════════════════════
    # 定时器回调
    # ════════════════════════════════════════════════════════

    def _result_timer_cb(self):
        """定时回调: 锁定后 5Hz 输出最新统计摘要."""
        if not self._locked:
            return
        stats = self._cell_stats
        results = classify_cells(
            stats, C.CELL_NAMES,
            self._empty_threshold,
            C.MIN_VALID_SCORE_ABS, C.MIN_VALID_SCORE_RATIO, C.DOMINANT_RATIO,
        )
        summary = summarize_results(results)
        self._debug_log.write_event(
            "timer_report",
            dominant=summary["dominant"],
            red_cells=summary["red_cells"],
            blue_cells=summary["blue_cells"],
        )

    def _marker_timer_cb(self):
        """定时回调: 独立于累积周期持续发布 Marker, 消除闪烁.

        使用存储的最新 TF 状态 + _cell_results,
        确保累积期间 Marker 始终可见 (不会过期消失).
        """
        if not self._locked:
            return
        stamp = self.get_clock().now().to_msg()
        self._publish_cell_markers(
            stamp, self._last_tf_x, self._last_tf_y,
            self._last_tf_yaw, self._last_is_blue)

    # ════════════════════════════════════════════════════════
    # 公共读数接口 (供外部/测试调用)
    # ════════════════════════════════════════════════════════

    @property
    def cell_stats(self) -> List[Tuple[int, int, int]]:
        """最新 9 格统计 [(r,b,t),...]."""
        return list(self._cell_stats)

    @property
    def cell_results(self) -> List[CellResult]:
        """最新 9 格分类结果 [CellResult,...]."""
        return list(self._cell_results)

    @property
    def is_locked(self) -> bool:
        """zone3 TF 是否就绪."""
        return self._locked

    @property
    def zone3_frame(self) -> str:
        """当前使用的 zone3 frame 名."""
        return self._zone3_frame

    def __del__(self):
        if hasattr(self, '_debug_log'):
            self._debug_log.close()


def main():
    """节点入口 — 根据 ENABLE_QT_GUI 自动选择模式.

    ENABLE_QT_GUI=True  → Qt 窗口模式 (ROS spin 后台线程 + Qt 主线程)
    ENABLE_QT_GUI=False → 无头模式 (纯终端, 生产环境)
    PyQt5 不可用时自动回退到无头模式.
    """
    rclpy.init()
    node = KfsGridDetectorNode()
    qt_managed_shutdown = False
    try:
        if C.ENABLE_QT_GUI:
            try:
                from zone_detection.kfs_grid.qt_window import run_qt
                node.get_logger().info("ENABLE_QT_GUI=True, 启动 Qt 调试窗口...")
                qt_managed_shutdown = True
                return run_qt(node)
            except ImportError:
                node.get_logger().warn(
                    "ENABLE_QT_GUI=True 但 PyQt5 不可用, 回退到无头模式")
                rclpy.spin(node)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not qt_managed_shutdown:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main_qt():
    """节点入口 — 强制 Qt 调试窗口模式 (kfs_grid_detector_qt executable).

    与 main() 共用 KfsGridDetectorNode 处理逻辑,
    Qt 窗口通过 node.cell_stats / node.cell_results / node.is_locked 读取结果.
    """
    rclpy.init()
    node = KfsGridDetectorNode()
    try:
        from zone_detection.kfs_grid.qt_window import run_qt
        raise SystemExit(run_qt(node))
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
