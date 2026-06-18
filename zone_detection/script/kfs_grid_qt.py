#!/home/inkc/Software/miniconda3/envs/env_ros2/bin/python3
# -*- coding: utf-8 -*-
"""KFS 九宫格格子颜色调试工具 — Qt 窗口 + Marker 可视化.

订阅 /odin1/cloud_slam, 等待 zone3 TF 锁定后,
统计每个格子内红色/蓝色点云数量并实时显示,
同时发布透明 Marker 标出格子范围.

用法:
  python3 src/custom/actions/zone_detection/script/kfs_grid_qt.py
"""

from __future__ import annotations

import math
import signal
import sys
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

try:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtGui import QFont
    from PyQt5.QtWidgets import (
        QApplication, QGridLayout, QHBoxLayout, QLabel,
        QMainWindow, QVBoxLayout, QWidget,
    )
except ImportError as exc:
    raise SystemExit("缺少 PyQt5。请在带 PyQt5 的环境运行本脚本。") from exc


# ═════════════════════════════════════════════════════════════════════
# 参数区 ─── 改这里
# ═════════════════════════════════════════════════════════════════════

# ── 点云输入 ─────────────────────────────────────────────────────
CLOUD_TOPIC = "/odin1/cloud_slam"       # 点云话题名
CLOUD_IN_ODOM_FRAME = False             # True=点云已在 odom 系, 跳过 TF 变换

# ── 多帧累积 ─────────────────────────────────────────────────────
ACCUMULATE_FRAMES = 20                   # 累积 N 帧后再输出统计. 1=不累积

# ── 地面高度 ─────────────────────────────────────────────────────
GROUND_Z = -0.3100                      # odom 系中地面 Z 值 (m). 与 zone3 config 一致

# ── 格子中心位置 ──────────────────────────────────────────────
# 原点在网格中心, grid-local 系.  COL_CENTERS: 3 列中心 Y (左→右)
COL_CENTERS   = [-0.54, 0.0, 0.54]
LAYER_CENTERS = [1.07, 1.61, 2.15]      # 3 层中心 Z (离地高度, 底→顶)

# ── 检测框: 从格子中心向各方向延伸的距离 ─────────────────────
# 检测框以格子中心为原点, 在 X/Y/Z 方向各向外延伸对应的米数.
#   即: 检测总宽 = EXPAND × 2 (双边).
#
#   例如 EXPAND_X=0.45: X 方向从格子中心向前后各延伸 0.45m,
#                       检测总宽 = 0.45×2 = 0.90m.
#   EXPAND=0: 该方向不延伸, 检测范围退化为一条线 (无体积).
#
EXPAND_X = 0.6   # X 方向 (前后/depth) 从中心延伸.  官方块大小 0.30/2
EXPAND_Y = 0.27   # Y 方向 (左右/width) 从中心延伸.  避开亚克力边框
EXPAND_Z = 0.27   # Z 方向 (上下/height) 从中心延伸.  避开亚克力边框

# ── 颜色净胜分算法 (Numpy 向量化) ──
# 不再使用 HSV 硬阈值，改用色彩纯度权重矩阵计算
EMPTY_THRESHOLD = 15    # 格子总点数低于此 → 判空

# ── 格子名称 (Qt 界面显示用) ────────────────────────────────────
CELL_NAMES = [
    ["底左", "底中", "底右"],   # layer=0, Qt 显示在底部行
    ["中左", "中中", "中右"],   # layer=1, Qt 显示在中间行
    ["顶左", "顶中", "顶右"],   # layer=2, Qt 显示在顶部行
]

# ── GUI ─────────────────────────────────────────────────────────
GUI_REFRESH_HZ = 10.0   # Qt 窗口刷新频率

# ── Marker 可视化 ───────────────────────────────────────────────
MARKER_ALPHA = 0.25      # 透明度 0~1, 越小越透明
# Marker 立方体尺寸 = 格子物理尺寸 + 双边膨胀
# 自动计算, 如需独立调节可在此覆盖
MARKER_SCALE_X = EXPAND_X * 2   # 前后总宽
MARKER_SCALE_Y = EXPAND_Y * 2   # 左右总宽
MARKER_SCALE_Z = EXPAND_Z * 2   # 上下总高

# ── 场地固定偏移 (不常改) ──────────────────────────────────────
# grid_center 在 zone3_root 坐标系中的位置 (来自 rc26_field.py 模型)
_BLUE_GRID_CENTER_X = -3.025    # 蓝方 grid_center 相对 zone3_root 的 X 偏移
_BLUE_GRID_CENTER_Y = -0.15     # grid_center 相对 zone3_root 的 Y 偏移 (红蓝同)


# ═════════════════════════════════════════════════════════════════════
# 工具函数
# ═════════════════════════════════════════════════════════════════════

def _parse_xyzrgb(cloud: PointCloud2):
    """PointCloud2 → (x,y,z,r,g,b). 自动适配 field layout."""
    n = cloud.width * cloud.height if cloud.height > 1 else cloud.width
    dt = np.dtype({
        "names": ["x", "y", "z", "rgb"],
        "formats": [np.float32, np.float32, np.float32, np.uint32],
        "offsets": [0, 4, 8, 12],
        "itemsize": cloud.point_step,
    })
    pts = np.frombuffer(cloud.data, dtype=dt, count=n)
    x = pts["x"].astype(np.float64)
    y = pts["y"].astype(np.float64)
    z = pts["z"].astype(np.float64)
    packed = pts["rgb"]
    r = ((packed >> 16) & 0xFF).astype(np.uint8)
    g = ((packed >> 8) & 0xFF).astype(np.uint8)
    b = (packed & 0xFF).astype(np.uint8)
    return x, y, z, r, g, b


def quat_to_yaw(q) -> float:
    """四元数 → 绕 Z 轴偏航角 (rad)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# ═════════════════════════════════════════════════════════════════════
# ROS2 节点
# ═════════════════════════════════════════════════════════════════════

class KfsGridQtNode(Node):
    """订阅 /odin1/cloud_slam, 统计 9 格红蓝点数, 供 Qt 窗口读取."""

    def __init__(self) -> None:
        """初始化: 订阅点云, 创建 TF listener, 初始化累积缓存."""
        super().__init__("kfs_grid_qt")
        self.sub = self.create_subscription(
            PointCloud2, CLOUD_TOPIC, self._cloud_cb, 10)
        self._pub_markers = self.create_publisher(
            MarkerArray, "/rc26/zone3/cell_markers", 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._lock = threading.Lock()
        self._cell_stats: list = [(0, 0, 0)] * 9
        self._locked = False
        self._zone3_frame = ""
        self._tf_x = 0.0
        self._tf_y = 0.0
        self._tf_yaw = 0.0
        self._stamp_sec = 0.0
        self._frame_total = 0      # 诊断: 整帧点数
        self._roi_total = 0        # 诊断: 进入 ROI 的点数
        self._last_marker_stamp = 0.0

        # 多帧累积
        self._acc_buf: list = []
        self._acc_count = 0

        self.get_logger().info(
            f"KfsGridQtNode started | accum={ACCUMULATE_FRAMES}帧"
            f" | odom_frame={CLOUD_IN_ODOM_FRAME}")

    # ────────────────────────────────────────────────────────────
    # 点云回调 (主处理管线)
    # ────────────────────────────────────────────────────────────

    def _cloud_cb(self, msg: PointCloud2):
        """每帧: 解析点云 → 查 zone3 TF → 投影到 grid-local → ROI 预筛 → 累积 → 逐格统计 → 发布 Marker."""
        try:
            x, y, z, r, g, b = _parse_xyzrgb(msg)
        except Exception:
            return
        n_total = len(x)
        if n_total == 0:
            return

        stamp_sec = (float(msg.header.stamp.sec)
                     + float(msg.header.stamp.nanosec) * 1e-9)

        # 1. 查 zone3 TF
        z3 = self._resolve_zone3_tf()
        if z3 is None:
            with self._lock:
                self._locked = False
            return
        tf_x, tf_y, tf_yaw, frame_name = z3
        c, s = math.cos(tf_yaw), math.sin(tf_yaw)

        # 2. 点云 → odom 系
        if CLOUD_IN_ODOM_FRAME:
            px, py, pz = x.copy(), y.copy(), z.copy()
        else:
            try:
                t = self._tf_buffer.lookup_transform(
                    "odom", msg.header.frame_id, RclpyTime(),
                    timeout=rclpy.duration.Duration(seconds=0.05))
            except Exception:
                return
            px, py, pz = self._transform_points(x, y, z, t)

        # 3. 计算网格中心在 odom 中的位姿
        is_blue = "blue" in frame_name
        gdx = _BLUE_GRID_CENTER_X if is_blue else -_BLUE_GRID_CENTER_X
        gdy = _BLUE_GRID_CENTER_Y
        gx = tf_x + c * gdx - s * gdy
        gy = tf_y + s * gdx + c * gdy

        # 4. 投影到 grid-local 系 (lx=深度, ly=宽度, h=离地高度)
        h = pz - GROUND_Z
        lx = c * (px - gx) + s * (py - gy)
        ly = -s * (px - gx) + c * (py - gy)

        # 5. 广域 ROI 预筛: 只保留格子周边点, 减少后续计算量
        roi_half_w = 0.81 + EXPAND_Y   # 覆盖 3 列 (列距 0.54×1.5=0.81)
        roi = (
            (np.abs(lx) <= 2.0)
            & (np.abs(ly) <= roi_half_w)
            & (h >= 0.5) & (h <= 2.8)
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
        if self._acc_count < ACCUMULATE_FRAMES:
            return

        # 7. 合并累积的所有帧
        acc_h  = np.concatenate([a[0] for a in self._acc_buf])
        acc_lx = np.concatenate([a[1] for a in self._acc_buf])
        acc_ly = np.concatenate([a[2] for a in self._acc_buf])

        # 必须转为 int16 防止 np.uint8 相减时发生溢出
        acc_r  = np.concatenate([a[3] for a in self._acc_buf]).astype(np.int16)
        acc_g  = np.concatenate([a[4] for a in self._acc_buf]).astype(np.int16)
        acc_b  = np.concatenate([a[5] for a in self._acc_buf]).astype(np.int16)

        self._acc_buf.clear()
        self._acc_count = 0

        # --- 🚀 核心优化：矩阵化色彩纯度净胜分 (0.0 ~ 1.0) ---
        # 红色权重 = (R - max(G,B)) / 255 * (R / 255)
        red_diff = np.maximum(0, acc_r - np.maximum(acc_g, acc_b))
        w_red = (red_diff / 255.0) * (acc_r / 255.0)

        # 蓝色权重 = (B - max(R,G)) / 255 * (B / 255)
        # 亚克力反光(青色) G 和 B 接近，差值极小，会被此公式自动归零
        blue_diff = np.maximum(0, acc_b - np.maximum(acc_r, acc_g))
        w_blue = (blue_diff / 255.0) * (acc_b / 255.0)

        # 8. 逐格统计: 每格统计 (红净胜分, 蓝净胜分, 总点数)
        stats = []
        for layer in range(3):
            zc = LAYER_CENTERS[layer]
            for col in range(3):
                yc = COL_CENTERS[col]
                mask = (
                    (np.abs(acc_lx) <= EXPAND_X)
                    & (np.abs(acc_ly - yc) <= EXPAND_Y)
                    & (np.abs(acc_h  - zc) <= EXPAND_Z)
                )
                idx = np.where(mask)[0]
                ct = len(idx)
                if ct == 0:
                    stats.append((0, 0, 0))
                else:
                    # 分数等于所有点的权重总和，不再逐点调用 if-else
                    red_score = int(np.sum(w_red[idx]))
                    blue_score = int(np.sum(w_blue[idx]))
                    stats.append((red_score, blue_score, ct))

        total_cell = sum(s[2] for s in stats)
        if n_roi > 0:
            self.get_logger().info(
                f"帧点={n_total}  ROI筛={n_roi}  入格={total_cell}  "
                f"({n_total//1000}k → {n_roi} → {total_cell})",
                throttle_duration_sec=2.0)

        # 写入线程安全缓存
        with self._lock:
            self._cell_stats = stats
            self._locked = True
            self._zone3_frame = frame_name
            self._tf_x = tf_x
            self._tf_y = tf_y
            self._tf_yaw = tf_yaw
            self._stamp_sec = stamp_sec
            self._frame_total = n_total
            self._roi_total = n_roi

        # 9. 发布 Marker (每 0.5s 一次)
        if stamp_sec - self._last_marker_stamp >= 0.5:
            self._publish_cell_markers(msg.header.stamp, tf_x, tf_y, tf_yaw, is_blue)
            self._last_marker_stamp = stamp_sec

    # ────────────────────────────────────────────────────────────
    # zone3 TF 查找
    # ────────────────────────────────────────────────────────────

    def _resolve_zone3_tf(self):
        """查找 odom→blue/red_zone3_root TF. 返回 (x,y,yaw,frame) 或 None."""
        for prefix in ("blue_", "red_"):
            frame = prefix + "zone3_root"
            try:
                t = self._tf_buffer.lookup_transform(
                    "odom", frame, RclpyTime(),
                    timeout=rclpy.duration.Duration(seconds=0.02))
            except Exception:
                continue
            return (t.transform.translation.x,
                    t.transform.translation.y,
                    quat_to_yaw(t.transform.rotation),
                    frame)
        return None

    # ────────────────────────────────────────────────────────────
    # 点云坐标系变换
    # ────────────────────────────────────────────────────────────

    @staticmethod
    def _transform_points(x, y, z, t):
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

    # ────────────────────────────────────────────────────────────
    # Marker 发布
    # ────────────────────────────────────────────────────────────

    def _publish_cell_markers(self, stamp, tf_x, tf_y, tf_yaw, is_blue):
        """发布 9 格 MarkerArray (CUBE, 半透明, 尺寸=CELL+EXPAND×2)."""
        ma = MarkerArray()
        c, s = math.cos(tf_yaw), math.sin(tf_yaw)
        gdx = _BLUE_GRID_CENTER_X if is_blue else -_BLUE_GRID_CENTER_X
        gdy = _BLUE_GRID_CENTER_Y

        for lidx in range(3):
            for cidx in range(3):
                yc = COL_CENTERS[cidx]
                zc = LAYER_CENTERS[lidx]
                ox = tf_x + c * gdx - s * (gdy + yc)
                oy = tf_y + s * gdx + c * (gdy + yc)
                oz = GROUND_Z + zc

                mk = Marker()
                mk.header.stamp = stamp
                mk.header.frame_id = "odom"
                mk.ns = "kfs_cell"
                mk.id = lidx * 3 + cidx
                mk.type = Marker.CUBE
                mk.action = Marker.ADD
                mk.pose.position.x = ox
                mk.pose.position.y = oy
                mk.pose.position.z = oz
                mk.pose.orientation.x = 0.0
                mk.pose.orientation.y = 0.0
                mk.pose.orientation.z = math.sin(tf_yaw / 2.0)
                mk.pose.orientation.w = math.cos(tf_yaw / 2.0)
                mk.scale.x = MARKER_SCALE_X
                mk.scale.y = MARKER_SCALE_Y
                mk.scale.z = MARKER_SCALE_Z
                mk.color.a = MARKER_ALPHA
                if lidx == 0:
                    mk.color.r, mk.color.g, mk.color.b = 0.2, 0.2, 1.0
                elif lidx == 1:
                    mk.color.r, mk.color.g, mk.color.b = 0.2, 1.0, 0.2
                else:
                    mk.color.r, mk.color.g, mk.color.b = 1.0, 0.2, 0.2
                mk.lifetime.sec = 1
                ma.markers.append(mk)
        self._pub_markers.publish(ma)

    # ────────────────────────────────────────────────────────────
    # 线程安全读取
    # ────────────────────────────────────────────────────────────

    def get_stats(self):
        """返回 (stats, locked, z3_frame, stamp_sec, frame_total, roi_total). 线程安全."""
        with self._lock:
            return (list(self._cell_stats), self._locked,
                    self._zone3_frame, self._stamp_sec,
                    self._frame_total, self._roi_total)


# ═════════════════════════════════════════════════════════════════════
# Qt 窗口
# ═════════════════════════════════════════════════════════════════════

class CellStatWidget(QWidget):
    """单个格子控件: 大字显示主导颜色 + R/B/T 数值条."""

    def __init__(self, layer: int, col: int) -> None:
        super().__init__()
        self.setMinimumSize(170, 100)
        self._layer = layer
        self._col = col

        layout = QVBoxLayout(self)
        layout.setSpacing(1)
        layout.setContentsMargins(4, 3, 4, 3)

        # 标题 (格子名)
        self._title = QLabel(CELL_NAMES[layer][col])
        self._title.setAlignment(Qt.AlignCenter)
        self._title.setFont(QFont("", 10))
        self._title.setStyleSheet("font-weight: bold; color: #FFF;")
        layout.addWidget(self._title)

        # 颜色结果大字 (RED / BLUE / EMPTY / ?)
        self._result = QLabel("—")
        self._result.setAlignment(Qt.AlignCenter)
        self._result.setFont(QFont("", 18, QFont.Bold))
        layout.addWidget(self._result)

        # R / B 数值条
        bar = QHBoxLayout()
        bar.setSpacing(8)
        self._red_label = QLabel("R: 0")
        self._red_label.setAlignment(Qt.AlignCenter)
        self._red_label.setStyleSheet("color: #FF6060; font-weight: bold; font-size: 13px;")
        bar.addWidget(self._red_label)
        self._blue_label = QLabel("B: 0")
        self._blue_label.setAlignment(Qt.AlignCenter)
        self._blue_label.setStyleSheet("color: #6080FF; font-weight: bold; font-size: 13px;")
        bar.addWidget(self._blue_label)
        self._total_label = QLabel("T:0")
        self._total_label.setAlignment(Qt.AlignCenter)
        self._total_label.setStyleSheet("color: #888; font-size: 11px;")
        bar.addWidget(self._total_label)
        layout.addLayout(bar)

        self._default_style = (
            "background-color: #1E1E1E; border: 2px solid #444; border-radius: 6px;"
        )
        self.setStyleSheet(self._default_style)

    def update_stats(self, red: int, blue: int, total: int):
        """刷新显示, 根据主导颜色切换背景."""
        self._red_label.setText(f"R:{red}")
        self._blue_label.setText(f"B:{blue}")
        self._total_label.setText(f"T:{total}")

        # 动态纯净分门槛：绝对分 > 20，且大于总点数的 3%
        min_valid_score = max(20, int(total * 0.03))

        if total < EMPTY_THRESHOLD:
            self._result.setText("EMPTY")
            self._result.setStyleSheet("color: #666; font-size: 14px;")
            self.setStyleSheet(
                "background-color: #1E1E1E; border: 2px solid #444; border-radius: 6px;")
        elif red > blue and red >= blue * 1.5 and red >= min_valid_score:
            self._result.setText("🔴")
            self._result.setStyleSheet("color: #FF4040; font-size: 22px;")
            self.setStyleSheet(
                "background-color: #3A1818; border: 2px solid #FF4040; border-radius: 6px;")
        elif blue > red and blue >= red * 1.5 and blue >= min_valid_score:
            self._result.setText("🔵")
            self._result.setStyleSheet("color: #4080FF; font-size: 22px;")
            self.setStyleSheet(
                "background-color: #18183A; border: 2px solid #4080FF; border-radius: 6px;")
        else:
            self._result.setText("?")
            self._result.setStyleSheet("color: #AAA; font-size: 18px;")
            self.setStyleSheet(
                "background-color: #2A2A2A; border: 2px solid #888; border-radius: 6px;")


class KfsGridQtWindow(QMainWindow):
    """主窗口: 上 3x3 格子 + 下状态 + 底栏参数."""

    def __init__(self, node: KfsGridQtNode) -> None:
        super().__init__()
        self._node = node
        self.setWindowTitle("KFS Grid Color Detector")
        self.resize(620, 580)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setSpacing(4)
        layout.setContentsMargins(8, 6, 8, 6)

        # ── 主区域 3x3 格子面板 ──
        gw = QWidget()
        gl = QGridLayout(gw)
        gl.setSpacing(6)
        self._cells = []
        for li in range(2, -1, -1):       # 顶→底
            row = []
            for ci in range(3):
                w = CellStatWidget(li, ci)
                gl.addWidget(w, 2 - li, ci)
                row.append(w)
            self._cells.append(row)
        layout.addWidget(gw, 1)            # 1 = stretch, 填满

        # ── 底部状态栏 ──
        self._status = QLabel("⏳ 等待 zone3 TF 锁定...")
        self._status.setAlignment(Qt.AlignCenter)
        self._status.setStyleSheet(
            "font-size: 14px; padding: 6px; color: #FFA500; "
            "background-color: #222; border-radius: 4px;")
        layout.addWidget(self._status)

        # ── 底栏参数 ──
        self._summary = QLabel("")
        self._summary.setAlignment(Qt.AlignCenter)
        self._summary.setStyleSheet(
            "font-size: 11px; color: #AAA; padding: 4px; "
            "background-color: #1A1A1A; border-radius: 3px;")
        layout.addWidget(self._summary)

        # ── 刷新定时器 ──
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update)
        self._timer.start(int(round(1000.0 / GUI_REFRESH_HZ)))

    def _update(self):
        """定时读取 node 统计, 刷新 9 格 + 状态栏."""
        s = self._node.get_stats()
        stats, locked, z3f, stamp, ft, _roi = s
        if not locked:
            return
        tr = tb = ta = 0
        for li in range(3):
            for ci in range(3):
                idx = (2 - li) * 3 + ci
                r, bl, t = stats[idx]
                self._cells[li][ci].update_stats(r, bl, t)
                tr += r
                tb += bl
                ta += t

        # 判定全局主导色
        if tr > tb and tr >= tb * 1.3:
            color_tag = "🔴 偏红"
        elif tb > tr and tb >= tr * 1.3:
            color_tag = "🔵 偏蓝"
        else:
            color_tag = "⚪ 混合"

        self._status.setText(
            f"✅ {z3f}   |   "
            f"帧 {ft//1000}k → 格 {ta}   |   "
            f"R{tr}  B{tb}  {color_tag}")
        self._status.setStyleSheet(
            "font-size: 14px; padding: 6px; color: #4C4; "
            "background-color: #1A2A1A; border-radius: 4px;")

        self._summary.setText(
            f"累积 {ACCUMULATE_FRAMES}帧  |  "
            f"延伸 X{EXPAND_X:.2f} Y{EXPAND_Y:.2f} Z{EXPAND_Z:.2f}  |  "
            f"色彩净胜分 空<{EMPTY_THRESHOLD}")


# ═════════════════════════════════════════════════════════════════════
# 启动
# ═════════════════════════════════════════════════════════════════════

def _spin_ros(node: Node, stop_event: threading.Event) -> None:
    """后台 ROS spin 线程: 0.05s 超时轮询, stop_event 控制退出."""
    while rclpy.ok() and not stop_event.is_set():
        try:
            rclpy.spin_once(node, timeout_sec=0.05)
        except KeyboardInterrupt:
            break
        except Exception:
            if stop_event.is_set() or not rclpy.ok():
                break


def _install_sigint_handler(app: QApplication, stop_event: threading.Event) -> None:
    """Ctrl+C → 设 stop_event + quit Qt, 避免 KeyboardInterrupt 栈混乱."""
    def _on_sigint(_sig, _frame):
        stop_event.set()
        app.quit()
    signal.signal(signal.SIGINT, _on_sigint)


def main() -> int:
    """入口: 初始化 ROS → 创建 Node → 后台 spin → 启动 Qt → 等待退出 → 清理."""
    rclpy.init()
    node = KfsGridQtNode()
    stop_event = threading.Event()
    spin_thread = threading.Thread(target=_spin_ros, args=(node, stop_event), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    _install_sigint_handler(app, stop_event)
    window = KfsGridQtWindow(node)
    window.show()

    ec = 0
    try:
        ec = int(app.exec_())
    except KeyboardInterrupt:
        ec = 130
    finally:
        stop_event.set()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return ec


if __name__ == "__main__":
    raise SystemExit(main())
