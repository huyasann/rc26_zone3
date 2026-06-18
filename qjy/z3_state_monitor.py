#!/usr/bin/env python3
"""Z3 状态监视器 + 点云渲染。

订阅 /odin1/odometry_highfreq 检测平地/上坡/Z3平台。
订阅 /odin1/cloud_slam 提取坡道点云并可视化。
显示 Qt 窗口 + CSV 日志。
"""

from __future__ import annotations

import math
import os
import sys
from collections import deque
from datetime import datetime

import numpy as np
from scipy.ndimage import label as nd_label
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from PyQt5 import QtCore, QtGui, QtWidgets

# ── 坡道几何（blue_zone3_root 局部坐标） ──────────────────────
Z3_PLATFORM_Z = 0.40     # 平台相对地面高度
HISTOGRAM_BIN_WIDTH = 0.02


def quat_to_pitch(q) -> float:
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    return math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)


def quat_to_rpy(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw




# ══════════════════════════════════════════════════════════════════
# Qt 控件
# ══════════════════════════════════════════════════════════════════

class _RampBar(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._ratio = 0.0
        self.setFixedHeight(32)
        self.setMinimumWidth(260)

    def set_ratio(self, v: float):
        self._ratio = max(0.0, min(1.0, v))
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()
        r = h // 2 - 2
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor(40, 40, 40))
        p.drawRoundedRect(0, 2, w, h - 4, r, r)
        fill_w = int((w - 4) * self._ratio)
        if fill_w > 2 * r:
            c = QtGui.QColor(255, 180, 30) if self._ratio < 1.0 else QtGui.QColor(40, 200, 100)
            p.setBrush(c)
            p.drawRoundedRect(2, 4, fill_w, h - 8, r, r)
        p.setPen(QtGui.QPen(QtCore.Qt.white))
        p.setFont(QtGui.QFont("monospace", 10, QtGui.QFont.Bold))
        p.drawText(0, 0, w, h, QtCore.Qt.AlignCenter, f"{self._ratio * 100:.0f}%")


class _MonitorWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Z3 状态监视器")
        self.setFixedSize(430, 340)
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.WindowStaysOnTopHint)
        self.setStyleSheet("background:#1a1a1a; color:#ccc;")
        cw = QtWidgets.QWidget()
        self.setCentralWidget(cw)
        lay = QtWidgets.QVBoxLayout(cw)
        lay.setContentsMargins(16, 12, 16, 12)
        lay.setSpacing(6)

        self._state_label = QtWidgets.QLabel("标定基线中…")
        self._state_label.setAlignment(QtCore.Qt.AlignCenter)
        self._state_label.setFont(QtGui.QFont("sans-serif", 20, QtGui.QFont.Bold))
        self._state_label.setStyleSheet("color:#888;")
        lay.addWidget(self._state_label)

        self._bar = _RampBar()
        self._bar.setVisible(False)
        lay.addWidget(self._bar, alignment=QtCore.Qt.AlignCenter)

        grid = QtWidgets.QGridLayout()
        grid.setSpacing(3)
        font_val = QtGui.QFont("monospace", 11, QtGui.QFont.Bold)
        font_lbl = QtGui.QFont("sans-serif", 8)
        self._vals: dict[str, QtWidgets.QLabel] = {}
        for col, (key, lbl) in enumerate([
            ("z", "Z"), ("z_sm", "Z平滑"), ("pitch", "Pitch°"),
            ("pitch_d", "PitchΔ°"), ("z_rise", "Z上升"), ("gnd_z", "地面Z"),
        ]):
            hl = QtWidgets.QLabel(lbl)
            hl.setFont(font_lbl)
            hl.setStyleSheet("color:#666;")
            hl.setAlignment(QtCore.Qt.AlignCenter)
            grid.addWidget(hl, 0, col)
            vl = QtWidgets.QLabel("--")
            vl.setFont(font_val)
            vl.setStyleSheet("color:#eee;")
            vl.setAlignment(QtCore.Qt.AlignCenter)
            grid.addWidget(vl, 1, col)
            self._vals[key] = vl
        lay.addLayout(grid)

        self._diag = QtWidgets.QLabel("")
        self._diag.setFont(QtGui.QFont("monospace", 8))
        self._diag.setStyleSheet("color:#555;")
        self._diag.setAlignment(QtCore.Qt.AlignCenter)
        self._diag.setWordWrap(True)
        lay.addWidget(self._diag)

    def update_state(self, state: str, ramp_ratio: float, diag: str, vals: dict):
        style_map = {
            "flat":       ("🟢 平地",      "#4caf50", "#2e7d32"),
            "ramp":       ("🔺 上坡中",    "#ffb41e", "#b87800"),
            "platform":   ("🟦 Z3 平台",   "#42a5f5", "#1565c0"),
            "transition": ("·· 过渡中",    "#888888", "#444444"),
            "calib":      ("⏳ 标定基线",  "#666666", "#333333"),
        }
        label, fg, bg = style_map.get(state, style_map["calib"])
        self._state_label.setText(label)
        self._state_label.setStyleSheet(f"background:{bg}; color:{fg}; border-radius:8px; padding:6px;")
        self._bar.setVisible(state == "ramp")
        if state == "ramp":
            self._bar.set_ratio(ramp_ratio)
        for k, vk in [("z","z"),("z_sm","z_sm"),("pitch","pitch_deg"),("pitch_d","pitch_delta_deg"),("z_rise","z_rise"),("gnd_z","gnd_z")]:
            if k in self._vals and vk in vals:
                v = vals[vk]
                self._vals[k].setText(f"{v:+.3f}" if k not in ("pitch","pitch_d") else f"{v:+.1f}")
        self._diag.setText(diag)


# ══════════════════════════════════════════════════════════════════
# ROS2 Node
# ══════════════════════════════════════════════════════════════════

class Z3StateMonitor(Node):
    def __init__(self):
        super().__init__("z3_state_monitor")

        # ── 参数 ────────────────────────────────────────────────────
        self.declare_parameter("odom_topic", "/odin1/odometry_highfreq")
        self.declare_parameter("cloud_topic", "/odin1/cloud_slam")
        self.declare_parameter("baseline_frames", 20)
        self.declare_parameter("smooth_window", 8)
        self.declare_parameter("sustain_frames", 4)
        self.declare_parameter("pitch_tol_deg", 7.0)
        self.declare_parameter("climb_pitch_ratio", 1.4)
        self.declare_parameter("z_rise_thr", 0.03)
        self.declare_parameter("summit_pitch_tol_deg", 5.0)
        self.declare_parameter("platform_z_min", 0.25)
        self.declare_parameter("platform_z_ref", 0.45)
        self.declare_parameter("ground_min_points", 60)
        self.declare_parameter("cloud_cache_sec", 30.0)

        odom_topic = str(self.get_parameter("odom_topic").value)
        self._cloud_topic = str(self.get_parameter("cloud_topic").value)
        self._baseline_frames = int(self.get_parameter("baseline_frames").value)
        self._smooth_window = int(self.get_parameter("smooth_window").value)
        self._sustain_frames = int(self.get_parameter("sustain_frames").value)
        self._pitch_tol = math.radians(float(self.get_parameter("pitch_tol_deg").value))
        self._climb_pitch_thr = self._pitch_tol * float(self.get_parameter("climb_pitch_ratio").value)
        self._z_rise_thr = float(self.get_parameter("z_rise_thr").value)
        self._summit_pitch_tol = math.radians(float(self.get_parameter("summit_pitch_tol_deg").value))
        self._platform_z_min = float(self.get_parameter("platform_z_min").value)
        self._platform_z_ref = float(self.get_parameter("platform_z_ref").value)
        self._ground_min_points = int(self.get_parameter("ground_min_points").value)
        self._cloud_cache_sec = float(self.get_parameter("cloud_cache_sec").value)

        # ── 基线标定 ────────────────────────────────────────────────
        self._z_buf: deque[float] = deque(maxlen=self._baseline_frames)
        self._pitch_buf: deque[float] = deque(maxlen=self._baseline_frames)
        self._baseline_ok = False
        self._z_ground = 0.0
        self._pitch_ground = 0.0

        # ── 上坡检测 ────────────────────────────────────────────────
        self._z_smooth_buf: deque[float] = deque(maxlen=self._smooth_window)
        self._dz_dt_buf: deque[float] = deque(maxlen=10)
        self._z_sm = 0.0
        self._z_sm_prev = 0.0
        self._dz_dt = 0.0
        self._climb_counter = 0
        self._ground_counter = 0
        self._platform_counter = 0

        # ── 趋势检测（新增，抗干扰） ──────────────────────────────────
        self._pd_trend_buf: deque[float] = deque(maxlen=8)      # pd_abs 趋势窗口
        self._z_trend_buf: deque[float] = deque(maxlen=8)       # z_sm 趋势窗口

        # ── 状态 ────────────────────────────────────────────────────
        self._state: str = "calib"
        self._pending_state: str | None = None
        self._pending_count: int = 0
        self._ramp_start_z: float | None = None
        self._ramp_peak_z: float = 0.0
        self._platform_peak_z: float = 0.0

        # ── Odom 数据 ───────────────────────────────────────────────
        self._z_now = 0.0
        self._pitch_now = 0.0
        self._roll_now = 0.0
        self._yaw_now = 0.0
        self._x_now = 0.0
        self._y_now = 0.0
        self._last_t = 0.0
        self._odom_traj: deque[dict] = deque(maxlen=200)

        # ── 点云 ────────────────────────────────────────────────────
        self._cloud_buf: deque[dict] = deque()
        self._ground_z_cloud: float | None = None
        self._ground_z_samples: deque[float] = deque(maxlen=20)

        # ★ 坡道/平台点云：检测到后保存，持续发布
        self._ramp_cloud: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None
        self._ramp_cloud_yaw: float = 0.0
        self._ramp_capture_x: float = 0.0
        self._ramp_capture_y: float = 0.0
        self._platform_cloud: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None
        self._platform_cloud_yaw: float = 0.0
        self._platform_capture_delay = 0
        self._edge_cloud: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None
        self._ramp_line_cloud: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None
        self._plat_line_cloud: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None

        # ── 发布器 ──────────────────────────────────────────────────
        self._debug_cloud_pub = self.create_publisher(
            PointCloud2, "/rc26/zone3/ramp_monitor/cloud_debug", 10)

        # ── TF ──────────────────────────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── 订阅 & 定时器 ───────────────────────────────────────────
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, odom_topic, self._odom_cb, sensor_qos)
        self.create_subscription(PointCloud2, self._cloud_topic, self._cloud_cb, sensor_qos)
        self._timer = self.create_timer(0.05, self._tick)
        self._tick_idx = 0

        self._win = _MonitorWindow()
        self._win.show()
        self.get_logger().info(
            f"climb_thr={math.degrees(self._climb_pitch_thr):.1f}°  "
            f"summit_tol={math.degrees(self._summit_pitch_tol):.1f}°"
        )

    # ══════════════════════════════════════════════════════════════
    # 里程计回调
    # ══════════════════════════════════════════════════════════════

    def _odom_cb(self, msg: Odometry):
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        p = msg.pose.pose.position
        roll, pitch, yaw = quat_to_rpy(msg.pose.pose.orientation)
        self._z_now = float(p.z)
        self._pitch_now = pitch
        self._roll_now = roll
        self._yaw_now = yaw
        self._x_now = float(p.x)
        self._y_now = float(p.y)
        self._last_t = t
        self._odom_traj.append({"t": t, "x": self._x_now, "y": self._y_now, "z": self._z_now})

        if not self._baseline_ok:
            self._z_buf.append(self._z_now)
            self._pitch_buf.append(pitch)
            if len(self._z_buf) >= self._baseline_frames:
                self._z_ground = sum(self._z_buf) / len(self._z_buf)
                self._pitch_ground = sum(self._pitch_buf) / len(self._pitch_buf)
                self._baseline_ok = True
                self._z_sm = 0.0
                self._z_sm_prev = 0.0
                self.get_logger().info(
                    f"基线标定 z_ground={self._z_ground:.3f} "
                    f"pitch_ground={math.degrees(self._pitch_ground):+.1f}°"
                )
            return

    # ══════════════════════════════════════════════════════════════
    # 状态检测（含趋势）
    # ══════════════════════════════════════════════════════════════

    def _update_state(self) -> tuple[str, dict]:
        z_rise = self._z_now - self._z_ground
        pitch_delta = self._pitch_now - self._pitch_ground
        pd_abs = abs(pitch_delta)

        # Z 平滑
        self._z_smooth_buf.append(z_rise)
        z_sm = sum(self._z_smooth_buf) / len(self._z_smooth_buf)
        dz_dt = (z_sm - self._z_sm_prev) / 0.05
        self._z_sm_prev = z_sm
        self._dz_dt_buf.append(dz_dt)
        self._dz_dt = sum(self._dz_dt_buf) / len(self._dz_dt_buf)
        self._z_sm = z_sm

        # ★ 趋势计算：pitch_delta 和 z_sm 在滑动窗口内的变化量
        #   用于抑制升降机/噪声导致的假触发（只约束进入 ramp，不约束维持）
        pitch_delta_val = pitch_delta  # 带符号，可判断升降方向
        self._pd_trend_buf.append(pd_abs)
        self._z_trend_buf.append(z_sm)
        pd_trend = self._pd_trend_buf[-1] - self._pd_trend_buf[0] if len(self._pd_trend_buf) >= 4 else 0.0
        z_trend = self._z_trend_buf[-1] - self._z_trend_buf[0] if len(self._z_trend_buf) >= 4 else 0.0

        climb_thr_deg = math.degrees(self._climb_pitch_thr)
        summit_tol_deg = math.degrees(self._summit_pitch_tol)

        # ═══════════════════════════════════════════════════════════
        # 上坡检测：趋势约束只用于进入 ramp（抑制升降机假触发）
        # ═══════════════════════════════════════════════════════════
        on_flat = z_sm < self._z_rise_thr * 0.5 and pd_abs < self._pitch_tol

        # 是否首次进入 ramp？需要趋势确认
        # 是否首次进入 ramp？pd 超过阈值且 Z 持续上升即确认
        entering_ramp = (
            self._state != "ramp"
            and pd_abs > self._climb_pitch_thr
            and z_sm > self._z_rise_thr
            and z_trend > 0.0                         # Z 持续上升（不要求幅度）
        )
        # 在 ramp 中：宽松维持
        staying_ramp = (
            self._state == "ramp"
            and pd_abs > self._climb_pitch_thr * 0.6  # 退出阈值 9.8*0.6≈5.9°
            and z_sm > self._z_rise_thr * 0.5
        )
        on_slope = entering_ramp or staying_ramp

        # ═══════════════════════════════════════════════════════════
        # 登顶/平台检测（不限于 ramp 状态）
        # ═══════════════════════════════════════════════════════════
        on_platform_cond = pd_abs < self._summit_pitch_tol and z_sm >= self._platform_z_min

        if on_platform_cond:
            self._platform_counter += 1
        else:
            self._platform_counter = 0

        if on_slope:
            self._climb_counter += 1
            self._ground_counter = 0
        else:
            self._climb_counter = 0
            self._ground_counter = self._ground_counter + 1 if on_flat else 0

        # 状态判定
        if self._platform_counter >= self._sustain_frames:
            raw = "platform"
        elif self._climb_counter >= self._sustain_frames:
            raw = "ramp"
        elif self._ground_counter >= self._sustain_frames:
            raw = "flat"
        else:
            raw = "transition"

        d = {
            "z": self._z_now, "z_ground": self._z_ground,
            "z_rise": z_rise, "z_sm": z_sm, "dz_dt": self._dz_dt,
            "pitch_deg": math.degrees(self._pitch_now),
            "pitch_ground_deg": math.degrees(self._pitch_ground),
            "pitch_delta_deg": math.degrees(pitch_delta),
            "pd_abs_deg": math.degrees(pd_abs),
            "climb_pitch_thr_deg": climb_thr_deg,
            "summit_pitch_tol_deg": summit_tol_deg,
            "on_slope": on_slope, "on_flat": on_flat,
            "on_platform_cond": on_platform_cond,
            "pd_trend": math.degrees(pd_trend),
            "z_trend": z_trend,
            "climb_cnt": self._climb_counter,
            "ground_cnt": self._ground_counter,
            "platform_cnt": self._platform_counter,
            "gnd_z": self._ground_z_cloud or self._z_ground,
        }
        return raw, d

    # ══════════════════════════════════════════════════════════════
    # 点云回调
    # ══════════════════════════════════════════════════════════════

    def _cloud_cb(self, msg: PointCloud2):
        try:
            x, y, z = self._parse_xyz(msg)
            ox, oy, oz = self._to_odom_xyz(msg.header, x, y, z)
        except Exception:
            return
        if ox is None:
            return

        finite = np.isfinite(ox) & np.isfinite(oy) & np.isfinite(oz)
        ox, oy, oz = ox[finite], oy[finite], oz[finite]
        if len(oz) == 0:
            return

        step = max(1, len(oz) // 2000)
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self._cloud_buf.append({"t": t, "x": ox[::step], "y": oy[::step], "z": oz[::step]})
        while self._cloud_buf and t - self._cloud_buf[0]["t"] > self._cloud_cache_sec:
            self._cloud_buf.popleft()
        # 最多保留 800 帧（~40s @20Hz）
        while len(self._cloud_buf) > 800:
            self._cloud_buf.popleft()

        # 在 flat 状态时积累 ground Z 参考
        if self._state == "flat" and self._baseline_ok:
            self._accumulate_ground_z(ox, oy, oz)

    def _accumulate_ground_z(self, ox, oy, oz):
        """在 flat 时从点云直方图提取地面 Z"""
        # 简单 ROI：以当前位置为中心，5×5m 范围
        dx = ox - self._x_now
        dy = oy - self._y_now
        roi = (abs(dx) < 2.5) & (abs(dy) < 2.5) & (oz > self._z_ground - 0.15) & (oz < self._z_ground + 0.15)
        if int(roi.sum()) < self._ground_min_points:
            return

        z_peak = self._detect_z_peak(oz[roi], lowest=True)
        if z_peak is not None:
            self._ground_z_samples.append(z_peak)
            if len(self._ground_z_samples) >= self._ground_z_samples.maxlen:
                arr = np.asarray(self._ground_z_samples)
                if arr.max() - arr.min() < 0.04:
                    self._ground_z_cloud = float(np.median(arr))

    # ══════════════════════════════════════════════════════════════
    # 坡道/平台点云提取（参考 Zone2 2D 直方图密度过滤思路）
    # ══════════════════════════════════════════════════════════════

    def _filter_dense_cluster_2d(self, cx, cy, cz, bin_size=0.08, min_density=4):
        """2D 直方图 → 找密度最大的连通区域 → 滤除孤立点"""
        if len(cx) < 50:
            return cx, cy, cz  # 点太少，跳过过滤
        # 构建 2D 网格
        x_bins = np.floor(cx / bin_size).astype(np.int32)
        y_bins = np.floor(cy / bin_size).astype(np.int32)
        x_min, y_min = x_bins.min(), y_bins.min()
        x_bins -= x_min; y_bins -= y_min
        nx, ny = x_bins.max() + 1, y_bins.max() + 1
        # 用 bincount 构建 2D 密度图
        flat = x_bins * ny + y_bins
        counts = np.bincount(flat, minlength=int(nx * ny)).reshape(int(nx), int(ny))
        dense = counts >= min_density
        # 找密度最大的连通区域（取覆盖点数最多的）
        labeled, n_feat = nd_label(dense)
        if n_feat == 0:
            return cx, cy, cz
        best = 0; best_label = 0
        for lbl in range(1, n_feat + 1):
            n_cells = int((labeled == lbl).sum())
            if n_cells > best:
                best = n_cells
                best_label = lbl
        # 用 best_label 做 mask
        cell_labels = x_bins * ny + y_bins
        cell_dense_label = labeled[x_bins.clip(0, nx-1), y_bins.clip(0, ny-1)]
        in_cluster = cell_dense_label == best_label
        if int(in_cluster.sum()) < 20:
            return cx, cy, cz
        return cx[in_cluster], cy[in_cluster], cz[in_cluster]

    @staticmethod
    def _largest_connected_component_mask_2d(ax, ay, *, bin_size=0.06, min_density=3, min_points=40):
        """在 2D 栅格上保留点数最多的连通区域，用于裁掉侧边细长尾巴。"""
        if len(ax) < min_points:
            return np.ones(len(ax), dtype=bool)
        x_bins = np.floor(ax / bin_size).astype(np.int32)
        y_bins = np.floor(ay / bin_size).astype(np.int32)
        x_min, y_min = x_bins.min(), y_bins.min()
        x_bins -= x_min
        y_bins -= y_min
        nx, ny = x_bins.max() + 1, y_bins.max() + 1
        flat = x_bins * ny + y_bins
        counts = np.bincount(flat, minlength=int(nx * ny)).reshape(int(nx), int(ny))
        dense = counts >= min_density
        labeled, n_feat = nd_label(dense)
        if n_feat == 0:
            return np.ones(len(ax), dtype=bool)

        best_points = 0
        best_label = 0
        for lbl in range(1, n_feat + 1):
            comp = labeled == lbl
            comp_points = int(counts[comp].sum())
            if comp_points > best_points:
                best_points = comp_points
                best_label = lbl
        if best_label == 0 or best_points < min_points:
            return np.ones(len(ax), dtype=bool)
        return labeled[x_bins.clip(0, nx - 1), y_bins.clip(0, ny - 1)] == best_label

    @staticmethod
    def _trim_edge_lateral_strip_mask(
        lx,
        ly,
        *,
        bin_size=0.04,
        density_ratio=1.8,
        max_width=0.20,
        min_strip_points=80,
        min_lx_span=0.45,
    ):
        """裁掉贴在坡面边缘的窄高密条带，典型是内侧立面并到坡面线里。"""
        if len(ly) < min_strip_points:
            return np.ones(len(ly), dtype=bool)
        lo = np.floor(float(ly.min()) / bin_size) * bin_size
        hi = np.ceil(float(ly.max()) / bin_size) * bin_size
        edges = np.arange(lo, hi + bin_size, bin_size)
        hist, _ = np.histogram(ly, bins=edges)
        nonzero = hist[hist > 0]
        if len(nonzero) < 4:
            return np.ones(len(ly), dtype=bool)

        baseline = max(4.0, float(np.median(nonzero)))
        keep = np.ones(len(ly), dtype=bool)
        for side in ("left", "right"):
            edge_bins = range(len(hist)) if side == "left" else range(len(hist) - 1, -1, -1)
            run = []
            for i in edge_bins:
                if hist[i] <= 0:
                    if run:
                        break
                    continue
                center = 0.5 * (edges[i] + edges[i + 1])
                edge_dist = (center - edges[0]) if side == "left" else (edges[-1] - center)
                if edge_dist > max_width:
                    break
                if hist[i] >= baseline * density_ratio:
                    run.append(i)
                    continue
                if run:
                    break
            if not run:
                continue
            strip_lo = edges[min(run)]
            strip_hi = edges[max(run) + 1]
            strip = (ly >= strip_lo) & (ly < strip_hi)
            n_strip = int(strip.sum())
            if n_strip < min_strip_points:
                continue
            lx_span = float(np.percentile(lx[strip], 99) - np.percentile(lx[strip], 1))
            ly_span = float(np.percentile(ly[strip], 99) - np.percentile(ly[strip], 1))
            if lx_span < min_lx_span or ly_span > max_width:
                continue
            keep[strip] = False
        return keep

    def _ransac_ramp_plane(self, cx, cy, cz, n_iter=60, inlier_dist=0.06, min_inliers=60):
        """RANSAC 平面拟合：坡道面为 inlier，侧边墙壁点为 outlier"""
        n = len(cx)
        if n < min_inliers:
            return cx, cy, cz
        dx, dy = cx - self._x_now, cy - self._y_now
        c, s = math.cos(self._yaw_now), math.sin(self._yaw_now)
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        # 要拟合 z = a*lx + b*ly + c0
        pts = np.column_stack((lx, ly, cz))
        best_inliers = 0
        best_mask = None
        # 预生成随机索引
        rng = np.random.RandomState(42)
        for _ in range(n_iter):
            idx = rng.choice(n, 3, replace=False)
            p = pts[idx]
            v1 = p[1] - p[0]
            v2 = p[2] - p[0]
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm < 1e-8:
                continue
            normal /= norm
            d = -np.dot(normal, p[0])
            # 所有点到平面的距离
            dist = np.abs(pts @ normal + d)
            inlier = dist <= inlier_dist
            n_in = int(inlier.sum())
            if n_in > best_inliers:
                best_inliers = n_in
                best_mask = inlier
        if best_mask is None or best_inliers < min_inliers:
            return cx, cy, cz
        n_filtered = n - best_inliers
        if n_filtered > 10:
            self.get_logger().info(f"    RANSAC平面: inlier={best_inliers} 滤除={n_filtered}")
        return cx[best_mask], cy[best_mask], cz[best_mask]

    def _extract_under_robot(self, z_min: float, z_max: float, label: str) -> tuple | None:
        """从点云缓存提取机器人下方点云，经 ROI + Z + 2D 密度 + base_link Y 过滤"""
        if not self._cloud_buf:
            return None
        xs, ys, zs = [], [], []
        for item in self._cloud_buf:
            xs.append(item["x"]); ys.append(item["y"]); zs.append(item["z"])
        cx, cy, cz = np.concatenate(xs), np.concatenate(ys), np.concatenate(zs)
        if len(cz) == 0:
            return None

        # ① XY 空间 ROI: 机器人周围 1.0m（坡道宽 1.55m，足够了）
        nearby = (np.abs(cx - self._x_now) < 1.0) & (np.abs(cy - self._y_now) < 1.0)
        if int(nearby.sum()) < 30:
            return None
        cx, cy, cz = cx[nearby], cy[nearby], cz[nearby]

        # ② Z 范围过滤
        in_z = (cz >= z_min) & (cz <= z_max)
        if int(in_z.sum()) < 20:
            return None
        cx, cy, cz = cx[in_z].copy(), cy[in_z].copy(), cz[in_z].copy()

        # ③ base_link Y 过滤：用车体 yaw 旋转到车体坐标系，只保留 |ly| < 0.6m
        dx, dy = cx - self._x_now, cy - self._y_now
        c, s = math.cos(self._yaw_now), math.sin(self._yaw_now)
        ly = -s * dx + c * dy  # base_link 左侧方向
        in_width = np.abs(ly) < 0.6
        if int(in_width.sum()) < 10:
            return None
        cx, cy, cz = cx[in_width], cy[in_width], cz[in_width]

        # ④ 2D 直方图密度聚类（参考 Zone2 思路）：滤除侧边孤立物体
        cx, cy, cz = self._filter_dense_cluster_2d(cx, cy, cz)

        # ⑤ 对 RAMP 额外做一次车体系横向贴边条带裁剪。
        #    这一步比 2D 线筛更早，专门处理“内侧立面整条贴在白云边缘”的情况。
        if "RAMP" in label:
            dx, dy = cx - self._x_now, cy - self._y_now
            c, s = math.cos(self._yaw_now), math.sin(self._yaw_now)
            lx_body = c * dx + s * dy
            ly_body = -s * dx + c * dy
            strip_mask = self._trim_edge_lateral_strip_mask(
                lx_body,
                ly_body,
                bin_size=0.04,
                density_ratio=1.6,
                max_width=0.16,
                min_strip_points=60,
                min_lx_span=0.40,
            )
            removed_strip = int((~strip_mask).sum())
            if removed_strip > 0 and int(strip_mask.sum()) >= 30:
                cx, cy, cz = cx[strip_mask], cy[strip_mask], cz[strip_mask]
                self.get_logger().info(f"    RAMP边条预裁剪: 裁掉 {removed_strip} 点")

        # ⑥ RANSAC 平面拟合：坡道面为 inlier，侧边垂直墙壁为 outlier
        if "RAMP" in label:
            cx, cy, cz = self._ransac_ramp_plane(cx, cy, cz)

        # ⑦ 上限 3000 点
        if len(cz) > 3000:
            idx = np.linspace(0, len(cz)-1, 3000, dtype=int)
            cx, cy, cz = cx[idx], cy[idx], cz[idx]

        n = len(cx)
        self.get_logger().info(f"[{label}] 提取 {n} 点  z=[{cz.min():.2f},{cz.max():.2f}]")
        # 保存 debug CSV
        self._save_capture_csv(label, cx, cy, cz)
        return (cx, cy, cz)

    def _save_capture_csv(self, label: str, cx, cy, cz):
        """保存提取的点云到 CSV 供后续分析"""
        try:
            log_dir = "/home/inkc/inkc/Rc2026/src/tmp/tmp/logs"
            os.makedirs(log_dir, exist_ok=True)
            csv_path = f"{log_dir}/cloud_{label}_{datetime.now().strftime('%H%M%S')}.csv"
            import csv as _csv
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                w = _csv.writer(f)
                w.writerow(["x", "y", "z"])
                for i in range(min(len(cx), 2000)):
                    w.writerow([f"{cx[i]:.4f}", f"{cy[i]:.4f}", f"{cz[i]:.4f}"])
        except Exception:
            pass

    def _capture_ramp_cloud(self):
        """在进入 ramp 状态时捕获坡道点云（白色）"""
        gz = self._z_ground
        cloud = self._extract_under_robot(gz + 0.03, gz + 0.50, "RAMP")
        if cloud is not None:
            self._ramp_cloud = cloud
            self._ramp_cloud_yaw = self._yaw_now
            self._ramp_capture_x = self._x_now
            self._ramp_capture_y = self._y_now

    def _capture_platform_cloud(self):
        """在进入 platform 状态时捕获平台点云（青色）"""
        gz = self._z_ground
        # Z 窗口缩窄到 [0.28, 0.50]，排除平台上方结构物
        raw = self._extract_under_robot(gz + 0.28, gz + 0.50, "PLAT_RAW")
        if raw is None:
            return
        px, py, pz = raw
        z_peak = self._detect_z_peak(pz, lowest=False)
        if z_peak is None:
            return
        on_plane = np.abs(pz - z_peak) <= 0.03
        if int(on_plane.sum()) < 20:
            return
        px, py, pz = px[on_plane], py[on_plane], pz[on_plane]
        if len(pz) > 3000:
            idx = np.linspace(0, len(pz)-1, 3000, dtype=int)
            px, py, pz = px[idx], py[idx], pz[idx]
        self._platform_cloud = (px, py, pz)
        self._platform_cloud_yaw = self._yaw_now
        self.get_logger().info(f"[PLAT] z_peak={z_peak:.3f} 范围=[{pz.min():.2f},{pz.max():.2f}] 保留 {len(pz)} 点")
        if self._ramp_cloud is not None:
            self._refine_clouds_2d()

    # ══════════════════════════════════════════════════════════════
    # 2D 压缩过滤（参考 Zone2 2D 直方图 → 线拟合）
    # ══════════════════════════════════════════════════════════════

    def _refine_clouds_2d(self):
        """用 2D 轮廓线过滤坡道和平台点云：只保留投影到线附近的点，内侧立面被滤除。"""
        gz = self._z_ground
        yaw = self._ramp_cloud_yaw

        rx, ry, rz = self._ramp_cloud
        px, py, pz = self._platform_cloud

        # ── 旋转到 forward 坐标系 ──
        c, s = math.cos(yaw), math.sin(yaw)
        lx_r = c * rx + s * ry
        lx_p = c * px + s * py

        # ── 坡道 2D 轮廓线拟合 ──
        # 在 (lx, Z) 空间建 2D 直方图，每 lx bin 取密度最高 Z
        k_r, b_r, z_plat = None, None, None  # 初始化，避免 NameError

        # ── 坡道 2D RANSAC 线拟合（约束斜率逼近标准15°） ──
        SLOPE_15 = math.tan(math.radians(15.0))  # 0.268
        SLOPE_MIN = math.tan(math.radians(10.0))  # 0.176
        SLOPE_MAX = math.tan(math.radians(20.0))  # 0.364

        in_r = (rz >= gz + 0.03) & (rz <= gz + 0.50)
        if int(in_r.sum()) >= 30:
            lx_f, z_f = lx_r[in_r], rz[in_r]
            best_k, best_b, best_in = 0.0, 0.0, 0
            rng = np.random.RandomState(42)
            for _ in range(80):
                idx = rng.choice(len(lx_f), 2, replace=False)
                x1, z1 = lx_f[idx[0]], z_f[idx[0]]
                x2, z2 = lx_f[idx[1]], z_f[idx[1]]
                if abs(x2 - x1) < 1e-6: continue
                k = (z2 - z1) / (x2 - x1)
                b = z1 - k * x1
                # 约束斜率在坡道范围内
                if abs(k) < SLOPE_MIN or abs(k) > SLOPE_MAX: continue
                inlier = np.abs(z_f - (lx_f * k + b)) <= 0.05
                n_in = int(inlier.sum())
                if n_in > best_in: best_in = n_in; best_k, best_b = k, b
            # 若 RANSAC 没找到合格斜率的线，用标准 15°
            if best_in < 30 or abs(best_k) < SLOPE_MIN or abs(best_k) > SLOPE_MAX:
                best_k = SLOPE_15
                # 固定斜率，优化 b：取中位 (z - k*lx)
                best_b = float(np.median(z_f - lx_f * best_k))
                inlier = np.abs(z_f - (lx_f * best_k + best_b)) <= 0.05
                best_in = int(inlier.sum())
            if best_in >= 30:
                k_r, b_r = best_k, best_b
                keep = np.abs(rz - (lx_r * k_r + b_r)) <= 0.06
                if int(keep.sum()) >= 50:
                    rx_keep, ry_keep, rz_keep = rx[keep], ry[keep], rz[keep]
                    lx_keep = lx_r[keep]
                    ly_keep = -s * rx_keep + c * ry_keep
                    strip_mask = self._trim_edge_lateral_strip_mask(lx_keep, ly_keep)
                    removed_strip = int((~strip_mask).sum())
                    if int(strip_mask.sum()) >= 50:
                        rx_keep, ry_keep, rz_keep = (
                            rx_keep[strip_mask], ry_keep[strip_mask], rz_keep[strip_mask]
                        )
                        lx_keep = lx_keep[strip_mask]
                        ly_keep = ly_keep[strip_mask]
                    main_mask = self._largest_connected_component_mask_2d(
                        lx_keep, ly_keep, bin_size=0.06, min_density=3, min_points=80)
                    removed_tail = int((~main_mask).sum())
                    if int(main_mask.sum()) >= 50:
                        rx_keep, ry_keep, rz_keep = (
                            rx_keep[main_mask], ry_keep[main_mask], rz_keep[main_mask]
                        )
                        lx_keep = lx_keep[main_mask]
                    self._ramp_cloud = (rx_keep, ry_keep, rz_keep)
                    rmse = float(np.sqrt(np.mean((rz_keep - (lx_keep * k_r + b_r))**2)))
                    self.get_logger().info(
                        f"[2D] Ramp线: k={k_r:.3f} b={b_r:.3f} "
                        f"inlier={best_in}/{len(lx_f)} rmse={rmse:.3f} "
                        f"保留 {len(rz_keep)} 点 侧条裁掉 {removed_strip} 点 尾巴裁掉 {removed_tail} 点"
                    )

        # ── 平台 Z 先验估计 ──
        z_plat = None
        in_p = (pz >= gz + 0.25) & (pz <= gz + 0.55)
        if int(in_p.sum()) >= 20:
            z_plat = self._detect_z_peak(pz[in_p], lowest=False)

        # ── 坡顶边沿：两条线的交点 ──
        cross_lx = (z_plat - b_r) / k_r if (k_r is not None and z_plat is not None
                                              and abs(k_r) > 1e-6) else float('nan')

        # ── 平台 2D 空间约束：只保留坡顶边沿附近 lx ∈ [cross_lx, cross_lx+0.6] ──
        if np.isfinite(cross_lx) and z_plat is not None:
            lx_pf = c * px + s * py  # 平台点投影到 forward
            near_edge = (lx_pf >= cross_lx - 0.5) & (lx_pf <= cross_lx + 2.5) & (np.abs(pz - z_plat) <= 0.04)
            if int(near_edge.sum()) >= 15:
                self._platform_cloud = (px[near_edge], py[near_edge], pz[near_edge])
                self.get_logger().info(
                    f"[2D] 平台线: Z={z_plat:.3f} "
                    f"lx=[{lx_pf[near_edge].min():.2f},{lx_pf[near_edge].max():.2f}] "
                    f"保留 {int(near_edge.sum())} 点"
                )

        # ── 坡顶边沿：红色点云 ──
        if np.isfinite(cross_lx) and z_plat is not None:
            # 用坡道点云的 ly 宽度（与白色点云同宽）
            ly_r = -s * rx + c * ry
            ly_lo, ly_hi = float(np.percentile(ly_r, 5)), float(np.percentile(ly_r, 95))
            ly_span = max(ly_hi - ly_lo, 0.5)  # 至少 0.5m
            ly_edge = np.linspace(ly_lo, ly_hi, 50, dtype=np.float64)
            lx_edge = np.full_like(ly_edge, cross_lx)
            z_edge = np.full_like(ly_edge, z_plat)
            # 转回 odom 坐标系（lx 是绝对 odom 坐标的 yaw 旋转，直接逆旋转）
            c_, s_ = math.cos(yaw), math.sin(yaw)
            ex = c_ * lx_edge - s_ * ly_edge
            ey = s_ * lx_edge + c_ * ly_edge
            # 用交点处实际点云 Z 修正
            keep_near = (np.abs(lx_r - cross_lx) < 0.15) & (np.abs(rz - z_plat) < 0.08)
            if int(keep_near.sum()) >= 10:
                z_edge[:] = float(np.median(rz[keep_near]))
            self._edge_cloud = (ex, ey, z_edge)
            # 位置诊断：各云中位 odom 位置
            rx_m, ry_m = np.median(rx), np.median(ry)
            px_m, py_m = np.median(px), np.median(py)
            ex_m, ey_m = np.median(ex), np.median(ey)
            self.get_logger().info(
                f"[EDGE] lx={cross_lx:.2f}m Z={z_edge[0]:.3f} "
                f"白色=({rx_m:.2f},{ry_m:.2f}) 红=({ex_m:.2f},{ey_m:.2f}) "
                f"青=({px_m:.2f},{py_m:.2f}) yaw={math.degrees(yaw):.0f}°"
            )

            # ── 坡道拟合线 (黄色，从坡底到交点) ──
            lx_min = min(lx_r.min(), lx_p.min())
            lx_range = np.linspace(max(lx_min, -1.0), cross_lx, 80, dtype=np.float64)
            z_ramp_line = k_r * lx_range + b_r
            ly_line = np.zeros_like(lx_range)
            rl_x = c_ * lx_range - s_ * ly_line
            rl_y = s_ * lx_range + c_ * ly_line
            self._ramp_line_cloud = (rl_x, rl_y, z_ramp_line)

            # ── 平台拟合线 (绿色，从交点到坡顶) ──
            lx_range2 = np.linspace(cross_lx, max(lx_r.max(), lx_p.max(), cross_lx + 1.0), 40)
            z_plat_line = np.full_like(lx_range2, z_plat)
            pl_x = c_ * lx_range2 - s_ * np.zeros_like(lx_range2)
            pl_y = s_ * lx_range2 + c_ * np.zeros_like(lx_range2)
            self._plat_line_cloud = (pl_x, pl_y, z_plat_line)

        # ── 保存 2D 轮廓 CSV ──
        try:
            import csv as _csv
            ts = datetime.now().strftime('%H%M%S')
            path = f"/home/inkc/inkc/Rc2026/src/tmp/tmp/logs/profile2d_{ts}.csv"
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = _csv.writer(f)
                w.writerow(["lx","z","type"])
                for i in range(min(len(lx_r), 1000)):
                    w.writerow([f"{lx_r[i]:.4f}",f"{rz[i]:.4f}","ramp"])
                for i in range(min(len(lx_p), 1000)):
                    w.writerow([f"{lx_p[i]:.4f}",f"{pz[i]:.4f}","plat"])
        except Exception:
            pass

    def _publish_debug_cloud(self):
        """发布存储的坡道点云（白）+ 平台点云（青）"""
        r_parts, g_parts, b_parts = [], [], []
        x_parts, y_parts, z_parts = [], [], []

        if self._ramp_cloud is not None:
            rx, ry, rz = self._ramp_cloud
            x_parts.append(rx); y_parts.append(ry); z_parts.append(rz)
            r_parts.append(np.full(len(rx), 255, dtype=np.uint8))
            g_parts.append(np.full(len(rx), 255, dtype=np.uint8))
            b_parts.append(np.full(len(rx), 255, dtype=np.uint8))

        if self._platform_cloud is not None:
            px, py, pz = self._platform_cloud
            x_parts.append(px); y_parts.append(py); z_parts.append(pz)
            r_parts.append(np.full(len(px), 0, dtype=np.uint8))
            g_parts.append(np.full(len(px), 220, dtype=np.uint8))
            b_parts.append(np.full(len(px), 255, dtype=np.uint8))

        # 黄色 = 坡道拟合线 z=k*lx+b
        if self._ramp_line_cloud is not None:
            rlx, rly, rlz = self._ramp_line_cloud
            x_parts.append(rlx); y_parts.append(rly); z_parts.append(rlz)
            r_parts.append(np.full(len(rlx), 255, dtype=np.uint8))
            g_parts.append(np.full(len(rlx), 200, dtype=np.uint8))
            b_parts.append(np.full(len(rlx), 0, dtype=np.uint8))

        # 绿色 = 平台拟合线 Z=const
        if self._plat_line_cloud is not None:
            plx, ply, plz = self._plat_line_cloud
            x_parts.append(plx); y_parts.append(ply); z_parts.append(plz)
            r_parts.append(np.full(len(plx), 0, dtype=np.uint8))
            g_parts.append(np.full(len(plx), 255, dtype=np.uint8))
            b_parts.append(np.full(len(plx), 30, dtype=np.uint8))

        # 红色 = 坡顶边沿线（两条线交点）
        if self._edge_cloud is not None:
            ex, ey, ez = self._edge_cloud
            x_parts.append(ex); y_parts.append(ey); z_parts.append(ez)
            r_parts.append(np.full(len(ex), 255, dtype=np.uint8))
            g_parts.append(np.full(len(ex), 30, dtype=np.uint8))
            b_parts.append(np.full(len(ex), 30, dtype=np.uint8))

        if not x_parts:
            return

        cx = np.concatenate(x_parts)
        cy = np.concatenate(y_parts)
        cz = np.concatenate(z_parts)
        r = np.concatenate(r_parts)
        g = np.concatenate(g_parts)
        b = np.concatenate(b_parts)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "odom"
        self._debug_cloud_pub.publish(self._make_rgb_cloud(header, cx, cy, cz, r, g, b))

    # ══════════════════════════════════════════════════════════════
    # Root 位姿估计（TF 或 odom 轨迹）
    # ══════════════════════════════════════════════════════════════

    # ══════════════════════════════════════════════════════════════
    # 主循环
    # ══════════════════════════════════════════════════════════════

    def _tick(self):
        if not self._baseline_ok:
            self._win.update_state("calib", 0.0,
                f"标定 {len(self._z_buf)}/{self._baseline_frames}", {})
            return

        raw, d = self._update_state()
        self._tick_idx += 1

        # 滞回
        if raw == self._pending_state:
            self._pending_count += 1
        else:
            self._pending_state = raw
            self._pending_count = 1

        if self._pending_count >= self._sustain_frames and raw != self._state:
            old = self._state
            self._state = raw
            self.get_logger().info(
                f"[SWITCH] {old} → {raw}  "
                f"z_sm={d['z_sm']:+.3f}  pd={d['pd_abs_deg']:.1f}°  "
                f"trend=({d['pd_trend']:+.2f}°, {d['z_trend']:+.4f}m)  "
                f"slope={d['on_slope']}  flat={d['on_flat']}  plat={d['on_platform_cond']}"
            )
            # ★ 状态切换时捕获对应点云
            if raw == "ramp":
                self._capture_ramp_cloud()
            elif raw == "platform":
                # 平台捕获延迟（不清缓存，靠空间约束过滤）
                self._platform_capture_delay = 20

        # ★ 延迟捕获平台点云（等待缓存积累新帧）
        if self._platform_capture_delay > 0:
            self._platform_capture_delay -= 1
            if self._platform_capture_delay == 0 and self._platform_cloud is None:
                self._capture_platform_cloud()

        # 上坡追踪
        if self._state == "ramp":
            if self._ramp_start_z is None:
                self._ramp_start_z = d["z_ground"]
                self._ramp_peak_z = d["z"]
            elif d["z"] > self._ramp_peak_z:
                self._ramp_peak_z = d["z"]

        ramp_ratio = 0.0
        if self._state == "ramp" and self._ramp_start_z is not None:
            ramp_ratio = min(1.0, max(0.0, (self._ramp_peak_z - self._ramp_start_z) / self._platform_z_ref))

        diag = (
            f"raw={raw}  trend=({d['pd_trend']:+.1f}°, {d['z_trend']:+.2f}m)  "
            f"slope={d['on_slope']}  gnd_z={d['gnd_z']:.3f}"
        )

        vals = {"z": d["z"], "z_sm": d["z_sm"], "pitch_deg": d["pitch_deg"],
                "pitch_delta_deg": d["pitch_delta_deg"], "z_rise": d["z_rise"],
                "gnd_z": d["gnd_z"]}
        self._win.update_state(self._state, ramp_ratio, diag, vals)

        # 点云可视化（每 5 tick 刷新一次，节约资源）
        if self._tick_idx % 5 == 0 and self._state in ("ramp", "platform", "transition"):
            self._publish_debug_cloud()


    # ══════════════════════════════════════════════════════════════
    # 点云工具（从 blue_z3_live_ramp_fit 融合）
    # ══════════════════════════════════════════════════════════════

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
            return (x.astype(np.float64, copy=False),
                    y.astype(np.float64, copy=False),
                    z.astype(np.float64, copy=False))
        try:
            t = self._tf_buffer.lookup_transform("odom", src, Time())
        except Exception:
            return None, None, None
        q = t.transform.rotation
        tx, ty, tz = (float(t.transform.translation.x),
                      float(t.transform.translation.y),
                      float(t.transform.translation.z))
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
        xf, yf, zf = x.astype(np.float64), y.astype(np.float64), z.astype(np.float64)
        return (r00 * xf + r01 * yf + r02 * zf + tx,
                r10 * xf + r11 * yf + r12 * zf + ty,
                r20 * xf + r21 * yf + r22 * zf + tz)

    @staticmethod
    @staticmethod
    def _detect_z_peak(z, *, lowest: bool):
        z = z[np.isfinite(z)]
        if len(z) < 30:
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
            thresh = max(5, int(hist.max() * 0.15))
            idx = next((i for i, c in enumerate(hist) if c >= thresh), int(np.argmax(hist)))
        else:
            idx = int(np.argmax(hist))
        in_bin = (z >= edges[idx]) & (z < edges[idx + 1])
        if int(in_bin.sum()) < 10:
            return None
        return float(np.median(z[in_bin]))

    @staticmethod
    def _make_rgb_cloud(header, x, y, z, r, g, b) -> PointCloud2:
        n = len(x)
        a = np.full(n, 255, dtype=np.uint8)
        pts = np.zeros(n, dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32), ("rgb", np.uint32)])
        pts["x"] = np.asarray(x, dtype=np.float32)
        pts["y"] = np.asarray(y, dtype=np.float32)
        pts["z"] = np.asarray(z, dtype=np.float32)
        pts["rgb"] = ((a.astype(np.uint32) << 24) |
                      (np.asarray(r, dtype=np.uint8).astype(np.uint32) << 16) |
                      (np.asarray(g, dtype=np.uint8).astype(np.uint32) << 8) |
                      np.asarray(b, dtype=np.uint8).astype(np.uint32))
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

    def destroy_node(self):
        super().destroy_node()


def main():
    app = QtWidgets.QApplication(sys.argv)
    rclpy.init()
    node = Z3StateMonitor()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            app.processEvents()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
