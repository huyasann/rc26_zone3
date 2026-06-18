#!/usr/bin/env python3
"""地面高度标定工具 — 订阅 /odin1/cloud_slam, 投影到 odom 系, Qt 直方图.

使用方法:
  python3 ground_calibrator.py
  # 或添加 entry point 后:
  ros2 run zone_detection ground_calibrator

功能:
  - 实时点云 Z_odom 直方图 (与 ground_estimator 同算法)
  - 自动检测地面最低峰
  - 手动输入 GROUND_Z 并叠加高度层区间
  - 冻结/单帧模式
  - 数据帧信息显示
"""

#!/usr/bin/env python3
"""地面高度标定工具 — 订阅 /odin1/cloud_slam, 投影到 odom 系, Qt 直方图.

使用方法:
  conda activate env_ros2
  python3 script/ground_calibrator.py

功能:
  - 实时点云 Z_odom 直方图 (与 ground_estimator 同算法)
  - 自动检测地面最低峰
  - 手动输入 GROUND_Z 并叠加高度层区间
  - 将标定结果写入 config.py (带 .bak 备份)
  - 冻结/恢复、bin 数调节
"""

import os
import re
import shutil
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from PyQt5 import QtWidgets, QtCore
from matplotlib.backends.backend_qt5agg import (FigureCanvasQTAgg as FigureCanvas,
                                                NavigationToolbar2QT)
from matplotlib.figure import Figure

# ── 常量 ──
CLOUD_TOPIC = "/odin1/cloud_slam"
HEIGHT_FRAME = "odom"

# 自动检测参数 (与 ground_estimator.py 一致)
GROUND_PEAK_RATIO = 0.15
HISTOGRAM_BIN_WIDTH = 0.02

# 直方图固定显示范围 (odom 系 Z, 覆盖地面到层3顶部)
HIST_Z_MIN = -2.0
HIST_Z_MAX = 3.0
HIST_N_BINS = 250

# 高度层区间 (h = Z_odom - ground_z)
HEIGHT_BANDS = [
    (-0.10, 0.07, "#B0B0B0", "Ground"),
    (0.07,  0.50, "#FFFF00", "Base"),
    (0.50,  0.80, "#00C8C8", "Gap"),
    (0.80,  1.34, "#FF5014", "Layer1"),
    (1.34,  1.88, "#50C850", "Layer2"),
    (1.88,  2.50, "#C850FF", "Layer3"),
]

# config.py 路径 (相对于此脚本)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.normpath(os.path.join(_SCRIPT_DIR, ".."))
CONFIG_PATHS = [
    os.path.join(_PKG_DIR, "zone_detection", "zone2", "config.py"),
    os.path.join(_PKG_DIR, "zone_detection", "zone3", "config.py"),
]

# ─────────────────────────── ROS2 Node ──────────────────────────


class GroundCalibratorNode(Node):
    """订阅 cloud_slam, 将点云投影到 odom 系, 供 GUI 消费."""

    def __init__(self):
        super().__init__("ground_calibrator")

        self.sub = self.create_subscription(
            PointCloud2, CLOUD_TOPIC, self._cloud_cb, 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # 线程安全数据缓冲
        self._lock = threading.Lock()
        self._z_odom = np.array([], dtype=np.float64)
        self._frame_id = ""
        self._stamp_sec = 0.0
        self._n_total = 0
        self._tf_ok = False
        self._tf_error = ""
        self._frozen = False
        self._last_tf_warn = 0.0

        self.get_logger().info(f"Subscribed to {CLOUD_TOPIC}")

    def _cloud_cb(self, msg: PointCloud2):
        if self._frozen:
            return
        try:
            x, y, z = self._parse_xyz(msg)
        except Exception as e:
            self.get_logger().warn(f"PointCloud2 parse error: {e}")
            return
        n = len(x)
        if n == 0:
            return

        z_odom = self._to_odom_z(msg.header, x, y, z)
        with self._lock:
            if z_odom is not None:
                self._z_odom = z_odom
            self._frame_id = msg.header.frame_id
            self._stamp_sec = (float(msg.header.stamp.sec)
                               + float(msg.header.stamp.nanosec) * 1e-9)
            self._n_total = n

    # ── 点云解析 ──────────────────────────────────────────────

    @staticmethod
    def _parse_xyz(cloud: PointCloud2):
        """从 PointCloud2 提取 x/y/z (兼容任意 field offset)."""
        offsets = {}
        for f in cloud.fields:
            if f.name in ("x", "y", "z"):
                offsets[f.name] = f.offset
        if len(offsets) < 3:
            raise ValueError("PointCloud2 missing x/y/z fields")
        n = cloud.width * cloud.height if cloud.height > 1 else cloud.width
        dt = np.dtype({
            "names": ["x", "y", "z"],
            "formats": [np.float32] * 3,
            "offsets": [offsets[nm] for nm in ("x", "y", "z")],
            "itemsize": cloud.point_step,
        })
        pts = np.frombuffer(cloud.data, dtype=dt, count=n)
        return pts["x"], pts["y"], pts["z"]

    # ── TF 投影 ──────────────────────────────────────────────

    def _to_odom_z(self, header, x, y, z):
        """将点云 Z 投影到 HEIGHT_FRAME (odom) 系. 返回 np.ndarray."""
        src = header.frame_id
        if not src or src == HEIGHT_FRAME:
            self._tf_ok = True
            self._tf_error = ""
            return z.astype(np.float64, copy=False)

        try:
            t = self._tf_buffer.lookup_transform(HEIGHT_FRAME, src, Time())
        except Exception as e:
            now = time.monotonic()
            if now - self._last_tf_warn >= 1.0:
                self._last_tf_warn = now
                self.get_logger().warn(f"TF {src}->{HEIGHT_FRAME}: {e}")
            self._tf_ok = False
            self._tf_error = str(e)[:60]
            return None

        self._tf_ok = True
        self._tf_error = ""
        rot = t.transform.rotation
        xx, yy = rot.x ** 2, rot.y ** 2
        xz, yz = rot.x * rot.z, rot.y * rot.z
        wx, wy = rot.w * rot.x, rot.w * rot.y
        r20 = 2.0 * (xz - wy)
        r21 = 2.0 * (yz + wx)
        r22 = 1.0 - 2.0 * (xx + yy)
        return (r20 * x.astype(np.float64, copy=False)
                + r21 * y.astype(np.float64, copy=False)
                + r22 * z.astype(np.float64, copy=False)
                + float(t.transform.translation.z))

    # ── 地面检测 (同 ground_estimator.py) ─────────────────────

    def detect_ground(self, z: np.ndarray):
        """直方图检测最低地面峰. 返回 (z_mid, peak_pct)."""
        if len(z) == 0:
            return None, 0.0
        bins = self._hist_bins(z)
        hist, edges = np.histogram(z, bins=bins)
        peak = hist.max()
        if peak == 0:
            return None, 0.0
        idx = next((i for i, v in enumerate(hist)
                    if v >= peak * GROUND_PEAK_RATIO),
                   int(np.argmax(hist)))
        z_mid = (edges[idx] + edges[idx + 1]) / 2.0
        pct = 100.0 * hist[idx] / len(z)
        return z_mid, pct

    def _hist_bins(self, z: np.ndarray):
        """从 z 范围生成等宽直方图 bin 边界 (同 ground_estimator)."""
        zf = z[np.isfinite(z)]
        if len(zf) == 0:
            return np.array([0.0, HISTOGRAM_BIN_WIDTH], dtype=np.float64)
        lo = np.floor(zf.min() / HISTOGRAM_BIN_WIDTH) * HISTOGRAM_BIN_WIDTH
        hi = np.ceil(zf.max() / HISTOGRAM_BIN_WIDTH) * HISTOGRAM_BIN_WIDTH
        if hi <= lo:
            hi = lo + HISTOGRAM_BIN_WIDTH
        return np.arange(lo, hi + HISTOGRAM_BIN_WIDTH, HISTOGRAM_BIN_WIDTH)

    # ── 获取最新数据 (线程安全) ──────────────────────────────

    def get_data(self):
        """线程安全读取, 返回 (z_odom, frame_id, stamp, n_total, tf_ok, tf_err)."""
        with self._lock:
            return (self._z_odom.copy(),
                    self._frame_id,
                    self._stamp_sec,
                    self._n_total,
                    self._tf_ok,
                    self._tf_error)

    def set_frozen(self, frozen: bool):
        self._frozen = frozen


# ─────────────────────────── Qt 窗口 ─────────────────────────


class HistogramWindow(QtWidgets.QMainWindow):
    """主窗口: matplotlib 直方图 + 控制面板."""

    UPDATE_INTERVAL_MS = 100  # 10 Hz 刷新

    def __init__(self, node: GroundCalibratorNode):
        super().__init__()
        self._node = node
        self._auto_ground_z = None
        self._auto_ground_pct = 0.0
        self._show_bands = True
        self._n_bins = HIST_N_BINS

        self._init_ui()
        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self._update_plot)
        self._timer.start(self.UPDATE_INTERVAL_MS)

    def _init_ui(self):
        self.setWindowTitle("Ground Height Calibrator - Z_odom Histogram")
        self.resize(1400, 800)

        # 状态栏
        self.statusBar().showMessage("Waiting for point cloud...")

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        # ── 左侧: 画布 ──
        plot_panel = QtWidgets.QVBoxLayout()
        self._fig = Figure(figsize=(10, 7))
        self._ax = self._fig.add_subplot(111)
        self._canvas = FigureCanvas(self._fig)
        self._toolbar = NavigationToolbar2QT(self._canvas, self)
        plot_panel.addWidget(self._toolbar)
        plot_panel.addWidget(self._canvas)
        layout.addLayout(plot_panel, 3)

        # ── 右侧: 控制面板 ──
        ctrl = QtWidgets.QVBoxLayout()
        ctrl.setSpacing(8)

        # 信息区
        info_group = QtWidgets.QGroupBox("Status")
        info_layout = QtWidgets.QFormLayout(info_group)
        self._lbl_topic = QtWidgets.QLabel(CLOUD_TOPIC)
        self._lbl_frame = QtWidgets.QLabel("—")
        self._lbl_points = QtWidgets.QLabel("0")
        self._lbl_tf = QtWidgets.QLabel("Waiting for TF...")
        self._lbl_tf.setStyleSheet("color: orange")
        self._lbl_auto_gz = QtWidgets.QLabel("—")
        self._lbl_auto_gz.setStyleSheet("font-weight: bold")
        info_layout.addRow("Topic:", self._lbl_topic)
        info_layout.addRow("Frame:", self._lbl_frame)
        info_layout.addRow("Points:", self._lbl_points)
        info_layout.addRow("TF:", self._lbl_tf)
        info_layout.addRow("Auto ground_z:", self._lbl_auto_gz)
        ctrl.addWidget(info_group)

        # ── Manual ground_z ──
        gz_group = QtWidgets.QGroupBox("Manual GROUND_Z")
        gz_layout = QtWidgets.QVBoxLayout(gz_group)
        gz_row = QtWidgets.QHBoxLayout()
        self._spin_gz = QtWidgets.QDoubleSpinBox()
        self._spin_gz.setRange(-5.0, 5.0)
        self._spin_gz.setSingleStep(0.005)
        self._spin_gz.setDecimals(4)
        self._spin_gz.setValue(-0.270)
        self._spin_gz.setPrefix("z = ")
        self._spin_gz.setSuffix(" m")

        self._btn_auto_gz = QtWidgets.QPushButton("Take Auto")
        self._btn_auto_gz.clicked.connect(self._take_auto_gz)

        gz_row.addWidget(self._spin_gz)
        gz_row.addWidget(self._btn_auto_gz)
        gz_layout.addLayout(gz_row)

        self._chk_manual = QtWidgets.QCheckBox("Use manual (overrides auto)")
        self._chk_manual.stateChanged.connect(self._on_manual_toggle)
        gz_layout.addWidget(self._chk_manual)

        self._btn_save = QtWidgets.QPushButton("Save to config.py")
        self._btn_save.clicked.connect(self._save_to_config)
        gz_layout.addWidget(self._btn_save)
        ctrl.addWidget(gz_group)

        # ── 显示控制 ──
        disp_group = QtWidgets.QGroupBox("Display")
        disp_layout = QtWidgets.QVBoxLayout(disp_group)

        self._chk_bands = QtWidgets.QCheckBox("Show height bands")
        self._chk_bands.setChecked(True)
        self._chk_bands.stateChanged.connect(
            lambda v: (setattr(self, '_show_bands', bool(v)),
                       self._update_plot()))
        disp_layout.addWidget(self._chk_bands)

        bin_row = QtWidgets.QHBoxLayout()
        bin_row.addWidget(QtWidgets.QLabel("Bins:"))
        self._spin_bins = QtWidgets.QSpinBox()
        self._spin_bins.setRange(30, 500)
        self._spin_bins.setValue(HIST_N_BINS)
        self._spin_bins.valueChanged.connect(self._on_bins_changed)
        bin_row.addWidget(self._spin_bins)
        disp_layout.addLayout(bin_row)

        ctrl.addWidget(disp_group)

        # ── 冻结 / 退出 ──
        ctrl.addStretch()
        btn_freeze = QtWidgets.QPushButton("Freeze")
        btn_freeze.setCheckable(True)
        btn_freeze.clicked.connect(self._on_freeze)
        ctrl.addWidget(btn_freeze)

        btn_quit = QtWidgets.QPushButton("Quit")
        btn_quit.clicked.connect(self.close)
        ctrl.addWidget(btn_quit)

        layout.addLayout(ctrl, 1)

    # ── 更新 ────────────────────────────────────────────────

    def _update_plot(self):
        z_odom, frame_id, stamp_sec, n_total, tf_ok, tf_err = \
            self._node.get_data()

        # 更新状态
        self._lbl_frame.setText(frame_id or "—")
        self._lbl_points.setText(str(n_total))
        if tf_ok:
            self._lbl_tf.setText("OK")
            self._lbl_tf.setStyleSheet("color: green")
        else:
            txt = f"Fail: {tf_err}" if tf_err else "Waiting for TF..."
            self._lbl_tf.setText(txt)
            self._lbl_tf.setStyleSheet("color: orange")

        if len(z_odom) < 10:
            self._ax.clear()
            self._ax.set_xlim(HIST_Z_MIN, HIST_Z_MAX)
            self._ax.set_title("Waiting for point cloud...")
            self._ax.set_xlabel("Z_odom (m)")
            self._ax.set_ylabel("Count")
            self._canvas.draw()
            return

        # 地面检测
        self._auto_ground_z, self._auto_ground_pct = \
            self._node.detect_ground(z_odom)
        if self._auto_ground_z is not None:
            self._lbl_auto_gz.setText(
                f"{self._auto_ground_z:.4f} m  "
                f"(peak {self._auto_ground_pct:.1f}%)")
            self._lbl_auto_gz.setStyleSheet("color: #2a7; font-weight: bold")
        else:
            self._lbl_auto_gz.setText("None")
            self._lbl_auto_gz.setStyleSheet("color: gray")

        # 绘图
        self._ax.clear()

        # —— 直方图 ——
        finite = np.isfinite(z_odom)
        if finite.any():
            self._ax.hist(z_odom[finite], bins=self._n_bins,
                          range=(HIST_Z_MIN, HIST_Z_MAX),
                          color="#4080C0", alpha=0.7, edgecolor="none")

        # —— 高度层区间 ——
        ground_z = (self._spin_gz.value()
                    if self._chk_manual.isChecked()
                    else (self._auto_ground_z or self._spin_gz.value()))
        if self._show_bands and ground_z is not None:
            for h_min, h_max, color, label in HEIGHT_BANDS:
                lo = ground_z + h_min
                hi = ground_z + h_max
                self._ax.axvspan(lo, hi, alpha=0.12, color=color,
                                 label=f"{label} ({h_min:.2f}~{h_max:.2f}h)")

        # —— 自动 ground_z 标记线 ——
        if self._auto_ground_z is not None:
            self._ax.axvline(self._auto_ground_z,
                             color="#2a7", linewidth=2, linestyle="--",
                             label=f"Auto ground_z={self._auto_ground_z:.4f}")

        # —— 手动 ground_z 标记线 ——
        if self._chk_manual.isChecked():
            manual_z = self._spin_gz.value()
            self._ax.axvline(manual_z,
                             color="#E03030", linewidth=2, linestyle="-",
                             label=f"Manual ground_z={manual_z:.4f}")

        self._ax.set_xlim(HIST_Z_MIN, HIST_Z_MAX)
        self._ax.set_xlabel("Z_odom (m)")
        self._ax.set_ylabel("Count")
        self._ax.set_title(
            f"Z_odom Histogram  |  "
            f"Frame: {stamp_sec:.1f}s  |  "
            f"Points: {len(z_odom)}")
        self._ax.grid(True, alpha=0.3)
        self._ax.legend(fontsize=8, loc="upper right")
        self._fig.tight_layout()
        self._canvas.draw()

    # ── 交互回调 ─────────────────────────────────────────────

    def _on_manual_toggle(self, state):
        """手动/自动切换 —— 立即刷新 plot + 状态栏."""
        self._update_plot()
        if state:
            self.statusBar().showMessage(
                f"Manual mode: ground_z = {self._spin_gz.value():.4f} m")
        else:
            if self._auto_ground_z is not None:
                self.statusBar().showMessage(
                    f"Auto mode: ground_z = {self._auto_ground_z:.4f} m")
            else:
                self.statusBar().showMessage("Auto mode: waiting for data...")

    def _take_auto_gz(self):
        """从自动检测值复制到手动 spinbox —— 立即刷新 plot + 状态栏."""
        if self._auto_ground_z is None:
            self.statusBar().showMessage(
                "Cannot take auto value: no data or ground not detected yet")
            return
        self._spin_gz.setValue(round(self._auto_ground_z, 4))
        self._chk_manual.setChecked(True)
        self._update_plot()
        self.statusBar().showMessage(
            f"Auto ground_z applied: {self._auto_ground_z:.4f} m")

    def _on_bins_changed(self, val):
        """bin 数改变 —— 立即刷新 plot."""
        self._n_bins = val
        self._update_plot()

    def _on_freeze(self, checked):
        self._node.set_frozen(checked)
        btn = self.sender()
        btn.setText("Resume" if checked else "Freeze")
        self.statusBar().showMessage("Frozen" if checked else "Resumed")

    # ── 保存到 config.py ─────────────────────────────────────

    @staticmethod
    def _write_ground_z_to_file(filepath: str, new_gz: float) -> bool:
        """替换文件中 GROUND_Z = 所在行, 自动 .bak 备份."""
        if not os.path.isfile(filepath):
            return False
        backup = filepath + ".bak"
        try:
            shutil.copy2(filepath, backup)
        except OSError:
            pass

        pat = re.compile(r'^(GROUND_Z\s*=\s*)(-?\d+\.?\d*)', re.MULTILINE)
        with open(filepath, "r", encoding="utf-8") as f:
            content = f.read()

        new_content = pat.sub(
            lambda m: f"{m.group(1)}{new_gz:.4f}", content)
        if new_content == content:
            return False  # 没有匹配到 GROUND_Z

        with open(filepath, "w", encoding="utf-8") as f:
            f.write(new_content)
        return True

    def _save_to_config(self):
        """保存当前 ground_z 到 config.py, 带确认 + 备份."""
        gz = self._spin_gz.value()

        # 确认对话框
        paths_display = "\n".join(f"  • {p}" for p in CONFIG_PATHS)
        reply = QtWidgets.QMessageBox.question(
            self, "Save GROUND_Z",
            f"Write GROUND_Z = {gz:.4f} to config files?\n\n"
            f"Auto-backup -> *.bak will be created.\n{paths_display}",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No)
        if reply != QtWidgets.QMessageBox.Yes:
            return

        ok, fail = 0, []
        for fp in CONFIG_PATHS:
            if self._write_ground_z_to_file(fp, gz):
                ok += 1
            else:
                fail.append(os.path.basename(os.path.dirname(fp)))
        total = len(CONFIG_PATHS)

        if not fail:
            self.statusBar().showMessage(
                f"Saved GROUND_Z = {gz:.4f} to {ok}/{total} config files")
            QtWidgets.QMessageBox.information(
                self, "Save Complete",
                f"GROUND_Z = {gz:.4f} saved to {ok} config files.\n"
                f"Backup files created: *.bak\n\n"
                f"Restart detection nodes to apply.")
        else:
            self.statusBar().showMessage(
                f"Saved {ok}/{total} (failed: {', '.join(fail)})")
            QtWidgets.QMessageBox.warning(
                self, "Save Partial",
                f"Saved {ok}/{total} files.\n"
                f"Failed: {', '.join(fail)}\n"
                f"Check file permissions.")

    def closeEvent(self, event):
        self._timer.stop()
        super().closeEvent(event)


# ─────────────────────────── 启动 ──────────────────────────


def main(args=None):
    rclpy.init(args=args)
    node = GroundCalibratorNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    window = HistogramWindow(node)
    window.show()

    try:
        ret = app.exec_()
    except KeyboardInterrupt:
        ret = 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
