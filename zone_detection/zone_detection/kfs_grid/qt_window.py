"""KFS Grid Qt 调试窗口 — 3×3 格子实时颜色显示.

从 KfsGridDetectorNode 读取统计结果, Qt GUI 实时刷新.
仅在 ENABLE_QT_GUI=True 时加载, 生产环境无 Qt 依赖.

用法:
  ros2 run zone_detection kfs_grid_detector_qt
"""

from __future__ import annotations

import math
import signal
import sys
import threading
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

try:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtGui import QFont
    from PyQt5.QtWidgets import (
        QApplication, QGridLayout, QHBoxLayout, QLabel,
        QMainWindow, QVBoxLayout, QWidget,
    )
except ImportError as exc:
    raise SystemExit("缺少 PyQt5。请在带 PyQt5 的环境运行本脚本。") from exc

from zone_detection.kfs_grid import config as C


# ════════════════════════════════════════════════════════════
# 单个格子控件
# ════════════════════════════════════════════════════════════

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

        self._title = QLabel(C.CELL_NAMES[layer][col])
        self._title.setAlignment(Qt.AlignCenter)
        self._title.setFont(QFont("", 10))
        self._title.setStyleSheet("font-weight: bold; color: #FFF;")
        layout.addWidget(self._title)

        self._result = QLabel("—")
        self._result.setAlignment(Qt.AlignCenter)
        self._result.setFont(QFont("", 18, QFont.Bold))
        layout.addWidget(self._result)

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

        min_valid_score = max(C.MIN_VALID_SCORE_ABS, int(total * C.MIN_VALID_SCORE_RATIO))

        if total < C.EMPTY_THRESHOLD:
            self._result.setText("EMPTY")
            self._result.setStyleSheet("color: #666; font-size: 14px;")
            self.setStyleSheet(
                "background-color: #1E1E1E; border: 2px solid #444; border-radius: 6px;")
        elif red > blue and red >= blue * C.DOMINANT_RATIO and red >= min_valid_score:
            self._result.setText("🔴")
            self._result.setStyleSheet("color: #FF4040; font-size: 22px;")
            self.setStyleSheet(
                "background-color: #3A1818; border: 2px solid #FF4040; border-radius: 6px;")
        elif blue > red and blue >= red * C.DOMINANT_RATIO and blue >= min_valid_score:
            self._result.setText("🔵")
            self._result.setStyleSheet("color: #4080FF; font-size: 22px;")
            self.setStyleSheet(
                "background-color: #18183A; border: 2px solid #4080FF; border-radius: 6px;")
        else:
            self._result.setText("?")
            self._result.setStyleSheet("color: #AAA; font-size: 18px;")
            self.setStyleSheet(
                "background-color: #2A2A2A; border: 2px solid #888; border-radius: 6px;")


# ════════════════════════════════════════════════════════════
# 主窗口
# ════════════════════════════════════════════════════════════

class KfsGridQtWindow(QMainWindow):
    """主窗口: 上 3×3 格子 + 下状态 + 底栏参数.

    通过 node 的公共属性读取数据:
      - node.cell_stats: [(r,b,t), ...] × 9
      - node.is_locked: bool
      - node.zone3_frame: str
    """

    def __init__(self, node) -> None:
        super().__init__()
        self._node = node
        self.setWindowTitle("KFS Grid Color Detector")
        self.resize(620, 580)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setSpacing(4)
        layout.setContentsMargins(8, 6, 8, 6)

        # ── 主区域 3×3 格子面板 ──
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
        layout.addWidget(gw, 1)

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
        self._timer.start(int(round(1000.0 / C.GUI_REFRESH_HZ)))

    def _update(self):
        """定时读取 node 统计, 刷新 9 格 + 状态栏."""
        # 从 KfsGridDetectorNode 的公共属性读取
        stats = self._node.cell_stats
        locked = self._node.is_locked
        z3f = self._node.zone3_frame

        if not locked:
            return
        if len(stats) < 9:
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

        if tr > tb and tr >= tb * 1.3:
            color_tag = "🔴 偏红"
        elif tb > tr and tb >= tr * 1.3:
            color_tag = "🔵 偏蓝"
        else:
            color_tag = "⚪ 混合"

        self._status.setText(
            f"✅ {z3f}   |   "
            f"格 {ta}   |   "
            f"R{tr}  B{tb}  {color_tag}")
        self._status.setStyleSheet(
            "font-size: 14px; padding: 6px; color: #4C4; "
            "background-color: #1A2A1A; border-radius: 4px;")

        self._summary.setText(
            f"累积 {self._node._accumulate_frames}帧  |  "
            f"延伸 X{C.EXPAND_X:.2f} Y{C.EXPAND_Y:.2f} Z{C.EXPAND_Z:.2f}  |  "
            f"色彩净胜分 空<{C.EMPTY_THRESHOLD}")


# ════════════════════════════════════════════════════════════
# Qt 模式启动
# ════════════════════════════════════════════════════════════

def _spin_ros_background(node: Node, stop_event: threading.Event) -> None:
    """后台 ROS spin 线程: 0.05s 超时轮询."""
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


def run_qt(node) -> int:
    """以 Qt GUI 模式运行 (ROS spin 在后台线程, Qt 在主线程).

    Args:
        node: KfsGridDetectorNode 实例 (已初始化, 未 spin)

    Returns:
        exit code (int)
    """
    stop_event = threading.Event()
    spin_thread = threading.Thread(
        target=_spin_ros_background, args=(node, stop_event), daemon=True)
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
