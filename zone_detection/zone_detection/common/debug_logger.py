"""调试日志工具。

- DebugLogger: 逐帧 CSV 事件日志
- HeightDiagLogger: Z3 高度区间诊断 (各层点数/odom_z/采样点)
"""

import csv
import os
import time
from datetime import datetime
from typing import Any, Optional

import numpy as np


def _make_log_stamp(value: str = "") -> str:
    stamp = str(value or "").strip()
    return stamp or datetime.now().strftime("%m%d_%H%M")


def _safe_prefix(prefix: str) -> str:
    return os.path.basename(str(prefix or "debug").strip()).replace(os.sep, "_")


def _unique_path(path: str) -> str:
    if not os.path.exists(path):
        return path
    root, ext = os.path.splitext(path)
    idx = 1
    while True:
        candidate = f"{root}_{idx}{ext}"
        if not os.path.exists(candidate):
            return candidate
        idx += 1


class DebugLogger:
    """逐帧 CSV 事件日志."""

    def __init__(self, enabled: bool, log_dir: str, prefix: str = "debug", stamp: str = ""):
        """初始化: 启用时创建 CSV 文件并写表头."""
        self._enabled = enabled
        self._path = ""
        if enabled:
            os.makedirs(log_dir, exist_ok=True)
            self._path = _unique_path(os.path.join(log_dir, f"{_safe_prefix(prefix)}_{_make_log_stamp(stamp)}.csv"))
            with open(self._path, "w", newline="", encoding="utf-8") as f:
                csv.writer(f).writerow(["time", "event", "data"])
        self._ready = bool(self._path)

    def write_event(self, event: str, **data: Any):
        """写一行事件日志: time, event, key=val ..."""
        if not self._ready:
            return
        payload = " ".join(f"{k}={v}" for k, v in data.items())
        with open(self._path, "a", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow([f"{time.time():.3f}", event, payload])

    def close(self):
        """关闭日志 (禁止后续写入)."""
        self._ready = False


class Zone3CsvLogger:
    """Z3 逐帧检测 CSV 日志 (含 confidence/grid/root 等字段)."""

    HEADER = [
        "stamp_sec", "event", "source_points", "high_points", "confidence",
        "grid_x", "grid_y", "grid_yaw", "root_x", "root_y", "root_yaw",
        "component_points", "width", "depth", "layers", "extra",
    ]

    def __init__(self, enabled: bool, log_dir: str, prefix: str = "zone3_debug", stamp: str = ""):
        """初始化: 启用时创建 CSV 并写 HEADER."""
        self._enabled = enabled
        self._path = ""
        if enabled:
            os.makedirs(log_dir, exist_ok=True)
            self._path = _unique_path(os.path.join(log_dir, f"{_safe_prefix(prefix)}_{_make_log_stamp(stamp)}.csv"))
            self._write_header()

    def _write_header(self):
        """写 CSV 列头."""
        with open(self._path, "w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(self.HEADER)

    def write(self, stamp_sec: float, event: str, data: dict):
        """按 HEADER 顺序写一行检测数据."""
        if not self._enabled or not self._path:
            return
        row = [stamp_sec, event] + [data.get(k, "") for k in self.HEADER[2:]]
        with open(self._path, "a", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(row)

    def close(self):
        """关闭日志."""
        self._enabled = False


class HeightDiagLogger:
    """Z3 高度区间诊断日志 (每帧各层点分布/odom_z中位数/采样坐标)."""

    HEADER = [
        "stamp_sec", "frame_id", "n_total",
        "n_gray", "n_yellow", "n_cyan", "n_orange", "n_green", "n_purple", "n_unbanded",
        "orange_green_purple_medianZ_and_count",
        "sample_points_xyz_h",
    ]

    def __init__(self, enabled: bool, log_dir: str, stamp: str = ""):
        """初始化: 启用时创建 CSV 并写 HEADER."""
        self._enabled = enabled
        self._path = ""
        if enabled:
            os.makedirs(log_dir, exist_ok=True)
            self._path = _unique_path(os.path.join(log_dir, f"zone3_height_diag_{_make_log_stamp(stamp)}.csv"))
            with open(self._path, "w", newline="", encoding="utf-8") as f:
                csv.writer(f).writerow(self.HEADER)

    def write(
        self,
        stamp_sec: float,
        src: str,
        x: np.ndarray,
        y: np.ndarray,
        z_h: np.ndarray,
        h: np.ndarray,
        height_bands: list,
    ):
        """逐帧写入各高度区间的点数 / odom_z 中位数 / 采样坐标."""
        if not self._enabled or not self._path or len(h) == 0:
            return

        n_total = len(h)
        cnt = [0] * (len(height_bands) + 1)
        z_by_band = {}
        samp = {}
        for pi in range(n_total):
            hi, zi = h[pi], z_h[pi]
            matched = False
            for bi, (lo, hi_r, *_) in enumerate(height_bands):
                if lo <= hi < hi_r:
                    cnt[bi] += 1
                    if bi in (3, 4, 5):  # orange/green/purple
                        z_by_band.setdefault(bi, []).append(float(zi))
                        if len(samp.get(bi, [])) < 5:
                            samp.setdefault(bi, []).append(
                                f"({float(x[pi]):.2f},{float(y[pi]):.2f},"
                                f"{float(zi):.3f},{float(hi):.3f})")
                    matched = True
                    break
            if not matched:
                cnt[-1] += 1

        def med(arr):
            return f"{float(np.median(arr)):.3f}" if arr else "none"

        band_info = ";".join(
            f"{i}:med_z={med(z_by_band.get(i, []))} n={cnt[i]}"
            for i in (3, 4, 5))
        samp_info = ";".join(
            f"b{i}:{','.join(samp.get(i, ['none']))}"
            for i in (3, 4, 5))

        with open(self._path, "a", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow([
                f"{stamp_sec:.3f}", src, n_total,
                *cnt, band_info, samp_info,
            ])

    def close(self):
        """关闭日志."""
        self._enabled = False
