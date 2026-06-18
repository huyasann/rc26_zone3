"""地面估计与高度归一化。

将点云 Z 投影到 HEIGHT_FRAME (odom) 后，用直方图检测地面高度。
支持手动预设 ground_z 或自动检测 + EMA 更新。
"""

import math
import time
from typing import Optional, Tuple

import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# ── 可调参数 ──────────────────────────────────────────────
HEIGHT_FRAME = "odom"
GROUND_TOLERANCE = 0.035
GROUND_PEAK_RATIO = 0.15
HISTOGRAM_BIN_WIDTH = 0.02
GROUND_UPDATE_ALPHA = 0.18
GROUND_MAX_UPDATE_STEP = 0.06


class GroundEstimator:
    """地面高度估计器。

    Args:
        node: ROS2 node (用于创建 TF listener 和 logger)
        known_z: 手动 ground_z, None 表示首帧自动估计
    """

    def __init__(self, node: Node, known_z: Optional[float] = None):
        """初始化: 创建 TF listener, 设定手动或自动地面."""
        self._node = node
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        self._last_tf_warn = 0.0

        if known_z is not None:
            self._ground_z = float(known_z)
            self._ground_pct = 100.0
        else:
            self._ground_z = None
            self._ground_pct = 0.0

    @property
    def ground_z(self) -> Optional[float]:
        """当前地面高度 (m)."""
        return self._ground_z

    def to_height_frame(
        self, header, x: np.ndarray, y: np.ndarray, z: np.ndarray
    ) -> Optional[np.ndarray]:
        """将点云 Z 投影到 HEIGHT_FRAME (odom) 系.

        Returns:
            投影后的 Z 数组，或 None (TF 不可用)
        """
        src = header.frame_id
        if not src or src == HEIGHT_FRAME:
            return z

        try:
            t = self._tf_buffer.lookup_transform(HEIGHT_FRAME, src, Time())
        except Exception as e:
            now = time.monotonic()
            if now - self._last_tf_warn >= 1.0:
                self._last_tf_warn = now
                self._node.get_logger().warn(
                    f"TF {src}->{HEIGHT_FRAME} 不可用: {e}")
            return None

        rot = t.transform.rotation
        xx, yy = rot.x ** 2, rot.y ** 2
        xz, yz = rot.x * rot.z, rot.y * rot.z
        wx, wy = rot.w * rot.x, rot.w * rot.y
        r20 = 2 * (xz - wy)
        r21 = 2 * (yz + wx)
        r22 = 1 - 2 * (xx + yy)
        return (r20 * x.astype(np.float32, copy=False)
                + r21 * y.astype(np.float32, copy=False)
                + r22 * z.astype(np.float32, copy=False)
                + np.float32(t.transform.translation.z))

    def update_ground(self, z_h: np.ndarray) -> float:
        """直方图检测地面 + EMA 更新, 返回当前 ground_z."""
        if len(z_h) == 0:
            return self._ground_z or 0.0

        hist_z, mp = self._detect_ground(z_h)

        if self._ground_z is None:
            self._ground_z, self._ground_pct = hist_z, mp
        elif abs(hist_z - self._ground_z) <= GROUND_MAX_UPDATE_STEP:
            self._ground_z += GROUND_UPDATE_ALPHA * (hist_z - self._ground_z)
            self._ground_pct = mp

        return self._ground_z or 0.0

    def _detect_ground(self, z: np.ndarray) -> Tuple[float, float]:
        """直方图找最低地面峰."""
        bins = self._hist_bins(z)
        hist, edges = np.histogram(z, bins=bins)
        peak = hist.max()
        idx = next((i for i, v in enumerate(hist)
                    if v >= peak * GROUND_PEAK_RATIO), int(np.argmax(hist)))
        z_mid = (edges[idx] + edges[idx + 1]) / 2.0
        return z_mid, 100.0 * hist[idx] / len(z)

    def _hist_bins(self, z: np.ndarray) -> np.ndarray:
        """从 z 范围生成等宽直方图 bin 边界."""
        zf = z[np.isfinite(z)]
        if len(zf) == 0:
            return np.array([0.0, HISTOGRAM_BIN_WIDTH], dtype=np.float32)
        lo = np.floor(zf.min() / HISTOGRAM_BIN_WIDTH) * HISTOGRAM_BIN_WIDTH
        hi = np.ceil(zf.max() / HISTOGRAM_BIN_WIDTH) * HISTOGRAM_BIN_WIDTH
        if hi <= lo:
            hi = lo + HISTOGRAM_BIN_WIDTH
        return np.arange(lo, hi + HISTOGRAM_BIN_WIDTH, HISTOGRAM_BIN_WIDTH)

    def height_above_ground(self, z_h: np.ndarray) -> np.ndarray:
        """返回离地高度 h = z_h - ground_z."""
        if self._ground_z is None:
            raise RuntimeError("ground_z 未初始化, 先调用 update_ground")
        return z_h - self._ground_z
