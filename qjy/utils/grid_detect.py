"""九宫格几何检测算法 (纯函数, 无 ROS 依赖).

流程:
  高位点筛选 → 连通域分析 → PCA 方向估计 → 几何评分 (width/depth/layer/density)
  → 多帧稳定锁 → zone3_root TF 位姿计算

所有函数 stateless, 可独立测试。
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class GridDetection:
    """单帧九宫格检测结果."""
    center_x: float
    center_y: float
    yaw: float
    confidence: float
    point_count: int
    width: float
    depth: float
    layer_count: int
    mask: np.ndarray  # bool mask 标记哪些输入点属于此候选


def detect_grid_pose(x: np.ndarray, y: np.ndarray, h: np.ndarray,
                      grid_min_h: float = 0.75, grid_max_h: float = 2.60,
                      min_high_points: int = 40,
                      min_confidence: float = 0.25,
                      grid_width_y: float = 1.62) -> Optional[GridDetection]:
    """从高位点中检测九宫格位姿.

    Args:
        x, y, h: 点云坐标及离地高度 (numpy 数组)
        grid_min_h: 高位筛选下限 (m)
        grid_max_h: 高位筛选上限 (m)
        min_high_points: 高位点数量门槛
        min_confidence: 单帧评分门槛 (低于此不返回)
        grid_width_y: 期望九宫格 Y 向宽度 (m), 用于评分

    Returns:
        GridDetection 或 None (未检出)
    """
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    h = np.asarray(h, dtype=np.float64)
    n = min(len(x), len(y), len(h))
    if n == 0:
        return None
    x, y, h = x[:n], y[:n], h[:n]

    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(h)
    high = finite & (h >= grid_min_h) & (h <= grid_max_h)
    if int(high.sum()) < min_high_points:
        return None

    comps = _connected_components(x[high], y[high], cell=0.08, min_cell_points=2)
    if not comps:
        return None

    best: Optional[GridDetection] = None
    best_score = -1.0
    high_indices = np.flatnonzero(high)

    for comp in comps:
        if len(comp) < min_high_points:
            continue
        hx, hy, hh = x[high][comp], y[high][comp], h[high][comp]
        result = _fit_component(hx, hy, hh, len(x), high_indices[comp],
                                grid_width_y)
        if result is None:
            continue
        score = result.confidence * math.log1p(result.point_count)
        if score > best_score:
            best = result
            best_score = score
    return best


def _connected_components(x: np.ndarray, y: np.ndarray,
                          cell: float = 0.08,
                          min_cell_points: int = 2) -> List[np.ndarray]:
    """连通域分析 (网格法)."""
    ix = np.floor(x / cell).astype(np.int32)
    iy = np.floor(y / cell).astype(np.int32)
    keys = np.column_stack((ix, iy))
    unique, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    active_cell = counts >= min_cell_points
    active_points = active_cell[inv]
    if not active_points.any():
        return []

    cell_to_points: dict = {}
    for pi, ci in enumerate(inv):
        if active_cell[ci]:
            cell_to_points.setdefault(int(ci), []).append(pi)

    coord_to_ci = {tuple(coord): int(i) for i, coord in enumerate(unique) if active_cell[i]}
    visited: set = set()
    comps: list = []

    for ci in list(cell_to_points):
        if ci in visited:
            continue
        stack = [ci]
        visited.add(ci)
        point_ids: list = []
        while stack:
            cur = stack.pop()
            point_ids.extend(cell_to_points[cur])
            cx, cy = unique[cur]
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nb = coord_to_ci.get((int(cx + dx), int(cy + dy)))
                    if nb is not None and nb not in visited:
                        visited.add(nb)
                        stack.append(nb)
        comps.append(np.asarray(point_ids, dtype=np.int32))

    comps.sort(key=len, reverse=True)
    return comps


def _fit_component(x: np.ndarray, y: np.ndarray, h: np.ndarray,
                    total_count: int, original_indices: np.ndarray,
                    grid_width_y: float) -> Optional[GridDetection]:
    """对单个连通域做 PCA 方向估计 + 几何评分."""
    pts = np.column_stack((x, y))
    center0 = pts.mean(axis=0)
    demean = pts - center0
    if len(pts) < 3:
        return None
    cov = np.cov(demean.T)
    if not np.all(np.isfinite(cov)):
        return None
    vals, vecs = np.linalg.eigh(cov)
    long_axis = vecs[:, int(np.argmax(vals))]
    long_axis /= max(1e-9, float(np.hypot(long_axis[0], long_axis[1])))

    yaw = _normalize_half_turn(math.atan2(long_axis[1], long_axis[0]) - math.pi / 2.0)
    c = math.cos(yaw)
    s = math.sin(yaw)
    lx = c * (x - center0[0]) + s * (y - center0[1])
    ly = -s * (x - center0[0]) + c * (y - center0[1])

    width = _robust_span(ly)
    depth = _robust_span(lx)
    if width < 0.65 or width > 2.35:
        return None
    if depth > 0.80:
        return None

    local_center_x = 0.5 * (_percentile(lx, 5.0) + _percentile(lx, 95.0))
    local_center_y = 0.5 * (_percentile(ly, 3.0) + _percentile(ly, 97.0))
    center_x = center0[0] + c * local_center_x - s * local_center_y
    center_y = center0[1] + s * local_center_x + c * local_center_y

    layer_count = _count_height_layers(h)
    width_score = math.exp(-abs(width - grid_width_y) / 0.42)
    depth_score = math.exp(-max(0.0, depth - 0.55) / 0.35)
    layer_score = min(1.0, layer_count / 3.0)
    density_score = min(1.0, len(x) / 420.0)
    confidence = (0.40 * width_score + 0.10 * depth_score
                  + 0.35 * layer_score + 0.15 * density_score)
    if confidence < 0.25:
        return None

    mask = np.zeros(total_count, dtype=bool)
    mask[original_indices] = True
    return GridDetection(
        center_x=float(center_x),
        center_y=float(center_y),
        yaw=float(yaw),
        confidence=float(confidence),
        point_count=int(len(x)),
        width=float(width),
        depth=float(depth),
        layer_count=int(layer_count),
        mask=mask,
    )


def _count_height_layers(h: np.ndarray) -> int:
    """层数统计: h 分 3 个区间, 每层 ≥30 票算一层."""
    ranges = ((0.80, 1.34), (1.34, 1.88), (1.88, 2.42))
    return sum(int(((h >= lo) & (h < hi)).sum() >= 30) for lo, hi in ranges)


def _robust_span(values: np.ndarray) -> float:
    """97% 分位 - 3% 分位, 剔除离群值."""
    return float(_percentile(values, 97.0) - _percentile(values, 3.0))


def _percentile(values: np.ndarray, p: float) -> float:
    """计算 p 分位数 (0~100)."""
    return float(np.percentile(values, p))


def _normalize_half_turn(angle: float) -> float:
    """归一化到 [-π/2, π/2]."""
    while angle <= -math.pi / 2.0:
        angle += math.pi
    while angle > math.pi / 2.0:
        angle -= math.pi
    return angle


def norm_angle(angle: float) -> float:
    """归一化到 [-π, π)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def choose_team_root_pose(
    grid_x: float, grid_y: float, grid_yaw: float,
    grid_center_rel_x: float, grid_center_rel_y: float,
    is_blue_team: bool,
) -> Tuple[float, float, float]:
    """从九宫格中心位姿 → zone3_root TF 位姿.

    有两组候选解 (yaw 和 yaw+π), 选蓝队 root_y 更大 / 红队更小的那个。
    """
    candidates = []
    for yaw in (grid_yaw, norm_angle(grid_yaw + math.pi)):
        c = math.cos(yaw)
        s = math.sin(yaw)
        root_x = grid_x - (c * grid_center_rel_x - s * grid_center_rel_y)
        root_y = grid_y - (s * grid_center_rel_x + c * grid_center_rel_y)
        expected_side = 1.0 if is_blue_team else -1.0
        candidates.append((expected_side * root_y, root_x, root_x, root_y, yaw))
    candidates.sort(key=lambda item: (item[0], item[1]), reverse=True)
    _, _, rx, ry, ryaw = candidates[0]
    return float(rx), float(ry), float(ryaw)
