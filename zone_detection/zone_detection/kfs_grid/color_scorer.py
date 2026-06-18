"""KFS Grid 颜色评分 — 纯函数 (零 ROS 依赖)。

核心算法: 色彩纯度权重矩阵 — 不依赖 HSV 硬阈值, 用 R/B 通道差值 + 自乘归一化计算净胜分。

来源: kfs_grid_qt.py 中向量化色彩净胜分逻辑。
"""

from __future__ import annotations

from typing import List, Tuple

import numpy as np


# ════════════════════════════════════════════════════════════
# 颜色净胜分计算
# ════════════════════════════════════════════════════════════

def compute_red_blue_scores(
    r: np.ndarray,
    g: np.ndarray,
    b: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """计算色彩纯度净胜分 (向量化)。

    原理:
      红色权重 = max(0, R - max(G,B)) / 255 * (R / 255)
      蓝色权重 = max(0, B - max(R,G)) / 255 * (B / 255)

    第一项 (差值项): 该通道对次大通道的优势, 滤除灰色/白色。
    第二项 (亮度项): 该通道自身的亮度, 减少暗区噪声。
    乘积: 两条件同时满足 → 高纯度色彩才有高分。

    亚克力反光(青色): G 和 B 接近, B - max(R,G) → 0, 自动归零。

    Args:
        r: uint8 或 int16 红色通道 (n,)
        g: uint8 或 int16 绿色通道 (n,)
        b: uint8 或 int16 蓝色通道 (n,)

    Returns:
        (w_red, w_blue): float64 权重数组, 范围 [0.0, 1.0]
    """
    r_i = r.astype(np.int16)
    g_i = g.astype(np.int16)
    b_i = b.astype(np.int16)

    red_diff = np.maximum(0, r_i - np.maximum(g_i, b_i))
    w_red = (red_diff / 255.0) * (r_i / 255.0)

    blue_diff = np.maximum(0, b_i - np.maximum(r_i, g_i))
    w_blue = (blue_diff / 255.0) * (b_i / 255.0)

    return w_red.astype(np.float64), w_blue.astype(np.float64)


# ════════════════════════════════════════════════════════════
# 逐格统计
# ════════════════════════════════════════════════════════════

def compute_cell_stats(
    lx: np.ndarray,
    ly: np.ndarray,
    h: np.ndarray,
    w_red: np.ndarray,
    w_blue: np.ndarray,
    col_centers: List[float],
    layer_centers: List[float],
    expand_x: float,
    expand_y: float,
    expand_z: float,
) -> List[Tuple[int, int, int]]:
    """逐格统计红蓝净胜分。

    对每个格子 (3层 × 3列 = 9格), 筛选在检测框内的点,
    累加其红/蓝权重得到净胜分。

    Args:
        lx, ly, h: grid-local 坐标 (深度, 宽度, 离地高度)
        w_red, w_blue: 每点的红/蓝色纯度权重
        col_centers: 列中心 Y 坐标列表 (3 个)
        layer_centers: 层中心 Z 坐标列表 (3 个)
        expand_x, expand_y, expand_z: 检测框半宽

    Returns:
        stats: [(red_score, blue_score, total_points), ...] × 9
    """
    stats: List[Tuple[int, int, int]] = []
    n_layers = len(layer_centers)
    n_cols = len(col_centers)

    for layer in range(n_layers):
        zc = layer_centers[layer]
        for col in range(n_cols):
            yc = col_centers[col]
            mask = (
                (np.abs(lx) <= expand_x)
                & (np.abs(ly - yc) <= expand_y)
                & (np.abs(h - zc) <= expand_z)
            )
            idx = np.where(mask)[0]
            ct = len(idx)
            if ct == 0:
                stats.append((0, 0, 0))
            else:
                red_score = int(round(np.sum(w_red[idx])))
                blue_score = int(round(np.sum(w_blue[idx])))
                stats.append((red_score, blue_score, ct))

    return stats


# ════════════════════════════════════════════════════════════
# 格子分类
# ════════════════════════════════════════════════════════════

class CellResult:
    """单个格子的分类结果。"""

    __slots__ = ("layer", "col", "name", "red_score", "blue_score",
                 "total_points", "label")

    def __init__(self, layer: int, col: int, name: str,
                 red_score: int, blue_score: int, total_points: int,
                 label: str):
        self.layer = layer
        self.col = col
        self.name = name
        self.red_score = red_score
        self.blue_score = blue_score
        self.total_points = total_points
        self.label = label      # "RED" | "BLUE" | "EMPTY" | "UNKNOWN"

    def __repr__(self) -> str:
        return (f"Cell({self.name}: {self.label} "
                f"R={self.red_score} B={self.blue_score} T={self.total_points})")


def classify_cells(
    stats: List[Tuple[int, int, int]],
    cell_names: List[List[str]],
    empty_threshold: int = 15,
    min_valid_score_abs: int = 20,
    min_valid_score_ratio: float = 0.03,
    dominant_ratio: float = 1.5,
) -> List[CellResult]:
    """对 9 格统计结果进行分类。

    分类规则 (优先级从高到低):
      1. total_points < empty_threshold → EMPTY
      2. red > blue AND red >= blue × dominant_ratio AND red >= min_valid → RED
      3. blue > red AND blue >= red × dominant_ratio AND blue >= min_valid → BLUE
      4. 其他 → UNKNOWN

    min_valid = max(min_valid_score_abs, total_points × min_valid_score_ratio)
    (动态门槛，点数越多要求越高)

    Args:
        stats: [(red_score, blue_score, total_points), ...] × 9
        cell_names: 3×3 格子名列表
        empty_threshold: 空格判据点数
        min_valid_score_abs: 颜色净胜分绝对下限
        min_valid_score_ratio: 颜色净胜分占总点数比例下限
        dominant_ratio: 主导色须 ≥ 次色 × 此值

    Returns:
        results: 9 个 CellResult 列表
    """
    results: List[CellResult] = []
    n_layers = len(cell_names)
    n_cols = len(cell_names[0]) if n_layers > 0 else 0

    for layer in range(n_layers):
        for col in range(n_cols):
            idx = layer * n_cols + col
            red, blue, total = stats[idx]
            name = cell_names[layer][col]

            if total < empty_threshold:
                results.append(CellResult(layer, col, name, red, blue, total, "EMPTY"))
                continue

            min_valid = max(min_valid_score_abs, int(total * min_valid_score_ratio))

            if red > blue and red >= blue * dominant_ratio and red >= min_valid:
                results.append(CellResult(layer, col, name, red, blue, total, "RED"))
            elif blue > red and blue >= red * dominant_ratio and blue >= min_valid:
                results.append(CellResult(layer, col, name, red, blue, total, "BLUE"))
            else:
                results.append(CellResult(layer, col, name, red, blue, total, "UNKNOWN"))

    return results


def summarize_results(results: List[CellResult]) -> dict:
    """汇总 9 格结果: 统计红/蓝/空/未知格子数及总分。

    Returns:
        dict with keys: red_cells, blue_cells, empty_cells, unknown_cells,
                       total_red_score, total_blue_score, total_points, dominant
    """
    red_cells = sum(1 for r in results if r.label == "RED")
    blue_cells = sum(1 for r in results if r.label == "BLUE")
    empty_cells = sum(1 for r in results if r.label == "EMPTY")
    unknown_cells = sum(1 for r in results if r.label == "UNKNOWN")

    total_red = sum(r.red_score for r in results)
    total_blue = sum(r.blue_score for r in results)
    total_pts = sum(r.total_points for r in results)

    if total_red > total_blue and total_red >= total_blue * 1.3:
        dominant = "RED"
    elif total_blue > total_red and total_blue >= total_red * 1.3:
        dominant = "BLUE"
    else:
        dominant = "MIXED"

    return {
        "red_cells": red_cells,
        "blue_cells": blue_cells,
        "empty_cells": empty_cells,
        "unknown_cells": unknown_cells,
        "total_red_score": total_red,
        "total_blue_score": total_blue,
        "total_points": total_pts,
        "dominant": dominant,
    }
