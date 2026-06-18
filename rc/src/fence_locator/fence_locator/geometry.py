"""几何拟合小工具。"""

from __future__ import annotations

import math

import numpy as np


def local_to_odom(base_x: float, base_y: float, yaw: float, forward: float, lateral: float) -> tuple[float, float]:
    return (
        base_x + math.cos(yaw) * forward - math.sin(yaw) * lateral,
        base_y + math.sin(yaw) * forward + math.cos(yaw) * lateral,
    )


def boundary_by_bins(
    axis: np.ndarray,
    value: np.ndarray,
    bin_size: float,
    percentile: float,
    min_count: int,
) -> tuple[np.ndarray, np.ndarray]:
    if len(axis) == 0:
        return np.empty(0), np.empty(0)
    bins = np.floor(axis / bin_size).astype(int)
    out_axis = []
    out_value = []
    for bin_id in np.unique(bins):
        m = bins == bin_id
        if int(m.sum()) < min_count:
            continue
        out_axis.append(float(np.median(axis[m])))
        out_value.append(float(np.percentile(value[m], percentile)))
    return np.asarray(out_axis), np.asarray(out_value)


def fit_simple_line(x: np.ndarray, y: np.ndarray, trim: float) -> tuple[float, float, int, float] | None:
    if len(x) < 4:
        return None
    a = np.column_stack((x, np.ones_like(x)))
    k, b = np.linalg.lstsq(a, y, rcond=None)[0]
    err = y - (k * x + b)
    keep = np.abs(err) <= trim
    if int(keep.sum()) >= 4 and int(keep.sum()) < len(x):
        x = x[keep]
        y = y[keep]
        a = np.column_stack((x, np.ones_like(x)))
        k, b = np.linalg.lstsq(a, y, rcond=None)[0]
        err = y - (k * x + b)
    rmse = float(np.sqrt(np.mean(err**2))) if len(err) else float("nan")
    return float(k), float(b), int(len(x)), rmse


def ransac_line_2d(points: np.ndarray, dist_thr: float, min_inliers: int, seed: int) -> dict | None:
    if len(points) < max(2, min_inliers):
        return None
    rng = np.random.default_rng(seed)
    best_inliers = None
    best_count = 0
    best_line = None
    n = len(points)
    for _ in range(180):
        i, j = rng.choice(n, 2, replace=False)
        p1 = points[i]
        p2 = points[j]
        delta = p2 - p1
        length = float(np.linalg.norm(delta))
        if length < 1e-5:
            continue
        a = float(delta[1] / length)
        b = float(-delta[0] / length)
        c = -a * float(p1[0]) - b * float(p1[1])
        dist = np.abs(a * points[:, 0] + b * points[:, 1] + c)
        inliers = dist <= dist_thr
        count = int(inliers.sum())
        if count > best_count:
            best_count = count
            best_inliers = inliers
            best_line = (a, b, c)
    if best_inliers is None or best_line is None or best_count < min_inliers:
        return None
    pts = points[best_inliers]
    center = pts.mean(axis=0)
    _, _, vh = np.linalg.svd(pts - center, full_matrices=False)
    direction = vh[0]
    a = float(direction[1])
    b = float(-direction[0])
    norm = max(1e-6, math.hypot(a, b))
    a /= norm
    b /= norm
    c = -a * float(center[0]) - b * float(center[1])
    dist = np.abs(a * pts[:, 0] + b * pts[:, 1] + c)
    rmse = float(np.sqrt(np.mean(dist**2))) if len(dist) else float("nan")
    span = float(np.ptp((pts - center) @ direction)) if len(pts) else 0.0
    return {"line": (a, b, c), "inliers": int(len(pts)), "rmse": rmse, "span": span}


def intersect_lines_2d(
    line1: tuple[float, float, float],
    line2: tuple[float, float, float],
) -> tuple[float, float] | None:
    a1, b1, c1 = line1
    a2, b2, c2 = line2
    det = a1 * b2 - a2 * b1
    if abs(det) < 1e-6:
        return None
    x = (b1 * c2 - b2 * c1) / det
    y = (c1 * a2 - c2 * a1) / det
    return float(x), float(y)


def line_to_v_of_u(line: tuple[float, float, float]) -> tuple[float, float] | None:
    a, b, c = line
    if abs(b) < 1e-6:
        return None
    return float(-a / b), float(-c / b)


def line_to_u_of_v(line: tuple[float, float, float]) -> tuple[float, float] | None:
    a, b, c = line
    if abs(a) < 1e-6:
        return None
    return float(-b / a), float(-c / a)
