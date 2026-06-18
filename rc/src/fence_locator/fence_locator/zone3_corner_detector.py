"""第三区关键角点检测。"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .geometry import (
    boundary_by_bins,
    fit_simple_line,
    intersect_lines_2d,
    line_to_u_of_v,
    line_to_v_of_u,
    local_to_odom,
    ransac_line_2d,
)


@dataclass(frozen=True)
class Zone3CornerConfig:
    enable_ransac_refine: bool = True
    ransac_radius: float = 0.85
    ransac_dist_thr: float = 0.045
    ransac_min_inliers: int = 35


def fit_zone3_corner_from_samples(
    cloud_samples: list[np.ndarray],
    pose: tuple[float, float, float, float] | None,
    config: Zone3CornerConfig,
) -> tuple[dict | None, str]:
    if not cloud_samples:
        return None, "no_samples"
    if pose is None:
        return None, "no_pose"
    base_x, base_y, _base_z, model_yaw = pose
    pts = np.concatenate(cloud_samples, axis=0)
    if len(pts) < 500:
        return None, f"few_cloud_samples:{len(pts)}"

    x = pts[:, 0]
    y = pts[:, 1]
    z = pts[:, 2]
    dx = x - base_x
    dy = y - base_y
    u = dx * math.cos(model_yaw) + dy * math.sin(model_yaw)
    v = -dx * math.sin(model_yaw) + dy * math.cos(model_yaw)
    roi = (
        (u >= -0.40) & (u <= 3.00)
        & (v >= -1.60) & (v <= 1.60)
        & (z >= -0.35) & (z <= 0.75)
        & np.isfinite(z)
    )
    if int(roi.sum()) < 600:
        return None, f"few_roi:{int(roi.sum())}"
    u = u[roi]
    v = v[roi]
    z = z[roi]

    z_hi = float(np.percentile(z, 86.0))
    platform = (z >= z_hi - 0.09) & (z <= z_hi + 0.12) & (u >= 0.55)
    pu = u[platform]
    pv = v[platform]
    pz = z[platform]
    if len(pu) < 600:
        return None, f"few_platform:{len(pu)}"

    candidates = []
    for side_name, side_pct, far_pct in (
        ("positive", 96.0, 96.0),
        ("negative", 4.0, 96.0),
    ):
        su, sv = boundary_by_bins(pu, pv, 0.10, side_pct, 8)
        side_fit = fit_simple_line(su, sv, 0.08)
        if side_fit is None:
            continue
        sk, sb, sn, srmse = side_fit
        fallback_far = far_edge_from_side_endpoint(su, sv, sk, sb)
        far_fit_full = robust_far_edge_by_vertical_bins(pu, pv, sk, sb)
        if far_fit_full is None:
            fu, fv = boundary_by_bins(pv, pu, 0.10, far_pct, 8)
            far_fit = fit_simple_line(fu, fv, 0.08)
            if far_fit is None and fallback_far is not None:
                far_fit, fu, fv = fallback_far
        else:
            fk0, fb0, fn0, frmse0, fu0, fv0 = far_fit_full
            far_fit = (fk0, fb0, fn0, frmse0)
            fu, fv = fu0, fv0
        if far_fit is None:
            continue
        fk, fb, fn, frmse = far_fit
        denom = 1.0 - fk * sk
        if abs(denom) < 1e-6:
            continue
        corner_u = float((fk * sb + fb) / denom)
        corner_v = float(sk * corner_u + sb)
        dot = abs((fk + sk) / (math.hypot(1.0, sk) * math.hypot(fk, 1.0)))
        angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
        vertical = zone3_vertical_support(u, v, z, corner_u, corner_v)
        support = (np.abs(pu - corner_u) <= 0.25) & (np.abs(pv - corner_v) <= 0.25)
        vertical_bonus = min(4.0, vertical["count"] / 40.0) * min(3.0, vertical["span"] / 0.06)
        angle_bonus = max(0.1, 1.0 - max(0.0, dot - 0.20))
        score = int(support.sum()) * angle_bonus * (1.0 + vertical_bonus) / max(0.005, srmse + frmse)
        candidates.append((score, side_name, sk, sb, sn, srmse, fk, fb, fn, frmse, corner_u, corner_v, angle_deg, vertical, su, sv, fu, fv))

    if not candidates:
        return None, "no_edge_pair"
    _, side_name, sk, sb, sn, srmse, fk, fb, fn, frmse, corner_u, corner_v, angle_deg, vertical, su, sv, fu, fv = max(
        candidates, key=lambda item: item[0]
    )

    refined = None
    if config.enable_ransac_refine:
        refined = refine_zone3_corner_ransac(
            pu, pv, corner_u, corner_v, sk, sb, fk, fb, config
        )
    if refined is not None:
        corner_u = refined["corner_u"]
        corner_v = refined["corner_v"]
        sk = refined["outer_k"]
        sb = refined["outer_b"]
        srmse = refined["outer_rmse"]
        sn = refined["outer_inliers"]
        fk = refined["far_k"]
        fb = refined["far_b"]
        frmse = refined["far_rmse"]
        fn = refined["far_inliers"]
        angle_deg = refined["angle_deg"]
        vertical = zone3_vertical_support(u, v, z, corner_u, corner_v)

    corner_x, corner_y = local_to_odom(base_x, base_y, model_yaw, corner_u, corner_v)
    result = {
        "side": side_name,
        "refined": bool(refined is not None),
        "corner_u": float(corner_u),
        "corner_v": float(corner_v),
        "corner_x": float(corner_x),
        "corner_y": float(corner_y),
        "corner_z": float(np.median(pz)),
        "outer_k": float(sk),
        "outer_b": float(sb),
        "outer_bins": int(sn),
        "outer_rmse": float(srmse),
        "far_k": float(fk),
        "far_b": float(fb),
        "far_bins": int(fn),
        "far_rmse": float(frmse),
        "angle_deg": float(angle_deg),
        "vertical_count": int(vertical["count"]),
        "vertical_span": float(vertical["span"]),
        "vertical_z_low": float(vertical["z_low"]),
        "vertical_z_high": float(vertical["z_high"]),
        "outer_bin_u": su,
        "outer_bin_v": sv,
        "far_bin_axis": fu,
        "far_bin_value": fv,
        "centroid_u": float(np.median(pu)),
        "centroid_v": float(np.median(pv)),
    }
    if refined is not None:
        result["ransac_outer_points"] = int(refined["outer_points"])
        result["ransac_far_points"] = int(refined["far_points"])
    return result, "ok"


def refine_zone3_corner_ransac(
    pu: np.ndarray,
    pv: np.ndarray,
    corner_u: float,
    corner_v: float,
    outer_k: float,
    outer_b: float,
    far_k: float,
    far_b: float,
    config: Zone3CornerConfig,
) -> dict | None:
    radius = max(0.25, config.ransac_radius)
    outer_seed_dist = np.abs(pv - (outer_k * pu + outer_b))
    far_seed_dist = np.abs(pu - (far_k * pv + far_b))
    outer_mask = (
        (outer_seed_dist <= 0.16)
        & (pu >= corner_u - radius * 1.7)
        & (pu <= corner_u + radius * 0.35)
        & (np.hypot(pu - corner_u, pv - corner_v) <= radius * 1.8)
    )
    far_mask = (
        (far_seed_dist <= 0.16)
        & (np.abs(pu - corner_u) <= radius * 0.55)
        & (np.abs(pv - corner_v) <= radius * 1.6)
    )
    outer_pts = np.column_stack((pu[outer_mask], pv[outer_mask]))
    far_pts = np.column_stack((pu[far_mask], pv[far_mask]))
    if len(outer_pts) < config.ransac_min_inliers or len(far_pts) < config.ransac_min_inliers:
        return None

    outer_line = ransac_line_2d(outer_pts, config.ransac_dist_thr, config.ransac_min_inliers, seed=20260618)
    far_line = ransac_line_2d(far_pts, config.ransac_dist_thr, config.ransac_min_inliers, seed=20260619)
    if outer_line is None or far_line is None:
        return None

    intersection = intersect_lines_2d(outer_line["line"], far_line["line"])
    if intersection is None:
        return None
    refined_u, refined_v = intersection
    if math.hypot(refined_u - corner_u, refined_v - corner_v) > 0.35:
        return None

    outer_kb = line_to_v_of_u(outer_line["line"])
    far_kb = line_to_u_of_v(far_line["line"])
    if outer_kb is None or far_kb is None:
        return None
    sk, sb = outer_kb
    fk, fb = far_kb
    dot = abs((fk + sk) / (math.hypot(1.0, sk) * math.hypot(fk, 1.0)))
    angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
    if angle_deg < 70.0 or angle_deg > 110.0:
        return None
    return {
        "corner_u": float(refined_u),
        "corner_v": float(refined_v),
        "outer_k": float(sk),
        "outer_b": float(sb),
        "outer_rmse": float(outer_line["rmse"]),
        "outer_inliers": int(outer_line["inliers"]),
        "outer_points": int(len(outer_pts)),
        "far_k": float(fk),
        "far_b": float(fb),
        "far_rmse": float(far_line["rmse"]),
        "far_inliers": int(far_line["inliers"]),
        "far_points": int(len(far_pts)),
        "angle_deg": float(angle_deg),
    }


def far_edge_from_side_endpoint(side_u: np.ndarray, side_v: np.ndarray, side_k: float, side_b: float):
    if len(side_u) < 4:
        return None
    order = np.argsort(side_u)
    tail_count = min(len(order), max(4, int(math.ceil(len(order) * 0.25))))
    tail = order[-tail_count:]
    corner_u = float(np.median(side_u[tail]))
    corner_v = float(side_k * corner_u + side_b)
    far_k = -float(side_k)
    far_b = corner_u - far_k * corner_v
    span = max(0.45, min(1.25, float(np.ptp(side_v)) if len(side_v) > 1 else 0.8))
    far_v = np.linspace(corner_v - 0.5 * span, corner_v + 0.5 * span, max(4, tail_count))
    far_u = far_k * far_v + far_b
    return (far_k, far_b, int(len(far_v)), 0.045), far_v, far_u


def robust_far_edge_by_vertical_bins(pu: np.ndarray, pv: np.ndarray, outer_k: float, outer_b: float):
    outer_pred = outer_k * pu + outer_b
    side_band = np.abs(pv - outer_pred) <= 1.25
    if int(side_band.sum()) < 400:
        side_band = np.ones_like(pu, dtype=bool)
    u0 = pu[side_band]
    v0 = pv[side_band]
    if len(u0) < 400:
        return None
    hist, edges = np.histogram(u0, bins=np.arange(0.5, 3.05, 0.04))
    if len(hist) == 0:
        return None
    smooth = np.convolve(hist.astype(np.float64), np.ones(5) / 5.0, mode="same")
    threshold = max(20.0, float(smooth.max()) * 0.20)
    valid = np.where(smooth >= threshold)[0]
    if len(valid) == 0:
        return None
    right_bin = int(valid[-1])
    ref_u = float((edges[right_bin] + edges[right_bin + 1]) * 0.5)
    near = (u0 >= ref_u - 0.18) & (u0 <= ref_u + 0.12)
    if int(near.sum()) < 180:
        near = (u0 >= ref_u - 0.25) & (u0 <= ref_u + 0.16)
    if int(near.sum()) < 120:
        return None
    y_axis, x_val = boundary_by_bins(v0[near], u0[near], 0.08, 50.0, 6)
    fit = fit_simple_line(y_axis, x_val, 0.06)
    if fit is None:
        return None
    k, b, n, rmse = fit
    if abs(k) > 0.12:
        b = float(np.median(x_val))
        k = 0.0
        err = x_val - b
        rmse = float(np.sqrt(np.mean(err**2)))
        n = int(len(x_val))
    return k, b, n, rmse, y_axis, x_val


def zone3_vertical_support(u: np.ndarray, v: np.ndarray, z: np.ndarray, corner_u: float, corner_v: float) -> dict:
    near = np.hypot(u - corner_u, v - corner_v) <= 0.16
    if int(near.sum()) < 20:
        near = np.hypot(u - corner_u, v - corner_v) <= 0.22
    nz = z[near]
    if len(nz) == 0:
        return {"count": 0, "span": 0.0, "z_low": float("nan"), "z_high": float("nan")}
    z_low, z_high = np.percentile(nz, [8.0, 92.0])
    return {"count": int(len(nz)), "span": float(z_high - z_low), "z_low": float(z_low), "z_high": float(z_high)}
