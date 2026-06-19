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
    ransac_max_shift: float = 0.28
    min_refine_angle_deg: float = 78.0
    max_refine_angle_deg: float = 102.0
    vertical_bin_size: float = 0.04
    vertical_min_count: int = 3
    vertical_min_span: float = 0.055
    vertical_min_points_per_line: int = 10
    forced_side: str | None = None


def dense_z_mode(values: np.ndarray, bin_size: float = 0.025) -> float | None:
    """Return the densest Z layer center so sparse people/vertical clutter do not set platform height."""
    finite = values[np.isfinite(values)]
    finite = finite[(finite >= -0.35) & (finite <= 0.75)]
    if len(finite) < 80:
        return None
    edges = np.arange(-0.35, 0.76 + bin_size, bin_size)
    hist, edges = np.histogram(finite, bins=edges)
    if len(hist) == 0 or int(hist.max()) <= 0:
        return None
    smooth = np.convolve(hist.astype(np.float64), np.ones(3) / 3.0, mode="same")
    idx = int(np.argmax(smooth))
    return float((edges[idx] + edges[idx + 1]) * 0.5)


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

    platform_seed = z[(u >= 0.55) & (np.abs(v) <= 1.35)]
    platform_z = dense_z_mode(platform_seed)
    if platform_z is None:
        platform_z = float(np.percentile(z, 86.0))
    platform = (z >= platform_z - 0.12) & (z <= platform_z + 0.16) & (u >= 0.55)
    pu = u[platform]
    pv = v[platform]
    pz = z[platform]
    if len(pu) < 450:
        return None, f"few_platform:{len(pu)}"

    side_specs = (
        ("positive", 96.0, 96.0),
        ("negative", 4.0, 96.0),
    )
    if config.forced_side in ("positive", "negative"):
        side_specs = tuple(spec for spec in side_specs if spec[0] == config.forced_side)

    candidates = []
    for side_name, side_pct, far_pct in side_specs:
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

    coarse_corner_u = float(corner_u)
    coarse_corner_v = float(corner_v)
    coarse_angle_deg = float(angle_deg)
    coarse_outer_rmse = float(srmse)
    coarse_far_rmse = float(frmse)
    refined = None
    if config.enable_ransac_refine:
        refined = refine_zone3_corner_vertical_ransac(
            u, v, z, corner_u, corner_v, sk, sb, fk, fb, config
        )
        if refined is None:
            refined = refine_zone3_corner_ransac_2d(
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
    coarse_corner_x, coarse_corner_y = local_to_odom(base_x, base_y, model_yaw, coarse_corner_u, coarse_corner_v)
    refine_shift = float(math.hypot(corner_u - coarse_corner_u, corner_v - coarse_corner_v))
    result = {
        "side": side_name,
        "refined": bool(refined is not None),
        "refine_reason": refined.get("reason", "ok") if refined is not None else "not_used",
        "refine_shift": refine_shift,
        "coarse_corner_u": coarse_corner_u,
        "coarse_corner_v": coarse_corner_v,
        "coarse_corner_x": float(coarse_corner_x),
        "coarse_corner_y": float(coarse_corner_y),
        "coarse_angle_deg": coarse_angle_deg,
        "coarse_outer_rmse": coarse_outer_rmse,
        "coarse_far_rmse": coarse_far_rmse,
        "corner_u": float(corner_u),
        "corner_v": float(corner_v),
        "corner_x": float(corner_x),
        "corner_y": float(corner_y),
        "corner_z": float(np.median(pz)),
        "platform_z": float(platform_z),
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
        result["ransac_outer_span"] = float(refined.get("outer_span", 0.0))
        result["ransac_far_span"] = float(refined.get("far_span", 0.0))
        result["ransac_source"] = str(refined.get("source", "unknown"))
        result["orthogonal_primary"] = str(refined.get("orthogonal_primary", "unknown"))
    return result, "ok"


def refine_zone3_corner_vertical_ransac(
    u: np.ndarray,
    v: np.ndarray,
    z: np.ndarray,
    corner_u: float,
    corner_v: float,
    outer_k: float,
    outer_b: float,
    far_k: float,
    far_b: float,
    config: Zone3CornerConfig,
) -> dict | None:
    vu, vv, vz_span, vcount = vertical_edge_candidates(u, v, z, corner_u, corner_v, config)
    if len(vu) < config.ransac_min_inliers:
        return None

    direct = refine_zone3_corner_from_vertical_bins(
        vu,
        vv,
        corner_u,
        corner_v,
        config,
    )
    if direct is not None:
        direct["source"] = "vertical_bins:" + str(direct.get("source", "unknown"))
        direct["vertical_candidates"] = int(len(vu))
        direct["vertical_span_median"] = float(np.median(vz_span)) if len(vz_span) else 0.0
        direct["vertical_count_median"] = float(np.median(vcount)) if len(vcount) else 0.0
        return direct

    radius = max(0.25, config.ransac_radius)
    outer_seed_dist = np.abs(vv - (outer_k * vu + outer_b))
    far_seed_dist = np.abs(vu - (far_k * vv + far_b))
    # 这里优先相信围栏的竖直点列，而不是平台地面外轮廓。
    # 旧窗口太窄，车斜着上坡或行人残留时，粗角点会把真实竖直边排除掉。
    outer_mask = (
        (outer_seed_dist <= 0.28)
        & (vu >= corner_u - radius * 2.6)
        & (vu <= corner_u + radius * 0.75)
        & (np.hypot(vu - corner_u, vv - corner_v) <= radius * 2.8)
    )
    far_mask = (
        (far_seed_dist <= 0.28)
        & (np.abs(vu - corner_u) <= radius * 0.95)
        & (np.abs(vv - corner_v) <= radius * 2.4)
    )
    if int(outer_mask.sum()) < config.vertical_min_points_per_line or int(far_mask.sum()) < config.vertical_min_points_per_line:
        return None

    outer_pts = np.column_stack((vu[outer_mask], vv[outer_mask]))
    far_pts = np.column_stack((vu[far_mask], vv[far_mask]))
    refined = refine_from_two_point_sets(
        outer_pts,
        far_pts,
        corner_u,
        corner_v,
        config,
        seed_outer=20260620,
        seed_far=20260621,
    )
    if refined is None:
        return None
    refined["source"] = "vertical_ransac:" + str(refined.get("source", "unknown"))
    refined["vertical_candidates"] = int(len(vu))
    refined["vertical_span_median"] = float(np.median(vz_span)) if len(vz_span) else 0.0
    refined["vertical_count_median"] = float(np.median(vcount)) if len(vcount) else 0.0
    return refined


def refine_zone3_corner_from_vertical_bins(
    vu: np.ndarray,
    vv: np.ndarray,
    corner_u: float,
    corner_v: float,
    config: Zone3CornerConfig,
) -> dict | None:
    """Fit the two visible fence axes from vertical point columns first.

    The platform/ramp surface can create a strong 2D outline, but the target corner is defined
    by vertical fence columns. This path extracts a side boundary and a far boundary from the
    vertical-column cloud, then forces the final pair to be orthogonal.
    """
    if len(vu) < config.ransac_min_inliers:
        return None
    outer_pct = 96.0
    if config.forced_side == "negative":
        outer_pct = 4.0

    ou, ov = boundary_by_bins(vu, vv, 0.10, outer_pct, 2)
    fv, fu = boundary_by_bins(vv, vu, 0.10, 96.0, 2)
    if len(ou) < 6 or len(fu) < 6:
        return None

    outer_pts = np.column_stack((ou, ov))
    far_pts = np.column_stack((fu, fv))
    # Keep only the edge sections near the expected corner. This rejects people/wall columns
    # that are vertical but do not belong to the two fence axes.
    radius = max(0.35, config.ransac_radius * 1.35)
    outer_keep = (
        (np.abs(outer_pts[:, 1] - corner_v) <= radius * 1.25)
        & (outer_pts[:, 0] >= corner_u - radius * 2.8)
        & (outer_pts[:, 0] <= corner_u + radius * 0.9)
    )
    far_keep = (
        (np.abs(far_pts[:, 0] - corner_u) <= radius * 1.25)
        & (far_pts[:, 1] >= corner_v - radius * 2.2)
        & (far_pts[:, 1] <= corner_v + radius * 2.2)
    )
    outer_pts = outer_pts[outer_keep]
    far_pts = far_pts[far_keep]
    if len(outer_pts) < 5 or len(far_pts) < 5:
        return None

    refined = refine_from_two_point_sets(
        outer_pts,
        far_pts,
        corner_u,
        corner_v,
        config,
        seed_outer=20260622,
        seed_far=20260623,
        max_shift=max(config.ransac_max_shift, 0.45),
    )
    if refined is None:
        return None
    return refined


def refine_zone3_corner_ransac_2d(
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

    refined = refine_from_two_point_sets(
        outer_pts,
        far_pts,
        corner_u,
        corner_v,
        config,
        seed_outer=20260618,
        seed_far=20260619,
    )
    if refined is not None:
        refined["source"] = "platform_2d_ransac:" + str(refined.get("source", "unknown"))
    return refined


def refine_from_two_point_sets(
    outer_pts: np.ndarray,
    far_pts: np.ndarray,
    corner_u: float,
    corner_v: float,
    config: Zone3CornerConfig,
    seed_outer: int,
    seed_far: int,
    max_shift: float | None = None,
) -> dict | None:
    min_inliers = max(2, min(config.ransac_min_inliers, len(outer_pts), len(far_pts)))
    outer_line = ransac_line_2d(outer_pts, config.ransac_dist_thr, min_inliers, seed=seed_outer)
    far_line = ransac_line_2d(far_pts, config.ransac_dist_thr, min_inliers, seed=seed_far)
    if outer_line is None or far_line is None:
        return None

    orthogonal = force_orthogonal_edge_pair(outer_pts, far_pts, outer_line, far_line, config)
    if orthogonal is None:
        return None

    intersection = intersect_lines_2d(orthogonal["outer_line"], orthogonal["far_line"])
    if intersection is None:
        return None
    refined_u, refined_v = intersection
    shift = math.hypot(refined_u - corner_u, refined_v - corner_v)
    allowed_shift = config.ransac_max_shift if max_shift is None else max_shift
    if shift > allowed_shift:
        return None

    outer_kb = line_to_v_of_u(orthogonal["outer_line"])
    far_kb = line_to_u_of_v(orthogonal["far_line"])
    if outer_kb is None or far_kb is None:
        return None
    sk, sb = outer_kb
    fk, fb = far_kb
    angle_deg = 90.0
    if orthogonal["outer_span"] < 0.35 or orthogonal["far_span"] < 0.18:
        return None
    source = "orthogonal_" + str(orthogonal["primary"])
    return {
        "reason": "ok",
        "corner_u": float(refined_u),
        "corner_v": float(refined_v),
        "outer_k": float(sk),
        "outer_b": float(sb),
        "outer_rmse": float(orthogonal["outer_rmse"]),
        "outer_inliers": int(orthogonal["outer_inliers"]),
        "outer_span": float(orthogonal["outer_span"]),
        "outer_points": int(len(outer_pts)),
        "far_k": float(fk),
        "far_b": float(fb),
        "far_rmse": float(orthogonal["far_rmse"]),
        "far_inliers": int(orthogonal["far_inliers"]),
        "far_span": float(orthogonal["far_span"]),
        "far_points": int(len(far_pts)),
        "angle_deg": float(angle_deg),
        "shift": float(shift),
        "source": source,
        "orthogonal_primary": str(orthogonal["primary"]),
    }


def force_orthogonal_edge_pair(
    outer_pts: np.ndarray,
    far_pts: np.ndarray,
    outer_line: dict,
    far_line: dict,
    config: Zone3CornerConfig,
) -> dict | None:
    """Use RANSAC lines as seeds, then force the final two axes to be perpendicular."""
    candidates = []
    outer_raw = outer_line["line"]
    far_raw = far_line["line"]

    candidates.append(
        build_orthogonal_candidate(
            outer_pts,
            far_pts,
            primary_line=outer_raw,
            primary_name="outer",
            dist_thr=config.ransac_dist_thr,
        )
    )
    candidates.append(
        build_orthogonal_candidate(
            outer_pts,
            far_pts,
            primary_line=far_raw,
            primary_name="far",
            dist_thr=config.ransac_dist_thr,
        )
    )
    candidates = [item for item in candidates if item is not None]
    if not candidates:
        return None
    return max(candidates, key=lambda item: item["score"])


def build_orthogonal_candidate(
    outer_pts: np.ndarray,
    far_pts: np.ndarray,
    primary_line: tuple[float, float, float],
    primary_name: str,
    dist_thr: float,
) -> dict | None:
    a, b, _c = primary_line
    # RANSAC line normal is (a,b), so its direction is (b,-a).
    primary_dir = np.asarray([b, -a], dtype=np.float64)
    primary_norm = float(np.linalg.norm(primary_dir))
    if primary_norm < 1e-6:
        return None
    primary_dir /= primary_norm
    perpendicular_dir = np.asarray([-primary_dir[1], primary_dir[0]], dtype=np.float64)

    if primary_name == "outer":
        outer_line_orth = refit_parallel_line(outer_pts, primary_dir)
        far_line_orth = refit_parallel_line(far_pts, perpendicular_dir)
    else:
        far_line_orth = refit_parallel_line(far_pts, primary_dir)
        outer_line_orth = refit_parallel_line(outer_pts, perpendicular_dir)

    outer_stats = line_fit_stats(outer_pts, outer_line_orth, dist_thr)
    far_stats = line_fit_stats(far_pts, far_line_orth, dist_thr)
    if outer_stats["inliers"] < 2 or far_stats["inliers"] < 2:
        return None
    score = (
        4.0 * outer_stats["inliers"]
        + 4.0 * far_stats["inliers"]
        + 1.5 * outer_stats["span"]
        + 1.5 * far_stats["span"]
        - 80.0 * (outer_stats["rmse"] + far_stats["rmse"])
    )
    return {
        "primary": primary_name,
        "outer_line": outer_line_orth,
        "far_line": far_line_orth,
        "outer_rmse": outer_stats["rmse"],
        "far_rmse": far_stats["rmse"],
        "outer_inliers": outer_stats["inliers"],
        "far_inliers": far_stats["inliers"],
        "outer_span": outer_stats["span"],
        "far_span": far_stats["span"],
        "score": float(score),
    }


def refit_parallel_line(points: np.ndarray, direction: np.ndarray) -> tuple[float, float, float]:
    direction = direction / max(1e-6, float(np.linalg.norm(direction)))
    normal = np.asarray([-direction[1], direction[0]], dtype=np.float64)
    offsets = points @ normal
    c = -float(np.median(offsets))
    return float(normal[0]), float(normal[1]), c


def line_fit_stats(points: np.ndarray, line: tuple[float, float, float], dist_thr: float) -> dict:
    a, b, c = line
    dist = np.abs(a * points[:, 0] + b * points[:, 1] + c)
    keep = dist <= max(0.02, dist_thr * 1.5)
    inliers = int(keep.sum())
    if inliers > 0:
        pts = points[keep]
        rmse = float(np.sqrt(np.mean(dist[keep] ** 2)))
    else:
        pts = points
        rmse = float("inf")
    direction = np.asarray([b, -a], dtype=np.float64)
    direction /= max(1e-6, float(np.linalg.norm(direction)))
    span = float(np.ptp(pts @ direction)) if len(pts) else 0.0
    return {"inliers": inliers, "rmse": rmse, "span": span}


def vertical_edge_candidates(
    u: np.ndarray,
    v: np.ndarray,
    z: np.ndarray,
    corner_u: float,
    corner_v: float,
    config: Zone3CornerConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    radius = max(0.25, config.ransac_radius)
    roi = (
        (u >= corner_u - radius * 1.9)
        & (u <= corner_u + radius * 0.8)
        & (v >= corner_v - radius * 1.9)
        & (v <= corner_v + radius * 1.9)
        & np.isfinite(z)
    )
    if int(roi.sum()) < 60:
        return np.empty(0), np.empty(0), np.empty(0), np.empty(0)

    ru = u[roi]
    rv = v[roi]
    rz = z[roi]
    bin_size = max(0.02, config.vertical_bin_size)
    bu = np.floor(ru / bin_size).astype(int)
    bv = np.floor(rv / bin_size).astype(int)
    keys = bu.astype(np.int64) * 1000003 + bv.astype(np.int64)

    out_u = []
    out_v = []
    out_span = []
    out_count = []
    for key in np.unique(keys):
        m = keys == key
        count = int(m.sum())
        if count < config.vertical_min_count:
            continue
        local_z = rz[m]
        z_low, z_high = np.percentile(local_z, [10.0, 90.0])
        span = float(z_high - z_low)
        if span < config.vertical_min_span:
            continue
        out_u.append(float(np.median(ru[m])))
        out_v.append(float(np.median(rv[m])))
        out_span.append(span)
        out_count.append(float(count))
    return (
        np.asarray(out_u, dtype=np.float64),
        np.asarray(out_v, dtype=np.float64),
        np.asarray(out_span, dtype=np.float64),
        np.asarray(out_count, dtype=np.float64),
    )


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
