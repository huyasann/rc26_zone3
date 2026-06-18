#!/usr/bin/env python3
"""点云优先的 Z3 上平台角点验证。

只用 odom 粗略截取上坡/上平台时间窗；几何位置由点云侧边和高度折线推出。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2


ODOM_TOPIC = "/odin1/odometry_highfreq"
CLOUD_TOPIC = "/odin1/cloud_slam"


def quat_to_rpy(q):
    t0 = 2.0 * (q.w * q.x + q.y * q.z)
    t1 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(t0, t1)
    t2 = 2.0 * (q.w * q.y - q.z * q.x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)
    t3 = 2.0 * (q.w * q.z + q.x * q.y)
    t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def parse_cloud(cloud: PointCloud2) -> np.ndarray:
    count = cloud.width * cloud.height if cloud.height > 1 else cloud.width
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [np.float32, np.float32, np.float32],
            "offsets": [0, 4, 8],
            "itemsize": cloud.point_step,
        }
    )
    arr = np.frombuffer(cloud.data, dtype=dtype, count=count)
    valid = np.isfinite(arr["x"]) & np.isfinite(arr["y"]) & np.isfinite(arr["z"])
    return np.column_stack((arr["x"][valid], arr["y"][valid], arr["z"][valid])).astype(np.float64, copy=False)


def open_reader(path: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader, {item.name: item.type for item in reader.get_all_topics_and_types()}


def read_bag(path: str):
    reader, topic_types = open_reader(path)
    type_map = {topic: get_message(t) for topic, t in topic_types.items()}
    odoms = []
    clouds = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic == ODOM_TOPIC:
            msg: Odometry = deserialize_message(data, type_map[topic])
            _, pitch, yaw = quat_to_rpy(msg.pose.pose.orientation)
            p = msg.pose.pose.position
            odoms.append({"t": t_ns * 1e-9, "x": p.x, "y": p.y, "z": p.z, "pitch": pitch, "yaw": yaw})
        elif topic == CLOUD_TOPIC:
            clouds.append((t_ns * 1e-9, deserialize_message(data, type_map[topic])))
    return odoms, clouds


def unwrap_yaws(yaws):
    return np.unwrap(np.asarray(yaws, dtype=np.float64))


def pick_window(odoms):
    z0 = float(np.median([o["z"] for o in odoms[:30]]))
    p0 = float(np.median([o["pitch"] for o in odoms[:30]]))
    t0 = odoms[0]["t"]
    enriched = []
    for o in odoms:
        pitch_abs = abs(math.degrees(angle_diff(o["pitch"], p0)))
        z_rise = o["z"] - z0
        enriched.append({**o, "rel": o["t"] - t0, "pitch_abs": pitch_abs, "z_rise": z_rise})
    ramp_i = None
    for i, o in enumerate(enriched):
        if o["z_rise"] >= 0.08 and o["pitch_abs"] >= 5.0:
            ramp_i = i
            break
    if ramp_i is None:
        return None
    plat_i = None
    for i in range(ramp_i, len(enriched)):
        o = enriched[i]
        if o["z_rise"] >= 0.34 and o["pitch_abs"] <= 10.5:
            plat_i = i
            break
    if plat_i is None:
        plat_i = min(len(enriched) - 1, ramp_i + 120)
    win = enriched[ramp_i:plat_i + 1]
    yaw = float(np.median(unwrap_yaws([o["yaw"] for o in win if o["pitch_abs"] >= 5.0])))
    anchor = win[0]
    return {
        "start_t": win[0]["t"] - 1.0,
        "end_t": win[-1]["t"] + 1.0,
        "anchor_x": float(anchor["x"]),
        "anchor_y": float(anchor["y"]),
        "yaw": yaw,
        "ramp_rel": float(win[0]["rel"]),
        "platform_rel": float(win[-1]["rel"]),
        "z0": z0,
    }


def to_local(pts, anchor_x, anchor_y, yaw):
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    dx = x - anchor_x
    dy = y - anchor_y
    c = math.cos(yaw)
    s = math.sin(yaw)
    u = dx * c + dy * s
    v = -dx * s + dy * c
    return u, v, z


def to_odom(anchor_x, anchor_y, yaw, u, v, z):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return anchor_x + c * u - s * v, anchor_y + s * u + c * v, z


def ransac_line(x, y, max_slope=0.55, threshold=0.08, rounds=220):
    if len(x) < 5:
        return None
    rng = np.random.default_rng(20260616)
    best = None
    best_count = 0
    for _ in range(rounds):
        i, j = rng.choice(len(x), 2, replace=False)
        if abs(x[j] - x[i]) < 1e-6:
            continue
        k = float((y[j] - y[i]) / (x[j] - x[i]))
        if abs(k) > max_slope:
            continue
        b = float(y[i] - k * x[i])
        err = np.abs(y - (k * x + b))
        keep = err <= threshold
        count = int(keep.sum())
        if count > best_count:
            best = (k, b, keep)
            best_count = count
    if best is None or best_count < 5:
        return None
    k, b, keep = best
    a = np.column_stack((x[keep], np.ones(best_count)))
    k, b = np.linalg.lstsq(a, y[keep], rcond=None)[0]
    err = y[keep] - (k * x[keep] + b)
    return float(k), float(b), keep, float(np.sqrt(np.mean(err**2)))


def edge_bins(u, v, z, side_percentile):
    z_lo = float(np.percentile(z, 5))
    z_hi = min(float(np.percentile(z, 92)), float(np.percentile(z, 50) + 0.45))
    m = (u >= -0.30) & (u <= 2.60) & (z >= z_lo) & (z <= z_hi)
    bins = np.floor(u[m] / 0.10).astype(int)
    uu = u[m]
    vv = v[m]
    out_u = []
    out_v = []
    for b in np.unique(bins):
        bm = bins == b
        if int(bm.sum()) < 10:
            continue
        out_u.append(float(np.median(uu[bm])))
        out_v.append(float(np.percentile(vv[bm], side_percentile)))
    return np.asarray(out_u), np.asarray(out_v)


def fit_height_break(u, v, z, k, b):
    line_v = k * u + b
    near = np.abs(v - line_v) <= 0.24
    z_use = z[near]
    u_use = u[near]
    if len(u_use) < 120:
        return None
    bins = np.floor(u_use / 0.08).astype(int)
    pu = []
    pz = []
    for bin_id in np.unique(bins):
        bm = bins == bin_id
        if int(bm.sum()) < 8:
            continue
        pu.append(float(np.median(u_use[bm])))
        pz.append(float(np.percentile(z_use[bm], 65.0)))
    pu = np.asarray(pu)
    pz = np.asarray(pz)
    order = np.argsort(pu)
    pu = pu[order]
    pz = pz[order]
    best = None
    for split in range(5, len(pu) - 4):
        left_u, left_z = pu[:split], pz[:split]
        right_u, right_z = pu[split:], pz[split:]
        la = np.column_stack((left_u, np.ones_like(left_u)))
        ra = np.column_stack((right_u, np.ones_like(right_u)))
        lk, lb = np.linalg.lstsq(la, left_z, rcond=None)[0]
        rk, rb = np.linalg.lstsq(ra, right_z, rcond=None)[0]
        if not (0.10 <= lk <= 0.55 and abs(rk) <= 0.09):
            continue
        lerr = left_z - (lk * left_u + lb)
        rerr = right_z - (rk * right_u + rb)
        rmse = math.sqrt(float(np.mean(np.concatenate((lerr, rerr)) ** 2)))
        denom = lk - rk
        if abs(denom) < 0.05:
            continue
        cross_u = float((rb - lb) / denom)
        if not (left_u.min() <= cross_u <= right_u.max()):
            continue
        # 图册中 Z3 坡道有效长度约 1.5m；太靠前的高度折线更可能是坡脚/遮挡。
        if not (1.05 <= cross_u <= 1.85):
            continue
        score = rmse + 0.02 * abs(cross_u - pu[split])
        if best is None or score < best["score"]:
            best = {
                "cross_u": cross_u,
                "cross_z": float(lk * cross_u + lb),
                "ramp_k": float(lk),
                "plat_k": float(rk),
                "rmse": rmse,
                "profile_u": pu,
                "profile_z": pz,
                "score": score,
            }
    return best


def fit_height_break_1d(u_use, z_use):
    if len(u_use) < 120:
        return None
    bins = np.floor(u_use / 0.08).astype(int)
    pu = []
    pz = []
    for bin_id in np.unique(bins):
        bm = bins == bin_id
        if int(bm.sum()) < 8:
            continue
        pu.append(float(np.median(u_use[bm])))
        pz.append(float(np.percentile(z_use[bm], 65.0)))
    pu = np.asarray(pu)
    pz = np.asarray(pz)
    order = np.argsort(pu)
    pu = pu[order]
    pz = pz[order]
    best = None
    for split in range(5, len(pu) - 4):
        left_u, left_z = pu[:split], pz[:split]
        right_u, right_z = pu[split:], pz[split:]
        la = np.column_stack((left_u, np.ones_like(left_u)))
        ra = np.column_stack((right_u, np.ones_like(right_u)))
        lk, lb = np.linalg.lstsq(la, left_z, rcond=None)[0]
        rk, rb = np.linalg.lstsq(ra, right_z, rcond=None)[0]
        if not (0.10 <= lk <= 0.55 and abs(rk) <= 0.09):
            continue
        lerr = left_z - (lk * left_u + lb)
        rerr = right_z - (rk * right_u + rb)
        rmse = math.sqrt(float(np.mean(np.concatenate((lerr, rerr)) ** 2)))
        denom = lk - rk
        if abs(denom) < 0.05:
            continue
        cross_u = float((rb - lb) / denom)
        if not (left_u.min() <= cross_u <= right_u.max()):
            continue
        if not (1.05 <= cross_u <= 1.85):
            continue
        score = rmse + 0.02 * abs(cross_u - pu[split])
        if best is None or score < best["score"]:
            best = {
                "cross_u": cross_u,
                "cross_z": float(lk * cross_u + lb),
                "ramp_k": float(lk),
                "plat_k": float(rk),
                "rmse": rmse,
                "profile_u": pu,
                "profile_z": pz,
                "score": score,
            }
    return best


def ransac_x_of_y(y, x, threshold=0.08, rounds=220):
    if len(y) < 4:
        return None
    rng = np.random.default_rng(20260616)
    best = None
    best_count = 0
    for _ in range(rounds):
        i, j = rng.choice(len(y), 2, replace=False)
        if abs(y[j] - y[i]) < 1e-6:
            continue
        k = float((x[j] - x[i]) / (y[j] - y[i]))
        if abs(k) > 0.45:
            continue
        b = float(x[i] - k * y[i])
        err = np.abs(x - (k * y + b))
        keep = err <= threshold
        count = int(keep.sum())
        if count > best_count:
            best = (k, b, keep)
            best_count = count
    if best is None or best_count < 4:
        return None
    k, b, keep = best
    a = np.column_stack((y[keep], np.ones(best_count)))
    k, b = np.linalg.lstsq(a, x[keep], rcond=None)[0]
    err = x[keep] - (k * y[keep] + b)
    return float(k), float(b), keep, float(np.sqrt(np.mean(err**2)))


def fit_top_break_line(u, v, z, side_k, side_b, side_name):
    side_v = side_k * u + side_b
    if side_name == "positive":
        usable = (v >= -0.95) & (v <= side_v + 0.10)
    else:
        usable = (v <= 0.95) & (v >= side_v - 0.10)
    usable &= (u >= 0.15) & (u <= 2.55)
    v_bins = np.floor(v[usable] / 0.12).astype(int)
    uu = u[usable]
    vv = v[usable]
    zz = z[usable]
    break_u = []
    break_v = []
    break_z = []
    break_rmse = []
    for b in np.unique(v_bins):
        bm = v_bins == b
        if int(bm.sum()) < 90:
            continue
        fit = fit_height_break_1d(uu[bm], zz[bm])
        if fit is None:
            continue
        if fit["rmse"] > 0.035:
            continue
        break_u.append(float(fit["cross_u"]))
        break_v.append(float(np.median(vv[bm])))
        break_z.append(float(fit["cross_z"]))
        break_rmse.append(float(fit["rmse"]))
    if len(break_u) < 4:
        return None
    bx = np.asarray(break_u)
    by = np.asarray(break_v)
    line = ransac_x_of_y(by, bx, threshold=0.10)
    if line is None:
        return None
    k, b, keep, rmse = line
    kept_u = bx[keep]
    kept_v = by[keep]
    kept_z = np.asarray(break_z)[keep]
    kept_rmse = np.asarray(break_rmse)[keep]
    if len(kept_u) < 4:
        return None
    # top line: u = k * v + b. side line: v = side_k * u + side_b.
    denom = 1.0 - k * side_k
    if abs(denom) < 1e-6:
        return None
    corner_u = float((k * side_b + b) / denom)
    corner_v = float(side_k * corner_u + side_b)
    corner_z = float(np.median(kept_z))
    return {
        "k": float(k),
        "b": float(b),
        "rmse": float(rmse),
        "bins": int(len(kept_u)),
        "u": kept_u,
        "v": kept_v,
        "z": kept_z,
        "bin_rmse": kept_rmse,
        "corner_u": corner_u,
        "corner_v": corner_v,
        "corner_z": corner_z,
    }


def analyze(path):
    odoms, clouds = read_bag(path)
    win = pick_window(odoms)
    if win is None:
        print("window failed")
        return
    print(
        f"window ramp_rel={win['ramp_rel']:.3f} platform_rel={win['platform_rel']:.3f} "
        f"anchor=({win['anchor_x']:.3f},{win['anchor_y']:.3f}) yaw={math.degrees(win['yaw']):.2f}"
    )
    samples = []
    for t, msg in clouds:
        if win["start_t"] <= t <= win["end_t"]:
            xyz = parse_cloud(msg)
            if len(xyz) > 4500:
                xyz = xyz[:: max(1, len(xyz) // 4500)]
            samples.append(xyz)
    pts = np.concatenate(samples, axis=0)
    u, v, z = to_local(pts, win["anchor_x"], win["anchor_y"], win["yaw"])
    # 只保留机器人周围的 Z3 局部区域；几何拟合后续自己决定边。
    roi = (
        (u >= -0.60) & (u <= 2.80)
        & (v >= -1.40) & (v <= 1.40)
        & (z >= -0.35) & (z <= 0.70)
        & np.isfinite(z)
    )
    u, v, z = u[roi], v[roi], z[roi]
    print(f"roi n={len(u)} z_pct={np.percentile(z, [5, 50, 95])}")
    candidates = []
    for name, percentile in (("positive", 96.0), ("negative", 4.0)):
        eu, ev = edge_bins(u, v, z, percentile)
        line = ransac_line(eu, ev)
        if line is None:
            print(f"{name} side failed bins={len(eu)}")
            continue
        k, b, keep, edge_rmse = line
        brk = fit_height_break(u, v, z, k, b)
        if brk is None:
            print(f"{name} height break failed k={k:.3f} b={b:.3f} edge_rmse={edge_rmse:.3f}")
            continue
        top_line = fit_top_break_line(u, v, z, k, b, name)
        if top_line is not None:
            cu = top_line["corner_u"]
            cv = top_line["corner_v"]
            cz = top_line["corner_z"]
        else:
            cu = brk["cross_u"]
            cv = k * cu + b
            cz = brk["cross_z"]
        ox, oy, oz = to_odom(win["anchor_x"], win["anchor_y"], win["yaw"], cu, cv, cz)
        support = (
            (np.abs(u - cu) <= 0.22)
            & (np.abs(v - cv) <= 0.22)
            & (z >= cz - 0.12)
            & (z <= cz + 0.18)
        )
        candidates.append(
            {
                "name": name,
                "k": k,
                "b": b,
                "edge_rmse": edge_rmse,
                "corner_u": cu,
                "corner_v": cv,
                "corner_z": cz,
                "odom": (ox, oy, oz),
                "support": int(support.sum()),
                "break": brk,
                "top_line": top_line,
                "edge_u": eu,
                "edge_v": ev,
            }
        )
    if not candidates:
        print("result failed")
        return
    best = max(candidates, key=lambda c: c["support"] / max(0.02, c["edge_rmse"]))
    print(
        f"best side={best['name']} edge_k={best['k']:.4f} edge_b={best['b']:.4f} "
        f"edge_rmse={best['edge_rmse']:.4f} support={best['support']}"
    )
    print(
        f"corner local=({best['corner_u']:.3f},{best['corner_v']:.3f},{best['corner_z']:.3f}) "
        f"odom=({best['odom'][0]:.3f},{best['odom'][1]:.3f},{best['odom'][2]:.3f}) "
        f"ramp_k={best['break']['ramp_k']:.3f} plat_k={best['break']['plat_k']:.3f} "
        f"height_rmse={best['break']['rmse']:.3f}"
    )
    if best["top_line"] is not None:
        tl = best["top_line"]
        print(
            f"top_edge u=k*v+b k={tl['k']:.4f} b={tl['b']:.4f} "
            f"rmse={tl['rmse']:.4f} bins={tl['bins']}"
        )
    else:
        print("top_edge line unavailable, used side-band height break only")
    out_png = "/mnt/c/Users/22240/rc2026_snapshot/other/top_corner_cloud_first_debug.png"
    plot(out_png, u, v, z, best)


def plot(out_png, u, v, z, best):
    rng = np.random.default_rng(20260616)
    idx = np.arange(len(u))
    if len(idx) > 90000:
        idx = rng.choice(idx, 90000, replace=False)
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(14, 6), dpi=150)
    sc = ax0.scatter(u[idx], v[idx], c=z[idx], s=1, alpha=0.45, cmap="viridis")
    fig.colorbar(sc, ax=ax0, label="z")
    xs = np.linspace(-0.4, 2.7, 100)
    ax0.plot(xs, best["k"] * xs + best["b"], "r-", lw=2, label="cloud side edge")
    ax0.scatter(best["edge_u"], best["edge_v"], c="white", edgecolors="black", s=18, label="edge bins")
    if best["top_line"] is not None:
        tl = best["top_line"]
        ys = np.linspace(-1.10, 1.10, 100)
        ax0.plot(tl["k"] * ys + tl["b"], ys, color="orange", lw=2, label="top break edge")
        ax0.scatter(tl["u"], tl["v"], c="orange", edgecolors="black", s=26, label="top break bins")
    ax0.scatter([best["corner_u"]], [best["corner_v"]], c="red", s=60, label="top corner")
    ax0.set_aspect("equal", adjustable="box")
    ax0.grid(alpha=0.25)
    ax0.legend()
    ax0.set_xlabel("rough forward")
    ax0.set_ylabel("rough lateral")

    brk = best["break"]
    pu, pz = brk["profile_u"], brk["profile_z"]
    ax1.scatter(pu, pz, c="black", s=18, label="edge z profile")
    ax1.axvline(brk["cross_u"], color="red", lw=2, label="height break")
    ax1.set_xlabel("along fitted edge")
    ax1.set_ylabel("z")
    ax1.grid(alpha=0.25)
    ax1.legend()
    Path(out_png).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, bbox_inches="tight")
    print(f"debug_png={out_png}")


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} BAG_DIR", file=sys.stderr)
        sys.exit(2)
    rclpy.init()
    analyze(sys.argv[1])


if __name__ == "__main__":
    main()
