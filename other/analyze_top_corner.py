#!/usr/bin/env python3
"""离线验证 Z3 上平台角点是否能由 /odin1/cloud_slam 稳定推出。"""

from __future__ import annotations

import math
import sys
from collections import deque
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
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    return reader, topic_types


def detect_window(odoms):
    baseline = odoms[:20]
    z0 = float(np.mean([o["z"] for o in baseline]))
    p0 = float(np.mean([o["pitch"] for o in baseline]))
    t0 = odoms[0]["t"]
    hist = deque()
    pitch_ema = None
    z_ema = None
    alpha = 0.35
    state = "flat"
    candidate_since = {}
    transitions = []

    def held(key, rel, cond, hold):
        if not cond:
            candidate_since[key] = None
            return False
        start = candidate_since.get(key)
        if start is None:
            candidate_since[key] = rel
            return False
        return rel - start >= hold

    for o in odoms[20:]:
        rel = o["t"] - t0
        pitch_abs = abs(math.degrees(angle_diff(o["pitch"], p0)))
        z_rise = o["z"] - z0
        pitch_ema = pitch_abs if pitch_ema is None else pitch_ema * (1.0 - alpha) + pitch_abs * alpha
        z_ema = z_rise if z_ema is None else z_ema * (1.0 - alpha) + z_rise * alpha
        hist.append((rel, o["x"], o["y"]))
        while hist and rel - hist[0][0] > 0.40:
            hist.popleft()
        vx = 0.0
        if len(hist) >= 2 and hist[-1][0] > hist[0][0]:
            vx = (hist[-1][1] - hist[0][1]) / (hist[-1][0] - hist[0][0])
        moving = vx > 0.08
        old = state
        if state == "flat":
            if held("flat_to_uphill", rel, moving and (pitch_ema >= 1.20 or z_ema >= 0.015), 0.04):
                state = "flat_to_uphill"
        elif state == "flat_to_uphill":
            if held("to_uphill", rel, moving and pitch_ema >= 12.69 and z_ema >= 0.06, 0.15):
                state = "uphill"
        elif state == "uphill":
            if held("to_platform_transition", rel, z_ema >= 0.36 and moving and pitch_ema < 12.69, 0.15):
                state = "uphill_to_platform"
        elif state == "uphill_to_platform":
            if held("to_platform", rel, z_ema >= 0.36 and pitch_ema <= 9.0, 0.15):
                state = "platform"
        if state != old:
            transitions.append((old, state, o, rel, pitch_ema, z_ema, vx))
            candidate_since.clear()
            if state == "platform":
                break
    return transitions


def fit_trimmed_line(x, y, trim):
    if len(x) < 3:
        return None
    a = np.column_stack((x, np.ones_like(x)))
    k, b = np.linalg.lstsq(a, y, rcond=None)[0]
    pred = k * x + b
    err = y - pred
    keep = np.abs(err) <= trim
    if int(keep.sum()) >= 3 and int(keep.sum()) < len(x):
        x = x[keep]
        y = y[keep]
        a = np.column_stack((x, np.ones_like(x)))
        k, b = np.linalg.lstsq(a, y, rcond=None)[0]
        pred = k * x + b
        err = y - pred
    rmse = float(np.sqrt(np.mean(err**2)))
    return float(k), float(b), rmse, int(len(x))


def profile_percentile(axis, values, bin_size, percentile, min_count):
    bins = np.floor(axis / bin_size).astype(int)
    out_axis = []
    out_values = []
    out_counts = []
    for b in np.unique(bins):
        m = bins == b
        if int(m.sum()) < min_count:
            continue
        out_axis.append(float(np.median(axis[m])))
        out_values.append(float(np.percentile(values[m], percentile)))
        out_counts.append(int(m.sum()))
    return np.asarray(out_axis), np.asarray(out_values), np.asarray(out_counts)


def localize(points, start_pose):
    sx, sy, _sz, syaw = start_pose
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    dx = x - sx
    dy = y - sy
    f = dx * math.cos(syaw) + dy * math.sin(syaw)
    l = -dx * math.sin(syaw) + dy * math.cos(syaw)
    return f, l, z


def odom_from_local(start_pose, f, l, z):
    sx, sy, _sz, syaw = start_pose
    return (
        sx + math.cos(syaw) * f - math.sin(syaw) * l,
        sy + math.sin(syaw) * f + math.cos(syaw) * l,
        z,
    )


def analyze(samples, start_pose, end_pose, out_png):
    pts = np.concatenate(samples, axis=0)
    if len(pts) > 500000:
        pts = pts[:: max(1, len(pts) // 500000)]
    f, l, z = localize(pts, start_pose)

    main = (f >= -0.15) & (f <= 2.25) & (l >= -0.95) & (l <= 0.95)
    f0, l0, z0 = f[main], l[main], z[main]
    print(f"main_roi n={len(f0)}")
    if len(f0) < 300:
        print("result: not_enough_cloud")
        return

    z_pct = np.percentile(z0, [2, 10, 30, 50, 70, 90, 98])
    print("z_pct " + " ".join(f"{p}:{v:.3f}" for p, v in zip([2, 10, 30, 50, 70, 90, 98], z_pct)))

    # 侧边：按 forward 分桶，取蓝区可见侧的高 lateral 边界。
    side_mask = (f0 >= 0.20) & (f0 <= 1.95) & (z0 >= z_pct[1]) & (z0 <= z_pct[5] + 0.05)
    side_f, side_l, side_n = profile_percentile(f0[side_mask], l0[side_mask], 0.10, 96.0, 8)
    side_fit = fit_trimmed_line(side_f, side_l, 0.10) if len(side_f) >= 5 else None
    if side_fit is None:
        print(f"side_fit failed bins={len(side_f)}")
        return
    side_k, side_b, side_rmse, side_bins = side_fit
    print(f"side_edge lateral=k*forward+b k={side_k:.4f} b={side_b:.4f} rmse={side_rmse:.4f} bins={side_bins}")

    # 顶端横边：在侧边附近取 z-forward 剖面，坡面与平台面的高度线求交。
    side_pred = side_k * f0 + side_b
    band = np.abs(l0 - side_pred) <= 0.22
    profile = band & (f0 >= 0.35) & (f0 <= 2.10)
    pf, pz, pn = profile_percentile(f0[profile], z0[profile], 0.08, 65.0, 8)
    ramp_m = (pf >= 0.45) & (pf <= 1.30)
    plat_m = (pf >= 1.35) & (pf <= 2.05)
    ramp_fit = fit_trimmed_line(pf[ramp_m], pz[ramp_m], 0.055) if int(ramp_m.sum()) >= 4 else None
    plat_fit = fit_trimmed_line(pf[plat_m], pz[plat_m], 0.035) if int(plat_m.sum()) >= 3 else None
    if ramp_fit is None or plat_fit is None:
        print(f"top_transition failed ramp_bins={int(ramp_m.sum())} plat_bins={int(plat_m.sum())}")
        return
    rk, rb, rrmse, rbins = ramp_fit
    pk, pb, prmse, pbins = plat_fit
    denom = rk - pk
    if abs(denom) < 0.05:
        print(f"top_transition failed parallel rk={rk:.4f} pk={pk:.4f}")
        return
    top_f = float((pb - rb) / denom)
    top_z = float(rk * top_f + rb)
    top_l = float(side_k * top_f + side_b)
    print(
        "top_cross "
        f"forward={top_f:.3f} lateral={top_l:.3f} z={top_z:.3f} "
        f"ramp_k={rk:.3f} plat_k={pk:.3f} ramp_rmse={rrmse:.3f} plat_rmse={prmse:.3f}"
    )

    # 角点附近验证：是否有一小团点同时靠近侧边、顶端横边和平台/坡交界高度。
    corner_support = (
        (np.abs(f0 - top_f) <= 0.18)
        & (np.abs(l0 - top_l) <= 0.18)
        & (z0 >= top_z - 0.10)
        & (z0 <= top_z + 0.16)
    )
    vertical_support = (
        (np.abs(f0 - top_f) <= 0.20)
        & (np.abs(l0 - top_l) <= 0.20)
        & (z0 >= np.percentile(z0, 20))
        & (z0 <= np.percentile(z0, 98))
    )
    print(f"corner_support n={int(corner_support.sum())} vertical_support n={int(vertical_support.sum())}")
    ox, oy, oz = odom_from_local(start_pose, top_f, top_l, top_z)
    print(f"corner_odom x={ox:.3f} y={oy:.3f} z={oz:.3f}")

    plot_debug(out_png, f0, l0, z0, side_f, side_l, side_k, side_b, pf, pz, rk, rb, pk, pb, top_f, top_l)


def plot_debug(out_png, f, l, z, side_f, side_l, side_k, side_b, pf, pz, rk, rb, pk, pb, top_f, top_l):
    rng = np.random.default_rng(20260616)
    idx = np.arange(len(f))
    if len(idx) > 90000:
        idx = rng.choice(idx, 90000, replace=False)
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(14, 6), dpi=150)
    sc = ax0.scatter(f[idx], l[idx], c=z[idx], s=1, cmap="viridis", alpha=0.45)
    fig.colorbar(sc, ax=ax0, label="z")
    xs = np.linspace(0.2, 2.05, 100)
    ax0.plot(xs, side_k * xs + side_b, "r-", lw=2, label="side edge")
    ax0.axvline(top_f, color="orange", lw=2, label="top transition")
    ax0.scatter([top_f], [top_l], c="red", s=55, label="corner")
    if len(side_f):
        ax0.scatter(side_f, side_l, c="white", edgecolors="black", s=20, label="edge bins")
    ax0.set_xlabel("forward")
    ax0.set_ylabel("lateral")
    ax0.set_aspect("equal", adjustable="box")
    ax0.grid(alpha=0.25)
    ax0.legend(loc="best")

    ax1.scatter(pf, pz, c="black", s=18, label="z profile")
    ax1.plot(xs, rk * xs + rb, "tab:orange", lw=2, label="ramp z fit")
    ax1.plot(xs, pk * xs + pb, "tab:green", lw=2, label="platform z fit")
    ax1.axvline(top_f, color="red", lw=2, label="cross")
    ax1.set_xlabel("forward")
    ax1.set_ylabel("z")
    ax1.grid(alpha=0.25)
    ax1.legend(loc="best")

    Path(out_png).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, bbox_inches="tight")
    print(f"debug_png={out_png}")


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} BAG_DIR", file=sys.stderr)
        sys.exit(2)
    rclpy.init()
    reader, topic_types = open_reader(sys.argv[1])
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
            msg: PointCloud2 = deserialize_message(data, type_map[topic])
            clouds.append((t_ns * 1e-9, msg))
    transitions = detect_window(odoms)
    for old, new, o, rel, pitch, z, vx in transitions:
        print(
            f"transition {old}->{new} rel={rel:.3f} x={o['x']:.3f} y={o['y']:.3f} "
            f"z={o['z']:.3f} yaw={math.degrees(o['yaw']):.2f} pitch={pitch:.2f} zema={z:.3f} vx={vx:.3f}"
        )
    start = next((o for old, new, o, *_ in transitions if new == "flat_to_uphill"), None)
    end = next((o for old, new, o, *_ in transitions if new == "platform"), None)
    if start is None or end is None:
        print("no complete window")
        return
    samples = []
    for t, msg in clouds:
        if start["t"] <= t <= end["t"] + 0.8:
            xyz = parse_cloud(msg)
            if len(xyz) > 3500:
                xyz = xyz[:: max(1, len(xyz) // 3500)]
            samples.append(xyz)
    print(f"cloud_frames={len(samples)}")
    analyze(
        samples,
        (start["x"], start["y"], start["z"], start["yaw"]),
        (end["x"], end["y"], end["z"], end["yaw"]),
        "/mnt/c/Users/22240/rc2026_snapshot/other/top_corner_debug.png",
    )


if __name__ == "__main__":
    main()
