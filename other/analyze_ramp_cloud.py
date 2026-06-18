#!/usr/bin/env python3
"""离线分析 Z3 坡道点云，不依赖 RViz。

目标：
1. 用 odom 找出上坡窗口；
2. 累计 /odin1/cloud_slam；
3. 在起点局部坐标中统计坡面/围栏侧边候选；
4. 输出可用于修正 fence_locator 的数值。
"""

from __future__ import annotations

import math
import sys
from collections import deque
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
import rclpy
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
    return np.frombuffer(cloud.data, dtype=dtype, count=count)


def open_reader(path: str):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = {
        item.name: item.type
        for item in reader.get_all_topics_and_types()
    }
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


def fit_line(points_xy):
    pts = np.asarray(points_xy, dtype=np.float64)
    if len(pts) < 2:
        return None
    center = pts.mean(axis=0)
    _, _, vh = np.linalg.svd(pts - center, full_matrices=False)
    axis = vh[0]
    perp = np.array([-axis[1], axis[0]])
    err = (pts - center) @ perp
    rmse = float(np.sqrt(np.mean(err**2)))
    return center, axis, rmse


def analyze_cloud(cloud_samples, start_pose, end_pose, out_png: str | None = None):
    sx, sy, sz, syaw = start_pose
    ex, ey, ez, _ = end_pose
    odom_len = math.hypot(ex - sx, ey - sy)
    bottom_forward = max(0.0, odom_len - 1.50)
    pts = np.concatenate(cloud_samples, axis=0)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    dx, dy = x - sx, y - sy
    f = dx * math.cos(syaw) + dy * math.sin(syaw)
    l = -dx * math.sin(syaw) + dy * math.cos(syaw)
    roi = (
        (f >= bottom_forward - 0.40)
        & (f <= bottom_forward + 1.75)
        & (np.abs(l) <= 1.40)
        & np.isfinite(z)
    )
    f, l, z = f[roi], l[roi], z[roi]
    print(f"cloud_roi n={len(z)} bottom_forward_prior={bottom_forward:.3f} odom_len={odom_len:.3f}")
    if len(z) == 0:
        return
    print(
        "z_pct "
        + " ".join(f"{p}:{v:.3f}" for p, v in zip([1, 5, 10, 30, 50, 70, 90], np.percentile(z, [1, 5, 10, 30, 50, 70, 90])))
    )
    print(
        "l_pct "
        + " ".join(f"{p}:{v:.3f}" for p, v in zip([1, 5, 10, 50, 90, 95, 99], np.percentile(l, [1, 5, 10, 50, 90, 95, 99])))
    )
    print(
        "f_pct "
        + " ".join(f"{p}:{v:.3f}" for p, v in zip([1, 5, 10, 50, 90, 95, 99], np.percentile(f, [1, 5, 10, 50, 90, 95, 99])))
    )

    # 直接从 lateral 直方图找长边：每个 forward 分桶取高侧边缘，再拟合线。
    side_pts = []
    for lo in np.arange(bottom_forward - 0.10, bottom_forward + 1.55, 0.12):
        m = (f >= lo) & (f < lo + 0.12) & (z >= np.percentile(z, 5)) & (z <= np.percentile(z, 85))
        if int(m.sum()) < 10:
            continue
        side_pts.append((float(np.median(f[m])), float(np.percentile(l[m], 97.0))))
    print(f"side_bins={len(side_pts)}")
    if len(side_pts) >= 3:
        line = fit_line(side_pts)
        if line is not None:
            center, axis, rmse = line
            yaw_local = math.atan2(axis[1], axis[0])
            # line expressed approximately lateral = k*forward+b
            a = np.column_stack((np.asarray([p[0] for p in side_pts]), np.ones(len(side_pts))))
            k, b = np.linalg.lstsq(a, np.asarray([p[1] for p in side_pts]), rcond=None)[0]
            print(f"side_fit k={k:.4f} b={b:.4f} rmse={rmse:.4f} yaw_local={math.degrees(yaw_local):.2f}deg")
            print(f"corner_est forward={bottom_forward:.3f} lateral={k*bottom_forward+b:.3f}")
            if out_png:
                plot_debug(out_png, f, l, z, side_pts, bottom_forward, k, b)
    print("side_pts " + " ".join(f"({a:.2f},{b:.2f})" for a, b in side_pts[:20]))


def plot_debug(out_png, f, l, z, side_pts, bottom_forward, k, b):
    rng = np.random.default_rng(20260616)
    n = len(f)
    idx = np.arange(n)
    if n > 50000:
        idx = rng.choice(idx, 50000, replace=False)
    fig, ax = plt.subplots(figsize=(9, 7), dpi=150)
    sc = ax.scatter(f[idx], l[idx], c=z[idx], s=1, cmap="viridis", alpha=0.55)
    plt.colorbar(sc, ax=ax, label="odom z")
    xs = np.linspace(bottom_forward - 0.1, bottom_forward + 1.55, 100)
    ax.plot(xs, k * xs + b, color="red", lw=2.0, label="点云侧边拟合")
    ax.plot(xs, np.full_like(xs, 0.775), color="cyan", lw=2.0, label="图册先验 lateral=0.775")
    ax.axvline(bottom_forward, color="orange", lw=1.5, label=f"坡脚 forward={bottom_forward:.2f}")
    if side_pts:
        sp = np.asarray(side_pts)
        ax.scatter(sp[:, 0], sp[:, 1], c="white", edgecolors="black", s=30, label="侧边候选点")
    ax.set_xlabel("forward, odom start local")
    ax.set_ylabel("lateral, odom start local")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    ax.set_title("Z3 ramp side-edge debug")
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
    # First pass: collect odom and sparse cloud references.
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
        print(f"transition {old}->{new} rel={rel:.3f} t={o['t']:.3f} x={o['x']:.3f} y={o['y']:.3f} z={o['z']:.3f} yaw={math.degrees(o['yaw']):.2f} pitch={pitch:.2f} zema={z:.3f} vx={vx:.3f}")
    start = next((o for old, new, o, *_ in transitions if new == "flat_to_uphill"), None)
    end = next((o for old, new, o, *_ in transitions if new == "platform"), None)
    if start is None or end is None:
        print("no complete window")
        return
    samples = []
    for t, msg in clouds:
        if start["t"] <= t <= end["t"]:
            arr = parse_cloud(msg)
            valid = np.isfinite(arr["x"]) & np.isfinite(arr["y"]) & np.isfinite(arr["z"])
            xyz = np.column_stack((arr["x"][valid], arr["y"][valid], arr["z"][valid]))
            if len(xyz) > 2500:
                xyz = xyz[:: max(1, len(xyz) // 2500)]
            samples.append(xyz.astype(np.float64, copy=False))
    print(f"cloud_frames={len(samples)}")
    out_png = "/mnt/c/Users/22240/rc2026_snapshot/other/ramp_cloud_debug.png"
    analyze_cloud(
        samples,
        (start["x"], start["y"], start["z"], start["yaw"]),
        (end["x"], end["y"], end["z"], end["yaw"]),
        out_png,
    )


if __name__ == "__main__":
    main()
