#!/usr/bin/env python3
"""离线验证第三区立体角点：两条水平边 + 一条竖向边。"""

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


def pick_platform_window(odoms):
    z0 = float(np.median([o["z"] for o in odoms[:30]]))
    p0 = float(np.median([o["pitch"] for o in odoms[:30]]))
    t0 = odoms[0]["t"]
    enriched = []
    for o in odoms:
        pitch_abs = abs(math.degrees(angle_diff(o["pitch"], p0)))
        z_rise = o["z"] - z0
        enriched.append({**o, "rel": o["t"] - t0, "pitch_abs": pitch_abs, "z_rise": z_rise})
    start_i = None
    for i, o in enumerate(enriched):
        if o["z_rise"] >= 0.08 and o["pitch_abs"] >= 5.0:
            start_i = i
            break
    if start_i is None:
        return None
    platform_i = None
    for i in range(start_i, len(enriched)):
        o = enriched[i]
        if o["z_rise"] >= 0.34 and o["pitch_abs"] <= 10.5:
            platform_i = i
            break
    if platform_i is None:
        platform_i = min(len(enriched) - 1, start_i + 160)
    win = enriched[start_i:platform_i + 80]
    yaws = np.unwrap(np.asarray([o["yaw"] for o in win if o["pitch_abs"] >= 4.0], dtype=np.float64))
    yaw = float(np.median(yaws)) if len(yaws) else float(enriched[start_i]["yaw"])
    anchor = enriched[start_i]
    return {
        "start_t": enriched[start_i]["t"] - 1.0,
        "end_t": enriched[platform_i]["t"] + 2.0,
        "anchor_x": float(anchor["x"]),
        "anchor_y": float(anchor["y"]),
        "yaw": yaw,
        "start_rel": float(enriched[start_i]["rel"]),
        "platform_rel": float(enriched[platform_i]["rel"]),
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


def fit_y_of_x(x, y, trim):
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
    return float(k), float(b), int(len(x)), float(np.sqrt(np.mean(err**2)))


def fit_x_of_y(y, x, trim):
    fit = fit_y_of_x(y, x, trim)
    if fit is None:
        return None
    k, b, n, rmse = fit
    return k, b, n, rmse


def robust_far_edge_by_vertical_bins(pu, pv, outer_k, outer_b):
    # 远端横边不能直接取每个 lateral 的最大 forward；外凸点会把线拉歪。
    # 先只用靠近外侧长边的一段，再找 forward 的密集右边界。
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
    # 找右侧仍然有大量点的最后一段，而不是直接最大 u。
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
    # 这个边按场地几何应接近 v 方向，拟合 u = k*v+b，但限制斜率，避免外凸点拉倾斜。
    y_axis, x_val = boundary_by_bins(v0[near], u0[near], 0.08, 50.0, 6)
    if len(y_axis) < 5:
        return None
    fit = fit_x_of_y(y_axis, x_val, 0.06)
    if fit is None:
        return None
    k, b, n, rmse = fit
    if abs(k) > 0.12:
        # 保守回退：远端横边近似固定 forward，用中位数代表。
        b = float(np.median(x_val))
        k = 0.0
        err = x_val - b
        rmse = float(np.sqrt(np.mean(err**2)))
        n = int(len(x_val))
    return k, b, n, rmse, y_axis, x_val


def boundary_by_bins(axis, value, bin_size, percentile, min_count):
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


def vertical_support(u, v, z, corner_u, corner_v):
    near = (np.hypot(u - corner_u, v - corner_v) <= 0.16)
    if int(near.sum()) < 20:
        near = (np.hypot(u - corner_u, v - corner_v) <= 0.22)
    nz = z[near]
    nu = u[near]
    nv = v[near]
    if len(nz) == 0:
        return {
            "count": 0,
            "span": 0.0,
            "z_low": float("nan"),
            "z_high": float("nan"),
            "u": np.empty(0),
            "v": np.empty(0),
            "z": np.empty(0),
        }
    z_low, z_high = np.percentile(nz, [8, 92])
    return {
        "count": int(len(nz)),
        "span": float(z_high - z_low),
        "z_low": float(z_low),
        "z_high": float(z_high),
        "u": nu,
        "v": nv,
        "z": nz,
    }


def analyze(path):
    odoms, clouds = read_bag(path)
    win = pick_platform_window(odoms)
    if win is None:
        print("window failed")
        return
    print(
        f"window start_rel={win['start_rel']:.3f} platform_rel={win['platform_rel']:.3f} "
        f"anchor=({win['anchor_x']:.3f},{win['anchor_y']:.3f}) yaw={math.degrees(win['yaw']):.2f}"
    )
    samples = []
    for t, msg in clouds:
        if win["start_t"] <= t <= win["end_t"]:
            xyz = parse_cloud(msg)
            if len(xyz) > 5000:
                xyz = xyz[:: max(1, len(xyz) // 5000)]
            samples.append(xyz)
    pts = np.concatenate(samples, axis=0)
    u, v, z = to_local(pts, win["anchor_x"], win["anchor_y"], win["yaw"])
    roi = (
        (u >= -0.40) & (u <= 3.00)
        & (v >= -1.60) & (v <= 1.60)
        & (z >= -0.35) & (z <= 0.75)
    )
    u, v, z = u[roi], v[roi], z[roi]
    print(f"roi n={len(u)}")
    # 只用高度筛掉明显杂点；后续只做俯视边线和角点。
    z_hi = float(np.percentile(z, 86))
    platform = (z >= z_hi - 0.09) & (z <= z_hi + 0.12) & (u >= 0.55)
    pu, pv, pz = u[platform], v[platform], z[platform]
    if len(pu) < 600:
        print(f"platform failed n={len(pu)}")
        return
    print(f"platform_points n={len(pu)}")

    candidates = []
    for side_name, side_pct, far_pct in (
        ("positive", 96.0, 96.0),
        ("negative", 4.0, 96.0),
    ):
        # 外侧长边：每个 u 小段取 v 的外侧边界。
        su, sv = boundary_by_bins(pu, pv, 0.10, side_pct, 8)
        side_fit = fit_y_of_x(su, sv, 0.08)
        if side_fit is None:
            print(f"{side_name} side failed bins={len(su)}")
            continue
        sk, sb, sn, srmse = side_fit
        # 远端横边：按密集右边界找，不追最外凸噪声点。
        far_fit_full = robust_far_edge_by_vertical_bins(pu, pv, sk, sb)
        if far_fit_full is None:
            fu, fv = boundary_by_bins(pv, pu, 0.10, far_pct, 8)
            far_fit = fit_x_of_y(fu, fv, 0.08)
        else:
            fk0, fb0, fn0, frmse0, fu0, fv0 = far_fit_full
            far_fit = (fk0, fb0, fn0, frmse0)
            fu, fv = fu0, fv0
        if far_fit is None:
            print(f"{side_name} far failed bins={len(fu)}")
            continue
        fk, fb, fn, frmse = far_fit
        # side: v=sk*u+sb; far: u=fk*v+fb.
        denom = 1.0 - fk * sk
        if abs(denom) < 1e-6:
            continue
        corner_u = float((fk * sb + fb) / denom)
        corner_v = float(sk * corner_u + sb)
        corner_z = 0.0
        # 两条水平边应接近正交。side direction=(1,sk), far direction=(fk,1)。
        dot = abs((1.0 * fk + sk * 1.0) / (math.hypot(1.0, sk) * math.hypot(fk, 1.0)))
        angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
        vs = vertical_support(u, v, z, corner_u, corner_v)
        support = (
            (np.abs(pu - corner_u) <= 0.25)
            & (np.abs(pv - corner_v) <= 0.25)
        )
        vertical_bonus = min(4.0, vs["count"] / 40.0) * min(3.0, vs["span"] / 0.06)
        angle_bonus = max(0.1, 1.0 - max(0.0, dot - 0.20))
        score = int(support.sum()) * angle_bonus * (1.0 + vertical_bonus) / max(0.005, srmse + frmse)
        candidates.append(
            {
                "side": side_name,
                "sk": sk,
                "sb": sb,
                "sn": sn,
                "srmse": srmse,
                "fk": fk,
                "fb": fb,
                "fn": fn,
                "frmse": frmse,
                "angle_deg": angle_deg,
                "orth_dot": float(dot),
                "vertical": vs,
                "corner_u": corner_u,
                "corner_v": corner_v,
                "corner_z": corner_z,
                "support": int(support.sum()),
                "score": score,
                "su": su,
                "sv": sv,
                "fu_axis": fu,
                "fv_val": fv,
            }
        )
    if not candidates:
        print("result failed")
        return
    best = max(candidates, key=lambda c: c["score"])
    ox, oy, oz = to_odom(win["anchor_x"], win["anchor_y"], win["yaw"], best["corner_u"], best["corner_v"], best["corner_z"])
    print(
        f"best side={best['side']} support={best['support']} "
        f"outer_rmse={best['srmse']:.4f} far_rmse={best['frmse']:.4f} "
        f"angle={best['angle_deg']:.1f}deg vertical_n={best['vertical']['count']} "
        f"vertical_span={best['vertical']['span']:.3f}"
    )
    print(
        f"corner local_xy=({best['corner_u']:.3f},{best['corner_v']:.3f}) "
        f"odom_xy=({ox:.3f},{oy:.3f})"
    )
    print(
        f"outer_edge v={best['sk']:.4f}*u+{best['sb']:.4f} bins={best['sn']} | "
        f"far_edge u={best['fk']:.4f}*v+{best['fb']:.4f} bins={best['fn']}"
    )
    out_png = "/mnt/c/Users/22240/rc2026_snapshot/other/zone3_platform_corner_debug.png"
    plot(out_png, u, v, z, pu, pv, best)


def plot(out_png, u, v, z, pu, pv, best):
    rng = np.random.default_rng(20260616)
    idx = np.arange(len(u))
    if len(idx) > 100000:
        idx = rng.choice(idx, 100000, replace=False)
    fig, (ax, axz) = plt.subplots(1, 2, figsize=(13, 6), dpi=150)
    ax.scatter(u[idx], v[idx], c="0.75", s=1, alpha=0.22, label="cloud")
    ax.scatter(pu[:: max(1, len(pu) // 25000)], pv[:: max(1, len(pv) // 25000)], c="lightgray", s=1, alpha=0.35, label="platform layer")
    xs = np.linspace(-0.2, 2.9, 120)
    ax.plot(xs, best["sk"] * xs + best["sb"], color="dodgerblue", lw=2.2, label="outer side edge")
    ys = np.linspace(-1.45, 1.45, 120)
    ax.plot(best["fk"] * ys + best["fb"], ys, color="red", lw=2.2, label="far edge")
    ax.scatter(best["su"], best["sv"], c="white", edgecolors="black", s=18, label="side bins")
    ax.scatter(best["fv_val"], best["fu_axis"], c="orange", edgecolors="black", s=18, label="far bins")
    if best["vertical"]["count"] > 0:
        ax.scatter(best["vertical"]["u"], best["vertical"]["v"], c="purple", s=10, alpha=0.85, label="vertical support")
    ax.scatter([best["corner_u"]], [best["corner_v"]], c="red", s=65, label="target corner")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("rough forward")
    ax.set_ylabel("rough lateral")
    ax.grid(alpha=0.25)
    ax.legend(loc="best")

    if best["vertical"]["count"] > 0:
        dz = np.hypot(best["vertical"]["u"] - best["corner_u"], best["vertical"]["v"] - best["corner_v"])
        axz.scatter(dz, best["vertical"]["z"], c="purple", s=12, alpha=0.8)
        axz.axhline(best["vertical"]["z_low"], color="purple", ls="--", lw=1.3)
        axz.axhline(best["vertical"]["z_high"], color="purple", ls="--", lw=1.3)
    axz.set_xlabel("distance to corner xy")
    axz.set_ylabel("z")
    axz.set_title("vertical edge check")
    axz.grid(alpha=0.25)
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
