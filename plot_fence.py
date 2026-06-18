#!/usr/bin/env python3
"""画"platform"时的地面层点云俯视图，标注机器人位置。"""

import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def parse_cloud(cloud):
    n = cloud.width * cloud.height if cloud.height > 1 else cloud.width
    dt = np.dtype({"names": ["x","y","z"], "formats": [np.float32]*3,
                   "offsets": [0,4,8], "itemsize": cloud.point_step})
    pts = np.frombuffer(cloud.data, dtype=dt, count=n)
    return pts["x"], pts["y"], pts["z"]


class Plotter(Node):
    def __init__(self):
        super().__init__("plotter")
        self.state = "flat"
        self.last_odom = None
        self.clouds = []

        self.create_subscription(String, "/uphill/state", self.on_state, 10)
        self.create_subscription(Odometry, "/odin1/odometry_highfreq", self.on_odom, 50)
        self.create_subscription(PointCloud2, "/odin1/cloud_slam", self.on_cloud, 10)
        self.get_logger().info("等待 platform...")

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        self.last_odom = (p.x, p.y, p.z, math.atan2(siny, cosy))

    def on_state(self, msg):
        self.state = msg.data

    def on_cloud(self, msg):
        if self.state != "platform": return
        if len(self.clouds) >= 10: return
        self.clouds.append(msg)
        if len(self.clouds) >= 10:
            self.plot()
            rclpy.shutdown()

    def plot(self):
        xs, ys, zs = [], [], []
        for c in self.clouds:
            x, y, z = parse_cloud(c)
            xs.append(x); ys.append(y); zs.append(z)
        x = np.concatenate(xs); y = np.concatenate(ys); z = np.concatenate(zs)
        fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[fin], y[fin], z[fin]

        rx, ry, rz, ryaw = self.last_odom or (0,0,0,0)

        fig, axes = plt.subplots(1, 3, figsize=(18, 5))

        # 图1: 全部地面层点 (Z=rz-0.3 ~ rz+0.5)
        g = (z >= rz-0.3) & (z <= rz+0.5)
        ax = axes[0]
        ax.scatter(x[g][::20], y[g][::20], s=1, c="gray", alpha=0.3)
        ax.scatter(rx, ry, c="red", s=80, marker="x", linewidths=2, label="上平台位置")
        ax.set_xlabel("X (odom, m)")
        ax.set_ylabel("Y (odom, m)")
        ax.set_title(f"地面层点云 (Z={rz-0.3:.1f}~{rz+0.5:.1f})")
        ax.axis("equal")
        ax.legend()

        # 图2: 走廊放大 (±2m Y)
        corridor = g & (abs(y-ry) < 2.0)
        ax = axes[1]
        ax.scatter(x[corridor][::5], y[corridor][::5], s=1, c="blue", alpha=0.4)
        ax.scatter(rx, ry, c="red", s=80, marker="x", linewidths=2)
        ax.set_xlabel("X (odom, m)")
        ax.set_ylabel("Y (odom, m)")
        ax.set_title(f"走廊放大 Y=±2m (车在 y={ry:.1f})")
        ax.axis("equal")

        # 图3: Y 直方图
        ax = axes[2]
        cy = y[corridor]
        if len(cy) > 0:
            ax.hist(cy, bins=150, color="steelblue", edgecolor="none")
            ax.axvline(ry, color="red", linestyle="--", linewidth=1.5, label=f"车 Y={ry:.1f}")
        ax.set_xlabel("Y (odom, m)")
        ax.set_ylabel("点数")
        ax.set_title("走廊内 Y 直方图")
        ax.legend()

        plt.tight_layout()
        out = "/mnt/c/Users/22240/rc2026_snapshot/fence_topdown.png"
        plt.savefig(out, dpi=150)
        print(f"图已保存: {out}")
        print(f"robot platform position: odom({rx:.2f}, {ry:.2f}), yaw={math.degrees(ryaw):.1f}°")


def main():
    rclpy.init()
    node = Plotter()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
