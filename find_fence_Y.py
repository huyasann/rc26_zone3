#!/usr/bin/env python3

import json
import math
import numpy as np
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


class FenceLocator(Node):
    def __init__(self):
        super().__init__("fence_locator")
        self.state = "flat"
        self.last_odom = None          # (x, y, z, yaw)
        self.state_odom = {}           # state -> first odom
        self.platform_clouds = []

        self.create_subscription(String, "/uphill/state", self.on_state, 10)
        self.create_subscription(Odometry, "/odin1/odometry_highfreq", self.on_odom, 50)
        self.create_subscription(PointCloud2, "/odin1/cloud_slam", self.on_cloud, 10)

        self.get_logger().info("等待状态机完成 flat→platform...")

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny, cosy)
        self.last_odom = (float(p.x), float(p.y), float(p.z), yaw)

    def on_state(self, msg):
        old = self.state
        self.state = msg.data
        if self.state != old and self.last_odom is not None:
            self.state_odom[self.state] = self.last_odom
            x, y, z, yaw = self.last_odom
            self.get_logger().info(
                f"[odom] {old} -> {self.state}: x={x:.2f} y={y:.2f} z={z:.2f} yaw={math.degrees(yaw):.1f}°"
            )
            if self.state == "platform":
                self.get_logger().info("开始捕获 platform 点云...")

    def on_cloud(self, msg):
        if self.state != "platform":
            return
        if len(self.platform_clouds) >= 15:
            return
        self.platform_clouds.append(msg)
        if len(self.platform_clouds) >= 15:
            self.analyze()

    def analyze(self):
        # 收集所有点
        xs, ys, zs = [], [], []
        for cloud in self.platform_clouds:
            x, y, z = parse_cloud(cloud)
            xs.append(x); ys.append(y); zs.append(z)
        x = np.concatenate(xs); y = np.concatenate(ys); z = np.concatenate(zs)
        fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[fin], y[fin], z[fin]

        # 机器人上平台后的位置
        if "platform" in self.state_odom:
            rx, ry, rz, ryaw = self.state_odom["platform"]
        else:
            rx = ry = rz = ryaw = 0.0

        self.get_logger().info(f"=== platform odom: x={rx:.2f} y={ry:.2f} z={rz:.2f} yaw={math.degrees(ryaw):.1f}° ===")
        self.get_logger().info(f"点云: {len(x)}点, X=[{x.min():.1f},{x.max():.1f}] Y=[{y.min():.1f},{y.max():.1f}] Z=[{z.min():.2f},{z.max():.2f}]")

        # 围栏在地面: Z≈rz (机器人地面附近) ~ rz+0.3
        ground = (z >= rz - 0.2) & (z <= rz + 0.3)
        gx, gy = x[ground], y[ground]
        self.get_logger().info(f"地面层 (Z={rz-0.2:.1f}~{rz+0.3:.1f}): {len(gx)}点")

        # 沿机器人朝向 (odom+X) 方向切走廊, 找 Y 方向线结构
        # ramp 沿 odom+X, 围栏在 X 方向延伸, 在 Y 上呈窄峰
        corridor_half = 1.5
        nx, ny = gx[abs(gy) < corridor_half], gy[abs(gy) < corridor_half]
        self.get_logger().info(f"走廊 (±{corridor_half}m Y): {len(nx)}点")

        # X 方向切片
        self.get_logger().info("\n=== X切片 Y峰值 ===")
        x_range = (nx.min(), nx.max())
        for i in range(15):
            xl = x_range[0] + i*(x_range[1]-x_range[0])/15
            xr = x_range[0] + (i+1)*(x_range[1]-x_range[0])/15
            m = (nx >= xl) & (nx < xr)
            if m.sum() < 50: continue
            sy = ny[m]
            bins = max(40, int((sy.max()-sy.min())/0.02))
            h, e = np.histogram(sy, bins=bins)
            pi = np.argmax(h)
            py = (e[pi]+e[pi+1])/2
            sharp = h[pi]/max(1, sy[(abs(sy-py)<0.1)].shape[0])
            self.get_logger().info(f"  X=[{xl:.1f},{xr:.1f}]: peak Y={py:.2f} n={h[pi]} sharp={sharp:.2f}")

        print(f"\n=== 完成 === robot platform position: odom({rx:.2f}, {ry:.2f})")


def main():
    rclpy.init()
    node = FenceLocator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
