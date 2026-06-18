#!/usr/bin/env python3
"""compare_marker_cloud_v2.py — 在 uphill 阶段采集点云，和 marker 对比。"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker


def parse_cloud(cloud):
    n = cloud.width * cloud.height if cloud.height > 1 else cloud.width
    dt = np.dtype({"names": ["x","y","z"], "formats": [np.float32]*3,
                   "offsets": [0,4,8], "itemsize": cloud.point_step})
    return np.frombuffer(cloud.data, dtype=dt, count=n)


class CompareMarkerCloud(Node):
    def __init__(self):
        super().__init__("compare_marker_cloud")
        self.marker = None
        self.cloud_frames = []
        self.odom = None
        self.state = "flat"
        self.enabled = False
        self.frame_count = 0

        self.create_subscription(Marker, "/ramp/model_marker", self.on_marker, 10)
        self.create_subscription(PointCloud2, "/odin1/cloud_slam", self.on_cloud, 10)
        self.create_subscription(Odometry, "/odin1/odometry_highfreq", self.on_odom, 10)
        self.create_subscription(String, "/uphill/state", self.on_state, 10)

        self.get_logger().info("等待 uphill 阶段采集点云...")

    def on_state(self, msg):
        old = self.state
        self.state = msg.data
        if self.state == "uphill":
            self.enabled = True
            self.cloud_frames.clear()
            self.get_logger().info("开始采集 uphill 点云")
        elif self.state == "platform":
            self.enabled = False
            self.get_logger().info(f"停止采集，共 {len(self.cloud_frames)} 帧")
            self.analyze()

    def on_odom(self, msg):
        self.odom = msg

    def on_marker(self, msg):
        if msg.action == Marker.DELETEALL:
            self.marker = None
            return
        self.marker = msg

    def on_cloud(self, msg):
        if not self.enabled:
            return
        pts = parse_cloud(msg)
        if len(pts) == 0:
            return
        x, y, z = pts["x"], pts["y"], pts["z"]
        fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        self.cloud_frames.append((x[fin], y[fin], z[fin]))

    def analyze(self):
        if not self.cloud_frames or self.marker is None:
            self.get_logger().warn("数据不足")
            return

        # 合并所有帧
        all_x = np.concatenate([f[0] for f in self.cloud_frames])
        all_y = np.concatenate([f[1] for f in self.cloud_frames])
        all_z = np.concatenate([f[2] for f in self.cloud_frames])

        self.get_logger().info(f"\n=== 分析结果 ===")
        self.get_logger().info(f"cloud: {len(all_x)} 点, X=[{all_x.min():.2f},{all_x.max():.2f}], Y=[{all_y.min():.2f},{all_y.max():.2f}], Z=[{all_z.min():.2f},{all_z.max():.2f}]")

        if self.marker:
            pts = self.marker.points
            self.get_logger().info(f"marker: {len(pts)} 点")
            for i, p in enumerate(pts):
                self.get_logger().info(f"  marker[{i}]: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")

            # 找 marker 附近的点
            for i, p in enumerate(pts):
                dist = np.sqrt((all_x - p.x)**2 + (all_y - p.y)**2 + (all_z - p.z)**2)
                near = dist < 0.5  # 放大到 50cm
                if near.sum() > 0:
                    near_z = all_z[near]
                    near_x = all_x[near]
                    near_y = all_y[near]
                    self.get_logger().info(
                        f"  marker[{i}] 附近 {near.sum()} 点: "
                        f"X=[{near_x.min():.2f},{near_x.max():.2f}], "
                        f"Z=[{near_z.min():.3f},{near_z.max():.3f}], "
                        f"Z均值={near_z.mean():.3f}, "
                        f"marker_z={p.z:.3f}, 差={p.z - near_z.mean():.3f}"
                    )
                else:
                    self.get_logger().info(f"  marker[{i}] 50cm 内无点")

        # 找地面层 Z
        ground_z = np.percentile(all_z, 5)
        fence_z = np.percentile(all_z, 95) - ground_z
        self.get_logger().info(f"地面 Z≈{ground_z:.3f}, 围栏高度≈{fence_z:.3f}m")


def main():
    rclpy.init()
    node = CompareMarkerCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
