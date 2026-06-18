#!/usr/bin/env python3
"""diagnose_fence.py — 诊断围栏检测过程，打印每帧的关键数据。

用法:
  终端1: ros2 launch fence_locator launch_fence.launch.py
  终端2: python3 diagnose_fence.py
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def parse_cloud(cloud):
    n = cloud.width * cloud.height if cloud.height > 1 else cloud.width
    dt = np.dtype({"names": ["x","y","z"], "formats": [np.float32]*3,
                   "offsets": [0,4,8], "itemsize": cloud.point_step})
    return np.frombuffer(cloud.data, dtype=dt, count=n)


class DiagnoseFence(Node):
    def __init__(self):
        super().__init__("diagnose_fence")
        self.state = "flat"
        self.enabled = False
        self.last_pose = None
        self.frame_count = 0

        self.create_subscription(String, "/uphill/state", self.on_state, 10)
        self.create_subscription(Odometry, "/odin1/odometry_highfreq", self.on_odom, 50)
        self.create_subscription(PointCloud2, "/odin1/cloud_slam", self.on_cloud, 10)

        self.get_logger().info("诊断模式：等待上坡...")

    def on_odom(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.last_pose = (float(pos.x), float(pos.y), yaw)

    def on_state(self, msg):
        old = self.state
        self.state = msg.data
        if self.state == old:
            return
        self.get_logger().info(f"[state] {old} -> {self.state}")
        if self.state == "uphill":
            self.enabled = True
            self.frame_count = 0
        elif self.state == "platform":
            self.enabled = False

    def on_cloud(self, msg):
        if not self.enabled or self.last_pose is None:
            return
        self.frame_count += 1

        pts = parse_cloud(msg)
        if len(pts) == 0:
            return
        x, y, z = pts["x"], pts["y"], pts["z"]
        fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[fin], y[fin], z[fin]

        rx, ry, ryaw = self.last_pose
        dx = x - rx
        dy = y - ry
        forward = dx * math.cos(ryaw) + dy * math.sin(ryaw)
        lateral = -dx * math.sin(ryaw) + dy * math.cos(ryaw)

        # 走廊内点
        corridor_near, corridor_far, corridor_half = 0.3, 5.0, 1.0
        in_corr = (forward >= corridor_near) & (forward <= corridor_far) & (abs(lateral) <= corridor_half)
        cx, cy, cz = forward[in_corr], lateral[in_corr], z[in_corr]

        # local_ground
        local_ground = np.percentile(cz, 5) if len(cz) > 10 else -999
        in_fence = (cz >= local_ground) & (cz <= local_ground + 0.15)
        fence_pts = in_fence.sum()

        self.get_logger().info(
            f"[frame {self.frame_count:3d}] "
            f"robot=({rx:.2f},{ry:.2f},{math.degrees(ryaw):+.1f}°) "
            f"total={len(x)} corridor={len(cx)} fence_band={fence_pts} "
            f"local_ground={local_ground:.2f} z_range=[{cz.min():.2f},{cz.max():.2f}]"
        )

        # Y 直方图（地面层）
        if fence_pts >= 10:
            sy = cy[in_fence]
            bins = max(20, int((sy.max() - sy.min()) / 0.02))
            h, e = np.histogram(sy, bins=bins)
            centers = (e[:-1] + e[1:]) / 2
            top3 = np.argsort(h)[-3:][::-1]
            peaks = " | ".join(f"Y={centers[i]:.2f} n={h[i]}" for i in top3 if h[i] > 0)
            self.get_logger().info(f"  Y peaks: {peaks}")


def main():
    rclpy.init()
    node = DiagnoseFence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
