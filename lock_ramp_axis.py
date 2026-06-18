#!/usr/bin/env python3
"""订阅 /uphill/transition_pose, 收集坡道进出口位姿, 计算坡道 X 轴。

用法：和 uphill 同时启动
  /usr/bin/python3 lock_ramp_axis.py
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RampAxisLocker(Node):
    def __init__(self):
        super().__init__("ramp_axis_locker")
        self.transitions: list[dict] = []
        self.create_subscription(PoseStamped, "/uphill/transition_pose", self.on_pose, 10)
        self.get_logger().info("等待坡道状态切换位姿...")
        self.get_logger().info("期望收集: '平地上坡中' -> '上坡中' (坡道入口), '坡上上平台' (平台)")

    def on_pose(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        yaw = quat_to_yaw(msg.pose.orientation)
        self.transitions.append({"t": t, "x": x, "y": y, "z": z, "yaw": yaw})
        self.get_logger().info(f"  [{len(self.transitions)}] t={t:.2f}s x={x:.3f} y={y:.3f} z={z:.3f}")

        if len(self.transitions) >= 3:
            self.analyze()

    def analyze(self):
        self.get_logger().info("\n=== 坡道 X 轴分析 ===")
        ts = self.transitions

        # 找"uphill"（坡道入口）和"uphill_to_platform"（平台接触）
        entry = ts[-3]  # flat_to_uphill -> uphill
        ramp = ts[-2]   # flat_to_uphill -> uphill
        plat = ts[-1]   # uphill -> uphill_to_platform

        dx = plat["x"] - ramp["x"]
        dy = plat["y"] - ramp["y"]
        dist = math.hypot(dx, dy)
        direction = math.degrees(math.atan2(dy, dx))
        z_rise = plat["z"] - ramp["z"]

        self.get_logger().info(f"ramp entry point: x={ramp['x']:.3f} y={ramp['y']:.3f} z={ramp['z']:.3f}")
        self.get_logger().info(f"platform contact point: x={plat['x']:.3f} y={plat['y']:.3f} z={plat['z']:.3f}")
        self.get_logger().info(f"位移向量: dx={dx:.3f} dy={dy:.3f} dz={z_rise:.3f}")
        self.get_logger().info(f"平面距离: {dist:.3f}m")
        self.get_logger().info(f"高度抬升: {z_rise:.3f}m")
        self.get_logger().info(f"方向角(偏离odom+X): {direction:.2f}°")

        # 判断是否可绑定 odom +X
        if abs(direction) < 5.0:
            self.get_logger().info("✅ 坡道 X = odom +X（偏差 < 5°）")
        else:
            self.get_logger().info(f"⚠️ 偏差 {direction:.2f}°，需检查")

        # 反推 zone3_root 在 odom 中的位置
        # 场地模型：坡道入口在 zone3_root 局部坐标 Y=1.30, Z=0.05
        # 坡道沿 zone3_root 的 -Y 方向, 长度 1.5m
        # 所以 zone3_root 的 odom 位置 = 坡道入口 + 偏移
        ramp_len_field = 1.50  # 场地模型坡道长度
        self.get_logger().info(f"\n=== 场地反推 ===")
        self.get_logger().info(f"ramp entry odom: ({ramp['x']:.3f}, {ramp['y']:.3f})")
        self.get_logger().info(f"measured ramp length: {dist:.3f}m (场地模型: {ramp_len_field}m)")

        print(f"\nramp entry odom: ({ramp['x']:.3f}, {ramp['y']:.3f})")
        print(f"坡道位移: ({dx:.3f}, {dy:.3f}), 长{dist:.3f}m, 方向{direction:.2f}°")
        print(f"抬升: {z_rise:.3f}m")
        print(f"X轴绑定: odom +X, 偏差 {direction:.2f}°")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = RampAxisLocker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
