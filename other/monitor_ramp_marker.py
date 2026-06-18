#!/usr/bin/env python3
"""无头记录 /ramp/model_marker 的关键坐标。"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker


class MonitorRampMarker(Node):
    def __init__(self):
        super().__init__("monitor_ramp_marker")
        self.count = {}
        self.create_subscription(Marker, "/ramp/model_marker", self.cb, 50)

    def cb(self, msg: Marker):
        if msg.action == Marker.DELETEALL:
            self.get_logger().info("DELETEALL")
            return
        if msg.action == Marker.DELETE:
            self.get_logger().info(f"DELETE id={msg.id}")
            return
        if msg.ns != "ramp_model":
            return
        self.count[msg.id] = self.count.get(msg.id, 0) + 1
        if msg.id in (0, 1, 10) and msg.points:
            if msg.id == 10:
                text = []
                for i, p in enumerate(msg.points[:4]):
                    text.append(f"p{i}=({p.x:.3f},{p.y:.3f},{p.z:.3f})")
                self.get_logger().info(f"id=10 n={self.count[msg.id]} " + " ".join(text))
            else:
                p0 = msg.points[0]
                p1 = msg.points[-1]
                self.get_logger().info(
                    f"id={msg.id} n={self.count[msg.id]} "
                    f"p0=({p0.x:.3f},{p0.y:.3f},{p0.z:.3f}) "
                    f"p1=({p1.x:.3f},{p1.y:.3f},{p1.z:.3f})"
                )
        elif msg.id in (20, 21):
            p = msg.pose.position
            self.get_logger().info(
                f"id={msg.id} n={self.count[msg.id]} "
                f"p=({p.x:.3f},{p.y:.3f},{p.z:.3f}) "
                f"scale=({msg.scale.x:.3f},{msg.scale.y:.3f},{msg.scale.z:.3f})"
            )


def main():
    rclpy.init()
    node = MonitorRampMarker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
