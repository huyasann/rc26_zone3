from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class PointCloudHoldNode(Node):
    """缓存最新点云并持续重发，方便 rosbag 暂停时 RViz 画面不消散。"""

    def __init__(self) -> None:
        super().__init__("pointcloud_hold")
        self.input_topic = str(self.declare_parameter("input_topic", "/odin1/cloud_slam").value)
        self.output_topic = str(self.declare_parameter("output_topic", "/odin1/cloud_slam_hold").value)
        self.publish_hz = float(self.declare_parameter("publish_hz", 5.0).value)
        self.restamp = bool(self.declare_parameter("restamp", True).value)

        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.latest: PointCloud2 | None = None
        self.create_subscription(PointCloud2, self.input_topic, self.on_cloud, sub_qos)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, pub_qos)
        self.create_timer(1.0 / max(0.2, self.publish_hz), self.publish_latest)
        self.get_logger().info(f"点云保持: {self.input_topic} -> {self.output_topic}")

    def on_cloud(self, msg: PointCloud2) -> None:
        self.latest = msg

    def publish_latest(self) -> None:
        if self.latest is None:
            return
        msg = PointCloud2()
        msg.header = self.latest.header
        if self.restamp:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = self.latest.height
        msg.width = self.latest.width
        msg.fields = self.latest.fields
        msg.is_bigendian = self.latest.is_bigendian
        msg.point_step = self.latest.point_step
        msg.row_step = self.latest.row_step
        msg.data = self.latest.data
        msg.is_dense = self.latest.is_dense
        self.pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PointCloudHoldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
