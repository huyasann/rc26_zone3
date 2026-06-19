from __future__ import annotations

import math
import sys

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except Exception as exc:  # pragma: no cover
    raise RuntimeError("缺少 PyQt5：sudo apt install python3-pyqt5") from exc

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rosbag2_interfaces.srv import TogglePaused
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class Zone3TfTunerNode(Node):
    def __init__(self) -> None:
        super().__init__("zone3_tf_tuner")
        self.parent_frame = str(self.declare_parameter("parent_frame", "odom").value)
        self.source_frame = str(self.declare_parameter("source_frame", "blue_zone3_root_auto").value)
        self.output_frame = str(self.declare_parameter("output_frame", "blue_zone3_root").value)
        self.marker_topic = str(self.declare_parameter("marker_topic", "/zone3_tf_tuner/axes").value)
        self.axis_length = float(self.declare_parameter("axis_length_m", 0.45).value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 20.0).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.pause_client = self.create_client(TogglePaused, "/rosbag2_player/toggle_paused")

        self.manual_pose = [0.0, 0.0, 0.0, 0.0]
        self.publish_enabled = True
        self.follow_source = True
        self.pause_request_in_flight = None

    def lookup_source(self) -> tuple[float, float, float, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.source_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return None
        trans = tf.transform.translation
        rot = tf.transform.rotation
        return (
            float(trans.x),
            float(trans.y),
            float(trans.z),
            yaw_from_quat(rot.x, rot.y, rot.z, rot.w),
        )

    def set_manual_pose(self, x: float, y: float, z: float, yaw_deg: float) -> None:
        self.manual_pose = [x, y, z, math.radians(yaw_deg)]

    def publish_manual_tf(self) -> None:
        if not self.publish_enabled:
            return
        x, y, z, yaw = self.manual_pose
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.parent_frame
        msg.child_frame_id = self.output_frame
        msg.transform.translation.x = float(x)
        msg.transform.translation.y = float(y)
        msg.transform.translation.z = float(z)
        qx, qy, qz, qw = quat_from_yaw(yaw)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(msg)
        self.publish_axes_marker(x, y, z, yaw)

    def publish_axes_marker(self, x: float, y: float, z: float, yaw: float) -> None:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.parent_frame
        marker.ns = "zone3_tf_tuner"
        marker.id = 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.035
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.9
        marker.color.b = 0.0

        def add_axis(dx: float, dy: float, dz: float) -> None:
            p0 = Point()
            p0.x = x
            p0.y = y
            p0.z = z
            p1 = Point()
            p1.x = x + dx
            p1.y = y + dy
            p1.z = z + dz
            marker.points.append(p0)
            marker.points.append(p1)

        c = math.cos(yaw)
        s = math.sin(yaw)
        add_axis(c * self.axis_length, s * self.axis_length, 0.0)
        add_axis(-s * self.axis_length, c * self.axis_length, 0.0)
        add_axis(0.0, 0.0, self.axis_length)
        self.marker_pub.publish(marker)

    def toggle_bag_pause(self) -> bool:
        if not self.pause_client.service_is_ready():
            return False
        self.pause_request_in_flight = self.pause_client.call_async(TogglePaused.Request())
        return True


class Zone3TfTunerWindow(QtWidgets.QWidget):
    def __init__(self, node: Zone3TfTunerNode) -> None:
        super().__init__()
        self.node = node
        self.setWindowTitle("Zone3 TF 手动校准")
        self.setMinimumWidth(420)
        self._updating = False

        layout = QtWidgets.QVBoxLayout(self)
        self.status_label = QtWidgets.QLabel("等待 TF...")
        layout.addWidget(self.status_label)

        form = QtWidgets.QFormLayout()
        self.parent_edit = QtWidgets.QLineEdit(self.node.parent_frame)
        self.source_edit = QtWidgets.QLineEdit(self.node.source_frame)
        self.output_edit = QtWidgets.QLineEdit(self.node.output_frame)
        form.addRow("父 frame", self.parent_edit)
        form.addRow("读取 frame", self.source_edit)
        form.addRow("发布 frame", self.output_edit)

        self.spin_x = self._spin(-30.0, 30.0, 0.001, " m")
        self.spin_y = self._spin(-30.0, 30.0, 0.001, " m")
        self.spin_z = self._spin(-5.0, 5.0, 0.001, " m")
        self.spin_yaw = self._spin(-180.0, 180.0, 0.01, " deg")
        form.addRow("X", self.spin_x)
        form.addRow("Y", self.spin_y)
        form.addRow("Z", self.spin_z)
        form.addRow("Yaw", self.spin_yaw)
        layout.addLayout(form)

        row = QtWidgets.QHBoxLayout()
        self.follow_box = QtWidgets.QCheckBox("跟随检测 TF")
        self.follow_box.setChecked(True)
        self.publish_box = QtWidgets.QCheckBox("发布校准 TF")
        self.publish_box.setChecked(True)
        row.addWidget(self.follow_box)
        row.addWidget(self.publish_box)
        layout.addLayout(row)

        buttons = QtWidgets.QHBoxLayout()
        self.read_button = QtWidgets.QPushButton("读取一次")
        self.zero_yaw_button = QtWidgets.QPushButton("Yaw 置零")
        self.pause_button = QtWidgets.QPushButton("空格暂停/继续 bag")
        buttons.addWidget(self.read_button)
        buttons.addWidget(self.zero_yaw_button)
        buttons.addWidget(self.pause_button)
        layout.addLayout(buttons)

        hint = QtWidgets.QLabel(
            "流程：先跟随读取当前拟合 TF；确认数值后取消“跟随检测 TF”；"
            "手动改 X/Y/Z/Yaw；场地模型会跟随 blue_zone3_root 变化。"
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)

        for spin in (self.spin_x, self.spin_y, self.spin_z, self.spin_yaw):
            spin.valueChanged.connect(self._on_spin_changed)
        self.parent_edit.editingFinished.connect(self._on_frames_changed)
        self.source_edit.editingFinished.connect(self._on_frames_changed)
        self.output_edit.editingFinished.connect(self._on_frames_changed)
        self.follow_box.stateChanged.connect(self._on_options_changed)
        self.publish_box.stateChanged.connect(self._on_options_changed)
        self.read_button.clicked.connect(self.read_source_once)
        self.zero_yaw_button.clicked.connect(lambda: self.spin_yaw.setValue(0.0))
        self.pause_button.clicked.connect(self.toggle_bag_pause)
        self.pause_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Space), self)
        self.pause_shortcut.activated.connect(self.toggle_bag_pause)

        period_ms = max(20, int(1000.0 / max(1.0, self.node.publish_rate_hz)))
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(period_ms)

    @staticmethod
    def _spin(lo: float, hi: float, step: float, suffix: str) -> QtWidgets.QDoubleSpinBox:
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setDecimals(4 if step < 0.01 else 2)
        spin.setSingleStep(step)
        spin.setSuffix(suffix)
        spin.setKeyboardTracking(False)
        return spin

    def _on_frames_changed(self) -> None:
        self.node.parent_frame = self.parent_edit.text().strip() or self.node.parent_frame
        self.node.source_frame = self.source_edit.text().strip() or self.node.source_frame
        self.node.output_frame = self.output_edit.text().strip() or self.node.output_frame

    def _on_options_changed(self) -> None:
        self.node.follow_source = self.follow_box.isChecked()
        self.node.publish_enabled = self.publish_box.isChecked()

    def _on_spin_changed(self) -> None:
        if self._updating:
            return
        if self.follow_box.isChecked():
            self.follow_box.setChecked(False)
            self.node.follow_source = False
            self.status_label.setText("\u5df2\u8fdb\u5165\u624b\u52a8\u6821\u51c6\uff1a\u505c\u6b62\u8ddf\u968f\u68c0\u6d4b TF")
        self.node.set_manual_pose(
            self.spin_x.value(),
            self.spin_y.value(),
            self.spin_z.value(),
            self.spin_yaw.value(),
        )

    def toggle_bag_pause(self) -> None:
        if self.node.toggle_bag_pause():
            self.status_label.setText("已发送 bag 暂停/继续请求")
        else:
            self.status_label.setText("未找到 /rosbag2_player/toggle_paused 服务")

    def read_source_once(self) -> None:
        self._on_frames_changed()
        pose = self.node.lookup_source()
        if pose is None:
            self.status_label.setText(f"未找到 TF: {self.node.parent_frame} -> {self.node.source_frame}")
            return
        self._set_spins_from_pose(pose)

    def _set_spins_from_pose(self, pose: tuple[float, float, float, float]) -> None:
        x, y, z, yaw = pose
        self._updating = True
        self.spin_x.setValue(x)
        self.spin_y.setValue(y)
        self.spin_z.setValue(z)
        self.spin_yaw.setValue(math.degrees(yaw))
        self._updating = False
        self.node.set_manual_pose(x, y, z, math.degrees(yaw))

    def tick(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if self.follow_box.isChecked():
            pose = self.node.lookup_source()
            if pose is not None:
                self._set_spins_from_pose(pose)
                self.status_label.setText(f"读取 {self.node.source_frame}，发布 {self.node.output_frame}")
            else:
                self.status_label.setText(f"未找到 TF: {self.node.parent_frame} -> {self.node.source_frame}")
        self.node.publish_manual_tf()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    node = Zone3TfTunerNode()
    window = Zone3TfTunerWindow(node)
    window.show()
    try:
        app.exec_()
    finally:
        marker = Marker()
        marker.header.frame_id = node.parent_frame
        marker.ns = "zone3_tf_tuner"
        marker.id = 1
        marker.action = Marker.DELETE
        node.marker_pub.publish(marker)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
