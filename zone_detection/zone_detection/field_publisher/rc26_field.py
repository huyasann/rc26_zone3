#!/usr/bin/env python3.10
"""ROBOCON 2026 场地图模型发布节点。

发布 Z1/Z2/Z3 的 TF 和 Marker，支持单队和双队模式。
坐标系: X+→蓝队, X-→红队, Y+→武馆区, Z+→上。
"""

import math
import time
import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


# =============================================================================
# 1. 运行配置 — 需要时才改
# =============================================================================

# 默认显示模式，可通过 ROS 参数 display_mode 覆盖:
#   0 = 仅蓝方
#   1 = 仅红方
#   2 = 红蓝双发
DISPLAY_MODE = 0

# 总根 TF 开关:
#   True  -> 发布 field_root, 公共区域可统一挂到 field_root
#   False -> 不发布 field_root, 各区域 root 作为独立根帧使用
PUBLISH_FIELD_ROOT = 0

# Z1 挂到 Z2 根帧的开关，优先级低于 PUBLISH_FIELD_ROOT:
#   True  -> PUBLISH_FIELD_ROOT=False 且 Z1/Z2 开启时，发布 zone2_root -> zone1_root
#   False -> 不发布该父子 TF，zone1_root 仍由外部维护
PUBLISH_Z1_UNDER_Z2_ROOT = 1

# 区域发布开关: False 会完全跳过该区域的 TF 和 Marker
PUBLISH_Z1 = 1
PUBLISH_Z2 = 1
PUBLISH_Z3 = 1

# Marker 总开关: 关闭后不会发布任何 Marker
PUBLISH_MARKERS = True
# Helper Marker 开关: 仅控制辅助球等调试显示
PUBLISH_HELPER_MARKERS = True

# Marker 话题与发布频率: 根据 RViz/调试需求调整
MARKER_TOPIC = "/arena/field_markers"
MARKER_RATE = 2.0


# =============================================================================
# 2. 固定参数 — 图册/几何常量，默认不改
# =============================================================================

FIELD_ROOT_FRAME = "field_root"

# 场地基础尺寸
CARPET_HEIGHT = 0.05
CARPET_TOP_Z = CARPET_HEIGHT
CENTER_PARTITION_THICKNESS = 0.05
HALF_FIELD_X_OFFSET = CENTER_PARTITION_THICKNESS / 2.0
AREA_MARKER_HEIGHT = 0.002
AREA_MARKER_CENTER_Z = CARPET_TOP_Z + AREA_MARKER_HEIGHT / 2.0

# 配色
ZONE_RED_COLOR = (223, 34, 34)
ZONE_BLUE_COLOR = (50, 0, 255)
ZONE_ALPHA = 0.6
Z13_RED_CLR = (250, 220, 218)
Z13_BLUE_CLR = (128, 199, 226)
Z2_CHANNEL_RED_CLR = (236, 162, 151)
Z2_CHANNEL_BLUE_CLR = (117, 175, 191)
FLOOR_ALPHA = 0.9
Z1_FLOOR_RED_CLR = Z13_RED_CLR
Z1_FLOOR_BLUE_CLR = Z13_BLUE_CLR
Z3_PLATFORM_RED_CLR = Z13_RED_CLR
Z3_PLATFORM_BLUE_CLR = Z13_BLUE_CLR
Z3_PLATFORM_ALPHA = 1.0
Z3_CARPET_MAIN_RED_CLR = Z13_RED_CLR
Z3_CARPET_MAIN_BLUE_CLR = Z13_BLUE_CLR
Z3_CARPET_EXT_RED_CLR = Z13_RED_CLR
Z3_CARPET_EXT_BLUE_CLR = Z13_BLUE_CLR
Z3_CARPET_ALPHA = 1.0
WOOD_COLOR = (155, 95, 0)
WOOD_ALPHA_FULL = 1.0
WOOD_ALPHA_RACK = 0.8
Z3_RAMP_COLOR = (192, 189, 182)
Z3_RAMP_ALPHA = 1.0
GRID_BASE_COLOR = (255, 255, 255)
GRID_BASE_ALPHA = 1.0
GRID_BLOCK_COLOR = (0, 0, 0)
GRID_BLOCK_ALPHA = 0.35
BLOCK_COLORS = {2: {"rgb": (41, 82, 16), "alpha": 1.0}, 4: {"rgb": (42, 113, 56), "alpha": 1.0}, 6: {"rgb": (152, 166, 80), "alpha": 1.0}}

# 场地图级根帧与共享结构
ZONE1_ROOT = {"RED": [-3.0 - HALF_FIELD_X_OFFSET, 5.0, 0.0], "BLUE": [3.0 + HALF_FIELD_X_OFFSET, 5.0, 0.0]}
ZONE2_ROOT = {"RED": [-3.0 - HALF_FIELD_X_OFFSET, 0.55, 0.0], "BLUE": [3.0 + HALF_FIELD_X_OFFSET, 0.55, 0.0]}
ZONE3_ROOT = {"RED": [-3.0 - HALF_FIELD_X_OFFSET, -4.60, 0.0], "BLUE": [3.0 + HALF_FIELD_X_OFFSET, -4.60, 0.0]}
FIELD_Z3_SURFACE_CENTER = [0.0, -4.75, 0.4]
FIELD_Z3_SURFACE_FRAME = "field_zone3_surface"
ENDPIECE_CENTER = [0.0, 5.05, CARPET_TOP_Z + 0.25]
ENDPIECE_SIZE = [0.3, 1.2, 0.5]
ENDPIECE_SLOT_YS = [5.55, 5.35, 5.15, 4.95, 4.75, 4.55]
ENDPIECE_SLOT_Z = CARPET_TOP_Z + 0.5
GRID_BASE_REL = [0.0, 0.0, 0.2]
GRID_BASE_SIZE = [0.32, 1.62, 0.4]
GRID_BLOCK_SIZE = [0.30, 0.54, 0.54]
GRID_COL_YS_REL = [0.54, 0.0, -0.54]
GRID_LAYER_ZS_REL = [0.47, 1.01, 1.55]
GRID_COL_NAMES = {0: "inner", 1: "mid", 2: "outer"}
GRID_LAYER_NAMES = {0: "1", 1: "2", 2: "3"}
Z3_PARTITION_REL = [0.0, 0.0, -0.125]
Z3_PARTITION_SIZE = [0.05, 2.60, 0.55]

# Z1
Z1_FLOOR_REL = {"RED": [-0.025, 0.040, CARPET_HEIGHT / 2.0], "BLUE": [0.025, 0.040, CARPET_HEIGHT / 2.0]}
Z1_FLOOR_SIZE = [6.05, 2.02, CARPET_HEIGHT]
WEAPON_RACK_REL = {"RED": [-0.4, 0.85, CARPET_TOP_Z + 0.25], "BLUE": [0.4, 0.85, CARPET_TOP_Z + 0.25]}
WEAPON_RACK_SIZE = [0.8, 0.3, 0.5]
R1_ZONE_REL = {"RED": [-2.5, 0.5, AREA_MARKER_CENTER_Z], "BLUE": [2.5, 0.5, AREA_MARKER_CENTER_Z]}
R1_ZONE_SIZE = [1.0, 1.0, AREA_MARKER_HEIGHT]
R2_ZONE_REL = {"RED": [1.6, 0.6, AREA_MARKER_CENTER_Z], "BLUE": [-1.6, 0.6, AREA_MARKER_CENTER_Z]}
R2_ZONE_SIZE = [0.8, 0.8, AREA_MARKER_HEIGHT]
Z1_PARTITION_CENTER = [0.0, 5.040, 0.075]
Z1_PARTITION_SIZE = [0.05, 2.02, 0.15]
Z1_BACK_FENCE_REL = {"RED": [-0.025, 1.025, CARPET_TOP_Z + 0.05], "BLUE": [0.025, 1.025, CARPET_TOP_Z + 0.05]}
Z1_BACK_FENCE_SIZE = [6.05, 0.05, 0.10]
Z1_SIDE_FENCE_X = {"RED": -3.025, "BLUE": 3.025}
Z1_SIDE_FENCE_Y = 0.015
Z1_SIDE_FENCE_Z = CARPET_TOP_Z + 0.05
Z1_SIDE_FENCE_SIZE = [0.05, 1.97, 0.10]

# Z2
Z2_R2_ENTRY_REL = [0.0, 2.85, CARPET_HEIGHT / 2.0]
Z2_R2_ENTRY_SIZE = [3.54, 1.2, CARPET_HEIGHT]
Z2_R1_INNER_CHANNEL_REL = {"RED": [2.4, -0.275, CARPET_HEIGHT / 2.0], "BLUE": [-2.4, -0.275, CARPET_HEIGHT / 2.0]}
Z2_R1_INNER_CHANNEL_SIZE = [1.2, 7.45, CARPET_HEIGHT]
Z2_R1_OUTER_CHANNEL_REL = {"RED": [-2.425, 0.465, CARPET_HEIGHT / 2.0], "BLUE": [2.425, 0.465, CARPET_HEIGHT / 2.0]}
Z2_R1_OUTER_CHANNEL_SIZE = [1.25, 5.97, CARPET_HEIGHT]
Z2_R2_EXIT_MAIN_REL = {"RED": [0.135, -3.275, CARPET_HEIGHT / 2.0], "BLUE": [-0.135, -3.275, CARPET_HEIGHT / 2.0]}
Z2_R2_EXIT_MAIN_SIZE = [3.27, 1.45, CARPET_HEIGHT]
Z2_R2_EXIT_EXT_REL = {"RED": [-2.275, -3.2, CARPET_HEIGHT / 2.0], "BLUE": [2.275, -3.2, CARPET_HEIGHT / 2.0]}
Z2_R2_EXIT_EXT_SIZE = [1.55, 1.3, CARPET_HEIGHT]
Z2_MERLIN_CARPET_REL = {"RED": [0.0, -0.15, CARPET_HEIGHT / 2.0], "BLUE": [0.0, -0.15, CARPET_HEIGHT / 2.0]}
Z2_MERLIN_CARPET_SIZE = [3.6, 4.8, CARPET_HEIGHT]
Z2_BEST_OBS_POINT_REL = {"RED": [0.0, 3.70, CARPET_TOP_Z + 0.002], "BLUE": [0.0, 3.70, CARPET_TOP_Z + 0.002]}
Z2_BEST_OBS_POINT_SIZE = [1.5, 1.5, 0.002]
BLOCK_HEIGHTS = {"RED": [4, 2, 4, 2, 4, 6, 4, 6, 4, 2, 4, 2], "BLUE": [4, 2, 4, 6, 4, 2, 4, 6, 4, 2, 4, 2]}
BLOCK_COL_OFFSETS = [-1.2, 0.0, 1.2]
BLOCK_ROW_OFFSETS = [1.65, 0.45, -0.75, -1.95]
BLOCK_SIZE_XY = 1.2
MERLIN_COL_NAMES = {"RED": {0: "out", 1: "mid", 2: "in"}, "BLUE": {0: "in", 1: "mid", 2: "out"}}
MERLIN_ROW_NAMES = {0: "1", 1: "2", 2: "3", 3: "4"}
Z2_PARTITION_CENTER = [0.0, 0.275, 0.075]
Z2_PARTITION_SIZE = [0.05, 7.45, 0.15]
Z2_R1_OUTER_FENCE_X = {"RED": -3.025, "BLUE": 3.025}
Z2_R1_OUTER_FENCE_Y = 0.465
Z2_R1_OUTER_FENCE_Z = CARPET_TOP_Z + 0.05
Z2_R1_OUTER_FENCE_SIZE = [0.05, 5.97, 0.10]
Z2_R2_EXIT_EXT_FENCE_X = {"RED": -3.025, "BLUE": 3.025}
Z2_R2_EXIT_EXT_FENCE_Y = -3.2
Z2_R2_EXIT_EXT_FENCE_Z = CARPET_TOP_Z + 0.05
Z2_R2_EXIT_EXT_FENCE_SIZE = [0.05, 1.3, 0.10]

# Z3
Z3_MAIN_REL = {"RED": [0.75, -0.15, -0.15], "BLUE": [-0.75, -0.15, -0.15]}
Z3_MAIN_SIZE = [4.5, 2.60, 0.4]
Z3_SIDE_REL = {"RED": [-2.275, -0.825, -0.15], "BLUE": [2.275, -0.825, -0.15]}
Z3_SIDE_SIZE = [1.55, 1.25, 0.4]
Z3_RETRY_REL = {"RED": [-2.50, -0.90, 0.0505], "BLUE": [2.50, -0.90, 0.0505]}
Z3_RETRY_SIZE = [1.0, 1.0, 0.001]
Z3_CARPET_MAIN_REL = {"RED": [0.75, -0.15, CARPET_HEIGHT / 2.0], "BLUE": [-0.75, -0.15, CARPET_HEIGHT / 2.0]}
Z3_CARPET_MAIN_SIZE = [4.5, 2.60, CARPET_HEIGHT]
Z3_CARPET_EXT_REL = {"RED": [-2.275, -0.075, CARPET_HEIGHT / 2.0], "BLUE": [2.275, -0.075, CARPET_HEIGHT / 2.0]}
Z3_CARPET_EXT_SIZE = [1.55, 2.75, CARPET_HEIGHT]
Z3_RAMP_WIDTH = 1.55
Z3_RAMP_HLEN = 1.5
Z3_RAMP_Y_SOUTH = -0.20
Z3_RAMP_Y_NORTH = 1.30
Z3_RAMP_Z_LOW = CARPET_HEIGHT
Z3_RAMP_Z_HIGH = CARPET_HEIGHT + 0.4
Z3_SIDE_FENCE_X = {"RED": -0.75, "BLUE": 0.75}
Z3_SIDE_FENCE_Y = 0.0
Z3_SIDE_FENCE_Z = 0.25
Z3_SIDE_FENCE_SIZE = [0.05, 1.25, 0.10]
Z3_RAMP_FENCE_X = {"RED": -0.75, "BLUE": 0.75}
Z3_RAMP_FENCE_Y = 1.375
Z3_RAMP_FENCE_Z = 0.05
Z3_RAMP_FENCE_SIZE = [0.05, 1.57, 0.10]


# =============================================================================
#  辅助函数
# =============================================================================

def _rgbaf(rgb, alpha):
    """0~255 RGB + alpha → 0.0~1.0 RGBA."""
    return (rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0, alpha)


def _quat_from_rpy(roll, pitch, yaw):
    """欧拉角 (rad) → 四元数."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


# =============================================================================
#  发布节点
# =============================================================================

class Zone12CarpetPublisher(Node):
    """发布三区域 TF 树 + MarkerArray 的 ROS2 节点。"""

    def __init__(self):
        """初始化节点: 发布根 TF → 构建场地 TF 树 → 启动 Marker 定时器。"""
        super().__init__("zone12_carpet_publisher")

        mode_map = {
            0: (["BLUE"], "仅蓝方"),
            1: (["RED"], "仅红方"),
            2: (["RED", "BLUE"], "红蓝双发"),
        }
        self._display_mode = int(
            self.declare_parameter("display_mode", DISPLAY_MODE).value
        )
        self._active_teams, self._mode_label = mode_map.get(
            self._display_mode, (["BLUE"], f"未知({self._display_mode})")
        )

        if self._display_mode == 2 and not PUBLISH_FIELD_ROOT:
            raise ValueError(
                "display_mode=2 requires PUBLISH_FIELD_ROOT=True; "
                "shared public areas must be attached to field_root."
            )

        self._tf_static = StaticTransformBroadcaster(self)
        marker_qos = QoSProfile(depth=1)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._marker_pub = self.create_publisher(MarkerArray, MARKER_TOPIC, marker_qos)

        all_tfs = []
        all_tfs.extend(self._build_shared_transforms())
        for team in self._active_teams:
            all_tfs.extend(self._build_team_transforms(team))
        if all_tfs:
            self._send_static_transforms(all_tfs)

        if PUBLISH_MARKERS:
            self._timer = self.create_timer(1.0 / MARKER_RATE, self._publish_markers)

        self._print_config()

    # ------------------------------------------------------------------
    #  TF 构建
    # ------------------------------------------------------------------

    def _field_root_frame(self):
        """返回场地根帧名."""
        return FIELD_ROOT_FRAME

    def _shared_context(self):
        """共享/单队模式下的挂载父帧和根偏移."""
        if PUBLISH_FIELD_ROOT and self._display_mode == 2:
            zero = (0.0, 0.0, 0.0)
            return {
                "z1_root": zero,
                "z2_root": zero,
                "z3_root": zero,
                "ep_parent": self._field_root_frame(),
                "z1_parent": self._field_root_frame(),
                "z2_parent": self._field_root_frame(),
                "z3_parent": self._field_root_frame(),
            }

        team = self._active_teams[0]
        prefix = f"{team.lower()}_"
        z1_parent = f"{prefix}zone1_root"
        z2_parent = f"{prefix}zone2_root"
        z3_parent = f"{prefix}zone3_root"
        return {
            "z1_root": tuple(ZONE1_ROOT[team]),
            "z2_root": tuple(ZONE2_ROOT[team]),
            "z3_root": tuple(ZONE3_ROOT[team]),
            "ep_parent": z1_parent,
            "z1_parent": z1_parent,
            "z2_parent": z2_parent,
            "z3_parent": z3_parent,
        }

    def _append_tf(self, tfs, parent, child, xyz):
        """追加单位四元数的静态 TF."""
        tfs.append(self._tf(parent, child, xyz[0], xyz[1], xyz[2]))

    def _send_static_transforms(self, tfs):
        """校验并发布静态 TF，避免非法帧名污染 TF2 Buffer。"""
        valid_tfs = []
        child_to_parent = {}
        for tf_msg in tfs:
            parent = tf_msg.header.frame_id.strip()
            child = tf_msg.child_frame_id.strip()
            if not parent or not child:
                self.get_logger().error(
                    f"跳过非法 TF: parent='{parent}' child='{child}'"
                )
                continue

            previous_parent = child_to_parent.get(child)
            if previous_parent is not None:
                self.get_logger().error(
                    f"跳过重复 child_frame_id='{child}': "
                    f"已有 parent='{previous_parent}', 新 parent='{parent}'"
                )
                continue

            tf_msg.header.frame_id = parent
            tf_msg.child_frame_id = child
            child_to_parent[child] = parent
            valid_tfs.append(tf_msg)

        if valid_tfs:
            self._tf_static.sendTransform(valid_tfs)

    def _push_marker(self, ma, marker, mid):
        """追加 marker 并返回下一个 id."""
        ma.markers.append(marker)
        return mid + 1

    def _push_cube(self, ma, mid, frame, ns, xyz, size, rgba, stamp):
        """追加 CUBE marker 并返回下一个 id."""
        return self._push_marker(
            ma,
            self._cube(
                frame, ns, mid,
                xyz[0], xyz[1], xyz[2],
                size[0], size[1], size[2],
                rgba, stamp=stamp,
            ),
            mid,
        )

    def _push_cube_rot(self, ma, mid, frame, ns, xyz, size, rgba, quat, stamp):
        """追加带旋转的 CUBE marker 并返回下一个 id."""
        return self._push_marker(
            ma,
            self._cube_rot(
                frame, ns, mid,
                xyz[0], xyz[1], xyz[2],
                size[0], size[1], size[2],
                rgba,
                quat[0], quat[1], quat[2], quat[3],
                stamp=stamp,
            ),
            mid,
        )

    def _push_triangle(self, ma, mid, frame, ns, vertices, rgba, stamp):
        """追加 TRIANGLE_LIST marker 并返回下一个 id."""
        return self._push_marker(
            ma,
            self._triangle_list(frame, ns, mid, vertices, rgba, stamp=stamp),
            mid,
        )

    def _tf(self, parent, child, x, y, z, stamp=None):
        """构建 TransformStamped (旋转为单位四元数)。"""
        t = TransformStamped()
        t.header.stamp = stamp or self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.w = 1.0
        return t

    def _tf_yaw(self, parent, child, x, y, z, yaw, stamp=None):
        """构建仅含 yaw 的 TransformStamped。"""
        t = TransformStamped()
        t.header.stamp = stamp or self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        qx, qy, qz, qw = _quat_from_rpy(0.0, 0.0, yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        return t

    def _build_shared_transforms(self):
        """构建共享/可切换 TF 树: 端头架/分割栏/九宫格/Z3上表面。
        mode=2: 挂在 field_root 下 (共享模式)。
        mode=0/1: 挂在 {team}_zone{1,2,3}_root 下 (每队独立模式)。"""
        tfs = []
        ctx = self._shared_context()
        z1rx, z1ry, z1rz = ctx["z1_root"]
        z2rx, z2ry, z2rz = ctx["z2_root"]
        z3rx, z3ry, z3rz = ctx["z3_root"]

        if PUBLISH_Z1:
            self._append_tf(
                tfs, ctx["ep_parent"], "field_endpiece_rack",  # 端头架
                (
                    ENDPIECE_CENTER[0] - z1rx,
                    ENDPIECE_CENTER[1] - z1ry,
                    ENDPIECE_CENTER[2] - z1rz,
                ),
            )
            for i, sy in enumerate(ENDPIECE_SLOT_YS):
                self._append_tf(
                    tfs, "field_endpiece_rack", f"field_endpiece_slot_{i + 1}",
                    (0.0, sy - ENDPIECE_CENTER[1], ENDPIECE_SLOT_Z - ENDPIECE_CENTER[2]),
                )
            self._append_tf(
                tfs, ctx["z1_parent"], "field_zone1_center_partition",  # Z1 中心分割栏
                (
                    Z1_PARTITION_CENTER[0] - z1rx,
                    Z1_PARTITION_CENTER[1] - z1ry,
                    Z1_PARTITION_CENTER[2] - z1rz,
                ),
            )

        if PUBLISH_Z2:
            self._append_tf(
                tfs, ctx["z2_parent"], "field_zone2_center_partition",  # Z2 中心分割栏
                (
                    Z2_PARTITION_CENTER[0] - z2rx,
                    Z2_PARTITION_CENTER[1] - z2ry,
                    Z2_PARTITION_CENTER[2] - z2rz,
                ),
            )

        if PUBLISH_Z3 and self._display_mode == 2:
            self._append_tf(
                tfs, ctx["z3_parent"], FIELD_Z3_SURFACE_FRAME,  # Z3 共享上表面
                (
                    FIELD_Z3_SURFACE_CENTER[0] - z3rx,
                    FIELD_Z3_SURFACE_CENTER[1] - z3ry,
                    FIELD_Z3_SURFACE_CENTER[2] - z3rz,
                ),
            )

            bx, by, bz = GRID_BASE_REL
            self._append_tf(tfs, FIELD_Z3_SURFACE_FRAME, "field_zone3_grid_base", (bx, by, bz))  # 九宫格基座
            GRID_BASE_FRAME = "field_zone3_grid_base"
            for li, lz in enumerate(GRID_LAYER_ZS_REL):
                for ci, cy in enumerate(GRID_COL_YS_REL):
                    col_name = GRID_COL_NAMES[ci]
                    layer_name = GRID_LAYER_NAMES[li]
                    name = f"field_zone3_grid_{col_name}{layer_name}"
                    self._append_tf(tfs, GRID_BASE_FRAME, name, (0.0, cy, lz))
            px, py, pz = Z3_PARTITION_REL
            self._append_tf(
                tfs, FIELD_Z3_SURFACE_FRAME, "field_zone3_center_partition", (px, py, pz)  # Z3 中心分割栏
            )

        return tfs

    def _build_team_transforms(self, team):
        """构建每队专有 TF 树: Z1(地毯/武器架/启动区/边栏) +
        Z2(通道地毯/梅林/边栏) + Z3(地毯/坡道/高台/重试区/挡板/边栏)。"""
        tfs = []

        prefix = f"{team.lower()}_"
        z1_frame = f"{prefix}zone1_root"
        z2_frame = f"{prefix}zone2_root"
        z3_frame = f"{prefix}zone3_root"

        if PUBLISH_Z1:
            z1x, z1y, z1z = ZONE1_ROOT[team]
            if PUBLISH_FIELD_ROOT:
                self._append_tf(
                    tfs, self._field_root_frame(), z1_frame, (z1x, z1y, z1z)
                )  # Z1 根帧
            elif PUBLISH_Z1_UNDER_Z2_ROOT and PUBLISH_Z2:
                z2x, z2y, z2z = ZONE2_ROOT[team]
                self._append_tf(
                    tfs, z2_frame, z1_frame, (z1x - z2x, z1y - z2y, z1z - z2z)
                )  # Z1 根帧挂到 Z2 根帧
            z1_rel = Z1_FLOOR_REL[team]
            self._append_tf(tfs, z1_frame, f"{prefix}zone1", tuple(z1_rel))  # Z1 地毯
            wrx, wry, wrz = WEAPON_RACK_REL[team]
            self._append_tf(tfs, z1_frame, f"{prefix}weapon_rack", (wrx, wry, wrz))  # Z1 武器架
            self._append_tf(
                tfs, f"{prefix}weapon_rack", f"{prefix}weapon_rack_top",  # 武器架顶部
                (0.0, 0.0, WEAPON_RACK_SIZE[2] / 2.0),
            )
            bfx, bfy, bfz = Z1_BACK_FENCE_REL[team]
            self._append_tf(tfs, z1_frame, f"{prefix}zone1_back_fence", (bfx, bfy, bfz))  # Z1 后沿边栏
            sfx = Z1_SIDE_FENCE_X[team]
            self._append_tf(
                tfs, z1_frame, f"{prefix}zone1_side_fence",  # Z1 外侧边栏
                (sfx, Z1_SIDE_FENCE_Y, Z1_SIDE_FENCE_Z),
            )
            r1x, r1y, r1z = R1_ZONE_REL[team]
            self._append_tf(tfs, z1_frame, f"{prefix}r1_start", (r1x, r1y, r1z))  # R1 启动区
            r2x, r2y, r2z = R2_ZONE_REL[team]
            self._append_tf(tfs, z1_frame, f"{prefix}r2_start", (r2x, r2y, r2z))  # R2 启动区
            # R2 启动姿态参考（朝向梅林/九宫格方向）
            tfs.append(
                self._tf_yaw(
                    z1_frame,
                    f"{prefix}zone1_r2_spawn_ref",
                    r2x, r2y, r2z,
                    -math.pi / 2.0,
                )
            )

        if PUBLISH_Z2:
            z2x, z2y, z2z = ZONE2_ROOT[team]
            if PUBLISH_FIELD_ROOT:
                self._append_tf(
                    tfs, self._field_root_frame(), z2_frame, (z2x, z2y, z2z)
                )  # Z2 根帧
            self._append_tf(tfs, z2_frame, f"{prefix}zone2_r2_entry_area", tuple(Z2_R2_ENTRY_REL))  # Z2 入口
            inner = Z2_R1_INNER_CHANNEL_REL[team]
            self._append_tf(tfs, z2_frame, f"{prefix}zone2_r1_inner_channel", tuple(inner))  # Z2 内通道
            outer = Z2_R1_OUTER_CHANNEL_REL[team]
            self._append_tf(tfs, z2_frame, f"{prefix}zone2_r1_outer_channel", tuple(outer))  # Z2 外通道
            exit_main = Z2_R2_EXIT_MAIN_REL[team]
            self._append_tf(tfs, z2_frame, f"{prefix}zone2_r2_exit_main_area", tuple(exit_main))  # Z2 主出口
            exit_ext = Z2_R2_EXIT_EXT_REL[team]
            self._append_tf(tfs, z2_frame, f"{prefix}zone2_r2_exit_ext_area", tuple(exit_ext))  # Z2 扩展出口
            r1ofx = Z2_R1_OUTER_FENCE_X[team]
            self._append_tf(
                tfs, z2_frame, f"{prefix}zone2_r1_outer_fence",  # Z2 R1 外通道边栏
                (r1ofx, Z2_R1_OUTER_FENCE_Y, Z2_R1_OUTER_FENCE_Z),
            )
            eefx = Z2_R2_EXIT_EXT_FENCE_X[team]
            self._append_tf(
                tfs, z2_frame, f"{prefix}zone2_r2_exit_ext_fence",  # Z2 R2 扩展出口边栏
                (eefx, Z2_R2_EXIT_EXT_FENCE_Y, Z2_R2_EXIT_EXT_FENCE_Z),
            )
            merlin_carpet_x, merlin_carpet_y, merlin_carpet_z = Z2_MERLIN_CARPET_REL[team]
            self._append_tf(
                tfs, z2_frame, f"{prefix}zone2_merlin_carpet",  # Z2 梅林整体地毯
                (merlin_carpet_x, merlin_carpet_y, merlin_carpet_z),
            )
            heights = BLOCK_HEIGHTS[team]
            for i in range(12):
                row, col = i // 3, i % 3
                bx = BLOCK_COL_OFFSETS[col]
                by = BLOCK_ROW_OFFSETS[row]
                bz = CARPET_HEIGHT + heights[i] * 0.1
                col_name = MERLIN_COL_NAMES[team][col]
                row_name = MERLIN_ROW_NAMES[row]
                block_name = f"zone2_merlin_{col_name}{row_name}"
                self._append_tf(tfs, z2_frame, f"{prefix}{block_name}", (bx, by, bz))
            obs_x, obs_y, obs_z = Z2_BEST_OBS_POINT_REL[team]
            tfs.append(
                self._tf_yaw(
                    z2_frame, f"{prefix}zone2_best_obs_point",
                    obs_x, obs_y, obs_z, -math.pi / 2.0,
                )
            )

        if PUBLISH_Z3:
            z3x, z3y, z3z = ZONE3_ROOT[team]
            if PUBLISH_FIELD_ROOT:
                self._append_tf(
                    tfs, self._field_root_frame(), z3_frame, (z3x, z3y, z3z)
                )  # Z3 根帧
            cmx, cmy, cmz = Z3_CARPET_MAIN_REL[team]
            self._append_tf(tfs, z3_frame, f"{prefix}zone3_carpet_main", (cmx, cmy, cmz))  # Z3 主地毯
            cex, cey, cez = Z3_CARPET_EXT_REL[team]
            self._append_tf(tfs, z3_frame, f"{prefix}zone3_carpet_ext", (cex, cey, cez))  # Z3 扩展地毯
            ramp_x = Z3_SIDE_REL[team][0]
            ramp_y = (Z3_RAMP_Y_SOUTH + Z3_RAMP_Y_NORTH) / 2.0
            ramp_z = (Z3_RAMP_Z_LOW + Z3_RAMP_Z_HIGH) / 2.0
            self._append_tf(tfs, z3_frame, f"{prefix}zone3_ramp", (ramp_x, ramp_y, ramp_z))  # Z3 坡道中心
            z3_surface_frame = f"{prefix}zone3_surface"
            self._append_tf(tfs, z3_frame, z3_surface_frame, (0.0, 0.0, 0.4))  # Z3 上表面
            grid_base_frame = f"{prefix}field_zone3_grid_base"
            surface_dx = FIELD_Z3_SURFACE_CENTER[0] - z3x
            surface_dy = FIELD_Z3_SURFACE_CENTER[1] - z3y
            surface_dz = FIELD_Z3_SURFACE_CENTER[2] - z3z
            bx = surface_dx + GRID_BASE_REL[0]
            by = surface_dy + GRID_BASE_REL[1]
            bz = surface_dz - 0.4 + GRID_BASE_REL[2]
            self._append_tf(tfs, z3_surface_frame, grid_base_frame, (bx, by, bz))  # 九宫格基座
            for li, lz in enumerate(GRID_LAYER_ZS_REL):
                for ci, cy in enumerate(GRID_COL_YS_REL):
                    col_name = GRID_COL_NAMES[ci]
                    layer_name = GRID_LAYER_NAMES[li]
                    name = f"{prefix}field_zone3_grid_{col_name}{layer_name}"
                    self._append_tf(tfs, grid_base_frame, name, (0.0, cy, lz))
            px = surface_dx + Z3_PARTITION_REL[0]
            py = surface_dy + Z3_PARTITION_REL[1]
            pz = surface_dz - 0.4 + Z3_PARTITION_REL[2]
            self._append_tf(
                tfs, z3_surface_frame, f"{prefix}field_zone3_center_partition", (px, py, pz)
            )
            mrx, mry, mrz = Z3_MAIN_REL[team]
            self._append_tf(tfs, z3_surface_frame, f"{prefix}zone3_platform_main", (mrx, mry, mrz))  # Z3 主平台
            srx, sry, srz = Z3_SIDE_REL[team]
            self._append_tf(tfs, z3_surface_frame, f"{prefix}zone3_platform_side", (srx, sry, srz))  # Z3 侧平台
            main_frame = f"{prefix}zone3_platform_main"
            side_frame = f"{prefix}zone3_platform_side"
            rtrx_r = Z3_RETRY_REL[team][0] - Z3_SIDE_REL[team][0]
            rtry_r = Z3_RETRY_REL[team][1] - Z3_SIDE_REL[team][1]
            rtrz_r = Z3_RETRY_REL[team][2] - Z3_SIDE_REL[team][2]
            self._append_tf(tfs, side_frame, f"{prefix}zone3_retry_area", (rtrx_r, rtry_r, rtrz_r))  # Z3 重试区
            # Z3 重试区朝向九宫格中心参考（挂到 z3_root，便于直接用）
            retry_abs_x = z3x + Z3_RETRY_REL[team][0]
            retry_abs_y = z3y + Z3_RETRY_REL[team][1]
            grid_abs_x = FIELD_Z3_SURFACE_CENTER[0] + GRID_BASE_REL[0]
            grid_abs_y = FIELD_Z3_SURFACE_CENTER[1] + GRID_BASE_REL[1]
            retry_face_yaw = math.atan2(grid_abs_y - retry_abs_y, grid_abs_x - retry_abs_x)
            retry_top_center_z = 0.4 + Z3_RETRY_REL[team][2] + Z3_RETRY_SIZE[2] / 2.0
            tfs.append(
                self._tf_yaw(
                    z3_frame,
                    f"{prefix}zone3_retry_face_grid_ref",
                    Z3_RETRY_REL[team][0],
                    Z3_RETRY_REL[team][1],
                    retry_top_center_z,
                    retry_face_yaw,
                )
            )
            ssfx = Z3_SIDE_FENCE_X[team]
            self._append_tf(
                tfs, side_frame, f"{prefix}zone3_side_fence",  # Z3 侧边栏
                (ssfx, Z3_SIDE_FENCE_Y, Z3_SIDE_FENCE_Z),
            )
            rffx = Z3_RAMP_FENCE_X[team]
            self._append_tf(
                tfs, side_frame, f"{prefix}zone3_ramp_fence",  # Z3 坡道边栏
                (rffx, Z3_RAMP_FENCE_Y, Z3_RAMP_FENCE_Z),
            )
            baffle_z = Z3_MAIN_SIZE[2] / 2.0 + 0.05
            self._append_tf(
                tfs, main_frame, f"{prefix}zone3_baffle_front",  # Z3 前挡板
                (0.0, Z3_MAIN_SIZE[1] / 2.0 - 0.025, baffle_z),
            )
            s_l = Z3_SIDE_REL[team][0] - Z3_SIDE_SIZE[0] / 2.0
            s_r = Z3_SIDE_REL[team][0] + Z3_SIDE_SIZE[0] / 2.0
            m_l = Z3_MAIN_REL[team][0] - Z3_MAIN_SIZE[0] / 2.0
            m_r = Z3_MAIN_REL[team][0] + Z3_MAIN_SIZE[0] / 2.0
            c_l = min(s_l, m_l)
            c_r = max(s_r, m_r)
            back_baffle_x = (c_l + c_r) / 2.0 - Z3_MAIN_REL[team][0]
            self._append_tf(
                tfs, main_frame, f"{prefix}zone3_baffle_back",  # Z3 后挡板
                (back_baffle_x, -(Z3_MAIN_SIZE[1] / 2.0 - 0.025), baffle_z),
            )

        return tfs

    # ------------------------------------------------------------------
    #  Marker 辅助方法
    # ------------------------------------------------------------------

    def _cube(self, frame, ns, mid, x, y, z, sx, sy, sz, rgba, stamp=None):
        """构建 CUBE Marker."""
        m = Marker()
        m.header.stamp = stamp or self.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.ns = ns
        m.id = mid
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = sx
        m.scale.y = sy
        m.scale.z = sz
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.lifetime.sec = 0
        return m

    def _cube_rot(self, frame, ns, mid, x, y, z, sx, sy, sz, rgba,
                  qx, qy, qz, qw, stamp=None):
        """构建带旋转的 CUBE Marker (用于坡道斜边栏等)。"""
        m = self._cube(frame, ns, mid, x, y, z, sx, sy, sz, rgba, stamp)
        m.pose.orientation.x = qx
        m.pose.orientation.y = qy
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw
        return m

    def _sphere(self, frame, ns, mid, x, y, z, r, rgba, stamp=None):
        """构建 SPHERE Marker."""
        m = Marker()
        m.header.stamp = stamp or self.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = r
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.lifetime.sec = 0
        return m

    def _triangle_list(self, frame, ns, mid, vertices, rgba, stamp=None):
        """构建 TRIANGLE_LIST Marker (每3个 Point 为一个三角形面片, CCW 绕序正面可见)。"""
        m = Marker()
        m.header.stamp = stamp or self.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.ns = ns
        m.id = mid
        m.type = Marker.TRIANGLE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.lifetime.sec = 0
        for v in vertices:
            m.points.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))
        return m

    # ------------------------------------------------------------------
    #  Marker 构建
    # ------------------------------------------------------------------

    def _build_shared_markers(self, stamp, ma, mid):
        """构建共享/可切换 Marker: 端头架/分割栏/九宫格/Z3分割栏。
        mode=2: frame_id 为 field_root (共享模式)。
        mode=0/1: frame_id 为 {team}_zone{1,2}_root (每队独立模式)。"""
        ctx = self._shared_context()
        z1rx, z1ry, z1rz = ctx["z1_root"]
        z2rx, z2ry, z2rz = ctx["z2_root"]
        ep_frame = ctx["ep_parent"]
        slot_frame = ctx["ep_parent"]
        z1p_frame = ctx["z1_parent"]
        z2p_frame = ctx["z2_parent"]

        if PUBLISH_Z1:
            mid = self._push_marker(
                ma,
                self._cube(
                    ep_frame, "field_endpiece_rack", mid,  # 端头架
                    ENDPIECE_CENTER[0] - z1rx,
                    ENDPIECE_CENTER[1] - z1ry,
                    ENDPIECE_CENTER[2] - z1rz,
                    ENDPIECE_SIZE[0], ENDPIECE_SIZE[1], ENDPIECE_SIZE[2],
                    _rgbaf(WOOD_COLOR, WOOD_ALPHA_RACK), stamp=stamp,
                ),
                mid,
            )

            if PUBLISH_HELPER_MARKERS:
                gold = (1.0, 0.8, 0.0, 0.9)
                for sy in ENDPIECE_SLOT_YS:
                    mid = self._push_marker(
                        ma,
                        self._sphere(
                            slot_frame, "field_endpiece_slots", mid,  # 端头架插槽
                            -z1rx, sy - z1ry, ENDPIECE_SLOT_Z - z1rz,
                            0.08, gold, stamp=stamp,
                        ),
                        mid,
                    )

            z1psx, z1psy, z1psz = Z1_PARTITION_SIZE
            z1prgba = _rgbaf(WOOD_COLOR, WOOD_ALPHA_FULL)
            mid = self._push_marker(
                ma,
                self._cube(
                    z1p_frame, "field_zone1_center_partition", mid,  # Z1 中心分割栏
                    Z1_PARTITION_CENTER[0] - z1rx,
                    Z1_PARTITION_CENTER[1] - z1ry,
                    Z1_PARTITION_CENTER[2] - z1rz,
                    z1psx, z1psy, z1psz, z1prgba, stamp=stamp,
                ),
                mid,
            )

        if PUBLISH_Z2:
            z2psx, z2psy, z2psz = Z2_PARTITION_SIZE
            z2prgba = _rgbaf(WOOD_COLOR, WOOD_ALPHA_FULL)
            mid = self._push_marker(
                ma,
                self._cube(
                    z2p_frame, "field_zone2_center_partition", mid,  # Z2 中心分割栏
                    Z2_PARTITION_CENTER[0] - z2rx,
                    Z2_PARTITION_CENTER[1] - z2ry,
                    Z2_PARTITION_CENTER[2] - z2rz,
                    z2psx, z2psy, z2psz, z2prgba, stamp=stamp,
                ),
                mid,
            )

        if PUBLISH_Z3 and self._display_mode == 2:
            bx, by, bz = GRID_BASE_REL
            bsx, bsy, bsz = GRID_BASE_SIZE
            brgba = _rgbaf(GRID_BASE_COLOR, GRID_BASE_ALPHA)
            mid = self._push_marker(
                ma,
                self._cube(
                    FIELD_Z3_SURFACE_FRAME, "field_zone3_grid_base", mid,  # 九宫格基座
                    bx, by, bz, bsx, bsy, bsz, brgba, stamp=stamp,
                ),
                mid,
            )

            bsx2, bsy2, bsz2 = GRID_BLOCK_SIZE
            brgba2 = _rgbaf(GRID_BLOCK_COLOR, GRID_BLOCK_ALPHA)
            GRID_BASE_FRAME = "field_zone3_grid_base"
            for lz in GRID_LAYER_ZS_REL:
                for cy in GRID_COL_YS_REL:
                    mid = self._push_marker(
                        ma,
                        self._cube(
                            GRID_BASE_FRAME, "field_zone3_grid_blocks", mid,
                            0.0, cy, lz, bsx2, bsy2, bsz2, brgba2, stamp=stamp,
                        ),
                        mid,
                    )

            px, py, pz = Z3_PARTITION_REL
            psx, psy, psz = Z3_PARTITION_SIZE
            prgba = _rgbaf(WOOD_COLOR, WOOD_ALPHA_FULL)
            mid = self._push_marker(
                ma,
                self._cube(
                    FIELD_Z3_SURFACE_FRAME, "field_zone3_center_partition", mid,  # Z3 中心分割栏
                    px, py, pz, psx, psy, psz, prgba, stamp=stamp,
                ),
                mid,
            )

        return mid

    def _build_team_markers(self, team, stamp, ma, mid):
        """构建每队专有 Marker: Z1(地毯/武器架/启动区/边栏) +
        Z2(通道地毯/梅林/边栏) + Z3(地毯/高台/重试区/坡道/挡板/边栏)。"""
        prefix = f"{team.lower()}_"
        z1_frame = f"{prefix}zone1_root"
        z2_frame = f"{prefix}zone2_root"

        if PUBLISH_Z1:
            z1_floor_clr = Z1_FLOOR_RED_CLR if team == "RED" else Z1_FLOOR_BLUE_CLR
            z1_floor_rgba = _rgbaf(z1_floor_clr, FLOOR_ALPHA)
            z1_rel = Z1_FLOOR_REL[team]
            mid = self._push_cube(
                ma, mid, z1_frame, f"{prefix}zone1",  # Z1 地毯
                tuple(z1_rel), tuple(Z1_FLOOR_SIZE), z1_floor_rgba, stamp,
            )

            wrx, wry, wrz = WEAPON_RACK_REL[team]
            wrgba = _rgbaf(WOOD_COLOR, WOOD_ALPHA_RACK)
            mid = self._push_cube(
                ma, mid, z1_frame, f"{prefix}weapon_rack",  # Z1 武器架
                (wrx, wry, wrz), tuple(WEAPON_RACK_SIZE), wrgba, stamp,
            )

            bfx, bfy, bfz = Z1_BACK_FENCE_REL[team]
            bfgba = _rgbaf(WOOD_COLOR, WOOD_ALPHA_FULL)
            mid = self._push_cube(
                ma, mid, z1_frame, f"{prefix}zone1_back_fence",  # Z1 后沿边栏
                (bfx, bfy, bfz), tuple(Z1_BACK_FENCE_SIZE), bfgba, stamp,
            )

            sfx = Z1_SIDE_FENCE_X[team]
            sfgba = _rgbaf(WOOD_COLOR, WOOD_ALPHA_FULL)
            mid = self._push_cube(
                ma, mid, z1_frame, f"{prefix}zone1_side_fence",  # Z1 外侧边栏
                (sfx, Z1_SIDE_FENCE_Y, Z1_SIDE_FENCE_Z), tuple(Z1_SIDE_FENCE_SIZE), sfgba, stamp,
            )

            zone_clr = ZONE_RED_COLOR if team == "RED" else ZONE_BLUE_COLOR
            zrgba = _rgbaf(zone_clr, ZONE_ALPHA)

            r1x, r1y, r1z = R1_ZONE_REL[team]
            mid = self._push_cube(
                ma, mid, z1_frame, f"{prefix}r1_start",  # R1 启动区
                (r1x, r1y, r1z), tuple(R1_ZONE_SIZE), zrgba, stamp,
            )

            r2x, r2y, r2z = R2_ZONE_REL[team]
            mid = self._push_cube(
                ma, mid, z1_frame, f"{prefix}r2_start",  # R2 启动区
                (r2x, r2y, r2z), tuple(R2_ZONE_SIZE), zrgba, stamp,
            )

        if PUBLISH_Z2:
            z2_channel_clr = Z2_CHANNEL_RED_CLR if team == "RED" else Z2_CHANNEL_BLUE_CLR
            z2_channel_rgba = _rgbaf(z2_channel_clr, FLOOR_ALPHA)

            mid = self._push_cube(
                ma, mid, z2_frame, f"{prefix}zone2_r2_entry_area",  # Z2 入口
                tuple(Z2_R2_ENTRY_REL), tuple(Z2_R2_ENTRY_SIZE), z2_channel_rgba, stamp,
            )

            inner = Z2_R1_INNER_CHANNEL_REL[team]
            mid = self._push_cube(
                ma, mid, z2_frame, f"{prefix}zone2_r1_inner_channel",  # Z2 内通道
                tuple(inner), tuple(Z2_R1_INNER_CHANNEL_SIZE), z2_channel_rgba, stamp,
            )

            outer = Z2_R1_OUTER_CHANNEL_REL[team]
            mid = self._push_cube(
                ma, mid, z2_frame, f"{prefix}zone2_r1_outer_channel",  # Z2 外通道
                tuple(outer), tuple(Z2_R1_OUTER_CHANNEL_SIZE), z2_channel_rgba, stamp,
            )

            exit_main = Z2_R2_EXIT_MAIN_REL[team]
            mid = self._push_cube(
                ma, mid, z2_frame, f"{prefix}zone2_r2_exit_main_area",  # Z2 主出口
                tuple(exit_main), tuple(Z2_R2_EXIT_MAIN_SIZE), z2_channel_rgba, stamp,
            )

            exit_ext = Z2_R2_EXIT_EXT_REL[team]
            mid = self._push_cube(
                ma, mid, z2_frame, f"{prefix}zone2_r2_exit_ext_area",  # Z2 扩展出口
                tuple(exit_ext), tuple(Z2_R2_EXIT_EXT_SIZE), z2_channel_rgba, stamp,
            )

            r1ofx = Z2_R1_OUTER_FENCE_X[team]
            r1ofgba = _rgbaf(WOOD_COLOR, WOOD_ALPHA_FULL)
            mid = self._push_cube(
                ma, mid, z2_frame, f"{prefix}zone2_r1_outer_fence",  # Z2 R1 外通道边栏
                (r1ofx, Z2_R1_OUTER_FENCE_Y, Z2_R1_OUTER_FENCE_Z),
                tuple(Z2_R1_OUTER_FENCE_SIZE), r1ofgba, stamp,
            )

            eefx = Z2_R2_EXIT_EXT_FENCE_X[team]
            eefgba = _rgbaf(WOOD_COLOR, WOOD_ALPHA_FULL)
            mid = self._push_cube(
                ma, mid, z2_frame, f"{prefix}zone2_r2_exit_ext_fence",  # Z2 R2 扩展出口边栏
                (eefx, Z2_R2_EXIT_EXT_FENCE_Y, Z2_R2_EXIT_EXT_FENCE_Z),
                tuple(Z2_R2_EXIT_EXT_FENCE_SIZE), eefgba, stamp,
            )
            merlin_carp_clr = Z2_CHANNEL_RED_CLR if team == "RED" else Z2_CHANNEL_BLUE_CLR
            merlin_carp_rgba = _rgbaf(merlin_carp_clr, FLOOR_ALPHA)
            mcx, mcy, mcz = Z2_MERLIN_CARPET_REL[team]
            mid = self._push_cube(
                ma, mid, z2_frame, f"{prefix}zone2_merlin_carpet",  # Z2 梅林整体地毯
                (mcx, mcy, mcz), tuple(Z2_MERLIN_CARPET_SIZE), merlin_carp_rgba, stamp,
            )

            heights = BLOCK_HEIGHTS[team]
            for i in range(12):
                row, col = i // 3, i % 3
                bx = BLOCK_COL_OFFSETS[col]
                by = BLOCK_ROW_OFFSETS[row]
                bh = heights[i] * 0.1
                cfg = BLOCK_COLORS.get(heights[i], {"rgb": (128, 128, 128), "alpha": 0.5})
                rgba = _rgbaf(cfg["rgb"], cfg["alpha"])
                mid = self._push_cube(
                    ma, mid, z2_frame, f"{prefix}zone2_merlin_blocks",
                    (bx, by, CARPET_HEIGHT + bh / 2.0), (BLOCK_SIZE_XY, BLOCK_SIZE_XY, bh), rgba, stamp,
                )

            obs_rgba = _rgbaf((255, 215, 0), 0.6)  # 金色半透明
            mid = self._push_cube(
                ma, mid, z2_frame, f"{prefix}zone2_best_obs_point",
                tuple(Z2_BEST_OBS_POINT_REL[team]), tuple(Z2_BEST_OBS_POINT_SIZE), obs_rgba, stamp,
            )

        if PUBLISH_Z3:
            z3_root_frame = f"{prefix}zone3_root"
            cmc = Z3_CARPET_MAIN_RED_CLR if team == "RED" else Z3_CARPET_MAIN_BLUE_CLR
            cec = Z3_CARPET_EXT_RED_CLR if team == "RED" else Z3_CARPET_EXT_BLUE_CLR
            cm_rgba = _rgbaf(cmc, Z3_CARPET_ALPHA)
            ce_rgba = _rgbaf(cec, Z3_CARPET_ALPHA)

            cmx, cmy, cmz = Z3_CARPET_MAIN_REL[team]
            mid = self._push_cube(
                ma, mid, z3_root_frame, f"{prefix}zone3_carpet",  # Z3 主地毯
                (cmx, cmy, cmz), tuple(Z3_CARPET_MAIN_SIZE), cm_rgba, stamp,
            )

            cex, cey, cez = Z3_CARPET_EXT_REL[team]
            mid = self._push_cube(
                ma, mid, z3_root_frame, f"{prefix}zone3_carpet",  # Z3 扩展地毯
                (cex, cey, cez), tuple(Z3_CARPET_EXT_SIZE), ce_rgba, stamp,
            )

            ramp_x = Z3_SIDE_REL[team][0]
            rxb = ramp_x - Z3_RAMP_WIDTH / 2.0
            rxf = ramp_x + Z3_RAMP_WIDTH / 2.0
            ys = Z3_RAMP_Y_SOUTH
            yn = Z3_RAMP_Y_NORTH
            zl = Z3_RAMP_Z_LOW
            zh = Z3_RAMP_Z_HIGH

            # 6顶点: 南顶(BLt/BRt), 南底(BLg/BRg), 北地(FL/FR)
            BLt = (rxb, ys, zh);  BRt = (rxf, ys, zh)
            BLg = (rxb, ys, zl);  BRg = (rxf, ys, zl)
            FL  = (rxb, yn, zl);  FR  = (rxf, yn, zl)

            ramp_verts = [
                BLt, FR, FL,  BLt, BRt, FR,    # 斜面 (CCW朝上)
                BLg, FL, BRg,  FL, FR, BRg,    # 底面 (CCW朝下)
                BLg, BLt, FL,                   # 左侧面
                BRg, FR, BRt,                   # 右侧面
                BLg, BRt, BLt,  BLg, BRg, BRt,  # 南立面
            ]
            ramp_rgba = _rgbaf(Z3_RAMP_COLOR, Z3_RAMP_ALPHA)
            mid = self._push_triangle(
                ma, mid, z3_root_frame, f"{prefix}zone3_ramp", ramp_verts, ramp_rgba, stamp  # Z3 坡道
            )

            z3_surface_frame = f"{prefix}zone3_surface"
            z3_clr = Z3_PLATFORM_RED_CLR if team == "RED" else Z3_PLATFORM_BLUE_CLR
            z3rgba = _rgbaf(z3_clr, Z3_PLATFORM_ALPHA)

            z3x, z3y, z3z = ZONE3_ROOT[team]
            surface_dx = FIELD_Z3_SURFACE_CENTER[0] - z3x
            surface_dy = FIELD_Z3_SURFACE_CENTER[1] - z3y
            surface_dz = FIELD_Z3_SURFACE_CENTER[2] - z3z
            bx = surface_dx + GRID_BASE_REL[0]
            by = surface_dy + GRID_BASE_REL[1]
            bz = surface_dz - 0.4 + GRID_BASE_REL[2]
            bsx, bsy, bsz = GRID_BASE_SIZE
            brgba = _rgbaf(GRID_BASE_COLOR, GRID_BASE_ALPHA)
            grid_base_frame = f"{prefix}field_zone3_grid_base"
            mid = self._push_marker(
                ma,
                self._cube(
                    z3_surface_frame, "field_zone3_grid_base", mid,
                    bx, by, bz, bsx, bsy, bsz, brgba, stamp=stamp,
                ),
                mid,
            )

            bsx2, bsy2, bsz2 = GRID_BLOCK_SIZE
            brgba2 = _rgbaf(GRID_BLOCK_COLOR, GRID_BLOCK_ALPHA)
            for lz in GRID_LAYER_ZS_REL:
                for cy in GRID_COL_YS_REL:
                    mid = self._push_marker(
                        ma,
                        self._cube(
                            grid_base_frame, "field_zone3_grid_blocks", mid,
                            0.0, cy, lz, bsx2, bsy2, bsz2, brgba2, stamp=stamp,
                        ),
                        mid,
                    )

            px = surface_dx + Z3_PARTITION_REL[0]
            py = surface_dy + Z3_PARTITION_REL[1]
            pz = surface_dz - 0.4 + Z3_PARTITION_REL[2]
            psx, psy, psz = Z3_PARTITION_SIZE
            prgba = _rgbaf(WOOD_COLOR, WOOD_ALPHA_FULL)
            mid = self._push_marker(
                ma,
                self._cube(
                    z3_surface_frame, "field_zone3_center_partition", mid,
                    px, py, pz, psx, psy, psz, prgba, stamp=stamp,
                ),
                mid,
            )

            mrx, mry, mrz = Z3_MAIN_REL[team]
            mid = self._push_cube(
                ma, mid, z3_surface_frame, f"{prefix}zone3_platform",  # Z3 主平台
                (mrx, mry, mrz), tuple(Z3_MAIN_SIZE), z3rgba, stamp,
            )

            srx, sry, srz = Z3_SIDE_REL[team]
            mid = self._push_cube(
                ma, mid, z3_surface_frame, f"{prefix}zone3_platform",  # Z3 侧平台
                (srx, sry, srz), tuple(Z3_SIDE_SIZE), z3rgba, stamp,
            )

            side_frame = f"{prefix}zone3_platform_side"
            rtrx_r = Z3_RETRY_REL[team][0] - Z3_SIDE_REL[team][0]
            rtry_r = Z3_RETRY_REL[team][1] - Z3_SIDE_REL[team][1]
            rtrz_r = Z3_RETRY_REL[team][2] - Z3_SIDE_REL[team][2]
            retry_clr = ZONE_RED_COLOR if team == "RED" else ZONE_BLUE_COLOR
            retry_rgba = _rgbaf(retry_clr, ZONE_ALPHA)
            mid = self._push_cube(
                ma, mid, side_frame, f"{prefix}zone3_retry_area",  # Z3 重试区
                (rtrx_r, rtry_r, rtrz_r), tuple(Z3_RETRY_SIZE), retry_rgba, stamp,
            )

            ssfx = Z3_SIDE_FENCE_X[team]
            ssfgba = _rgbaf(WOOD_COLOR, WOOD_ALPHA_FULL)
            mid = self._push_cube(
                ma, mid, side_frame, f"{prefix}zone3_side_fence",  # Z3 侧边栏
                (ssfx, Z3_SIDE_FENCE_Y, Z3_SIDE_FENCE_Z), tuple(Z3_SIDE_FENCE_SIZE), ssfgba, stamp,
            )

            ramp_angle = math.atan2(Z3_RAMP_Z_HIGH - Z3_RAMP_Z_LOW,
                                    Z3_RAMP_Y_NORTH - Z3_RAMP_Y_SOUTH)
            rffx = Z3_RAMP_FENCE_X[team]
            rffgba = _rgbaf(WOOD_COLOR, WOOD_ALPHA_FULL)
            mid = self._push_cube_rot(
                ma, mid, side_frame, f"{prefix}zone3_ramp_fence",  # Z3 坡道斜边栏
                (rffx, Z3_RAMP_FENCE_Y, Z3_RAMP_FENCE_Z),
                tuple(Z3_RAMP_FENCE_SIZE),
                rffgba,
                (-math.sin(ramp_angle / 2.0), 0.0, 0.0, math.cos(ramp_angle / 2.0)),
                stamp,
            )

            main_frame = f"{prefix}zone3_platform_main"
            baffle_z = Z3_MAIN_SIZE[2] / 2.0 + 0.05
            brgba = _rgbaf(WOOD_COLOR, WOOD_ALPHA_FULL)

            mid = self._push_cube(
                ma, mid, main_frame, f"{prefix}zone3_baffle",  # Z3 前挡板
                (0.0, Z3_MAIN_SIZE[1] / 2.0 - 0.025, baffle_z),
                (4.5, 0.05, 0.1), brgba, stamp,
            )

            s_l = Z3_SIDE_REL[team][0] - Z3_SIDE_SIZE[0] / 2.0
            s_r = Z3_SIDE_REL[team][0] + Z3_SIDE_SIZE[0] / 2.0
            m_l = Z3_MAIN_REL[team][0] - Z3_MAIN_SIZE[0] / 2.0
            m_r = Z3_MAIN_REL[team][0] + Z3_MAIN_SIZE[0] / 2.0
            c_l = min(s_l, m_l)
            c_r = max(s_r, m_r)
            back_baffle_x = (c_l + c_r) / 2.0 - Z3_MAIN_REL[team][0]
            back_baffle_len = c_r - c_l
            mid = self._push_cube(
                ma, mid, main_frame, f"{prefix}zone3_baffle",  # Z3 后挡板
                (back_baffle_x, -(Z3_MAIN_SIZE[1] / 2.0 - 0.025), baffle_z),
                (back_baffle_len, 0.05, 0.1), brgba, stamp,
            )

        return mid

    @staticmethod
    def _marker_stamp():
        """Time(0,0) — RViz uses latest available TF, avoids cross-process flicker."""
        from builtin_interfaces.msg import Time as _T
        return _T(sec=0, nanosec=0)

    def _publish_markers(self):
        """定时器回调: 构建并发布 MarkerArray (共享 + 每队专有)。"""
        stamp = self._marker_stamp()
        try:
            ma = MarkerArray()
            mid = self._build_shared_markers(stamp, ma, 0)
            for team in self._active_teams:
                mid = self._build_team_markers(team, stamp, ma, mid)
            self._marker_pub.publish(ma)
        except Exception as exc:
            self.get_logger().error(f"Marker publish failed: {exc}", throttle_duration_sec=5.0)

    def clear_markers(self):
        """退出前清理 RViz 中的瞬态 MarkerArray。"""
        if not PUBLISH_MARKERS:
            return
        try:
            ma = MarkerArray()
            marker = Marker()
            marker.header.stamp = self._marker_stamp()
            marker.header.frame_id = "odom"
            marker.ns = ""
            marker.id = 0
            marker.action = Marker.DELETEALL
            ma.markers.append(marker)
            for _ in range(3):
                self._marker_pub.publish(ma)
                rclpy.spin_once(self, timeout_sec=0.05)
                time.sleep(0.05)
            self.get_logger().info("已清除 /arena/field_markers 残留 Marker")
        except Exception as exc:
            print(f"清除 /arena/field_markers 失败: {exc}")

    def _print_config(self):
        """日志输出当前运行配置。"""
        self.get_logger().info("=" * 56)
        self.get_logger().info(f"  模式        : {self._mode_label}")
        self.get_logger().info(f"  活跃队伍    : {', '.join(self._active_teams)}")
        self.get_logger().info(f"  发布区域    : "
                              f"Z1={'ON' if PUBLISH_Z1 else 'OFF'} "
                              f"Z2={'ON' if PUBLISH_Z2 else 'OFF'} "
                              f"Z3={'ON' if PUBLISH_Z3 else 'OFF'}")
        if PUBLISH_FIELD_ROOT:
            self.get_logger().info("  根TF        : 发布 field_root 作为总根")
            self.get_logger().info(
                f"  场地TF层级  : {self._field_root_frame()} -> zone1/2/3_root -> object"
            )
        elif PUBLISH_Z1_UNDER_Z2_ROOT and PUBLISH_Z1 and PUBLISH_Z2:
            self.get_logger().info("  根TF        : 不发布 field_root，Z1 根帧挂到 Z2 根帧")
            self.get_logger().info("  场地TF层级  : zone2_root -> zone1_root -> object")
        else:
            self.get_logger().info("  根TF        : 不发布 field_root，zone1/2/3_root 由外部维护")
            self.get_logger().info("  场地TF层级  : zone1/2/3_root -> object")
        self.get_logger().info(f"  Marker 话题 : {MARKER_TOPIC}")
        marker_count = (
            (2 if PUBLISH_Z1 else 0)  # endpiece + partition
            + (1 if PUBLISH_Z2 else 0)  # partition
            + (3 + 9 + 1 if PUBLISH_Z3 else 0)  # grid_base + 9 cubes + partition
        )
        team_count = len(self._active_teams)
        self.get_logger().info(
            f"  Marker数量  : ~{marker_count + team_count * (8 + 6 + 6 + 12 + 13)} "
            f"(Z1+Z2+Z3, {team_count}队)"
        )
        self.get_logger().info("  提示         : field_root 作为根帧，不再需要 parent TF")
        self.get_logger().info("=" * 56)


def main():
    """入口: 初始化 ROS2, 创建节点并 spin."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Zone12CarpetPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.clear_markers()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
