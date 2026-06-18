"""点云解析与构造。

从 sensor_msgs/PointCloud2 提取 xyz、构造 RGB 染色点云。
所有函数纯数据驱动，不依赖 ROS node。
"""

from typing import Tuple

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def parse_xyz(cloud: PointCloud2) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """从 PointCloud2 提取 x/y/z 数组."""
    n = cloud.width * cloud.height if cloud.height > 1 else cloud.width
    dt = np.dtype({
        "names": ["x", "y", "z"],
        "formats": [np.float32] * 3,
        "offsets": [0, 4, 8],
        "itemsize": cloud.point_step,
    })
    pts = np.frombuffer(cloud.data, dtype=dt, count=n)
    return pts["x"], pts["y"], pts["z"]


def make_rgb_cloud(
    header: Header,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    r: np.ndarray,
    g: np.ndarray,
    b: np.ndarray,
) -> PointCloud2:
    """用 x/y/z + r/g/b 构造 RGB PointCloud2."""
    n = len(x)
    a = np.full(n, 255, dtype=np.uint8)
    pts = np.zeros(n, dtype=[("x", np.float32), ("y", np.float32),
                             ("z", np.float32), ("rgb", np.uint32)])
    pts["x"], pts["y"], pts["z"] = x, y, z
    pts["rgb"] = (a.astype(np.uint32) << 24) \
                 | (r.astype(np.uint32) << 16) \
                 | (g.astype(np.uint32) << 8) \
                 | b.astype(np.uint32)

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
        for name, offset in [("x", 0), ("y", 4), ("z", 8), ("rgb", 12)]
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.data = pts.tobytes()
    msg.is_dense = True
    return msg


def empty_cloud(header: Header) -> PointCloud2:
    """构造空点云 (发布以清空 RViz 显示)."""
    return make_rgb_cloud(
        header,
        np.empty(0, dtype=np.float32),
        np.empty(0, dtype=np.float32),
        np.empty(0, dtype=np.float32),
        np.empty(0, dtype=np.uint8),
        np.empty(0, dtype=np.uint8),
        np.empty(0, dtype=np.uint8),
    )
