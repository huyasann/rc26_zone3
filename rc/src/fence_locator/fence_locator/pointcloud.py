"""PointCloud2 解析工具。"""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import PointCloud2


def parse_cloud(cloud: PointCloud2) -> np.ndarray:
    count = cloud.width * cloud.height if cloud.height > 1 else cloud.width
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [np.float32, np.float32, np.float32],
            "offsets": [0, 4, 8],
            "itemsize": cloud.point_step,
        }
    )
    return np.frombuffer(cloud.data, dtype=dtype, count=count)
