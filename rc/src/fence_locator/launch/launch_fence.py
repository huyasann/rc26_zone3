"""兼容旧命令的 launch 文件。

推荐使用:
  ros2 launch fence_locator launch_fence.launch.py

旧命令仍可用:
  ros2 launch fence_locator launch_fence.py
"""

from importlib.util import module_from_spec
from importlib.util import spec_from_file_location
from pathlib import Path


def generate_launch_description():
    launch_path = Path(__file__).with_name("launch_fence.launch.py")
    spec = spec_from_file_location("fence_locator_launch_fence", launch_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载 launch 文件: {launch_path}")
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()
