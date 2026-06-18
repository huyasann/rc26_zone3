"""
test.launch.py - Bag播放 + RViz2 可视化测试

功能：
  1. 播放 bag 文件（自动发布 /clock 实现 ROS Time 同步）
  2. 启动 RViz2 可视化

解决 TF_OLD_DATA 问题：
  - rosbag 通过 --clock 发布 /clock 话题
  - RViz2 设置 use_sim_time=true 跟随 bag 时间戳
  - 两者同时启动，时间同步

使用：
  ros2 launch bringup test.launch.py
"""

import os
import sys

import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
import yaml


COMMON_CONFIG_PATH = '/home/inkc/inkc/Rc2026/files/configs/challenge1_common.yaml'


def resolve_python_executable():
    conda_prefix = os.environ.get('CONDA_PREFIX')
    if conda_prefix:
        candidate = os.path.join(conda_prefix, 'bin', 'python3')
        if os.path.exists(candidate):
            return candidate
    return sys.executable


def load_common_config(path=COMMON_CONFIG_PATH):
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    challenge = data.get('challenge1', {})
    team_value = str(challenge.get('team_side', 0)).strip().upper()
    team_map = {
        '0': 'BLUE',
        'BLUE': 'BLUE',
        '1': 'RED',
        'RED': 'RED',
    }
    if team_value not in team_map:
        raise ValueError(f"challenge1.team_side must be 0/1 or BLUE/RED, got {team_value!r}")
    team_side = team_map[team_value]
    return {
        'team_side': team_side,
        'is_blue_team': 'true' if team_side == 'BLUE' else 'false',
        'field_display_mode': '0' if team_side == 'BLUE' else '1',
        'detailed_file_log': challenge.get('detailed_file_log', False),  # 直接传 bool, 不转 string
    }


def generate_launch_description() -> launch.LaunchDescription:
    from launch_ros.actions import Node

    common_config = load_common_config()
    python_executable = resolve_python_executable()

    # ========================================================================
    # Bag 播放（ExecuteProcess）
    # ========================================================================
    # -r 0.1      : 以 0.1 倍速回放 bag
    # --clock 100 : 以 100Hz 发布 /clock（ROS Time 源）
    # --start-offset : 从第 N 秒开始播放（默认 0）
    # -l          : 循环播放

    bag_path = LaunchConfiguration('bag_path')
    bag_offset = LaunchConfiguration('bag_offset')
    bag_rate = LaunchConfiguration('bag_rate')
    team_side = LaunchConfiguration('team_side')
    rviz_config = LaunchConfiguration('rviz_config')
    is_blue_team = PythonExpression([
        "'", team_side, "'.strip().upper() in ('0', 'BLUE')"
    ])
    field_display_mode = PythonExpression([
        "'0' if '", team_side, "'.strip().upper() in ('0', 'BLUE') else '1'"
    ])
    detailed_file_log = common_config['detailed_file_log']

    bag_play_action = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            bag_path,
            '-r', bag_rate,
            '--start-offset', bag_offset,
            '--clock', '100',
            '--remap', '/tf:=/bag_tf',
            '--remap', '/tf_static:=/bag_tf_static',
        ],
        output='screen'
    )

    # ========================================================================
    # RViz2 可视化（Node）
    # ========================================================================
    # use_sim_time=true 跟随 /clock 时间戳

    rviz2_action = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    #python3 /home/inkc/inkc/Rc2026/src/action/bringup/script/rc26_field.py
    action_field = ExecuteProcess(
        cmd=[
            python_executable,
            '/home/inkc/inkc/Rc2026/src/tmp/rc26_field.py',
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', ['display_mode:=', field_display_mode],
        ],
        output='screen'
    )

    # zone2_detector (zone_detection 包)
    action_zone2 = Node(
        package='zone_detection',
        executable='zone2_detector',
        name='zone2_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'is_blue_team': is_blue_team,
            'detailed_file_log': detailed_file_log,
        }]
    )

    # zone3_localizer (zone_detection 包)
    action_zone3 = Node(
        package='zone_detection',
        executable='zone3_localizer',
        name='zone3_localizer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'is_blue_team': is_blue_team,
            'detailed_file_log': detailed_file_log,
        }]
    )

    # kfs_grid_detector (zone_detection 包) — 九宫格颜色检测
    action_kfs_grid = Node(
        package='zone_detection',
        executable='kfs_grid_detector',
        name='kfs_grid_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'detailed_file_log': detailed_file_log,
        }]
    )

    # ========================================================================
    # 启动描述
    # ========================================================================

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value='/home/inkc/inkc/Rc2026/files/record/bag/cha1nav2_20260531_211101',
            #cha1nav2_20260523_201149 红
            #cha1nav2_20260523_133237 蓝
            #cha1nav2_20260531_211101 蓝
            #cha1nav2_20260606_233301
            description='rosbag2 directory to play for zone3 testing',
        ),
        DeclareLaunchArgument(
            'bag_offset',
            default_value='0',
            description='rosbag start offset in seconds',
        ),
        DeclareLaunchArgument(
            'bag_rate',
            default_value='3.0',
            description='rosbag playback rate multiplier (0.1=slow, 3.0=fast)',
        ),
        DeclareLaunchArgument(
            'team_side',
            default_value=common_config['team_side'],
            choices=['BLUE', 'RED', '0', '1'],
            description='Team side for field display and zone detectors: BLUE/0 or RED/1',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='RViz config for close-up zone3 debugging',
        ),
        bag_play_action,
        rviz2_action,
        action_field,
        action_zone2,
        action_zone3,
        action_kfs_grid,
    ])
