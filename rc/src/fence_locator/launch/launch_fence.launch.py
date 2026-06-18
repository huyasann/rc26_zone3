"""启动 bag 回放、uphill 状态机和 fence_locator 节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    bag_rate = LaunchConfiguration("bag_rate")
    read_ahead_queue_size = LaunchConfiguration("read_ahead_queue_size")
    field_side = LaunchConfiguration("field_side")
    publish_low_confidence = LaunchConfiguration("publish_low_confidence")
    publish_ramp_template = LaunchConfiguration("publish_ramp_template")
    publish_fence_top_marker = LaunchConfiguration("publish_fence_top_marker")
    entry_forward_source = LaunchConfiguration("entry_forward_source")
    node_start_delay = LaunchConfiguration("node_start_delay")
    bag_keyboard = LaunchConfiguration("bag_keyboard")
    publish_field_model = LaunchConfiguration("publish_field_model")
    enable_zone3_ransac_refine = LaunchConfiguration("enable_zone3_ransac_refine")
    zone3_ransac_radius = LaunchConfiguration("zone3_ransac_radius")
    zone3_ransac_dist_thr = LaunchConfiguration("zone3_ransac_dist_thr")
    zone3_ransac_min_inliers = LaunchConfiguration("zone3_ransac_min_inliers")

    args = [
        DeclareLaunchArgument(
            "bag_path",
            default_value="/mnt/c/Users/22240/rc2026_snapshot/bag/cha1nav2_20260523_133237/",
            #cha1nav2_20260523_133237
            #cha1nav2_20260612_154209/
            description="rosbag2 目录路径",
        ),
        DeclareLaunchArgument(
            "bag_rate",
            default_value="1.0",
            description="回放速率",
        ),
        DeclareLaunchArgument(
            "read_ahead_queue_size",
            default_value="1000",
            description="rosbag2 预读队列大小，用于减少 Message queue starved",
        ),
        DeclareLaunchArgument(
            "field_side",
            default_value="blue",
            description="场地侧选择：blue 固定蓝区，red 固定红区，auto 根据点云自动投票",
        ),
        DeclareLaunchArgument(
            "publish_low_confidence",
            default_value="true",
            description="置信度较低时也发布粗略模型，便于先看效果",
        ),
        DeclareLaunchArgument(
            "publish_ramp_template",
            default_value="false",
            description="是否发布绿色坡道宽度模板",
        ),
        DeclareLaunchArgument(
            "publish_fence_top_marker",
            default_value="false",
            description="是否发布围栏顶部 +10cm 辅助线",
        ),
        DeclareLaunchArgument(
            "entry_forward_source",
            default_value="odom_start",
            description="坡脚 forward 来源：odom_start 使用 flat_to_uphill 起点；odom_end 使用 platform 终点减坡长",
        ),
        DeclareLaunchArgument(
            "node_start_delay",
            default_value="2.0",
            description="bag 开始后再启动分析节点的延迟秒数，避免状态机在无 bag 数据时采集基准",
        ),
        DeclareLaunchArgument(
            "bag_keyboard",
            default_value="true",
            description="是否保留 rosbag 键盘控制；true 时空格可暂停，false 时禁用键盘控制",
        ),
        DeclareLaunchArgument(
            "publish_field_model",
            default_value="true",
            description="是否启动 rc26_field.py 发布 /arena/field_markers 场地模型",
        ),
        DeclareLaunchArgument(
            "enable_zone3_ransac_refine",
            default_value="true",
            description="是否用 RANSAC 对第三区角点两条边做二次精修",
        ),
        DeclareLaunchArgument(
            "zone3_ransac_radius",
            default_value="0.85",
            description="第三区角点 RANSAC 精修半径，单位 m",
        ),
        DeclareLaunchArgument(
            "zone3_ransac_dist_thr",
            default_value="0.045",
            description="第三区角点 RANSAC 内点距离阈值，单位 m",
        ),
        DeclareLaunchArgument(
            "zone3_ransac_min_inliers",
            default_value="35",
            description="第三区角点 RANSAC 单条边最少内点数",
        ),
    ]

    bag_play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "-r",
            bag_rate,
            "--read-ahead-queue-size",
            read_ahead_queue_size,
            "--clock",
            "100",
        ],
        output="log",
        sigterm_timeout="2.0",
        sigkill_timeout="2.0",
        condition=IfCondition(bag_keyboard),
    )

    bag_play_no_keyboard = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "-r",
            bag_rate,
            "--read-ahead-queue-size",
            read_ahead_queue_size,
            "--clock",
            "100",
            "--disable-keyboard-controls",
        ],
        output="log",
        sigterm_timeout="2.0",
        sigkill_timeout="2.0",
        condition=UnlessCondition(bag_keyboard),
    )

    uphill = Node(
        package="uphill",
        executable="uphill_state_node",
        name="uphill_state_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        sigterm_timeout="2.0",
        sigkill_timeout="2.0",
    )

    fence = Node(
        package="fence_locator",
        executable="fence_locator",
        name="fence_locator",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "field_side": field_side,
                "publish_low_confidence": publish_low_confidence,
                "publish_ramp_template": publish_ramp_template,
                "publish_legacy_ramp_markers": False,
                "publish_zone3_debug_markers": True,
                "publish_zone3_root_tf": True,
                "publish_zone3_field_marker": True,
                "publish_fence_top_marker": publish_fence_top_marker,
                "entry_forward_source": entry_forward_source,
                "use_odom_pca_yaw": False,
                "update_ramp_yaw_from_odom": False,
                "apply_cloud_ransac_yaw": False,
                "cloud_ransac_min_edge_points": 6,
                "cloud_ransac_min_inliers": 5,
                "apply_lateral_detection_to_model": False,
                "enable_zone3_ransac_refine": enable_zone3_ransac_refine,
                "zone3_ransac_radius_m": zone3_ransac_radius,
                "zone3_ransac_dist_thr_m": zone3_ransac_dist_thr,
                "zone3_ransac_min_inliers": zone3_ransac_min_inliers,
            }
        ],
        sigterm_timeout="2.0",
        sigkill_timeout="2.0",
    )

    field_model = ExecuteProcess(
        cmd=[
            "/usr/bin/python3",
            "/mnt/c/Users/22240/rc2026_snapshot/zone_detection/zone_detection/field_publisher/rc26_field.py",
        ],
        output="log",
        sigterm_timeout="2.0",
        sigkill_timeout="2.0",
        condition=IfCondition(publish_field_model),
    )

    analysis_nodes = TimerAction(
        period=node_start_delay,
        actions=[uphill, fence, field_model],
    )

    shutdown_log = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(
                    msg="检测到 launch 退出信号，停止 marker 发布并关闭 bag/uphill/fence_locator"
                ),
            ]
        )
    )

    return LaunchDescription(args + [bag_play, bag_play_no_keyboard, analysis_nodes, shutdown_log])
