"""启动 bag 回放、uphill 状态机和 fence_locator 节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    bag_start_offset = LaunchConfiguration("bag_start_offset")
    bag_start_delay = LaunchConfiguration("bag_start_delay")
    bag_rate = LaunchConfiguration("bag_rate")
    read_ahead_queue_size = LaunchConfiguration("read_ahead_queue_size")
    field_side = LaunchConfiguration("field_side")
    publish_low_confidence = LaunchConfiguration("publish_low_confidence")
    publish_ramp_template = LaunchConfiguration("publish_ramp_template")
    publish_fence_top_marker = LaunchConfiguration("publish_fence_top_marker")
    entry_forward_source = LaunchConfiguration("entry_forward_source")
    node_start_delay = LaunchConfiguration("node_start_delay")
    bag_keyboard = LaunchConfiguration("bag_keyboard")
    bag_in_xterm = LaunchConfiguration("bag_in_xterm")
    hold_pointcloud = LaunchConfiguration("hold_pointcloud")
    publish_field_model = LaunchConfiguration("publish_field_model")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_tuner = LaunchConfiguration("launch_tuner")
    enable_zone3_ransac_refine = LaunchConfiguration("enable_zone3_ransac_refine")
    zone3_ransac_radius = LaunchConfiguration("zone3_ransac_radius")
    zone3_ransac_dist_thr = LaunchConfiguration("zone3_ransac_dist_thr")
    zone3_ransac_min_inliers = LaunchConfiguration("zone3_ransac_min_inliers")

    args = [
        DeclareLaunchArgument(
            "bag_path",
            default_value="/mnt/c/Users/22240/rc2026_snapshot/bag/cha1nav2_20260612_154209/",
            #cha1nav2_20260523_133237
            #cha1nav2_20260612_154209/
            #cha1nav2_20260523_201149/
            description="rosbag2 目录路径",
        ),
        DeclareLaunchArgument(
            "bag_rate",
            default_value="1.0",
            description="回放速率",
        ),
        DeclareLaunchArgument(
            "bag_start_offset",
            default_value="14.0",
            description="Seconds to skip from bag start; default starts a few seconds before ramp detection.",
        ),
        DeclareLaunchArgument(
            "bag_start_delay",
            default_value="2.0",
            description="Seconds to wait before starting rosbag so subscribers are ready first.",
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
            default_value="0.0",
            description="bag 开始后再启动分析节点的延迟秒数，避免状态机在无 bag 数据时采集基准",
        ),
        DeclareLaunchArgument(
            "bag_keyboard",
            default_value="false",
            description="是否保留 rosbag 自带键盘控制；默认 false，改由 Qt 调用暂停服务",
        ),
        DeclareLaunchArgument(
            "bag_in_xterm",
            default_value="false",
            description="true 时在独立 xterm 中启动 rosbag；默认 false，避免 bag 数据没喂进状态机",
        ),
        DeclareLaunchArgument(
            "hold_pointcloud",
            default_value="true",
            description="true 时发布 /odin1/cloud_slam_hold，暂停 bag 后 RViz 点云仍保持显示",
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
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="是否随 launch 一起启动 RViz2",
        ),
        DeclareLaunchArgument(
            "launch_tuner",
            default_value="true",
            description="是否随 launch 一起启动 Zone3 TF 手动校准 Qt 窗口",
        ),
    ]

    bag_play = ExecuteProcess(
        cmd=[
            "xterm",
            "-T",
            "BAG 回放窗口：空格暂停/继续",
            "-e",
            "ros2",
            "bag",
            "play",
            bag_path,
            "--start-offset",
            bag_start_offset,
            "-r",
            bag_rate,
            "--read-ahead-queue-size",
            read_ahead_queue_size,
            "--clock",
            "100",
            "--topics",
            "/odin1/odometry_highfreq",
            "/odin1/cloud_slam",
            "/tf",
            "/tf_static",
        ],
        output="log",
        sigterm_timeout="2.0",
        sigkill_timeout="2.0",
        condition=IfCondition(bag_in_xterm),
    )

    bag_play_same_terminal = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--start-offset",
            bag_start_offset,
            "-r",
            bag_rate,
            "--read-ahead-queue-size",
            read_ahead_queue_size,
            "--clock",
            "100",
            "--topics",
            "/odin1/odometry_highfreq",
            "/odin1/cloud_slam",
            "/tf",
            "/tf_static",
        ],
        output="log",
        sigterm_timeout="2.0",
        sigkill_timeout="2.0",
        condition=IfCondition(PythonExpression(["'", bag_in_xterm, "' == 'false' and '", bag_keyboard, "' == 'true'"])),
    )

    bag_play_no_keyboard = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--start-offset",
            bag_start_offset,
            "-r",
            bag_rate,
            "--read-ahead-queue-size",
            read_ahead_queue_size,
            "--clock",
            "100",
            "--disable-keyboard-controls",
            "--topics",
            "/odin1/odometry_highfreq",
            "/odin1/cloud_slam",
            "/tf",
            "/tf_static",
        ],
        output="log",
        sigterm_timeout="2.0",
        sigkill_timeout="2.0",
        condition=IfCondition(PythonExpression(["'", bag_in_xterm, "' == 'false' and '", bag_keyboard, "' == 'false'"])),
    )

    uphill = Node(
        package="uphill",
        executable="uphill_state_node",
        name="uphill_state_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "pitch_platform_deg": 6.0,
                "hold_s": 0.20,
            }
        ],
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
                "field_side": "blue",
                "auto_field_side": False,
                "publish_low_confidence": publish_low_confidence,
                "publish_ramp_template": publish_ramp_template,
                "publish_legacy_ramp_markers": False,
                "publish_zone3_debug_markers": True,
                "publish_zone3_root_tf": True,
                "zone3_root_frame": "blue_zone3_root_auto",
                "publish_zone3_field_marker": True,
                "publish_fence_top_marker": publish_fence_top_marker,
                "cloud_ransac_max_points": 180000,
                "cloud_ransac_keep_first_frames": 10,
                "zone3_post_platform_cloud_frames": 24,
                "zone3_post_platform_skip_frames": 0,
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
                # 2026-06-19: 当前固定蓝区后，按 Qt 手动校准值反推的新补偿。
                # 目标约 (9.8665, 1.8655, -0.2648, 92.50deg)。
                "zone3_root_calib_forward_m": 0.009,
                "zone3_root_calib_lateral_m": 0.016,
                "zone3_root_calib_z_m": 0.0,
                "zone3_root_calib_yaw_deg": 0.66,
                "enable_zone3_inside_refine": True,
                "zone3_inside_refine_xy_range_m": 0.08,
                "zone3_inside_refine_xy_step_m": 0.02,
                "zone3_inside_refine_yaw_range_deg": 0.6,
                "zone3_inside_refine_yaw_step_deg": 0.2,
                "zone3_inside_refine_min_outside_ratio": 0.08,
                "zone3_inside_refine_min_outside_points": 60,
            }
        ],
        sigterm_timeout="2.0",
        sigkill_timeout="2.0",
    )

    cloud_hold = Node(
        package="fence_locator",
        executable="pointcloud_hold",
        name="pointcloud_hold",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "input_topic": "/odin1/cloud_slam",
                "output_topic": "/odin1/cloud_slam_hold",
                "publish_hz": 5.0,
                "restamp": False,
            }
        ],
        condition=IfCondition(hold_pointcloud),
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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_rviz),
        sigterm_timeout="3.0",
        sigkill_timeout="3.0",
    )

    tuner = Node(
        package="fence_locator",
        executable="zone3_tf_tuner",
        name="zone3_tf_tuner",
        output="log",
        parameters=[
            {
                "parent_frame": "odom",
                "source_frame": "blue_zone3_root_auto",
                "output_frame": "blue_zone3_root",
            }
        ],
        condition=IfCondition(launch_tuner),
        sigterm_timeout="3.0",
        sigkill_timeout="3.0",
    )

    analysis_nodes = TimerAction(
        period=node_start_delay,
        actions=[uphill, fence, cloud_hold, field_model, rviz, tuner],
    )
    bag_nodes = TimerAction(
        period=bag_start_delay,
        actions=[bag_play, bag_play_same_terminal, bag_play_no_keyboard],
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

    return LaunchDescription(
        args + [analysis_nodes, bag_nodes, shutdown_log]
    )
