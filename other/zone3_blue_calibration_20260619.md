# 2026-06-19 第三区蓝区自动拟合校准记录

## 当前结论

- 当前调试阶段写死蓝区，不再启用红蓝自动判断。
- `fence_locator` 自动发布 `odom -> blue_zone3_root_auto`。
- `zone3_tf_tuner` 默认读取 `blue_zone3_root_auto`，再发布 `odom -> blue_zone3_root`。
- `rc26_field.py` 的第三区模型跟随 `blue_zone3_root`。

## 用户手动校准基准

Qt 手动校准后，点云覆盖到拟合场地上，九宫格对齐：

```text
parent_frame = odom
source_frame = blue_zone3_root_auto
output_frame = blue_zone3_root
X = 9.8665 m
Y = 1.8655 m
Z = -0.2648 m
Yaw = 92.50 deg
```

## 本次代码调整

- `launch_fence.launch.py` 中 `field_side` 固定为 `blue`。
- `auto_field_side` 固定为 `False`。
- `Zone3CornerConfig` 增加 `forced_side`。
- 蓝区固定时，角点检测只允许 `positive` 候选边；红区后续如需启用，再切到 `negative`。
- 根据手动校准值反推当前补偿：

```text
zone3_root_calib_forward_m = -0.161
zone3_root_calib_lateral_m = 0.040
zone3_root_calib_z_m = 0.0
zone3_root_calib_yaw_deg = 0.40
```

## 无头验证结果

命令：

```bash
cd /mnt/c/Users/22240/rc2026_snapshot/rc
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 150 ros2 launch fence_locator launch_fence.launch.py \
  launch_rviz:=false \
  launch_tuner:=false \
  hold_pointcloud:=false \
  publish_field_model:=false \
  read_ahead_queue_size:=10000 \
  bag_rate:=0.5
```

关键输出：

```text
zone3 root tf: odom -> blue_zone3_root_auto, xyz=(9.867,1.866,-0.265), yaw=92.50deg
zone3 root fit: ... root_calib=(-0.161,+0.040,+0.000,+0.40deg), ... side=positive
```

该结果与用户手动校准目标基本一致。

## 仍需注意

- 这次对齐依赖当前 bag 和当前检测特征，后续换 bag 仍要看角点是否稳定。
- 当前检测日志里 `angle=41.5deg`、`outer_rmse=0.148`、`outer_inliers=7`，说明外侧边候选质量仍不理想。
- 目前主要靠固定蓝区、强制正交轴和校准补偿把模型拉正；后续若追求泛化，需要继续优化角点两条边的 RANSAC 质量。

## 2026-06-19 场地内点云约束

用户补充的关键约束：

- 雷达点云主要来自场地内部可见区域。
- RANSAC 拟合后的有效边点不应该大量落在围栏外侧。
- 如果拟合出来的第三区模型让大量平台点云跑到围栏外，说明候选姿态应被降分或拒绝。

本次新增约束：

- `fence_locator_node.py` 新增第三区 L 形场地足迹判断。
- 足迹使用 `rc26_field.py` 的蓝区第三区尺寸：
  - 主平台/主地毯近似范围：`x=[-3.0, 1.5]`, `y=[-1.45, 1.15]`
  - 侧平台/扩展地毯近似范围：`x=[1.5, 3.05]`, `y=[-1.45, 1.30]`
- 每次生成 `blue_zone3_root_auto` 后，把平台后采集的点云投影到该 root 局部坐标。
- 只评价场地附近的点，避免远处墙、人群影响评分。
- 统计：
  - `inside`：落在场地足迹内的点
  - `outside`：落在场地附近但超出足迹的点
  - `out_ratio`：场外点比例
- 只有 `out_ratio > 0.08` 或 `outside > 60` 时，才允许小范围搜索修正 root。
- 如果当前姿态已经没有明显场外点，则保持原校准，不强行移动。

新增 launch 参数：

```text
enable_zone3_inside_refine = True
zone3_inside_refine_xy_range_m = 0.08
zone3_inside_refine_xy_step_m = 0.02
zone3_inside_refine_yaw_range_deg = 0.6
zone3_inside_refine_yaw_step_deg = 0.2
zone3_inside_refine_min_outside_ratio = 0.08
zone3_inside_refine_min_outside_points = 60
```

无头验证命令：

```bash
cd /mnt/c/Users/22240/rc2026_snapshot/rc
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 170 ros2 launch fence_locator launch_fence.launch.py \
  launch_rviz:=false \
  launch_tuner:=false \
  hold_pointcloud:=false \
  publish_field_model:=false \
  read_ahead_queue_size:=10000 \
  bag_rate:=0.5 \
  node_start_delay:=0.0
```

关键输出：

```text
zone3 root tf: odom -> blue_zone3_root_auto, xyz=(9.862,1.876,-0.265), yaw=92.44deg
zone3 root fit: ... inside=7716, outside=0, out_ratio=0.00, inside_refine=(+0.000,+0.000,+0.00deg), ... side=positive
```

结论：

- 当前校准姿态已经满足“场地附近点云不应跑到场外”的约束。
- 因为 `outside=0`，局部优化没有继续移动 root。
- 这比上一版直接最大化 inside 分数更稳，避免为了追求 `outside=0` 把已校准好的姿态拉走。
