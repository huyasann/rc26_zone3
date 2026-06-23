# fence_locator

ROBOCON 2026 第三区定位功能包。当前目标是用 `odin1` 点云和 odom，在蓝区第三区内识别关键围栏角点，发布可用于对齐场地模型和九宫格的 TF / Marker。

## 当前定位思路

核心不再优先依赖坡道整体拟合，而是优先找第三区入口附近的关键角点：

```text
点云采集
-> 状态机判断进入第三区阶段
-> 点云筛选
-> 找两条近似垂直的围栏边/竖直边
-> 求交点得到第三区关键角点
-> 结合公开场地图尺寸先验
-> 发布 blue_zone3_root
-> 场地模型跟随 blue_zone3_root 对齐
```

关键约束：

- 当前默认锁定蓝区，红蓝自动判断暂不作为主路线。
- 点云有效边通常应在场地内部或围栏内侧，不能让行人、墙面、围栏外噪点主导拟合。
- 两条边应接近正交，不能让两条不垂直的线直接决定场地 yaw。
- 九宫格模型最终跟随 `blue_zone3_root`，不是单独乱漂。

## 主要节点

### `fence_locator`

主定位节点。

输入：

- `/odin1/cloud_slam`
- `/odin1/odometry_highfreq`
- `/uphill/state`
- `/uphill/debug`

输出：

- `/ramp/model_marker`
- `/uphill/endpoint_marker`
- `/arena/field_markers`，由场地模型发布器使用时显示
- `blue_zone3_root` TF，作为第三区模型根坐标

主要职责：

- 监听上坡状态机。
- 缓存并筛选点云。
- 检测第三区关键角点。
- 发布第三区根 TF。
- 发布调试 marker。

### `pointcloud_hold`

点云保持节点。

用途：

- bag 暂停后，RViz 点云不立刻消失。
- 输入 `/odin1/cloud_slam`。
- 输出 `/odin1/cloud_slam_hold`。

### `zone3_tf_tuner`

手动校准 Qt 工具。

用途：

- 读取自动生成的 `blue_zone3_root_auto`。
- 手动调 `X/Y/Z/Yaw`。
- 发布校准后的 `blue_zone3_root`。
- 用于把自动拟合结果和人工校准结果对照，反推参数。

## 启动

在 ROS2 工作空间内：

```bash
cd /mnt/c/Users/22240/rc2026_snapshot/rc
so
ros2 launch fence_locator launch_fence.launch.py
```

常用参数：

```bash
ros2 launch fence_locator launch_fence.launch.py bag_path:=/mnt/c/Users/22240/rc2026_snapshot/bag/cha1nav2_20260523_133237
```

```bash
ros2 launch fence_locator launch_fence.launch.py field_side:=blue
```

当前建议：

- `field_side:=blue`
- `hold_pointcloud:=true`
- `publish_field_model:=true`
- `launch_rviz:=true`
- `launch_tuner:=true`

## RViz 显示

建议 Fixed Frame：

```text
odom
```

建议添加：

- PointCloud2：`/odin1/cloud_slam_hold`
- Marker：`/ramp/model_marker`
- Marker：`/uphill/endpoint_marker`
- MarkerArray：`/arena/field_markers`
- TF

如果看到旧 marker 残留：

```bash
ros2 topic pub --once /ramp/model_marker visualization_msgs/msg/Marker "{header: {frame_id: odom}, ns: clear, id: 0, action: 3}"
```

更稳的办法是关闭 RViz 后重新打开，避免 latch/历史显示干扰判断。

## 关键参数

主节点参数集中在：

```text
fence_locator/fence_locator_node.py
```

常用参数：

- `field_side`
- `publish_low_confidence`
- `enable_cloud_ransac_line`
- `apply_cloud_ransac_yaw`
- `apply_cloud_ransac_lateral_offset`
- `enable_zone3_ransac_refine`
- `zone3_ransac_radius_m`
- `zone3_ransac_dist_thr_m`
- `zone3_ransac_min_inliers`
- `zone3_model_key_x_m`
- `zone3_model_key_y_m`
- `zone3_model_key_z_m`
- `zone3_root_calib_forward_m`
- `zone3_root_calib_lateral_m`
- `zone3_root_calib_z_m`
- `zone3_root_calib_yaw_deg`

手动校准参考值曾经有效的一组：

```text
X   9.8665 m
Y   1.8655 m
Z  -0.2648 m
Yaw 92.50 deg
```

这组值只能作为调试参考，不应直接视为最终比赛固定值。

## 角点检测模块

关键文件：

```text
fence_locator/zone3_corner_detector.py
```

主要流程：

```text
点云转局部坐标
-> 过滤第三区附近 ROI
-> 估计平台/地面层
-> 提取外轮廓边
-> 提取端部边
-> RANSAC/分 bin 拟合两条边
-> 求交点
-> 用竖直点列和场地内约束修正
```

当前重点不是坡道线，而是第三区蓝色区域、围栏和关键角点的几何关系。

## 已知问题

1. 行人和墙面会给 RANSAC 增加外部噪点。

   处理方向：

   - 只保留场地附近 ROI。
   - 优先使用场地内部点云。
   - 对围栏外侧点云降权。
   - 两条边必须满足正交约束。

2. 自动拟合可能存在整体平移。

   处理方向：

   - 使用 `zone3_tf_tuner` 手动校准对照。
   - 反推自动检测角点到人工角点的偏差。
   - 不要用单帧点云直接决定最终 TF。

3. 旧 marker 可能残留。

   处理方向：

   - launch 退出时发布 DELETEALL。
   - RViz 仍残留时手动清空或重启 RViz。

4. 坡道状态机曾经影响采集时机。

   当前建议：

   - 上坡中可以开始缓存点云。
   - 真正用于第三区角点精拟合的数据，应优先取上平台后的稳定点云。
   - 坡道围栏可以作为辅助，但不应主导第三区场地 TF。

## 关于梅林策略回头路的说明

梅林策略里出现 `A -> B -> A` 回头路的根因是局部可见：

```text
当前只知道局部状态
-> 先去 B 获取信息或拿 KFS
-> B 处理完后状态改变
-> 新信息显示 A 方向才是后续收益更高
-> 于是回到 A
```

这不是单纯最短路问题，而是信息不完整导致的在线决策问题。后续应使用规则反推和短程 rollout，减少不必要的回头路。

## Git 注意

提交前先看状态：

```bash
git status --short
```

不要误提交：

- bag
- PDF
- 临时截图
- 大型压缩包
- 其他 AI 生成但未确认的实验目录

