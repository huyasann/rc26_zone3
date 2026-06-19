# 平台后采集策略记录

日期：2026-06-18

当前共识：第三方位/三区场地拟合不要在坡上采集点云做最终定位。原因是坡道围栏、坡面边缘、斜坡高度变化都会混入角点拟合，导致场地模型被坡上的结构拉偏。

本次代码调整：

- `flat_to_uphill`：只记录坡起点、清空旧的三区点云缓存，不启动三区点云检测。
- `uphill`：只确认正在上坡，不再做 `detect_fence_single`，不再把坡上点云写入最终角点缓存。
- `uphill_to_platform`：仍然不采集最终定位点云。
- `platform`：状态机确认已经上平台后，清空缓存并开始采集 `zone3_post_platform_cloud_frames` 帧点云。
- 平台后采集结束才调用 `fuse_and_publish()`，此时再做三区角点、两边拟合、场地模型发布。

预期效果：

- 斜坡围栏不再污染三区场地拟合。
- 蓝线/红线角点拟合更偏向平台上的真实围栏和深蓝色关键角。
- 如果平台后可见点云不足，应该明确输出失败原因，而不是用坡上的数据强行拟合。

验证命令：

```bash
cd /mnt/c/Users/22240/rc2026_snapshot/rc
so
ros2 launch fence_locator launch_fence.launch.py
```

需要观察日志顺序：

```text
waiting for platform before zone3 point cloud collection
ramp confirmed, zone3 point cloud collection still waiting for platform
start post-platform point cloud collection
stop post-platform point cloud collection
```

只有 `start post-platform point cloud collection` 之后收到的点云，才应该参与最终场地拟合。

## 2026-06-18 正交轴与 RViz 启动更新

用户反馈：

- 红蓝两条拟合边不是严格 90 度。
- 三区/九宫格模型有时看不到或不跟拟合结果一致。
- `zone3_edge_normal_offset_m=0.04` 额外法向偏移会让模型更偏。
- `launch_fence.launch.py` 需要直接启动 RViz2。

本次处理：

- `zone3_edge_normal_offset_m` 默认值改回 `0.0`，先不做额外法向补偿。
- `_zone3_axes_from_corner()` 改成最终正交轴生成器：
  - RANSAC 仍然分别拟合两条候选边；
  - 根据边的支持度/RMSE/跨度计算可信度；
  - 可信度更高的一条作为主轴；
  - 另一条强制取主轴的 90 度垂线；
  - 再用点云评分选择正负方向。
- 红线、蓝线、黄色三区外框都使用同一套正交轴，不再各自用原始斜率画。
- `launch_fence.launch.py` 增加 `launch_rviz` 参数，默认 `true`，随启动一起打开 RViz2。

验证重点：

- 日志中 `zone3 root fit` 的 `angle` 应接近或等于 `90.0deg`。
- RViz 中红线和蓝线必须互相垂直。
- 黄色三区外框应与红蓝调试线共用同一方向。
- 如果九宫格模型仍不出现，优先检查 RViz 是否添加并启用 `/arena/field_markers` 的 `MarkerArray`，以及 TF 中是否存在 `blue_zone3_root`。

## 2026-06-18 内侧锚点修正

用户指出：雷达点云扫出来的两条线应该在模型围栏里面，不应该定位到模型围栏外面。

本次修正的语义：

- 橙点：原始 RANSAC 两条边交点，表示点云实际扫到的围栏交线位置。
- 红点：用于对齐场地模型的锚点，当前先与橙点同步，不再额外做 5cm 内侧偏移。
- 默认 `zone3_inner_corner_offset_m=0.0`。原因是当前可视化里橙点更贴近点云真实交点，额外 5cm 偏移会引入新的角度/位置误差。
- `blue_zone3_root` 现在由红点反推，不再直接由原始橙点反推。
- 黄色三区外框也从红点开始画，和场地模型锚点保持一致。

验证重点：

- 橙点应该贴近点云扫到的围栏交线。
- 红点应该位于场地内侧角点，不应跑到棕色围栏外面。
- 棕色围栏应该包在红蓝点云线外侧，点云线应位于围栏内侧边附近。

## 2026-06-18 Zone3 TF 手动校准工具

新增 `zone3_tf_tuner`，用于在 Qt 窗口里查看并手动修改拟合场地根坐标的 `X/Y/Z/Yaw`。

当前设计：

- 默认读取 `odom -> blue_zone3_root`，也就是 `fence_locator` 自动拟合出来的三区根坐标。
- 默认发布 `odom -> blue_zone3_root_manual`，避免和 `fence_locator` 同时发布 `blue_zone3_root` 造成 TF 冲突。
- Qt 窗口显示 `X/Y/Z/Yaw` 数值，可以直接输入修改。
- 勾选“跟随检测 TF”时，数值会持续跟随自动拟合结果。
- 取消“跟随检测 TF”后，数值保持当前帧，可以手动微调。
- 勾选“发布校准 TF”时，会持续发布手动校准后的 TF。
- 同时发布 `/zone3_tf_tuner/axes`，用于在 RViz 中看校准轴线。

启动方式：

```bash
cd /mnt/c/Users/22240/rc2026_snapshot/rc
so
ros2 run fence_locator zone3_tf_tuner
```

如果 WSL 缺少 Qt：

```bash
sudo apt install python3-pyqt5
```

RViz 观察方式：

- 打开 `TF` 显示，查看 `blue_zone3_root_manual`。
- 或添加 `Marker`，话题选择 `/zone3_tf_tuner/axes`。

注意：

- 这个工具目前只负责人工校准 TF，不会自动让 `rc26_field.py` 改用 `blue_zone3_root_manual`。
- 如果要让场地模型跟随手动校准结果，下一步需要给场地发布脚本增加“使用手动 root frame”的开关，或者临时让工具发布到 `blue_zone3_root` 并关闭 `fence_locator` 的同名 TF 发布。
