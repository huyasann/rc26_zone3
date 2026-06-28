# zone3_grid_fusion

独立的第三区九宫格辅助融合功能包。

## 作用

该包不替代 `fence_locator` 的边角定位，只读取它已经发布的角点场地根 TF：

- 输入 TF：`odom -> blue_zone3_root_auto`
- 输入点云：`/odin1/cloud_slam`
- 输入状态：`/uphill/state`

然后在角点根坐标系附近裁剪九宫格高层点云，做连通域 + PCA，得到九宫格中心和朝向偏差，对角点结果做小范围修正。

## 输出

- TF：`odom -> blue_zone3_root_grid`
- 位姿：`/zone3/grid_fusion/root_pose`
- 调试结果：`/zone3/grid_fusion/result`
- 调试 marker：`/zone3/grid_fusion/markers`

默认 launch 会让 Qt 手动校准窗口读取 `blue_zone3_root_grid`，再发布最终 `blue_zone3_root`，场地模型继续跟随最终 `blue_zone3_root`。

## 一键启动

```bash
cd /mnt/c/Users/22240/rc2026_snapshot/rc
so
ros2 launch fence_locator launch_fence.launch.py bag_path:=/mnt/c/Users/22240/rc2026_snapshot/bag/c2_20260626_151444
```

## 关闭九宫格辅助

```bash
ros2 launch fence_locator launch_fence.launch.py launch_grid_fusion:=false tuner_source_frame:=blue_zone3_root_auto
```

## 当前策略

- 蓝区写死。
- 九宫格检测只做辅助微调，不允许大幅拖动角点定位。
- 当前默认最大平移修正 `0.22m`，最大角度修正 `2.5deg`。
- 低分九宫格结果丢弃，高分结果短时保持，减少点云断帧导致的 TF 抖动。
- 可用 `save_grid_fusion_log:=true` 保存逐帧 JSONL，默认路径为 `/mnt/c/Users/22240/rc2026_snapshot/outputs/zone3_grid_fusion_debug.jsonl`。
- 2026-06-26 调试结论：原九宫格候选过宽时会误选大块点云，已加入宽度/深度/中心/yaw 硬约束，并将高度筛选改为先在九宫格 ROI 内估底面再分层。
- yaw 修正使用半圈等价角；例如 `-178deg` 会按接近 `+2deg` 处理，不再被错误裁成反向修正。
