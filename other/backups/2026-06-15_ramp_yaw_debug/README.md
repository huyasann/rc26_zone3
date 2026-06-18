# 2026-06-15 坡道围栏调试备份

## 目录说明

```text
current_source/
  保存 2026-06-15 晚上停止前的当前源码版本。
  注意：这是源码备份，不代表已经完整 ros2 launch 回放验证。

fundamental_issue_reconstructed/
  保存“发现本质问题”那一刻的思路版本说明。
  因为当前目录没有可用 git 历史，无法精确恢复聊天中某一秒的源码状态，所以这里记录可回退的参数和逻辑边界。
```

## 当前源码状态

```text
uphill_state_node:
  起坡触发被提前：
    pitch_start_deg = 1.20
    z_start_m = 0.015
    start_hold_s = 0.04
  稳定上坡判断仍保留：
    pitch_stable_deg = 12.69
    z_contact_m = 0.06

fence_locator:
  不再让 odom PCA 决定最终 yaw：
    use_odom_pca_yaw = false
    update_ramp_yaw_from_odom = false
  ramp_start_pose 改为优先使用 /uphill/transition_pose。
  点云 RANSAC yaw 允许接管，但需要通过质量门槛。
  marker Z 引入 marker_ground_z_locked，目标是减少上下跳。
```

## “发现本质问题”版本边界

```text
核心结论：
  不应该等车体接近 15deg 才记录坡道起点。
  起点应该来自 pitch/z 刚开始明显变化时的 odom 位姿。
  稳定坡面阶段再开始做围栏 lateral 检测是可以的。

当时更稳的部分：
  center_lateral 由稳定上坡阶段点云统计得出。
  低质量 RANSAC 不接管主模型。
  起点提前，但 yaw 仍需要谨慎处理。

后续证明的问题：
  如果直接用早期 odom yaw 作为坡道 yaw，蓝线会偏离边框。
  如果直接用 odom PCA/start-end 作为 yaw，又会受车行驶歪斜影响。
  因此最终 yaw 应该来自点云边线/场地边界拟合，而不是 odom。
```

## 下一步建议

```text
1. 起点位置继续使用早期 transition_pose。
2. 不使用 odom PCA 更新最终 yaw。
3. 点云边线拟合先只作为可视化，确认贴边后再接管主 yaw。
4. marker 高度问题单独处理，不再和起点/yaw 混在一起调。
5. 若 RViz 残留 marker，先清空 /ramp/model_marker 再重新启动。
```
