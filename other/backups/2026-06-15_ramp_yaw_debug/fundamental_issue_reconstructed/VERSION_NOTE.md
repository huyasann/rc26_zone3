# “发现本质问题”版本说明

## 当时要保留的核心

```text
坡道起点：
  不等 pitch 接近 15deg。
  在 pitch/z 刚开始明显变化时记录 odom 位姿。

围栏检测：
  不在刚起坡的噪声阶段做最终 lateral。
  接近稳定坡面后再累计点云并估计 center_lateral。

最终 yaw：
  不应由 odom PCA/start-end 直接决定。
  odom 只负责给时间窗口和起点位置。
```

## 当时应避免的错误

```text
1. 起点提前后，直接用早期车头 yaw 当坡道 yaw。
   结果：车还没顺坡，marker 角度可能偏。

2. 用 odom PCA 修正 yaw。
   结果：如果车歪着上坡，PCA 会拟合车轨迹，不一定拟合坡道边框。

3. 把低质量云 RANSAC 结果直接接进主模型。
   结果：边缘点少时会把模型拉偏。
```

## 推荐恢复边界

```text
如果明天要回到这个思路版本：
  保留 early transition_pose 作为 ramp_start_pose。
  保留 stable uphill 阶段做 lateral。
  禁止 odom PCA 更新最终 yaw。
  cloud RANSAC 先只画参考线，不通过 RViz 确认前不要接管主模型。
```
