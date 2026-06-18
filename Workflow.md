# 第三区九宫格定位 Workflow

## 2026-06-13 Ramp odom extraction for slope X-axis lock

Current side task:
- Use `ramp_drive_yaw_hold.py` as the reference behavior.
- Read `/odin1/odometry_highfreq` from `bag/cha1nav2_20260612_154209/`.
- Extract odom rows during the two main pitch angle changes while climbing.
- Rotate those odom poses into a temporary ramp-local coordinate frame.

Why this matters for the larger Zone3 workflow:
- Before finding the Zone3 grid, the robot has to climb through the ramp/platform transition.
- If the ramp/platform X axis can be locked from odom, later point-cloud filtering can use a cleaner local coordinate frame instead of only global odom boxes.
- This is separate from final grid-center TF publishing, but it can become a useful prior for "where the robot is after climbing" and for cropping slope/platform-related map points.

Generated offline data:
- `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\summary.json`
- `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\angle_change_1.csv`
- `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\angle_change_2.csv`
- `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_odom_axis_locked.csv`

Current extracted anchors:
- `angle_change_1`: `19.7627s ~ 20.8030s`, `ramp_x 0.000m ~ 0.235m`.
- `angle_change_2`: `26.7200s ~ 27.4728s`, `ramp_x 1.462m ~ 1.680m`.

Interpretation:
- First interval is likely the robot entering the ramp and pitch changing onto the slope.
- Second interval is likely the robot leaving the ramp and becoming flatter on the upper platform.
- A short later candidate around `29.7s` exists, but is not treated as one of the two main climb angle changes for now.

Next useful step:
- Use `ramp_odom_axis_locked.csv` to define ramp/platform crop windows.
- Then extract `/odin1/cloud_slam` frames in the same time window and convert points into the ramp-local frame for slope/platform map fitting.

## 2026-06-13 Odom validation workflow

Purpose:
- Use odom to verify which bag segment is truly the Zone3 ramp climb.
- This is a safety check before using ramp/platform geometry as a crop or map prior.
- It does not replace final Zone3 grid localization.

Inputs:
- Odom topic: `/odin1/odometry_highfreq`
- Reference control script: `ramp_drive_yaw_hold.py`
- Field geometry source:
  - `主赛图册V3.pdf`, page 4
  - `zone_detection/zone_detection/field_publisher/rc26_field.py`

Known ramp geometry:
```text
ramp horizontal length: 1.50m
ramp z low: 0.05m
ramp z high: 0.45m
z rise: 0.40m
expected pitch: atan(0.40 / 1.50) ~= 14.93deg
platform top: about 0.45m
```

Validation idea:
```text
Do not assume there is only one ramp event.

For the whole odom bag:
  1. Estimate baseline z/pitch/yaw before climb.
  2. Search every candidate where:
       pitch leaves baseline
       z rises
       pitch returns near baseline
       z stays high
  3. Score candidates by:
       pitch close to 15deg
       z rise close to 0.40~0.45m
       ramp-axis distance close to 1.50m
       yaw relatively stable
  4. Pick the best candidate as the real ramp climb.
```

Current extracted candidate in `cha1nav2_20260612_154209`:
```text
angle_change_1:
  time: 19.7627s ~ 20.8030s
  meaning: entering ramp / pitch leaves baseline
  ramp_x: 0.000m ~ 0.235m

angle_change_2:
  time: 26.7200s ~ 27.4728s
  meaning: reaching upper platform / pitch returns toward baseline
  ramp_x: 1.462m ~ 1.680m
```

Current interpretation:
- The event matches the field geometry reasonably well:
  - pitch change is near the expected ramp angle.
  - z rise approaches the expected `0.40m~0.45m` platform height.
  - ramp-axis distance between entering and leveling is close to `1.50m`.
- There is also a short later pitch-change candidate around `29.7s`, but it does not look like the main ramp climb.

Implementation note:
- Current extraction script:
  - `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\work\extract_ramp_odom_axis.py`
- Current output directory:
  - `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\`

Next improvement:
- Update the script from "extract the main two pitch changes" to "global candidate search + geometry score table".
- Output a CSV containing all candidates, not only the chosen one.
本文档记录当前用户的工作流程和思考路径，供后续自己、其他 AI 或队友继续推进时参考。

## 总目标

当前不是完整比赛全流程开发，而是优先解决对抗赛第三区能力：

> 机器人上到第三区后，能够稳定识别九宫格，并稳定发布九宫格地盘/中心相关 TF。

当前默认设备条件：

- 主要使用 `odin1`。
- 主要点云来源：`/odin1/cloud_slam`。
- 当前 bag 中该点云已在 `odom` frame。

## 比赛流程理解

根据规则和场地图，目前关注的流程是：

1. 机器人从一区进入二区，完成梅林相关任务。
2. 对抗阶段进入第三区。
3. 第三区是对抗区，深度约 2700mm。
4. 对抗区包含：
   - 坡道
   - 九宫格
   - 重试区
   - 已用兵器区
5. 对抗区中一般是 R1 先上，R2 再上。
6. R1/R2 上三区后会围绕九宫格放置、移除、防守 KFS。
7. 因此九宫格附近可能存在遮挡：
   - R1
   - R2
   - KFS
   - 兵器
   - 对方动作或局部干扰

关键结论：

- 不应假设九宫格完整可见。
- 不应假设机器人在第三区内的位置固定。
- 不应假设机器人转向角度固定。
- 比较稳定的条件是：机器人上三区后，会主动转向九宫格方向。

## 当前核心思路

与其在全场范围中找九宫格，不如利用比赛动作：

> 机器人转向九宫格后，以车头方向为中心，动态裁出一条局部观察区域。

这个局部观察区域不是普通扇形，而是等宽走廊。

## ROI 策略

### 不推荐：单纯扇形 ROI

扇形 ROI 形式：

```text
r_min < r < r_max
theta_min < theta < theta_max
```

问题：

- 越远越宽。
- 容易把道路外、九宫格外、其它场地结构也包含进来。
- 对第三区这种固定宽度通道/目标不够友好。

### 推荐：等宽走廊 ROI

等宽走廊 ROI 形式：

```text
forward = 点沿车头方向的距离
lateral = 点到车头中心线的左右偏移

keep if:
  near <= forward <= far
  abs(lateral) <= corridor_half_width
  h_min <= height <= h_max
```

直观含义：

- 只看车前方一定距离。
- 只看车头中心线左右固定宽度。
- 不随距离扩散。

推荐初始参数：

```text
forward near: 0.3m
forward far: 4.0m
corridor half width: 1.2m ~ 1.6m
height: 0.4m ~ 2.8m
```

后续根据 bag 和实测逐步收紧。

## 高度理解

点云中的 `z` 不一定直接等于真实离地高度。代码里要统一到 `odom` 的竖直方向后计算：

```text
h = z_odom - ground_z
```

当前 bag 的 `/odin1/cloud_slam` 已经是 `odom` frame，所以可近似理解为：

```text
h = z - ground_z
```

当前配置中：

```text
ground_z = -0.1500
```

九宫格相关高度可先按：

```text
候选高结构：0.75m ~ 2.60m
更宽观察：0.4m ~ 2.8m
```

## 当前算法问题

原 `zone3_localizer` 流程主要是：

1. 大 ROI 裁剪。
2. 高度筛选。
3. 多帧累积。
4. 连通域。
5. PCA 估计方向。
6. width/depth/layer/density 打分。
7. 稳定后发布 TF。

问题：

- 大 ROI 会混入太多结构。
- PCA 对遮挡和局部缺失敏感。
- R1/KFS/兵器遮挡时，九宫格不完整，PCA 可能偏。
- 只看整体连通域，不够利用九宫格固定几何。

## 推荐检测路线

建议按以下顺序推进：

### Step 1：离线验证走廊 ROI

先不做识别，只做可视化验证：

```text
bag 点云
  -> 车体系/局部方向变换
  -> 等宽走廊 ROI
  -> 高度筛选
  -> 看九宫格是否保留、杂点是否减少
```

目标：

- 确认走廊 ROI 不会把九宫格切掉。
- 确认走廊 ROI 比大 ROI 更干净。
- 调出初始 `near/far/half_width/yaw_offset/height` 参数。

### Step 2：增加可调参数

在离线查看器中加入：

- 走廊宽度
- forward near/far
- yaw offset
- height min/max

用 bag 拖参数观察。

目标：

- 估计机器人朝向误差允许范围。
- 找到“九宫格仍然完整保留但杂点较少”的参数。

### Step 3：走廊内做高度分层

把保留点按高度染色：

```text
0.80 ~ 1.34m  第一层
1.34 ~ 1.88m  第二层
1.88 ~ 2.50m  第三层
```

目标：

- 判断九宫格三层是否能从点云中分开。
- 检查 `ground_z` 是否偏。

### Step 4：模型拟合替代单纯 PCA

不要只看“最大连通域 + PCA”。

建议建立九宫格模型：

- 3 列
- 3 层
- 总宽约 1.62m
- 深度约 0.32m
- 高度层固定

在走廊 ROI 内搜索或拟合：

```text
候选参数：
  x
  y
  yaw

评分：
  九格内点数
  三层覆盖
  左右宽度
  深度
  外点惩罚
  与上一帧偏差
```

目标：

- 即使九宫格被部分遮挡，也能用模型补全中心。

### Step 5：TF 状态机

TF 发布不能每帧抖动。

建议状态机：

```text
UNLOCKED
  -> 候选出现
  -> 连续稳定
PRIOR_LOCKED / LOCKED
  -> 小范围精修
  -> 稳定发布
LOST
  -> 短时检测失败保留上次 TF
```

建议原则：

- 未锁定时可以较大范围搜索。
- 锁定后只允许小范围修正。
- 检测失败若只是短时遮挡，不立刻丢 TF。
- 发布 TF 需低通滤波或 EMA。

## TF 定义待确认

后续必须明确“九宫格地盘正中心 TF”到底指什么：

1. 九宫格几何中心投影到地面的点。
2. 九宫格架子底座中心。
3. 当前代码里的 `zone3_root`。
4. 机器人放置/对齐时最方便使用的操作坐标原点。

建议后续将 TF 拆开命名：

```text
odom -> zone3_root
odom -> zone3_grid_center
zone3_grid_center -> zone3_cell_{row}_{col}
```

这样上层控制更容易使用。

## 当前临时工具

已有一个离线 HTML 查看器：

```text
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\zone3_filter_viewer.html
```

用途：

- 从 bag 抽样 `/odin1/cloud_slam`。
- 查看原始点、ROI、高度分层、高位候选和当前检测。

注意：

- 这是临时调试工具。
- 后续应继续扩展“走廊 ROI 可视化”和参数滑条。

### 2026-06-13 工具扩展：深度切片模式

已基于离线 HTML 查看器增加：

```text
6 深度切片
```

当前实现：

- 同步读取 bag 中 `/odin1/odometry_highfreq`。
- 对每帧 `/odin1/cloud_slam`，使用最近的机器人 odom 位姿。
- 将点云按机器人当前 yaw 转成车体朝向下的距离。
- 保留车头前方：

```text
forward: 4.5m ~ 5.0m
```

当前尚未加入左右走廊限制，因此该模式只是“深度薄片”，不是最终“等宽走廊”：

```text
已做：near <= forward <= far
未做：abs(lateral) <= corridor_half_width
```

下一步可在此基础上增加：

- `corridor_half_width`
- `yaw_offset`
- `forward_near/forward_far` 滑条
- `height_min/height_max` 滑条

## 下一步最小可执行任务

优先不要直接改线上节点。

最小下一步：

1. 在离线查看器中增加“等宽走廊 ROI”模式。
2. 加入参数：
   - `forward_near`
   - `forward_far`
   - `corridor_half_width`
   - `yaw_offset`
   - `height_min`
   - `height_max`
3. 用 bag 验证不同帧下九宫格是否稳定保留。
4. 确认走廊 ROI 有效后，再设计模型拟合。

## 一句话总结

当前 workflow 是：

```text
不要在全场找九宫格。
利用“车上三区后会转向九宫格”这一动作，
用车头方向切一条等宽走廊，
再用高度和九宫格固定模型做局部识别，
最后用状态机稳定发布 TF。
```
