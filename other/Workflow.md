# 第三区九宫格定位工作流

## 2026-06-13 坡道里程计提取与 X 轴锁定

当前支线任务：
- 以 `ramp_drive_yaw_hold.py` 作为参考行为。
- 从 `bag/cha1nav2_20260612_154209/` 读取 `/odin1/odometry_highfreq`。
- 提取上坡过程中两次主要俯仰角变化区间内的里程计行。
- 将这些里程计位姿旋转到临时坡道局部坐标系。

这件事对第三区整体流程的意义：
- 在寻找第三区九宫格前，机器人需要先完成坡道/平台过渡。
- 如果能从里程计锁定坡道/平台 X 轴，后续点云筛选就可以使用更干净的局部坐标系，而不是只依赖全局里程计框。
- 这和最终九宫格中心 TF 发布是两件事，但可作为“机器人上坡后大致在哪里”的先验，也可用于裁剪坡道/平台相关地图点。

已生成的离线数据：
- `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\summary.json`
- `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\angle_change_1.csv`
- `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\angle_change_2.csv`
- `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_odom_axis_locked.csv`

当前提取出的锚点：
- `angle_change_1`: `19.7627s ~ 20.8030s`, `ramp_x 0.000m ~ 0.235m`.
- `angle_change_2`: `26.7200s ~ 27.4728s`, `ramp_x 1.462m ~ 1.680m`.

解释：
- 第一段大概率对应机器人进入坡道，俯仰角开始贴近坡面。
- 第二段大概率对应机器人离开坡面，在上平台逐渐回平。
- `29.7s` 左右还有一个较短候选，但暂时不作为两次主要上坡角度变化之一。

下一步可做：
- 使用 `ramp_odom_axis_locked.csv` 定义坡道/平台裁剪窗口。
- 再提取同一时间窗口内的 `/odin1/cloud_slam` 帧，并将点转换到坡道局部坐标系，用于坡道/平台地图拟合。

## 2026-06-13 里程计验证工作流

目的：
- 使用里程计验证数据包中哪一段真正对应第三区上坡。
- 这是使用坡道/平台几何作为裁剪或地图先验前的风险检查。
- 它不替代最终的第三区九宫格定位。

输入：
- 里程计话题：`/odin1/odometry_highfreq`
- 参考控制脚本：`ramp_drive_yaw_hold.py`
- 场地几何来源：
  - `主赛图册V3.pdf`，第 4 页
  - `zone_detection/zone_detection/field_publisher/rc26_field.py`

已知坡道几何：
```text
坡道水平长度：1.50m
坡道低端高度：0.05m
坡道高端高度：0.45m
高度抬升：0.40m
期望俯仰角：atan(0.40 / 1.50) ~= 14.93deg
平台顶面高度：约 0.45m
```

验证思路：
```text
不预设只有一个上坡事件。

对整段里程计数据包：
  1. 估计上坡前的基准 z、pitch、yaw。
  2. 搜索所有满足以下特征的候选：
       pitch 离开基准值
       z 上升
       pitch 回到基准值附近
       z 保持高位
  3. 按以下指标给候选打分：
       pitch 接近 15deg
       z 抬升接近 0.40~0.45m
       坡道轴向距离接近 1.50m
       yaw 相对稳定
  4. 选择最符合几何约束的候选作为真实上坡过程。
```

当前在 `cha1nav2_20260612_154209` 中提取到的候选：
```text
angle_change_1:
  时间：19.7627s ~ 20.8030s
  含义：进入坡道 / pitch 离开基准值
  ramp_x: 0.000m ~ 0.235m

angle_change_2:
  时间：26.7200s ~ 27.4728s
  含义：到达上平台 / pitch 回到基准值附近
  ramp_x: 1.462m ~ 1.680m
```

当前解释：
- 该事件和场地几何较匹配：
  - pitch 变化接近期望坡角。
  - z 抬升接近期望的 `0.40m~0.45m` 平台高度。
  - 进入坡面到回平之间的坡道轴向距离接近 `1.50m`。
- `29.7s` 左右还有一个较短俯仰角变化候选，但看起来不像主要上坡过程。

实现记录：
- 当前提取脚本：
  - `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\work\extract_ramp_odom_axis.py`
- 当前输出目录：
  - `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\`

下一步改进：
- 将脚本从“提取两次主要俯仰角变化”升级为“全局候选搜索 + 几何评分表”。
- 输出包含所有候选的 CSV，而不是只输出被选中的候选。
本文档记录当前用户的工作流程和思考路径，供后续自己、其他 AI 或队友继续推进时参考。

## 总目标

当前不是完整比赛全流程开发，而是优先解决对抗赛第三区能力：

> 机器人上到第三区后，能够稳定识别九宫格，并稳定发布九宫格地盘/中心相关 TF。

当前默认设备条件：

- 主要使用 `odin1`。
- 主要点云来源：`/odin1/cloud_slam`。
- 当前数据包中该点云已在 `odom` 坐标系。

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

保留条件：
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
前向近端：0.3m
前向远端：4.0m
走廊半宽：1.2m ~ 1.6m
高度范围：0.4m ~ 2.8m
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

### 步骤 1：离线验证走廊 ROI

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

### 步骤 2：增加可调参数

在离线查看器中加入：

- 走廊宽度
- 前向近端/远端
- 航向角偏移
- 高度最小值/最大值

用 bag 拖参数观察。

目标：

- 估计机器人朝向误差允许范围。
- 找到“九宫格仍然完整保留但杂点较少”的参数。

### 步骤 3：走廊内做高度分层

把保留点按高度染色：

```text
0.80 ~ 1.34m  第一层
1.34 ~ 1.88m  第二层
1.88 ~ 2.50m  第三层
```

目标：

- 判断九宫格三层是否能从点云中分开。
- 检查 `ground_z` 是否偏。

### 步骤 4：模型拟合替代单纯 PCA

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

### 步骤 5：TF 状态机

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
前向距离：4.5m ~ 5.0m
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

当前工作流是：

```text
不要在全场找九宫格。
利用“车上三区后会转向九宫格”这一动作，
用车头方向切一条等宽走廊，
再用高度和九宫格固定模型做局部识别，
最后用状态机稳定发布 TF。
```
## 2026-06-13 工作空间整理规则

当前工作空间策略：
- 保持 `C:\Users\22240\rc2026_snapshot` 根目录接近原始项目布局。
- 将参考资料和协作记录放到：

```text
C:\Users\22240\rc2026_snapshot\other
```

`other/` 用于存放：
- PDF 规则文件和图册文件。
- 已保存的场地布局图片。
- 工作流记录。
- AI 交接记录。
- 根据讨论整理出的设计文档。

文档语言规则：
- 协作文档的说明性文字必须使用中文。
- 变量名、文件名、路径、ROS 话题名、代码标识可以保留原文。
- 不修改 Hermes 自己维护的归档文件，除非用户明确要求。

除非用户明确要求，否则不要把新的文档或参考资料放在根目录。

代码和可运行项目文件保留在原本位置：
- `bag/`
- `configs/`
- `launch/`
- `zone_detection/`
- 根目录工具脚本，例如 `ramp_drive_yaw_hold.py`

## 2026-06-13 全局坡道候选验证结果

已新增离线脚本：

```text
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\work\validate_ramp_odom_candidates.py
```

脚本目的：
- 不再只取前两个俯仰角变化。
- 对整段 `/odin1/odometry_highfreq` 全局搜索俯仰角变化事件。
- 将“俯仰角进入坡面”和“俯仰角回平台”配对成坡道候选。
- 用图纸/场地模型约束打分：
  - 期望坡角约 `14.93deg`
  - 期望高度抬升约 `0.40m`
  - 期望坡道轴向长度约 `1.50m`
  - 航向角变化越小越好

当前数据包结果：

```text
数据包：bag/cha1nav2_20260612_154209
话题：/odin1/odometry_highfreq
总里程计行数：44335
总时长：112.714s
全局俯仰角变化事件数：2
通过基础门控的坡道候选数：1
```

唯一通过基础门控的候选：

```text
进入坡道变化：19.7627s ~ 20.8030s
回到平台变化：26.7200s ~ 27.4728s
候选持续时间：7.7101s
坡道轴向长度：1.6804m
高度抬升：0.4498m
最大俯仰角变化：18.0673deg
航向角跨度：10.2339deg
横向漂移跨度：0.2074m
评分：67.20
```

解释：
- 全局搜索没有发现第二个可疑的上坡候选。
- 当前唯一候选和图纸/场地模型基本匹配：
  - 坡道长度误差约 `0.18m`
  - 高度误差约 `0.05m`
  - 俯仰角比理论 `14.93deg` 大约 `3.14deg`
- 因此这段里程计可以作为后续坡道局部坐标和点云裁切的当前可信依据。

输出文件：

```text
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\pitch_change_events.csv
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_candidate_scores.csv
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_candidate_summary.json
```

下一步：
- 以该候选的坡道局部坐标为参考，提取同时间窗口内 `/odin1/cloud_slam`。
- 将点云转到坡道局部坐标系。
- 先验证坡道/平台几何是否能从点云中稳定看出来，再决定是否把它用于第三区九宫格初始裁切。

## 2026-06-13 坡道轨迹和 X 轴锁定可视化

已新增可视化脚本：

```text
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\work\make_ramp_xlock_visualization.py
```

输出文件：

```text
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_trajectory_xlock.html
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_trajectory_xlock.svg
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_trajectory_xlock.png
```

图中四块内容：
- `odom` 平面轨迹：蓝线为整段局部轨迹，红线为由“进入坡道”到“回到平台”的位移方向。
- 坡道局部剖面：横轴为 `ramp_x`，纵轴为高度；黑色虚线为图纸期望 `1.50m / 0.40m`。
- 俯仰角变化：黄色区间标出两次俯仰角变化，黑色虚线为期望坡角约 `14.93deg`。
- 高度抬升：黄色区间对应坡道两端事件，黑色虚线为图纸抬升 `0.40m`。

当前结论：
- 全局搜索得到 `2` 个俯仰角变化事件，能组成 `1` 个坡道候选。
- 该候选从 `19.76s` 进入坡道，到 `27.47s` 回到平台。
- 实测坡道轴向长度约 `1.680m`，实测抬升约 `0.450m`，和图纸期望接近。
- 因此当前可先用这两端 `odom` 位移方向作为坡道局部 `X` 轴，把后续点云投影到 `ramp_x / ramp_y` 后再做第三区裁切和九宫格初定位。

注意：
- 这张图不是九宫格最终识别结果。
- 它只证明当前 bag 中存在一段和坡道几何较匹配的 `odom` 轨迹，可作为后续点云裁切的坐标依据。

## 2026-06-13 坡道 X 轴解释图

已新增解释图脚本：

```text
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\work\make_ramp_x_axis_explain.py
```

输出文件：

```text
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_x_axis_explain.html
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_x_axis_explain.svg
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_x_axis_explain.png
```

图的目的：
- 用更直观的方式说明 `ramp_x` 不是来自场地图固定坐标，而是从当前 bag 的 `odom` 轨迹中算出来。
- 取进入坡道点 `A` 作为局部原点。
- 取回到平台点 `B`，用 `B-A` 的平面方向作为坡道局部 `X` 轴。
- 垂直于该方向的平面轴作为 `ramp_y`。

当前数值：

```text
A = (1.872, -0.021, 0.037)
B = (3.552, -0.013, 0.477)
B-A = (1.680, 0.009, 0.440) m
平面坡道长度约 1.680m
三维位移长度约 1.737m
坡道 X 轴相对 odom yaw 约 0.299deg
```

和 `rc26_field.py` 的关系：
- `rc26_field.py` 中标准坡道沿 `zone3_root` 的局部 `Y`，范围约 `-0.20 -> +1.30`，长度 `1.50m`。
- 离线算法中为了点云裁切，把这个“沿坡道前进方向”记作 `ramp_x`。
- 当前 bag 的 `ramp_x` 估计长度约 `1.680m`，比场地模型坡道长约 `0.18m`，因此它更适合作为方向和局部坐标基准，不宜直接当成精确场地尺寸。

后续用途：
- 点云点 `P` 先减去坡道起点 `A`，再投影到 `ramp_x / ramp_y / ramp_z`。
- `ramp_x` 用来做深度范围裁切。
- `ramp_y` 用来做等宽走廊裁切，避免传统扇形距离裁切越远越发散。

## 2026-06-13 坡道 X 轴锁定策略修正

用户指出：
- 当前阶段先不讨论九宫格定位。
- 目标是先标定坡道 `X` 轴，并把它绑定到全局 `odom` 的 `X` 方向。
- `pitch` 不是瞬时从 `0deg` 变到 `15deg`，也不是到平台后瞬时回到 `0deg`。
- 车体有长度，前轮先上坡、后轮再上坡；前轮先到平台、后轮再到平台，中间存在过渡带。

修正后的理解：
- `pitch/z` 只用于找“哪一段像坡道过程”的时间窗。
- 坡道 `X` 轴不直接由某一个 `pitch` 阈值点决定。
- 坡道 `X` 轴应由 `odom` 平面位移方向和场地先验共同确定。

新增可视化：

```text
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\work\make_ramp_x_lock_decision.py
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_x_lock_decision.html
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_x_lock_decision.png
```

当前 bag 中多种取轴方式对比：

```text
阈值外边界 A→B：yaw 约 0.299deg，平面长度约 1.680m
过渡带中心：yaw 约 2.770deg，平面长度约 1.463m
中间 30%→70%：yaw 约 3.935deg，平面长度约 0.676m
排除两端过渡带：yaw 约 6.981deg，平面长度约 1.235m
PCA 全窗：yaw 约 4.882deg
```

解释：
- 多种取法相差数度，说明不能只凭某个 `pitch` 阈值点锁轴。
- 中段和 PCA 容易受车体横摆、路径弯曲、上坡过渡带影响。
- 当前阈值外边界 `A→B` 的方向最接近全局 `odom +X`。

建议锁定规则：
1. 用 `pitch/z` 全局检测坡道候选时间窗。
2. 取候选的外边界点 `A/B` 计算粗略坡道方向。
3. 如果粗略方向和 `odom +X` 偏差较小，则将坡道 `X` 轴绑定为全局 `odom +X`。
4. 记录粗略方向和全局 `+X` 的 yaw 偏差作为质量指标。
5. 后续点云裁切使用绑定后的全局 `X` 作为 `ramp_x` 方向，而不是每次重新用 PCA 或中段轨迹拟合方向。

注意事项：
- 车身过渡带不能当作精确坡道端点。
- `pitch` 的上升/下降只是事件证据，不是几何端点。
- `z` 的抬升用于辅助确认坡道候选，但当前 `X` 轴锁定以平面 `odom` 为主。
- 如果后续 bag 中 `A→B` 和全局 `+X` 偏差明显变大，需要先判断是起步姿态、odom 漂移、还是实际上坡方向不一致。

## 2026-06-13 加入上坡前升降/姿态调整区分

用户补充：
- 车在真正上坡前可能会先手控升起来一些。
- 因此 bag 中上坡前就可能出现 `pitch` 或高度变化。
- 这类变化不能直接当成坡道几何端点。

处理方式：
- 不对原始数据做滤波删除。
- 保留姿态变化证据，但在事件分类上加入 `odom_x` 连续正向运动门控。
- 区分三类阶段：
  - 上坡前可疑姿态/升降段。
  - 坡面接触过渡段。
  - 稳定坡面段。

新增重跑脚本：

```text
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\work\rerun_ramp_motion_gate.py
```

输出文件：

```text
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_motion_gate_rerun.json
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_motion_gate_samples.csv
```

当前重跑结果：

```text
连续前进开始/上坡前可疑段起点：18.7438s
原始 pitch 变化起点：19.7627s
运动门控后的坡面接触点：20.0559s
稳定坡面段起点：20.4629s
原始回平台变化起点：26.7200s
运动门控后的平台点：27.4728s
```

原始 `pitch` 变化起点之前的变化：

```text
18.7438s -> 19.7627s
odom_x 前进约 0.142m
pitch_delta 变化约 -3.07deg
z 变化约 0.023m
```

解释：
- 这段已有前进、姿态变化和轻微高度变化。
- 它可能混有手控升降、坡脚接触前准备或坡脚过渡。
- 因此不宜作为精确坡道几何端点。

加入运动门控后的取轴对比：

```text
原始 pitch 外边界：yaw 约 0.299deg，平面长度约 1.680m
运动门控接触到平台：yaw 约 0.636deg，平面长度约 1.618m
稳定坡面到回平台：yaw 约 6.073deg，平面长度约 1.313m
绑定全局 odom +X：yaw = 0deg
```

当前建议：
- `pitch/z` 只用于判断坡道候选和阶段，不用于直接决定最终 X 轴。
- 运动门控后的结果说明真正坡面接触点大约在 `20.06s`，比原始 pitch 起点晚约 `0.29s`。
- 但最终坡道 X 轴仍建议绑定全局 `odom +X`。
- 数据拟合得到的 `0.299deg` 或 `0.636deg` 作为质量指标，而不是每次重新改变 X 轴方向。

## 2026-06-13 uphill 状态机节点

新增 ROS2 Python 节点：

```text
C:\Users\22240\rc2026_snapshot\rc\src\uphill\uphill\uphill_state_node.py
```

入口已加入：

```text
C:\Users\22240\rc2026_snapshot\rc\src\uphill\setup.py
```

依赖已加入：

```text
C:\Users\22240\rc2026_snapshot\rc\src\uphill\package.xml
```

节点目标：
- 订阅 `/odin1/odometry_highfreq`。
- 根据 `odom` 中的高度、俯仰角和 `odom_x` 连续正向运动判断当前坡道阶段。
- 发布中文状态名，方便 bag 回放时直接观察。

状态：

```text
平地
平地上坡中
上坡中
坡上上平台
上平台
```

发布话题：

```text
/uphill/state   std_msgs/String
/uphill/debug   std_msgs/String，内容为 JSON 调试量
```

默认关键参数：

```text
odom_topic = /odin1/odometry_highfreq
baseline_frames = 20
velocity_window_s = 0.40
vx_gate_mps = 0.08
pitch_contact_deg = 7.47
pitch_stable_deg = 12.69
pitch_platform_deg = 9.00
z_contact_m = 0.06
z_high_m = 0.36
hold_s = 0.25
ema_alpha = 0.18
```

旧 bag 离线模拟结果：

```text
20.306s  平地 -> 平地上坡中
20.737s  平地上坡中 -> 上坡中
27.296s  上坡中 -> 坡上上平台
27.617s  坡上上平台 -> 上平台
```

使用方式：

```bash
cd /mnt/c/Users/22240/rc2026_snapshot/rc
colcon build --packages-select uphill
source install/setup.bash
ros2 run uphill uphill_state_node
```

bag 回放时另开终端：

```bash
ros2 bag play /mnt/c/Users/22240/rc2026_snapshot/bag/cha1nav2_20260612_154209 --clock
ros2 topic echo /uphill/state
ros2 topic echo /uphill/debug
```

按键重置：
- `uphill_state_node` 运行时，在该节点所在终端按 `R` 或 `r`。
- 节点会清空当前状态、历史速度窗口、EMA、候选保持计时和平地基准。
- 重置后重新采集 `baseline_frames` 帧作为新的平地基准。

安全退出：
- 在节点终端按 `Ctrl+C`。
- 节点会请求关闭 `rclpy`，通知键盘监听线程停止，恢复终端输入模式。
- 随后销毁节点并退出进程，避免终端停留在异常按键模式。

## 2026-06-13 最终交接清单

当前阶段目标：
- 暂时不继续推进九宫格识别。
- 先把第三区坡道过程识别稳定下来。
- 用坡道状态机为后续点云裁切、坡道 X 轴固定和九宫格 ROI 提供可靠阶段信息。

当前推荐策略：
1. 用 `/odin1/odometry_highfreq` 建立初始平地基准。
2. 用 `pitch` 和高度变化判断是否出现坡道姿态特征。
3. 用 `odom_x` 连续正向速度排除原地升降或上坡前姿态调整。
4. 将过程分为：
   - `平地`
   - `平地上坡中`
   - `上坡中`
   - `坡上上平台`
   - `上平台`
5. 坡道 `X` 轴暂时绑定到全局 `odom +X`。
6. 从数据中得到的 yaw 偏差只作为质量指标，不直接改变轴向。

核心原因：
- 车体有长度，前轮和后轮上坡/到平台之间有时间差。
- `pitch` 不会瞬间从平地变到坡角，也不会瞬间回到平台角度。
- bag 中存在上坡前就发生的姿态/高度变化，疑似手控升降或坡脚过渡。
- 因此不能只用单个 `pitch` 阈值点确定坡道端点或坡道 X 轴。

当前已确认的旧 bag 现象：

```text
bag：C:\Users\22240\rc2026_snapshot\bag\cha1nav2_20260612_154209
话题：/odin1/odometry_highfreq

18.7438s：连续前进开始 / 上坡前可疑姿态段起点
19.7627s：原始 pitch 变化起点
20.0559s：运动门控后的坡面接触点
20.4629s：稳定坡面段起点
26.7200s：原始回平台变化起点
27.4728s：运动门控后的平台点
```

上坡前可疑段：

```text
18.7438s -> 19.7627s
odom_x 前进约 0.142m
pitch_delta 变化约 -3.07deg
z 变化约 0.023m
```

取轴对比：

```text
原始 pitch 外边界：yaw 约 0.299deg，平面长度约 1.680m
运动门控接触到平台：yaw 约 0.636deg，平面长度约 1.618m
稳定坡面到回平台：yaw 约 6.073deg，平面长度约 1.313m
最终绑定全局 odom +X：yaw = 0deg
```

新增/修改的 ROS2 包文件：

```text
C:\Users\22240\rc2026_snapshot\rc\src\uphill\uphill\uphill_state_node.py
C:\Users\22240\rc2026_snapshot\rc\src\uphill\setup.py
C:\Users\22240\rc2026_snapshot\rc\src\uphill\package.xml
```

节点默认参数：

```text
odom_topic = /odin1/odometry_highfreq
state_topic = /uphill/state
debug_topic = /uphill/debug
baseline_frames = 20
velocity_window_s = 0.40
vx_gate_mps = 0.08
pitch_contact_deg = 7.47
pitch_stable_deg = 12.69
pitch_platform_deg = 9.00
z_contact_m = 0.06
z_high_m = 0.36
hold_s = 0.25
ema_alpha = 0.18
```

节点输出：

```text
/uphill/state
  类型：std_msgs/String
  内容：中文状态名

/uphill/debug
  类型：std_msgs/String
  内容：JSON
  字段：state、rel_s、state_duration_s、pitch_delta_deg、pitch_abs_ema_deg、z_rise_ema_m、vx_mps
```

节点操作：

```text
R 或 r：重置状态机，重新采集平地基准
Ctrl+C：安全退出，恢复终端输入模式
```

构建和运行：

```bash
cd /mnt/c/Users/22240/rc2026_snapshot/rc
colcon build --packages-select uphill
source install/setup.bash
ros2 run uphill uphill_state_node
```

bag 回放验证：

```bash
ros2 bag play /mnt/c/Users/22240/rc2026_snapshot/bag/cha1nav2_20260612_154209 --clock
ros2 topic echo /uphill/state
ros2 topic echo /uphill/debug
```

已生成的离线分析脚本：

```text
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\work\extract_ramp_odom_axis.py
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\work\validate_ramp_odom_candidates.py
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\work\rerun_ramp_motion_gate.py
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\work\make_ramp_xlock_visualization.py
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\work\make_ramp_x_axis_explain.py
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\work\make_ramp_x_lock_decision.py
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\work\make_ramp_depth_clip_viewer.py
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\work\make_depth_only_viewer.py
```

重要输出文件：

```text
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_motion_gate_rerun.json
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_motion_gate_samples.csv
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_x_lock_decision.html
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_depth_only_viewer.html
```

深度片段筛选当前设置：

```text
原点：运动门控后的坡面接触点
方向：ramp_x 绑定 odom +X
ramp_x：-0.25m -> 1.85m
ramp_y：-0.90m -> 0.90m
z：-0.10m -> 0.80m
```

下一个 AI 优先任务：
1. 用用户新 bag 回放验证 `uphill_state_node` 的状态切换是否合理。
2. 如果状态提前或滞后，优先调整参数，不要直接改状态机结构。
3. 重点观察 `/uphill/debug` 中的 `pitch_abs_ema_deg`、`z_rise_ema_m`、`vx_mps`。
4. 验证新 bag 中坡道 X 是否仍可绑定 `odom +X`。
5. 若新 bag 出现明显 yaw 偏差，再判断是起步姿态、odom 漂移、路径弯曲，还是上坡方向假设不成立。
6. 等坡道状态和 X 轴稳定后，再继续第三区九宫格 ROI 和 TF 发布。

不要做的事：
- 不要修改 `.hermes_archive.md`。
- 不要把新文档、PDF、图片散放到项目根目录。
- 不要把 `pitch` 单帧阈值点当成精确坡道端点。
- 不要在没有新 bag 验证前贸然把状态机参数写死为最终值。

## 2026-06-14 任务计划

当天目标：
- 用新 bag 验证 `uphill_state_node`。
- 先把“上坡阶段判断 + 坡道 X 轴绑定”做稳。
- 暂时不推进九宫格识别和 TF 发布。

### 1. 验证 uphill 节点

构建并运行：

```bash
cd /mnt/c/Users/22240/rc2026_snapshot/rc
colcon build --packages-select uphill
source install/setup.bash
ros2 run uphill uphill_state_node
```

另开终端回放新 bag：

```bash
ros2 bag play 新bag路径 --clock
```

观察输出：

```bash
ros2 topic echo /uphill/state
ros2 topic echo /uphill/debug
```

期望状态顺序：

```text
平地
平地上坡中
上坡中
坡上上平台
上平台
```

重点观察 `/uphill/debug`：

```text
pitch_abs_ema_deg
z_rise_ema_m
vx_mps
state_duration_s
```

### 2. 优先调参数，不急着改结构

如果状态切换太早或太晚，优先调整参数：

```text
vx_gate_mps
pitch_contact_deg
pitch_stable_deg
pitch_platform_deg
z_contact_m
z_high_m
hold_s
ema_alpha
```

调参参考：
- 太早进入 `平地上坡中`：提高 `pitch_contact_deg` 或 `z_contact_m`。
- 太晚进入 `上坡中`：降低 `pitch_stable_deg`。
- 太晚进入 `上平台`：提高 `pitch_platform_deg` 或降低 `z_high_m`。
- 状态抖动：增大 `hold_s` 或减小 `ema_alpha`。

### 3. 确认坡道 X 轴是否仍可绑定 odom +X

用新 bag 检查：
- 坡面接触点到平台点的 yaw 偏差。
- 高度抬升是否接近 `0.40m~0.45m`。
- 平面前进距离是否接近 `1.5m~1.7m`。

如果 yaw 偏差仍较小：

```text
坡道 X = odom +X
```

如果 yaw 偏差明显变大，先判断：
- 起步朝向是否歪了。
- `odom` 是否漂移。
- 车是否斜着上坡。
- bag 是否不是蓝区或不是同一路径。

### 4. 重新生成深度片段

等状态机在新 bag 上正常后，再生成新的深度筛选画面：

```text
ramp_x：沿 odom +X
ramp_y：等宽走廊
z：坡道/平台高度
```

目标：
- 确认新 bag 也能裁出“直线手电筒式”的深度片段。
- 确认裁切结果不是扇形发散，而是固定宽度走廊。

### 5. 当天不做

- 不先改九宫格识别。
- 不直接接 TF 发布。
- 不把 `pitch` 单点阈值当坡道端点。
- 不在新 bag 没验证前把参数写死。

## 2026-06-15 uphill 参数调优记录

**问题**：`平地 -> 平地上坡中` 切换慢半拍，实际触发依赖 `z_contact`（高度）而非 `pitch_contact`（俯仰角），状态切换有约 0.5s 延迟。

**原因**：
- `ema_alpha=0.18` 太慢：pitch 突变时 EMA 需约 0.33s 才追到位
- `hold_s=0.25s` 叠加延迟
- 总计约 0.5s 滞后

**修改**（`uphill_state_node.py` 默认参数）：
```
ema_alpha: 0.18 -> 0.35    （响应快一倍）
hold_s:    0.25 -> 0.15    （防抖 150ms 足够）
```

**验证结果**（同 bag `cha1nav2_20260612_154209`）：

| 状态切换 | 旧 (ema=0.18) | 新 (ema=0.35) |
|---------|--------------|--------------|
| 平地→平地上坡中 触发 | z=0.085m（高度） | pitch=10.48°（pitch） |
| 平地上坡中→上坡中 间距 | 1.32s | 0.43s |
| 上坡中 pitch | 14.42° | 15.91° |

**结论**：EMA 加速后状态切换更及时，第一次切换改为 pitch 触发（更有物理意义），过渡段缩短。缺新 bag 验证，暂定 OK。

## 2026-06-15 DeepSeek 围栏 Y 轴定位尝试

**思路**：从 rc26_field.py 确认三区存在坡道围栏（Z3_RAMP_FENCE, 0.05×1.57×0.10m）和侧边栏（Z3_SIDE_FENCE, 0.05×1.25×0.10m）。两者都是 0.05m 薄壁结构，在点云中表现为细密线。利用上平台后沿 odom+X 切等宽走廊，在 Y 方向做点数直方图，峰值位置即为围栏 Y 坐标 → 反推 zone3_root Y。

**新增文件**：
- `find_fence_Y.py` — 离线围栏检测脚本，订阅 /uphill/state 和 /odin1/cloud_slam，在"上平台"状态下抓取点云并做 Y 直方图分析

**关键参数**：
- corridor_half = 0.9m（Y 方向搜索范围）
- x_near=0.0, x_far=6.0m（X 方向搜索范围）
- ground_z = -0.15m
- max_frames = 10（累积帧数）

## 2026-06-15 DeepSeek find_fence_Y v3

v1/v2 问题：写死 ROI（X=0~6m, Y=±0.9m），不同 bag 机器人位置完全不同导致走廊 0 点。

v3 修改：
- 同时订阅 /uphill/state + /odin1/odometry_highfreq + /odin1/cloud_slam
- 记录每个状态切换时的 odom 位姿
- 上平台后用机器人当前 odom 位置动态确定地面层 Z 范围
- 沿 odom+X 切等宽走廊，X 切片找 Y 峰值 + 尖锐度

## 2026-06-15 重要：bag 版本差异

**非常重要**：
- `cha1nav2_20260523_*` 三个 bag — **第一版本车**（v1）
- `cha1nav2_20260612_*` 和 `cha1nav2_20260613_*` — **第二版本车**（v2）
- 两版车差异很大，但当前工作不需要关心——我们只需要 odin 的 TF 坐标，用 odom 就足够

## 2026-06-15 find_fence_Y v3 结果

**运行 bag**: 20260613 或 20260523（用户第二个新 bag）

**odom 状态轨迹**:
```
平地:        x=8.95, y=3.33,  z=0.10, yaw=-7.4°
平地上坡中:  x=8.95, y=3.33,  z=0.10, yaw=-7.4°
上坡中:      x=9.12, y=3.30,  z=0.16, yaw=-8.3°
坡上上平台:  x=10.33, y=3.14, z=0.44, yaw=-15.0°
上平台:      x=10.52, y=2.95, z=0.44, yaw=-28.6°
```
坡道位移: (1.57, -0.38, 0.34), 长度 1.62m ✓

**围栏检测**:
- 地面层 (Z=0.2~0.7): 72378 点
- 走廊 (±1.5m Y): 14516 点
- X=[11.2,11.4]: peak Y=1.49, sharp=0.22
- X=[13.8,14.0]: peak Y=1.37, sharp=0.12
- sharpness 偏低 (<0.3), 信噪比不足

**下一步**（移交其他 AI 或新会话）:
- 将 /odin1/odometry_highfreq + /uphill/state + /uphill/debug 导出 CSV
- 批量验证 uphill 状态切换时机是否合理
- 用 bag 数据离线分析围栏位置
- 如需: 用 rc26_field.py 的围栏坐标 (RAMP_FENCE_Y=1.375, SIDE_FENCE_Y=0.0) 作为先验

## 2026-06-14 新 bag uphill 状态机校验

用户提供新 bag：

```text
C:\Users\22240\rc2026_snapshot\bag\cha1nav2_20260523_131930
```

用户现场运行日志：

```text
平地基准完成: z=-0.001, pitch=-9.66deg
平地 -> 平地上坡中, t=24.73s, pitch=8.37deg, z=0.086m, vx=0.505m/s
平地上坡中 -> 上坡中, t=25.05s, pitch=14.82deg, z=0.150m, vx=0.508m/s
上坡中 -> 坡上上平台, t=27.12s, pitch=6.80deg, z=0.444m, vx=0.529m/s
坡上上平台 -> 上平台, t=27.80s, pitch=3.49deg, z=0.443m, vx=0.510m/s
```

新增离线校验脚本：

```text
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\work\validate_uphill_bag.py
```

脚本用途：
- 读取 bag 中的 `/odin1/odometry_highfreq`。
- 按当前 `uphill_state_node.py` 的默认参数复现状态机。
- 输出状态切换表、逐帧 debug CSV 和 JSON 摘要。

输出文件：

```text
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\uphill_validation\cha1nav2_20260523_131930_uphill_validation.json
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\uphill_validation\cha1nav2_20260523_131930_uphill_debug_samples.csv
C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\uphill_validation\cha1nav2_20260523_131930_uphill_transitions.csv
```

离线复现结果：

```text
平地基准：z=-0.0022m, pitch=-9.72deg
24.727s  平地 -> 平地上坡中      pitch_abs_ema=8.31deg,  z=0.087m, vx=0.505m/s
25.072s  平地上坡中 -> 上坡中    pitch_abs_ema=13.52deg, z=0.155m, vx=0.511m/s
27.119s  上坡中 -> 坡上上平台    pitch_abs_ema=6.74deg,  z=0.445m, vx=0.529m/s
27.801s  坡上上平台 -> 上平台    pitch_abs_ema=3.43deg,  z=0.444m, vx=0.510m/s
72.254s  上平台 -> 平地          pitch_abs_ema=5.18deg,  z=0.034m, vx=0.047m/s
```

和现场日志对比：
- 状态顺序一致。
- 第一、三、四次切换时间与现场日志基本对齐。
- 第二次切换离线为 `25.072s`，现场为 `25.05s`，时间差很小。
- pitch 显示值略有差异，可能来自运行版本/安装版本与源码参数、EMA 中间值或日志取样时刻差异；状态切换本身正常。
- 完整 bag 后半段出现 `上平台 -> 平地`，与高度回到低位、速度下降一致，属于返回平地/下坡后的复位，不视为上坡段异常。

坡道段几何检查：

```text
平地上坡中 -> 上平台:
dx = 1.584m
dy = -0.374m
dz = 0.355m
平面长度 = 1.628m
yaw_from_odom_x = -13.29deg
持续时间 = 3.074s

上坡中 -> 坡上上平台:
dx = 1.225m
dy = -0.153m
dz = 0.288m
平面长度 = 1.234m
yaw_from_odom_x = -7.10deg
持续时间 = 2.047s
```

结论：
- `uphill_state_node` 在 `cha1nav2_20260523_131930` 上输出正常。
- 该 bag 为第一版本车，路径相对 `odom +X` 有明显负 yaw 偏差，不能简单套用旧 bag 的“坡道 X 近似 odom +X”结论。
- 但状态机本身能正确分出上坡相关阶段。
- 后续如果要用这个 bag 做点云走廊裁切，应优先使用状态切换点计算该 bag 的坡道局部方向，或结合场地先验判断 `odom` 坐标是否已旋转/漂移。

## 2026-06-15 围栏 Y 轴定位算法设计

新增 `other/fence_algorithm.md` — 完整算法设计文档。

**核心思路**：
- 不等上平台，在"上坡中/坡上上平台"就开始采点云
- 围栏特征：5cm 厚、10cm 高、紧贴地面 → 高度筛选 + 尖锐度判定
- 走廊：矩形不扇形（forward/lateral 等宽投影）
- 两侧都看到 → 直接求中心线；只一侧 → 用已知坡道宽 1.55m 推算
- 多帧融合：直方图峰值 + 置信度
- 遮挡/异常帧自动跳过

**输出**：坡道中心线 TF + 机器人 Y 偏移 + 置信度

## 2026-06-15 fence_locator 功能包

新增 ROS2 包 `rc/src/fence_locator/`：

**节点**: `fence_locator`
- 订阅 `/uphill/state` + `/odin1/cloud_slam`
- 在"上坡中 / 坡上上平台"采集点云，每帧检测围栏
- 发布 `/ramp/lateral_offset` + `/ramp/confidence`
- 发布 `odom → ramp_centerline` TF

**算法**:
- 局部坡面高度: 每 X 切片取 Z 第 5 分位
- 围栏尖锐度 > 0.4 判定
- 两侧都看到 → 直接中心线; 单侧 → 坡宽 1.55m 推算
- 多帧直方图峰值融合 + 置信度

**构建运行**:
```bash
cd /mnt/c/Users/22240/rc2026_snapshot/rc
colcon build --packages-select fence_locator
source install/setup.bash
ros2 run fence_locator fence_locator
```

**注意**: 和 uphill_state_node 独立运行，不修改 uphill 节点。
