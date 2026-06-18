# rc2026_snapshot 项目接手记录

## 2026-06-13 Codex atlas V3 field layout reread

- Source PDF: `C:\Users\22240\rc2026_snapshot\主赛图册V3.pdf`
- Clearest saved field layout image: `C:\Users\22240\rc2026_snapshot\main_atlas_v3_field_layout_page04.png`
- Page used: page 4, title `附录3 比赛场地尺寸`

Field layout notes read from the clearer atlas page:
- Red/blue half fields are shown side by side; each half-width is marked `6000mm`.
- Vertical zone split on each side:
  - Zone 1: `2000mm`
  - Zone 2: `7300mm`
  - Zone 3: `2700mm`
- Zone3 ramp/platform detail on the same page:
  - Zone3 platform label: `450mm Platform`
  - Ramp cross-section shows rise-related dimensions `400mm` and slope angle label `15°`.
  - Ramp horizontal segment is marked `1500mm`.
  - Upper platform segment is marked around `1250mm`.
  - Several edge/guard dimensions appear around `50mm`, `100mm`, and acrylic guard details; these should not be mixed into the main pitch/z odom validation unless specifically modeling guards.

Intended odom-validation use:
- Do not assume there is only one pitch-z event without checking.
- First globally search `/odin1/odometry_highfreq` for all candidate patterns:
  - pitch leaves baseline
  - z rises toward ramp/platform height
  - pitch returns near baseline while z stays high
- Then score candidates against atlas geometry:
  - expected pitch near `15°`
  - expected z rise roughly `0.40m~0.45m`
  - expected ramp-axis distance between enter/level transitions roughly `1.50m`

## 2026-06-13 Codex odom ramp validation concept

Odom 验证的含义:
- 不是直接做九宫格定位。
- 是用 `/odin1/odometry_highfreq` 的车体运动轨迹，检查 bag 里哪一段确实像“通过三区坡道”。
- 核心观察量:
  - `z`: 车体高度是否上升。
  - `pitch`: 车体俯仰角是否进入坡面角，再回到接近平地。
  - `x/y/yaw`: 车在坡道方向上走了多远，方向是否稳定。

理论上坡道过程应表现为:
```text
平地:
  z 基本不变
  pitch 接近 baseline

进入坡道:
  pitch 明显离开 baseline，接近图纸坡角
  z 开始持续升高

坡中:
  z 持续升高
  pitch 保持在坡面角附近

到上平台:
  z 接近高位并趋于稳定
  pitch 回到接近 baseline
```

图纸/场地模型约束:
- `主赛图册V3.pdf` 第 4 页:
  - 坡角标注约 `15°`
  - 坡道水平长度 `1500mm`
  - Zone3 platform 标注 `450mm`
  - 坡道抬升相关尺寸 `400mm`
- `zone_detection/zone_detection/field_publisher/rc26_field.py`:
  - `Z3_RAMP_WIDTH = 1.55`
  - `Z3_RAMP_HLEN = 1.5`
  - `Z3_RAMP_Z_LOW = 0.05`
  - `Z3_RAMP_Z_HIGH = 0.45`
  - `atan((0.45 - 0.05) / 1.5) ~= 14.93deg`

验证策略:
1. 先全局扫描整段 odom，不预设只有一次上坡事件。
2. 找所有候选模式:
   - `pitch` 离开 baseline。
   - `z` 开始上升。
   - `pitch` 回到 baseline 附近。
   - `z` 保持高位。
3. 对每个候选打分:
   - 最大/稳定 pitch 是否接近 `15°`。
   - 总 z rise 是否接近 `0.40m~0.45m`。
   - 进入坡面到回平台之间的坡道轴向距离是否接近 `1.50m`。
   - yaw 是否相对稳定，是否符合 `ramp_drive_yaw_hold.py` 的航向保持假设。
4. 只把最符合图纸/模型的候选认作真实坡道事件。

当前 bag 的已提取结果:
- Bag: `bag/cha1nav2_20260612_154209/`
- Topic: `/odin1/odometry_highfreq`
- 数据输出:
  - `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\summary.json`
  - `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\angle_change_1.csv`
  - `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\angle_change_2.csv`
  - `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\ramp_odom_axis_locked.csv`
- 目前识别出的两次主 pitch 变化:
  - `19.7627s ~ 20.8030s`: 进入坡道，`ramp_x 0.000m ~ 0.235m`
  - `26.7200s ~ 27.4728s`: 到达上平台并回平，`ramp_x 1.462m ~ 1.680m`
- 这两次变化之间的坡道轴向距离约接近 `1.5m`，与图纸/场地模型的坡道长度相符。

风险提醒:
- 不能只因为出现一次 `pitch + z` 变化就认定是坡道。
- 后续要保留“全局多候选搜索 + 图纸几何打分”的流程，可以不用某个候选，但不能跳过候选检查。
## 2026-06-13 Codex ramp odom axis-lock extraction

- Input bag: `bag/cha1nav2_20260612_154209/`
- Input topic: `/odin1/odometry_highfreq`
- Reference script: `ramp_drive_yaw_hold.py`
- Extraction script: `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\work\extract_ramp_odom_axis.py`
- Output dir: `C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\ramp_odom_axis_lock\`

Purpose:
- Use the same odom topic and ramp-yaw-hold assumptions to identify the two main pitch angle changes during climbing.
- Build a temporary ramp coordinate frame for offline map work:
  - origin: first main pitch-change start pose
  - X axis: baseline yaw from the first 20 odom frames, matching `ramp_drive_yaw_hold.py` startup yaw-hold idea
  - `ramp_x = cos(yaw0)*(odom_x-origin_x)+sin(yaw0)*(odom_y-origin_y)`
  - `ramp_y = -sin(yaw0)*(odom_x-origin_x)+cos(yaw0)*(odom_y-origin_y)`
  - `ramp_z = odom_z-baseline_z`

Detected main pitch-change intervals:
- `angle_change_1`: rel time `19.7627s ~ 20.8030s`, ramp_x `0.000m ~ 0.235m`; likely entering ramp / pitch down to slope.
- `angle_change_2`: rel time `26.7200s ~ 27.4728s`, ramp_x `1.462m ~ 1.680m`; likely leaving ramp / leveling onto platform.
- A short later candidate around `29.7s ~ 29.84s` exists, but was not used as the two main climb angle changes.

Generated files:
- `summary.json`: parameters, baseline yaw/z, interval boundaries, ramp-axis formula.
- `angle_change_1.csv`: odom rows only inside first pitch change.
- `angle_change_2.csv`: odom rows only inside second pitch change.
- `ramp_odom_axis_locked.csv`: climb window around the two changes with added `ramp_x/ramp_y/ramp_z`.
- `angle_change_all_candidates.csv`: all derivative-based pitch-change candidates for audit.

Notes:
- Baseline from this bag: yaw `0.0945deg`, z `-0.0020m`, pitch `-11.8314deg`.
- The extracted ramp axis is an offline temporary coordinate lock, not yet a ROS TF publisher.
- Next useful step: use the ramp_x anchors to crop point clouds from the ramp/platform interval and fit a more stable local slope/platform map.
本文档用于多人/多 AI 协作。请在修改代码、参数、调试方法或发现问题后持续追加记录，避免不同会话重复摸索。

## 当前接手目标

当前主目标不是完整比赛流程，而是聚焦第三区九宫格定位：

> 使用 `odin1` 这一套设备，稳定识别第三区九宫格，并稳定发布九宫格地盘正中心相关 TF 坐标。

更具体地说，当前工作重点是 `zone_detection.zone3.localizer_node.Zone3GridLocalizer`：

- 输入：`/odin1/cloud_slam`
- 当前 bag 中该点云 `frame_id=odom`
- 输出：`odom -> {blue|red}_zone3_root`
- 期望：让下游能够基于稳定的第三区/九宫格坐标完成放置、对齐、颜色/占用判断等动作

## 目录速览

- `bag/cha1nav2_20260612_154209/`
  - ROS2 bag，约 10.66GB。
  - 主要用于离线回放和调试第三区定位。
  - 关键话题：
    - `/odin1/cloud_slam`: `sensor_msgs/msg/PointCloud2`
    - `/odin1/odometry_highfreq`: `nav_msgs/msg/Odometry`
    - `/tf`, `/tf_static`
- `configs/challenge1_common.yaml`
  - 队伍侧、日志开关、相机外参等公共配置。
- `launch/test.launch.py`
  - bag 回放 + RViz + zone2/zone3/kfs 节点启动。
  - 注意：当前文件含有 `/home/inkc/inkc/Rc2026/...` 硬编码路径，迁移/复现时要检查。
- `zone_detection/`
  - ROS2 Python 包。
  - 当前重点：
    - `zone_detection/zone_detection/zone3/localizer_node.py`
    - `zone_detection/zone_detection/zone3/config.py`
    - `zone_detection/zone_detection/zone3/grid_detect.py`

## 当前对第三区定位流程的理解

`zone3_localizer` 当前主流程：

1. 订阅 `/odin1/cloud_slam`。
2. 解析 PointCloud2 的 `x/y/z`。
3. 按点云坐标系做 ROI 裁剪：
   - 若 `frame_id == odom`：
     - `X: 0.0 ~ 16.0`
     - `Y: -4.0 ~ 4.0`
     - `Z: -1.5 ~ 2.8`
   - 否则用车前方 ROI：
     - `X: 0.15 ~ 5.50`
     - `Y: -2.20 ~ 2.20`
     - `Z: -1.50 ~ 2.80`
4. 统一高度：
   - 目标高度系是 `odom`。
   - 当前 bag 的 `/odin1/cloud_slam` 已经是 `odom`，所以高度约等于 `h = z - ground_z`。
   - 当前 `zone3/config.py` 里 `GROUND_Z_KNOWN=1`, `GROUND_Z=-0.1500`。
5. 只保留九宫格高位结构点：
   - `GRID_MIN_H=0.75`
   - `GRID_MAX_H=2.60`
6. 累积最近 `ACCUMULATE_FRAMES=20` 帧高位点。
7. `grid_detect.detect_grid_pose()` 进行：
   - 连通域分析
   - PCA 估计主方向
   - 根据 width/depth/layer_count/density 评分
8. 把九宫格中心反推出 `{team}_zone3_root`。
9. 多帧稳定后锁定并以 10Hz 发布 TF。

## 二去三/二区上三区的现有逻辑

代码中存在 Z2 先验：

- 读取 `odom -> {team}_zone2_root`
- 使用固定偏移推算 `zone3_root`
  - `Z2_TO_Z3_OFFSET_X = 0.0`
  - `Z2_TO_Z3_OFFSET_Y = -5.15`
- 公式含义：场地模型里 `zone2_root` 到 `zone3_root` 的固定关系。

重要问题：

- 当前 `ENABLE_Z2_PRIOR=1`
- 但 `REQUIRE_ZONE3_ODOM_GATE=0`
- 在现有 `localizer_node.py` 中，`_try_z2_prior_lock()` 只在“重试区门控开启且位置门控通过”的代码块里调用。
- 所以默认配置下，Z2 先验实际不会触发，zone3 主要走纯点云检测锁定。

这点后续需要明确选择：

1. 启用 `require_zone3_odom_gate:=true`，让 Z2 先验按原设计触发。
2. 或改代码，让 Z2 先验独立于重试区门控运行。
3. 或完全不用 Z2 先验，改成 odom/场地固定先验 + 点云模型匹配。

## 已知问题

### 1. 当前九宫格检测可能不稳定

当前算法偏“粗几何”：

- 从大 ROI 中取高位点。
- 用连通域找最大/最像九宫格的高结构。
- 用 PCA 算方向。

潜在不稳定来源：

- ROI 太大，可能混入其它高结构。
- 点云稀疏或遮挡时，连通域会断裂。
- PCA 对局部缺失、外点、墙面/边框点敏感。
- 地面高度 `GROUND_Z` 偏差会直接影响三层高度筛选。
- 仅靠 `width/depth/layer_count/density` 评分，缺少对“3列 x 3层”结构的强约束。

### 2. `setup.py` 存在无效入口

`zone_detection/setup.py` 中注册了：

```text
zone3_platform_fitter = zone_detection.zone3_fit.localizer_node:main
```

但当前源码中没有 `zone3_fit` 包。运行该入口会失败。

### 3. launch 路径有硬编码

`launch/test.launch.py` 中存在多个 `/home/inkc/inkc/Rc2026/...` 路径。若从当前 snapshot 独立运行，需替换或参数化。

## 推荐接手方向

目标是“稳定识别九宫格并稳定发布九宫格地盘正中心 TF”，建议优先按以下方向推进。

### 方向 A：先验小 ROI + 模型匹配

不要在全场大 ROI 中直接找九宫格。优先使用先验缩小搜索范围：

- 如果有可靠 `zone2_root`：
  - 用 `zone2_root + (0, -5.15)` 得到 `zone3_root` 初值。
- 如果没有 Z2：
  - 使用场地模型/odom 预估位置作为初值。

然后只在初值附近做小范围搜索，例如：

- 平移范围：`±0.5m`
- yaw 范围：`±15deg`
- 高度用固定三层模型

对每个候选 `(x, y, yaw)` 计算九宫格模型评分。

### 方向 B：用 2.5D/结构化评分替代单纯 PCA

九宫格结构已知，不应只看整体连通域。

建议将点变换到候选九宫格局部坐标后，按结构统计：

- 3列：左/中/右
- 3层：底/中/顶
- 深度范围：九宫格厚度附近
- 可选：前表面/边框区域点密度

候选评分可由以下部分组成：

- 九格内点数覆盖率
- 三层高度是否分明
- 三列横向间距是否合理
- 外框/深度是否合理
- 外点惩罚
- 与上一帧/先验位姿偏差惩罚

这样即使某几格被遮挡，整体模型仍可稳定。

### 方向 C：加入时间滤波/状态机

发布 TF 不应完全依赖单帧检测。

建议状态机：

1. `UNLOCKED`
   - 等待先验或模型匹配候选。
2. `PRIOR_LOCKED`
   - 使用 Z2/场地先验先发布一个初值。
3. `REFINING`
   - 只允许候选在小范围内修正。
4. `LOCKED`
   - 连续稳定后发布固定/低通滤波后的 TF。
5. `LOST`
   - 短时间检测失败不立刻跳变，保留上次可信 TF。

滤波可先用 EMA，后续再考虑 Kalman。

### 方向 D：先做离线评估，再改线上节点

已经生成过一个离线 HTML 查看器：

- 位置：`C:\Users\22240\Documents\Codex\2026-06-12\base-ros-huya-mnt-c-users\outputs\zone3_filter_viewer.html`
- 作用：查看 `/odin1/cloud_slam` 的原始点、ROI、高度分层、高位候选和当前检测结果。
- 注意：这是临时调试产物，不在 snapshot 项目目录内。

建议接下来做一个更正式的离线评估脚本：

- 从 bag 中抽取 `/odin1/cloud_slam`。
- 对每帧输出：
  - high point count
  - detected center/yaw
  - confidence
  - width/depth/layer_count
  - 与上一帧的跳变量
- 生成 CSV，便于判断算法是否稳定。

## 当前最重要的决策问题

后续接手者请优先确认：

1. 最终要发布的是：
   - `zone3_root`？
   - 九宫格几何中心？
   - “九宫格地盘正中心”的定义是否等同于当前 `zone3_root`？
2. 是否允许/希望依赖 `zone2_root` 作为第三区初值？
3. `odom` 在比赛中是否足够稳定，还是需要用 `zone2`/场地结构重建局部坐标？
4. 是否需要同时输出九宫格 9 个格子的中心 TF，方便后续放置/颜色检测？

## 建议下一步任务

1. 明确“地盘正中心 TF”的精确定义。
2. 修正或重构 Z2 先验触发逻辑。
3. 编写离线评估脚本，对当前 bag 跑完整 `zone3_localizer` 检测指标。
4. 实现“小 ROI + 九宫格模型评分”的替代检测器。
5. 将替代检测器接入 `Zone3GridLocalizer`，保留原 PCA 检测作为 fallback 或 debug 对比。
6. 在 RViz/HTML/CSV 中对比：
   - 原始 PCA 方案
   - 新模型匹配方案
   - Z2 先验 + 精修方案

## 记录习惯

请每次修改后追加：

- 修改时间
- 修改者/AI 名称
- 修改文件
- 修改目的
- 关键参数
- 验证方式
- 发现的新问题

示例：

```text
2026-06-12 Codex
- 新增/修改：...
- 目的：...
- 验证：...
- 问题：...
```

## 思路记录

### 2026-06-12 用户思路：围绕比赛流程做第三区检测

比赛实际流程中，对抗区阶段通常是：

1. R1 先进入/上九宫格附近。
2. R2 随后进入。
3. 九宫格周边可能已有 KFS。
4. R1 和已放置/携带的 KFS 都可能遮挡 `odin1` 的部分视野。
5. 因此不应假设每次都能完整扫描到整个九宫格或整个场地。

用户提出的核心检测思路：

- 机器人到达第三区后，会先转向九宫格方向。
- 此时可以利用“朝向九宫格”这个动作，把点云按距离和角度切出一部分更干净的观察区域。
- 检测重点应放在：
  - 深度/距离切割
  - 角度/视锥切割
  - 在局部可见点云中稳定反推出九宫格中心/地盘正中心 TF

后续方案需要重点解决：

- 在 R1/KFS 遮挡导致九宫格不完整可见时，如何仍然稳定估计中心。
- 如何利用已知比赛流程和车辆朝向，减少误检和大 ROI 干扰。
- 如何让 TF 高精度、低抖动、可持续发布，而不是每帧受局部点云变化影响。

### 2026-06-13 用户思路：深度切割不应只做扩散扇形

用户提出：雷达/点云可以做距离切割，但如果只按角度切割，会像手电筒一样越远越扩散，容易把道路外侧或更多无关结构包含进来。

后续应区分两种 ROI：

1. 扇形/视锥 ROI：
   - 按距离 `r` 和角度 `theta` 保留点。
   - 优点：符合传感器视野。
   - 缺点：越远越宽，容易包含道路外结构。
2. 等宽走廊 ROI：
   - 沿机器人朝向或九宫格方向定义一条中心线。
   - 只保留中心线左右固定宽度内的点。
   - 形式上是一个带 yaw 的矩形/长条/有向包围盒。
   - 更符合“只看道路两边内内容”的需求。

建议第三区定位优先使用“距离范围 + 等宽走廊 + 高度范围”的组合，而不是单纯角度扇形：

```text
forward = dot(point - origin, direction)
lateral = dot(point - origin, normal)

keep if:
  near <= forward <= far
  abs(lateral) <= corridor_half_width
  h_min <= height <= h_max
```

如果道路/行驶路径不是直线，可进一步扩展为“沿路径 polyline 的走廊 ROI”，即保留离规划路径小于固定宽度的点。

### 2026-06-13 规则复核：对抗区检测位姿不能假设固定

重新复核规则后，对第三区定位有以下约束：

- 对抗区包括坡道、九宫格、重试区、已用兵器区。
- R1/R2 必须通过坡道进入和离开对抗区。
- R1 负责底层放置，R2 负责中层放置，被 R1 举起的 R2 负责顶层放置。
- R1 可以用兵器移除对方九宫格中的 KFS。
- 双方可能围绕同一格子放置、防守、移除，裁判可能不处理僵持。
- 因此九宫格周围可能存在：
  - 本队 R1
  - 本队 R2
  - 对方机器人局部伸入/干扰
  - 已放置 KFS
  - 正在放置/移除过程中的 KFS 或兵器

关键结论：

- 不能假设机器人到第三区后的检测位置固定。
- 也不能假设九宫格完整无遮挡。
- 比较可靠的流程条件是：机器人上到第三区后会主动转向九宫格方向。
- 后续 ROI/识别算法应使用“当前车体位姿 + 朝向九宫格”构造动态局部 ROI，而不是写死 odom 中某一块区域。

建议：

- 第一层使用车体系动态 ROI：
  - `forward` 表示沿当前车头/九宫格方向的距离。
  - `lateral` 表示左右偏移。
  - 保留一个等宽走廊，而不是全场大框。
- 第二层使用九宫格局部模型拟合：
  - 允许部分遮挡。
  - 使用已知九宫格尺寸补全中心。
- 第三层使用时间滤波和状态机：
  - 检测失败或被遮挡时短时保留上次 TF。
  - 只允许小范围精修，避免 TF 跳变。

### 2026-06-13 场地图补充：第三区几何对检测的影响

用户提供了《附录3 比赛场地尺寸》俯视尺寸图。当前按图初步理解：

- 场地整体约为 12m x 12m，红蓝两半场关于中线对称。
- 中间有纵向隔板/分界，红蓝半场各约 6000mm 宽。
- 场地按纵向分区：
  - 一区/武馆约 2000mm 深。
  - 二区/梅林约 7300mm 深，包含树林方块和通道。
  - 三区/对抗区约 2700mm 深。
- 三区不是全场自由空间，但比之前误读的 2000mm 更深，约 2700mm。
- 三区内存在九宫格、重试区、坡道/平台等结构；机器人上三区后会在该区域内围绕九宫格进行放置和攻防。

对九宫格定位的影响：

- 不能继续用全场大 ROI 粗找九宫格，场地图说明三区纵深有限，应该利用这个几何约束。
- 若 `odom`/场地模型可用，可先限制点云必须落在三区带状区域附近。
- 但实际对抗中机器人站位和角度仍不固定，所以最终裁切仍应基于当前车体朝向做动态 ROI。
- 更合理的组合是：

```text
场地先验：点应该在第三区带状区域附近
车体先验：九宫格在车头前方
ROI 形状：等宽走廊，不是扩散扇形
高度先验：只看九宫格/平台相关高度
模型先验：九宫格尺寸固定，允许遮挡后补全中心
```

后续建议在离线工具里增加“第三区场地边界/九宫格先验位置”的参考线，帮助判断点云与规则图坐标是否对齐。
