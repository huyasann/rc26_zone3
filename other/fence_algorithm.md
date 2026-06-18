# 坡道围栏 Y 轴定位算法设计

> 目标：在"uphill / 坡上platform"阶段，利用坡道两侧 5cm 围栏定位机器人横向 Y。
> 核心原则：不等platform，上坡过程就开始采；围栏是薄壁 + 地面高度 + 已知宽度。

---

## 1. 围栏在点云中的特征

```
        俯视图 (XY)                    侧视图 (XZ, 沿坡方向)
        
   Y↑                        Z↑
    |  左围栏    右围栏        |
    |    |         |          |    坡面 ~~~~~~~~
    |    |  车道   |          |   /         平台 ___
    |    |         |          |  / 围栏 █   
    +-------------→ X         +----------------→ X
                                 地面 _________
```

- **厚度**：5cm → 在 Y 方向极窄，点落在 1~3 个 Y bin（0.02m/bin）
- **高度**：10cm，紧贴地面（Z ≈ ground_z ~ ground_z+0.15m）
- **走向**：沿坡道方向延伸数米
- **数量**：左右各一条，间距 = 坡道宽度 1.55m

---

## 2. 点云筛选三步

### 2.1 高度筛选：局部坡面模型（不用固定 ground_z）

**问题**：坡面从低到高有 0.4m 抬升，围栏贴着坡面。固定 `ground_z` 在坡底看到的围栏和坡顶看到的围栏，Z 坐标差 0.4m。固定值不可能同时覆盖。

**方案**：沿坡道方向把点云切成 X 切片，每片单独估计"当前坡面高度"，再以此为基准筛围栏。

```
# 对每个 X 切片
xslice_pts = 走廊内, forward ∈ [xi-0.25, xi+0.25] 的点

# 取 Z 的第 5 分位作为局部坡面高度（坡面点占多数，围栏仅高出 10cm）
local_ground_z = np.percentile(z_slice, 5)

# 围栏 = 局部地面之上 0 ~ 0.15m
fence_pts = (z_slice >= local_ground_z) & (z_slice <= local_ground_z + 0.15)
```

**为什么用第 5 分位而不是均值**：
- 坡面点占大多数 → 低分位 ≈ 地面高度
- 围栏点比地面高 5~10cm → 不会把围栏自身的点拉进"地面"估计
- 均值会被围栏点和噪声拉高

**误滤原因**：
| 物体 | Z 范围 | 能否通过此过滤 |
|------|--------|---------------|
| 围栏 ✓ | ground+0~0.10m | ✅ |
| 地面 | ground±0.02m | ⚠️ 通过，但地面是广域散布，围栏是窄峰 |
| 坡面 | ground~0.45m | ✗ |
| 平台顶面 | ~0.45m | ✗ |
| 九宫格 | >0.75m | ✗ |
| 车体自身 | 0.2~1.5m | ✗ |

地面和围栏同时通过高度筛选 → 靠第二步的"密度/尖锐度"区分。

### 2.2 走廊切割：用 odom 位移方向估计坡道方向

**问题**：不能盲信 odom +X 就是坡道方向。车可能斜着起步，odom 坐标系和场地可能有偏角。

**方案**：从 uphill 状态切换的 odom 位移中估计坡道方向。

```
# 在"uphill"阶段记录 odom 起点和终点
ramp_start  = flatuphill → uphill 时的 odom (x, y)
ramp_end    = uphill → 坡上platform 时的 odom (x, y)

# 位移方向 = 坡道方向
ramp_dx = ramp_end.x - ramp_start.x
ramp_dy = ramp_end.y - ramp_start.y
ramp_yaw = atan2(ramp_dy, ramp_dx)
```

**等宽走廊投影**（沿坡道方向，不是车头朝向）：

```
# 点云 → 坡道局部坐标系
dx = x_odom - robot_x
dy = y_odom - robot_y

forward  = dx * cos(ramp_yaw) + dy * sin(ramp_yaw)   # 沿坡道方向
lateral  = -dx * sin(ramp_yaw) + dy * cos(ramp_yaw)  # 垂直坡道方向

keep if:
  0.3 < forward < 5.0          # 车前方 0.3~5.0m
  abs(lateral) < 1.0           # 左右各 1.0m
```

### 2.3 密度筛选：围栏 = 又窄又密的峰

地面层点云沿 Y 做直方图（0.02m bin）。围栏所在的 Y 位置会出现：

```
Y 直方图（高度筛选后，沿 X 某切片）
  ↑ 点数
  |          █ ← 右围栏 (Y≈0.7m)
  |    █     █
  |   ██ █  ██
  |  ██████████ ← 地面广域分布
  +----------------→ Y
```

判断标准：
- **峰值点数** > 同一 X 切片平均水平的 3 倍
- **尖锐度** = 峰值 ±0.02m 点数 / 峰值 ±0.1m 点数 > 0.4
  - 围栏：0.05m 厚 → ±0.02m 集中了大量点 → sharp > 0.5
  - 地面：散布在很宽范围 → sharp < 0.2
  - 平台边缘：散布也较宽 → sharp < 0.3

---

## 3. Y 轴估计算法

### 3.1 沿 X 方向切片，每片找左右围栏

```
for x_slice in [1.0, 1.5, 2.0, 2.5, 3.0, 3.5]:
    pts = 走廊内 x_slice ±0.25m 的点
    做 Y 直方图，找前两个峰值
    如果 sharpness > 0.4 且间距 ≈ 1.55m → 左右围栏都看到了
    如果只有一个 sharp > 0.4 → 只看到一侧
```

### 3.2 两侧都看到：求中心线

```
center_lateral = (lateral_left + lateral_right) / 2
# center_lateral 就是机器人相对坡道中心线的横向偏移
```

### 3.3 只看到一侧：用已知坡道宽度推算

```
从 rc26_field.py: 坡道宽 1.55m
如果看到的是左围栏 (lateral < 0): center_lateral = fence_lateral + 1.55/2
如果看到的是右围栏 (lateral > 0): center_lateral = fence_lateral - 1.55/2

# 左右判断在坡道局部坐标系下（lateral），不用全局 odom y
```

### 3.4 遮挡判定：这帧不可信

以下情况标记为不可信（不参与融合）：
- 走廊内总点数 < 100（被 R1/KFS 大面积遮挡）
- 所有 Y 峰值 sharpness < 0.3（看不到清晰围栏）
- 两侧都看到但间距偏离 1.55m 超过 ±0.3m（误判了别的结构）
- 只看到一侧但推测出的中心线和上一帧跳变 > 0.5m（异常跳变）

---

## 4. 多帧融合

### 4.1 策略

```
1. flatuphill → 清空所有缓存，记录初始 odom
2. uphill → 每帧采点云，立即做上述单帧围栏检测
3. 坡上platform → 继续采，权重 1.5x（离平台更近，视野更好）
4. platform → 停止采新帧，用历史结果做最终融合
```

### 4.2 融合方法

```
所有可信帧的 center_lateral 估计值做直方图（bin=0.02m）
取直方图最高峰 → 最终 center_lateral

置信度 = 峰值帧数 / 总可信帧数
        × 峰值附近 ±0.05m 内帧数占比
        × (看到两侧的帧数 / 总可信帧数)  # 两侧比单侧更可信
```

### 4.3 发布

```
一旦置信度 > 0.6 → 发布 /ramp/centerline TF
发布 /ramp/lateral_offset 话题（机器人相对坡道中心线的横向偏移，m）
发布 /ramp/confidence 话题（0~1 浮点数）
```

---

## 5. ROS2 节点结构

```
ramp_fence_locator (Node):
  订阅:
    /uphill/state          → 知道当前阶段
    /odin1/cloud_slam      → 点云
    /odin1/odometry_highfreq → 机器人位姿 (可选，用于坐标系转换)
  
  发布:
    /ramp/centerline       → 坡道中心线 TF (odom 系)
    /ramp/lateral_offset    → Float32, 机器人相对坡道中心线的横向偏移 (m)
    /ramp/confidence       → Float32, 置信度 [0,1]
  
  调试发布 (ENABLE_DEBUG=1):
    /ramp/debug/raw        → 原始走廊内点云 (染色: 高度)
    /ramp/debug/fence_candidates → 围栏候选点 (红色)
    /ramp/debug/fence_lines → Marker, 拟合出的左右围栏线
    /ramp/debug/centerline_marker → Marker, 中心线
```

---

## 6. 调试可视化 (RViz2)

建议用一个固定 RViz 配置，显示以下层次：

1. **原始点云** `/odin1/cloud_slam` — 半透明，看全貌
2. **高度筛选后** `/ramp/debug/raw` — 只留 ground_z ~ ground_z+0.15m，确认围栏高度带没选错
3. **走廊切割后** — 在 `raw` 上叠加 corridor 范围 Marker，看是否把围栏切进来了
4. **围栏候选点** `/ramp/debug/fence_candidates` — 红色，sharpness>0.4 的点，确认没把地面当围栏
5. **拟合围栏线** `/ramp/debug/fence_lines` — 两条绿线，看是否平行、间距是否是 1.55m
6. **中心线** `/ramp/debug/centerline_marker` — 黄色虚线，看是否在坡道正中间
7. **当前 Y 偏移** `/ramp/y_offset` — 终端 echo 或叠加文字 Marker

---

## 7. 参数汇总

```python
# 高度
ground_z_offset = -0.02       # 围栏下界相对地面 (m)
fence_height = 0.15           # 围栏上界相对地面 (m)

# 走廊
corridor_near = 0.3           # 车头最近距离 (m)
corridor_far = 5.0            # 车头最远距离 (m)
corridor_half = 1.0           # 走廊半宽 (m)

# 围栏检测
y_bin_size = 0.02             # Y 直方图分辨率 (m)
x_slice_width = 0.5           # X 切片宽度 (m)
sharpness_threshold = 0.4     # 尖锐度阈值
peak_ratio_threshold = 3.0    # 峰值/均值 倍数阈值

# 坡道几何 (来自 rc26_field.py)
ramp_width = 1.55             # 坡道总宽度 (m)

# 融合
min_trusted_frames = 5        # 最少可信帧数才发布
confidence_threshold = 0.6    # 置信度阈值才发布 TF
ema_alpha = 0.3               # 输出平滑系数
```

---

## 8. 和现有 uphill 节点的关系

**不修改 uphill 节点**。ramp_fence_locator 订阅 `/uphill/state` 即可。

**和 find_fence_Y.py 的区别**：
- find_fence_Y：离线脚本，等platform后抓 10 帧，一次性分析。实验性工具。
- ramp_fence_locator：线上节点，uphill就开始采，实时发布。最终交付物。

---

## 9. 注意事项

- 围栏是已知宽度 (1.55m) 的平行结构 → 这是最强的先验，必须用在算法里
- 不要把坡面点误判为围栏 → 坡面 Z 跨度大 (0~0.45m)，围栏只有 0~0.1m
- R1 或 KFS 遮挡时 → 走廊内点数骤降，该帧直接跳过
- 如果车偏航角大 (yaw > 15° 偏离坡道方向) → 走廊可能切不进围栏，改用更宽的 corridor_half
- 不同 bag 的 ground_z 不同 → 从 uphill 节点的基准值获取，不写死
