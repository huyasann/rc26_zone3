# Zone3 — 九宫格检测详细文档

> 节点: `zone3_grid_localizer` (Zone3GridLocalizer)
> 源文件: `zone_detection/zone3/localizer_node.py`, `config.py`, `grid_detect.py`
> 目标: 在重试区内检测 3×3 九宫格高台，输出 `odom→{team}_zone3_root` TF

---

## 1. 概述

九宫格是 3×3 排列的 3 层高台方块。Zone3 节点在机器人到达重试区后，从 LiDAR 点云中检测这些方块簇，计算九宫格中心位姿，反推 zone3_root 导航坐标系，并以 TF 形式输出。

核心挑战：
- 九宫格方块间有空隙，需要连通域分析找出整体
- 不同层（层1/2/3）高度不同，需要层数验证过滤误检
- Z2 先验通道提供初锁，避免纯点云检测的不确定性

---

## 2. 完整处理流程

### 2.1 流程总览

```
/odin1/cloud_slam
    │
    ▼
┌─ 2.2 点云降采样 + ROI 裁剪 ───────────────────────────┐
│  DOWNSAMPLE_STEP=1, 按 source_frame 选不同 ROI 范围     │
│  base_link: X=0.15~5.50, Y=±2.20, Z=-1.50~2.80        │
│  odom:      X=0~16,     Y=±4.0                         │
└───────────────────────────────────────────────────────┘
    │
    ▼
┌─ 2.3 高度归一 (_ground_est.to_height_frame) ────────┐
│  点云 Z 投影到 odom 系 → z_h                          │
└───────────────────────────────────────────────────────┘
    │
    ▼
┌─ 2.4 地面检测 (GroundEstimator.update_ground) ──────┐
│  手动 GROUND_Z=-0.310 (GROUND_Z_KNOWN=1)              │
│  离地高度 h = z_h - ground_z                          │
└───────────────────────────────────────────────────────┘
    │
    ▼
┌─ 2.5 高度诊断日志 + 染色发布 ───────────────────────┐
│  HeightDiagLogger 逐帧写各层分布 / _publish_height_bands│
└───────────────────────────────────────────────────────┘
    │
    ▼
┌─ 2.6 重试区门控 (两阶段) ────────────────────────┐
│  位置门控: 机器人在重试区 ±0.8m 内                    │
│  朝向门控: 机器人 yaw 对准九宫格 ±45°                 │
│  不通过 → 清空累积帧 + pending_locks, 跳过            │
└───────────────────────────────────────────────────────┘
    │ 通过
    ▼
┌─ 2.7 Z2 先验锁 (_try_z2_prior_lock) ──────────────┐
│  首次进入重试区, 读 zone2 TF + 偏移 (0, -5.15)        │
│  直接初锁, 1s 冷却防空转                               │
└───────────────────────────────────────────────────────┘
    │
    ▼
┌─ 2.8 高位点筛选 + 多帧累积 ───────────────────────┐
│  h ∈ [0.75, 2.60], 逐帧累积 20 帧高位候选             │
└───────────────────────────────────────────────────────┘
    │
    ▼
┌─ 2.9 九宫格检测 (grid_detect.detect_grid_pose) ────┐
│  连通域分析 → PCA 主方向估计                            │
│  → 几何评分 (width/depth/layer/density)               │
└───────────────────────────────────────────────────────┘
    │
    ▼
┌─ 2.10 坐标系转换 ──────────────────────────────────┐
│  检测位姿从 source_frame → odom                       │
│  → choose_team_root_pose (九宫格中心 → zone3_root)    │
└───────────────────────────────────────────────────────┘
    │
    ▼
┌─ 2.11 多帧稳定锁 + 精修 ──────────────────────────┐
│  候选质量检查 → 多帧离散度 → 稳定锁                    │
│  到达后小范围精修 (±0.40m, ±8°, EMA)                  │
└───────────────────────────────────────────────────────┘
    │
    ▼
┌─ 2.12 TF 发布 ────────────────────────────────────┐
│  odom → {team}_zone3_root, 10Hz                      │
└───────────────────────────────────────────────────────┘
```

---

### 2.2 点云降采样 + ROI 裁剪

**降采样**: `DOWNSAMPLE_STEP=1`（不降采样，保留全部点以保证九宫格检测精度）。

**ROI 分帧策略**: 根据 `msg.header.frame_id` 选择不同的 ROI 范围：

| 帧类型 | X 范围 | Y 范围 | Z 范围 | 场景 |
|--------|--------|--------|--------|------|
| base_link | 0.15 ~ 5.50 | -2.20 ~ 2.20 | -1.50 ~ 2.80 | 车行驶中, 近距离前向 |
| odom | 0.0 ~ 16.0 | -4.0 ~ 4.0 | -1.50 ~ 2.80 | 车到重试区, 宽 ROI |

> odom 系 ROI 更宽：车已到达重试区时，点云可能在 odom 系直接发布，需要更大的横向范围覆盖九宫格。

**空数据处理**: ROI 内无点 → 发布空点云清空 RViz → 直接 return。

---

### 2.3 高度归一

调用 `GroundEstimator.to_height_frame()` 将点云 Z 坐标从 `source_frame` 投影到 `odom` 系 Z 轴。

**原理**: 通过 TF 查询 `source_frame → odom` 的旋转矩阵第三行 `[r20, r21, r22]`:
```
z_h = r20*x + r21*y + r22*z + tz
```
消除车体俯仰/侧倾影响。若 TF 不可用 → 返回 None → 跳过本帧。

**最少点数**: `len(z_h) < 40` 时直接跳过，避免稀疏数据误检。

---

### 2.4 地面检测

**模式**: Z3 使用手动固定地面 (`GROUND_Z_KNOWN=1`, `GROUND_Z=-0.310`)，因为平台地面回波不稳定，自动检测易波动。

**离地高度**: `h = z_h - ground_z`。

**地面点**: `|h| ≤ GROUND_TOLERANCE (0.035m)`。

---

### 2.5 高度诊断日志 + 染色发布

#### 高度诊断日志 (HeightDiagLogger)

启用时 (`ENABLE_HEIGHT_DIAG_LOG=1`) 逐帧记录：
- 各高度区间的点数（按 HEIGHT_BANDS 调色板分区统计）
- 层1/2/3 的 odom_z 中位数
- 每层最多 5 个采样点的 (x, y, z_h, h)

**HEIGHT_BANDS 调色板**:

| 区间 h (m) | 颜色 | 含义 |
|------------|------|------|
| -0.10 ~ 0.07 | 灰色 (180,180,180) | 地面附近 |
| 0.07 ~ 0.50 | 黄色 (255,255,0) | 基座/平台顶 |
| 0.50 ~ 0.80 | 青色 (0,200,200) | 基座~层1 间隙 |
| 0.80 ~ 1.34 | 橙色 (255,80,20) | 层1 方块 |
| 1.34 ~ 1.88 | 绿色 (80,200,80) | 层2 方块 |
| 1.88 ~ 2.50 | 紫色 (200,80,255) | 层3 方块 |

#### 高度染色发布 (`/rc26/zone3/cloud_height_bands`)

按 HEIGHT_BANDS 对点云染色发布到 RViz，用于调试地面高度标定和层高边界。

---

### 2.6 重试区门控

**启用开关**: `REQUIRE_ZONE3_ODOM_GATE=1`。

**门控分为两阶段**: 位置门控 + 朝向门控，任一不通过则清空所有累积状态并跳过。

#### 位置门控

通过 TF 查询 `odom → ROBOT_TF_FRAME ("odin1_base_link")` 获取机器人位姿 `(rx, ry, ryaw)`。

**重试区定义** (odom 系):

| 队伍 | 中心 X | 中心 Y | 半边长 | 说明 |
|------|--------|--------|--------|------|
| BLUE | 11.100 - 0.4 = 10.700 | 4.100 | 0.80m | 蓝队重试区 |
| RED  | 11.100 | -4.100 | 0.80m | 红队重试区 |

> 红蓝队重试区关于 Y=0 对称。

**通过条件**: `|rx - center_x| ≤ half_size and |ry - center_y| ≤ half_size`

#### 朝向门控

**朝向参考** (odom 系, 正对九宫格):

| 队伍 | retry_face_yaw | 计算依据 |
|------|---------------|----------|
| BLUE | -1.708 rad | atan2(-5.525, -0.75)，基于场地模型推算 |
| RED  | +1.708 rad | atan2(5.525, -0.75)，镜像 |

**通过条件**: `|norm_angle(ryaw - retry_face_yaw)| ≤ RETRY_FACE_YAW_TOLERANCE (45°)`

#### 不通过时的行为

```python
self._frames.clear()         # 清空多帧累积的高位点
self._pending_locks.clear()  # 清空锁定候选
self._publish_empty_vis()    # 清空 RViz
return                       # 跳过本帧
```

**首次通过日志**: 门控首次全部通过时，打印 `[重试区到达]` INFO 日志。

**TF 查询失败**: 若 `odin1_base_link → odom` TF 不可用，同样清空状态并跳过，1s 节流 warn。

---

### 2.7 Z2 先验锁 (`_try_z2_prior_lock`)

**目的**: 在纯点云检测不够稳定的情况下，利用 Z2（梅林前立面）的已知 TF + 固定场地偏移，快速获得 zone3 初锁。

**启用**: `ENABLE_Z2_PRIOR=1`。

**触发条件**:
- 尚未锁定 (`self._locked == False`)
- 尚未精修 (`self._refined == False`)
- 启用 Z2 先验 (`enable_z2_prior == True`)
- 距上次尝试 ≥ 1s（冷却，避免每帧查 TF 空转）

**TF 查询**: `tf_buffer.lookup_transform("odom", z2_frame, Time())`

**固定偏移**: Z2 → Z3 在场地模型中的平移向量。

| 参数 | 值 | 说明 |
|------|-----|------|
| Z2_TO_Z3_OFFSET_X | 0.0 | X方向无偏移 |
| Z2_TO_Z3_OFFSET_Y | -5.15 | Z2_ROOT=(±3.025, 0.55), Z3_ROOT=(±3.025, -4.60) |

**计算**:
```python
c, s = cos(syaw), sin(syaw)
ox = c * offset_x - s * offset_y   # 将偏移旋转到 odom 系
oy = s * offset_x + c * offset_y
self._tf_x = z2_tf.x + ox
self._tf_y = z2_tf.y + oy
self._tf_yaw = norm_angle(syaw)
self._locked = True
```

**行为**: 直接锁定，不经过多帧稳定检查。锁定后依然可以走精修流程在小范围内修正。

**失败处理**: 若 Z2 TF 尚未就绪（zone2 还没锁），每 1s 重试，不阻塞点云检测流程。

---

### 2.8 高位点筛选 + 多帧累积

**高位筛选**: `h ∈ [GRID_MIN_H=0.75, GRID_MAX_H=2.60]`

| 边界 | 值 | 含义 |
|------|-----|------|
| GRID_MIN_H | 0.75m | 基座以上，第一层方块底部 |
| GRID_MAX_H | 2.60m | 第三层方块顶部 |

**最少点数**: 单帧高位点 `≥ 40` 才纳入累积（低于此跳过）。

**多帧累积**: 保留最近 `ACCUMULATE_FRAMES=20` 帧的高位点，拼接成一个大的点云数组 `(ax, ay, ah)`:
```python
self._frames.append((x[high], y[high], h[high]))
while len(self._frames) > self._accumulate_frames:
    self._frames.pop(0)
ax = np.concatenate([f[0] for f in self._frames])
ay = np.concatenate([f[1] for f in self._frames])
ah = np.concatenate([f[2] for f in self._frames])
```

> 累积多帧可以增加九宫格方块的点密度，弥补单帧遮挡和扫描稀疏。

---

### 2.9 九宫格检测 (`grid_detect.detect_grid_pose`)

详细算法见下文 [3. 九宫格检测算法详解](#3-九宫格检测算法详解)。

**输入**: 累积高位点 `(ax, ay, ah)`、高度范围 `(grid_min_h, grid_max_h)`。

**输出**: `GridDetection` 包含 `(center_x, center_y, yaw, confidence, point_count, width, depth, layer_count, mask)`，或 None（未检出）。

---

### 2.10 坐标系转换

#### source_frame → odom (`_grid_pose_to_odom`)

如果点云不在 odom 系（例如 `slam` 系），通过 TF 转换：
```python
t = tf_buffer.lookup_transform("odom", source_frame, Time())
syaw = quat_yaw(t.transform.rotation)
c, s = cos(syaw), sin(syaw)
ox = c * gx - s * gy + t.transform.translation.x
oy = s * gx + c * gy + t.transform.translation.y
oyaw = norm_angle(gyaw + syaw)
```

**异常**: TF 查询失败 → 1s 节流 warn → return None。

#### 九宫格中心 → zone3_root (`choose_team_root_pose`)

九宫格检测给出的是九宫格几何中心在 odom 系中的位姿 `(grid_ox, grid_oy, grid_oyaw)`。需要转换到 zone3_root。

**相对偏移** (在场地模型坐标系中):

| 参数 | 值 | 说明 |
|------|-----|------|
| GRID_FIELD_X | 0.0 | 九宫格中心 X |
| GRID_FIELD_Y | -4.75 | 九宫格中心 Y |
| BLUE_ZONE3_ROOT_X | 3.025 | 蓝队 zone3_root X |
| RED_ZONE3_ROOT_X | -3.025 | 红队 zone3_root X |
| ZONE3_ROOT_FIELD_Y | -4.60 | zone3_root Y |

**运行时的相对偏移**:
```
grid_center_rel_x = GRID_FIELD_X - ZONE3_ROOT_X    # 蓝队: 0.0 - 3.025 = -3.025, 红队: 0.0 - (-3.025) = 3.025
grid_center_rel_y = GRID_FIELD_Y - ZONE3_ROOT_FIELD_Y  # -4.75 - (-4.60) = -0.15
```

**两候选解**: yaw 可能存在 180° 模糊性（九宫格近似对称），生成 `(yaw, yaw+π)` 两组候选:
```python
for yaw in (grid_yaw, norm_angle(grid_yaw + π)):
    root_x = grid_x - (cos(yaw)*rel_x - sin(yaw)*rel_y)
    root_y = grid_y - (sin(yaw)*rel_x + cos(yaw)*rel_y)
```

**选择策略**: 蓝队选 `root_y` 更大的解，红队选 `root_y` 更小的解（对应各自的平台侧）:
```python
expected_side = 1.0 if is_blue_team else -1.0
candidates.sort(key=lambda item: (expected_side * root_y, root_x), reverse=True)
```

---

### 2.11 多帧稳定锁 + 精修

#### 候选质量过滤器 (`_is_good_grid_candidate`)

单帧检测必须同时满足以下全部条件，否则清空 pending_locks:

| 检查项 | 阈值 | 含义 |
|--------|------|------|
| confidence | ≥ MIN_LOCK_CONFIDENCE (0.76) | 综合几何评分 |
| layer_count | ≥ 3 | 高度层数（必须检测到 3 层） |
| point_count | ≥ 700 | 连通域内点数 |
| width | 1.30 ~ 1.95m | Y 向宽度 (九宫格 ~1.62m) |
| depth | 0.15 ~ 0.60m | X 向深度 (单层块厚 ~0.32m) |

#### 多帧稳定性 (`_lock_candidate_is_stable`)

连续 `STABLE_LOCK_COUNT=4` 帧候选都通过质量检查，且位姿离散在阈值内:

| 检查项 | 阈值 |
|--------|------|
| 中心离散 spread | ≤ STABLE_CENTER_TOL (0.35m) |
| yaw 离散 yaw_spread | ≤ STABLE_YAW_TOL (12°) |

> Z3 的稳定阈值比 Z2 宽松，因为九宫格距离更远，点云更稀疏。

#### 锁定 (`_do_lock`)

取末尾 `STABLE_LOCK_COUNT` 帧候选的位姿均值:
```python
self._tf_x = mean(xs)
self._tf_y = mean(ys)
self._tf_yaw = mean_yaw(yaws)
self._locked = True
```

#### 精修 (`_try_refine`)

**触发条件**: 已锁定 + 尚未精修 + 重试区门控重开 (`_zone3_odom_gate_open`)。

**精修约束**:

| 约束 | 阈值 |
|------|------|
| 与锁定点的平移距离 | ≤ Z3_REFINE_MAX_TRANSLATION (0.40m) |
| 与锁定点的 yaw 差 | ≤ Z3_REFINE_MAX_YAW_DELTA (8°) |

**EMA 修正**:
```python
self._tf_x += Z3_REFINE_ALPHA (0.65) * (mean_x - self._tf_x)
self._tf_y += Z3_REFINE_ALPHA (0.65) * (mean_y - self._tf_y)
self._tf_yaw = blend_yaw(self._tf_yaw, mean_yaw_v, Z3_REFINE_ALPHA)
```

**注意**: Z3 精修是**增量式**的（`+=`），直接在 Z2 先验锁的基础上叠加；而 Z2 精修是对锁定位姿做加权平均。Z3 的 `Z3_REFINE_MAX_TRANSLATION=0.40m` 比 Z2 的 `REFINE_MAX_TRANSLATION=0.65m` 更紧，因为九宫格定位更可靠。

**精修完成后**: `self._refined = True`，不再触发精修。

---

### 2.12 TF 发布

**发布时机**:
1. 锁定后：每次 `cloud_cb` 中的检测通过后，调用 `_publish_tf(msg.header.stamp)` 发布单帧 TF
2. 定时器：锁定后 10Hz (`DYNAMIC_TF_RATE`) 调用 `_publish_tf_timer` 持续发布 TF

**TF 内容**:
- parent: `odom` (`SOURCE_FIXED_FRAME`)
- child: `{team}_zone3_root`
- translation: `(self._tf_x, self._tf_y, GROUND_Z)`
- rotation: `quat_from_yaw(self._tf_yaw)`

**调试 TF** (`zone3_retry_test`): 定时器还发布重试区中心位置 TF，用于 RViz 可视化位置。

---

## 3. 九宫格检测算法详解 (`grid_detect.py`)

### 3.1 数据结构

```python
@dataclass
class GridDetection:
    center_x: float       # 九宫格中心 X (当前坐标系)
    center_y: float       # 九宫格中心 Y
    yaw: float            # 主朝向 (长轴方向转 -π/2, 归一化到 [-π/2, π/2])
    confidence: float     # 综合评分 [0, 1]
    point_count: int      # 连通域内点数
    width: float          # Y 向宽度 (m, robust span)
    depth: float          # X 向深度 (m, robust span)
    layer_count: int      # 检测到的高度层数 (0~3)
    mask: np.ndarray      # bool 标记哪些输入点属于此候选
```

### 3.2 入口函数 `detect_grid_pose()`

**输入**:
| 参数 | 默认值 | 说明 |
|------|--------|------|
| x, y, h | — | 点云坐标 + 离地高度 |
| grid_min_h | 0.75 | 高位点筛选下限 |
| grid_max_h | 2.60 | 高位点筛选上限 |
| min_high_points | 40 | 高位点最少数量 |
| min_confidence | 0.25 | 单帧最低评分（低于此不计） |
| grid_width_y | 1.62 | 期望九宫格 Y 向宽度 |

**流程**:
1. **预处理**: 转为 float64，截取 3 数组最短长度，过滤非有限值
2. **高位筛选**: `finite & (h >= grid_min_h) & (h <= grid_max_h)`，点数不足返回 None
3. **连通域分析**: `_connected_components()`，cell=0.08m, min_cell_points=2
4. **逐连通域拟合**: 对每个连通域调用 `_fit_component()`
5. **最优选择**: max(confidence * log(point_count))，返回最佳 GridDetection

### 3.3 连通域分析 `_connected_components()`

**网格法**: 将点云按 `cell=0.08m` 划分栅格，基于 8 邻域做 flood-fill 连通域搜索。

**步骤**:
1. **栅格化**: `ix = floor(x / cell)`, `iy = floor(y / cell)`，组成 `(ix, iy)` 键
2. **活跃细胞**: 点数 ≥ `min_cell_points=2` 的栅格
3. **构建邻接图**: 对每个活跃细胞，检查 8 个邻居是否也是活跃细胞
4. **BFS Flood Fill**: 从每个未访问的活跃细胞开始，广度优先搜索收集所有连通点
5. **排序**: 按连通域大小降序排列

### 3.4 单连通域拟合 `_fit_component()`

#### PCA 主方向估计

1. **中心化**: `demean = pts - center0`
2. **协方差矩阵**: `cov = np.cov(demean.T)`
3. **特征分解**: `np.linalg.eigh(cov)`，取最大特征值对应的特征向量为长轴方向
4. **方向归一化**: `yaw = atan2(long_axis[1], long_axis[0]) - π/2`，然后归一化到 `[-π/2, π/2]`

> 减 π/2: 因为 PCA 长轴是点云分布最广的方向（Y=宽度方向），而我们需要的是深度方向（X=垂直于九宫格正面的方向）。九宫格 Y 向宽 ~1.62m，X 向深 ~0.32m，所以长轴沿 Y，法向沿 X。

5. **半圈归一化** (`_normalize_half_turn`): 将 yaw 裁剪到 [-π/2, π/2]

#### 几何尺寸计算

**定义局部坐标系**:
```python
lx = cos(yaw) * (x - center0.x) + sin(yaw) * (y - center0.y)  # 局部 X (深度方向)
ly = -sin(yaw) * (x - center0.x) + cos(yaw) * (y - center0.y) # 局部 Y (宽度方向)
```

**鲁棒跨度** (`_robust_span`): 97% 分位 - 3% 分位，剔除两端 3% 离群值。
```python
width = _robust_span(ly)     # Y 向宽度
depth = _robust_span(lx)     # X 向深度
```

**宽度/深度过滤**:
- width < 0.65 或 width > 2.35 → 剔除
- depth > 0.80 → 剔除

> 这些阈值是 `_fit_component` 内部的硬过滤，独立于 `localizer_node.py` 中的 `_is_good_grid_candidate`。内部过滤更宽（0.65~2.35 vs 1.30~1.95），是粗筛。

**中心精化**: 用局部 X/Y 的 5%/95% 和 3%/97% 分位中点代替简单均值，更鲁棒。

#### 综合评分

**层数统计** (`_count_height_layers`):
```python
ranges = ((0.80, 1.34), (1.34, 1.88), (1.88, 2.42))
# 每区间 ≥ 30 点算一层
layer_count = sum(int(((h >= lo) & (h < hi)).sum() >= 30) for lo, hi in ranges)
```

**四项评分**:

| 评分项 | 公式 | 权重 |
|--------|------|------|
| width_score | exp(-|width - 1.62| / 0.42) | 40% |
| depth_score | exp(-max(0, depth - 0.55) / 0.35) | 10% |
| layer_score | min(1.0, layer_count / 3.0) | 35% |
| density_score | min(1.0, n_points / 420) | 15% |

**综合分数**: `confidence = 0.40*width + 0.10*depth + 0.35*layer + 0.15*density`

> 层数权重最高（35%），因为三层方块是九宫格区别于其他高台结构的关键特征。

**最终过滤**: confidence < 0.25 → 返回 None

### 3.5 team_root_pose 选择 (`choose_team_root_pose`)

**两候选解**: 九宫格近似对称，yaw 可能存在 180° 模糊性。

```python
for yaw in (grid_yaw, norm_angle(grid_yaw + π)):
    c, s = cos(yaw), sin(yaw)
    root_x = grid_x - (c*rel_x - s*rel_y)
    root_y = grid_y - (s*rel_x + c*rel_y)
    expected_side = 1.0 if is_blue else -1.0
    candidates.append((expected_side * root_y, root_x, root_x, root_y, yaw))
candidates.sort(key=lambda item: (item[0], item[1]), reverse=True)
# 选 root_y 最大(蓝)或最小(红)的解
```

---

## 4. 全部配置参数

### 4.1 输入点云 ROI

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| X_MIN | 0.15 | m | base_link 前向最小 |
| X_MAX | 5.50 | m | base_link 前向最大 |
| Y_MIN | -2.20 | m | base_link 横向最小 |
| Y_MAX | +2.20 | m | base_link 横向最大 |
| Z_MIN | -1.50 | m | 高度下限 |
| Z_MAX | +2.80 | m | 高度上限 |
| ODOM_X_MIN | 0.0 | m | odom 系 X 下限 |
| ODOM_X_MAX | 16.0 | m | odom 系 X 上限 |
| ODOM_Y_MIN | -4.0 | m | odom 系 Y 下限 |
| ODOM_Y_MAX | +4.0 | m | odom 系 Y 上限 |

### 4.2 坐标系

| 参数 | 默认值 | 说明 |
|------|--------|------|
| HEIGHT_FRAME | "odom" | 高度归一帧 |
| SOURCE_FIXED_FRAME | "odom" | TF 父帧 |
| ROBOT_TF_FRAME | "odin1_base_link" | 机器人 TF frame |

### 4.3 地面估计

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| GROUND_Z_KNOWN | 1 | — | 手动固定 |
| GROUND_Z | -0.310 | m | 手动地面高度 |
| GROUND_TOLERANCE | 0.035 | m | 地面容差 |
| GROUND_PEAK_RATIO | 0.15 | — | 主峰比例 |
| HISTOGRAM_BIN_WIDTH | 0.02 | m | 直方图 bin |
| GROUND_UPDATE_ALPHA | 0.18 | — | EMA 系数 |
| GROUND_MAX_UPDATE_STEP | 0.06 | m | 单帧最大跳变 |

### 4.4 队伍与场地

| 参数 | 默认值 | 说明 |
|------|--------|------|
| IS_BLUE_TEAM | 0 | 运行参数覆盖 |
| BLUE_ZONE3_ROOT_X | 3.025 | 蓝队 zone3_root X (场地模型) |
| RED_ZONE3_ROOT_X | -3.025 | 红队 zone3_root X |
| ZONE3_ROOT_FIELD_Y | -4.60 | zone3_root Y (场地模型) |
| GRID_FIELD_X | 0.0 | 九宫格中心 X (场地模型) |
| GRID_FIELD_Y | -4.75 | 九宫格中心 Y (场地模型) |
| GRID_WIDTH_Y | 1.62 | 九宫格 Y 向宽度 (m) |
| GRID_DEPTH_X | 0.32 | 单层块 X 向深度 (m) |
| GRID_MIN_H | 0.75 | 高位筛选下限 (m) |
| GRID_MAX_H | 2.60 | 高位筛选上限 (m) |

### 4.5 重试区门控 + Z2 先验

| 参数 | 默认值 | 说明 |
|------|--------|------|
| REQUIRE_ZONE3_ODOM_GATE | 1 | 启用重试区门控 |
| BLUE_RETRY_CENTER_X | 10.700 | 蓝队重试区 X |
| BLUE_RETRY_CENTER_Y | 4.100 | 蓝队重试区 Y |
| RED_RETRY_CENTER_X | 11.100 | 红队重试区 X |
| RED_RETRY_CENTER_Y | -4.100 | 红队重试区 Y |
| RETRY_AREA_HALF_SIZE | 0.80 | 重试区半宽 (m) |
| RETRY_AREA_Z | 0.30 | 调试 Marker 高度 |
| BLUE_RETRY_FACE_YAW | -1.708 rad | 蓝队朝向 |
| RED_RETRY_FACE_YAW | +1.708 rad | 红队朝向 |
| RETRY_FACE_YAW_TOLERANCE | 45° | 朝向容差 |
| ENABLE_Z2_PRIOR | 1 | 启用 Z2 先验 |
| Z2_TO_Z3_OFFSET_X | 0.0 | Z2→Z3 X 偏移 |
| Z2_TO_Z3_OFFSET_Y | -5.15 | Z2→Z3 Y 偏移 |
| Z3_REFINE_MAX_TRANSLATION | 0.40m | 精修最大平移 |
| Z3_REFINE_MAX_YAW_DELTA | 8° | 精修最大 yaw 变化 |
| Z3_REFINE_ALPHA | 0.65 | 精修 EMA 系数 |
| Z3_REFINE_STABLE_COUNT | 3 | 精修稳定帧数 |

### 4.6 锁定策略

| 参数 | 默认值 | 说明 |
|------|--------|------|
| MIN_LOCK_CONFIDENCE | 0.76 | 单帧评分阈值 |
| STABLE_LOCK_COUNT | 4 | 稳锁帧数 |
| STABLE_CENTER_TOL | 0.35m | 中心离散容差 |
| STABLE_YAW_TOL | 12° | yaw 离散容差 |
| DYNAMIC_TF_RATE | 10Hz | TF 重发频率 |
| DOWNSAMPLE_STEP | 1 | 点云降采样 (1=不降) |
| ACCUMULATE_FRAMES | 20 | 高位累计帧数 |

### 4.7 高度诊断

| 参数 | 默认值 | 说明 |
|------|--------|------|
| ENABLE_HEIGHT_DIAG_LOG | 0 | 高度诊断日志开关 |
| HEIGHT_DIAG_LOG_DIR | ~/files/record/logs | 日志目录 |

### 4.8 调试输出

| 参数 | 默认值 | 说明 |
|------|--------|------|
| LOG_INTERVAL | 0.5s | 日志节流间隔 |
| DETAILED_FILE_LOG | False | CSV 日志开关 |
| DEBUG_DIR | ~/files/record/logs | 调试日志目录 |
| ENABLE_DEBUG_VIS | 1 | 调试可视化总开关 |

---

## 5. 接口

### 订阅

| 话题 | 类型 | QOS | 用途 |
|------|------|-----|------|
| `/odin1/cloud_slam` | PointCloud2 | 10 | Odin LiDAR 实时拼接点云 |

### 发布

| 话题 | 类型 | 条件 | 用途 |
|------|------|------|------|
| `/rc26/zone3/cloud_grid_candidates` | PointCloud2 | ENABLE_DEBUG_VIS | 候选点: 高位(橙)+地面(白) |
| `/rc26/zone3/cloud_grid_model` | PointCloud2 | ENABLE_DEBUG_VIS | 九宫格骨架线框 (黄色) |
| `/rc26/zone3/cloud_height_bands` | PointCloud2 | ENABLE_DEBUG_VIS | 9段高度染色 |
| `/rc26/zone3/retry_area_marker` | Marker | ENABLE_DEBUG_VIS | 重试区绿色半透明方块 |

### TF

| Frame ID | 方向 | 含义 |
|----------|------|------|
| `{team}_zone3_root` | odom → child | Z3 导航系原点, 锁定后 10Hz |
| `zone3_retry_test` | odom → child | 重试区中心 (调试用) |

---

## 6. 生命周期

```
                    ┌─────────────────┐
                    │     启动        │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │  等待进入重试区 │
                    │  (位置+朝向门控)│
                    └────────┬────────┘
                             │ 进入
                    ┌────────▼────────┐
                    │  Z2 先验初锁    │──── Z2 TF 未就绪 ────► 1s 冷却重试
                    │  (仅首次, 1s冷却)│
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │  多帧累积 +     │──── 离
                    │  九宫格检测     │──── 开
                    └───┬────────┬───┘    重
                        │        │        试
                  检到  │   未检到        区
                        │        │        ↓
                    ┌───▼──┐ ┌──▼──┐  清空所有累积
                    │稳定锁│ │下帧 │  回到等待
                    └───┬──┘ └──┬──┘
                        │       │
                    ┌───▼───────▼──┐
                    │ 精修         │
                    │ (Z2基础上修正)│
                    └───────┬──────┘
                            │
                    ┌───────▼──────┐
                    │ TF 稳定输出  │
                    │ 10Hz 持续    │
                    └──────────────┘
```

**各阶段行为**:
- 未进重试区 → 无检测，门控跳过所有帧，累积帧清空
- 离开重试区 → 清空累积帧和 pending_locks，回到等待状态
- 九宫格检不出 → 持续尝试，每帧评估，累积帧保持
- Z2 TF 未就绪 → 1s 冷却重试，不阻塞点云检测流程

---

## 7. 异常保护

| 保护点 | 位置 | 机制 |
|--------|------|------|
| TF 查询失败 (grid→odom) | `_grid_pose_to_odom` | try/except + 1s 节流 warn |
| TF 查询失败 (robot→odom) | `_get_robot_pose` | try/except + 1s 节流 warn |
| TF 查询失败 (Z2→odom) | `_try_z2_prior_lock` | try/except + 1s 冷却 |
| 地面检测 TF 不可用 | `GroundEstimator.to_height_frame` | 返回 None → 跳过该帧 |
| 点云空数据 | `cloud_cb` | width==0 或 height==0 → return |
| 连通域分析空结果 | `_connected_components` | 返回空列表 → None |
| PCA 协方差非有限 | `_fit_component` | `np.all(np.isfinite(cov))` 检查 |
| `__del__` 半初始化 | `__del__` | hasattr 保护 `_debug_csv/_event_log/_height_diag_log` |

---

## 8. 调试

### RViz 可视化

1. **`/rc26/zone3/cloud_grid_candidates`**: 橙色=高位候选点，白色=地面点。检查高位筛选是否正确覆盖九宫格方块
2. **`/rc26/zone3/cloud_grid_model`**: 黄色骨架线框，与实测点云叠对比，验证检测位姿
3. **`/rc26/zone3/cloud_height_bands`**: 6 色高度染色，检查 ground_z 标定和层高边界
4. **`/rc26/zone3/retry_area_marker`**: 绿色半透明方块，验证重试区位置和范围
5. **TF `zone3_retry_test`**: 重试区中心坐标系

### 日志

- **终端日志**: 每 0.5s 输出 detection/detection_none 事件含 confidence/width/depth/layers
- **CSV 日志** (`DETAILED_FILE_LOG=True`): 逐帧检测数据含所有评分项
- **事件日志**: odom_gate_wait / detection / detection_none 事件
- **高度诊断日志** (`ENABLE_HEIGHT_DIAG_LOG=1`): 每帧各层点数分布 + 采样坐标

### 常见问题

| 现象 | 可能原因 | 排查步骤 |
|------|----------|----------|
| 检测不到 | 未进重试区 / ground_z 偏了 / 被遮挡 | 检查 gate 日志 + height_bands 染色 + confidence |
| Z2 先验锁不上 | zone2 还没锁 / offset 不对 | 等 zone2 锁定，检查 Z2_TO_Z3_OFFSET |
| 锁定后偏移大 | 选错候选解 (yaw 180° 模糊) | 检查 choose_team_root_pose 选择 |
| 层数统计不准 | 高度区间边界不对 | 看 height_diag 日志各层点数分布 |

---

## 9. 依赖

- **ROS2**: rclpy, tf2_ros, sensor_msgs, geometry_msgs, visualization_msgs
- **Python**: numpy, math, dataclasses
- **内部**: zone_detection.common (point_cloud_utils, tf_utils, ground_estimator, debug_logger), zone_detection.zone3.grid_detect
