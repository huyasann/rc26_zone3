# Zone2 — 梅林前立面检测详细文档

> 节点: `zone2_detector_node` (Zone2DetectorNode)
> 源文件: `zone_detection/zone2/detector_node.py`, `config.py`
> 目标: 从 LiDAR 点云检测梅林台阶阵列的前立面，输出 `odom→{team}_zone2_root` TF

---

## 1. 概述

梅林台阶由 200mm 高白线和 400mm 高黄线交替砌成，组成规则的台阶阵列。Zone2 节点从 Odin SLAM 的实时点云中检测这些台阶的前表面（前立面），定位靶心（白线 Y 中心），计算最佳观测位姿，并以 TF 形式输出导航坐标系 `{team}_zone2_root`。

核心挑战：
- 黄柱（黄线区）和白柱（白线区）需要准确区分，避免互相干扰
- 立面拟合需要抵抗侧面墙壁、武馆区等非台阶平面的误检
- 在多帧抖动中保持锁定位姿稳定

---

## 2. 完整处理流程

### 2.1 流程总览

```
/odin1/cloud_slam
    │
    ▼
┌─ 2.2 观测位姿门控 (_odom_gate_check) ─────────────────┐
│  机器人不在观测区内 → 跳过本帧                          │
└───────────────────────────────────────────────────────┘
    │ 通过
    ▼
┌─ 2.3 点云降采样 + ROI 裁剪 ─────────────────────────┐
│  DOWNSAMPLE_STEP=5, X=0~3.5, Y=±1.7, Z=±0.50         │
└───────────────────────────────────────────────────────┘
    │
    ▼
┌─ 2.4 高度归一 (_ground_est.to_height_frame) ────────┐
│  点云 Z 投影到 odom 系 → z_h                          │
└───────────────────────────────────────────────────────┘
    │
    ▼
┌─ 2.5 地面检测 (GroundEstimator.update_ground) ──────┐
│  直方图自动估计 或 手动 GROUND_Z=-0.310                │
│  离地高度 h = z_h - ground_z                          │
└───────────────────────────────────────────────────────┘
    │
    ▼
┌─ 2.6 前立面切片 + 2D 直方图边缘检测 ───────────────┐
│  h∈[0.005, 0.400], 分 200/400 带                     │
│  Y×X 直方图 → 每行取最近有效 bin → 边缘点集            │
└───────────────────────────────────────────────────────┘
    │
    ▼
┌─ 2.7 RANSAC 直线拟合 (_fit_ransac) ───────────────┐
│  拟合 x = k*y + b, |k|≤0.6                           │
│  → 立面基准线方向 yaw + 截距 b                        │
└───────────────────────────────────────────────────────┘
    │
    ▼
┌─ 2.8 墙面点筛选 + Y方向分柱统计 ──────────────────┐
│  距线 ±0.10m 内的点, 按 Y 方向分 100 柱              │
│  每柱统计 200 带点数和 400 带点数                     │
│  → 黄柱判定: 200≥3 and 400≥3                         │
│  → 白柱判定: 200≥3 and 400==0, 扣除黄柱膨胀区        │
└───────────────────────────────────────────────────────┘
    │
    ▼
┌─ 2.9 柱级→点级映射 ────────────────────────────────┐
│  将柱统计 bool 数组映射回点云原始索引                 │
│  确保后续靶心定位使用点级数据                          │
└───────────────────────────────────────────────────────┘
    │
    ▼
┌─ 2.10 靶心定位 (_locate_target_and_publish) ──────┐
│  白线 Y 中心 ± 黄柱边界约束 → 靶心 (y_center)        │
│  合成白线点 + 红心高斯簇 → 重建点云发布               │
└───────────────────────────────────────────────────────┘
    │
    ▼
┌─ 2.11 TF 发布 + 自动锁定/精修 ────────────────────┐
│  立面位姿 → zone2_root TF                            │
│  多帧稳定 → 自动锁                                   │
│  入口区触发精修 (EMA 修正)                             │
└───────────────────────────────────────────────────────┘
```

---

### 2.2 观测位姿门控 (`_odom_gate_check`)

**目的**: 防止机器人在 Z1（武器库）或未到达 Z2 观测区时误检立面。

**输入**: 机器人当前 odom 位姿 `(rx, ry, ryaw)`，由 `/odin1/odometry_highfreq` 实时更新。

**门控区域**: 以 `(gate_center_x, gate_center_y)` 为中心的矩形，半边长 `gate_half_size_x/y`。
- 蓝队默认: `(1.600, 1.350)`，半宽 0.75m
- 红队默认: `(-1.600, -1.350)`，镜像

**朝向约束**: 车头朝向 odom 正前方（yaw≈0），容差 ±30°。

**门控强度**: 机器人越靠近观测区中心，`gate_strength` 越高（1.0→0.1），影响后续锁定灵敏度。
```
ratio = max(|dx| / half_size_x, |dy| / half_size_y)
gate_strength = max(0.1, 1.0 - ratio * attenuation)
```

**启用开关**: `ENABLE_ODOM_GATE=1` 时启用，设为 0 则始终通过。

---

### 2.3 点云降采样 + ROI 裁剪

**降采样**: 每 5 点取 1 点 (`DOWNSAMPLE_STEP=5`)，减少后续计算量。

**ROI 边界** (base_link 系):

| 轴 | 最小值 | 最大值 | 说明 |
|----|--------|--------|------|
| X  | 0.00   | 3.50   | 车头前方, 覆盖梅林第一排 |
| Y  | -1.70  | 1.70   | 横向, 覆盖梅林全宽 |
| Z  | -0.50  | 0.50   | 高度薄片, 排除天花板 |

**空数据处理**: 如果 ROI 内无点，发布空点云清空 RViz 显示，直接返回。

---

### 2.4 高度归一

调用 `GroundEstimator.to_height_frame()` 将点云 Z 坐标从 `base_link`/`slam` 系投影到 `odom` 系的 Z 轴。

**原理**: 通过 TF 查询 `source_frame → odom` 的旋转矩阵，提取第三行 `[r20, r21, r22]`，计算:
```
z_h = r20*x + r21*y + r22*z + tz
```
消除车体俯仰/侧倾对高度的影响。若 TF 不可用，返回 None，跳过本帧。

---

### 2.5 地面检测

**两种模式:**

| 模式 | GROUND_Z_KNOWN | 行为 |
|------|---------------|------|
| 手动固定 | 1 | 直接使用 `GROUND_Z=-0.310` 作为地面高度 |
| 自动估计 | 0 | 首帧自动直方图检测 + 后续 EMA 更新 |

**直方图检测**: 将 z_h 以 0.02m 的 bin 宽度生成直方图，从最低处扫描到主峰 15% 处，认作地面高度。

**EMA 更新** (`GROUND_UPDATE_ALPHA=0.18`, `GROUND_MAX_UPDATE_STEP=0.06`): 每帧更新地面估计，限制单帧最大跳变 0.06m 防止拉飞。

**离地高度**: `h = z_h - ground_z`。地面点判据: `|h| ≤ GROUND_TOLERANCE (0.035m)`。

---

### 2.6 前立面切片 + 2D 直方图边缘检测

**立面有效窗口** (离地 h):
- 整体切片: `h ∈ [0.005, 0.400]`
  - 200mm 白线带: `h ∈ [0.050, 0.200]`
  - 400mm 黄线带: `h ∈ [0.250, 0.400]`

**2D 直方图**: 将切片内的点按 (Y, X) 网格计数。
- Y 方向: `num_y_bins = int((Y_MAX - Y_MIN) / Y_BIN_SIZE)`，Y_BIN_SIZE=0.05m
- X 方向: `num_x_bins = int((X_MAX - X_MIN) / X_BIN_WIDTH)`，X_BIN_WIDTH=0.05m
- 密度阈值: `MIN_DENSITY_PEAK=5`（单个 bin 至少 5 点才视为有效）

**边缘提取**: 对每一行（固定 Y bin），取有密度的最小 X bin（最近车头的那列），作为该行的边缘点。最少需要 `MIN_EDGE_POINTS=5` 个边缘点才继续。

---

### 2.7 RANSAC 直线拟合 (`_fit_ransac`)

**目的**: 从边缘点集中拟合前立面的直线方程 `x = k*y + b`。

**参数**:
| 参数 | 值 | 说明 |
|------|-----|------|
| RANSAC_N_ITER | 200 | 迭代次数 |
| RANSAC_RESIDUAL_THRESHOLD | 0.05m | 内点残差阈值 |
| MAX_ALLOWED_SLOPE | 0.6 | 斜率上限，防深度方向误检 |
| MIN_EDGE_POINTS | 5 | 最少内点数 |

**算法**:
1. 每次随机采样 2 个点，计算斜率 k = (x2-x1)/(y2-y1)
2. 若 |k| > MAX_ALLOWED_SLOPE 或 y1≈y2，跳过
3. 计算 b = x1 - k*y1，统计残差 ≤ 阈值的点数
4. 保留最佳内点集
5. **最小二乘精化**: 用所有内点做 `lstsq` 精化 k, b
6. 再次验证精化后的 |k| ≤ MAX_ALLOWED_SLOPE

**输出**: `(k_opt, b_opt, inlier_mask)`，其中 `yaw = atan(k_opt)`。

**异常处理**: 内点数不足或迭代中无有效样本 → raise ValueError → `_detect_facade` 捕获并跳过该帧。

---

### 2.8 墙面点筛选 + Y方向分柱统计

**旋转坐标系**: 将 ROI 内的所有点旋转到立面坐标系:
```
x_rot = cos_yaw * x - sin_yaw * y
y_rot = sin_yaw * x + cos_yaw * y
x_facade_rot = cos_yaw * b    (基准线在旋转系下的 X)
```

**墙面约束**: `|x_rot - x_facade_rot| ≤ WALL_HALF_WIDTH (0.10m)`，仅取距立面基准线 10cm 内的点。

**Y 方向分柱**:
- 柱宽: `COL_WIDTH=0.05m`
- Y 范围: `[-2.5, 2.5]` → 共 `Y_COLUMN_NUM=100` 柱
- `y_idx = floor((y_rot - Y_COLUMN_MIN) / COL_WIDTH)`

**双通道统计**:
- `cnt_200[col]`: 该柱内同时满足 200 带 mask 且 on_wall 的点数
- `cnt_400[col]`: 该柱内同时满足 400 带 mask 且 on_wall 的点数

**柱级判定**:

| 判定 | 条件 | 含义 |
|------|------|------|
| 黄柱 `is_yellow[col]` | cnt_200≥3 and cnt_400≥3 | 同时有白线和黄线，台阶的黄色区域 |
| 白柱候选 `is_white_cand[col]` | cnt_200≥3 and cnt_400==0 | 仅有白线，白线区候选 |
| 白柱 `is_white[col]` | is_white_cand 且不在黄柱膨胀区内 | 过滤掉黄柱边缘溢出 |

**黄柱膨胀** (`DILATE_BINS=2`): 用 `scipy.ndimage.binary_dilation` 将黄柱向左右各膨胀 2 格 (0.10m)，防止黄柱边缘因点数不足被误判为白柱。

---

### 2.9 柱级→点级映射

**关键修复**（见 README.md 纠错记录）: 柱统计数组 (`is_white`, `is_yellow`) 长度为 ~100（柱数），不能直接索引点级数组（长度 ~3000 点），否则导致 `IndexError: boolean index dimension mismatch`。

**映射逻辑** (参考原 zone2.py L502-508):
```python
is_white_pt = np.zeros(len(x), dtype=bool)
is_yellow_pt = np.zeros(len(x), dtype=bool)
m = valid_wall & on_wall & (h <= 1.0)
if m.any():
    ci = y_idx[m]         # 墙上每个点对应的柱号
    is_white_pt[m] = is_white[ci]    # 从柱级映射回点级
    is_yellow_pt[m] = is_yellow[ci]
```

此后所有消费函数 (`_locate_target_and_publish`) 只接收点级 bool 数组。

---

### 2.10 靶心定位 (`_locate_target_and_publish`)

**靶心 Y 坐标**:
1. 取白线点的 `y_rot` 范围: `y_white_min ~ y_white_max`
2. 初始中心: `y_center = (y_white_min + y_white_max) / 2`

**黄柱边界约束**（四级修正）:
- 两侧都有黄柱 → `y_center = (left_yellow_max + right_yellow_min) / 2`
- 仅右侧有黄柱 → `y_center = right_yellow_min - 0.60`
- 仅左侧有黄柱 → `y_center = left_yellow_max + 0.60`
- 无黄柱 → 保持初始中心

**合成可视化点云** (`/rc26/zone2/cloud_facade_recon`):

| 颜色 | 数据源 | 说明 |
|------|--------|------|
| 黄色 (255,255,0) | 原始黄柱点 | 真实墙面上的黄色区域 |
| 白色 (255,255,255) | 合成线段 | y_white_min→y_white_max 等间距 500 点，z 取白线 Z 均值 |
| 红色 (255,0,0) | 合成高斯簇 | 靶心周围 100 点，σ=0.015m 高斯散布 |

**输出**: `(x_facade_rot, y_center, yaw, n_white, n_yellow, y_white_min, y_white_max)`，无靶心时返回 None。

---

### 2.11 TF 发布 + 自动锁定/精修

#### TF 计算 (`_publish_zone2_root_tf`)

1. **立面坐标系 → base_link 坐标系**:
   ```
   lx = cos_yaw * x_facade_rot + sin_yaw * y_center
   ly = -sin_yaw * x_facade_rot + cos_yaw * y_center
   ```

2. **base_link → odom 转换**: 通过 TF 查询 `odom → source_frame` 得到 `(dx, dy, syaw)`:
   ```
   ox = cos(syaw) * lx - sin(syaw) * ly + dx
   oy = sin(syaw) * lx + cos(syaw) * ly + dy
   ```

3. **TF yaw 计算**:
   ```
   tf_yaw = ZONE2_ROOT_YAW_OFFSET - yaw + syaw
   ```
   其中 `ZONE2_ROOT_YAW_OFFSET = π/2`。

4. **方向判断**: 确保 zone2_root 朝向梅林（深度方向与视线方向点积 > 0），否则 yaw += π。

5. **TF 原点**: 从靶心位置减去 target 偏移:
   ```
   tf_x = ox - (cos(tf_yaw)*target_x - sin(tf_yaw)*target_y)
   tf_y = oy - (sin(tf_yaw)*target_x + cos(tf_yaw)*target_y)
   ```
   其中 `target_x=0.0, target_y=2.25`（梅林第一排中心在 zone2_root 系下的坐标）。

#### 自动锁定 (`_try_auto_lock`)

**触发条件**: `AUTO_LOCK_ZONE2_ROOT=1` 且尚未锁定。

**所需帧数**: 受门控强度影响，
```
required = max(STABLE_LOCK_COUNT, int(STABLE_LOCK_COUNT / max(gate_strength, 0.1)))
```
观测区边缘 `gate_strength` 低 → 需更多帧 → 更保守。

**稳定性判据**（对 pending_locks 末尾 required 帧）:

| 检查项 | 阈值 |
|--------|------|
| 中心离散 spread | ≤ STABLE_CENTER_TOL (0.22m) |
| yaw 离散 yaw_spread | ≤ STABLE_YAW_TOL (10°) |

通过后: 取末尾 required 帧的位姿均值（yaw 用 `mean_yaw`），标记 `_zone2_root_locked=True`。

#### 入口精修 (`_try_refine`)

**触发条件**: 已锁定 + 精修启用 + 机器人在入口精修区内。

**入口区定义** (zone2_root 系):
- X: `|lx| ≤ REFINE_ENTRY_X_ABS_MAX (2.70m)`
- Y: `ly ∈ [REFINE_ENTRY_Y_MIN=1.45, REFINE_ENTRY_Y_MAX=3.70]`
- 朝向: `|norm_angle(lyaw - REFINE_FACING_YAW)| ≤ REFINE_FACING_YAW_TOL (75°)`
- 朝向参考: `REFINE_FACING_YAW = -π/2`（正对梅林）

**精修约束**:

| 约束 | 阈值 |
|------|------|
| 与锁定点的平移距离 | ≤ REFINE_MAX_TRANSLATION (0.65m) |
| 与锁定点的 yaw 差 | ≤ REFINE_MAX_YAW_DELTA (20°) |

**EMA 修正公式**:
```
effective_alpha = REFINE_ALPHA * gate_strength
new_x = locked_x + effective_alpha * (candidate_x - locked_x)
new_y = locked_y + effective_alpha * (candidate_y - locked_y)
new_yaw = norm_angle(locked_yaw + effective_alpha * norm_angle(candidate_yaw - locked_yaw))
```

**REFINE_ONCE=1**: 精修只执行一次，完成后 `_refine_done=True`，不再触发。

#### TF 定时器

锁定后，10Hz 定时器持续重复发布 `odom → {team}_zone2_root` TF，保持 transform 树活跃。

---

## 3. 全部配置参数

### 3.1 输入点云裁剪

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| X_MIN | 0.0 | m | 前向最小距离 (base_link) |
| X_MAX | 3.5 | m | 前向最大距离 |
| Y_MIN | -1.7 | m | 横向最小 (左) |
| Y_MAX | +1.7 | m | 横向最大 (右) |
| Z_MIN | -0.50 | m | 高度下限 |
| Z_MAX | +0.50 | m | 高度上限 |

### 3.2 高度与地面

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| HEIGHT_FRAME | "odom" | — | 高度归一参考坐标系 |
| GROUND_Z_KNOWN | 0 | — | 1=手动固定 |
| GROUND_Z | -0.310 | m | 手动地面高度 |
| GROUND_TOLERANCE | 0.035 | m | 地面点判定容差 |
| GROUND_PEAK_RATIO | 0.15 | — | 直方图主峰比例 |
| HISTOGRAM_BIN_WIDTH | 0.02 | m | 直方图 bin 宽 |
| GROUND_UPDATE_ALPHA | 0.18 | — | EMA 系数 |
| GROUND_MAX_UPDATE_STEP | 0.06 | m | 单帧最大跳变 |

### 3.3 前立面分层与 RANSAC

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| FACADE_SLICE_Z_MIN | 0.005 | m | 立面最低 h |
| FACADE_SLICE_Z_MAX | 0.400 | m | 立面最高 h |
| BAND_200_Z_MIN | 0.050 | m | 白线带下界 |
| BAND_200_Z_MAX | 0.200 | m | 白线带上界 |
| BAND_400_Z_MIN | 0.250 | m | 黄线带下界 |
| BAND_400_Z_MAX | 0.400 | m | 黄线带上界 |
| Y_BIN_SIZE | 0.05 | m | Y 直方图 bin |
| X_BIN_WIDTH | 0.05 | m | X 直方图 bin |
| MIN_DENSITY_PEAK | 5 | 点 | 有效 bin 最少点数 |
| RANSAC_RESIDUAL_THRESHOLD | 0.05 | m | 内点残差 |
| MIN_EDGE_POINTS | 5 | 点 | 最少边缘点 |
| RANSAC_N_ITER | 200 | 次 | 迭代次数 |
| MAX_ALLOWED_SLOPE | 0.6 | — | 斜率上限 |

### 3.4 墙面与柱判决

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| WALL_HALF_WIDTH | 0.10 | m | 墙面半宽 |
| COL_WIDTH | 0.05 | m | 柱宽 |
| Y_COLUMN_MIN | -2.5 | m | 分柱 Y 起点 |
| Y_COLUMN_MAX | +2.5 | m | 分柱 Y 终点 |
| DILATE_BINS | 2 | 格 | 黄柱膨胀格数 |

### 3.5 场地先验

| 参数 | 默认值 | 说明 |
|------|--------|------|
| IS_BLUE_TEAM | True | 队伍颜色 |
| ZONE2_TARGET_X | 0.0 | 梅林第一排中心 X (zone2_root 系) |
| ZONE2_TARGET_Y | 2.25 | 前表面 Y |
| ZONE2_ROOT_YAW_OFFSET | π/2 | 检测线 yaw → root yaw 偏置 |
| ZONE2_ROOT_Z | -0.270 | root Z (与 GROUND_Z 一致) |

### 3.6 锁定策略

| 参数 | 默认值 | 说明 |
|------|--------|------|
| AUTO_LOCK_ZONE2_ROOT | 1 | 启用自动锁定 |
| STABLE_LOCK_COUNT | 4 | 稳锁所需连续帧 |
| STABLE_CENTER_TOL | 0.22m | 候选中心最大离散 |
| STABLE_YAW_TOL | 10° | 候选 yaw 最大离散 |
| DYNAMIC_TF_RATE | 10Hz | TF 重发频率 |
| ENABLE_ENTRY_REFINEMENT | 1 | 启用入口精修 |
| REFINE_STABLE_COUNT | 3 | 精修稳锁帧数 |
| REFINE_CENTER_TOL | 0.16m | 精修中心容差 |
| REFINE_YAW_TOL | 8° | 精修 yaw 容差 |
| REFINE_MAX_TRANSLATION | 0.65m | 精修最大平移 |
| REFINE_MAX_YAW_DELTA | 20° | 精修最大 yaw 变化 |
| REFINE_ENTRY_X_ABS_MAX | 2.70m | 入口 X 范围 |
| REFINE_ENTRY_Y_MIN | 1.45m | 入口 Y 下限 |
| REFINE_ENTRY_Y_MAX | 3.70m | 入口 Y 上限 |
| REFINE_FACING_YAW | -π/2 | 正对梅林的 yaw |
| REFINE_FACING_YAW_TOL | 75° | 朝向容差 |
| REFINE_ALPHA | 0.65 | EMA 系数 |
| REFINE_ONCE | 1 | 仅精修一次 |

### 3.7 观测位姿门控

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| ENABLE_ODOM_GATE | 1 | — | 启用门控开关 |
| GATE_CENTER_X_DEFAULT | 1.600 | m | 观测中心 X (BLUE) |
| GATE_CENTER_Y_DEFAULT | 1.350 | m | 观测中心 Y (BLUE) |
| GATE_HALF_SIZE_X | 0.75 | m | 观测区 X 半宽 |
| GATE_HALF_SIZE_Y | 0.75 | m | 观测区 Y 半高 |
| GATE_STRENGTH_ATTENUATION | 0.5 | — | 边缘衰减系数 |
| GATE_YAW_TOLERANCE_DEG | 30 | ° | 朝向容差 |

### 3.8 合成可视化

| 参数 | 默认值 | 说明 |
|------|--------|------|
| SYNTHETIC_LINE_N | 500 | 合成白线采样点 |
| SYNTHETIC_CLUSTER_N | 100 | 合成红心簇点 |
| SYNTHETIC_CLUSTER_SPREAD | 0.015m | 红心高斯 σ |
| HEIGHT_BAND_1_MAX | 0.20m | 低带 (绿) 上限 |
| HEIGHT_BAND_2_MAX | 0.40m | 高带 (红) 上限 |

### 3.9 降采样与日志

| 参数 | 默认值 | 说明 |
|------|--------|------|
| DOWNSAMPLE_STEP | 5 | 降采样步长 |
| LOG_ENABLED | False | 终端日志开关 |
| LOG_INTERVAL | 1.0s | 日志最小间隔 |
| DETAILED_FILE_LOG | False | CSV 详细日志 |
| DETAIL_LOG_DIR | ~/files/record/logs | 日志目录 |

---

## 4. 接口

### 订阅

| 话题 | 类型 | QOS | 用途 |
|------|------|-----|------|
| `/odin1/cloud_slam` | PointCloud2 | 10 | Odin LiDAR 实时拼接点云 |
| `/odin1/odometry_highfreq` | Odometry | 20 | 机器人实时位姿（门控判断） |

### 发布

| 话题 | 类型 | 用途 |
|------|------|------|
| `/rc26/zone2/cloud_height_bands` | PointCloud2 | 高度染色调试: 地面(白)/低带(绿)/高带(红) |
| `/rc26/zone2/cloud_facade_recon` | PointCloud2 | 立面重建: 黄柱(黄)/白线(白)/红心(红) |

### TF

| Frame ID | 方向 | 含义 |
|----------|------|------|
| `{team}_zone2_root` | odom → child | 梅林导航坐标系原点, 锁定后 10Hz |

---

## 5. 生命周期

```
                    ┌─────────────────┐
                    │     启动        │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │  等待门控通过   │
                    │  (可无限等)     │
                    └────────┬────────┘
                             │ 通过
                    ┌────────▼────────┐
                    │  每帧检测立面   │──── 检测不到 ────► 跳过该帧,等下帧
                    └────────┬────────┘
                             │ 有候选
                    ┌────────▼────────┐
                    │  多帧稳定判断   │──── 不稳定 ────► 清空候选,重新累积
                    └────────┬────────┘
                             │ 稳定
                    ┌────────▼────────┐
                    │  自动锁定       │
                    │  10Hz TF 发布   │
                    └────────┬────────┘
                             │ 车移动到入口
                    ┌────────▼────────┐
                    │  入口精修       │
                    │  (REFINE_ONCE=1 │
                    │   则仅一次)     │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │  TF 保持稳定    │
                    │  (锁后不变)     │
                    └─────────────────┘
```

**各阶段行为**:
- 观测点未到达 → 无检测，节点空转等门控
- 立面检测不到 → 每帧尝试，持续到门控超时或参数调整
- 锁定后车移动 → 锁定 TF 保持不变（精修只在入口区触发）

---

## 6. 异常保护

| 保护点 | 位置 | 机制 |
|--------|------|------|
| TF 查询失败 | `_publish_zone2_root_tf` | try/except + 2s 节流 warn |
| RANSAC 内点不足 | `_fit_ransac` | raise ValueError → `_detect_facade` 捕获跳过 |
| 地面检测 TF 不可用 | `GroundEstimator.to_height_frame` | 返回 None → 跳过该帧 |
| 点云空数据 | `cloud_cb` | width==0 或 height==0 → return |
| 空数组 concat | `_build_recon_cloud` | 检查 parts_x 非空才 concat |
| `__del__` 半初始化 | `__del__` | `hasattr` 保护 |
| 门控未启用时 odom 不可用 | `_odom_gate_check` | ENABLE_ODOM_GATE=0 → 直接通过 |

---

## 7. 调试

### RViz 可视化

1. **`/rc26/zone2/cloud_height_bands`**: 检查地面估计是否正确。白色=地面点，绿色=低带(0~0.20m)，红色=高带(0.20~0.40m)。地面白色应紧密贴合实际地面。
2. **`/rc26/zone2/cloud_facade_recon`**: 检建立面拟合结果。黄色=检测到的黄柱点，白色=合成白线，红色=靶心。白色线应与黄柱边界对齐。

### 日志

- **终端日志** (`LOG_ENABLED=True`): 每 1s 输出靶心 Y、Yaw、白线范围、黄柱数
- **CSV 日志** (`DETAILED_FILE_LOG=True`): 事件日志含 candidate/root_candidate/lock/refine 事件

### 常见问题

| 现象 | 根因 | 排查 |
|------|------|------|
| 一直不锁 | 车不在观测区 / 地面参数不对 / 立面遮挡 | 检查 gate_strength 日志 + height_bands 地面染色 + facade_recon |
| 锁定抖动 | STABLE_LOCK_COUNT 太小 / STABLE_CENTER_TOL 太松 | 增大帧数 / 收紧容差 |
| 入口修正拉飞 | REFINE_MAX_TRANSLATION 太大 | 收紧到 0.40~0.50m |

---

## 8. 依赖

- **ROS2**: rclpy, tf2_ros, sensor_msgs, geometry_msgs, nav_msgs, std_msgs
- **Python**: numpy, scipy (ndimage.binary_dilation)
- **内部**: zone_detection.common (point_cloud_utils, tf_utils, ground_estimator, debug_logger)
