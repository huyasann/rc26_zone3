# zone_detection

梅林前立面检测 (Z2) + 九宫格检测 (Z3) + 九宫格颜色检测 (KFS Grid) 点云定位节点。从 `script/kfs_grid_qt.py` 等重构，共享工具抽离为共用模块，按区划分子包。

**全自动运行** — 无需外部 Service 或手动干预。

---

## 包结构

```
zone_detection/
├── package.xml
├── setup.py                      # entry: zone2_detector, zone3_localizer, kfs_grid_detector
├── setup.cfg
├── README.md
└── zone_detection/
    ├── __init__.py
    ├── common/                   # 共享工具 (无 ROS 依赖的纯函数)
    │   ├── __init__.py
    │   ├── point_cloud_utils.py  #   parse_xyz / make_rgb_cloud / empty_cloud
    │   ├── tf_utils.py           #   quat↔rpy / norm_angle / mean_yaw / blend_yaw
    │   ├── ground_estimator.py   #   GroundEstimator 类 (高度投影 + 地面检测)
    │   └── debug_logger.py       #   DebugLogger / Zone3CsvLogger / HeightDiagLogger
    ├── zone2/                    # 梅林前立面检测
    │   ├── __init__.py
    │   ├── config.py             #   全部可调参数 + 详细中文注释
    │   ├── detector_node.py      #   Zone2DetectorNode (全自动)
    │   └── zone2.py              #   (原始文件参考)
    ├── zone3/                    # 九宫格检测
    │   ├── __init__.py
    │   ├── config.py             #   全部可调参数 + 详细中文注释
    │   ├── grid_detect.py        #   检测算法纯函数 (连通域/PCA/评分/位姿)
    │   ├── localizer_node.py     #   Zone3GridLocalizer (全自动)
    │   └── zone3.py              #   (原始文件参考)
    └── kfs_grid/                 # 九宫格颜色检测 (原 kfs_grid_qt.py)
        ├── __init__.py
        ├── config.py             #   全部可调参数
        ├── color_scorer.py       #   色彩纯度权重算法 (纯函数, 零 ROS 依赖)
        ├── detector_node.py      #   KfsGridDetectorNode (全自动)
        └── qt_window.py          #   Qt 调试窗口 (ENABLE_QT_GUI=True 时加载)
```

---

## 自动运行策略

| 节点 | 启动检测 | 锁定时机 | 失败行为 |
|------|----------|----------|----------|
| zone2 | odom 观测位姿门控通过后自动开始处理点云 | 多帧稳定后自动锁, 入口区自动精修 | 3 类 try/except: TF 查询、RANSAC、点云解析 | 
| zone3 | 重试区门控 (位置+朝向) 通过后自动开始 | Z2 先验初锁 + 多帧稳定锁 + 精修 | 4 类 try/except: TF 查询、连通域、地面估计、日志 |
| kfs_grid | 等待 zone3 TF 锁定后自动开始 | zone3 TF 就绪即开始统计, 累积 N 帧后持续输出 | 2 类 try/except: TF 查询、点云解析 |

**BT 侧只需消费 TF 结果：**
- `z2.zone2_ready` → 等价于 `tf_buffer.canTransform("odom", "{team}_zone2_root")`
- `z2.heading_zero_yaw` → 从 `{team}_zone2_root` TF 的 rotation 提取 yaw
- `z3.zone3_ready` → 等价于 `tf_buffer.canTransform("odom", "{team}_zone3_root")`

**kfs_grid 输出** (供后续决策消费):
- `/rc26/zone3/cell_markers` → RViz 显示 9 格半透明方块
- 终端日志输出每格 R/B 分数和全局主导色
- `kfs_grid.zone3_ready` → 等价于 zone3 TF 已锁定 + 累积完成

---

## 健壮性设计

### 异常保护

| 保护点 | 位置 | 机制 |
|--------|------|------|
| TF 查询失败 | zone2 `_publish_zone2_root_tf`, zone3 `_grid_pose_to_odom` / `_get_robot_pose` | try/except + 日志节流 (1~2s) |
| RANSAC 内点不足 | zone2 `_fit_ransac` | raise ValueError → `_detect_facade` 捕获 → 跳过该帧 |
| 地面检测 TF 不可用 | `GroundEstimator.to_height_frame` | 返回 None → 跳过该帧 |
| 点云空数据 | zone2/cloud_cb, zone3/cloud_cb | `width==0` 或 `height==0` 直接 return |
| 空数组拼接 | `_build_recon_cloud` | 检查 parts_x 非空才 concat |
| `__del__` 半初始化 | zone2/zone3 | `hasattr` 保护, 防止 init 中途失败时二次崩溃 |
| Z2 先验 TF 空转 | zone3 `_try_z2_prior_lock` | 1s 冷却避免每帧查 TF |

### 点云维度校验 (修复记录)

**IndexError: boolean index dimension mismatch** — 原重构中 `_detect_facade` 将柱级统计数组 (`is_white` / `is_yellow`, 长度 ~100 柱) 直接传给消费函数，函数内用它索引点级数组 (`y_rot`, 长度 ~3000 点) 导致维度不匹配。

修复：在 `_detect_facade` 中添加柱级→点级映射（参考原 zone2.py L502-508），`_locate_target_and_publish` 只接收点级 bool 数组。

---

## 启动

```bash
# zone2 - 梅林前立面检测
ros2 run zone_detection zone2_detector --ros-args -p is_blue_team:=true

# zone3 - 九宫格检测
ros2 run zone_detection zone3_localizer --ros-args -p is_blue_team:=true

# kfs_grid - 九宫格颜色检测
ros2 run zone_detection kfs_grid_detector
```

### Launch 文件集成

`src/action/bringup/launch/challenge1/test.launch.py` 已更新为直接调用 Node，不再执行原 tmp 脚本：

```python
action_zone2 = Node(
    package='zone_detection',
    executable='zone2_detector',
    parameters=[{'use_sim_time': True, 'is_blue_team': ...}])

action_zone3 = Node(
    package='zone_detection',
    executable='zone3_localizer',
    parameters=[{'use_sim_time': True, 'is_blue_team': ...}])
```

---

## Zone2DetectorNode — 梅林前立面定位

**目标**: 从 LiDAR 点云检测梅林台阶阵列的前立面，输出 `odom→{team}_zone2_root` TF。

### 接口

| 接口 | 方向 | 类型 | 作用 |
|------|------|------|------|
| **订阅** | | | |
| `/odin1/cloud_slam` | 订阅 | PointCloud2 | Odin LiDAR 拼接点云 |
| `/odin1/odometry_highfreq` | 订阅 | Odometry | 机器人位姿 (门控/精修判断) |
| **发布** | | | |
| `/rc26/zone2/cloud_height_bands` | 发布 | PointCloud2 | 地面(白)/低带(绿)/高带(红) 染色调试 |
| `/rc26/zone2/cloud_facade_recon` | 发布 | PointCloud2 | 黄柱(黄)+白线(白)+红心(红) 重建点云 |
| `odom → {team}_zone2_root` | TF | — | 梅林网格导航系原点, 锁定后 10Hz 持续发 |

### 处理流程

```
/odin1/cloud_slam
  ↓
┌─ 观测位姿门控 (odom_gate_check)
│  需在 Z2 观测矩形内 + 车头朝前 ±30°。越靠近中心 gate_strength→1.0
│  (影响锁定灵敏度), 否则跳过本帧
│
├─ ROI: X=0~3.5, Y=±1.7, Z=±0.5 (base_link 系)
├─ 地面检测 (直方图 / 手动 ground_z=-0.270)
├─ 离地高度 h = z_odom - ground_z
│
├─ 前立面切片: h∈[0.005, 0.400]
│  → 200mm 白线带 (0.05~0.20) / 400mm 黄线带 (0.25~0.40)
├─ 2D 直方图 (Y×X) → 每行最近 X bin → 边缘点集
├─ RANSAC x = k*y + b (|k|≤0.6, 防深度误检) → 立面 yaw + 基准线
│  └─ 内点不足时跳过该帧 (不阻塞)
│
├─ 墙面点筛选: 距线 ±0.10m 内
├─ Y 方向分柱: 黄柱 (200+400 都密) / 白柱 (仅 200 密, 黄色膨胀 2bin 屏蔽)
├─ 柱级→点级映射: 将柱统计映射回点云索引
├─ 靶心定位: 白 Y 中心 ± 黄柱边界约束
├─ 合成白线 + 红心高斯簇 → /rc26/zone2/cloud_facade_recon
│
├─ [自动锁定] facade_pose → x/y/yaw → 多帧稳定后锁
│  到入口区触发精修 (EMA, 限 REFINE_MAX_TRANSLATION)
│
└─ 锁定后 10Hz 持续发布 odom → {team}_zone2_root
```

### 生命周期

```
启动 → 等待门控通过 → 开始检测立面 → 候选稳定 → 锁定 zone2_root
                                          ↓
                                    (车移动到入口)
                                          ↓
                                   触发精修 (EMA 修正)
```

- **观测点未到达** → 无检测，node 空转等门控
- **立面检测不到** → 每帧尝试，持续到门控超时 或 参数调整
- **锁定后车移动** → 锁定 TF 保持不变 (精修只在入口区触发)

---

## Zone3GridLocalizer — 九宫格定位

**目标**: 在重试区内检测 3×3 九宫格，输出 `odom→{team}_zone3_root` TF。

### 接口

| 接口 | 方向 | 类型 | 作用 |
|------|------|------|------|
| **订阅** | | | |
| `/odin1/cloud_slam` | 订阅 | PointCloud2 | Odin LiDAR 拼接点云 |
| **发布** | | | |
| `/rc26/zone3/cloud_grid_candidates` | 发布 | PointCloud2 | 高位候选(橙)+地面(白), 调试 |
| `/rc26/zone3/cloud_grid_model` | 发布 | PointCloud2 | 九宫格骨架线框 (黄), 与实测比对 |
| `/rc26/zone3/cloud_height_bands` | 发布 | PointCloud2 | 9 段高度染色, ground_z/层高标定 |
| `/rc26/zone3/retry_area_marker` | 发布 | Marker | 重试区绿色半透明方块, RViz |
| `odom → {team}_zone3_root` | TF | — | Z3 导航系原点, 10Hz |

### 处理流程

```
/odin1/cloud_slam
  ↓
├─ ROI (base_link: X=0.15~5.5 / odom: X=0~16)
├─ 地面检测 (手动 ground_z=-0.270)
│
├─ ── 重试区门控 (两阶段) ──
│  位置: 机器人在重试区 (±0.8m)
│  朝向: 机器人 yaw 对准九宫格 (±45°)
│  不通过 → 清空累积帧, 跳过
│
├─ 高位筛选: h∈[0.75, 2.60], 多帧累积 (20 帧)
├─ [自动锁定通道]
│  连通域分析 → PCA 主朝向 yaw
│  几何评分: width/depth/layers/density → 多帧稳定锁
│  └─ 评分不足、宽度/深度超范围 → 跳过该帧
│
├─ [Z2 先验通道] (仅首次进入重试区, TF 查询带 1s 冷却)
│  读 zone2 TF + 偏移 (0, -5.15) → 初锁
│  到达后小范围精修 (±0.40m, ±8°, EMA)
│
└─ 锁定后 10Hz 发布 odom → {team}_zone3_root
```

### 生命周期

```
启动 → 等待进入重试区 → Z2 先验锁 → 多帧稳定检 → 精修 → zone3_root 稳定
                                ↓
                          (Z2 TF 未就绪时 1s 冷却重试)
```

- **未进重试区** → 无检测，门控跳过所有帧
- **九宫格检不出** → 持续尝试，每帧评估
- **Z2 TF 未就绪** → 1s 冷却重试，不空转

---

## KfsGridDetectorNode — 九宫格颜色检测

**目标**: 等待 zone3 TF 锁定后, 统计九宫格 9 个格子内红/蓝点云净胜分并分类 (RED/BLUE/EMPTY/UNKNOWN), 发布 Marker 可视化。

**原脚本**: `script/kfs_grid_qt.py` (包含 PyQt5 GUI 窗口), 已剥离 Qt 依赖重构为纯 ROS2 节点。

### 接口

| 接口 | 方向 | 类型 | 作用 |
|------|------|------|------|
| **订阅** | | | |
| `/odin1/cloud_slam` | 订阅 | PointCloud2 (含RGB) | Odin LiDAR 拼接点云 |
| **发布** | | | |
| `/rc26/zone3/cell_markers` | 发布 | MarkerArray | 9 格半透明彩色方块 (底蓝/中绿/顶红) |
| **TF 查询** | | | |
| `odom → {team}_zone3_root` | 查询 | — | 等待 zone3_localizer 锁定 |

### 处理流程

```
/odin1/cloud_slam
  ↓
┌─ 解析点云 (x/y/z/r/g/b)
│
├─ 查 zone3 TF (blue/red_zone3_root)
│  未就绪 → 跳过本帧
│
├─ 点云 → odom 系 (TF 变换)
├─ 计算 grid_center 在 odom 中的位姿
│  gx = tf_x + cos*(gdx) - sin*(gdy)
│  gy = tf_y + sin*(gdx) + cos*(gdy)
│  蓝方: gdx=-3.025, 红方: gdx=+3.025, gdy=-0.15 (固定场地偏移)
│
├─ 投影到 grid-local 系
│  lx = cos*(px-gx) + sin*(py-gy)    # 深度
│  ly = -sin*(px-gx) + cos*(py-gy)   # 宽度
│  h = pz - GROUND_Z                 # 离地高度
│
├─ ROI 预筛: |lx|≤2.0, |ly|≤1.08, h∈[0.5,2.8]
├─ 多帧累积 (20 帧): 将 ROI 内点 (h,lx,ly,r,g,b) 缓存
│
├─ 🚀 色彩纯度净胜分 (向量化)
│  red_diff = max(0, R - max(G,B))
│  w_red = (red_diff/255) * (R/255)       # 高纯度红色权重 [0,1]
│  blue_diff = max(0, B - max(R,G))
│  w_blue = (blue_diff/255) * (B/255)     # 高纯度蓝色权重 [0,1]
│  → 亚克力青色(0,200,200): B-max(R,G)=0 → 自动归零
│
├─ 逐格统计 (3层×3列=9格)
│  每格检测框: |lx|≤0.60, |ly-yc|≤0.27, |h-zc|≤0.27
│  red_score = sum(w_red[idx]), blue_score = sum(w_blue[idx])
│
├─ 分类 (RED/BLUE/EMPTY/UNKNOWN)
│  EMPTY:  total < 15
│  RED:    red > blue AND red/blue ≥ 1.5 AND red ≥ max(20, total*3%)
│  BLUE:   blue > red AND blue/red ≥ 1.5 AND blue ≥ max(20, total*3%)
│  UNKNOWN: 其他
│
├─ 全局主导色汇总
│  RED:   total_red/total_blue ≥ 1.3
│  BLUE:  total_blue/total_red ≥ 1.3
│  MIXED: 差距不足
│
├─ 终端日志 (2s 节流)
│  帧=Nk ROI=n 格=m | R=red_score B=blue_score 🔴/🔵/⚪ [主导色]
│
└─ Marker 发布 (0.5s 节流)
   9 格 CUBE, 颜色按层分: 底蓝/中绿/顶红, 透明度 0.25
```

### 色彩纯度权重 vs HSV 硬阈值

| 方案 | 优势 | 劣势 |
|------|------|------|
| HSV 硬阈值 | 直观易懂 | 光照变化敏感, 阈值难调, 亚克力反光误判 |
| **色彩纯度权重** (现方案) | 自适应亮度, 青色自动归零, 无硬阈值 | 需合理设置 dominant_ratio |

### 生命周期

```
启动 → 等待 zone3 TF 锁定 → 累积 20 帧 → 统计+分类 → 持续输出
                                                       ↓
                                              (zone3 TF 丢失则暂停)
```

- **zone3 未锁** → 空转等 TF
- **累积不足 20 帧** → 每帧缓存, 不发统计
- **累积完成** → 每 20 帧合并统计一次, 然后清空重新累积

### 启动

```bash
ros2 run zone_detection kfs_grid_detector --ros-args -p accumulate_frames:=20
```

---

## 配置调参指引

### zone2/config.py

| 分组 | 关键参数 | 调参场景 |
|------|----------|----------|
| ROI 裁剪 | `X_MIN/MAX`, `Y_MIN/MAX` | 检测不到立面 → 放宽范围 |
| 地面 | `GROUND_Z`, `GROUND_TOLERANCE` | 白/黄线高度漂移 → 标定 ground_z |
| 立面拟合 | `BAND_200_Z_*`, `BAND_400_Z_*`, `RANSAC_*` | 白/黄线分类不准 → 调高度带边界 |
| 柱判决 | `DILATE_BINS`, `WALL_HALF_WIDTH` | 黄白误判 → 调整膨胀/墙面宽度 |
| 锁定 | `STABLE_LOCK_COUNT`, `STABLE_CENTER_TOL` | 锁定抖动 → 增加帧数/收紧容差 |
| 精修 | `REFINE_*`, `REFINE_ENTRY_*` | 入口修正拉飞 → 收紧 max_translation |
| 门控 | `GATE_*`, `ENABLE_ODOM_GATE` | Z1 误检 → 开启或收紧门控 |

### zone3/config.py

| 分组 | 关键参数 | 调参场景 |
|------|----------|----------|
| ROI | `X_MIN/MAX`, `ODOM_*` | 检不到九宫格 → 放宽范围 |
| 高位 | `GRID_MIN_H`, `GRID_MAX_H` | 候选太少/误检 → 调高度区间 |
| 评分 | `MIN_LOCK_CONFIDENCE`, 评分系数 | 误检多 → 提高门槛 |
| 锁定 | `STABLE_LOCK_COUNT`, `STABLE_CENTER_TOL` | 抖动 → 增加帧数/收紧 |
| 门控 | `RETRY_AREA_HALF_SIZE`, `RETRY_FACE_YAW_TOL` | 提前/延迟检 → 调区域和朝向容差 |
| Z2 先验 | `Z2_TO_Z3_OFFSET_X/Y` | 初锁偏了 → 标定偏移量 |

### kfs_grid/config.py

| 分组 | 关键参数 | 调参场景 |
|------|----------|----------|
| 检测框 | `EXPAND_X/Y/Z` | 格子漏检/误检 → 调整检测框大小 |
| 颜色阈值 | `EMPTY_THRESHOLD`, `DOMINANT_RATIO`, `MIN_VALID_SCORE_*` | 红蓝误判 → 调整判定门槛 |
| 累积 | `ACCUMULATE_FRAMES` | 统计不稳定 → 增加累积帧数 |
| 场地 | `BLUE_GRID_CENTER_X`, `GRID_CENTER_Y` | 格子偏移 → 标定场地偏移量 |

---

## 常见问题排查

| 现象 | 可能原因 | 排查步骤 |
|------|----------|----------|
| zone2 一直不锁 | ① 车不在观测区 ② 地面参数不对 ③ 立面遮挡 | ① 检查 `_odom_gate_check` 日志 ② RViz 看 `/rc26/zone2/cloud_height_bands` 地面染色 ③ 看 `cloud_facade_recon` 显示 |
| zone2 锁定抖动 | `STABLE_LOCK_COUNT` 太小或 `STABLE_CENTER_TOL` 太松 | 增大帧数或收紧容差 |
| zone2 入口修正拉飞 | `REFINE_MAX_TRANSLATION` 太大 | 收紧到 0.40~0.50m |
| zone3 检测不到 | ① 未进重试区 ② `GROUND_Z` 偏了 ③ 九宫格被遮挡 | ① 检查 `_gate_logged` 日志 ② RViz 看高度染色 ③ 检查 confidence 输出 |
| zone3 Z2 先验锁不上 | ① zone2 还没锁 ② `Z2_TO_Z3_OFFSET` 不对 | ① 等 zone2 锁定 ② 检查 offset 值 |
| kfs_grid 无输出 | ① zone3 还没锁 ② 累积未完成 | ① 等 zone3 锁定 ② 检查 accumulate_frames |
| kfs_grid 红蓝误判 | `DOMINANT_RATIO` 或 `EXPAND_*` 不对 | 调整判定门槛或检测框大小 |

---

## 依赖

- ROS2 Humble: `rclpy`, `tf2_ros`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `visualization_msgs`
- Python: `numpy`, `scipy` (binary_dilation)

---

## 📋 文档-代码一致性

> 本文档覆盖以下源文件。若 SHA256 全部匹配，文档可信，无需全量读代码；任一不匹配则需交叉验证。

| 源文件 | SHA256 |
|--------|--------|
| `zone_detection/zone2/detector_node.py` | `b09711aab8b3088dd8a9363dc615a19d66896f036a1e7e3a8299a65c4aadf8bd` |
| `zone_detection/zone2/config.py` | `9cccbbe6dab2898a6842d0c56b6815c1943927b3c62d66b0ef687f32e51ceada` |
| `zone_detection/zone3/localizer_node.py` | `e9baa949777dce1a130cffb80c86c14096de19b7d0feb1677dc5fdacff432596` |
| `zone_detection/zone3/config.py` | `d1280b3f535e1e10985b9afd6b0913e3b91a8fc3dbe81f002de6029983466060` |
| `zone_detection/zone3/grid_detect.py` | `f9173466a90207d5224eab2a18787d844a514376e01372a5e68f3e36ddbef3d2` |
| `zone_detection/common/point_cloud_utils.py` | `a56327077fe3ad125a529d872f9ca5b4d8b2fd0ba904d706a6ec57a1a1a9b906` |
| `zone_detection/common/ground_estimator.py` | `b8dd72e0dc3c49f3e61471e51b59ea08ac9b9f37025e3b26fe3cc7ba26be5eaa` |
| `zone_detection/common/debug_logger.py` | `334909193ab0cccbc221a529ad7e37ded3d5637f9c719aab475858207fb207d2` |
| `zone_detection/kfs_grid/__init__.py` | `68e89e002e9845ce095f68fbf575182e000423ed4eaa99e34e2f712c6dc1c65e` |
| `zone_detection/kfs_grid/config.py` | `7b7b366abae2ad81eee12230c6ab79b4dbf22bdfa8b3865279a5e131234b383a` |
| `zone_detection/kfs_grid/color_scorer.py` | `92e315a61e6c36842eec8c72d2b017bb33c4419ba2e4e18227b4b9dae1f4b9e7` |
| `zone_detection/kfs_grid/detector_node.py` | `64992a4809956ca899120f042c397e69d7c35ba33916246ac97a1e20592fe3b0` |
| `zone_detection/kfs_grid/qt_window.py` | `7a8464f96513127d23ed757dfc325267c814274868f88a75333435fff42280f7` |
| `docs/zone2.md` | `700a1372723267c35093da2da6e51e4b20dd5757226803319379826f1b42177e` |
| `docs/zone3.md` | `9fe3041cd2e0327dc17d25a76dee573e52e3e1906bb0805cd50636649dccd0e6` |
| `docs/kfs_grid.md` | `e6f94708495ce4bb2c52180e80a0f5a6098150706d9b34522c05597a35749737` |

> **校验**: `sha256sum <file>` 对比上表。修改源文件后请重新计算并更新 hash。
