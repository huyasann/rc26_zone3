# rc26_zone3 — ROBOCON 2026 第三区九宫格定位

> 使用 Odin1（留形科技空间感知模组）点云与里程计，稳定识别第三区（Zone3）的九宫格高台，并发布 `{team}_zone3_root` 相关 TF，供下游完成放置、对齐、颜色/占用判断等动作。

---

## 一、接手过程

本仓库是 `rc2026_snapshot` 项目在误删事故后的恢复产物。

- 原始工程从误删事故中恢复后，初始化本地 Git 仓库并推送至 GitHub 私有仓库 `huyasann/rc26_zone3`。
- `bag/`、`build/`、`install/`、`log/` 等大体积/缓存目录已通过 `.gitignore` 排除，不进入版本库。
- 当前可编译运行的 ROS2 包：
  - `zone_detection`（核心定位）
  - `rc/src/uphill`（上坡状态机）
  - `rc/src/fence_locator`（第三区围栏/角点检测）

---

## 二、数据源与依赖

- 传感器：**Odin1**（ROS2 驱动，见独立工作空间 `27rc_ws_odin1`）
- 关键话题：
  - `/odin1/cloud_slam`：`sensor_msgs/msg/PointCloud2`，SLAM 点云（frame_id 通常为 `odom`）
  - `/odin1/odometry_highfreq`：`nav_msgs/msg/Odometry`，高频里程计（用于上坡状态机、z/pitch 判定）
  - `/tf`、`/tf_static`：坐标变换
- 运行时可用 bag 回放（`-r` 倍速、`--clock` 时间同步）替代真实设备，便于离线调试。

---

## 三、目录结构

```
rc26_zone3/
├── launch/test.launch.py       # bag 回放 + 场地模型 + zone2/zone3/kfs 节点 + RViz
├── configs/challenge1_common.yaml   # 队伍侧/相机外参等公共配置
├── zone_detection/             # ROS2 Python 包（核心定位）
│   └── zone_detection/
│       ├── common/             # 点云解析、TF、地面估计、调试日志
│       ├── field_publisher/    # rc26_field.py — 三区域 TF 树 + 场地 Marker
│       ├── zone2/              # 二区梅林定位
│       ├── zone3/              # 三区九宫格定位（核心）
│       │   ├── localizer_node.py   # Zone3GridLocalizer 主节点
│       │   ├── grid_detect.py      # 九宫格几何检测
│       │   └── config.py           # 参数
│       └── kfs_grid/           # 九宫格 KFS（五色石）颜色/占用检测
├── rc/                         # 上坡状态机、围栏/角点定位等
├── configs/                    # 公共配置
└── other/                      # 调试脚本（角点拟合、分析、标定等，非主链路）
```

---

## 四、实现思路

### 4.1 第三区九宫格定位（`zone3/localizer_node.py`）

核心流程（`Zone3GridLocalizer` 主回调，`/odin1/cloud_slam`）：

1. **点云解析与降采样**：解析 x/y/z，按步长降采样。
2. **ROI 裁剪**：按帧类型（odom / 车体系）选取不同范围，排除无关结构。
3. **地面估计与高度归一**：用 `GroundEstimator` 得到 `h = z - ground_z`，统一到 odom 高度系。
4. **高位筛选**：只保留九宫格高台结构点（`h ∈ [GRID_MIN_H, GRID_MAX_H]`）。
5. **多帧累积**：最近 `ACCUMULATE_FRAMES` 帧高位点累加，增强稀疏/遮挡下的稳定性。
6. **九宫格几何检测**（`grid_detect.detect_grid_pose`）：连通域分析 → PCA 估计主方向 → 按宽度/深度/层数/密度评分 → 得九宫格中心与朝向。
7. **锁定（LOCK）**：候选质量达标（confidence/层数/点数/尺寸）+ 多帧中心与朝向离散度达标 → 取平均位姿锁定 `{team}_zone3_root`。
8. **Z2 先验锁**（可选，`enable_z2_prior`）：进入重试区首帧，由 `zone2_root` TF + 固定偏移 `Z2_TO_Z3_OFFSET` 推算初锁。
9. **精修（REFINE）**：在锁定位姿基础上做小范围平移/朝向 EMA 收敛。
10. **发布 TF**：以 10Hz 动态发布 `odom -> {team}_zone3_root`，并发布候选/模型点云与重试区 Marker 便于调试。

### 4.2 上坡状态机与第三区角点（`rc/src/fence_locator`）

- `uphill_state_node` 读取 `/odin1/odometry_highfreq`，状态机区分平地/上坡/上平台阶段。
- 上坡触发后采集 `/odin1/cloud_slam`，转换到局部坐标系。
- 按点云边界寻找第三区关键角点（bin 分组 + 分位数取外轮廓边界、拟合长边与端部边、两线求交得角点），结合场地模型先验发布 `zone3_root` 相关 TF/Marker。

### 4.3 场地模型（`field_publisher/rc26_field.py`）

- 按图册/几何常量构建 Z1/Z2/Z3 三区域 TF 树与 MarkerArray（地毯、围栏、坡道、九宫格、重试区等）。
- 支持单队（蓝/红）与双队（红蓝，需 `PUBLISH_FIELD_ROOT`）模式。
- 坐标系约定：X+→蓝队，X-→红队，Y+→武馆区，Z+→上。
- 关键几何：Z3 坡道宽 1.55m、水平长 1.5m、抬升约 0.40m（约 15°）；九宫格 3 列 × 3 层。

---

## 五、运行方式

### 5.1 离线回放调试（bag）

```bash
# 依赖 ros2, 已配置 zone_detection 等包的环境
ros2 launch launch/test.launch.py \
  bag_path:=<ROS2 bag 目录> \
  team_side:=BLUE \
  bag_rate:=3.0 \
  bag_offset:=0
```

- `bag_rate`：回放倍速（0.1 慢速、3.0 快速）
- `team_side`：`BLUE`/`RED`/`0`/`1`
- 会自动播放 bag（`--clock 100` 时间同步）、启场地模型、zone2/zone3/kfs 节点与 RViz。
- **注意**：`launch/test.launch.py` 与公共配置路径含本机硬编码路径（`/home/inkc/...`），迁移环境时需替换。

### 5.2 真实设备（Odin1）

先启动 Odin1 驱动（见 `27rc_ws_odin1/odin_start.sh`），让 `/odin1/cloud_slam`、`/odin1/odometry_highfreq` 与 TF 可用，再单独启动本仓库的定位节点（`zone3_localizer` 等）。

---

## 六、当前效果与局限

### 已实现（基线可用）
- RViz 中可见：场地模型三区域、九宫格高台候选/模型点云、重试区 Marker、动态发布的 `{team}_zone3_root` TF。
- 九宫格九格中心、场地/坡道几何均有 TF/Marker 表达，可供下游放置与颜色判断。
- 可跑、可看、能粗略拟合到第三区场地位置。

### 已知局限（改进方向）
1. **角点/九宫格偏粗**：当前 PCA/粗几何方案对稀疏、遮挡、外点敏感，精度不够稳定。
2. **端部边不稳定**：点云不完整时端部边拟合易漂移；待补全"两条边/两个竖直面 RANSAC 精拟合"。
3. **高度筛选偏粗**：仅适合作初筛，不能直接决定最终位置。
4. **Z2 先验触发**：默认 `require_zone3_odom_gate:=false` 时 Z2 先验实际不触发，zone3 主要走纯点云检测；是否启用 Z2 先验需结合比赛流程确认。
5. **`setup.py` 无效入口**：注册了 `zone3_platform_fitter` 但源码无 `zone3_fit` 包，运行该入口会失败。
6. **launch 路径硬编码**：含 `/home/inkc/...` 绝对路径，跨环境需参数化。

### 建议的改进主线
- 优先提升第三区关键角点精度：以粗角点为初值，在其附近小 ROI 内用两条近似垂直边的 RANSAC 精拟合求交角点，并建立评分机制（内点数、RMSE、夹角误差、竖直点校验等）后才发布最终 `zone3_root`。
- 可改用"小 ROI + 九宫格 3×3 模型评分"替代全场大 ROI 粗检，增强对遮挡的鲁棒性。

---

## 七、Git 使用

仓库约定：每完成一个可运行的小阶段后手动提交备份。

```bash
cd /home/huya/rc26_zone3
git status                      # 查看修改
git add . && git commit -m "说明本次修改" && git push
git log --oneline --max-count=10
git diff                        # 查看未提交改动
```

> 注：旧版 Readme 为逐日接手/试错记录（含 Windows 路径与过程日志）。本版已重写为面向项目功能与交接的说明文档。