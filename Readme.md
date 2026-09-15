# rc26_zone3 — ROBOCON 2026 第三区九宫格定位

> 使用 Odin1 点云与里程计，识别第三区（Zone3）九宫格高台，动态发布 `{team}_zone3_root` TF，供下游放置、对齐、颜色/占用判断等动作使用。

---

## 一、数据源与依赖

- 传感器：**Odin1**（ROS2 驱动见独立工作空间 `27rc_ws_odin1`）
- 关键话题：
  - `/odin1/cloud_slam`：SLAM 点云（`frame_id` 通常为 `odom`）
  - `/odin1/odometry_highfreq`：高频里程计（上坡状态机、z/pitch 判定）
  - `/tf`、`/tf_static`：坐标变换
- 支持 bag 离线回放调试（`-r` 倍速、`--clock` 时间同步），无需真实设备。

---

## 二、目录结构

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
└── other/                      # 调试脚本（角点拟合、分析、标定等，非主链路）
```

---

## 三、实现思路

### 3.1 第三区九宫格定位（`zone3/localizer_node.py`）

`Zone3GridLocalizer` 主回调（订阅 `/odin1/cloud_slam`）：

1. **解析与降采样** → ROI 裁剪（按 odom/车体系取范围）。
2. **地面估计与高度归一**：`h = z - ground_z`，统一到 odom 高度系。
3. **高位筛选**：只保留九宫格高台结构点（`h ∈ [GRID_MIN_H, GRID_MAX_H]`）。
4. **多帧累积**：最近 `ACCUMULATE_FRAMES` 帧高位点累加，增强稀疏/遮挡下的稳定性。
5. **九宫格几何检测**（`grid_detect.detect_grid_pose`）：连通域分析 → PCA 主方向 → 按宽度/深度/层数/密度评分 → 得中心与朝向。
6. **锁定（LOCK）**：质量达标 + 多帧中心/朝向离散度达标 → 平均位姿锁定 `{team}_zone3_root`。
7. **Z2 先验锁**（可选）：进入重试区首帧，由 `zone2_root` TF + `Z2_TO_Z3_OFFSET` 推算初锁。
8. **精修（REFINE）**：锁定位姿上做小范围平移/朝向 EMA 收敛。
9. **发布**：10Hz 动态发布 `odom -> {team}_zone3_root`，并发布候选/模型点云与重试区 Marker。

### 3.2 上坡状态机与第三区角点（`rc/src/fence_locator`）

- `uphill_state_node` 读 `/odin1/odometry_highfreq`，区分平地/上坡/上平台阶段。
- 上坡触发后采集 `/odin1/cloud_slam` 转局部坐标系，按点云边界（bin 分组 + 分位数取外轮廓、拟合长边与端部边、两线求交）找关键角点，结合场地先验发布 `zone3_root` TF/Marker。

### 3.3 场地模型（`field_publisher/rc26_field.py`）

- 按图册/几何常量构建 Z1/Z2/Z3 三区域 TF 树与 MarkerArray（地毯、围栏、坡道、九宫格、重试区等）。
- 支持单队（蓝/红）与双队（红蓝，需 `PUBLISH_FIELD_ROOT`）模式。
- 坐标系：X+→蓝队，X-→红队，Y+→武馆区，Z+→上。
- 关键几何：Z3 坡道宽 1.55m、水平长 1.5m、抬升约 0.40m（约 15°）；九宫格 3 列 × 3 层。

---

## 四、运行方式

### 4.1 离线回放调试（bag）

```bash
ros2 launch launch/test.launch.py \
  bag_path:=<ROS2 bag 目录> \
  team_side:=BLUE \
  bag_rate:=3.0 \
  bag_offset:=0
```

- `bag_rate`：回放倍速（0.1 慢速 / 3.0 快速）；`team_side`：`BLUE`/`RED`/`0`/`1`
- 自动播放 bag、启场地模型、zone2/zone3/kfs 节点与 RViz。
- **注意**：`launch/test.launch.py` 与公共配置含本机硬编码路径（`/home/inkc/...`），迁移环境时需替换。

### 4.2 真实设备（Odin1）

先启动 Odin1 驱动（`27rc_ws_odin1/odin_start.sh`），使 `/odin1/cloud_slam`、`/odin1/odometry_highfreq` 与 TF 可用，再单独启动本仓库定位节点。

---

## 五、当前效果（已实现）

- RViz 可见：场地模型三区域、九宫格高台候选/模型点云、重试区 Marker、动态发布的 `{team}_zone3_root` TF。
- 九宫格九格中心、场地/坡道几何均有 TF/Marker 表达，可供下游放置与颜色判断。
- 可跑、可看、能粗略拟合到第三区场地位置。

---

## 六、Git 使用

每完成一个可运行的小阶段后手动提交：

```bash
cd /home/huya/rc26_zone3
git status                      # 查看修改
git add . && git commit -m "说明本次修改" && git push
git log --oneline --max-count=10
git diff                        # 查看未提交改动
```