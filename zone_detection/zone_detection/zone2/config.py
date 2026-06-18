"""Zone2 梅林前立面检测 — 可调参数。

所有常量按功能分组，便于现场标定调参。
每个参数一行，行末附带详细中文注释。

调参顺序:
  1. 检测不到立面 → 先放松 ROI (X_MIN~X_MAX) 或 MIN_DENSITY_PEAK
  2. 立面抖动/误检 → 收紧 STABLE_CENTER_TOL / STABLE_YAW_TOL, 或增加 STABLE_LOCK_COUNT
  3. 白线/黄柱判决不准 → 调整 BAND_200_Z_MIN~MAX / BAND_400_Z_MIN~MAX 或 DILATE_BINS
  4. TF 对不上场地模型 → 检查 IS_BLUE_TEAM 和 ZONE2_TARGET_X/Y
"""

import math

# ════════════════════════════════════════════════════════════
# 1. 输入点云裁剪 (odin1_base_link 系)
# ════════════════════════════════════════════════════════════
# 原始点云来自 Odin SLAM, 先做空间 ROI 过滤, 避免退缩区和武馆区干扰。
# X=车前方向, Y=车左方向, Z=垂直向上 (base_link 系)

X_MIN, X_MAX = 0.0, 3.5         # 前向范围 (m): 车头起~3.5m 涵盖梅林第一排
Y_MIN, Y_MAX = -1.7, 1.7        # 横向范围 (m): 覆盖梅林宽度 ±1.7m
Z_MIN, Z_MAX = -0.50, 0.50      # 高度范围 (m): 仅保留接近地面的薄片, 排除天花板

# ════════════════════════════════════════════════════════════
# 2. 高度计算参考系
# ════════════════════════════════════════════════════════════
# 投影到 odom 的 Z 轴做物理高度归一, 消除车体俯仰/侧倾影响。
HEIGHT_FRAME = "odom"            # 高度归一目标坐标系 (所有点云的 Z 投影到此系)

# ════════════════════════════════════════════════════════════
# 3. 地面估计
# ════════════════════════════════════════════════════════════
# 地面高度用于计算离地高度 h, 影响白线/黄柱高度带判别。
# 赛场地面平整, 建议手动固定 GROUND_Z, 避免自动检测波动。

GROUND_Z_KNOWN = 0               # 1=手动固定 ground_z, 0=首帧自动估计+EMA更新
GROUND_Z = -0.1500               # 手动地面高度 (m), odom 系 Z。GROUND_Z_KNOWN=1 时生效
GROUND_TOLERANCE = 0.035         # 地面点判定容差 (m): |h|≤此值视为地面
GROUND_PEAK_RATIO = 0.15         # 自动检测时, 从低到高扫到主峰此比例即认作地面
HISTOGRAM_BIN_WIDTH = 0.02       # 直方图 bin 宽度 (m)
GROUND_UPDATE_ALPHA = 0.18       # 自动更新 EMA 系数 (越大跟随越快, 越小越稳)
GROUND_MAX_UPDATE_STEP = 0.06    # 自动更新单帧最大跳变 (m), 防误检拉飞

# ════════════════════════════════════════════════════════════
# 4. 高度颜色带 (调试可视化, 仅用于 RViz 染色, 不参与拟合决策)
# ════════════════════════════════════════════════════════════
HEIGHT_BAND_1_MAX = 0.20         # 低带上限 (m), 染绿色 (0.00~0.20m)
HEIGHT_BAND_2_MAX = 0.40         # 高带上限 (m), 染红色 (0.20~0.40m)

# ════════════════════════════════════════════════════════════
# 5. 队伍与 TF 命名
# ════════════════════════════════════════════════════════════
# 红蓝队共用同份代码, 运行时参数 is_blue_team 控制镜像。
IS_BLUE_TEAM = True              # 代码默认值, 运行时由 ROS 参数 is_blue_team 覆盖

ZONE2_ROOT_FRAME = ("blue_" if IS_BLUE_TEAM else "red_") + "zone2_root"  # TF child frame 名

# 场地先验: 第一排中间 200mm 块的前表面中心 (rc26_field.py 模型)
ZONE2_TARGET_X = 0.0             # 梅林第一排中心 X (zone2_root 系)
ZONE2_TARGET_Y = 2.25            # 前表面 Y (0.6+1.65=2.25, 方块半宽+中心偏移)
ZONE2_ROOT_YAW_OFFSET = math.pi / 2  # 检测线 yaw → zone2_root yaw 的偏置 (90°)
ZONE2_ROOT_Z = -0.270            # zone2_root 的 Z 偏移 (m), 与 GROUND_Z 一致

# ════════════════════════════════════════════════════════════
# 6. 前立面分层与 RANSAC 拟合
# ════════════════════════════════════════════════════════════
# 梅林台阶前立面由 200mm 高白线和 400mm 高黄线交替构成。
# 检测策略: 在立面切片内做 2D 直方图密度边缘检测 + RANSAC 直线拟合。

# 前立面有效高度窗口 (离地 h)
FACADE_SLICE_Z_MIN = 0.005       # 立面切片最低 h (m): 接近地面, 排除地面以下噪点
FACADE_SLICE_Z_MAX = 0.400       # 立面切片最高 h (m): 覆盖到 400mm 黄线顶部
# 200mm 白线高度带
BAND_200_Z_MIN = 0.050           # 白线下界 h (m): 排除地面附近噪点, 从 50mm 开始
BAND_200_Z_MAX = 0.200           # 白线上界 h (m): 200mm 方块顶
# 400mm 黄线高度带
BAND_400_Z_MIN = 0.250           # 黄线下界 h (m): 高于白线顶 50mm, 避免高度带重叠
BAND_400_Z_MAX = 0.400           # 黄线上界 h (m): 400mm 方块顶

# 2D 直方图分辨率
Y_BIN_SIZE = 0.05                # Y 方向 bin 宽度 (m)
X_BIN_WIDTH = 0.05               # X 方向 bin 宽度 (m)
MIN_DENSITY_PEAK = 5             # 单个 (Y,X) bin 最少点数, 低于此视为稀疏噪点

# RANSAC 直线拟合
RANSAC_RESIDUAL_THRESHOLD = 0.05 # 内点残差阈值 (m)
MIN_EDGE_POINTS = 5              # RANSAC 最少内点数, 低于此跳过该帧
RANSAC_N_ITER = 200              # RANSAC 随机采样迭代次数
MAX_ALLOWED_SLOPE = 0.6          # 拟合斜率上限 |k|, 防深度方向误检 (台阶侧面/墙面)

# ════════════════════════════════════════════════════════════
# 7. 墙面空间约束与柱状判决
# ════════════════════════════════════════════════════════════
# 在拟合立面附近约束墙面点, 按 Y 方向分柱统计 200/400 点密度。
# 黄色(黄柱): 200+400 都够密; 白色(白线候选): 仅 200 够密。

WALL_HALF_WIDTH = 0.10           # 距拟合基准线半宽 (m), 只取此范围内点做柱统计
COL_WIDTH = 0.05                 # Y 方向分柱宽度 (m)
Y_COLUMN_MIN = -2.5              # 柱状统计 Y 起始 (m), 旋转后坐标系
Y_COLUMN_MAX = 2.5               # 柱状统计 Y 结束 (m), 旋转后坐标系
Y_COLUMN_NUM = int((Y_COLUMN_MAX - Y_COLUMN_MIN) / COL_WIDTH)  # 柱总数 (100柱)
DILATE_BINS = 2                  # 黄色柱膨胀格数, 屏蔽白色候选 (黄色优先, 膨胀范围=2×0.05=0.10m)

# ════════════════════════════════════════════════════════════
# 8. 合成视觉几何 (调试可视化, 发布到 /rc26/zone2/cloud_facade_recon)
# ════════════════════════════════════════════════════════════
SYNTHETIC_LINE_N = 500           # 合成白线采样点数 (沿 Y 方向等间距)
SYNTHETIC_CLUSTER_N = 100        # 合成红心簇点数 (靶心附近高斯散布)
SYNTHETIC_CLUSTER_SPREAD = 0.015 # 红心高斯散布标准差 (m)

# ════════════════════════════════════════════════════════════
# 9. 锁定策略
# ════════════════════════════════════════════════════════════
# 自动锁定: 连续多帧候选位姿稳定后自锁, 避免手动按 Enter。
AUTO_LOCK_ZONE2_ROOT = 1         # 1=启用自动锁定, 0=需外部 Service 触发
STABLE_LOCK_COUNT = 4            # 稳锁所需连续帧数
STABLE_CENTER_TOL = 0.22         # 候选中心最大离散 (m), 超过则不锁
STABLE_YAW_TOL = math.radians(10.0)  # 候选 yaw 最大离散 (rad), 10°
DYNAMIC_TF_RATE = 10.0           # 锁定后 TF 重发频率 (Hz)

# 入口精修: 车走到梅林入口区、正对台阶时做小幅修正
ENABLE_ENTRY_REFINEMENT = 1      # 1=启用入口精修, 0=禁用
REFINE_STABLE_COUNT = 3          # 精修稳锁帧数 (进入入口区后需连续稳定帧)
REFINE_CENTER_TOL = 0.16         # 精修中心容差 (m), 比粗锁更紧
REFINE_YAW_TOL = math.radians(8.0)   # 精修 yaw 容差 (rad), 8°
REFINE_MAX_TRANSLATION = 0.65    # 精修最大平移 (m), 候选距锁定点超过此值则拒
REFINE_MAX_YAW_DELTA = math.radians(20.0)  # 精修最大 yaw 变化 (rad), 超过则拒
REFINE_ENTRY_X_ABS_MAX = 2.70    # 入口区 X 范围上限 (zone2_root 系, m)
REFINE_ENTRY_Y_MIN = 1.45        # 入口区 Y 下限 (zone2_root 系, m)
REFINE_ENTRY_Y_MAX = 3.70        # 入口区 Y 上限 (zone2_root 系, m)
REFINE_FACING_YAW = -math.pi / 2.0  # 正对梅林的期望 yaw (zone2_root 系), -90°
REFINE_FACING_YAW_TOL = math.radians(75.0)  # 入口朝向容差 (rad), 75°
REFINE_ALPHA = 0.65              # 精修 EMA 系数 (越大跟随越快, 越小越平滑)
REFINE_ONCE = 1                  # 1=仅精修一次, 0=入口区内持续精修

# ════════════════════════════════════════════════════════════
# 10. 观测位置门控
# ════════════════════════════════════════════════════════════
# 仅在机器人在 Z2 最佳观测区域内才开始检测, 避免 Z1 阶段误检。

ENABLE_ODOM_GATE = 1             # 1=启用观测位置门控, 0=始终检测
GATE_CENTER_X_DEFAULT = 1.600-0.6    # 观测区域中心 X (BLUE, odom 系), RED 取反 = -1.600
GATE_CENTER_Y_DEFAULT = 1.5    # 观测区域中心 Y (BLUE, odom 系), RED 取反 = -1.350
GATE_HALF_SIZE_X = 0.75          # 观测区 X 半宽 (m)
GATE_HALF_SIZE_Y = 0.75          # 观测区 Y 半高 (m)
GATE_STRENGTH_ATTENUATION = 0.7  # 边缘检测强度衰减: 1.0=完全衰减, 0.0=不衰减 (越靠近中心强度越高)
GATE_YAW_TOLERANCE_DEG = 30      # 朝向容差 (度), 车头朝向误差超过此值则门控关闭

# 调试: 在观测区域中心发布 TF (odom 系), 用于 RViz 确认门控位置
PUBLISH_GATE_DEBUG_TF = 1        # 1=10Hz 发布 zone2_gate_debug TF (GATE_CENTER 原地)

# ════════════════════════════════════════════════════════════
# 11. 降采样与日志
# ════════════════════════════════════════════════════════════
DOWNSAMPLE_STEP = 5              # 降采样步长: 每 N 点取 1, 减少计算量 (5=取20%)
LOG_ENABLED = False              # 是否打印终端调试日志 (INFO级别, 1s节流)
LOG_INTERVAL = 1.0               # 终端日志最小间隔 (s)
DETAILED_FILE_LOG = False        # 是否写入逐帧 CSV 详细日志
DETAIL_LOG_DIR = "/home/inkc/inkc/Rc2026/files/record/logs"  # CSV 日志输出目录
