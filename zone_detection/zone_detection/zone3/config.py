"""Zone3 九宫格检测 — 可调参数。

所有常量按功能分组，便于现场标定调参。
每个参数一行，行末附带详细中文注释。

调参顺序:
  1. 检测不到候选 → 先放宽 ROI 或 GRID_MIN_H/GRID_MAX_H
  2. 候选太多误检 → 提高 MIN_LOCK_CONFIDENCE, 收紧 width/depth 范围
  3. 锁定后抖动 → 增大 STABLE_LOCK_COUNT, 减小 STABLE_CENTER_TOL
  4. TF 对不上场地 → 检查 IS_BLUE_TEAM 和 grid_center_rel_x/y
  5. Z2 先验偏了 → 检查 Z2_TO_Z3_OFFSET_X/Y
"""

import math

# ════════════════════════════════════════════════════════════
# 1. 输入点云 ROI 裁剪
# ════════════════════════════════════════════════════════════
# base_link 系: 车前检测, X 足够长才能覆盖九宫格
X_MIN, X_MAX = 0.15, 5.50        # base_link 系 X 范围 (车前方向, m)
Y_MIN, Y_MAX = -2.20, 2.20       # base_link 系 Y 范围 (横向, m)
Z_MIN, Z_MAX = -1.50, 2.80       # base_link 系 Z 范围 (高度, m), 覆盖基座到顶层
# odom 系: 车已到重试区时用宽 ROI
ODOM_X_MIN, ODOM_X_MAX = 0.0, 16.0   # odom 系 X 范围 (m), 覆盖全场
ODOM_Y_MIN, ODOM_Y_MAX = -4.0, 4.0   # odom 系 Y 范围 (m), 覆盖双侧平台

# ════════════════════════════════════════════════════════════
# 2. 坐标系
# ════════════════════════════════════════════════════════════
HEIGHT_FRAME = "odom"              # 高度归一目标坐标系 (所有点云的 Z 投影到此系)
SOURCE_FIXED_FRAME = "odom"        # TF 父帧 (zone3_root 发布在此帧下)
ROBOT_TF_FRAME = "odin1_base_link" # 机器人 base_link TF frame 名 (用于门控位姿查询)

# ════════════════════════════════════════════════════════════
# 3. 地面估计
# ════════════════════════════════════════════════════════════
# 与 zone2 共用策略。Z3 平台地面回波不稳, 比赛建议手动固定。
GROUND_Z_KNOWN = 1                 # 1=手动固定 ground_z, 0=首帧自动估计+EMA更新
GROUND_Z = -0.1500                 # odom 系地面 Z 值 (m), GROUND_Z_KNOWN=1 时生效
GROUND_TOLERANCE = 0.035           # 地面点判定容差 (m): |h|≤此值视为地面
HISTOGRAM_BIN_WIDTH = 0.02         # 直方图 bin 宽度 (m)
GROUND_PEAK_RATIO = 0.15           # 自动检测时, 从低到高扫到主峰此比例即认作地面
GROUND_UPDATE_ALPHA = 0.18         # EMA 更新系数 (越大跟随越快, 越小越稳定)
GROUND_MAX_UPDATE_STEP = 0.06      # 自动更新时单帧最大跳变 (m), 防误检拉飞

# ════════════════════════════════════════════════════════════
# 4. 队伍与 TF 命名
# ════════════════════════════════════════════════════════════
IS_BLUE_TEAM = 0                   # 代码默认值, 运行时由 ROS 参数 is_blue_team 覆盖

# ════════════════════════════════════════════════════════════
# 5. 九宫格场地先验 (rc26_field.py 模型坐标系)
# ════════════════════════════════════════════════════════════
# zone3_root 在场地模型中的绝对坐标
BLUE_ZONE3_ROOT_X = 3.025          # 蓝队 zone3_root X 坐标 (场地模型系)
RED_ZONE3_ROOT_X = -3.025          # 红队 zone3_root X 坐标 (场地模型系, Y轴对称)
ZONE3_ROOT_FIELD_Y = -4.60         # zone3_root Y 坐标 (场地模型系, 红蓝相同)
# 九宫格中心在场地模型中的坐标 (用于反推 zone3_root 位姿)
GRID_FIELD_X = 0.0                 # 九宫格中心 X (场地模型系)
GRID_FIELD_Y = -4.75               # 九宫格中心 Y (场地模型系)

# 九宫格几何尺寸
GRID_WIDTH_Y = 1.62                # Y 向总宽度 (m), 3格×0.50 + 间隙 = 1.62m
GRID_DEPTH_X = 0.32                # X 向深度 (m), 单层块的厚度

# 高位点筛选高度范围 (离地 h)
GRID_MIN_H = 0.75                  # 高位筛选下限 h (m): 基座以上的第一层块底部
GRID_MAX_H = 2.60                  # 高位筛选上限 h (m): 第三层块顶部

# ════════════════════════════════════════════════════════════
# 6. 重试区门控 + Z2 先验
# ════════════════════════════════════════════════════════════
# 机器人必须位于重试区内才检测九宫格, 避免提前误检。

REQUIRE_ZONE3_ODOM_GATE = 0        # 1=启用重试区门控 (位置+朝向), 0=始终检测

# 重试区位置 (odom 系, 红蓝独立)
BLUE_RETRY_CENTER_X = 11.100 - 0.4 # 蓝队重试区中心 X (odom 系 m), 10.700
BLUE_RETRY_CENTER_Y = 4.100        # 蓝队重试区中心 Y (odom 系 m)
RED_RETRY_CENTER_X = 11.100        # 红队重试区中心 X (odom 系 m)
RED_RETRY_CENTER_Y = -4.100        # 红队重试区中心 Y (odom 系 m), Y轴对称
RETRY_AREA_HALF_SIZE = 0.8         # 重试区半边长 (m), 正方形
RETRY_AREA_Z = 0.3                 # 调试 Marker 放置高度 (odom 系 Z, m)

# 重试区朝向九宫格的期望 yaw (odom 系, 基于场地模型推算)
# BLUE: atan2(-5.525, -0.75) ≈ -1.708 rad
# RED:  atan2(+5.525, -0.75) ≈ +1.708 rad
BLUE_RETRY_FACE_YAW = -1.708       # 蓝队: 正对九宫格的期望 yaw (odom 系 rad)
RED_RETRY_FACE_YAW = 1.708         # 红队: 正对九宫格的期望 yaw (odom 系 rad)
RETRY_FACE_YAW_TOLERANCE = math.radians(45)  # 朝向门控容差 (rad), 45°

# Z2 先验: 第一次进入重试区时, 读取 zone2 TF 推算 Z3 初锁
ENABLE_Z2_PRIOR = 1                # 1=启用 Z2 先验初锁, 0=仅依赖点云检测
# Z2 → Z3 在场地模型中的固定平移 (Z2_ROOT = (±3.025, 0.55), Z3_ROOT = (±3.025, -4.60))
Z2_TO_Z3_OFFSET_X = 0.0            # Z2→Z3 X 方向偏移 (m): X 相同无偏移
Z2_TO_Z3_OFFSET_Y = -5.15          # Z2→Z3 Y 方向偏移 (m): 0.55→-4.60 = -5.15

# Z3 精修参数 (到达重试区后, 在 Z2 初锁基础上小范围修正)
Z3_REFINE_MAX_TRANSLATION = 0.40                # 最大平移修正 (m), 候选距锁定点超过则拒
Z3_REFINE_MAX_YAW_DELTA = math.radians(8.0)     # 最大 yaw 修正量 (rad), 8°
Z3_REFINE_ALPHA = 0.65                          # EMA 平滑系数 (越大跟随越快)
Z3_REFINE_STABLE_COUNT = 3                      # 精修稳定帧数 (需连续稳定帧)

# ════════════════════════════════════════════════════════════
# 7. 锁定策略
# ════════════════════════════════════════════════════════════
MIN_LOCK_CONFIDENCE = 0.76         # 单帧几何评分阈值 [0,1]: width/depth/layer/density 加权分 ≥ 此值
STABLE_LOCK_COUNT = 4              # 稳锁所需连续帧数
STABLE_CENTER_TOL = 0.35           # 候选中心最大离散 (m), 超过则不锁 (Z3 比 Z2 宽松)
STABLE_YAW_TOL = math.radians(12.0) # 候选 yaw 最大离散 (rad), 12° (Z3 比 Z2 宽松)
DYNAMIC_TF_RATE = 10.0             # 锁定后 TF 重发频率 (Hz)
DOWNSAMPLE_STEP = 1                # 点云降采样步长: 1=不降采样 (保留全部点)
ACCUMULATE_FRAMES = 20             # 累计最近 N 帧高位候选再拟合 (增加点密度)

# ════════════════════════════════════════════════════════════
# 8. 调试输出
# ════════════════════════════════════════════════════════════
LOG_INTERVAL = 0.5                 # INFO 日志节流间隔 (s)
DETAILED_FILE_LOG = False          # 是否写入逐帧 CSV 详细日志
DEBUG_DIR = "/home/inkc/inkc/Rc2026/files/record/logs"  # 调试日志输出目录
ENABLE_DEBUG_VIS = 1               # 调试可视化总开关: 0=全部关闭, 1=开启所有调试话题

# 高度诊断日志 (排查层2/层3误染问题)
ENABLE_HEIGHT_DIAG_LOG = 0         # 1=启用高度诊断 CSV 日志, 0=关闭
HEIGHT_DIAG_LOG_DIR = "/home/inkc/inkc/Rc2026/files/record/logs"  # 高度诊断日志输出目录

# 高度区间调色板: (h_min, h_max, r, g, b)
# 用于调试标定 ground_z 和层高边界, 每区间用不同颜色染色
HEIGHT_BANDS = [
    (-0.10, 0.07, 180, 180, 180),    # 地面附近 ~0m: 灰色
    (0.07,  0.50, 255, 255, 0),      # 基座/平台顶 0.07~0.50m: 黄色
    (0.50,  0.80, 0,   200, 200),    # 基座~层1间隙 0.50~0.80m: 青色
    (0.80,  1.34, 255, 80,  20),     # 层1方块 0.80~1.34m: 橙色
    (1.34,  1.88, 80,  200, 80),     # 层2方块 1.34~1.88m: 绿色
    (1.88,  2.50, 200, 80,  255),    # 层3方块 1.88~2.50m: 紫色
]
