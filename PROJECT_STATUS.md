# rc26_zone3 当前进度与下一步修改方向

更新时间：2026-06-18

## 当前状态

- 项目已从误删事故中恢复到 `C:\Users\22240\rc2026_snapshot`。
- 已初始化本地 Git 仓库，并推送到 GitHub 私有仓库：
  - `https://github.com/huyasann/rc26_zone3`
- 当前远程分支：
  - `main`
- 当前可用提交：
  - `27c423a restore rc2026 snapshot source`
- 已确认可以编译运行的主要包：
  - `rc/src/uphill`
  - `rc/src/fence_locator`
  - `zone_detection`
- `bag/`、`build/`、`install/`、`log/`、缓存目录已被 `.gitignore` 排除，不应上传。

## 当前能做什么

当前代码已经能做第三区粗定位流程：

1. `uphill_state_node` 读取 `/odin1/odometry_highfreq`。
2. 状态机判断平地、上坡、上平台等阶段。
3. `fence_locator` 在上坡/上平台相关阶段采集 `/odin1/cloud_slam`。
4. 点云被转换到局部坐标系。
5. 根据点云边界寻找第三区关键角点。
6. 根据关键角点和场地模型先验发布 `zone3_root` 相关 TF/marker。
7. RViz 中可以看到粗略拟合到第三区场地的位置。

当前版本是“可跑、可看、能粗略拟合”的基线，不是最终高精度版本。

## 当前检测逻辑概括

核心文件：

- `rc/src/fence_locator/fence_locator/fence_locator_node.py`
- `rc/src/fence_locator/fence_locator/zone3_corner_detector.py`
- `rc/src/fence_locator/fence_locator/geometry.py`
- `rc/src/fence_locator/fence_locator/pointcloud.py`

当前角点检测大致流程：

1. 上坡状态机触发后开始收集点云。
2. 用 odom 起点和朝向建立粗局部坐标。
3. 裁切第三区附近 ROI。
4. 用高度做粗筛，取平台/围栏附近点。
5. 用 bin 分组和分位数提取外轮廓边界点。
6. 拟合外侧长边。
7. 拟合端部边。
8. 两条边求交得到目标角点。
9. 角点附近竖直点列、两边夹角、拟合残差、内点数量用于判断质量。
10. 根据模型中已知关键点和检测到的实际角点对齐第三区模型。

## 当前主要问题

1. 角点还是偏粗，精度不够稳定。
2. 端部边在点云不完整时容易不稳定。
3. 目前有 RANSAC refine 雏形，但还不是完整的“两条边/两个竖直面 RANSAC 精拟合”。
4. 高度筛选仍偏粗，只适合作为初筛，不能直接决定最终角点。
5. `fence_locator_node.py` 仍然偏大，ROS 逻辑、检测逻辑、marker 发布混在一起，后续维护不舒服。

## 下一步修改主线

短期不要继续折腾坡道 marker。重点转向第三区关键角点精度。

优先保留的能力：

- 状态机触发采集
- 红蓝区判断
- 关键角点检测
- 根据关键角点对齐场地模型
- 发布 `zone3_root` TF/marker
- RViz 调试 marker

暂时弱化或搁置：

- 坡道中心线高精度拟合
- 坡道 marker 高低补偿
- 依赖整车 odom 起点/终点直接决定最终场地位置

## 推荐修改路线

### 第一步：整理代码结构

目标：不要再把检测细节塞进 2000 行 ROS 节点。

建议职责拆分：

- `geometry.py`
  - RANSAC 直线拟合
  - 两线求交
  - 点到线距离
  - 角度计算
  - 线段长度估计

- `pointcloud.py`
  - PointCloud2 解析
  - ROI 裁切
  - 高度粗筛
  - 点云降采样

- `zone3_corner_detector.py`
  - 第三区角点检测主逻辑
  - 粗角点检测
  - 局部 RANSAC 精修
  - 置信度评分

- `fence_locator_node.py`
  - ROS 参数
  - topic 订阅
  - 状态机响应
  - TF 发布
  - marker 发布

### 第二步：保留当前粗角点作为初值

不要直接删掉当前能跑的算法。

当前粗角点的作用：

- 快速给出大概位置
- 给后续局部 ROI 提供中心
- 作为失败时的 fallback

### 第三步：在粗角点附近做局部精拟合

流程：

1. 以粗角点为中心截取小 ROI。
2. ROI 半径建议先用：
   - `0.35m ~ 0.50m`
3. 高度只做粗筛，不直接决定最终角点。
4. 在小 ROI 内找两类边：
   - 外侧长边
   - 端部边
5. 分别用 RANSAC 拟合两条线。
6. 要求两条线接近 90 度。
7. 两条线求交得到精角点。
8. 检查角点附近是否存在竖直点列。
9. 检查角点是否落在场地模型合理范围。
10. 置信度够才发布最终 `zone3_root`。

### 第四步：建立评分机制

每个候选角点至少计算：

- 外侧边内点数
- 端部边内点数
- 外侧边 RMSE
- 端部边 RMSE
- 两边夹角误差
- 角点附近竖直点数量
- 角点附近竖直高度跨度
- 角点相对场地模型的位置合理性

建议先不要追求复杂模型匹配，先把这套评分做稳定。

## RViz 调试 marker 要求

后续每次改检测算法，必须保留这些可视化：

- 粗角点：红色球
- 精角点：绿色或紫色小球
- 外侧长边：蓝线
- 端部边：红线
- 小 ROI：半透明框或线框
- 最终 `zone3_root`：TF 名称可见
- 场地模型：只显示必要边线，避免杂乱 marker 干扰判断

## 日志输出要求

终端日志不要刷屏。每次完成一次检测后输出一行摘要即可：

- ROI 点数
- 高度筛后点数
- 外侧边内点数/RMSE
- 端部边内点数/RMSE
- 两边夹角
- 粗角点坐标
- 精角点坐标
- 是否发布 `zone3_root`
- 失败原因

## Git 备份规则

每次确认一个小阶段能跑，就由使用者手动执行 Git 命令备份。

查看当前状态：

```bash
cd /mnt/c/Users/22240/rc2026_snapshot
git status
```

保存一次修改：

```bash
git add .
git commit -m "这里写本次修改内容"
git push
```

角点检测改进示例：

```bash
cd /mnt/c/Users/22240/rc2026_snapshot
git status
git add .
git commit -m "improve zone3 corner fitting"
git push
```

只更新文档示例：

```bash
cd /mnt/c/Users/22240/rc2026_snapshot
git status
git add .
git commit -m "update project status and zone3 fitting plan"
git push
```

查看最近提交：

```bash
git log --oneline --max-count=10
```

查看还没保存的修改：

```bash
git diff
```

放弃某个文件的未提交修改：

```bash
git restore 路径/文件名
```

例子：

```bash
git restore rc/src/fence_locator/fence_locator/zone3_corner_detector.py
```

## 当前建议的下一次开发任务

下一次开发不要大改 launch 和状态机，先只做：

1. 把 `zone3_corner_detector.py` 里的检测函数继续拆小。
2. 保留当前粗角点输出。
3. 新增局部 RANSAC 精修函数。
4. 新增精角点 marker。
5. 在 RViz 对比粗角点和精角点。
6. 只在精角点评分通过时发布 `zone3_root`。

判断是否成功：

- 红点粗角点仍能出现。
- 精角点比粗角点更贴近用户指定的第三区关键角。
- 蓝线和红线分别贴合两条真实边。
- 场地模型不再明显整体偏移。
- 失败时能明确输出失败原因，而不是乱发布。
