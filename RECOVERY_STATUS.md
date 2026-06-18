# rc2026_snapshot 恢复状态

## 结论

- 可用恢复目录：`E:\rc2026_recovered_merged_no_bag`
- 已验证：
  - `python3 -m py_compile` 通过
  - `rc/colon.sh` 下 `colcon build` 通过
  - `zone_detection` 单包 `colcon build` 通过
  - ROS2 包：`fence_locator`、`uphill`、`zone_detection`
- 未直接写回 `C:\Users\22240\rc2026_snapshot`，避免继续写 C 盘影响磁盘级恢复。

## 来源

- 原始项目基底：`G:\rc2026_snapshot`
- 后续新增/修改源码：`E:\rc2026_recovered_best`
- 辅助来源：VSCode/Gemini/Claude 打开文件缓存、Codex/Claude 日志中的 `Read` 和 `apply_patch` 记录
- bag 来源：`G:\rc2026_snapshot\bag` 或 `C:\Users\22240\rc2026_snapshot - 副本\bag`

## 当前恢复到的重点文件

- `rc/src/fence_locator/fence_locator/fence_locator_node.py`
- `rc/src/fence_locator/fence_locator/geometry.py`
- `rc/src/fence_locator/fence_locator/pointcloud.py`
- `rc/src/fence_locator/fence_locator/zone3_corner_detector.py`
- `rc/src/fence_locator/launch/launch_fence.launch.py`
- `rc/src/uphill/uphill/uphill_state_node.py`
- `qjy/blue_z3_live_ramp_fit.py`
- `qjy/ground_calibrator.py`
- `qjy/z3_state_monitor.py`
- `zone_detection/zone_detection/field_publisher/rc26_field.py`
- `zone_detection/zone_detection/zone2/...`
- `zone_detection/zone_detection/zone3/...`
- `zone_detection/zone_detection/kfs_grid/...`
- `Readme.md`
- `Workflow.md`

## 缺口

- `other/` 下 PDF、PNG 等大文件未从删除区恢复；若副本没有，只能继续查 VSCode 缓存或走 Windows 文件恢复工具。
- `bag/` 未复制进 `E:\rc2026_recovered_merged_no_bag`，按当前要求暂时不处理 bag。

## 编译命令

```bash
cd /mnt/e/rc2026_recovered_merged_no_bag/rc
. ./colon.sh
```

## 如需恢复回 C 盘

先确认是否放弃磁盘级恢复；确认后再把 `E:\rc2026_recovered_merged_no_bag` 内容复制回：

```text
C:\Users\22240\rc2026_snapshot
```

如果需要继续使用 bag，可从：

```text
G:\rc2026_snapshot\bag
```

复制，或保留 launch 参数指向该路径。
