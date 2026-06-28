# depth_grid_plane_probe

Headless prototype for "forward depth cut -> visible plane/layer split -> simulated Zone3 nine-grid cloud candidate".

It does not publish Zone3 TF and does not modify `fence_locator`.

## Inputs

- Point cloud: `/odin1/cloud_slam`
- TF: `odom -> odin1_base_link`
- TF: `odom -> blue_zone3_root_auto`

## Outputs

- JSON result: `/zone3/depth_grid_probe/result`
- Candidate markers: `/zone3/depth_grid_probe/markers`

## Pipeline

1. Parse `PointCloud2` x/y/z with field offsets.
2. Transform cloud into `odin1_base_link`.
3. Try a forward rectangle in four base-frame directions (`+x/-x/+y/-y`) when `auto_cut_axis=true`:
   - `forward_x_min_m <= x <= forward_x_max_m`
   - `forward_y_min_m <= y <= forward_y_max_m`
   - `forward_z_min_m <= z <= forward_z_max_m`
4. Optional foreground depth cut: angular bins in base frame keep only points within `foreground_keep_depth_m` after each ray's nearest range.
5. Transform the same surviving points into `odom`.
6. Transform into `blue_zone3_root_auto` local coordinates and keep a broad root-frame ROI around the expected nine-grid center.
7. If base-frame depth cut leaves no points in the model ROI, optionally use `root_roi_fallback=true`: keep the root-frame nine-grid ROI first, then continue the same plane/layer analysis. In `c2_20260626_151444`, this fallback is the path that produces stable candidates.
8. Estimate visible bottom height from low points in the ROI.
9. Keep high points between `bottom + grid_min_h_m` and `bottom + grid_max_h_m`.
10. Split high points by 2D connected components.
11. For each component:
    - PCA yaw from the long axis.
    - Depth-support filter: model-depth bins must have enough lateral span.
    - Estimate center, width, depth.
    - Count three height layers and three lateral columns.
    - Check low/base plane support in the candidate footprint.
  - Score confidence from points, width, depth, layers, columns, plane support, and distance from the expected model center.

## Current Bag Finding

`c2_20260626_151444` headless test:

- Pure base-frame forward/depth cut produced `root_roi=0` for all four axes at first.
- Raw model ROI later contained about `3380~3406` points.
- With `root_roi_fallback=true`, candidate output stabilized around:
  - `confidence ~= 0.86`
  - `points ~= 540`
  - `yaw ~= 95.5~96.5deg`
  - `width ~= 1.63~1.66m`
  - `layers=3`
  - `columns=3`

Interpretation: this bag does contain usable nine-grid structure, but a vehicle-frame forward depth cut alone is not a safe first gate. The safer sequence is `corner/root TF ROI -> plane/layer/grid check`; vehicle-frame depth cutting should remain diagnostic or secondary until its corridor is calibrated.

## Run

```bash
cd /mnt/c/Users/22240/rc2026_snapshot/rc
source install/setup.bash
ros2 run zone3_grid_fusion depth_grid_plane_probe
```

Typical bag replay workflow:

```bash
cd /mnt/c/Users/22240/rc2026_snapshot/rc
source install/setup.bash
ros2 run zone3_grid_fusion depth_grid_plane_probe
ros2 topic echo /zone3/depth_grid_probe/result
```

In another shell, launch the existing fence stack or replay the bag so that `/odin1/cloud_slam`, `blue_zone3_root_auto`, and `odin1_base_link` TF are available.

## Useful Parameters

```bash
ros2 run zone3_grid_fusion depth_grid_plane_probe --ros-args \
  -p forward_x_min_m:=0.25 \
  -p forward_x_max_m:=5.20 \
  -p forward_y_min_m:=-1.35 \
  -p forward_y_max_m:=1.35 \
  -p foreground_keep_depth_m:=0.35 \
  -p auto_cut_axis:=true \
  -p root_roi_fallback:=true \
  -p root_roi_x_m:=1.80 \
  -p root_roi_y_m:=1.80
```

Output JSON fields to watch:

- `counts.forward_rect`: points after the base-frame forward rectangle.
- `counts.foreground`: points after per-ray nearest-depth cutting.
- `counts.root_roi`: points in the Zone3-root local ROI.
- `counts.raw_root_roi`: points in the model ROI before vehicle-frame depth cut fallback.
- `counts.high`: high-layer candidate points.
- `cut_axis`: selected cut source, such as `+x` or `root_roi_fallback`.
- `best.confidence`: 0..1 prototype confidence.
- `best.center`: candidate center in `odom`.
- `best.local_center`: candidate center in `blue_zone3_root_auto`.
- `best.yaw_deg`: candidate yaw in `odom`.
- `best.points`, `best.layers`, `best.columns`, `best.plane_points`.
