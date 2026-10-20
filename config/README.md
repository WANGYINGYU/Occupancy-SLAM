# Configuration reference

The public entry point is `tools/occupancy-slam {txt|bag|pcd}`. YAML keys use underscores; corresponding CLI options use hyphens. CLI values override YAML. Input paths in YAML are relative to the YAML file; CLI input paths are relative to the calling directory. Relative outputs are resolved inside a newly created data-disk run directory. Explicit unknown keys, invalid types, and missing configured topics are errors.

## Input profiles

| Profile | Purpose |
| --- | --- |
| [inputs/txt.yaml](inputs/txt.yaml) | Original 2D text data, initialization source, frame selection, and display resolution. |
| [inputs/rosbag_laserscan.yaml](inputs/rosbag_laserscan.yaml) | LaserScan bags, odometry, TF topics, and frontend settings. |
| [inputs/rosbag_pointcloud.yaml](inputs/rosbag_pointcloud.yaml) | PointCloud2 bags, projection frame, height interval, and angular bins. |
| [inputs/pcd.yaml](inputs/pcd.yaml) | 3D PCD/TUM or manifest inputs, sampling, voxel size, and solver limits. |

## Bag topics and frames

For example:

```yaml
scan_topic: /robot/front/scan
odom_source: odom
odom_topic: /robot/wheel/odometry
tf_topic: /robot/tf
tf_static_topic: /robot/tf_static
odom_frame: robot_odom
projection_frame: null
```

`scan_topic: auto` selects a topic only when the bag contains exactly one supported scan topic. With multiple sensors, specify the topic. `odom_source` accepts `auto`, `odom`, `tf`, or `scan-match`. Automatic selection uses configured Odometry, available TF, then scan matching. Explicit `odom` rejects a topic without usable Odometry messages. TF frame identifiers and topic names are independent settings.

For PointCloud2, `projection_frame` selects a frame available through TF; use a level frame when necessary. `min_height`, `max_height`, and `bins` control the projected scan. The range and time-gap parameters must match the sensor and TF frequency.

Select bag topics in the input YAML. [slam_toolbox.yaml](slam_toolbox.yaml) controls scan matching, loop closure, and the Ceres solver. The adapter overrides internal scan/frame names, simulation time, TF publication, sequential processing, and range settings. The effective frontend configuration is saved in the run directory.

## Original 2D text data

- `Range.txt`: one scan per row, ranges in metres. The number of columns must match `NumBeam` in the backend profile.
- `Pose.txt`: absolute sensor poses `x y yaw`, with yaw in radians.
- `Odom.txt`: optional; the first row is an absolute initial pose and subsequent rows are SE(2) increments in the preceding frame.
- `timestamps.txt`: optional increasing timestamps in seconds. Otherwise, display timestamps are generated from the original frame index and `scan_period` and recorded as synthetic.

`--ranges`, `--poses`, `--odom`, and `--timestamps` override dataset filenames. `pose_source: auto` respects `PosefromOdom` in the backend profile and can initialize from odometry when poses are absent. `WeightO > 0` requires odometry; use `WeightO 0` when only poses are available. Pose differences are not substituted for independent odometry.

Set `NumBeam`, `MinAngle`, `MaxAngle`, `MinRange`, and `MaxRange` for the scan geometry. [backend/intel.txt](backend/intel.txt) uses 180 beams; [backend/museum_b0.txt](backend/museum_b0.txt) uses 1079. The TXT adapter composes skipped odometry increments in SE(2). It honors `ModeMulti`; its batch path disables the legacy interactive options `ModeKeyFrame`, `EvaluateGT`, and `ModeShowMap`.

## Resolution and optimization

| Setting | Meaning |
| --- | --- |
| `frontend_resolution` | Optional override of the slam_toolbox map resolution. |
| `resolution` / backend `Scale` | 2D optimization grid spacing in metres. |
| `display_resolution` | Initial/final-pose display integration spacing; default 0.05 m. |
| `voxel` | 3D optimization grid spacing. |
| `point_voxel`, `point_stride` | 3D input downsampling, independent of the map grid. |
| `step_policy: legacy` | Original update policy. |
| `step_policy: decrease` | Optional cost-decrease check with the current N held fixed. |
| `max_observations` | Limit on generated observations; not a process memory limit. |

N pose derivatives remain omitted. N is recomputed from zero and smoothed after each accepted update. The optional decrease policy does not guarantee monotonic cost after N is refreshed.

`ModeSparseSolver=true` selects CG and uses `SolverFirstMaxIter` / `SolverFirstTolerance` for the full-grid stage and `SolverSecondMaxIter` / `SolverSecondTolerance` for the fine stage. `false` selects direct solving. `ModeMulti=true` uses coarse spacing `Scale * DownRate` followed by a selected fine grid. `DownTime` controls the coarse budget; `MaxIter` is the total iteration limit. Early stopping follows the configured update thresholds.

`FixedGrid` selects explicit `Sizei`, `Sizej`, `OriginX`, and `OriginY` instead of automatic bounds. `InitialMapRound` controls initial/reintegrated binning; `NormalizationRound` controls N refresh. Legacy defaults are floor/round. `WeightSmoothN` and `WeightSmoothNSecond` are separate from the map regularization weights.

## Simulation profiles

| Profile | Intended use |
| --- | --- |
| [simulation_multires_extended.txt](backend/simulation_multires_extended.txt) | Complete coarse-to-fine simulation runs: at most 40 coarse and 10 fine updates, with configured early stopping. |
| [simulation_multires.txt](backend/simulation_multires.txt) | Short-budget diagnostic: at most 8 coarse and 3 fine updates. |
| [simulation_matlab.txt](backend/simulation_matlab.txt) | Single-resolution compatibility with the supplied MATLAB reference configuration. |

The single-resolution profile uses a 0.1 m, 500 x 500 grid with origin (-25,-25), up to 20 updates, direct solving, initial round / iterative N floor binning, N smoothing weight 10, active map regularization 1e-7, and update threshold 300. These describe the reference configuration, not every experiment in the papers.

Simulation conversion preserves exact `Scan.xy` and `Scan.Odd` samples. The benchmark defaults to those samples and the extended profile, with a 100000000-observation limit. `--resample-endpoints` selects a different observation model explicitly. Ground truth is used only for evaluation.

## Records and outputs

`input_manifest.json` identifies input poses and odometry provenance. Bag results distinguish external odometry from the slam_toolbox trajectory and increments derived from that trajectory. `run_status.json` records execution status; `summary.json` records the solver stop reason. Completion does not imply convergence.

The frontend trajectory is collected during processing; it is not replaced by final loop-closed graph poses. Output states and visualization resolution are documented in [map outputs](../docs/MAP_OUTPUTS.md) and [RViz](../docs/RVIZ.md).
