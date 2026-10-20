# Reproducibility and evaluation

This guide specifies the supported tests, simulation workflow, and evaluation conventions. Preserve the input data, initialization, effective configuration, software revision, and final accepted state when reporting results. Identify the ROS2 frontend and C++ backend configurations separately when reporting results. Standalone MATLAB optimizers are outside this branch.

## Functional and numerical tests

```bash
./tools/docker.sh test unit
./tools/docker.sh test inputs
./tools/docker.sh test architecture
```

Native equivalents use `./tools/test.sh`. Test fixtures are generated locally on the data disk; no dataset checkout or download is required. Install `requirements-test.txt` for native tests, or `requirements-matlab.txt` for simulation conversion and evaluation as well.

| Suite | Coverage |
| --- | --- |
| `unit` | Seven CTest cases: N approximation/refresh and solvers, scan I/O, display classification, pose metrics, Cartesian endpoints, multi-resolution execution, and simulator geometry. |
| `inputs` | TXT initialization variants; actual slam_toolbox processing of renamed LaserScan/PointCloud2, odometry, and TF topics; rejection of incorrect configured topics. |
| `architecture` | 2D/3D execution, solver configuration, PCD manifests, 3D Jacobian check, and 0.05/0.1 m map export. |

`architecture --reference-binaries /absolute/bin_directory` additionally compares the current backends with supplied reference binaries under the test's existing numerical tolerances. Graphical topic/window checks are provided by `tests/test_rviz.py` and `tests/test_rviz_topics.py`; supply their required result paths and an X11 session. Functional checks are distinct from dataset accuracy evaluation.

## Simulation inputs

Supply the original `Simu*_Data` MATLAB folders beneath a source directory. The converter supports MATLAB v5 and v7.3 files. It preserves Cartesian endpoints, the stored `Scan.xy` / `Scan.Odd` observation samples, ground truth for evaluation, and each supplied noisy-odometry or Cartographer initialization.

The converted directory records source paths, frame associations, geometry, and initialization metadata. An initialization missing the first pose uses its corresponding scan range and GT rows; no GT pose is inserted to fill the missing frame. Repeated initializations of one scene are not independent scenes.

## Conversion and optimization

From the project root, with the Docker image built:

```bash
export occupancy_runs="$OCCUPANCY_DATA_ROOT/experiments/sdf_ba/codex_runs"
./tools/docker.sh --run-name simulation_conversion exec \
  python3 /opt/occupancy-slam/tools/convert_matlab_simulations.py \
  --source /absolute/simulation --output converted
export occupancy_converted="$occupancy_runs/simulation_conversion/converted"

./tools/docker.sh --run-name simulation_benchmark exec \
  python3 /opt/occupancy-slam/tools/benchmark_matlab_simulations.py \
  --converted "$occupancy_converted" \
  --backend-config /opt/occupancy-slam/config/backend/simulation_multires_extended.txt \
  --jobs 1
```

Run names must be unused. The benchmark runs every initialization in the inventory; `--case Simu2680/odom_1` selects one case for adapter checks. For older conversions lacking exact samples, make the original source directory accessible with `--mount-input /absolute/simulation` before `exec`, or regenerate the conversion.

The default benchmark uses the exact stored samples and the extended coarse-to-fine profile. `--resample-endpoints` changes the observations and must be reported as a separate experiment. `simulation_matlab.txt` is a single-resolution reference profile; `simulation_multires.txt` is a short-budget diagnostic. Parameters are described in [config/README.md](../config/README.md).

The benchmark records effective configurations, actual commands, execution and solver status, per-case pose metrics, wall time, and sampled peak RSS/swap. Concurrent runs are unsuitable for isolated timing claims; use one worker and comparable hardware/settings for performance comparisons.

## Trajectory evaluation

`tools/evaluate_2d.py` evaluates the initial, joint, and reintegrated poses against independent GT. The primary convention is:

1. Match every result frame by explicit GT index or by a unique timestamp within one microsecond. No unmatched frame is silently discarded.
2. Compute a rigid SE(2) transform from the initial estimate's first pose to its GT pose and apply that same transform to all methods. No scale is fitted.
3. Exclude the fixed first pose from absolute-error statistics. Wrap heading differences before reporting degrees.
4. Report the final accepted optimization state. GT is not used to select an iteration or determine convergence.

The Euclidean position RMSE is `sqrt(mean(dx^2 + dy^2))`. The coordinate RMSE is `sqrt(mean([dx^2, dy^2]))`, which is smaller by a factor of `sqrt(2)` for the same errors. The evaluator also exports the mean of the per-axis RMSEs, heading error statistics, and relative-motion errors at offsets of 1 and 10 matched frames. These metrics must not be interchanged.

Each method's independently fitted SE(2) result is labeled as a secondary diagnostic. The primary comparison retains the common initial-pose alignment. Joint and reintegrated poses are checked for agreement.

For an existing result:

```bash
./tools/docker.sh --mount-input /absolute/gt_directory exec \
  python3 /opt/occupancy-slam/tools/evaluate_2d.py \
  --result /absolute/pipeline --ground-truth /absolute/gt_directory/gt.txt \
  --output evaluation
```

GT is `x y yaw`, or `timestamp x y yaw`. Use `--indices` with three-column GT when selecting frames. Per-frame CSV files preserve the matched GT rows and whether the frame contributes to absolute metrics.

## Map evaluation and aggregation

For a complete benchmark:

```bash
./tools/docker.sh --run-name simulation_maps exec \
  python3 /opt/occupancy-slam/tools/evaluate_simulation_maps.py \
  --converted "$occupancy_converted" --results "$occupancy_runs/simulation_benchmark"
./tools/docker.sh --run-name simulation_summary exec \
  python3 /opt/occupancy-slam/tools/summarize_matlab_simulations.py \
  --converted "$occupancy_converted" --results "$occupancy_runs/simulation_benchmark" \
  --maps "$occupancy_runs/simulation_maps"
```

Map evaluation uses native M/N fields and the simulator's independent geometry, not RViz classifications. It extracts probability contours at 0.5 and 0.65, samples predicted contours at 0.01 m, and samples reference geometry at 0.005 m. Precision is evaluated against the full true boundary; recall uses true boundary samples within 0.10 m of geometry-projected raw GT hits. Reported quantities include directional MAE and 95th percentile, symmetric distance capped at 1 m, and precision/recall/F1 at 0.10 and 0.20 m.

All three map states use the same trajectory alignment. No map ICP or method-specific map alignment is performed. The aggregation tool writes per-case/group CSV, LaTeX, and PNG/PDF outputs. Report grouping and repeated initializations explicitly. The stored grid resolution and observation sampling are part of the experimental setup.

## Scope of reported evidence

Repository tests establish the covered software behavior. Dataset-level accuracy requires matched observations, initialization, parameter settings, and evaluation conventions. A smaller optimization objective or a visually cleaner classified map is insufficient evidence of a ground-truth improvement. The qualitative images linked in the README belong to the original research release; quantitative results for the publications should be interpreted using their stated protocols.
