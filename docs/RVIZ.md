# RViz2 visualization

The viewer publishes saved results as standard ROS2 messages in one coordinate frame. It opens a single map/cloud view; switch the display's **Topic** to compare methods while preserving the camera.

```bash
./tools/occupancy-slam view 2d --result /absolute/pipeline --display-resolution 0.05
./tools/occupancy-slam view 3d --result /absolute/optimized
```

For Docker, use `./tools/docker.sh --gui view ...`. Build the native display extension with `bash tools/build_rviz.sh` through `tools/run.py` before running the native viewer.

## Topics

| Method | 2D OccupancyGrid | 3D PointCloud2 |
| --- | --- | --- |
| Input poses | `/occupancy_slam/initial/map_display` | `/occupancy_slam/initial/points` |
| Joint optimization | `/occupancy_slam/optimized/map_display` | `/occupancy_slam/optimized/points` |
| Final-pose reintegration | `/occupancy_slam/reintegrated/map_display` | `/occupancy_slam/reintegrated/points` |

Each method also publishes `trajectory` as `nav_msgs/msg/Path`. Raw 2D probabilities are available at `/occupancy_slam/{method}/map`. Three-dimensional `occupancy` topics carry occupancy grid vertices as PointCloud2.

Double-click the Topic value in RViz's Map or PointCloud2 display, then select the desired topic. Path is optional and has its own topic selector. The default method is `reintegrated` for 2D and `optimized` for 3D; override it with `--method initial|optimized|reintegrated`.

Use Fixed Frame `map` and QoS **Reliable / Transient Local / Keep Last / Depth 1**. Keep the publisher running when switching topics. `--frame`, `--namespace`, and `--domain-id` customize the frame, topic prefix, and ROS domain.

## 2D display

The Map extension uses black for obstacles, white for free space, and neutral gray for unknown space. It inherits RViz Map and changes only the unknown palette entry; the message remains `nav_msgs/msg/OccupancyGrid` with normal unknown semantics.

The classified display follows the original implementation's accumulated-log-odds rule `sigmoid(M)`:

| Condition within observed support | OccupancyGrid value | Color |
| --- | --- | --- |
| Probability <= 0.35 | 0 | White |
| Probability >= 0.65 | 100 | Black |
| Otherwise, or unobserved | -1 | Gray |

The raw probability topic uses `sigmoid(M/N)` and quantizes to 0-100. Select it directly or use `--map-view probability`. Classification is a visualization convention and is not a change to the numerical map or an accuracy metric.

`--display-resolution 0.05` (default) or `0.1` independently selects initial/final-pose display integration spacing. The directly optimized display retains the native optimization grid. The exporter uses original observations with fixed poses and records the actual grid metadata. A finer display does not imply finer sensor or pose accuracy.

The initial/reintegrated support mask comes from unsmoothed integration counts. Joint-map support uses the actual observation interpolation support. The viewer does not fill walls, dilate obstacles, or modify optimized map variables. See [map outputs](MAP_OUTPUTS.md).

## 3D display

The default PointCloud2 view shows sampled input points transformed by the selected poses. Its `rgb` field uses a blue/cyan/yellow/orange height palette; select Color Transformer `RGB8`. All methods use the same height interval, determined by the initial cloud's 1st and 99th percentiles, or by the optimized cloud if no initial cloud is available. Set a physical range explicitly with `--height-range MIN_Z MAX_Z`.

Points outside the color interval are color-clipped, not removed. `--max-points` defaults to 200000 per cloud with deterministic sampling; `0` displays all points. This limit affects visualization only.

`--show-occupancy` displays grid vertices instead of transformed scan points. `--occupancy-threshold` defaults to 0.6. Their `intensity` field stores probability. The two final-pose point clouds are identical; direct versus reintegrated map differences are visible in the occupancy fields.

## Saved results and process control

A 2D input directory contains the prepared observations, poses, timestamps, backend configuration, and `optimized/`. A 3D input is the backend output directory. `map_outputs.json` identifies the states. For older results, `--initial` can supply a separate initial state with matching frames.

The generated manifest records source files, point counts, message types, frame, colors, thresholds, and display grids. Messages use the current publication time (`use_sim_time=false`); original timestamps remain in the saved data. The static display TF registers a reference origin and does not represent robot odometry.

Closing RViz or pressing Ctrl+C stops its publisher and child RViz process. `--duration 20` limits execution; `--no-rviz` publishes without opening a window. For a separate RViz instance, use the same ROS domain and compatible QoS.

Graphical checks are in `tests/test_rviz.py` and `tests/test_rviz_topics.py`; they require actual result directories and an X11 session. Earlier Jazzy/Mesa checks observed an isolated MapDisplay shader startup diagnostic with successful rendering and clean exit. Persistent rendering errors should be investigated separately.
