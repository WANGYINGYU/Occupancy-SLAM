# Map states and output definitions

Every completed 2D/3D backend run exports three states. Their canonical directories are identified by `map_outputs.json`.

| State | Poses | Map construction |
| --- | --- | --- |
| `initial/` | Input poses | Initial integration before optimization. |
| `joint/` | Final optimized poses | Map variables directly updated by the joint solver. |
| `reintegrated/` | Same final optimized poses | New accumulation of original observations with poses fixed. |

Reintegration starts from an empty field and does not read the optimized field values. It retains the observation model, input sampling, native grid, and supported domain. Root-level compatibility files correspond to `joint/`.

## 2D native outputs

The three states live under `pipeline/optimized/`. Each saves `poses.txt`, `occupancy.txt`, `normalization.txt`, `map.yaml`, `map.pgm`, and `trajectory.tum` when timestamps are available.

`occupancy.txt` stores the map variable M, and `normalization.txt` stores N, in double precision. The native probability field is `sigmoid(M / max(N, 1e-9))`. PGM files are display exports; the matrices are the numerical results.

Initial and reintegrated fields use `InitialMapRound` for observation binning and the configured N smoothing. Their `integration_counts.txt` stores counts before smoothing. Iterative N refresh uses `NormalizationRound`, recomputes counts from zero, and retains the paper's omitted N pose derivative.

The three native fields share grid bounds and origin. Unknown support for display is derived from actual observations, not from smoothed N, which may be positive outside observed regions.

## Independent 2D display resolution

`--display-resolution` controls only initial-pose and final-pose display integration. It defaults to 0.05 m and may be set to 0.1 m or another positive spacing. The exporter integrates the original observations again on a shared display grid; it does not enlarge the coarse map image by interpolation.

The directly optimized field retains its native resolution. Native matrices and raw probability topics are preserved. Display exports contain `map.yaml`, `map.pgm`, and `grid.npz`, together with a manifest recording resolution, dimensions, origin, classification, and integration settings.

The public pipeline creates `pipeline/display_maps/`. Viewers reuse a persistent cache or create a compatible export when another resolution is requested. Cache invalidation depends on source metadata. The exporter can also run without ROS:

```bash
./tools/occupancy-slam export --result /absolute/pipeline \
  --resolution 0.1 --output maps_010
```

## 3D outputs

Each state in the 3D output directory contains:

| File | Contents |
| --- | --- |
| `trajectory.tum` | World-frame sensor poses. |
| `selected_scans.txt` | Selected sensor scans and their associations. |
| `map_variables.txt` | `key_x key_y key_z log_odds`, preserving numerical variables. |
| `grid.json`, `world_from_grid.txt` | Grid spacing and world/grid transform. |
| `occupancy_vertices.pcd` | World-frame grid vertices; intensity is occupancy probability. |
| `optimized_points.pcd` | Input sample points transformed by the state's poses. |
| `integration_weights.txt` | Initial/reintegrated accumulation weights. |

World coordinates are `T_world_grid * (voxel_size * grid_key)`. Occupancy vertices are grid samples, not a surface mesh.

Reintegration accumulates trilinear interpolation weight times observation target and the corresponding weights, including the free-space normalization used by the backend. A supported vertex receives their ratio; unsupported vertices remain zero. It uses the retained sensor-frame hit/free observations, not the already transformed output cloud.

`joint/` and `reintegrated/` have identical final poses and therefore identical transformed point clouds. Their occupancy fields can differ. Use those fields, rather than the point cloud alone, to study the effect of directly optimizing map variables.

## Interpretation and visualization

RViz topic method `optimized` refers to the `joint/` state. All states share the display coordinate frame. [RViz](RVIZ.md) documents topic selection and visual thresholds. `tools/compare_maps.py` provides static map comparison plots; quantitative simulation evaluation uses native fields and independent geometry as described in [VALIDATION.md](VALIDATION.md).

`run_status.json` distinguishes completed, failed, and interrupted execution. `summary.json` records the solver stop reason and iteration statistics. Neither successful completion nor a lower optimization objective establishes improved ground-truth accuracy.
