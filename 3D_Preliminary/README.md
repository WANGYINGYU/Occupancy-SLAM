# 3D Preliminary (MATLAB)

Preliminary MATLAB implementation of 3D Occupancy-SLAM.

This folder contains two modules:

- `3D_Occupancy_SLAM/`: single-map joint optimization of pose and occupancy
- `3D_Occupancy_Joining/`: submap joining for larger environments

## References

- [Occupancy-SLAM (IEEE T-RO)](https://arxiv.org/pdf/2502.06292)
- Video: [YouTube](https://www.youtube.com/watch?v=-_-39SrPxKk&t=4s)

## Environment

Recommended:

- MATLAB R2022b or newer
- Robotics System Toolbox (for occupancy map visualization utilities)
- Parallel Computing Toolbox (optional, for `parfor` acceleration)

## Quick Start

### A) 3D Occupancy-SLAM (recommended first)

1. Open MATLAB and set working directory to:
   `3D_Preliminary/3D_Occupancy_SLAM`
2. Run:

```matlab
Main_3D_Occpancy
```

Default demo input:

- `3D_Occupancy_SLAM/Data/Demo.mat`

Expected variables in the dataset:

- `Pose`: `N x 6` (`x,y,z,roll,pitch,yaw`)
- `Scan`: `1 x N` cell, each cell is a point cloud in the local LiDAR frame

### B) 3D Occupancy Joining (large-scale)

1. Set working directory to:
   `3D_Preliminary/3D_Occupancy_Joining`
2. Run:

```matlab
Main_3D_Occupancy_Joining
```

Note:

- This module assumes submap internal poses are already optimized.
- `Main_3D_Occupancy_Joining.m` references `Data/Voxgraph_Demo.mat`; this file is not included in the current repository snapshot, so prepare your own data with a matching format.

## Key Parameters (3D_Occupancy_SLAM)

Edit in `3D_Occupancy_SLAM/FuncLoadParams.m`.

Core optimization:

- `Param.MaxIter`: max iterations
- `Param.SmoothWeight`: map smoothness weight
- `Param.OptimizerType`: `'GN'` or `'LM'`
- `Param.UsePCGSolver`: `1` enables PCG solver for large sparse systems
- `Param.LambdaO`: overall weight of the odometry term
- `Param.OdomSigma`: odometry standard deviations `[sx, sy, sz, sroll, spitch, syaw]` in meters/radians; smaller means a stronger odometry prior on that axis

Observation preprocessing:

- `Param.UseVoxelDownsample`
- `Param.UseAdaptiveVoxelDownsample`
- `Param.UseGlobalVoxelDensityFilter`
- `Param.UseCoarseOccupancyStabilityFilter`
- `Param.ReprocessObsEachIter`

Truncated optimization:

- `Param.UseTruncatedRegionOptimization`
- `Param.TruncatedRegionLayers`
- `Param.TruncatedObsClipExtraLayers`

Robustness:

- `Param.UseRobustKernel`
- `Param.RobustKernel` (`huber`, etc.)
- `Param.RobustDelta`

Convergence:

- `Param.PoseThreshold`
- `Param.ObsThreshold`
- `Param.ScanOnlyDecreaseWindow`
- `Param.ScanOnlyDecreaseTol`

## Outputs

Typical outputs are controlled in `FuncLoadParams.m`:

- `Param.FileDict`: output directory
- `Param.SavePCD`: save reconstructed point cloud
- `Param.VisualizationPC`: point cloud visualization
- `Param.VisualizationOGM`: occupancy-map visualization

## Data Preparation Notes

For your own dataset:

- Keep strict time alignment between `Pose(i,:)` and `Scan{i}`.
- Use consistent units (meters, radians).
- Ensure each scan is in the local LiDAR frame expected by the scripts.
- Start with conservative settings (`UseTruncatedRegionOptimization=0`, `UsePCGSolver=0`) and then enable acceleration modes.

## Example Qualitative Results

| Odometry | Ours |
|---|---|
| ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_%20Preliminary/Odom_1.png?raw=true) | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_%20Preliminary/Ours_1.png?raw=true) |
| ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_%20Preliminary/Odom_2.png?raw=true) | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_%20Preliminary/Ours_2.png?raw=true) |
| ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_%20Preliminary/Odom_3.png?raw=true) | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_%20Preliminary/Ours_3.png?raw=true) |
| ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_%20Preliminary/Odom_4.png?raw=true) | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_%20Preliminary/Ours_4.png?raw=true) |
| ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_%20Preliminary/Odom_5.png?raw=true) | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_%20Preliminary/Ours_5.png?raw=true) |
| ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_%20Preliminary/Odom_6.png?raw=true) | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_%20Preliminary/Ours_6.png?raw=true) |

## Status

This MATLAB version is a preliminary implementation. A more optimized C++ implementation is planned.
