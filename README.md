# Occupancy-SLAM

Research code for **joint optimization of robot trajectories and occupancy maps**.

**Occupancy-SLAM: An Efficient and Robust Algorithm for Simultaneously Optimizing Robot Poses and Occupancy Map**<br>
Yingyu Wang, Liang Zhao, and Shoudong Huang<br>
*IEEE Transactions on Robotics*, vol. 41, pp. 4057-4077, 2025.<br>
[Paper](https://doi.org/10.1109/TRO.2025.3578227) | [Preprint](https://arxiv.org/abs/2502.06292) | [Video](https://www.youtube.com/watch?v=-_-39SrPxKk)

**Occupancy-SLAM: Simultaneously Optimizing Robot Poses and Continuous Occupancy Map**<br>
Liang Zhao, Yingyu Wang, and Shoudong Huang<br>
*Robotics: Science and Systems (RSS)*, 2022.<br>
[Paper and supplementary material](https://www.roboticsproceedings.org/rss18/p003.html)

## Overview

Occupancy-SLAM estimates robot poses and occupancy values at grid vertices in a common optimization problem. The 2D backend operates on a batch of scans and supports coarse-to-fine optimization. The normalization term N is held fixed when computing pose Jacobians and is recomputed after each accepted update, retaining the approximation used in the method.

This branch provides ROS2 integration around the C++ backends: ROS1/ROS2 bag input, original 2D text input, 3D PCD/trajectory input, Docker execution, and RViz2 visualization. The sparse C++ 3D backend is an additional implementation; the preliminary MATLAB 3D optimizer and submap joining code are provided separately in [3D_Preliminary/](3D_Preliminary/). Their supported options differ.

| Branch | Implementation |
| --- | --- |
| [main](https://github.com/WANGYINGYU/Occupancy-SLAM/tree/main) | C++ backends with ROS2 input and visualization interfaces, Docker, and evaluation tools. |
| [C_plus_plus](https://github.com/WANGYINGYU/Occupancy-SLAM/tree/C_plus_plus) | Original standalone C++ implementation. **Use this branch if you do not use ROS2.** |

## Installation

The supported integration environment is **Ubuntu 24.04 and ROS2 Jazzy**. The C++ build requires CMake >= 3.16, a C++17 compiler, Eigen >= 3.4, OpenMP, and libigl headers (tested with v2.5.0). PCL is required for 3D input. ROS2 supplies `slam_toolbox` and RViz2; the numerical backends are independent of ROS.

```bash
git clone --branch main https://github.com/WANGYINGYU/Occupancy-SLAM.git
cd Occupancy-SLAM
```

### Docker

The image includes ROS2, the C++ backends, RViz2, and the Python conversion and evaluation dependencies. The host requires Linux, Python 3, and Docker Engine; a host ROS2 installation is unnecessary.

```bash
export OCCUPANCY_DATA_ROOT=/mnt/data   # Existing writable data-disk mount
./tools/docker.sh build
```

The wrappers write results, logs, caches, and temporary files below `$OCCUPANCY_DATA_ROOT/experiments/sdf_ba/codex_runs/`. This variable must identify a mounted data disk. Docker image/build storage must also reside on that disk. Configure Docker's `data-root` and, if applicable, the separate containerd image store before building; see the [Docker guide](docs/DOCKER.md).

### Native build

After installing [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html):

```bash
sudo apt-get install build-essential cmake git libeigen3-dev libpcl-dev \
  python3-venv python3-pip ros-jazzy-slam-toolbox ros-jazzy-rviz2 \
  ros-jazzy-rmw-fastrtps-cpp
git clone --depth 1 --branch v2.5.0 https://github.com/libigl/libigl.git third_party/libigl
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r requirements-matlab.txt
source /opt/ros/jazzy/setup.bash
export OCCUPANCY_DATA_ROOT=/mnt/data
python3 tools/run_on_data.py --name native_build -- \
  bash "$PWD/tools/build.sh" -DBUILD_3D=ON -DBUILD_LEGACY=OFF
python3 tools/run_on_data.py --name rviz_build -- bash "$PWD/tools/build_rviz.sh"
```

Each run name must be unused. `BUILD_3D=OFF` omits PCL; `LIBIGL_INCLUDE_DIR` selects an existing libigl include directory. `BUILD_LEGACY=ON` additionally requires OpenCV and builds the original interactive application. Native usage is through `./tools/occupancy-slam`; see [pipeline details](docs/PIPELINES.md).

## Running the examples

### 2D text input

The repository includes example laser scans and initial trajectories in [Data/](Data/).

```bash
./tools/docker.sh --run-name intel_demo txt \
  --dataset Data/Intel --backend-config config/backend/intel.txt \
  --max-scans 20 --iterations 5 --display-resolution 0.05
./tools/docker.sh --gui view 2d \
  --result "$OCCUPANCY_DATA_ROOT/experiments/sdf_ba/codex_runs/intel_demo/pipeline"
```

For a complete dataset, omit `--max-scans`. Select the scan geometry and optimization parameters appropriate to that dataset. The short command above is a functional example.

### ROS1 or ROS2 bag input

```bash
./tools/docker.sh inspect /absolute/input.bag
./tools/docker.sh bag --config config/inputs/rosbag_laserscan.yaml \
  --bag /absolute/input.bag --scan-topic /robot/front/scan \
  --odom-source odom --odom-topic /robot/wheel/odom
```

Use [rosbag_pointcloud.yaml](config/inputs/rosbag_pointcloud.yaml) for PointCloud2 and configure its projection frame, height range, and angular bins. ROS2 input is a bag directory. Scan, odometry, and TF topics are configurable; no ROS1 installation or `ros1_bridge` is needed. When external odometry/TF is unavailable, `--odom-source scan-match` generates a scan-matching prior for `slam_toolbox`. Exported poses are associated by scan timestamp during processing; final loop-closed graph poses are not re-exported to replace this trajectory.

### 3D point clouds and poses

```bash
./tools/docker.sh --run-name pcd_demo pcd \
  --config config/inputs/pcd.yaml --dataset /absolute/NC20_sequence \
  --start 20 --stride 3 --max-frames 40
./tools/docker.sh --gui view 3d \
  --result "$OCCUPANCY_DATA_ROOT/experiments/sdf_ba/codex_runs/pcd_demo/optimized"
```

The NC20 layout uses `odometry.txt` and `PCD/full0.pcd`, `full1.pcd`, etc. General inputs can use `--poses /absolute/poses.tum --manifest /absolute/scans.txt`. Each manifest row contains `timestamp pcd_path`.

## Data and configuration

| Input | Convention |
| --- | --- |
| `Range.txt` | One scan per row; one range in metres per beam, in increasing angular order. |
| `Pose.txt` | Absolute sensor poses `x y yaw`, in metres and radians. |
| `Odom.txt` | Optional: an absolute first pose, followed by SE(2) increments in the preceding sensor frame. |
| `timestamps.txt` | Optional strictly increasing timestamps in seconds. |
| 3D poses | TUM rows `timestamp x y z qx qy qz qw`; poses transform sensor coordinates into world coordinates. |
| PCD | Individual scans in sensor coordinates. |

Input profiles are in [config/inputs/](config/inputs/); C++ 2D profiles are in [config/backend/](config/backend/); frontend settings are in [config/slam_toolbox.yaml](config/slam_toolbox.yaml). CLI options override YAML. YAML input paths are relative to the configuration file; CLI input paths are relative to the calling directory. Detailed conventions and solver settings are documented in [config/README.md](config/README.md).

## Outputs and visualization

Each run exports three map states:

| Directory | Poses | Occupancy field |
| --- | --- | --- |
| `initial/` | Input | Initial integration. |
| `joint/` | Optimized | Directly optimized map variables. |
| `reintegrated/` | Same optimized poses | New integration of the original observations with poses fixed. |

These directories are under `pipeline/optimized/` for 2D and `optimized/` for 3D. Numerical maps, trajectories, parameters, and execution status are saved alongside them. See [output definitions](docs/MAP_OUTPUTS.md).

RViz2 compares the states in a common coordinate frame through its **Topic** selector. The method names are `initial`, `optimized` (corresponding to `joint/`), and `reintegrated`.

| Topic suffix under `/occupancy_slam/{method}/` | ROS2 message |
| --- | --- |
| `map_display` (2D) | `nav_msgs/msg/OccupancyGrid` |
| `points` (3D) | `sensor_msgs/msg/PointCloud2` |
| `trajectory` | `nav_msgs/msg/Path` |

Initial and reintegrated 2D display maps default to **0.05 m** resolution; **0.1 m** or another positive value can be selected independently of the optimization grid. The directly optimized field retains its native resolution. Three-dimensional points are colored by height. The final-pose point clouds coincide for `joint` and `reintegrated`; their occupancy fields can be inspected with `--show-occupancy`. See the [RViz guide](docs/RVIZ.md).

## Reproducibility and evaluation

```bash
./tools/docker.sh test unit
./tools/docker.sh test inputs
./tools/docker.sh test architecture
```

Native equivalents are `./tools/test.sh unit`, `inputs`, and `architecture`. Tests cover the normalization approximation, solvers, scan geometry, input adapters, evaluation conventions, and map export. They do not establish reproduction of all paper benchmarks.

The simulation workflow converts the original MATLAB data while preserving stored observations and supplied initializations, runs the C++ optimizer, and evaluates trajectories and maps against independent ground truth. It uses one shared initial-pose SE(2) alignment for the primary trajectory comparison, excludes the fixed first pose, and reports the final accepted state. The conversion, benchmark, map evaluation, and aggregation commands are in the [reproducibility guide](docs/VALIDATION.md). Raw simulation data and external MATLAB functions must be supplied separately.

## Qualitative results

The following images are examples distributed with the original research implementation. Dataset-level quantitative results and experimental protocols are reported in the papers.

| Intel Lab initialization | Occupancy-SLAM |
| --- | --- |
| ![Initial Intel Lab map](images/Intel_Ini.jpg) | ![Optimized Intel Lab map](images/Intel_Our.jpg) |

<details>
<summary>Additional qualitative examples</summary>

| Dataset | Initialization | Occupancy-SLAM |
|---|---|---|
| ACES | ![Initial ACES map](images/ACES_Ini.jpg) | ![Optimized ACES map](images/ACES_Our.jpg) |
| C5 | ![Initial C5 map](images/C5_Ini.jpg) | ![Optimized C5 map](images/C5_Our.jpg) |
| Museum b0 | ![Initial b0 map](images/b0_Ini.jpg) | ![Optimized b0 map](images/b0_Our.jpg) |
| Freiburg Building 079 | ![Initial fr079 map](images/fr079_Ini.jpg) | ![Optimized fr079 map](images/fr079_Our.jpg) |
| Garage | ![Initial garage map](images/garage_Ini.jpg) | ![Optimized garage map](images/garage_Our.jpg) |
| MIT | ![Initial MIT map](images/MIT_Ini.jpg) | ![Optimized MIT map](images/MIT_Our.jpg) |
| Simu 1 | ![Initial Simu1 map](images/Simu1_Ini.jpg) | ![Optimized Simu1 map](images/Simu1_Our.jpg) |
| Simu 2 | ![Initial Simu2 map](images/Simu2_Ini.jpg) | ![Optimized Simu2 map](images/Simu2_Our.jpg) |
| Simu 3 | ![Initial Simu3 map](images/Simu3_Ini.jpg) | ![Optimized Simu3 map](images/Simu3_Our.jpg) |

### High-frequency scan comparison

High-frequency scan comparison distributed with the original implementation.

![High-Frequency Scan Results](images/High_Frequency.png)

</details>

## Citation

If you use this work, please cite the relevant publication:

```bibtex
@article{wang2025occupancy,
  author  = {Wang, Yingyu and Zhao, Liang and Huang, Shoudong},
  title   = {Occupancy-SLAM: An Efficient and Robust Algorithm for Simultaneously Optimizing Robot Poses and Occupancy Map},
  journal = {IEEE Transactions on Robotics},
  volume  = {41},
  pages   = {4057--4077},
  year    = {2025},
  doi     = {10.1109/TRO.2025.3578227}
}

@inproceedings{zhao2022occupancy,
  author    = {Zhao, Liang and Wang, Yingyu and Huang, Shoudong},
  title     = {Occupancy-SLAM: Simultaneously Optimizing Robot Poses and Continuous Occupancy Map},
  booktitle = {Proceedings of Robotics: Science and Systems},
  address   = {New York City, NY, USA},
  month     = {June},
  year      = {2022},
  doi       = {10.15607/RSS.2022.XVIII.003}
}
```

## License

This repository is distributed under the [MIT License](LICENSE.txt).
