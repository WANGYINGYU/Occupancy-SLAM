# Occupancy-SLAM

Research code for **joint optimization of robot trajectories and occupancy maps**.

**For the standalone version without ROS2, use the [C_plus_plus branch](https://github.com/WANGYINGYU/Occupancy-SLAM/tree/C_plus_plus).** The `main` branch provides the ROS2 integration and Docker workflow.

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

This branch provides ROS2 integration around the C++ backends: ROS1/ROS2 bag input, original 2D text input, 3D PCD/trajectory input, Docker execution, and RViz2 visualization. This branch contains the ROS2 workflows and their C++ backends, configuration, tests, and documentation. The standalone C++ application and preliminary MATLAB 3D implementation are maintained in [C_plus_plus](https://github.com/WANGYINGYU/Occupancy-SLAM/tree/C_plus_plus).

## Branch selection

| Branch | Implementation |
| --- | --- |
| [main](https://github.com/WANGYINGYU/Occupancy-SLAM/tree/main) | C++ backends with ROS2 input and visualization interfaces, Docker, and evaluation tools. |
| [C_plus_plus](https://github.com/WANGYINGYU/Occupancy-SLAM/tree/C_plus_plus) | Original standalone C++ implementation. **Use this branch if you do not use ROS2.** |

To obtain the original standalone C++ version without ROS2:

```bash
git clone --branch C_plus_plus https://github.com/WANGYINGYU/Occupancy-SLAM.git
cd Occupancy-SLAM
```

For an existing clone, switch branches from the repository directory:

```bash
git fetch origin
git switch C_plus_plus
```

Follow the installation and execution instructions in that branch's README. The instructions below apply to `main`.

## Repository structure

| Path | Purpose |
| --- | --- |
| [cpp/](cpp/), [src/](src/), [include/occupancy/](include/occupancy/) | Batch C++ applications, optimization, scan I/O, map export, and backend interfaces. |
| [FuncConvertObs.cpp](FuncConvertObs.cpp) | 2D observation conversion. |
| [MyStruct.h](MyStruct.h), [MyStruct.cpp](MyStruct.cpp) | 2D parameter structures and defaults. |
| [SubFuncs.h](SubFuncs.h), [SubFuncs.cpp](SubFuncs.cpp) | Shared 2D optimization and mapping utilities, including configuration loading. |
| [config/](config/) | Defaults and profiles for input adapters, C++ backends, RViz2, and `slam_toolbox`. |
| [tools/](tools/), [ros2/](ros2/), [docker/](docker/) | Pipeline commands, data conversion and evaluation, ROS2 visualization, and container setup. |
| [tests/](tests/), [docs/](docs/) | Correctness checks and detailed usage documentation. |

## Installation

The supported integration environment is **Ubuntu 24.04 and ROS2 Jazzy**. The C++ build requires CMake >= 3.16, a C++17 compiler, Eigen >= 3.4, OpenMP, and libigl headers (tested with v2.5.0). PCL is required for 3D input. ROS2 supplies `slam_toolbox` and RViz2; the numerical backends are independent of ROS.

```bash
git clone --branch main https://github.com/WANGYINGYU/Occupancy-SLAM.git
cd Occupancy-SLAM
```

### Docker

The image includes ROS2, the C++ backends, RViz2, and the Python conversion and evaluation dependencies. The host requires Linux, Python >= 3.10, and [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) accessible to the current user (`docker info` must succeed). A host ROS2 installation is unnecessary.

```bash
export OCCUPANCY_WORKSPACE="$HOME/occupancy-slam-workspace"
./tools/docker.sh build
```

The workspace is an ordinary writable directory, created automatically. It defaults to `~/occupancy-slam-workspace`; set `OCCUPANCY_WORKSPACE` to choose another location. Runs are saved in `runs/` and reusable display maps in `cache/`. No separate disk mount or Docker storage reconfiguration is required. See the [Docker guide](docs/DOCKER.md).

### Native build

On Ubuntu 24.04, install [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) first. Run the following commands from the cloned repository using the system Python 3.12:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake git libeigen3-dev libpcl-dev \
  python3-venv python3-pip python3-numpy python3-scipy python3-yaml python3-pil \
  ros-jazzy-slam-toolbox ros-jazzy-rviz2 \
  ros-jazzy-rmw-fastrtps-cpp
git clone --depth 1 --branch v2.5.0 https://github.com/libigl/libigl.git third_party/libigl
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
source /opt/ros/jazzy/setup.bash
export OCCUPANCY_WORKSPACE="$HOME/occupancy-slam-workspace"
python3 tools/run.py --name native_build -- \
  bash "$PWD/tools/build.sh" -DBUILD_3D=ON
python3 tools/run.py --name rviz_build -- bash "$PWD/tools/build_rviz.sh"
```

Each run name must be unused. `BUILD_3D=OFF` omits PCL; `LIBIGL_INCLUDE_DIR` selects an existing libigl include directory. Native usage is through `./tools/occupancy-slam`; see [pipeline details](docs/PIPELINES.md). Keep the virtual environment active in each new terminal. `--system-site-packages` makes the system ROS2 Python modules available to it; do not use a different Python minor version.

| Python dependency file | Purpose |
| --- | --- |
| [requirements.txt](requirements.txt) | TXT/bag/PCD workflows, map export, RViz publishing, and functional tests: NumPy, PyYAML, SciPy, rosbags, Pillow, and contourpy. |
| [requirements-evaluation.txt](requirements-evaluation.txt) | Optional simulation conversion and research figures; includes the runtime requirements and adds h5py and Matplotlib. No MATLAB runtime is required. |

Install the second file only when using those evaluation tools: `python3 -m pip install -r requirements-evaluation.txt`. Docker includes both sets. ROS2 Python packages come from the ROS installation, not pip.

## Running the examples

### 2D text input

Example scans and trajectories are stored in the [standalone branch](https://github.com/WANGYINGYU/Occupancy-SLAM/tree/C_plus_plus/Data). Download only the dataset needed for a run; datasets are not bundled with `main` or the Docker image.

```bash
./tools/docker.sh --run-name example_intel exec \
  python3 /opt/occupancy-slam/tools/download_example.py --example Intel
export OCCUPANCY_INTEL_DATA="$OCCUPANCY_WORKSPACE/runs/example_intel/Intel"
```

For native use, run `tools/download_example.py` through `tools/run.py`. The downloader records the versioned source URLs alongside the data.

```bash
./tools/docker.sh --run-name intel_demo txt \
  --dataset "$OCCUPANCY_INTEL_DATA" --backend-config config/backend/intel.txt \
  --max-scans 20 --iterations 5 --display-resolution 0.05
./tools/docker.sh --gui view 2d \
  --result "$OCCUPANCY_WORKSPACE/runs/intel_demo/pipeline"
```

The Museum b0 example from the original implementation is also supported:

```bash
./tools/docker.sh --run-name example_museum exec \
  python3 /opt/occupancy-slam/tools/download_example.py --example Museum_b0
./tools/docker.sh --run-name museum_demo txt \
  --dataset "$OCCUPANCY_WORKSPACE/runs/example_museum/Museum_b0" \
  --backend-config config/backend/museum_b0.txt \
  --max-scans 20 --iterations 5 --display-resolution 0.05
```

For a complete dataset, omit `--max-scans`. Select the scan geometry and optimization parameters appropriate to that dataset. These short commands are functional examples.

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
  --result "$OCCUPANCY_WORKSPACE/runs/pcd_demo/optimized"
```

The NC20 layout uses `odometry.txt` and `PCD/full0.pcd`, `full1.pcd`, etc. General inputs can use `--poses /absolute/poses.tum --manifest /absolute/scans.txt`. Each manifest row contains `timestamp pcd_path`.

## Data and configuration

| Input | Convention |
| --- | --- |
| `Range.txt` | `m x n`: one scan per row; one range in metres per beam, in increasing angular order. |
| `Pose.txt` | `m x 3`: absolute sensor poses `x y yaw`, in metres and radians. |
| `Odom.txt` | Optional `m x 3`: an absolute first pose, followed by SE(2) increments in the preceding sensor frame. |
| `timestamps.txt` | Optional strictly increasing timestamps in seconds. |
| 3D poses | TUM rows `timestamp x y z qx qy qz qw`; poses transform sensor coordinates into world coordinates. |
| PCD | Individual scans in sensor coordinates. |

For 2D text input, `m` is the number of scans and `n` is the number of beams. Scan and pose rows must correspond one-to-one; optional odometry and timestamp files must have the same number of rows. Set `NumBeam`, `MinAngle`, and `MaxAngle` in the backend configuration to match the scan geometry.

### Parameters

The key/value backend defaults are in [config/backend/default.txt](config/backend/default.txt). Parameter structures are defined in [MyStruct.h](MyStruct.h), with defaults in [MyStruct.cpp](MyStruct.cpp) and configuration loading in [SubFuncs.cpp](SubFuncs.cpp). For the batch pipeline, select a dataset-specific file with `--backend-config`; configure that file rather than assuming the original defaults match every sensor.

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

Native equivalents are `./tools/test.sh unit`, `inputs`, and `architecture`. Tests generate their TXT/bag/PCD fixtures in the workspace and cover the normalization approximation, solvers, scan geometry, input adapters, evaluation conventions, and map export. They do not require bundled datasets or network access. They do not establish reproduction of all paper benchmarks.

The simulation workflow converts the original MATLAB data while preserving stored observations and supplied initializations, runs the C++ optimizer, and evaluates trajectories and maps against independent ground truth. It uses one shared initial-pose SE(2) alignment for the primary trajectory comparison, excludes the fixed first pose, and reports the final accepted state. The conversion, benchmark, map evaluation, and aggregation commands are in the [reproducibility guide](docs/VALIDATION.md). Raw simulation data and external MATLAB functions must be supplied separately.

## Original standalone and MATLAB implementations

The original interactive 2D C++ application is available in [C_plus_plus](https://github.com/WANGYINGYU/Occupancy-SLAM/tree/C_plus_plus). That branch also retains the [preliminary MATLAB 3D optimizer and submap joining code](https://github.com/WANGYINGYU/Occupancy-SLAM/tree/C_plus_plus/3D_Preliminary), their data conventions, parameters, and execution instructions. Their supported options differ from the ROS2 C++ pipeline in this branch.

## Showcase video

[![Occupancy-SLAM video](https://img.youtube.com/vi/-_-39SrPxKk/maxresdefault.jpg)](https://www.youtube.com/watch?v=-_-39SrPxKk&t=4s)

## Qualitative results

The following images are referenced from the original research implementation on `C_plus_plus`. Dataset-level quantitative results and experimental protocols are reported in the papers.

| Dataset | Initialization | Occupancy-SLAM |
|---|---|---|
| ACES | ![Initial ACES map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/ACES_Ini.jpg) | ![Optimized ACES map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/ACES_Our.jpg) |
| Intel Lab | ![Initial Intel Lab map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/Intel_Ini.jpg) | ![Optimized Intel Lab map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/Intel_Our.jpg) |
| C5 | ![Initial C5 map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/C5_Ini.jpg) | ![Optimized C5 map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/C5_Our.jpg) |
| Museum b0 | ![Initial b0 map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/b0_Ini.jpg) | ![Optimized b0 map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/b0_Our.jpg) |
| Freiburg Building 079 | ![Initial fr079 map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/fr079_Ini.jpg) | ![Optimized fr079 map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/fr079_Our.jpg) |
| Garage | ![Initial garage map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/garage_Ini.jpg) | ![Optimized garage map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/garage_Our.jpg) |
| MIT | ![Initial MIT map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/MIT_Ini.jpg) | ![Optimized MIT map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/MIT_Our.jpg) |
| Simu 1 | ![Initial Simu1 map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/Simu1_Ini.jpg) | ![Optimized Simu1 map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/Simu1_Our.jpg) |
| Simu 2 | ![Initial Simu2 map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/Simu2_Ini.jpg) | ![Optimized Simu2 map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/Simu2_Our.jpg) |
| Simu 3 | ![Initial Simu3 map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/Simu3_Ini.jpg) | ![Optimized Simu3 map](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/Simu3_Our.jpg) |

### High-frequency scan comparison

Comparison of different methods using high-frequency scans on a simulation dataset, as distributed with the original implementation.

![High-Frequency Scan Results](https://raw.githubusercontent.com/WANGYINGYU/Occupancy-SLAM/C_plus_plus/images/High_Frequency.png)

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
