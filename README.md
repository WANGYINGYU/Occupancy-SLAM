# Occupancy-SLAM

C++ implementation of:

- [Occupancy-SLAM: An Efficient and Robust Algorithm for Simultaneously Optimizing Robot Poses and Occupancy Map (IEEE T-RO)](https://arxiv.org/pdf/2502.06292)
- [Occupancy-SLAM: Simultaneously Optimizing Robot Poses and Continuous Occupancy Map (RSS 2022)](https://www.roboticsproceedings.org/rss18/p003.pdf)

Authors: Yingyu Wang, Liang Zhao, Shoudong Huang.

**For the standalone version without ROS2, use the [C_plus_plus branch](https://github.com/WANGYINGYU/Occupancy-SLAM/tree/C_plus_plus).** The `main` branch provides the ROS2 integration and Docker workflow.

## Overview

Occupancy-SLAM jointly optimizes:

- robot poses
- occupancy values on map grid vertices

Unlike pipelines that optimize poses first and then build maps, this method solves pose and occupancy together in one optimization framework.

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

Each run name must be unused. Regression tests are disabled by default; `BUILD_TESTING=ON` enables them. `BUILD_3D=OFF` omits PCL; `LIBIGL_INCLUDE_DIR` selects an existing libigl include directory. Native usage is through `./tools/occupancy-slam`; see [pipeline details](docs/PIPELINES.md). Keep the virtual environment active in each new terminal. `--system-site-packages` makes the system ROS2 Python modules available to it; do not use a different Python minor version.

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

The optional tests check solver behavior, the N approximation, input adapters, and map exports. They generate their own fixtures and do not require personal datasets, historical results, or a desktop session. Normal builds and the default Docker image omit the test binaries; the default image also omits `tests/`.

To build a separate test image and run the checks:

```bash
./tools/docker.sh --image occupancy-slam:test build --build-arg BUILD_TESTING=ON
./tools/docker.sh --image occupancy-slam:test test unit
./tools/docker.sh --image occupancy-slam:test test inputs
./tools/docker.sh --image occupancy-slam:test test architecture
```

Native test setup and suite coverage are in the [validation guide](docs/VALIDATION.md). These checks do not establish reproduction of all paper benchmarks.

The simulation workflow converts the original MATLAB data while preserving stored observations and supplied initializations, runs the C++ optimizer, and evaluates trajectories and maps against independent ground truth. It uses one shared initial-pose SE(2) alignment for the primary trajectory comparison, excludes the fixed first pose, and reports the final accepted state. The conversion, benchmark, map evaluation, and aggregation commands are in the [reproducibility guide](docs/VALIDATION.md). Raw simulation data and external MATLAB functions must be supplied separately.

## Original standalone and MATLAB implementations

The original interactive 2D C++ application is available in [C_plus_plus](https://github.com/WANGYINGYU/Occupancy-SLAM/tree/C_plus_plus). That branch also retains the [preliminary MATLAB 3D optimizer and submap joining code](https://github.com/WANGYINGYU/Occupancy-SLAM/tree/C_plus_plus/3D_Preliminary), their data conventions, parameters, and execution instructions. Their supported options differ from the ROS2 C++ pipeline in this branch.

## Showcase Video

[![Occupancy-SLAM Video](https://img.youtube.com/vi/-_-39SrPxKk/maxresdefault.jpg)](https://www.youtube.com/watch?v=-_-39SrPxKk&t=4s)

## More Results

### More datasets

| Dataset | Initialization | Ours |
|---|---|---|
| ACES | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/ACES_Ini.jpg?raw=true) | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/ACES_Our.jpg?raw=true) |
| Intel Lab | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/Intel_Ini.jpg?raw=true) | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/Intel_Our.jpg?raw=true) |
| C5 | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/C5_Ini.jpg?raw=true) | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/C5_Our.jpg?raw=true) |
| Museum b0 | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/b0_Ini.jpg?raw=true) | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/b0_Our.jpg?raw=true) |
| Freiburg Building 079 | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/fr079_Ini.jpg?raw=true) | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/fr079_Our.jpg?raw=true) |
| Garage | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/garage_Ini.jpg?raw=true) | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/garage_Our.jpg?raw=true) |
| MIT | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/MIT_Ini.jpg?raw=true) | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/MIT_Our.jpg?raw=true) |
| Simu 1 | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/Simu1_Ini.jpg?raw=true) | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/Simu1_Our.jpg?raw=true) |
| Simu 2 | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/Simu2_Ini.jpg?raw=true) | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/Simu2_Our.jpg?raw=true) |
| Simu 3 | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/Simu3_Ini.jpg?raw=true) | ![](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/Simu3_Our.jpg?raw=true) |

### High-frequency scan comparison

We compare different methods using high-frequency scans on one simulation dataset.

![High-Frequency Scan Results](https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/High_Frequency.png?raw=true)

## Citation

```bibtex
@article{wang2025occupancy,
  author={Wang, Yingyu and Zhao, Liang and Huang, Shoudong},
  journal={IEEE Transactions on Robotics},
  title={Occupancy-SLAM: An Efficient and Robust Algorithm for Simultaneously Optimizing Robot Poses and Occupancy Map},
  year={2025},
  pages={1-20},
  doi={10.1109/TRO.2025.3578227}
}
```

```bibtex
@INPROCEEDINGS{Zhao-RSS-22,
  AUTHOR    = {Liang Zhao AND Yingyu Wang AND Shoudong Huang},
  TITLE     = {{Occupancy-SLAM: Simultaneously Optimizing Robot Poses and Continuous Occupancy Map}},
  BOOKTITLE = {Proceedings of Robotics: Science and Systems},
  YEAR      = {2022},
  ADDRESS   = {New York City, NY, USA},
  MONTH     = {June},
  DOI       = {10.15607/RSS.2022.XVIII.003}
}
```

## License

This repository is distributed under the [MIT License](LICENSE.txt).
