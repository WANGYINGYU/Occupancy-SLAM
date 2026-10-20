# Occupancy-SLAM

C++ implementation of:

- [Occupancy-SLAM: An Efficient and Robust Algorithm for Simultaneously Optimizing Robot Poses and Occupancy Map (IEEE T-RO)](https://arxiv.org/pdf/2502.06292)
- [Occupancy-SLAM: Simultaneously Optimizing Robot Poses and Continuous Occupancy Map (RSS 2022)](https://www.roboticsproceedings.org/rss18/p003.pdf)

Authors: Yingyu Wang, Liang Zhao, Shoudong Huang.

**For the original version without ROS, use the [C_plus_plus branch](https://github.com/WANGYINGYU/Occupancy-SLAM/tree/C_plus_plus) and its installation instructions.** The instructions below are for the ROS2 `main` branch.

## Overview

Occupancy-SLAM jointly optimizes:

- robot poses
- occupancy values on map grid vertices

Unlike pipelines that optimize poses first and then build maps, this method solves pose and occupancy together in one optimization framework.

This branch integrates the C++ 2D and 3D backends with ROS2. It accepts original 2D TXT data, ROS1/ROS2 bags containing LaserScan or PointCloud2 messages, and 3D PCD scans with poses. Results include the initial map, the directly optimized occupancy field, and a map rebuilt using the optimized poses, with RViz2 visualization.

[Installation](#installation) · [First example](#first-example) · [Your own data](#your-own-data) · [Configuration](#configuration) · [Outputs and visualization](#outputs-and-visualization) · [Evaluation](#evaluation) · [Citation](#citation)

## Installation

Choose **one** installation method. Docker is the recommended starting point; native installation is useful when developing the C++ or ROS2 code.

| | [Docker](#option-a-docker) | [Native](#option-b-native) |
| --- | --- | --- |
| Host environment | Linux with Git, Python >= 3.10, and Docker Engine; tested on Ubuntu 24.04 x86_64. | Ubuntu 24.04 with Git, ROS2 Jazzy, and system Python 3.12. |
| Dependencies | Installed inside the image by the Dockerfile. | Installed on the host using the commands below. |
| Build command | `./tools/docker.sh build` | `./tools/build.sh` and `bash tools/build_rviz.sh` |
| Runtime entry point | `./tools/docker.sh` | `./tools/occupancy-slam` |

### Get the source

Use a Bash terminal. Clone the repository into any directory you own, then run the remaining commands from its root:

```bash
git clone --branch main https://github.com/WANGYINGYU/Occupancy-SLAM.git
cd Occupancy-SLAM
export OCCUPANCY_WORKSPACE="$HOME/occupancy-slam-workspace"
```

`OCCUPANCY_WORKSPACE` is the output directory, separate from the source checkout. It is created automatically; choose another writable absolute path if needed. It defaults to `~/occupancy-slam-workspace`. Results and logs are stored in `runs/`, and reusable display maps in `cache/`. No machine-specific paths or dedicated disk are required.

### Option A: Docker

Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) and enable access for your current user using Docker's [Linux post-installation instructions](https://docs.docker.com/engine/install/linux-postinstall/). Then verify access and build:

```bash
docker info
./tools/docker.sh build
```

This creates `occupancy-slam:jazzy` with ROS2 Jazzy, `slam_toolbox`, both C++ backends, the RViz2 grayscale Map extension, and the Python runtime/evaluation dependencies. You do not need to install ROS2, PCL, libigl, or Python packages on the host for this route.

Continue to the [Docker example](#docker). Container options and GUI setup are described in the [Docker guide](docs/DOCKER.md).

### Option B: Native

#### 1. Install ROS2 and system dependencies

On **Ubuntu 24.04**, complete the [ROS2 Jazzy deb installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), including the `ros-jazzy-ros-base` or desktop package. This configures the ROS package repository needed by the following commands.

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git libeigen3-dev libpcl-dev \
  python3-venv python3-pip python3-numpy python3-scipy python3-yaml python3-pil \
  ros-jazzy-slam-toolbox ros-jazzy-rviz2 ros-jazzy-rmw-fastrtps-cpp
```

The C++ backends require a C++17 compiler, CMake >= 3.16, Eigen >= 3.4, OpenMP, and libigl headers. PCL supplies 3D point-cloud I/O. ROS2 supplies the bag frontend and visualization interfaces.

#### 2. Install libigl and Python dependencies

From the repository root:

```bash
git clone --depth 1 --branch v2.5.0 https://github.com/libigl/libigl.git third_party/libigl
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

libigl is used as a header dependency; it does not need a separate build. The virtual environment must use Ubuntu's Python 3.12. `--system-site-packages` allows it to use the ROS2 Python modules installed by apt.

| Python requirements | When to install |
| --- | --- |
| [requirements.txt](requirements.txt) | Required for normal TXT, bag, PCD, map export, and visualization workflows. |
| [requirements-evaluation.txt](requirements-evaluation.txt) | Optional MATLAB-data conversion and evaluation; includes the runtime requirements and adds h5py and Matplotlib. |

Only for evaluation, also run `python3 -m pip install -r requirements-evaluation.txt`. ROS2 Python modules are supplied by apt, not pip. No MATLAB installation is required for the conversion tools.

#### 3. Compile the backends and RViz2 extension

Keep the virtual environment active:

```bash
source /opt/ros/jazzy/setup.bash
./tools/build.sh -DBUILD_3D=ON
bash tools/build_rviz.sh
```

The executables are `build/occupancy_slam_2d` and `build/occupancy_slam_3d`. The grayscale Map extension is installed under `build/rviz_install/` and loaded automatically by the viewer. These scripts use CMake directly; a separate colcon workspace is not required.

<details>
<summary>Optional build settings</summary>

Pass CMake options to `tools/build.sh`:

| Option | Purpose |
| --- | --- |
| `-DBUILD_3D=OFF` | Build only the 2D backend; `libpcl-dev` can be omitted from the dependency installation. |
| `-DLIBIGL_INCLUDE_DIR=/absolute/libigl/include` | Use an existing libigl checkout instead of cloning into `third_party/`. |
| `-DBUILD_TESTING=ON` | Build optional regression tests; disabled by default. See [validation](docs/VALIDATION.md). |

After C++ edits, rerun the corresponding build script. Parameter-file edits do not require recompilation.

</details>

#### 4. Activate the environment in a new terminal

Return to your clone's root before running:

```bash
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
export OCCUPANCY_WORKSPACE="$HOME/occupancy-slam-workspace"
```

If you chose a custom workspace, use that same path in each terminal. Continue to the [native example](#native).

## First example

The Intel Lab example checks installation using 20 scans and up to five optimization iterations. Run **only the block for your installation method**. The downloader retrieves the original versioned data from the [standalone release](https://github.com/WANGYINGYU/Occupancy-SLAM/tree/C_plus_plus/Data); datasets are not bundled with `main`.

### Docker

```bash
# Download the example data.
./tools/docker.sh --run-name example_intel exec \
  python3 /opt/occupancy-slam/tools/download_example.py --example Intel

# Optimize and export maps.
./tools/docker.sh --run-name intel_demo txt \
  --dataset "$OCCUPANCY_WORKSPACE/runs/example_intel/Intel" \
  --backend-config config/backend/intel.txt \
  --max-scans 20 --iterations 5 --display-resolution 0.05

# View the result on an X11/XWayland desktop.
./tools/docker.sh --gui view 2d \
  --result "$OCCUPANCY_WORKSPACE/runs/intel_demo/pipeline"
```

### Native

```bash
# Download the same example data.
python3 tools/run.py --name example_intel -- \
  python3 "$PWD/tools/download_example.py" --example Intel

# Optimize and export maps.
python3 tools/run.py --name intel_demo -- \
  bash "$PWD/tools/occupancy-slam" txt \
  --dataset "$OCCUPANCY_WORKSPACE/runs/example_intel/Intel" \
  --backend-config "$PWD/config/backend/intel.txt" \
  --max-scans 20 --iterations 5 --display-resolution 0.05

# View the result.
./tools/occupancy-slam view 2d \
  --result "$OCCUPANCY_WORKSPACE/runs/intel_demo/pipeline"
```

Both routes save results to `$OCCUPANCY_WORKSPACE/runs/intel_demo/pipeline/`. In RViz2, switch the Map display's **Topic** between `/occupancy_slam/initial/map_display` and `/occupancy_slam/reintegrated/map_display` to compare the map before and after pose optimization. `/occupancy_slam/optimized/map_display` shows the directly optimized field.

Each named run must be new. To repeat the example, use another run name and update the result path; the downloaded data can be reused. Omit `--max-scans` to process the complete dataset and choose an appropriate iteration budget. This short example verifies execution, not reproduction of the paper benchmarks.

On a machine without a desktop, skip the viewer; optimization and map export still work. Add `--no-rviz` to a view command to publish ROS2 topics without opening a GUI. In Docker, also omit `--gui` for this mode.

## Your own data

The following examples use Docker. For native execution, replace `./tools/docker.sh` with `./tools/occupancy-slam`; all input options below are identical. Replace `/absolute/...` with your actual input path. These commands choose a new run name automatically and print its output location.

### 2D TXT scans and poses

```bash
./tools/docker.sh txt --dataset /absolute/Museum_b0 \
  --backend-config config/backend/museum_b0.txt --display-resolution 0.05
```

Use `config/backend/intel.txt` for Intel data. For another sensor, copy a backend profile and set its beam count, angle limits, range limits, and optimizer parameters. The Museum example can also be downloaded by selecting `--example Museum_b0` in the downloader command above.

| File | Required content |
| --- | --- |
| `Range.txt` | One scan per row; ranges in metres, ordered by increasing beam angle. |
| `Pose.txt` | Absolute sensor poses `x y yaw`, in metres and radians. |
| `Odom.txt` | Optional: an absolute first pose, then SE(2) increments in the preceding sensor frame. |
| `timestamps.txt` | Optional: strictly increasing timestamps in seconds. |

Files must have corresponding rows. The number of range columns must match backend `NumBeam`; set `MinAngle` and `MaxAngle` to the sensor's angular limits. Initialization from odometry instead of `Pose.txt` is also supported; see [TXT configuration](config/README.md#original-2d-text-data).

### ROS1 or ROS2 bags

Inspect the available topics before selecting a profile:

```bash
./tools/docker.sh inspect /absolute/input.bag
./tools/docker.sh bag --config config/inputs/rosbag_laserscan.yaml \
  --bag /absolute/input.bag --scan-topic /robot/front/scan \
  --odom-source odom --odom-topic /robot/wheel/odom
```

For PointCloud2, use `config/inputs/rosbag_pointcloud.yaml`, set `scan_topic` to the cloud topic, and configure the projection frame, height interval, and angular bins. Supply a directory for a ROS2 bag. ROS1 and `ros1_bridge` are not required.

Scan, odometry, and TF topic names are configurable in the input YAML. Use `--odom-source scan-match` when external odometry/TF is unavailable. `slam_toolbox` supplies the trajectory used to initialize the 2D C++ backend; it is collected during processing and is not replaced by a final loop-closed graph trajectory. See [bag processing](docs/PIPELINES.md#bag-adapter-and-frontend).

### 3D PCD scans and poses

For an NC20-style dataset containing `odometry.txt` and `PCD/full0.pcd`, `full1.pcd`, etc.:

```bash
./tools/docker.sh pcd --config config/inputs/pcd.yaml \
  --dataset /absolute/NC20_sequence --start 20 --stride 3 --max-frames 40
```

For a general PCD sequence:

```bash
./tools/docker.sh pcd \
  --poses /absolute/poses.tum --manifest /absolute/scans.txt
```

TUM pose rows are `timestamp x y z qx qy qz qw`; poses transform sensor coordinates into world coordinates. Each manifest row is `timestamp pcd_path`, and each PCD contains one scan in sensor coordinates. The default output is `optimized/` in the printed run directory.

## Configuration

| What to change | File or option |
| --- | --- |
| Input files, bag topics, TF frames, frame selection | [config/inputs/](config/inputs/) YAML profiles. |
| 2D scan geometry and optimizer settings | [config/backend/](config/backend/) text profiles, selected with `--backend-config`. |
| Bag frontend scan matching and loop closure | [config/slam_toolbox.yaml](config/slam_toolbox.yaml). |
| 3D voxel size, filtering, solver limits | [config/inputs/pcd.yaml](config/inputs/pcd.yaml). |
| Initial/reintegrated 2D visualization resolution | `--display-resolution 0.05` or `0.1`, independent of the optimization grid. |

CLI options override YAML values. CLI input paths are relative to the calling directory; paths written inside YAML are relative to that YAML file. When moving a profile, update its relative references. In Docker, input paths supplied on the CLI are mounted automatically; directories referenced only inside custom YAML/manifests may need `--mount-input /absolute/directory` **before** the subcommand.

Configuration edits take effect on the next run and do not require rebuilding. See the [configuration reference](config/README.md) for all conventions, including the frozen-N approximation and single-/multi-resolution settings.

## Outputs and visualization

Each optimization exports three states:

| Directory | Poses | Map |
| --- | --- | --- |
| `initial/` | Input poses | Initial integration. |
| `joint/` | Optimized poses | Directly optimized occupancy variables. |
| `reintegrated/` | Same optimized poses | Original observations integrated again with poses fixed. |

These are under `pipeline/optimized/` for 2D and `optimized/` for 3D. Trajectories, numerical maps, effective parameters, and execution status are saved alongside them. The initial and reintegrated 2D display maps default to 0.05 m; the directly optimized field retains its native grid resolution. See [output definitions](docs/MAP_OUTPUTS.md).

Open an existing result with the appropriate viewer:

```bash
# Docker: supply the 2D pipeline directory or the 3D optimized directory.
./tools/docker.sh --gui view 2d --result /absolute/pipeline
./tools/docker.sh --gui view 3d --result /absolute/optimized
```

For native execution, use `./tools/occupancy-slam view` with the same arguments and omit `--gui`. Docker GUI forwarding requires a Linux X11/XWayland desktop; details are in the [Docker guide](docs/DOCKER.md#rviz2).

RViz2 uses standard ROS2 topics in a common coordinate frame. Select `initial`, `optimized` (the `joint/` state), or `reintegrated` in the display's **Topic** field:

| Topic under `/occupancy_slam/{method}/` | Message type |
| --- | --- |
| `map_display` | 2D `nav_msgs/msg/OccupancyGrid`, with black/gray/white rendering. |
| `points` | 3D `sensor_msgs/msg/PointCloud2`, colored by height. |
| `trajectory` | `nav_msgs/msg/Path`. |

The 3D point clouds for `optimized` and `reintegrated` use the same final poses and coincide; their occupancy fields can be inspected separately with `--show-occupancy`. See the [RViz guide](docs/RVIZ.md).

## Evaluation

Evaluation and regression tests are optional and are separate from installation and normal execution. Default builds and the default Docker image omit test binaries; the default image also omits `tests/`.

The [validation guide](docs/VALIDATION.md) describes how to enable tests and how to convert MATLAB-format simulations, run optimization, and evaluate trajectories/maps against independent ground truth. The simulation workflow preserves the stored observations and initializations, uses one shared initial-pose SE(2) alignment for the primary trajectory comparison, excludes the fixed first pose, and reports the final accepted state. Raw simulation data and external MATLAB functions must be supplied separately. Passing functional tests does not establish reproduction of all paper benchmarks.

## Further documentation

| Guide | Contents |
| --- | --- |
| [Docker](docs/DOCKER.md) | Image options, input mounts, GUI forwarding, and container execution. |
| [Pipelines](docs/PIPELINES.md) | TXT/bag/PCD processing, runtime environments, and the C++ API. |
| [Configuration](config/README.md) | Input conventions, frontend settings, and backend parameters. |
| [Map outputs](docs/MAP_OUTPUTS.md) | Initial, joint, and reintegrated results. |
| [RViz2](docs/RVIZ.md) | Topic selection, display resolution, and visualization settings. |
| [Validation](docs/VALIDATION.md) | Optional tests and quantitative simulation evaluation. |

The core source is in `src/`, public interfaces in `include/occupancy/`, batch entry points in `cpp/`, runtime tools in `tools/`, and the RViz2 extension in `ros2/`. The original interactive C++ application and [preliminary MATLAB 3D optimizer](https://github.com/WANGYINGYU/Occupancy-SLAM/tree/C_plus_plus/3D_Preliminary) remain on `C_plus_plus`; their supported options differ from this branch.

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
