# Occupancy-SLAM

C++ implementation of joint robot-pose and occupancy-map optimization:

- [Occupancy-SLAM: An Efficient and Robust Algorithm for Simultaneously Optimizing Robot Poses and Occupancy Map (IEEE T-RO)](https://arxiv.org/pdf/2502.06292)
- [Occupancy-SLAM: Simultaneously Optimizing Robot Poses and Continuous Occupancy Map (RSS 2022)](https://www.roboticsproceedings.org/rss18/p003.pdf)

Authors: Yingyu Wang, Liang Zhao, Shoudong Huang.

## Choose a branch

| Branch | Intended use |
| --- | --- |
| **[main](https://github.com/WANGYINGYU/Occupancy-SLAM/tree/main)** | ROS2 edition: ROS1/ROS2 bag adapters, 2D TXT and 3D PCD inputs, C++ optimization, RViz2 and Docker. |
| **[C_plus_plus](https://github.com/WANGYINGYU/Occupancy-SLAM/tree/C_plus_plus)** | Original standalone C++ implementation, preserved from the previous `master` branch. |

**If you do not use ROS2, use the `C_plus_plus` branch and follow its README.**
**不使用 ROS2 时，请切换到 `C_plus_plus` 分支，按照该分支的 README 使用原版独立 C++ 工程。**

```bash
# Original standalone C++ version, without ROS2
git clone --branch C_plus_plus https://github.com/WANGYINGYU/Occupancy-SLAM.git
```

The instructions below are for **main**, tested on **Ubuntu 24.04 / ROS2 Jazzy**. Its TXT/PCD numerical backends remain independent of ROS; ROS2 supplies the bag frontend and visualization. Docker includes ROS2, so a host ROS2 installation is not required for Docker use.

## Supported workflows

- **2D TXT:** original `Range.txt`, `Pose.txt` and optional `Odom.txt` directly enter the C++ optimizer.
- **2D bag:** read ROS1 or ROS2 bags with `rosbags`, process LaserScan or projected PointCloud2 through ROS2 `slam_toolbox`, then run C++ joint pose/map optimization. No ROS1 installation or `ros1_bridge` is required.
- **3D:** sensor-frame PCD scans plus TUM poses, including the NC20 directory layout and timestamp manifests, enter a sparse C++ joint pose/occupancy optimizer.
- **Results:** save the initial map, directly optimized map variables and a new map integrated using the final poses; compare them by changing standard topics in RViz2.

Bag scan, odometry and TF topics are configurable in [config/inputs/](config/inputs/). The frontend uses external odometry/TF when provided, or scan matching to obtain its motion prior. Exported `slam_toolbox` poses are associated by scan timestamp; they are the trajectory observed during processing, not a re-export of all final loop-closed graph nodes. See [pipeline details](docs/PIPELINES.md).

The paper's approximation for **N** is retained: pose Jacobians omit derivatives of N, and N is recomputed after every accepted update. The default `legacy` update policy is preserved. The new C++ 3D backend does not implement every feature of the independent MATLAB code in `3D_Preliminary/`.

## Docker quick start

Requirements: Linux, Python 3 on the host and working Docker Engine access. RViz needs an X11/XWayland desktop. The image includes ROS2 Jazzy, `slam_toolbox`, the C++ backends, RViz2 and Python dependencies.

```bash
git clone --branch main https://github.com/WANGYINGYU/Occupancy-SLAM.git
cd Occupancy-SLAM

# Set this to an existing writable data-disk mount point on your machine.
export OCCUPANCY_DATA_ROOT=/mnt/data
./tools/docker.sh build

# Run the bundled Intel TXT example.
./tools/docker.sh --run-name intel_demo txt \
   --dataset Data/Intel --backend-config config/backend/intel.txt \
   --max-scans 20 --iterations 5 --display-resolution 0.05

# Compare results in RViz2.
./tools/docker.sh --gui view 2d \
   --result "$OCCUPANCY_DATA_ROOT/experiments/sdf_ba/codex_runs/intel_demo/pipeline"
```

`OCCUPANCY_DATA_ROOT` must be a real writable disk mount, not just a newly created directory. The default is `/media/yingyuwang/Data`; set it for your machine. Outputs, caches and temporary files go under `$OCCUPANCY_DATA_ROOT/experiments/sdf_ba/codex_runs/`. Each run name must be new; omit `--run-name` to generate one automatically.

The Docker wrapper also requires Docker image/build storage under that disk. Configure Docker's `data-root` there; when using the containerd image store, configure containerd's storage separately, as described in the [Docker daemon documentation](https://docs.docker.com/engine/daemon/#daemon-data-directory). The wrapper checks the Docker root and does not change host services. See the [Docker guide](docs/DOCKER.md) for mounts, GUI, custom image names and MATLAB conversion.

### ROS1 / ROS2 bags

```bash
./tools/docker.sh inspect /absolute/input.bag
./tools/docker.sh bag \
   --config config/inputs/rosbag_laserscan.yaml \
   --bag /absolute/input.bag \
   --scan-topic /robot/front/scan \
   --odom-source odom --odom-topic /robot/wheel/odom
```

For PointCloud2, use [rosbag_pointcloud.yaml](config/inputs/rosbag_pointcloud.yaml) and set the point-cloud topic, projection frame and height range. With no odometry, use `--odom-source scan-match`. ROS2 bag input is the bag directory. CLI arguments override YAML; input paths in YAML are relative to that YAML file. For files referenced inside YAML/manifests outside the mounted input directories, add `--mount-input /absolute/directory` before the subcommand.

### 3D PCD + poses

```bash
./tools/docker.sh --run-name pcd_demo pcd \
   --config config/inputs/pcd.yaml --dataset /absolute/NC20_sequence \
   --start 20 --stride 3 --max-frames 40
./tools/docker.sh --gui view 3d \
   --result "$OCCUPANCY_DATA_ROOT/experiments/sdf_ba/codex_runs/pcd_demo/optimized"
```

An NC20 sequence contains `odometry.txt` and `PCD/full0.pcd`, `full1.pcd`, etc. Alternatively supply `--poses /absolute/poses.tum --manifest /absolute/scans.txt`; each manifest row is `timestamp pcd_path`. Pose rows are `timestamp x y z qx qy qz qw` and transform sensor coordinates into world coordinates. See [PCD configuration](config/inputs/pcd.yaml).

## Native ROS2 installation

Install [ROS2 Jazzy for Ubuntu 24.04](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), then, from the repository root:

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
./tools/occupancy-slam txt --dataset Data/Intel \
   --backend-config config/backend/intel.txt --max-scans 20
```

CMake >= 3.16, C++17, Eigen >= 3.4, OpenMP and libigl headers are required. `-DLIBIGL_INCLUDE_DIR=/absolute/include` selects existing libigl headers. `BUILD_3D=OFF` omits PCL. `BUILD_LEGACY=ON` additionally requires OpenCV and builds the original interactive application. The Python requirements above include conversion and tests; `requirements-bag.txt` is sufficient for normal bag/TXT/PCD/RViz operation.

Native commands use `./tools/occupancy-slam {txt|bag|pcd|view|export|inspect}` with the same pipeline arguments as Docker. For example, `./tools/occupancy-slam view 2d --result /absolute/pipeline`. The wrappers create data-disk run directories automatically. CMake also installs executables, libraries, public headers, configurations and the `occupancy-slam` entry point.

## Inputs and configuration

| File / directory | Purpose |
| --- | --- |
| [config/inputs/txt.yaml](config/inputs/txt.yaml) | TXT files, pose source, scan selection and output settings. |
| [config/inputs/rosbag_laserscan.yaml](config/inputs/rosbag_laserscan.yaml) | Bag topics, TF frames, odometry source and frontend settings. |
| [config/inputs/rosbag_pointcloud.yaml](config/inputs/rosbag_pointcloud.yaml) | PointCloud2 topic and conversion to horizontal scans. |
| [config/inputs/pcd.yaml](config/inputs/pcd.yaml) | PCD/TUM inputs, point sampling, grid and solver limits. |
| [config/backend/](config/backend/) | 2D optimizer profiles, including Intel, Museum and simulation. |
| [config/slam_toolbox.yaml](config/slam_toolbox.yaml) | Scan matching, loop closure and frontend solver settings. |

For original TXT data, `Range.txt` has one scan per row and one beam per column; `Pose.txt` has absolute sensor poses `x y yaw` in metres/radians. Optional `Odom.txt` starts with an absolute pose, followed by SE(2) increments expressed in the preceding frame. `timestamps.txt` is optional. Match the backend's beam count, angles and range limits to your sensor. With only poses, use `WeightO 0`; pose differences are not silently substituted for independent odometry. Full conventions are in the [configuration guide](config/README.md).

Frontend map resolution, optimizer `Scale` / `--resolution`, and `--display-resolution` are independent. Initial and final-pose display maps default to **0.05 m**, optionally **0.1 m**. The directly optimized map stays at the optimizer's resolution.

## Outputs and RViz comparison

2D results are in `pipeline/optimized/`; 3D results are in `optimized/`:

| Subdirectory | Pose | Map |
| --- | --- | --- |
| `initial/` | Input poses | Initial occupancy field. |
| `joint/` | Optimized poses | Directly optimized occupancy variables. |
| `reintegrated/` | The same optimized poses | New integration of the original observations with poses fixed. |

Each result includes trajectories, numerical map values and `map_outputs.json`. `summary.json` records the solver stop reason; `run_status.json` records execution status. High-resolution 2D display maps are separate exports in `display_maps/`. See [map outputs](docs/MAP_OUTPUTS.md).

RViz uses standard ROS2 topics in the same `map` frame. Change the display's **Topic** to compare `initial`, `optimized` (the `joint/` result) and `reintegrated`:

- 2D: `/occupancy_slam/{method}/map_display`, `nav_msgs/msg/OccupancyGrid`, with black obstacles, white free space and gray unknown space. The bundled display extends RViz Map only to set a neutral gray palette.
- 3D: `/occupancy_slam/{method}/points`, `sensor_msgs/msg/PointCloud2`, colored by height.
- Trajectories: `/occupancy_slam/{method}/trajectory`, `nav_msgs/msg/Path`.

The two final 3D point clouds are identical because their poses are identical; direct-variable versus reintegrated **occupancy-field** comparison is available with `--show-occupancy`. Raw 2D probability topics remain at `/occupancy_slam/{method}/map`. More controls and QoS settings: [RViz guide](docs/RVIZ.md).

## Tests and simulation evaluation

```bash
./tools/docker.sh test unit
./tools/docker.sh test inputs
./tools/docker.sh test architecture
# Native equivalents: ./tools/test.sh unit|inputs|architecture
```

Tests cover the N approximation and refresh, linear solvers, input geometry, quantitative evaluation, TXT/PCD adapters and display-grid resolution. Input tests exercise actual `slam_toolbox` with renamed LaserScan/PointCloud2/odometry/TF topics and reject invalid topics.

For MATLAB simulation data, `tools/convert_matlab_simulations.py` preserves Cartesian endpoints, exact stored free-space samples and each supplied initialization. `tools/benchmark_matlab_simulations.py` runs the converted cases using [simulation_multires_extended.txt](config/backend/simulation_multires_extended.txt); `tools/evaluate_simulation_maps.py` evaluates maps against independent simulator geometry. Install `requirements-matlab.txt` for native use. Conversion uses Python and does not require MATLAB; running the separate MATLAB reference does.

Validation records and reproducible commands are in [docs/VALIDATION.md](docs/VALIDATION.md). These include 41 native simulation cases, Docker/native numerical comparisons, real Revo LDS and NC20 segments, and actual RViz topic/window checks. Real-data segment checks are not full-sequence accuracy benchmarks. Local artifact paths in that document refer to the development machine and are not required to build or run this repository.

## Code layout

`include/occupancy/` provides the C++ APIs. `src/optimization/`, `src/io/`, `src/mapping/` and `src/apps/` contain solvers, adapters, exports and applications. `SubFuncs.cpp` / `FuncConvertObs.cpp` retain the original 2D numerical functions. Python orchestration lives in `tools/`; the RViz display package is in `ros2/occupancy_slam_rviz/`. The preliminary MATLAB 3D implementation remains in `3D_Preliminary/`.

## Showcase Video

[![Occupancy-SLAM Video](https://img.youtube.com/vi/-_-39SrPxKk/maxresdefault.jpg)](https://www.youtube.com/watch?v=-_-39SrPxKk&t=4s)

## More Results

### More datasets

| Dataset | Initialization | Ours |
|---|---|---|
| ACES | ![](images/ACES_Ini.jpg) | ![](images/ACES_Our.jpg) |
| Intel Lab | ![](images/Intel_Ini.jpg) | ![](images/Intel_Our.jpg) |
| C5 | ![](images/C5_Ini.jpg) | ![](images/C5_Our.jpg) |
| Museum b0 | ![](images/b0_Ini.jpg) | ![](images/b0_Our.jpg) |
| Freiburg Building 079 | ![](images/fr079_Ini.jpg) | ![](images/fr079_Our.jpg) |
| Garage | ![](images/garage_Ini.jpg) | ![](images/garage_Our.jpg) |
| MIT | ![](images/MIT_Ini.jpg) | ![](images/MIT_Our.jpg) |
| Simu 1 | ![](images/Simu1_Ini.jpg) | ![](images/Simu1_Our.jpg) |
| Simu 2 | ![](images/Simu2_Ini.jpg) | ![](images/Simu2_Our.jpg) |
| Simu 3 | ![](images/Simu3_Ini.jpg) | ![](images/Simu3_Our.jpg) |

### High-frequency scan comparison

We compare different methods using high-frequency scans on one simulation dataset.

![High-Frequency Scan Results](images/High_Frequency.png)

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

This project is released under the [MIT License](./LICENSE.txt).
