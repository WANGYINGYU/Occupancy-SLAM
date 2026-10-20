# Docker

The image contains ROS2 Jazzy, slam_toolbox, the C++ 2D/3D applications, RViz2 with the grayscale Map extension, and Python input/evaluation dependencies. Standalone and MATLAB optimizers are maintained on `C_plus_plus`. Python readers support MATLAB-format simulation data without a MATLAB installation.

## Build and storage

Run from the repository root on Linux with Docker Engine and Python 3 available:

```bash
export OCCUPANCY_DATA_ROOT=/mnt/data
./tools/docker.sh build
```

The data root must be an existing writable disk mount. Each command creates a new directory below `$OCCUPANCY_DATA_ROOT/experiments/sdf_ba/codex_runs/`. Use `--run-name` to name it, or let the wrapper generate a name. Existing run names are rejected. Inputs are mounted read-only; outputs, temporary files, caches, ROS logs, and RViz settings are stored on the data disk.

Docker image/build storage must also be on that disk. The wrapper checks `docker info`'s Docker Root Dir and does not modify host service settings. Configure Docker's `data-root`; installations using the containerd image store also need a separate containerd storage root. Follow the official [Docker daemon storage documentation](https://docs.docker.com/engine/daemon/#daemon-data-directory).

The default image is `occupancy-slam:jazzy`, based on `ros:jazzy-ros-base-noble`, with libigl v2.5.0. `--image` selects another tag. The image compiles the backends and RViz extension from source; host `build/`, `.runtime`, `.venv`, datasets, and third-party directories are excluded. Rebuild after implementation changes. Editing a mounted input configuration does not require a rebuild.

## 2D text data

```bash
./tools/docker.sh --run-name example_intel exec \
  python3 /opt/occupancy-slam/tools/download_example.py --dataset Intel
./tools/docker.sh --run-name intel_demo txt \
  --dataset "$OCCUPANCY_DATA_ROOT/experiments/sdf_ba/codex_runs/example_intel/Intel" \
  --backend-config config/backend/intel.txt \
  --max-scans 20 --iterations 5 --display-resolution 0.05
```

The output is `.../codex_runs/intel_demo/pipeline/`. Native map states are in `optimized/{initial,joint,reintegrated}/`; display exports are in `display_maps/`.

## ROS1/ROS2 bags

```bash
./tools/docker.sh inspect /absolute/input.bag
./tools/docker.sh bag --config config/inputs/rosbag_laserscan.yaml \
  --bag /absolute/input.bag --scan-topic /robot/front/scan \
  --odom-source odom --odom-topic /robot/wheel/odom
```

For PointCloud2, use `config/inputs/rosbag_pointcloud.yaml` and configure the point topic, projection frame, height interval, and angular bins. Use `--odom-source scan-match` when an external prior is unavailable. ROS2 bags are supplied as directories. See the [configuration reference](../config/README.md) for TF topics and precedence rules.

## 3D PCD and poses

```bash
./tools/docker.sh --run-name pcd_demo pcd \
  --config config/inputs/pcd.yaml --dataset /absolute/NC20_sequence \
  --start 20 --stride 3 --max-frames 40
```

Alternatively, specify `--poses /absolute/poses.tum --manifest /absolute/scans.txt`. When a YAML or manifest references another external directory, add `--mount-input /absolute/directory` before the subcommand. The default 3D output is the run's `optimized/` directory.

## RViz2

On a Linux X11 or XWayland desktop:

```bash
./tools/docker.sh --gui view 2d \
  --result "$OCCUPANCY_DATA_ROOT/experiments/sdf_ba/codex_runs/intel_demo/pipeline" \
  --display-resolution 0.05
./tools/docker.sh --gui view 3d \
  --result "$OCCUPANCY_DATA_ROOT/experiments/sdf_ba/codex_runs/pcd_demo/optimized"
```

The wrapper forwards the current DISPLAY, X11 socket, and available Xauthority, and uses Mesa software rendering. It does not require `xhost +`, privileged containers, or NVIDIA Container Toolkit. Pure Wayland without XWayland and Docker Desktop GUI forwarding have not been validated.

To publish for a separate host RViz instance:

```bash
./tools/docker.sh view 2d --result /absolute/pipeline --no-rviz --domain-id 88
```

Containers use host networking. Subscribers must use the same ROS domain and compatible QoS. Map/cloud publishers use Reliable / Transient Local so late subscribers can receive saved results. See [RViz controls](RVIZ.md).

## Conversion, evaluation, and testing

`exec` runs tools inside the image. The image source directory is `/opt/occupancy-slam`:

```bash
./tools/docker.sh --run-name simulation_conversion exec \
  python3 /opt/occupancy-slam/tools/convert_matlab_simulations.py \
  --source /absolute/simulation --output converted
./tools/docker.sh test unit
./tools/docker.sh test inputs
./tools/docker.sh test architecture
```

Simulation conversion uses Python MAT readers; no MATLAB runtime is required. Full benchmark and evaluation commands are in [VALIDATION.md](VALIDATION.md).

## Argument and filesystem conventions

Wrapper options (`--gui`, `--image`, `--run-name`, `--mount-input`) precede the subcommand. Common CLI input paths are resolved on the host and mounted at the same absolute location. Paths inside custom YAML/manifests may require additional explicit mounts. Relative output paths belong to the new run; absolute outputs must remain within the experiment storage root and must be empty.

The container runs with the host UID/GID, a read-only root filesystem, dropped capabilities, and `no-new-privileges`. The data disk is mounted at the same absolute path, with the experiment subtree writable. Container home, `/tmp`, `/var/tmp`, and `/dev/shm` are backed by the run's data-disk scratch directories.

`container_command.json`, `container.log`, `container_status.json`, and `image.json` record runtime execution. Builds record `build_command.json`, `build.log`, and `build_status.json`. See Docker's [bind mount](https://docs.docker.com/engine/storage/bind-mounts/) and [run](https://docs.docker.com/reference/cli/docker/container/run/) references for the underlying options.
