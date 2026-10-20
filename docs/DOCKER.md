# Docker 运行指南

镜像包含 C++ 二维／三维后端、ROS2 Jazzy、slam_toolbox、RViz2 黑灰白地图扩展，以及 TXT、bag、PCD、MATLAB 数据转换与评价所需的 Python 依赖。宿主机只需 Linux Docker Engine；运行时使用镜像中编译的程序。原交互式 OpenCV 应用和 MATLAB 解释器不在镜像内。

## 构建

在工程根目录执行：

```bash
cd /path/to/Occupancy-SLAM
export OCCUPANCY_DATA_ROOT=/mnt/data  # 已存在、可写的真实数据盘挂载点
./tools/docker.sh build
```

默认镜像名 `occupancy-slam:jazzy`，可在命令前用 `--image` 更改。基础镜像为官方 `ros:jazzy-ros-base-noble`，libigl 使用 v2.5.0。构建需要网络，不复制宿主机的 `build/`、`.runtime`、第三方依赖或原始数据。C++ 后端和 RViz 扩展均在镜像中重新编译。

入口自动调用工程 `run_on_data.py`，将命令、日志、容器镜像 ID 和输出保存在数据盘新目录；也可以先用工作区的 `sdf_ba/tools/run_on_data.py` 启动。默认数据盘 `/media/yingyuwang/Data`，实验根目录为其 `experiments/sdf_ba/`。其他 Linux 机器可通过 `OCCUPANCY_DATA_ROOT` 指定真实挂载的数据盘；没有可写数据盘时明确停止。

Docker 自身的镜像／构建存储也应在数据盘；入口构建前检查 Docker Root Dir。Docker 29 使用 containerd image store 时，其独立的 containerd `root` 也须配置到数据盘；不会擅自修改宿主机服务配置。配置方式见 [Docker daemon data directory](https://docs.docker.com/engine/daemon/#daemon-data-directory)。

## 二维 TXT

```bash
./tools/docker.sh --run-name occupancy_docker_intel_new txt \
  --dataset Data/Intel --backend-config config/backend/intel.txt \
  --max-scans 20 --iterations 5 --display-resolution 0.05
```

输出位于 `.../experiments/sdf_ba/codex_runs/occupancy_docker_intel_new/pipeline/`。三种原生地图在 `pipeline/optimized/{initial,joint,reintegrated}/`，独立分辨率的显示图在 `pipeline/display_maps/`。`--run-name` 必须是新名字，省略时自动生成。

## ROS1／ROS2 bag 和可配置话题

先查看 bag 中的话题：

```bash
./tools/docker.sh inspect /absolute/path/input.bag
./tools/docker.sh --run-name occupancy_docker_bag_new bag \
  --config config/inputs/rosbag_laserscan.yaml \
  --bag /absolute/path/input.bag --scan-topic /robot/front/scan \
  --odom-source odom --odom-topic /robot/wheel/odom
```

PointCloud2 投影输入使用 `config/inputs/rosbag_pointcloud.yaml`，可在 YAML 修改高度范围、角度分箱、TF、外参和输入 topic。YAML 路径相对该 YAML 文件解析，命令行覆盖配置；前端分辨率、优化分辨率和显示分辨率分别设置。没有 odometry 时可用 `--odom-source scan-match`。两种扫描类型都实际运行容器内的 slam_toolbox，再调用 C++ 后端。

下载官方 Revo LDS 数据后，例如：

```bash
./tools/docker.sh --run-name occupancy_docker_revo_new bag \
  --bag /absolute/cartographer_paper_revo_lds.bag \
  --start 1 --max-scans 80 --odom-source scan-match --iterations 5
```

## 三维 PCD 与 odometry

```bash
./tools/docker.sh --run-name occupancy_docker_nc20_new pcd \
  --config config/inputs/pcd.yaml \
  --dataset /absolute/NC20_sequence \
  --start 20 --stride 3 --max-frames 40 \
  --voxel 0.5 --point-voxel 0.4 --point-stride 2 --max-range 40
```

也可用 `--poses /absolute/odometry.txt --manifest /absolute/scans.txt`。如果 manifest 内的 PCD 路径或 YAML 内的输入位于另一个目录，在子命令前加 `--mount-input /that/directory`。原始输入只读，输出默认是本次运行目录中的 `optimized/`。

## RViz2 与标准 Topic

Linux X11 或 XWayland 桌面中：

```bash
./tools/docker.sh --gui view 2d \
  --result "$OCCUPANCY_DATA_ROOT/experiments/sdf_ba/codex_runs/occupancy_docker_intel_new/pipeline" \
  --display-resolution 0.05

./tools/docker.sh --gui view 3d \
  --result "$OCCUPANCY_DATA_ROOT/experiments/sdf_ba/codex_runs/occupancy_docker_nc20_new/optimized"
```

GUI 使用当前 `DISPLAY`、只读 X11 socket 和现有 Xauthority 文件，不需要 `xhost +` 或 privileged 容器。默认 Mesa 软件渲染，不要求 NVIDIA Container Toolkit。纯 Wayland 而无 XWayland、macOS／Windows 的 Docker Desktop GUI 转发未覆盖。

二维使用标准 OccupancyGrid，初始和重建地图默认 0.05 m，直接优化变量保持优化分辨率；三维默认显示按高度着色的 PointCloud2。通过 RViz 原生 Topic 下拉框切换 initial／optimized／reintegrated，操作与 [RViz 指南](RVIZ.md) 相同。关闭窗口或 Ctrl+C 会停止发布器及其 RViz 子进程。

仅发布话题供宿主 RViz 订阅：

```bash
./tools/docker.sh view 2d --result /absolute/pipeline --no-rviz --domain-id 88
```

容器使用 host network，宿主订阅者使用相同的 ROS_DOMAIN_ID 和兼容的 ROS2 QoS。地图和点云保持 Reliable / Transient Local，支持迟到订阅。

## 转换、评价和测试

`exec` 可运行镜像自带工具，镜像工程路径固定为 `/opt/occupancy-slam`：

```bash
./tools/docker.sh --mount-input /absolute/downloads exec \
  python3 /opt/occupancy-slam/tools/convert_matlab_simulations.py \
  --source /absolute/downloads --output converted

./tools/docker.sh exec python3 /opt/occupancy-slam/tools/benchmark_matlab_simulations.py \
  --converted /absolute/converted --case Simu2680/odom_1 --jobs 1

./tools/docker.sh test unit
./tools/docker.sh test inputs
./tools/docker.sh test architecture
```

转换执行 Python MAT reader，不执行原 MATLAB 程序。`test inputs` 生成带不同话题名字的 ROS1 bag 并调用实际 slam_toolbox；`test architecture` 覆盖两类后端、PCD manifest 与地图导出。测试使用的仓库 `Data/` 单独只读挂载，不打包进镜像。完整 benchmark 不加 `--case`；地图评价等命令沿用 [VALIDATION.md](VALIDATION.md) 中的参数，将脚本路径替换为镜像内路径即可。

## 路径与输出

- 包装器选项（`--gui`、`--image`、`--run-name`、`--mount-input`）放在子命令之前。
- 常用 CLI 输入路径自动解析为宿主绝对路径并只读挂载到容器相同位置。自定义 YAML／manifest 引用的其他外部目录用 `--mount-input` 补充。
- 数据盘以原绝对路径只读挂载，实验子目录另以可写方式挂载。容器 root filesystem 只读，使用当前宿主 UID/GID；不会生成 root 属主的结果。
- 容器用户目录（含 RViz 持久设置）、临时目录、缓存、ROS 日志、`/tmp`、`/var/tmp` 和 `/dev/shm` 都指向数据盘本次运行目录，不使用系统盘或 RAM disk 保存实验中间数据。
- 相对 `--output` 位于新运行目录；绝对输出必须在实验根目录内。输出目录仍须为空。
- `container_command.json`、`container.log`、`container_status.json`、`image.json` 保存实际调用和退出状态。构建对应 `build_command.json`、`build.log`、`build_status.json`。
- 更新 C++ 或 Python 实现后重新执行 `build`；运行入口不会使用宿主机已有二进制。仅编辑宿主配置不需要重建镜像。

使用的 Docker 挂载与进程行为见官方 [bind mounts](https://docs.docker.com/engine/storage/bind-mounts/) 和 [docker run](https://docs.docker.com/reference/cli/docker/container/run) 文档。
