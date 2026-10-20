# 输入与运行参数

统一入口为 `tools/run_2d.sh txt` 或 `tools/run_2d.sh bag`。本机入口自动调用数据盘启动器，打印本次输出目录。YAML 中的参数名使用下划线，与命令行的连字符参数对应；命令行覆盖 YAML。未知键、非法类型或不存在的明确指定话题会报错。

| 配置文件 | 用途 |
|---|---|
| [inputs/txt.yaml](inputs/txt.yaml) | 原始 Range/Pose/Odom TXT 路径、初始化方式、抽帧、时间间隔 |
| [inputs/rosbag_laserscan.yaml](inputs/rosbag_laserscan.yaml) | LaserScan bag 的输入 topic、TF、里程计、采样、优化和运行参数 |
| [inputs/rosbag_pointcloud.yaml](inputs/rosbag_pointcloud.yaml) | PointCloud2 bag 的输入 topic、投影坐标系、高度切片和角度分箱 |
| [backend/museum_b0.txt](backend/museum_b0.txt) | 原版键值格式；Museum_b0 的 1079 束扫描参数 |
| [backend/intel.txt](backend/intel.txt) | 原版键值格式；Intel 的 180 束扫描参数及 odometry 权重 |
| [slam_toolbox.yaml](slam_toolbox.yaml) | 本地可编辑的 scan matching、回环、求解器参数 |

## 更换 rosbag 话题

修改输入 YAML 中的实际 bag 话题，例如：

```yaml
scan_topic: /robot/front/scan       # LaserScan；PointCloud2 时填写点云 topic
odom_source: odom                  # 明确使用 odom；也可 tf、scan-match、auto
odom_topic: /robot/wheel/odometry
tf_topic: /robot/tf
tf_static_topic: /robot/tf_static
odom_frame: robot_odom             # frame_id，不是 topic
projection_frame: null             # 默认传感器坐标系；需要投影时填写有 TF 的目标 frame
```

```bash
cd ~/Documents/Occupancy-SLAM
./tools/run_2d.sh bag --config config/inputs/rosbag_laserscan.yaml --bag /absolute/input.bag
./tools/run_2d.sh bag --config config/inputs/rosbag_pointcloud.yaml --bag /absolute/cloud.bag
```

`scan_topic: auto` 仅在 bag 中只有一个 LaserScan/PointCloud2 topic 时自动选择；有多个时必须明确指定。`odom_source: auto` 按已配置 Odometry → 可用 TF → 扫描匹配的顺序选择。要确保使用某个里程计，设为 `odom`；指定话题没有 Odometry 消息时不会静默替换。输入话题不要求带固定命名空间。TF 的 frame 名和 topic 名独立设置。

bag 话题在 `inputs/*.yaml` 中修改。驱动会将输入转换到内部虚拟扫描 topic/frame，再交给 slam_toolbox；不要通过修改 `slam_toolbox.yaml` 中的 scan_topic 来选 bag 话题。驱动固定 sim time、内部 frame/topic、逐帧处理、TF 发布和范围等设置；运行目录里的 `slam_toolbox.yaml` 是最终生效参数。其余扫描匹配、回环和 Ceres 参数来自这里的可编辑配置。

## 原始 TXT

```bash
./tools/run_2d.sh txt --config config/inputs/txt.yaml --max-scans 20 --resolution 0.3 --iterations 3
./tools/run_2d.sh txt --dataset Data/Intel --backend-config config/backend/intel.txt --max-scans 20
# 仅有 Range.txt 和 Odom.txt 时，auto 自动由里程计生成初值：
./tools/run_2d.sh txt --dataset /absolute/txt_directory --backend-config /absolute/config.txt --pose-source odom
```

- `Range.txt`：每行一帧，每列一束距离，单位 m；列数必须等于后端 `NumBeam`。
- `Pose.txt`：每行 `x y yaw` 绝对传感器位姿，角度为 rad。
- `Odom.txt`：可选；第 0 行是绝对初始位姿，之后每行是前一帧坐标系下的 SE(2) 增量。与原 `FuncPosefromOdom` 约定相同。
- `timestamps.txt`：可选，每行一个递增秒时间戳。缺失时按原始帧号 × `scan_period` 生成仅用于显示／文件索引的时间，记录为 synthetic。
- 可用 `--ranges`、`--poses`、`--odom`、`--timestamps` 覆盖目录内同名文件；`pose_source: auto` 优先遵从原配置 `PosefromOdom`，没有 Pose 而有 Odom 时也会选择里程计初始化。
- `WeightO > 0` 要求提供 Odom。仅有 Pose 时使用 `WeightO 0`，不会擅自将 Pose 差分当成独立里程计。

根据数据集修改原版键值参数中的 `NumBeam/MinAngle/MaxAngle/MinRange/MaxRange`；输入格式相同不表示不同雷达共用同一几何参数。新 TXT 入口按 `ModeMulti` 选择单分辨率或粗到细 C++ 后端，抽帧由 `start/scan_stride/max_scans` 控制；抽帧后的 odometry 会按 SE(2) 组合跨越的增量。`ModeMulti` 原样传递；`ModeKeyFrame/EvaluateGT/ModeShowMap` 在该入口关闭；原交互式程序仍保留。

## 路径、输出和显示

YAML 中的输入路径相对 YAML 文件目录，命令行输入路径相对调用命令的目录。`output` 的相对路径落在新的数据盘实验目录。输出目录必须为空；原数据不会覆盖。

两种输入均产出相同的 `pipeline/Range.txt`、`Pose.txt`、可选 `Odom.txt`、`timestamps.txt`、`backend_config.txt` 和 `optimized/{initial,joint,reintegrated}/`。保留输入配置副本、生效参数、实际后端命令和日志，TXT 另保存原始帧索引。TXT 直接调用 C++，bag 先调用 ROS2 slam_toolbox。

计算完成后：

```bash
./tools/view_rviz.sh 2d --result /absolute/run/pipeline --display-resolution 0.05
```

初始／重建显示分辨率仍独立于优化网格，可改为 0.1。原始优化变量保持自身网格。详细操作见 [RViz 指南](../docs/RVIZ.md)。

## 重构后的参数边界（2026-10-20）

- `inputs/pcd.yaml`：三维 PCD/TUM 或 manifest，使用 `tools/run_3d.sh --config ...`。
- `backend/bag.txt`：bag 的后端设置；TXT 与 bag 都经 `backend_config.py` 校验，再由 `backend_runner.py` 执行。`--backend-config` 可切换此文件。
- `frontend_resolution`：可选，只覆盖 slam_toolbox 地图分辨率；null 保留 `slam_toolbox.yaml` 原值。
- `resolution` / 后端 `Scale`：优化变量分辨率；`display_resolution`：初始和固定最终 pose 重建图的分辨率，默认 0.05 m。
- `step_policy: legacy`：默认，保留论文近似更新；`decrease` 为可选的固定当前 N 的代价下降检查。两者均不加入 N 的 pose 导数，并在接受更新后重算 N；重算后的总代价不保证单调。
- `max_observations`：二维在展开射线前检查，三维在生成该帧观测前检查；与地图格点限制分别作用。默认 20000000。它不是总进程内存上限。

`ModeSparseSolver`、`SolverFirstMaxIter`、`SolverFirstTolerance` 现在对二维含/不含 odometry 的完整网格路径都生效；不再有隐藏的 sqrt(系统大小) 迭代上限。批量入口的精细阶段使用 `SolverSecondMaxIter/SolverSecondTolerance`；`ModeSparseSolver=false` 在两阶段都使用直接求解。原交互式程序仍是独立 legacy 实现。使用以前的参数文件可能因此改变结果，这是参数开始生效带来的变化。有效设置保存在输出目录。

`input_manifest.json` 区分外部 odometry、slam_toolbox 因果轨迹和从该轨迹差分得到的相关先验；本次没有把因果轨迹替换成最终回环图节点。`run_status.json` 和 `summary.json` 分别描述执行状态和求解停止原因。

## MATLAB simulation 精度对齐

`backend/simulation_matlab.txt` 对应 Downloads 中 `2D_Occupancy_SLAM_MATLAB/loadParam.m` 当前实际活动的单层路径：0.1 m、500×500、原点 (-25,-25)、20 步上限、直接求解、N 平滑权重 10。`DOWN_TIME=0` 使活动地图正则使用 `SecondSmoothWeigh=1e-7`，停止阈值使用 `MinMeanDeltaSecond=300`，因此映射到 C++ 的 `MapSmoothingWeightFirst/MinDelta` 是这两个实际值。数据使用无损 `--samples`，不按网格重采样；原 Main 中的 `Pose=Pose_GT` 不进入噪声初值测试。

- `FixedGrid=true`：使用显式 `Sizei/Sizej/OriginX/OriginY`；false 根据输入自动计算范围。
- `InitialMapRound=true`：初始及最终 pose 重建使用 round；false 使用 floor。
- `NormalizationRound=false`：每次更新后从零按 floor 重算 N；true 使用 round。两者都不对 N 求 pose 导数。
- `WeightSmoothN`：完整网格 N 的平滑权重；`WeightSmoothNSecond`：选区 N 的平滑权重（默认 1），与地图 M 的正则权重分开。
- `ModeMulti=true`：`Scale` 为精细网格，粗网格为 `Scale*DownRate`；`DownTime` 是粗阶段预算，`MaxIter` 是总上限。较小总预算会保留至少一次精细更新，粗阶段也可因配置阈值提前结束。
- `--save-iterations poses` 保存每次接受更新的位姿；`true` 同时保存 M/N。三类最终地图始终输出。

默认 `config.txt` 的两阶段 CG 设置改为 2000 / 1e-8；已有显式数据集配置不会被覆盖。运行状态区分迭代达到上限与收敛。

`backend/simulation_multires.txt` 保留 8 次粗阶段 + 3 次细阶段的短预算诊断；`backend/simulation_multires_extended.txt` 将上限统一扩大为 40 + 10，其他数值设置相同，仍按原有步长阈值提前停止。simulation 完整流程使用后一份；`simulation_matlab.txt` 只用于当前 MATLAB 保存状态的单层一致性参考。较充足预算不是强制执行 50 次，也不依据 GT 决定停止。

Simulation 转换默认生成 `samples.bin`；批量验证优先读取它，旧转换目录则根据 `conversion.json` 自动无损导出到本次数据盘运行目录。`--samples` 可复用已有导出；`--resample-endpoints` 才会显式按网格重新生成自由空间样本。批量验证默认使用 `simulation_multires_extended.txt` 和 100000000 样本上限；其他二维入口的默认上限保持 20000000。
