# RViz2 原生 Topic 切换

`tools/view_rviz.sh` 读取 C++ 后端已保存的结果，生成独立分辨率的二维重建显示图并发布 ROS2 消息，打开标准 RViz2 单场景。二维默认显示固定优化 pose 重建的 0.05 m 占据图，三维默认显示联合优化 pose 下的扫描点云，在左侧 **Map → Topic**（二维）或 **PointCloud2 → Topic**（三维）下拉框切换结果（双击 Topic 的值，再展开下拉列表）。所有消息都在同一 `map` 坐标系；切换时相机不变。

## 本机 demo

```bash
cd /home/yingyuwang/Documents/Occupancy-SLAM
./tools/view_rviz_demo.sh 2d
# 另一终端：
./tools/view_rviz_demo.sh 3d
```

二维为 Cartographer Revo LDS 的 297 帧结果，三维为 NC20 Short_Exp 的 40 帧结果。分别使用 ROS domain 88、89。入口自动使用本机 `run_on_data.py`，新配置、截图和日志写入数据盘。demo 直接读取已有结果；新结果用通用入口：

```bash
./tools/view_rviz.sh 2d --result /absolute/path/to/pipeline
./tools/view_rviz.sh 3d --result /absolute/path/to/optimized
```

用 `--method initial`、`--method optimized` 或 `--method reintegrated` 指定打开时选中的结果。二维默认 reintegrated，三维默认 optimized。三维默认仅显示按高度着色的扫描点云，`--show-points` 为兼容选项；占据顶点仅在显式使用 `--show-occupancy` 时显示。旧的 `--layout compare` 已移除。

## 独立指定二维显示分辨率

`--display-resolution` 只控制初始地图和优化 pose 重建图的显示栅格，单位 m，默认 **0.05**，也可设为 **0.1**：

```bash
./tools/view_rviz_demo.sh 2d --display-resolution 0.05
./tools/view_rviz_demo.sh 2d --display-resolution 0.1
# 通用入口，直接优化变量仍按原网格显示：
./tools/view_rviz.sh 2d --result /absolute/pipeline --display-resolution 0.05 --method optimized
```

`initial/map_display` 和 `reintegrated/map_display` 分别用固定初始 pose、固定最终 pose，从原始 `Range.txt` 重新累积命中和自由空间。采样规则与 C++ `FuncConvertObs` / `FuncInitialiseGridMap` 一致：按显示分辨率确定射线采样密度，再做 floor 分箱及 log-odds 累积。两张图共用分辨率、原点、覆盖范围、阈值与配色；不再依赖优化网格的采样密度。

`optimized/map_display` 是直接优化变量，保持求解器的原始分辨率；所有 `/map` 连续概率 topics 和后端导出也保持原网格。因此在本机 demo 中，初始／重建显示为 0.05 m，直接优化变量为 0.15 m。切换时 RViz 的 **Resolution** 属性会反映真实尺度，相机不移动。新的显示栅格来自原始扫描重新积分，没有把粗网格图像插值放大，也没有重新优化 pose。

显示图导出目录包含 `{initial,optimized,reintegrated}/`：

- `map.yaml` / `map.pgm`：该显示图的标准地图元数据与黑灰白图像。
- `grid.npz`：与 ROS 消息逐格一致的 `values`，初始／重建还含原始累计 `accumulated_log_odds` 和 `integration_counts`。
- 根目录 `manifest.json` 的 `display_maps` 记录实际分辨率、尺寸、原点与文件位置；`maps` 记录原后端地图，`display_integration` 记录重建模型和共享网格。

显示分辨率需为有限正数，单张显示网格限 1000 万单元以避免意外内存耗尽。更细的显示网格不代表传感器或位姿精度提高。原输入文件与后端结果不会被覆盖。

## Topic 选择

| 结果 | 二维 Map (`nav_msgs/msg/OccupancyGrid`) | 三维 PointCloud2 (`sensor_msgs/msg/PointCloud2`) |
|---|---|---|
| 优化前 | `/occupancy_slam/initial/map_display` | `/occupancy_slam/initial/points` |
| 联合优化变量 | `/occupancy_slam/optimized/map_display` | `/occupancy_slam/optimized/points` |
| 固定最终 pose 重建 | `/occupancy_slam/reintegrated/map_display` | `/occupancy_slam/reintegrated/points` |

二维表中是默认的黑白分类显示。原始连续概率仍在 `/occupancy_slam/{initial,optimized,reintegrated}/map`，可直接切换，也可用 `--map-view probability` 打开。两个系列都使用标准 `nav_msgs/msg/OccupancyGrid`。

左侧 `Path` 初始不勾选；二维还保留可选的 `Scan points`。需要时勾选并选择对应 topic：

- 轨迹：`/occupancy_slam/{initial,optimized,reintegrated}/trajectory`，类型 `nav_msgs/msg/Path`。
- 扫描点云：`/occupancy_slam/{initial,optimized,reintegrated}/points`，类型 `sensor_msgs/msg/PointCloud2`。

每个显示项的 Topic 独立选择。比较一组地图和轨迹时，两项选择相同的方法。三维的联合优化与重建使用完全相同的最终 pose，所以两个 `points` 点云相同；两种最终地图的差异应看 `occupancy`。这些是占据网格顶点，intensity 为概率，不是曲面 mesh。

三维使用原生 `PointCloud2`，可通过 `Add → By topic` 添加。二维默认配置加载 `occupancy_slam_rviz/GrayscaleMap`；它继承原生 Map，仅把未知区域改为中性灰。手动添加时可在 `By display type` 中选这个显示项，再选择标准 OccupancyGrid topic。QoS 保持 **Reliable / Transient Local / Keep Last / Depth 1**，切换后可获取已经发布的结果，无需重跑优化。发布器需持续运行。滚轮缩放；三维左键旋转，Shift + 左键平移。`Fixed Frame` 默认为 `map`。

## 输入与数据语义

二维 `--result` 包含 `Pose.txt`、`Range.txt`、`timestamps.txt`、`backend_config.txt` 和 `optimized/`。查看器自动加载 `optimized/initial/`、`optimized/reintegrated/`。三维 `--result` 包含 `trajectory.tum`、`optimized_points.pcd`、`occupancy_vertices.pcd`，自动加载 `initial/`、`reintegrated/`。旧输出可用 `--initial /absolute/path/to/initial` 加载相同选帧的零迭代结果；缺失的结果不会发布。查看器核对初始位姿／选帧，并要求重建轨迹等于最终轨迹。

`initial` 是输入 pose 建图，`optimized` 对应 `joint/` 的直接优化变量，`reintegrated` 是固定最终 pose、清空地图后使用同一批命中／自由空间观测重建。算法和导出见 [MAP_OUTPUTS.md](MAP_OUTPUTS.md)。

二维 `/map` 值仍为 `sigmoid(M/N)` 连续概率量化到 0–100；默认 `/map_display` 的规则见下文。矩阵按低 y 到高 y 发布，不沿用 PGM 翻转。三维数据已在世界坐标系，不再乘一次位姿。消息使用当前发布时刻，`use_sim_time=false`，原始时间戳仍保存在输入文件中。静态 TF 只注册同坐标显示原点，不表示机器人 odometry。兼容保留 `landmarks` 标准 `visualization_msgs/msg/MarkerArray` topic，默认不显示。

`--max-points` 默认每张点云最多 200000 点，0 表示全部；超出时确定性抽样。`--occupancy-threshold` 默认 0.6，只过滤三维显示。各方法共用色标和阈值。输入目录、点数、frame、完整 ROS2 消息类型和当前默认方法记录在 `manifest.json`。

## 三维高度配色与二维清晰显示

三维参考 [hdl_graph_slam 的地图按 z 高度着色方式](https://github.com/koide3/hdl_graph_slam/blob/master/rviz/hdl_graph_slam.rviz#L145-L174)。当前使用较柔和的**蓝 → 青 → 浅黄 → 橙红**色带、深灰背景和 2 像素点。颜色写入标准 `PointCloud2.rgb`（UINT32），由 RViz 原生 `RGB8` 渲染，无自定义插件。x/y/z、占据概率 intensity 和点的选择规则保持原样。

所有方法使用同一高度范围：初始扫描点云 z 的 1%～99% 分位数（无初始结果时使用优化后的扫描点云），在抽样前计算。范围外只截断颜色，不删点；切换 Topic 不重新归一化。可通过 `--height-range MIN_Z MAX_Z` 指定世界坐标高度范围。占据概率可在 RViz 改为 `Color Transformer: Intensity`、`Channel Name: intensity` 查看；扫描点云本身没有占据概率字段。

二维之前显示不清晰的原因是 `sigmoid(M/N)` 中自由区域大多约为 0.4，初始图大量未观测位置为 0.5。默认 `/map_display` 采用原仓库 `SubFuncs.cpp::FuncShowMap` 的 `sigmoid(M)` 累积 log-odds 显示方式，再做统一分级：

- 有观测且概率 ≤ 0.35：0，白色自由空间。
- 有观测且概率 ≥ 0.65：100，黑色障碍。
- 其余：−1，纯灰色（RGB 128,128,128）未知／未决区域。

参照 [Cartographer 的单通道灰度地图输出](https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/cartographer_ros/ros_map.cc)，当前窗口背景和未知区域都使用 RGB(128,128,128)，障碍黑色、自由空间白色。[ROS2 Jazzy 原生 Map 的 −1 色板](https://github.com/ros2/rviz/blob/jazzy/rviz_default_plugins/src/rviz_default_plugins/displays/map/palette_builder.cpp)是带绿色的 `(112,137,134)`，所以工程增加了仅替换此色板项的 C++ 显示扩展。消息仍是 `nav_msgs/msg/OccupancyGrid`，−1 的未知语义、0–100 数值、位姿、分辨率、TF 和 Topic 订阅均交给原 Map 实现。没有把未知单元改成 50 来伪造灰色。普通地图及 binary 地图均保持中性灰未知区。

观测范围来自实际命中与自由空间样本。初始／重建显示图使用新分辨率下重新积分的未平滑计数，联合优化变量使用原网格中实际样本的非零双线性支撑顶点。平滑后的 N 在整张图上为正，不能用它判断是否被观测。所有方法共用分级阈值；两张重积分显示图共用显示网格，直接优化变量保留原生网格。没有图像滤波、补墙、膨胀或修改优化变量。

`manifest.json` 保存高度色带和范围、二维显示规则、各方法观测范围来源及黑／白／灰单元数。原 `occupancy.txt`、`normalization.txt`、PCD 和 pose 文件均不重写。优化场在未观测区的外推和棋盘纹仍可通过原 `/map` topic 检查；分类显示本身不是地图精度改善。

## 运行环境与验证

关闭窗口或 Ctrl+C 会停止本次发布器及其 RViz。`--duration 20` 可限时运行；`--no-rviz` 只发布消息，可在同一 ROS domain 打开其他 RViz 并加载生成的 `.rviz`。`--frame` 和 `--namespace` 可调整公共坐标系与 topic 前缀。

二维灰度扩展在本机已编译。新机器先构建一次（需要 rviz_default_plugins、pluginlib 的开发文件及 C++ 编译器）：

```bash
# 本机遵守数据盘实验存储规则：
cd /home/yingyuwang/Documents/ws_SDF_BA/src
python3 sdf_ba/tools/run_on_data.py --name occupancy_gray_build_new -- \
  bash /home/yingyuwang/Documents/Occupancy-SLAM/tools/build_rviz.sh
```

编译／安装产品在工程 `build/rviz_plugin` 和 `build/rviz_install`，`tools/env.sh` 自动加载该包。无需修改系统 RViz；源码在 `ros2/occupancy_slam_rviz/`。

环境需 ROS2（本机 Jazzy）、rviz2、rclpy、tf2_ros、标准消息包、numpy 和 PyYAML。`tools/env.sh` 加载本机 runtime 或 `/opt/ros/jazzy/setup.bash`。其他机器无本机数据盘启动器时，应配置日志／临时文件位置，并用 `--output /absolute/path/to/new/viewer-directory` 指定新目录。

`tests/test_rviz.py` 用真实保存结果检查 DDS 晚订阅、消息数值与文件一致性、RViz 实际订阅和窗口截图。Topic 切换验证及其运行记录见 [VALIDATION.md](VALIDATION.md)。

二维原始场仍有棋盘纹和边界晕影。MapDisplay 首次加载可能记录 `active samplers with a different type refer to the same texture image unit`；本机实际显示与原始日志均保留。查看器验证不代表地图精度改善。

## 独立地图导出

新 pipeline 已包含默认 0.05 m 的 `display_maps/`。RViz 发布器从该导出读取网格；切换显示分辨率或打开历史结果时，命令入口先调用不依赖 ROS 的 `export_maps.py`，再启动发布器。`--display-resolution 0.1` 仍可直接使用，直接优化变量保留自身分辨率。所有结果仍通过原生 Map/PointCloud2 的 Topic 下拉框切换。

显示图现在可跨查看器启动复用。入口先检查结果目录已有的 `display_maps/`；不匹配时，在数据盘 `$SDF_BA_STORAGE_ROOT/occupancy_display_cache/` 下准备持久缓存，默认根目录为 `/media/yingyuwang/Data/experiments/sdf_ba`。缓存按结果绝对路径和显示分辨率区分，不再在每个新查看器目录中重复积分。查看器 `manifest.json` 记录实际使用的显示图路径。

输入扫描、位姿、原生地图、参数和绘图代码的路径、大小、纳秒修改时间，以及缓存格式版本共同决定是否复用；缺少输出文件或文件大小变化也会重建。它是普通的文件元数据缓存，不检查保留大小和修改时间的原地内容篡改。手动改变这类文件时应正常更新修改时间。并发写入使用文件锁；完整生成后才原子切换入口，已有查看器仍可读取旧代文件。旧代文件不会自动删除。

历史导出缺少缓存元数据时会首次重建；缓存位置须处于已挂载、可写的数据盘实验目录，否则明确报错。缓存不改变消息、地图分辨率、颜色或地图数值。2026-10-20 的 Simu2680 实测首次地图准备约 5.08 秒、命中缓存约 0.00066 秒；这只计算地图准备，不包括 ROS 消息构造和 RViz 启动。完整回归见 [VALIDATION.md](VALIDATION.md)。
