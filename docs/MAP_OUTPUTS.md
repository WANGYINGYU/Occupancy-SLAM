# 优化变量地图与固定 pose 重建地图

二维、三维 C++ 后端每次运行都会导出三组结果，完成普通运行即可，不需要增加参数或另跑一次零迭代：

| 目录 | 位姿 | 占据场来源 |
|---|---|---|
| `initial/` | 输入位姿 | 求解开始前的初始占据场 |
| `joint/` | 联合优化后的位姿 | 求解器直接优化的地图变量 |
| `reintegrated/` | 与 `joint/` 完全相同 | 从零重新累积命中和自由空间观测 |

这些目录位于二维 pipeline 的 `optimized/` 下，或三维 `--output` 目录下。旧根目录文件仍保留，内容对应 `joint/`，已有消费者继续可用。`map_outputs.json` 记录目录、方法和参数。

重积分保持位姿固定，不再次进行 pose/map 联合求解，不读取联合优化后的地图值；沿用相同的传感器观测、过滤／采样、分辨率、网格坐标及支撑域。三维使用求解器已经保留的原始传感器坐标采样点生成的命中／自由空间观测，不把 `optimized_points.pcd` 重新当作传感器观测，也不重复套用位姿。

## 二维

每组保存 `poses.txt`、可用时的 `trajectory.tum`、`occupancy.txt`、`normalization.txt`、`map.yaml`、`map.pgm`。

- `joint/occupancy.txt` 是求解器中的 M，保留双精度原值；原始连续场 topic `/map` 的概率为 `sigmoid(M / max(N, 1e-9))`。另有 `/map_display` 使用原仓库的 `sigmoid(M)` 显示规则及观测范围做黑／白／灰分级，见 [RVIZ.md](RVIZ.md#三维高度配色与二维清晰显示)。
- `initial/` 和 `reintegrated/` 按显式 `InitialMapRound` 取整规则累积原始命中及自由观测，再通过原有 `FuncSmoothN2` 平滑 N。额外的 `integration_counts.txt` 保存平滑前计数，方便独立核对。
- 初始/重建由 `InitialMapRound` 控制，迭代 N 由 `NormalizationRound` 控制；legacy 默认 floor/round，MATLAB 参考配置为 round/floor。N 每次从零重算，不继承上一步计数，再以独立 N 权重平滑。
- 后端导出的三组地图保持相同范围和原点。原始导出文件与 `/map` 保留全部网格变量，已有棋盘纹、晕影等问题保留；新增 `/map_display` 根据实际射线样本支撑标记未知区域；查看器中初始／重建显示图可使用独立分辨率，见下文。PGM 是显示图像，精确数值以矩阵文件为准。

查看器支持 `--display-resolution 0.05`（默认）或 `0.1` 等正数，用原始扫描和固定初始／最终 pose 从空场重新生成对应的 `initial/map_display`、`reintegrated/map_display`。这两张图共用显示网格，结果保存在本次查看器目录的 `display_maps/` 中（PGM、YAML、精确栅格 NPZ）。`optimized/map_display` 仍显示原始优化变量，不改变其分辨率。后端 `initial/joint/reintegrated` 三组矩阵及所有原始 `/map` topics 继续保留优化网格，不被显示重建覆盖。

## 三维

每组保存：

- `trajectory.tum`、`selected_scans.txt`：世界坐标位姿及选定传感器扫描。
- `map_variables.txt`：`key_x key_y key_z log_odds`，双精度；`joint/` 中为直接优化的变量值。
- `grid.json`、`world_from_grid.txt`：体素尺度 h 和网格到世界的变换，`p_world = T_world_grid * (h * key)`。
- `occupancy_vertices.pcd`：世界坐标网格顶点，intensity = sigmoid(log_odds)，用于 RViz；这是顶点集合，不是曲面 mesh。
- `optimized_points.pcd`：该组 pose 拼接的采样点云；`joint/` 与 `reintegrated/` 的点云相同是预期行为。
- `initial/`、`reintegrated/` 额外保存 `integration_weights.txt`。

重积分对每个原始命中／自由空间观测重新计算三线性权重，累积 `sum(weight × target)` 和 `sum(weight)`；非零权重顶点取加权平均 log-odds，其余为 0。权重包含原有每条射线的自由空间归一化。没有从联合优化地图取值，也没有把点云简单拼接冒充占据重建。三组共享支撑域，既可逐顶点比较，也避免显示范围不同造成误判。

## 查看对比

```bash
cd /home/yingyuwang/Documents/Occupancy-SLAM
./tools/view_rviz_demo.sh 2d
# 在另一终端
./tools/view_rviz_demo.sh 3d
```

新结果可使用 `view_rviz.sh 2d --result /absolute/pipeline` 或 `view_rviz.sh 3d --result /absolute/output`。在原生 `Map`／`PointCloud2` 显示项的 `Topic` 下拉框中切换 `initial`、`optimized`（joint）和 `reintegrated`，保持同一坐标系和相机；三维默认显示扫描点云；两个最终 pose 相同，因此最终两组拼接点云也相同。占据场差异可显式用 `--show-occupancy` 查看。详见 [RVIZ.md](RVIZ.md)。

`tools/compare_maps.py` 输出三列静态 PNG/PDF、相同局部范围放大图和概率差值图，接受 `--result2d`（C++ 输出目录）、`--result3d` 和 `--output`。所有方法共用坐标范围、色标及阈值；局部窗口按输入轨迹中点选取，不按某个方法的效果挑选。

## 验证边界

真实 Cartographer 297 帧、NC20 Short_Exp 40 帧已重新运行。最终 pose 与之前结果逐项相同，联合优化地图也与之前相同。二维 671016 个命中／自由空间观测由独立 Python 实现重新累积，计数和数值匹配；三维用非单位世界变换的合成数据独立核对三线性公式，0 次和 3 次优化均通过。零迭代时三维三类场完全一致；真实优化后两种最终场确实不同。

这些检查证明导出和重建符合定义，不证明某一种地图精度更高。三维 p≥0.6 的顶点数分别为 35578 / 73114 / 33301；顶点更多不等于表面更准确。详细证据与运行路径见 [VALIDATION.md](VALIDATION.md)。

## 重构后的元数据与导出

新结果 `map_outputs.json` 使用 schema 2，保留 schema 1 的兼容读取。初始、联合变量和重新积分的目录由该文件定位。二维元数据明确记录 N 的 pose 导数被有意忽略、每次接受更新后按生效配置重算并平滑；初始/重建使用同一初始化取整规则。

`pipeline/display_maps/{initial,optimized,reintegrated}/` 保存 `map.pgm`、`map.yaml` 和 `grid.npz`；`display_manifest.json` 记录分辨率、来源与分类规则。这些是独立导出产品。原始 `/map` 的 sigmoid(M/N) 与 `/map_display` 的 sigmoid(M) 分类保持区分，灰度更清晰不代表模型概率或真实精度更优。

多分辨率运行额外保留 `coarse/`、`coarse_iterations.csv` 和 `fine_iterations.csv`。三类正式地图均在同一精细网格上；joint 是精细选区的联合变量，initial/reintegrated 使用全部原始观测。两个阶段的观测域不同，不能把粗阶段初始代价与精细阶段最终代价作同一目标的下降率；summary 的代价限定为最后优化阶段，完整过程读取分阶段日志。
