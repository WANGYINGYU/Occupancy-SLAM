# 验证记录：2026-10-19

工程：`/home/yingyuwang/Documents/Occupancy-SLAM`。
上游提交：`705b2178f2d6f863a85d1705a6631389b9a8de9a`。
以下按时间记录开发阶段的验证；早期“未提交／未改动”的描述仅指当时状态，后续架构、数值兼容和 Docker 验证见追加章节。公开使用入口与分支说明以 [README](../README.md) 为准，数据盘路径为开发机证据位置，不随源码发布。

## 构建

GCC 13.3 / C++17、Eigen 3.4、OpenCV 4.6、PCL 1.14、libigl v2.5.0。两个新增可执行程序和原交互程序均构建成功。运行环境是 ROS2 Jazzy，slam_toolbox 2.8.5、rosbags 0.11.8；slam_toolbox 和缺失的运行库局部解包到数据盘，没有系统 apt 安装。

## 实际运行结果

| 验证 | 实际结果 |
|---|---|
| Cartographer 官方 Revo LDS bag → 扫描匹配先验 → ROS2 slam_toolbox → C++ 二维联合优化 | 80 帧选中、77 帧精确时间匹配、完成 5 次 C++ 优化 |
| 同一真实片段最终二进制复核 | 与原验证轨迹逐项相同，绝对容差 `1e-12` |
| NC20 Short_Exp 小样本 | 原始帧 0/5/10/15，4 帧，860 个观测，1932 个网格顶点，5 个接受步 |
| NC20 Short_Exp 移动片段 | 原始帧 20:5:75，12 帧，43885 个观测，37177 个顶点，10 个接受步 |
| 5 项 Python 单元测试 | TF 插值和非零外参、缺失 TF 拒绝、组织化大端 PointCloud2 行填充、投影高度及最近返回、ICP 变换方向全部通过 |
| 4 条实际 ROS2 集成测试 | LaserScan+TF、LaserScan+Odometry、PointCloud2+TF、PointCloud2+扫描匹配全部通过；每例 24 帧中 21 帧接收，均调用 C++ 后端 |
| 三维格式和求解测试 | 同一合成场景混合 ASCII / binary_compressed PCD；NC20 索引模式与 timestamp manifest 模式轨迹一致；首帧固定，平移与旋转都发生优化更新 |
| Jacobian 检查 | 合成三维场景及两个 NC20 片段的所选位姿／网格有限差分检查通过 |
| 错误输入 | 非法 voxel、重复时间戳、非空输出目录均以非零状态退出 |
| 静态检查 | Python AST、shell 语法、Git diff whitespace 检查通过 |

目标函数的真实数值：

| 运行 | 初始 | 最终 | 指标 |
|---|---:|---:|---|
| Cartographer Revo LDS 77 个接收帧 | 4863.290127 | 1995.663134 | 二维扫描观测 SSE，未包括 odometry/平滑项 |
| NC20 4 帧小样本 | 40.945094 | 18.369729 | 三维完整联合目标 |
| NC20 12 帧移动片段 | 518.096577 | 180.894097 | 三维完整联合目标 |
| 4 帧合成三维场景 | 15.948513 | 4.338320 | 三维完整联合目标 |

这些是优化目标下降与流程验证，**不是相对于 GT 的轨迹误差／地图质量改善证明**。二维原有更新并非每步单调：Cartographer 片段第 3 到第 4 次记录的观测 SSE 从约 2069.89 上升至 2124.86，最终为 1995.66；保留原始记录，没有改写结果。三维接受步都满足完整目标下降。

## 保留的边界与失败记录

- 第一次 Cartographer 尝试 `occupancy_cartographer_smoke_20261019_01` 因首帧不完整、降采样后有效点少于 20 而退出。最终命令明确 `--start 1`，没有悄悄将该帧当作有效扫描。
- slam_toolbox Jazzy 2.8.5 自身启动逻辑忽略其第 2–4 次扫描。全部运行的 3 个跳过帧均写入 `scan_acceptance.json`，C++ 只读取时间戳匹配的扫描和位姿。
- 无 odometry 情况使用扫描 ICP 先验；slam_toolbox 仍实际执行。未将单位位姿或已有轨迹冒充计算结果。
- `/pose` 提供的是当时的 SLAM 轨迹，不是回环之后重导出的完整最终节点轨迹；最终 posegraph 另行保存。
- 三维是新增稀疏连续占据联合优化器，不包含 MATLAB 的全部 NDT、分层、截断与 joining 功能。
- 尚未完成整个 Revo bag 或全部 NC20 序列的精度、资源占用评测。没有从短片段推断全序列质量。

## 数据盘证据

所有下列目录位于 `/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/`：

- `occupancy_runtime_20261019_01/`：局部依赖、venv、`setup.bash`。
- `occupancy_cartographer_data_20261019_01/`：官方原始 demo bag。
- `occupancy_cartographer_smoke_20261019_02/`：真实 bag 的整个二维链路，输入导出、有效配置、posegraph、日志及优化输出。
- `occupancy_final_check_20261019_01/`：最终二进制结果、`run.log`、`cartographer_preview.png`。
- `occupancy_nc20_smoke_20261019_01/`：4 帧三维结果。
- `occupancy_nc20_segment_20261019_01/`：12 帧三维结果。
- `occupancy_integration_20261019_01/`：合成 ROS1 bag、混合编码 PCD、四路 ROS2 端到端测试和错误输入测试。
- `occupancy_build_20261019_01/`、`occupancy_build_20261019_02/`：构建启动记录与编译临时目录。

每个顶层运行目录均有启动器生成的 `launcher.json`；管线另有 `arguments.json`、实际 slam_toolbox YAML、后端配置和命令。可复现命令见 [PIPELINES.md](PIPELINES.md)。

官方数据来源：[Cartographer Revo LDS demo](https://google-cartographer-ros.readthedocs.io/en/latest/demos.html#revo-lds)。

## 追加：用户要求的可视化测试

已增加真实数据长度，未改动优化算法或用新参数追逐预期结果。

| 测试 | 设置 | 实测 |
|---|---|---|
| Cartographer Revo LDS | 原始扫描从 1 开始，步长 2，选 300 帧；所有 beam；0.15 m；10 次更新 | 297 帧接收，覆盖 119.60 s；观测 SSE 47705.565873 → 23065.947921 |
| NC20 Short_Exp | full20 到 full137，步长 3，共 40 帧；其余参数同先前 12 帧测试 | 覆盖 58.50 s、约 24.70 m 输入路径；414225 观测，150453 顶点；完整目标 3744.755103 → 1401.514464 |

三维输出有 82845 个经输入过滤／抽样后的点，可视化确定性显示其中 70000 个。两个画面使用相同点索引、坐标尺度、视角及高度色标。另用 `--iterations 0` 导出相同配置的初始地图进行对比。

**实际发现：**二维迭代误差明显振荡，最终占据图存在棋盘纹和边界晕影，不能判定地图质量已经达标；三维点云在整体视角下变化较小，最大平移改变量 0.304 m。二维最大平移改变量 0.855 m。这些都是相对初始化的变化，不是 GT 误差。本次没有新增 GT 精度结论。

最终图集目录：
`/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_visual_report_20261019_03/figures/`

- `2d_map_comparison.png` / `.pdf`：二维扫描端点及占据图，左为 slam_toolbox 初始化，右为 C++ 优化。
- `3d_cloud_comparison.png` / `.pdf`：三维点云俯视与透视，左为输入 odometry，右为联合优化。
- `2d_diagnostics.png`、`3d_diagnostics.png`：轨迹、改变量和实际收敛曲线。
- `3d_interactive.html`：包含 Plotly JS，完全离线；支持同步旋转、缩放及点云／占据顶点切换。
- `metrics.json`、`REPORT.md`：输入路径、精确统计、已发现的问题及图集索引。

本轮数据和日志：`occupancy_visual_2d_20261019_01`、`occupancy_visual_2d_initial_20261019_01`、`occupancy_visual_3d_20261019_01`、`occupancy_visual_3d_initial_20261019_01`。全部在既定数据盘运行目录。交互图使用独立本地 Chromium 验证，截图和检查记录在 `occupancy_viewer_ui_20261019_02`。

可视化脚本为 `tools/visualize_results.py`，接受 `--pipeline`、`--initial2d`、`--initial3d`、`--optimized3d` 与 `--output`；应通过同一数据盘启动器运行。精确复现命令保存于 `occupancy_visual_report_20261019_03/launcher.json`。新增脚本依赖 plotly，已安装在项目的数据盘局部 venv 中。

## 追加：原生 RViz2 可视化实现与实测

新增 `tools/rviz_viewer.py`、`tools/view_rviz.sh`、本机 demo 入口 `tools/view_rviz_demo.sh`、二维／三维 `.rviz` 配置；共享 PCD 读取器移到 `tools/result_io.py`，原静态绘图脚本复用它。没有修改 C++ 求解器。用上节已完成的真实结果进行保存结果发布和显示验证，没有重跑优化或更换结果。

最终实际运行：
`/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_rviz_validation_20261019_03/checks/`

| 原生窗口 | 验证内容 | 结果 |
|---|---|---|
| 二维地图 + 前后轨迹 | 每条轨迹 297 帧；214 × 137 栅格，0.15 m；每组扫描端点 53813 点 | DDS 数值与文件一致，RViz Global Status OK |
| 三维点云 + 前后轨迹 | 每条轨迹 40 帧，每组点云 82845 点 | 世界坐标逐值核对通过，Global Status OK |
| 三维占据顶点 | 150453 个源顶点；p≥0.6 后初始 35578、优化后 73114 | 坐标和概率逐值核对通过，Global Status OK |
| 三个窗口的晚加入订阅 | 发布器每 topic 仅发布一次，3 秒后建立订阅 | 每例 8 个 topic（含 TF）均收到 |
| 三个窗口退出 | 关闭发布器时清理其 RViz 子进程 | RViz 返回码均为 0 |

可直接查看 `2d/rviz.png`、`3d/rviz.png`、`3d_occupancy/rviz.png`，均为本机实际 RViz 窗口截图，已人工检查。每例保留 `verification.json`、`publisher.log`、`viewer/manifest.json`、实际 `.rviz`、`viewer/rviz.log` 和 `viewer/exit.json`；总检查记录在 `checks/verification.json`，完整启动命令在父目录 `launcher.json`。

二维 RViz MapDisplay 启动时有一次 shader 诊断；截图证明实际地图正常显示，未出现持续 GL_INVALID 或崩溃。记录中明确标为 `known_map_shader_startup_diagnostic`，没有声称渲染日志完全无错误。来源和影响见 [RVIZ.md](RVIZ.md#验证与渲染诊断)。

初次运行 `occupancy_rviz_validation_20261019_01` 在消息接收完成后过早检查 DDS 节点发现，测试断言失败；改为等待实际 RViz 订阅发现。第二次 `_02` 窗口已绘制，但有“无 TF 数据”的 Fixed Frame 警告；随后增加仅用于显示参考原点的静态 TF，最终 `_03` 三个窗口均为 OK。上述历史日志保留。

补充 CLI 验证在 `occupancy_rviz_cli_20261019_01`：无初始结果、`--no-rviz`、每云 25 点限制和 1 秒限时干净退出均通过。静态语法与 `git diff --check` 通过。入口和操作见 [RViz 指南](RVIZ.md)。

## 追加：联合变量地图、固定最终 pose 重建及三列对比

二维／三维后端已默认保存 `initial/`、`joint/`、`reintegrated/`；格式和重建公式见 [MAP_OUTPUTS.md](MAP_OUTPUTS.md)。编译成功后用相同输入和参数重新运行，未修改求解目标、更新规则或最终位姿。

本轮全部目录位于 `/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/`：

- `occupancy_dual_map_build_20261019_01/`：编译日志。
- `occupancy_dual_map_2d_20261019_01/pipeline/optimized/`：Cartographer 297 帧的三组地图；pipeline 输入链接到原数据盘匹配结果，未重跑或更换 slam_toolbox 轨迹。
- `occupancy_dual_map_3d_20261019_01/optimized/`：NC20 Short_Exp 40 帧的三组地图、原始双精度地图变量和网格变换。
- `occupancy_dual_map_verify_20261019_01/checks/verification.json`：独立建图公式与数值核对。
- `occupancy_dual_map_figures_20261019_02/figures/`：最终三列地图、相同位置局部放大、概率差值 PNG/PDF、统计。
- `occupancy_dual_map_rviz_20261019_03/checks/`：三列 RViz 的实际窗口截图、DDS 检查和日志。

核对结果：

| 检查 | 结果 |
|---|---|
| 新运行最终位姿、原有联合优化占据场 | 与之前真实数据结果逐项相同 |
| `joint/` 与 `reintegrated/` 最终位姿 | 完全相同 |
| 二维重新累积 | 671016 个命中／自由空间观测；独立 Python 分箱、累积、计数与 C++ 匹配 |
| 三维重新累积 | 非单位世界变换的合成数据，独立 Python 三线性权重和 log-odds 累积在 0／3 次优化后均匹配 |
| 三维零迭代 | `initial`、`joint`、`reintegrated` 原始地图变量完全一致 |
| 三维真实结果支撑域 | 三组 150453 个顶点，键坐标及网格变换一致 |
| 三维 p≥0.6 顶点数 | 初始 35578，联合变量 73114，重新积分 33301 |
| RViz 并排 | 二维／三维各 21 个 topic 晚订阅接收；9 个对比消息仅 frame 不同，全部数值不变 |
| RViz 固定坐标系／退出 | 实际窗口 Global Status OK；两个子进程均返回 0 |

三维两个最终拼接点云完全一致，这是同一 pose 作用于同一点集的预期结果；真正不同的是占据网格的变量和概率。二维联合变量与重积分的平均概率绝对差为 0.100046，三维为 0.077821；这些是两种地图的差异，不是 GT 精度。

并排 RViz 的额外 TF 只用于移开三列；原始消息和导出文件都保留真实坐标。三维最终并排布局使用正交俯视，保证每列投影一致。旋转的三维图可用 overlay 模式切换方法，静态图也提供三张同相机透视图。

首次并排 DDS 检查 `_rviz_01` 比较序列化缓冲区，遇到未定义 CDR 填充字节差异而失败；改为对所有消息字段做完整相等检查，`_02`／`_03` 通过。`_02` 的三维透视并排投影已替换为 `_03` 的正交布局，避免两侧不同透视视线影响比较。二维仍记录一次已知 MapDisplay 启动 shader 诊断；三张地图均已实际显示。未删除历史失败记录。

仍保留二维原始地图棋盘纹／晕影。三维顶点变多不能证明地图变准；本轮未新增 GT 精度结论。

最终留给用户的窗口截图和总索引在 `occupancy_dual_map_final_capture_20261019_03/`（`REPORT.md`、`rviz_2d.png`、`rviz_3d.png`）。截图通过窗口管理器激活目标窗口后捕获，已逐张核对；此前 `_final_capture_01`／`_02` 的三维截图被另一窗口遮挡，目录报告已标为无效，不作为证据。


## 追加：原生 Topic 切换（取代默认三列展示）

按用户要求，查看器改为同一 `map` 坐标系的标准 RViz2 单场景。二维只默认启用 `Map`（`nav_msgs/msg/OccupancyGrid`），三维只默认启用 `PointCloud2`（`sensor_msgs/msg/PointCloud2` 占据场）。`Path` 和扫描点云按需启用；不再发布平移后的 comparison topics。优化数据、两种最终地图输出均保持原有语义。

以下路径均位于 `/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/`：

| 实测项目 | 证据路径 | 结果 |
|---|---|---|
| 默认二维、三维点云、三维占据场 | `occupancy_topic_dds_20261019_01/checks/` | 三例各 11 个 topic 晚订阅成功；消息与源文件一致；实际 RViz 订阅正确，退出码 0 |
| 二维原生 Topic 下拉切换 | `occupancy_topic_switch_20261019_04/checks/2d/` | initial → reintegrated → optimized；RViz 每次只订阅选中的地图；同一相机、固定坐标系、Transient Local；退出码 0 |
| 三维原生 Topic 下拉切换 | `occupancy_topic_switch_20261019_05/checks/3d/` | 三种 occupancy 实际切换成功；相机不变、坐标系不变；退出码 0 |
| CLI 方法选择 | `occupancy_topic_cli_20261019_01/` | initial 二维、reintegrated 三维；仅发布模式、点数上限 25；正常限时退出 |

切换测试 `tests/test_rviz_topics.py` 通过 X11 操作真实 RViz 原生下拉列表，随后从 DDS 检查 RViz 订阅，并核对 RViz 自己保存的配置中的 Topic、QoS 和相机。每种方法保留 `initial.png` / `reintegrated.png` / `optimized.png`、对应 `.rviz` 与 `verification.json`。已查看实际地图截图；验证使用 Cartographer 297 帧与 NC20 Short_Exp 40 帧的已有计算结果，没有为展示重跑优化。

复现入口为 `tests/test_rviz.py` 与 `tests/test_rviz_topics.py`，具体绝对输入参数、环境、输出位置记录在各运行根目录的 `launcher.json`。GUI 切换测试需要 X11、标准桌面缩放和测试期间无并行鼠标键盘操作；可用 `--mode 2d` 或 `--mode 3d` 单独运行。

历史失败保留：`occupancy_topic_switch_20261019_01`～`_03` 的自动文字编辑未让 Qt Topic 属性成功切换；期间调整了测试进程输入法环境，最终改为直接选择原生下拉列表。`_04` 的二维已完整通过，三维首次选择初始场成功，但测试把六个 PointCloud2 topics 当成三个方法项，选中了错误项目；改为按真实消息类型排序后 `_05` 三维完整通过。以上修正均在测试脚本中，未增加 RViz 自定义插件。

二维仍有一次已知 MapDisplay 启动 shader 诊断，实际三种地图均已绘制，未出现持续 GL_INVALID。三维无 RViz ERROR。它们证明消息和显示切换可用，不代表地图精度改善。此前三列布局的记录是历史验证，不再是当前使用方式；当前操作见 [RVIZ.md](RVIZ.md)。


## 追加：三维高度色带与二维高对比显示

用户反馈原三维占据概率彩虹配色刺眼、二维灰度不清晰。此次仅修改保存结果的显示层。三维新增标准 PointCloud2 `rgb` 字段并使用 RViz 原生 RGB8；二维新增标准 OccupancyGrid `map_display`。原始连续概率 `/map` 逐单元保持不变，三维 x/y/z 与 intensity 逐值保持不变，C++ 优化与输出文件未改动。

三维统一色带为蓝、青、浅黄、橙红。NC20 本次显示高度范围为 **−1.1546990681 ～ 11.6941833496 m**，来自未抽样初始扫描点云的 1%／99% 分位数，供三种结果及扫描点云／占据场共同使用。范围外仅截断颜色。两侧高度颜色分别为 RGB(36,93,170) 和 RGB(205,62,62)；原始占据概率仍保留在 intensity。采用深灰背景和 2 像素点。

二维沿用原工程 `FuncShowMap` 的 `sigmoid(M)` 显示规则，在实际观测范围内按 0.35／0.65 做白／黑分级，其余为未知／未决的 −1。原始 `/map` 仍为 `sigmoid(M/N)` 的 0–100 连续量化。统计如下（同一个 214 × 137、0.15 m 栅格）：

| 方法 | 有观测支撑的格点 | 白色自由空间 | 黑色占据 | 灰色未知／未决 |
|---|---:|---:|---:|---:|
| initial | 8606 | 6891 | 1213 | 21214 |
| optimized | 10052 | 8024 | 1831 | 19463 |
| reintegrated | 8928 | 7177 | 1228 | 20913 |

支撑范围来自未平滑计数或联合优化中实际样本的非零双线性支撑。灰色不是对不理想数值的人工擦除；完整原场可切回 `/map` 检查。分级图提高阅读对比度，不证明优化精度提高。原有零散噪点和 0.15 m 网格锯齿仍存在。

实测路径均在 `/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/`：

- `occupancy_style_inspect_20261019_01/statistics.json`：原始概率、归一化和三维高度分布。
- `occupancy_style_validation_20261019_03/checks/`：最终 DDS 数值核对及原生窗口截图；二维接收 14 个 topics，三维两种视图各 11 个；三个 RViz 返回码均为 0。截图为 `2d/rviz.png`、`3d/rviz.png`（扫描点云）、`3d_occupancy/rviz.png`（占据场），已实际查看。
- `occupancy_style_switch_20261019_01/checks/`：二维和三维各 initial → reintegrated → optimized 三次真实下拉切换；实际订阅正确，相机不变；三维 RGB8 和共享高度范围保持一致。每种方法有单独截图和 `.rviz` 配置。
- `occupancy_style_options_20261019_01/`：原概率查看模式和显式 `--height-range -2 10` 验证。

`tests/test_display_styles.py` 还通过已知坐标检查 floor／双线性支撑、零权重邻点、旋转栅格、未观测单元保持未知、RGB 端点与截断。DDS 检查逐值验证了原始字段，并核对 rgb 的标准 UINT32 类型和打包位置。所有运行的绝对命令与环境位于对应 `launcher.json`。

历史记录保留：`occupancy_style_validation_20261019_01` 在新测试脚本语法检查时停止，随后修复；`_02` 已通过 DDS，但三维仍用原生全彩虹高度色，最终替换为 `_03` 的柔和 RGB 色带。二维继续记录一次已知 MapDisplay 启动 shader 诊断，实际地图与切换均已渲染；三维无 RViz ERROR。


## 追加：二维中性黑灰白、三维默认扫描点云

按用户要求，三维默认的唯一启用显示项改为标准 PointCloud2 扫描点云，Topic 为 `/occupancy_slam/optimized/points`；继续保留共享高度 RGB 色带。占据顶点需显式指定 `--show-occupancy`。二维仍发布标准 OccupancyGrid，新增继承原生 MapDisplay 的 C++ 显示类，仅将地图色板中 −1 的颜色改为 RGB(128,128,128)，窗口背景采用同一纯灰。障碍为黑，自由空间为白。原始消息、概率、优化结果和重积分结果均未修改。

此扩展是 `ros2/occupancy_slam_rviz`，通过 `tools/build_rviz.sh` 构建；`tools/env.sh` 自动加载包。先前关于“无自定义插件”的记录是历史状态；当前二维使用这个色板扩展，三维继续使用原生 PointCloud2。所有结果仍可在同一个 Topic 属性中切换。

以下实测路径均相对于 `/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/`：

| 验证 | 路径 | 结果 |
|---|---|---|
| C++ 灰度显示扩展 | `occupancy_grayscale_build_20261019_01/build.log` | 编译、链接、安装成功 |
| 真实 DDS 与 RViz | `occupancy_grayscale_validation_20261019_01/checks/` | 二维 14 个 topics、三维点云及显式占据场各 11 个；消息与源文件一致，实际订阅正确，三个窗口退出码均为 0 |
| 原生 Topic 下拉切换 | `occupancy_grayscale_switch_20261019_01/checks/` | 二维 map_display、三维 points 均完成 initial → reintegrated → optimized；相机和 QoS 不变，三维 RGB8 与高度范围不变，退出码均为 0 |
| 二维截图像素 | `occupancy_grayscale_pixels_20261019_01/pixels.json` | 地图视口 ROI 共 510000 像素，仅 RGB(0,0,0)、(128,128,128)、(255,255,255)，无带色像素 |

已实际查看 `checks/2d/rviz.png` 和 `checks/3d/rviz.png`。切换测试目录保留三种方法各自的 PNG、RViz 配置和验证 JSON；完整命令、输入位置与环境见各运行根目录的 `launcher.json`。验证复用 Cartographer Revo LDS 297 帧和 NC20 Short_Exp 40 帧已有结果，没有重新计算优化。

二维仍记录一次已知 MapDisplay 启动 shader 诊断，实际地图与切换均成功显示；三维无 RViz ERROR。本次只改变显示配色与默认显示对象，地图中已有噪点、锯齿和几何误差仍保留。


## 追加：独立二维显示分辨率（2026-10-20）

新增查看器参数 `--display-resolution`，默认 0.05 m。初始／最终 pose 重建显示图从同一份原始 Range.txt 重新生成命中和自由空间样本，在共享显示网格中累积。直接优化变量显示及全部原始 /map topics 保持后端分辨率；不修改 C++ 优化、位姿或既有输出。二维默认选中 reintegrated/map_display。

使用已有 Cartographer Revo LDS 297 帧进行实测：

| 显示分辨率 | 初始／重建栅格尺寸 | 每种 pose 下重放的命中及自由样本 | 原始优化栅格 |
|---|---|---:|---|
| 0.05 m | 645 × 414 | 2068747 | 214 × 137，0.15000000596046448 m |
| 0.10 m | 324 × 208 | 1020502 | 214 × 137，0.15000000596046448 m |

两张显示重建图在各自分辨率下共用原点和范围；直接优化变量保留真实尺度。新的 `display_maps/{initial,optimized,reintegrated}/map.pgm`、`map.yaml`、`grid.npz` 保存实际显示图。manifest 分开记录原后端地图与显示地图的尺寸、分辨率、来源。

以下证据路径相对于 `/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/`：

- `occupancy_display_resolution_numeric_20261020_02/verification.json`：已知旋转／平移坐标、无效量程、非法分辨率及网格内存限检查通过；新积分器在原生网格上与原 C++ 的初始／重建计数逐格相等，log-odds 在 1e-10 绝对误差内一致；0.05/0.1 m 均保持全部原始概率网格和联合变量显示不变。
- `occupancy_display_resolution_rviz_20261020_01/resolution_005/2d/` 与 `resolution_010/2d/`：两档均实际收到 14 个 topics，原始概率消息与文件一致，显示消息与新导出逐格一致，实际 RViz 订阅默认重建 topic，退出码均为 0。两张 `rviz.png` 已实际查看，Resolution 属性分别为 0.05 与 0.1。
- 同一 RViz 测试目录 `resolution_005/3d/` 和 `3d_occupancy/`：三维默认点云／显式占据场两例兼容性检查通过，原始 XYZ/intensity 与颜色字段正常，退出码均为 0。
- `occupancy_display_resolution_switch_20261020_01/checks/2d/`：真实下拉切换 initial → reintegrated → optimized，实际 DDS 订阅、相机、坐标系与 QoS 验证通过；前两者 0.05 m，后一者原始 0.15 m。各方法保留 PNG、RViz 配置与 verification.json。
- `occupancy_display_resolution_exports_20261020_01/verification.json`：两档共六张 PGM/YAML 重读均还原 NPZ 中的原始显示栅格，上下方向、黑白灰和未知值正确；两张 RViz 截图视口均为中性灰度。

第一次数值测试 `occupancy_display_resolution_numeric_20261020_01` 的合成例把旋转后的采样点精确放在浮点分箱边界，导致两格的预期不稳定；改用远离边界的已知坐标后 `_02` 通过，未为迎合测试改变 floor 算法。原有精确格点／零权重支撑测试仍通过。完整命令和输入位置见各运行的 launcher.json。

二维保留一次已知 MapDisplay 启动 shader 诊断，真实渲染与切换正常；三维无 RViz ERROR。这是显示分辨率与重积分行为验证，不新增地图或轨迹精度结论。


## 追加：原始 TXT 入口与 rosbag 话题配置（2026-10-20）

新增统一入口 `tools/run_2d.sh txt|bag`。TXT 直接调用现有 C++ 后端，支持 Range+Pose、Range+Pose+Odom、Range+Odom；bag 保留真正的 ROS2 slam_toolbox→C++ 流程。可编辑参数集中在 `config/inputs/`、`config/backend/` 和 `config/slam_toolbox.yaml`。YAML 输入路径相对于配置文件，命令行路径相对于调用目录，命令行覆盖配置。输出与中间文件都在本机数据盘。

实际测试目录：`/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_input_modes_20261020_02/`。完整结果见 `verification.json`、`tests.log` 及各 pipeline 的配置副本、实际后端命令、阶段日志。

| 测试 | 输入与执行 | 结果 |
|---|---|---|
| 原始 Museum TXT | 原仓库 1036 帧中选择 8 帧，Range+Pose，C++ 2 次迭代 | 输入逐行匹配、三类地图完整、0.05 m 查看器加载通过 |
| 原始 Intel TXT | 原仓库 910 帧中选择 8 帧，Range+Pose+Odom，C++ 2 次迭代 | 输入位姿保持原值，跨帧 odometry 的独立 SE(2) 组合检查通过 |
| 无 Pose.txt 的 Intel 输入 | 仅指定 Range/Odom，auto 初始化，C++ 2 次迭代 | 由增量生成初值并与独立标量实现一致；时间戳明确标记 synthetic |
| 改名 LaserScan + TF | 24 帧 ROS1 合成 bag，真实 slam_toolbox 与 C++ | 接收 21 帧并完成优化、图序列化 |
| 改名 LaserScan + Odometry | 同一 bag，真实 slam_toolbox 与 C++ | 接收 21 帧，先验与 TF 路径一致 |
| 改名 PointCloud2 + TF | 带额外高处点的 24 帧 ROS1 合成 bag | 接收 21 帧，投影距离与 LaserScan 在 2e-6 内一致 |

bag 的扫描话题为 `/robot/front/ranges` 和 `/lidar/filtered_cloud`，里程计为 `/robot/wheel/odometry`，动态与静态 TF 为 `/robot/transforms`、`/robot/static_transforms`。每个 bag 还包含另一个 `/unused/sensor`，验证配置选择而非依赖唯一话题自动识别。非单位激光外参 (.4, -.1, .3 rad) 的先验检查通过。明确选择 odom 时，不存在的 odom topic 会报错；错误扫描 topic 会列出可用候选。YAML 未知键、错误类型／枚举和 CLI 优先级均已检查。

TXT 抽帧为原始索引 2,5,...,23，优化分辨率 0.5 m，仅作为格式和执行测试。Intel Range+Pose+Odom 的原始 scan SSE 从 1105.07 增至 4843.02，保留原始记录；这里未调整求解器或挑选改善结果，不声称这些测试证明精度收益或全序列性能。

Museum TXT 的真实 DDS/RViz 验证在 `occupancy_txt_rviz_20261020_01/checks/2d/`：14 个 topics 均接收，原始地图与文件一致，0.05 m 初始／重建显示为 793×643，优化变量保留 79×64、0.5 m。RViz 默认订阅重建地图、正常退出，截图 `rviz.png` 已实际查看。保留一次已知 MapDisplay 启动 shader 诊断，实际绘制正常。

历史 `occupancy_input_modes_20261020_01` 已通过六个真实流程，但最后一个错误话题测试同时把 scan/odom 设为同名，先触发了名称冲突诊断，未满足测试预期字符串；修正测试输入后 `_02` 全部通过。原有 5 项 scan_io 测试亦通过。没有重写二维数学核心，也没有修改三维流程。


## 追加：工程模块重构与 N 近似保留（2026-10-20）

本轮按输入适配、前端、C++ 优化、地图输出、ROS 显示拆分工程。二维／三维核心库可独立安装并被外部 CMake 工程调用；TXT、PCD 和显示地图导出不依赖 ROS；二维无界面构建不要求 OpenCV/PCL。输入参数、求解器参数及 frontend/backend/display 三种分辨率分别管理。现有入口兼容，统一入口为 `tools/occupancy-slam txt|bag|pcd|view|export|inspect`。

**N 的位姿导数继续按论文近似省略。** 原来的两处完整网格 `FuncDiffJacobian` 函数与重构前归档逐字相同；每次接受联合更新后，继续按原来的 round 分箱重算 N，再执行原平滑。初始和重新建图的 floor 分箱也保留。测试使用常量 M、非恒定 N，明确检查近似位姿导数为零，而移动位姿后的实际残差仍可能变化，防止把有意近似误改为完整商法则。

默认 `step_policy=legacy` 保留原更新接受方式。可选 `decrease` 仅在当前固定 N 下检查扫描、里程计、平滑三项总目标；接受后再刷新 N，不承诺刷新后总目标单调。CG 的配置迭代上限和容差现在实际生效，删除了隐藏的规模上限及带 odometry 路径的硬编码参数；因此使用原配置文本未必复现此前被硬编码覆盖的求解结果。匹配旧程序的实际求解参数后，二维状态与地图通过一致性检查。

结果继续分为 initial、joint、reintegrated；schema 2 清单标记规范目录。独立显示导出保存 0.05/0.1 m 初始及优化 pose 重建地图，joint 显示保留变量网格。`summary.json` 记录停止原因、各目标项、线性求解状态、N 更新次数；`run_status.json` 区分运行中、完成、失败、中断。完成不等于收敛或精度改善。`input_manifest.json` 明确 bag 当前输出为 slam_toolbox 当时接收的位姿，而不是后续闭环结束后的最终图节点位姿；由这些位姿差分生成的约束也明确标为相关来源。

以下目录都位于 `/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/`，完整命令见各自 `launcher.json`。

| 验证 | 实测结果 | 证据目录 |
|---|---|---|
| 原生核心及现有 Python 单元检查 | CTest 3/3；覆盖 N 近似／刷新、CG 参数和直接求解、扫描转换、显示分类 | `occupancy_refactor_final_20261020_01/` |
| 对照修改前二进制 | Intel 8 帧两步，匹配实际 CG 56 次／0.005；三类轨迹和场在 1e-12 内一致，PGM 像素相同；三维合成 4 帧同样通过 | `occupancy_refactor_final_20261020_01/verification.json` |
| 可独立运行的架构测试 | 自动准备 TXT／PCD；求解参数确实影响输出；观测预算超限有失败状态；固定 N 接受条件通过；禁用 rclpy 时仍能导出／重读 0.05 和 0.1 m 图 | `occupancy_refactor_portable_test_20261020_02/verification.json` |
| 六种输入流程 | Museum TXT、Intel Pose/Odom、Intel Odom-only；改名 LaserScan+TF、LaserScan+Odometry、PointCloud2+TF；后三种各接收 21/24 帧并实际运行 slam_toolbox；错误配置拒绝及前后端分辨率独立检查通过 | `occupancy_refactor_inputs_final_20261020_02/verification.json` |
| 独立安装／依赖隔离 | 安装后的 TXT 入口通过；外部 CMake 工程链接安装的核心库成功；显式禁用 OpenCV/PCL 的二维构建通过 | `occupancy_refactor_install_20261020_01/verification.json`、`occupancy_refactor_delivery_20261020_02/verification.json` |
| 最终安装 API | 再次安装并编译／执行外部 C++ 调用方；只安装公共接口，包含可选 RViz 扩展的源码 | `occupancy_refactor_package_final_20261020_01/verification.json` |
| Cartographer Revo LDS | 选择 80 帧，实际接收 77 帧，启动阶段跳过 3 帧；真实 slam_toolbox→二维 C++ 五步→自动显示图导出 | `occupancy_refactor_cartographer_20261020_01/pipeline/` |
| NC20 Short_Exp | start=20，40 帧，stride=3，十次接受更新；初始／联合／重建三套 trajectory 和 map_variables 与重构前结果逐数值完全相同 | `occupancy_refactor_nc20_20261020_01/optimized/`、`occupancy_refactor_evidence_20261020_01/verification.json` |
| 实际 RViz 与消息 | 二维 14 个 topics，三维点云及显式占据场各 11 个；晚加入订阅与文件数值一致；窗口正常退出；已查看黑灰白 0.05 m 地图与高度着色点云截图 | `occupancy_refactor_rviz_20261020_01/checks/` |
| 实际 Topic 下拉切换 | 二维与三维均完成 initial→reintegrated→optimized，订阅正确，相机保持；各状态保留截图和配置 | `occupancy_refactor_switch_20261020_01/checks/` |

数值结果及限制：

- Cartographer 77 帧的扫描 SSE 为 4863.290127→1954.792855，总目标 4874.289845→1981.325127，五步迭代上限停止。优化网格为 0.2 m、75×70；独立显示地图为 0.05 m、303×283。
- NC20 40 帧共有 414225 个观测、150453 个顶点，总目标 3744.755103→1401.514464，十步迭代上限停止。这里验证重构前后数值一致性，不宣称新的 GT 精度提升。
- Intel Pose/Odom 八帧按现在实际生效的配置 CG 200／0.05，扫描 SSE 1105.066844→6179.333412，总目标 2435.212307→6229.211710，`objective_increased=true`。匹配旧 CG 56／0.005 时总目标也从 2435.212307 增至 4945.501145，与旧程序一致。退化如实保留；没有将完成状态当作优化成功。
- 同一小输入的可选 decrease 策略总目标降至 2050.558295，但扫描 SSE 为 1148.024451，仍略高于初值。它仅验证接受策略，不构成轨迹／地图精度改善证据，也不改变 N 近似。
- 二维保留一次已知 MapDisplay 启动 shader 诊断，实际渲染和 topic 切换正常；三维无 RViz ERROR。RViz 验证主动 SIGINT 结束发布器时，状态记录为 interrupted，属于测试退出方式。
- 未执行所有数据集的全序列精度评估；MATLAB 三维仍是独立参考，未声称全部算法移植；未新增在线子图系统或闭环最终位姿回溯导出。

开发失败记录保留：`occupancy_refactor_build_20261020_01` 首次构建缺失 namespace 结束括号，修复后构建通过；`occupancy_refactor_delivery_20261020_01` 的可移植测试在准备输入时将 MaxIter=0 传入后续验证，导致“两次 N 更新”断言失败，改正测试夹具后 `_delivery_02` 与 `occupancy_refactor_portable_test_20261020_02` 全部通过。没有改变算法来满足这两个失败用例。

修改前源码及二进制保存在 `occupancy_refactor_source_20261020_01/`；源码归档、原 Jacobian 核对与真实 NC20 数值核对见 `occupancy_refactor_evidence_20261020_01/`。源码保持未提交状态，未覆盖其他工作区。


## 追加：二维 simulation 定量评估——现有房间用例（2026-10-20）

**覆盖范围尚不等于全部论文 simulation。** 当前仓库 Data 中没有 Simu 1／2／3 的扫描、初始位姿、真值或配置；README 中仅有展示图片。本轮实际重跑并定量评估的是 `tests/make_fixtures.py` 中已有的一个 9×7 m 矩形房间，分别经过四种输入链路。四条链路共享场景和轨迹，不是四个独立场景；没有注入量程或先验噪声，轨迹短、转角小，不能据此判断所有场景的鲁棒性。此段记录的是提供数据位置之前的房间用例；随后从 Downloads 找到的四组 MATLAB 数据由本文末尾“MATLAB simulation 数据集”单独记录。房间用例不能代替那些数据集。

未修改优化器或 N 近似。采用当前默认 legacy 策略、0.3 m 优化网格、10 次迭代，真实运行 slam_toolbox 后再调用 C++ 后端。四条链路各有 24 帧输入、21 帧前端接收，3 帧在前端启动阶段跳过；后端均完成 10 次更新并以 iteration_limit 停止，不宣称收敛。初始指 slam_toolbox 输出的 C++ 初值；TF／Odometry 输入是其前端先验。

### 轨迹真值和评估方法

真值按原合成器的机器人轨迹和非单位激光外参独立用标量公式计算，保存为 timestamp/x/y/yaw。GT 只进入后处理评估，不传给优化器。每个结果时间戳必须在 1 微秒内唯一匹配真值；不静默丢弃未匹配帧。三种结果共用由 initial 首帧确定的 SE(2) 刚性变换，不拟合尺度，不分别拟合主指标的坐标系。固定首帧不计入绝对误差均值，即每条链路评价其余 20 帧。旋转差使用 atan2(sin Δθ, cos Δθ)，避免跨 ±π 的伪大误差。

平移 RMSE 使用二维欧氏距离；`coordinate_rmse_m` 是汇总 x/y 坐标平方误差后取根的指标，比二维欧氏 RMSE 小 √2。更正：它不是原 MATLAB FuncEval 的 `(RMSE_x+RMSE_y)/2`，后者另存 `matlab_axis_mean_rmse_m`，三者不能混称。另保存相邻帧及间隔十帧的相对位姿误差、MAE、P95、最大误差和每帧 CSV；独立拟合 SE(2) 的结果仅作次要指标。

| 输入链路 | 初始平移 RMSE (mm) | 最终平移 RMSE (mm) | 初始航向 RMSE (°) | 最终航向 RMSE (°) | 初始／最终相邻帧平移 RPE (mm) |
|---|---:|---:|---:|---:|---:|
| LaserScan + TF | 239.308 | 0.618 | 2.22219 | 0.01317 | 20.000 / 0.168 |
| LaserScan + Odometry | 239.308 | 0.618 | 2.22219 | 0.01317 | 20.000 / 0.168 |
| PointCloud2 + TF | 239.308 | 0.618 | 2.22219 | 0.01317 | 20.000 / 0.168 |
| PointCloud2 + scan matching | 221.208 | 1.184 | 2.07590 | 0.01543 | 18.930 / 0.286 |

joint 与 reintegrated 使用相同位姿，故轨迹误差相同；其地图独立评价。优化目标总值也已记录，但上述误差来自真值，不用 cost 下降代替精度证据。

### 直接优化场与重新建图的独立定量评价

从同一组真值传感器位姿发射固定射线，每帧 180 条，共 3780 条。角度相对观测偏移 0.5°，以解析矩形边界的首次相交距离作为真值。查询原生场 `bilinear(M)/bilinear(N)`，不读取显示分级图，不做地图 ICP；三种方法使用同一首帧坐标变换。射线步长 0.025 m、最大距离 15 m；这引入最多一个步长的离散定位误差。分别报告概率阈值 0.5 与 0.65，恰好 0.5 的无证据格不视作命中。

下表给出阈值 0.65（原生 map.yaml 的占据阈值）。为避免漏检降低均值，必须同时看命中覆盖率和缺失惩罚：无命中射线误差记为 15 m，有命中误差截断到 15 m。

| 输入 | 地图 | 命中覆盖率 (%) | 命中 MAE (m) | 含缺失惩罚 MAE (m) |
|---|---|---:|---:|---:|
| LaserScan+TF／LaserScan+Odometry／PointCloud2+TF | initial | 53.545 | 0.23775 | 7.09555 |
| 同上 | joint | 100.000 | 0.02426 | 0.02426 |
| 同上 | reintegrated | 83.624 | 0.20028 | 2.62383 |
| PointCloud2+scan matching | initial | 54.286 | 0.24365 | 6.98941 |
| 同上 | joint | 100.000 | 0.02567 | 0.02567 |
| 同上 | reintegrated | 66.984 | 0.16557 | 5.06328 |

阈值 0.5 时三类地图覆盖率均为 100%；前三条链路 initial/joint/reintegrated 的 MAE 为 0.27421/0.12565/0.23229 m，scan-matching 链路为 0.26975/0.14946/0.21882 m。直接优化场在这两种阈值下均优于初始和重建场，但绝对误差明显依赖概率等值阈值。本结果只适用于此房间、这套原生 0.3 m 网格和已声明的射线评价；不是高分辨率显示图的精度结论。

### 文件与复现

以下目录相对于 `/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/`：

- `occupancy_simulation_quantitative_20261020_01/`：四条真实运行、完整输入／参数／后端日志、三种地图、真值、逐帧误差、`metrics.csv`、`metrics.json`、`pose_errors.png/.pdf`。各场景 `*_evaluation/metrics.json` 保存匹配索引与坐标变换。`commands.json` 是实际前端命令。
- `occupancy_simulation_maps_20261020_01/`：首次三类地图射线评估，含逐射线 NPZ、两个阈值下全部方法的 CSV/JSON。
- `occupancy_simulation_eval_checks_20261020_01/`：可复现地图入口输出与首次评估完全一致；CTest 4/4，其中新增评估器含 5 项已知答案检查（坐标变换、尺度不拟合、角度跨界、相对运动、严格对应、场插值与缺失射线在相应测试中覆盖）；保存评估源码快照。

从 `/home/yingyuwang/Documents/ws_SDF_BA/src` 执行，运行名须使用新名字：

```bash
python3 sdf_ba/tools/run_on_data.py --name occupancy_room_quant_new -- bash -c '
  source /home/yingyuwang/Documents/Occupancy-SLAM/tools/env.sh
  python3 /home/yingyuwang/Documents/Occupancy-SLAM/tests/evaluate_room_simulation.py
'
python3 sdf_ba/tools/run_on_data.py --name occupancy_room_maps_new --   python3 /home/yingyuwang/Documents/Occupancy-SLAM/tests/evaluate_room_maps.py   --source /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_room_quant_new
```

通用轨迹评估入口为 `tools/evaluate_2d.py --result ... --ground-truth ... --output ...`，同样经数据盘启动器运行。真值可用三列 x/y/yaw 或四列 timestamp/x/y/yaw；TXT 抽帧时显式提供 `--indices selected_indices.txt`。其他 simulation 的地图若有栅格真值、墙面线段或生成器，需按其真实坐标和传感器模型提供对应地图评价，不能套用本房间边界。


## MATLAB simulation 数据集（2026-10-20）

以下为早期适配和诊断历史；正式入口、默认配置及最终 41 组精度已由文末“原 MATLAB 对齐与完整粗到细复测”更新。历史命令使用当时源码；当前默认入口已改为无损观测和完整粗到细，不能用新默认值重现旧端点重采样测试。

**后续精度复核更正：** 下述 41 组验证的是强制单分辨率、CG 200/0.05 的适配入口，不能作为完整粗到细算法的精度结论。独立 MATLAB 直接读取原始 MAT 真值核验 82 份轨迹，与 Python 最大差为 2.22e-15；同输入充分求解及原选区精细阶段的受控对照，已证实求解精度和遗漏阶段会明显影响结果。两例最终位置 RMSE 分别为 0.03349 m（Simu2680/odom_1）和 0.03541 m（Simu2640/carto_1），使用固定的第三次细化结果，没有按 GT 选择最优步。N 的导数近似未改。

详见[独立核验与诊断报告](/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_accuracy_report_20261020_01/REPORT.html)。原报告已加更正说明，历史测量保留。本次只完成两组诊断；选区细化位于 `tests/refine_2d_reference.cpp`，尚未接入正式入口，尚未完成修正流程下全部 41 组复测。原二维 MATLAB 入口及 simulation 参数仍待定位，不能将诊断声称为原论文流程的精确复现。

数据来自 `~/Downloads/Simu2640_Data`、`Simu2680_Data`、`Simu3640_Data`、`Simu3720_Data`。转换保留传感器局部 XY 端点和角度噪声，核验原始自由空间采样，处理 MATLAB v5/v7.3 以及部分 Cartographer 缺首帧的对应关系。优化器及 N 近似未修改。算法配置为共同的单分辨率 0.2 m、MaxIter=10、legacy、WeightO=0、CG 200/0.05；完整低频 Scan 每帧保留 1081 射线。该测试不等同于原 MATLAB 多分辨率论文实验，也不重新运行 ROS bag 前端。

实验结果、运行日志、报告、科学图及源码归档均物理存储在 `/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/`：

- [完整定量报告](/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_summary_20261020_02/REPORT.html)：逐组／分组轨迹误差、三种原生地图在两个阈值下的几何误差、退化、计算成本和评价限制。
- [轨迹 CSV](/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_summary_20261020_02/all_trajectory_metrics.csv)、[地图 CSV](/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_summary_20261020_02/all_map_metrics.csv)、[汇总及图表](/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_summary_20261020_02/)；均值按每场景五份里程计和五份 Cartographer 初值分组，Simu2680 的额外 Noise_24 单列。重复初值不当作独立场景。
- [转换结果](/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_conversion_20261020_01/converted/)：输入来源、帧对应、扫描重现检查和 GT；[全部 C++ 运行](/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_all_20261020_01/)：参数、实际 argv、三地图、每次迭代、状态、逐帧误差；[独立几何评价](/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_maps_20261020_01/)：几何模型验证、可见支持、全部指标。
- [原生 RViz topic 切换截图](/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_rviz_20261020_01/switches/)、[其他三个场景 RViz 截图](/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_rviz_scenes_20261020_01/)：标准 OccupancyGrid；initial／reintegrated 显示分辨率 0.05 m，joint 保持原生 0.2 m。定量评价读取原生 M/N，不读取分级显示图。
- [构建、CTest 和源码归档](/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_final_checks_20261020_01/)：包含本轮与重构完成后数值源码的逐字节核对。N 仍忽略位姿导数并在每次接受更新后重新计算。

以下从 `ws_SDF_BA/src` 执行，使用新的运行名。Python 环境先安装 `requirements-matlab.txt`；本机本轮 h5py 隔离安装在数据盘 `occupancy_matlab_runtime_20261020_01/python`，没有修改全局 Python。

```bash
python3 sdf_ba/tools/run_on_data.py --name occupancy_matlab_conversion_new -- \
  env PYTHONPATH=/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_runtime_20261020_01/python \
  python3 /home/yingyuwang/Documents/Occupancy-SLAM/tools/convert_matlab_simulations.py \
  --source /home/yingyuwang/Downloads \
  --output /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_conversion_new/converted

python3 sdf_ba/tools/run_on_data.py --name occupancy_matlab_all_new -- \
  python3 /home/yingyuwang/Documents/Occupancy-SLAM/tools/benchmark_matlab_simulations.py \
  --converted /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_conversion_new/converted --jobs 2

python3 sdf_ba/tools/run_on_data.py --name occupancy_matlab_maps_new -- \
  python3 /home/yingyuwang/Documents/Occupancy-SLAM/tools/evaluate_simulation_maps.py \
  --converted /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_conversion_new/converted \
  --results /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_all_new

python3 sdf_ba/tools/run_on_data.py --name occupancy_matlab_summary_new -- \
  python3 /home/yingyuwang/Documents/Occupancy-SLAM/tools/summarize_matlab_simulations.py \
  --converted /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_conversion_new/converted \
  --results /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_all_new \
  --maps /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_maps_new
```

打开任意场景的既有结果（下面固定为 Simu2640/carto_1；场景名可替换为其余三个）：

```bash
python3 sdf_ba/tools/run_on_data.py --name occupancy_matlab_view_new -- \
  bash /home/yingyuwang/Documents/Occupancy-SLAM/tools/view_rviz.sh 2d \
  --result /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_all_20261020_01/Simu2640/carto_1 \
  --display-resolution 0.05
```

在 RViz 的 Map / Topic 中选择 `/occupancy_slam/initial/map_display`、`/occupancy_slam/optimized/map_display` 或 `/occupancy_slam/reintegrated/map_display`。


## 原 MATLAB 对齐与完整粗到细复测（2026-10-20）

原入口已在 `/home/yingyuwang/Downloads/2D_Occupancy_SLAM_MATLAB` 找到。早期批量入口强制单层、使用较松 CG 设置并改变了自由空间采样，不能代表完整算法。现在正式 C++、TXT 和 bag 包装都传递 `ModeMulti`；无损 `samples.bin` 保留每个原始 `Scan.xy/Odd`；初始分箱、迭代 N 分箱、完整/选区 N 平滑、两阶段地图和求解参数独立生效。

**全部 41 个初值已实际完成：位置和航向 RMSE 均 41/41 改善，最终二维欧氏位置 RMSE 1.107–2.368 cm，航向 0.0340–0.0963°。** 四场景各五份里程计、五份 Cartographer，加 Simu2680/Noise_24；初值重复不当作独立场景。最终精细阶段均按既有步长阈值停止；Simu3720 部分粗阶段仍到预算上限，不声称整个目标达到严格驻点。

共同参数为 `config/backend/simulation_multires_extended.txt`：精细 0.1 m、粗 0.5 m、最多 40 粗 + 10 细，阈值提前停止；直接求解、相同原始样本及权重。该预算在短预算个例耗尽后选定，再统一重跑全部 41 组；属于同批次诊断修复验证，不是新场景泛化证明。没有按 GT 选择中间步。首帧确定所有方法共享的 SE(2)，不重新拟合最终轨迹、不拟合尺度，固定首帧不计入误差。下表为每组五次 RMSE 的均值。

| 场景 | 初值来源 | 初始位置 m | 最终位置 cm | 最终航向 ° |
|---|---|---:|---:|---:|
| Simu2640 | Odometry | 1.0668 | 2.091 | 0.0637 |
| Simu2640 | Cartographer | 0.3747 | 2.324 | 0.0735 |
| Simu2680 | Odometry | 1.3806 | 1.922 | 0.0691 |
| Simu2680 | Cartographer | 0.2690 | 1.609 | 0.0551 |
| Simu3640 | Odometry | 1.4021 | 1.620 | 0.0395 |
| Simu3640 | Cartographer | 0.4338 | 1.462 | 0.0417 |
| Simu3720 | Odometry | 1.5098 | 1.349 | 0.0710 |
| Simu3720 | Cartographer | 0.2246 | 1.419 | 0.0819 |

原 MATLAB 当前保存状态含活动 `Pose=Pose_GT`，`MULTI_MODE=FALSE`、0.1 m、20 步；参考入口改用真实噪声初值，原文件不修改。`DOWN_TIME=0` 使首步就使用 `SecondSmoothWeigh=1e-7` 和 `MinMeanDeltaSecond=300`，完整 N 平滑硬编码为 10；初始 round、更新 N floor。`simulation_matlab.txt` 映射的是这条活动路径。Simu2680/odom_1 的完整 20 次 MATLAB/C++ 迭代最大坐标分量差 **1.01555e-11 m**，最大角度差 **2.18737e-12 rad**；最终欧氏 RMSE 同为 0.055899495 m。原 `FuncEval` 的 `(RMSE_x+RMSE_y)/2` 为 0.036418382 m，另存独立字段，不能与欧氏 RMSE 混用。

**N 的 pose 导数没有增加。** 每次接受更新后从零累计并平滑 N。原数值函数与前一交付源码比对；新流式法方程与原 Jacobian 乘积（含非恒定 N、固定首帧、odometry 和选区）核对。完整单层参考上的逐步相等不外推为 MATLAB 与 C++ 粗到细逐步相等。

短预算 `simulation_multires.txt` 的 8+3 仍有 Simu3640/odom_2 的 1.003913 m 结果，已保留。仅增加预算的个例诊断和之后统一全量重跑均得到 0.017061596 m、19 次更新。默认入口现使用较充足预算；不再默认使用早期单层低预算参数。

### 地图、RViz 与资源

`initial/joint/reintegrated` 全部保存；joint/reintegrated pose 完全相同，重建从空场使用全部原始样本。四场景共 246 份原生地图几何评价：p=0.50 时 joint 和 reintegrated 的 capped 双向距离都在 41/41 组优于 initial，joint 仅在 10/41 组该距离优于 reintegrated；F1@0.1m 的排序不同。p=0.65 的 82 份 initial/reintegrated 没有等值线，未截断距离记缺失、capped 记 1m、F1 记 0，未删行。这不是 0.05m 显示图的精度评估，不能凭轨迹改善宣称联合地图所有指标都更好。

RViz 实测四个场景的 initial → reintegrated → optimized 原生 Topic 切换：相机、map 坐标系与 Transient Local 不变。初始/重建 0.05m，联合变量保留 0.1m；黑灰白标准 OccupancyGrid。选区联合地图只在所优化的邻近区域提供变量，远处灰色不能解释为重建未覆盖。二维保留既有启动 shader 诊断，实际绘制和干净退出通过。

主测试每次实测 25.26–274.24s，最大采样 RSS 2.902GiB，所测进程 VmSwap 均为 0；各次时间/内存配对保存，运行存在同机并发，不能据此声称独占 CPU 性能。

数据盘目录均相对 `/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/`：

| 内容 | 目录 |
|---|---|
| 主结果完整图表、逐组 CSV、LaTeX | `occupancy_simulation_extended_summary_20261020_01/REPORT.html` |
| 主结果全部实际运行与每步位姿 | `occupancy_simulation_extended_all_20261020_01/` |
| 三种原生地图、独立几何评价 | `occupancy_simulation_extended_maps_20261020_01/` |
| 短预算完整对照 | `occupancy_simulation_multires_summary_20261020_02/REPORT.html` |
| 原 MATLAB 完整 20 步、源码副本及实际参数 | `occupancy_matlab_reference_full_20261020_01/` |
| MATLAB 单层配置的 C++ 对照 | `occupancy_matlab_aligned_all_20261020_01/` |
| 最新无损转换和配置 | `occupancy_simulation_conversion_final_20261020_01/converted/` |
| 新旧样本、位姿、真值逐字节核对 | `occupancy_simulation_conversion_check_20261020_01/verification.json` |
| 默认入口无额外 profile/samples 参数的复测 | `occupancy_simulation_default_entry_20261020_01/` |
| Simu2680/odom_1 最终 RViz 切换 | `occupancy_simulation_extended_rviz_20261020_01/checks/2d/` |
| 其余三场景固定 carto_1 的实际 RViz | `occupancy_simulation_scene_views_20261020_01/REPORT.html` |
| 7/7 CTest、构建 | `occupancy_alignment_final_unit_20261020_01/` |
| TXT/PCD/显示架构集成检查 | `occupancy_alignment_architecture_20261020_01/` |
| 带注释参数显示导出回归 | `occupancy_multires_display_regression_20261020_01/` |
| 新法方程的 N 近似和非恒定 N 回归 | `occupancy_streaming_n_regression_20261020_01/` |

RViz 首次新结果测试因显示层旧参数解析器不能处理注释而失败；改为公共解析器后通过。汇总器首次遇到缺失等值线距离而停止；现在保留缺失、有效计数与惩罚，不删除案例。历史失败日志保留。端点独立重建测试曾在恰好落格点的浮点边界上差两格；保留数值诊断，将合成测试原点移离浮点边界后严格计数/数值核验通过。原 MATLAB 参考逐步相等另行实际验证，不用合成测试代替。

### 重新运行

所有源/配置路径使用绝对路径，运行名换成未使用的新名字。Python 环境需 `requirements-matlab.txt`；本机 h5py 位于已有数据盘运行的 `python/` 中，示例如下。默认转换直接生成无损 samples，旧转换目录也能按来源记录自动无损导出；只有显式 `--resample-endpoints` 才重采样。

```bash
python3 /home/yingyuwang/Documents/ws_SDF_BA/src/sdf_ba/tools/run_on_data.py --name occupancy_exact_conversion_new -- \
  env PYTHONPATH=/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_matlab_runtime_20261020_01/python \
  python3 /home/yingyuwang/Documents/Occupancy-SLAM/tools/convert_matlab_simulations.py \
  --source /home/yingyuwang/Downloads \
  --output /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_exact_conversion_new/converted

python3 /home/yingyuwang/Documents/ws_SDF_BA/src/sdf_ba/tools/run_on_data.py --name occupancy_complete_sim_new -- \
  python3 /home/yingyuwang/Documents/Occupancy-SLAM/tools/benchmark_matlab_simulations.py \
  --converted /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_exact_conversion_new/converted --jobs 4

python3 /home/yingyuwang/Documents/ws_SDF_BA/src/sdf_ba/tools/run_on_data.py --name occupancy_complete_maps_new -- \
  python3 /home/yingyuwang/Documents/Occupancy-SLAM/tools/evaluate_simulation_maps.py \
  --converted /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_exact_conversion_new/converted \
  --results /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_complete_sim_new

python3 /home/yingyuwang/Documents/ws_SDF_BA/src/sdf_ba/tools/run_on_data.py --name occupancy_complete_report_new -- \
  python3 /home/yingyuwang/Documents/Occupancy-SLAM/tools/summarize_matlab_simulations.py \
  --converted /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_exact_conversion_new/converted \
  --results /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_complete_sim_new \
  --maps /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_complete_maps_new

# 原 MATLAB 单层路径的独立数值参考：
python3 /home/yingyuwang/Documents/ws_SDF_BA/src/sdf_ba/tools/run_on_data.py --name occupancy_original_matlab_new -- \
  python3 /home/yingyuwang/Documents/Occupancy-SLAM/tools/run_matlab_reference.py \
  --source /home/yingyuwang/Downloads/2D_Occupancy_SLAM_MATLAB \
  --converted /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_exact_conversion_new/converted \
  --scene Simu2680 --case odom_1 --iterations 20
```

打开最终实测结果：

```bash
/home/yingyuwang/Documents/Occupancy-SLAM/tools/view_rviz.sh 2d \
  --result /media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_simulation_extended_all_20261020_01/Simu2680/odom_1 \
  --display-resolution 0.05
```


## 追加：保持数值结果的效率改进（2026-10-20）

本轮修改邻居索引、法方程存储、重复积分和显示／评价缓存，不更换目标函数、采样、网格分辨率、停止阈值或求解器。**N 的 pose 导数仍按论文近似省略，每次接受更新后从零累计并平滑 N。**

实现范围：

- 选区平滑图的已排序邻居索引改用二分查找，避免逐邻居扫描整个数组。
- 二维直接 LDLT 分支只组装下三角，残差检查使用完整自伴随视图。CG 分支仍保留原完整矩阵和两侧独立累加顺序；三维默认完整矩阵接口保持兼容。
- 每个优化阶段仅建立一次不变的变量索引。N 更新只累计计数；只需 M 的高分辨率建图只累计 M。选区初始化复用刚生成且采用相同 floor 规则的原始计数；round 模式仍按原规则重算。
- 二维显示图跨查看器启动持久缓存，输入或绘图代码元数据变化后自动重建；地图评价在同一方法的两个阈值间复用 M/N，在同一场景复用参考 KD-tree。

### 完整结果回归

最终核对：`/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/occupancy_efficiency_final_20261020_02/REPORT.html`。

| 检查 | 实测结果 |
|---|---|
| 四场景全部 41 组完整优化 | 41/41 通过；共核对 2272 个后端文件 |
| 714 份实际迭代位姿、全部初始／联合／重建 M/N、地图与最终位姿 | 与效率修改前主结果逐字节相同；迭代次数、停因和 N 更新次数相同 |
| 41 × 3 方法 × 2 阈值原生地图评价 | 246 行指标全部精确相同，空等值线及其缺失值保留 |
| 轨迹精度 | 欧氏 RMSE 仍为 0.011066110793–0.023679659784 m，没有精度退化 |
| 唯一后端日志数值差异 | `linear_relative_residual` 的最大绝对差 3.649e-15；下三角自伴随乘法的舍入差异，不影响解、成本、接受步或停止条件 |
| 单元测试 | 最终 CTest 7/7 通过，含 N 近似、法方程、下三角求解、计数／场独立积分、显示缓存失效与重建 |
| 架构回归 | 原始 TXT CG、legacy/decrease、PCD/manifest、三维 Jacobian、0.05/0.1 m 导出通过；保持既有 1e-12 容差 |
| 输入管线 | 3 类 TXT、3 条更名 topic 的 LaserScan/PointCloud2 ROS1 bag 链路、2 个错误 topic 拒绝测试全部通过，实际调用 slam_toolbox |
| NC20 Short_Exp 40 帧 | 三种地图、轨迹、点云及数值输出与既有结果逐字节相同；10 个接受步，目标 3744.755103 → 1401.514464 |
| 标准 ROS2 显示消息 | 迟到订阅收到全部 3 个 OccupancyGrid，逐格等于缓存数组；本轮未重新操作 RViz GUI |

评价 JSON 的结果目录与 GT 目录字符串不同；逐字节检查了新旧路径所指向的 GT 文件，然后对全部评价数值、帧对应和对齐参数做精确比较。NC20 的 Python wrapper 元数据和既有 schema 版本变化单列，不计作数值相同文件，见 `nc20_equivalence.json`。

### 同机顺序配对性能

全部正确性作业结束后，将新旧二进制固定到 CPU 0 顺序执行；同一输入和参数。Simu2680 采用旧／新／新／旧顺序，各两轮，表中取中位数。Simu3720 仅比较全网格首步各一次，不代表完整 20 步速度；进程 VmSwap 均为 0，RSS 为同次进程的 VmHWM 采样。

| 工作负载 | 旧耗时 | 新耗时 | 耗时减少 | 旧峰值 RSS | 新峰值 RSS | RSS 减少 |
|---|---:|---:|---:|---:|---:|---:|
| Simu2680/odom_1，完整粗到细 9 步 | 13.533 s | 11.523 s | 14.86% | 1.806 GiB | 1.762 GiB | 2.46% |
| Simu3720/odom_1，单层全网格首步 | 32.825 s | 31.983 s | 2.56% | 6.408 GiB | 5.895 GiB | 8.00% |

同次 CPU 时间分别为 13.514 → 11.503 s、32.809 → 31.964 s。两轮完整运行显示稳定改善；首步时间仅一对测量，不据此宣称普遍加速。没有使用旧全量批次与新全量批次的并发 wall time 比较速度。稀疏分解仍是主要耗时；本轮未改变求解精度或在结构变化时复用符号分解。

真实 Simu2680 显示地图首次准备 5.0815 s，复用缓存 0.000656 s；这只统计 `prepare_display`，不包括加载 ROS 消息或启动 RViz。缓存行为和存储位置见 [RVIZ.md](RVIZ.md#独立地图导出)。地图评价的复用已做完整数值回归，但没有单独宣称其速度收益。

### 证据与复现

以下目录均位于 `/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/`：

| 内容 | 目录 |
|---|---|
| 最终报告、核对脚本、逐组核对、补丁、源码快照 | `occupancy_efficiency_final_20261020_02/` |
| 修改前源码与原二进制 | `occupancy_efficiency_implementation_20261020_01/` |
| 最终完整 41 组结果与每步位姿 | `occupancy_efficiency_all41_20261020_02/` |
| 最终 246 份原生地图指标 | `occupancy_efficiency_maps_20261020_01/` |
| 新旧二进制、6 次配对命令、时间、RSS、输出 | `occupancy_efficiency_paired_final_20261020_01/` |
| 最终单元测试 | `occupancy_efficiency_unit_final_20261020_01/` |
| 架构与 CG 兼容 | `occupancy_efficiency_architecture_20261020_03/` |
| TXT 与实际 ROS1 bag 管线 | `occupancy_efficiency_inputs_20261020_01/` |
| 真实三维 40 帧 | `occupancy_efficiency_nc20_20261020_01/` |
| 显示缓存与 ROS2 DDS | `occupancy_efficiency_display_validation_20261020_01/` |

重新运行 41 组和地图评价可沿用上一节命令及 `simulation_multires_extended.txt`。配对运行的每个子目录保存 `command.json`；将其中输出路径替换为新的数据盘目录，并通过 `run_on_data.py` 执行，保留 CPU 绑定和其余参数。`verify_equivalence.py` 核对本次归档结果；源码副本和 `task.patch` 用于明确此次修改与既有未提交内容的边界。

修正记录：最初把 CG 也改为下三角时触发既有 1e-12 回归失败，已恢复 CG 原始完整组装，未放宽容差。初次全量运行因这个修正主动终止，`all41_01` 不作为最终结果。首次最终比较把 GT 路径字符串差异计作失败；确认所有 GT 文件内容精确相同后单列元数据，`final_02` 通过。失败和被替代的记录均保留。


## 追加：Docker 功能交付（2026-10-20）

已构建 `occupancy-slam:jazzy`，使用官方 ROS2 Jazzy/Noble 基础镜像；C++ 二维／三维后端和 RViz 灰度扩展在镜像内重新编译。宿主 `build/`、`.runtime`、原始数据不进入镜像。入口和使用方法见 [DOCKER.md](DOCKER.md)。未修改优化数学、N 近似、采样或迭代参数。

实测镜像 ID：`sha256:d2d9dfe7a135ebc79183f2c6d9d680354d0b6bdd79a48710c69806c32410bc44`。Python 3.12.3、NumPy 1.26.4、SciPy 1.11.4、rosbags 0.11.8、slam_toolbox 2.8.5；完整构建包版本保留在日志中。

| 验证 | 结果 |
|---|---|
| 镜像全新编译 | 二维、三维、单元测试和 RViz 扩展成功；不使用宿主二进制 |
| 容器 CTest | 7/7 通过；宿主缓存适配回归也为 7/7 |
| 架构兼容 | TXT CG、PCD/manifest、三维 Jacobian、0.05/0.1 m 导出通过，与原二进制保持既有 1e-12 检查 |
| 参数和输入 | 3 类 TXT、3 条更名 topic 的 LaserScan/PointCloud2 bag 链路、2 个错误 topic 拒绝测试全部通过 |
| 公共快捷入口 | 从宿主工程目录使用相对 Data/Intel 和配置路径，自动创建数据盘目录，20 帧完成；bag inspect 可列出实际话题 |
| 完整 simulation 个例 | Simu2680/odom_1，268 帧、9 次更新；47 个后端文件全部与宿主结果逐字节相同，含每步位姿和三种地图；主评价指标精确相同，欧氏 RMSE 0.019567642968 m |
| 真实 NC20 | Short_Exp 40 帧、10 个接受步；29 个数值／地图文件逐字节相同，4 份扫描清单路径指向相同文件 |
| 真实 Cartographer Revo LDS | 80 帧选中，77 帧获得精确时间匹配的 slam_toolbox pose，完成 C++ 优化并输出三种地图 |
| 容器原生 RViz | 二维接收 14 个 topics，三维点云及显式占据场各 11 个；迟到订阅数值与文件一致，实际窗口和订阅正确，3 个窗口退出码均为 0 |
| 存储与用户 | UID 1000，输入和镜像只读，输出可写；临时目录、ROS 日志及 RViz 用户设置物理存储在数据盘 |

本次 Docker 未重跑全部 41 组 simulation 或完整 Revo/NC20 序列；不从这些功能检查推断全序列质量或 Docker 性能优势。C++ 个例输出精确相同；Python 附加的独立 SE2 拟合评价有最大 2.621e-14 的舍入差异，全部逐值保留，主评价不受影响。

全部证据位于 `/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/`：

- `occupancy_docker_report_20261020_01/REPORT.html`：报告、实际窗口截图、最终证据索引、源码快照。
- `occupancy_docker_build_20261020_01/`、`occupancy_docker_build_20261020_02/`：首次构建和最终构建日志。
- `occupancy_docker_unit_20261020_01/`、`occupancy_docker_architecture_20261020_01/`、`occupancy_docker_inputs_20261020_01/`：容器内测试。
- `occupancy_docker_simulation_20261020_01/`、`occupancy_docker_nc20_20261020_01/`、`occupancy_docker_revo_20261020_01/`：实际数据运行。
- `occupancy_docker_equivalence_20261020_02/verification.json`：逐文件数值核对和附加评价舍入差异。
- `occupancy_docker_rviz_20261020_02/checks/`：二维／三维／可选占据场的截图、DDS 和退出检查。
- `occupancy_docker_environment_20261020_01/`：只读输入、UID、临时文件与输出挂载检查。
- `occupancy_docker_quickstart_check_20261020_01/`、`occupancy_docker_quickstart_20261020_01/`、`occupancy_docker_inspect_20261020_01/`：公开命令验证。

修正记录：首轮 RViz 已显示地图并完成消息检查，但其默认用户目录在只读镜像内，保存 persistent_settings 报错；宿主 Docker 入口增加独立数据盘用户目录挂载后，`rviz_02` 全部通过，不修改宿主用户目录。二维保留已知单次 shader 启动诊断，实际地图清晰显示，三维无 RViz ERROR。首次逐 JSON 比较把附加独立拟合的浮点舍入差异视为失败；最终对主评价严格相等、附加项逐项记录并用既有 1e-12 数值精度核对。失败记录保留，后端输出没有改写。


## 追加：GitHub main 发布前验证（2026-10-20）

原远程默认分支名为 `master`，其提交 `705b2178f2d6f863a85d1705a6631389b9a8de9a` 完整保存在 `C_plus_plus`。`main` 包含当前 ROS2、Docker、TXT/bag/PCD、三类地图输出和 RViz 工程；README 首部明确不使用 ROS2 时选择 `C_plus_plus`。本次发布整理修改文档、忽略本地 Python 环境并清理尾随空白，没有修改优化数学、N 近似或配置值。

从待提交 Git 文件导出干净源码目录，不带宿主 `build/`、`.runtime`、`third_party/` 或 Python 环境，在 Docker 中重新获取 libigl/Python 依赖并编译二维、三维后端和 RViz 扩展。系统 apt 依赖层复用了已有 Docker 缓存，C++ 编译未复用宿主二进制。

| 发布检查 | 结果 |
|---|---|
| 干净源码 Docker 构建 | 通过，镜像 `occupancy-slam:github-main` |
| 单元测试 | 7/7 通过 |
| 架构与数值兼容 | 二维／三维与已有本机二进制按既有 1e-12 容差通过；PCD manifest、Jacobian、0.05/0.1 m 导出通过 |
| 输入配置 | 3 类 TXT、更名的 LaserScan+TF、LaserScan+Odometry、PointCloud2+TF，以及 2 个错误 topic 拒绝检查全部通过 |
| README Intel 示例 | 相对 Data/Intel 与参数路径，20 帧完成，initial/joint/reintegrated 三类输出齐全 |
| 发布文件 | README 本地链接、Python/Shell 语法及 git diff 空白检查通过；不新增实验结果或依赖二进制 |

数据盘证据目录仍为 `/media/yingyuwang/Data/experiments/sdf_ba/codex_runs/`：`occupancy_github_release_20261020_01`、`occupancy_github_unit_20261020_01`、`occupancy_github_architecture_20261020_01`、`occupancy_github_inputs_20261020_01`、`occupancy_github_quickstart_20261020_01`。发布目录的 `verification.json` 汇总退出状态及示例输出检查。本次未重新运行全部 simulation 或 RViz 窗口；对应完整检查见前面的效率和 Docker 验证记录。
