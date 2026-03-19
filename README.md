# Occupancy-SLAM

C++ implementation of:

- [Occupancy-SLAM: An Efficient and Robust Algorithm for Simultaneously Optimizing Robot Poses and Occupancy Map (IEEE T-RO)](https://arxiv.org/pdf/2502.06292)
- [Occupancy-SLAM: Simultaneously Optimizing Robot Poses and Continuous Occupancy Map (RSS 2022)](https://www.roboticsproceedings.org/rss18/p003.pdf)

Authors: Yingyu Wang, Liang Zhao, Shoudong Huang.

## Overview

Occupancy-SLAM jointly optimizes:

- robot poses
- occupancy values on map grid vertices

Unlike pipelines that optimize poses first and then build maps, this method solves pose and occupancy together in one optimization framework.

This repository mainly provides:

- **2D Occupancy-SLAM (C++)**
- **Preliminary 3D Occupancy-SLAM (MATLAB)** in `3D_Preliminary/`

## Repository Structure

- `Main_Occupancy_SLAM.cpp`: main entry for 2D
- `FuncLeastSquares.cpp`: optimization core
- `FuncConvertObs.cpp`: observation conversion
- `MyStruct.h/.cpp`: parameters and structures
- `SubFuncs.h/.cpp`: utilities
- `config.txt`: runtime configuration
- `Data/`: example datasets
- `3D_Preliminary/`: MATLAB 3D preliminary implementation

## Dependencies

Minimum requirements:

1. Eigen >= 3.4.0
2. CMake >= 3.16
3. OpenMP
4. OpenCV
5. libigl
6. Intel MKL (optional)

## Build (2D C++)

### 1) Clone

```bash
git clone https://github.com/WANGYINGYU/Occupancy-SLAM.git
cd Occupancy-SLAM
```

### 2) Configure dependencies

- Set `libigl` path in `CMakeLists.txt`.
- If you use MKL, set MKL path in `CMakeLists.txt`.
- If you do not use MKL, disable/remove MKL-related CMake parts.

### 3) Compile

```bash
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

On macOS, replace `$(nproc)` with `$(sysctl -n hw.ncpu)`.

## Run (2D C++)

```bash
./Occupancy_SLAM
```

Then input (example):

```text
../Data/Museum_b0/Range.txt ../Data/Museum_b0/Pose.txt
```

## Input Data Format

Prepare text files as below.

### Laser Scan

- Matrix size: `m x n`
- `m`: number of scans
- `n`: number of beams
- Each row is one scan; columns follow beam angles from small to large.

### Pose

- Matrix size: `m x 3`
- Columns: `x, y, yaw`

### Odometry (optional)

- Matrix size: `m x 3`
- Same format as pose, but in **incremental** form.

## Parameters

- Edit runtime parameters in `config.txt`.
- Parameter definitions are in `MyStruct.cpp`.

## 3D Preliminary (MATLAB)

The preliminary 3D implementation is under:

- `3D_Preliminary/`

Use MATLAB scripts in that folder for 3D experiments.

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

This project is released under the [MIT License](./LICENSE.txt).
