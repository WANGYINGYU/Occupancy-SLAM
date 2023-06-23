<h1 align="center">
    Occupancy-SLAM-C++
</h1>

C++ implementation of Occupancy-SLAM: An Efficient and Robust Algorithm for Simultaneously Optimizing Robot Poses and Occupancy Map. Yingyu Wang, Liang Zhao, and Shoudong Huang. and [Occupancy-SLAM: Simultaneously Optimizing Robot Poses and Continuous Occupancy Map](https://www.roboticsproceedings.org/rss18/p003.pdf). Liang Zhao, Yingyu Wang, and Shoudong Huang. In Robotics Science and Systems (RSS), 2022.



This paper considers the 2D SLAM problem using 2D laser scans (and odometry) information. We propose an optimization based SLAM approach to simultaneously optimize the robot trajectory and the occupancy map. ***The key novelty is that the robot poses and the occupancy map are optimized together, which is significantly different from existing occupancy mapping strategies where the robot poses need to be obtained first before the map can be estimated.*** In this formulation, the map is represented as a continuous occupancy map where each 2D point in the environment has a corresponding evidence value, and the state variables include all the robot poses and the occupancy values at the discrete grid cell nodes of the occupancy map. Based on this formulation, a multi-resolution optimization framework which uses occupancy maps with different resolutions in different stages is introduced to improve the robustness, convergence and efficiency of the algorithm. A variation of Gauss-Newton method is proposed to solve the optimization problem to obtain the optimized occupancy map and robot trajectory. The proposed algorithm can easily converge with initialization from either odometry inputs or scan matching even when only limited selected key frame scans are used. Evaluations using simulations and practical 2D laser datasets demonstrate that the proposed approach can robustly obtain more accurate robot trajectories and maps than the state-of-the-art techniques, with less computation time if only selected key frames are used. 



## Dependencies

1. [Eigen >= 3.4.0](https://eigen.tuxfamily.org/index.php?title=Main_Page)
2. [CMake >= 3.16](https://cmake.org)
3. [OpenMP](https://www.openmp.org)
4. [OpenCV](https://opencv.org)
5. [libigl](https://libigl.github.io)



## Quickstart

### Download datasets

Download [datasets](https://drive.google.com/file/d/1EDZfsOru4z0j2OEq57DIKOyvudecWrkM/view?usp=share_link), and place the file under the main folder. 

### Modify the path of libigl

Specify the path to libigl in `CMakeLists.txt`.

### Compile

```bash
git https://github.com/WANGYINGYU/Occupancy-SLAM-CXX.git
cd Occupancy-SLAM-CXX
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j12
```

### Run

```bash
./Occupancy_SLAM
```

and then input

`../Data/Museum/b0/Range.txt ../Data/Museum/b0/Pose.txt `





## Guidance

### Data Format

Preprocess the required data into the following format and store them as txt files.

#### Laser Scan

$m \times n$ matrix, where $m$ is the number of scans and $n$ is the number of beams. The range of the laser beam corresponds to each column of the matrix in the order of the corresponding angle from smallest to largest. 

#### Pose

$m \times 3$ matrix, where $m$ is the number of poses. The three elements of each row of the matrix correspond to x, y and angle.

#### Odometry (Option)

$m \times 3$ matrix with the same format as poses. In addtion, this input must be increments.

### Parameters Setting

Set parameters in `MyStruct.cpp`. Refer to the comments in the file for the effect of each parameter.



## Citation

If you find our work useful to your research, please cite fllowing paper:

```
  
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

Our code is under [MIT](./LICENSE.txt) License. 



## Our Results of Map

<center>
<figure>
<img src="https://github.com/WANGYINGYU/Occupancy-SLAM-CXX/blob/master/images/ACES_Ini.jpg?raw=true" />
·
·
·
<img src="https://img2018.cnblogs.com/blog/1735896/202001/1735896-20200116162140471-237299356.png" />
</figure>
</center>
