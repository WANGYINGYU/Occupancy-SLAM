<h1 align="center">
    Occupancy-SLAM
</h1>

C++ implementation of *<u>Occupancy-SLAM: An Efficient and Robust Algorithm for Simultaneously Optimizing Robot Poses and Occupancy Map. Yingyu Wang, Liang Zhao, and Shoudong Huang</u>* and *<u>[Occupancy-SLAM: Simultaneously Optimizing Robot Poses and Continuous Occupancy Map](https://www.roboticsproceedings.org/rss18/p003.pdf). Liang Zhao, Yingyu Wang, and Shoudong Huang. In Robotics Science and Systems (RSS), 2022</u>*.



This paper considers the SLAM problem using 2D laser scans (and odometry). We propose an optimization based SLAM approach to optimize the robot trajectory and the occupancy map simultaneously. **The key novelty is that the robot poses and the 2D occupancy map are optimized together, which is significantly different from existing occupancy mapping strategies where the robot poses need to be obtained first before the map can be estimated.** In this formulation, the map is represented as a continuous occupancy map where each 2D point in the environment has a corresponding evidence value, and the state variables include all the robot poses and the occupancy values at the discrete grid cell nodes of the occupancy map. Based on this formulation, a multi-resolution optimization framework that uses occupancy maps with different resolutions in different stages is introduced. A variation of Gauss-Newton method is proposed to solve the optimization problem in different stages to obtain the optimized occupancy map and robot trajectory. The proposed algorithm is very efficient and can easily converge with initialization from either odometry inputs or scan matching, even when only limited key frame scans are used. Furthermore, we propose an occupancy submap joining method so that large-scale problems can be more effectively handled by integrating the submap joining method with the proposed Occupancy-SLAM. Evaluations using simulations and practical 2D laser datasets demonstrate that the proposed approach can robustly obtain more accurate robot trajectories and occupancy maps than the state-of-the-art techniques with comparable computational time.



## Associated Video

[![Alt text](https://img.youtube.com/vi/WH2noA4KQCM/0.jpg)](https://www.youtube.com/watch?v=WH2noA4KQCM)

## Dependencies

1. [Eigen >= 3.4.0](https://eigen.tuxfamily.org/index.php?title=Main_Page)
2. [CMake >= 3.16](https://cmake.org)
3. [OpenMP](https://www.openmp.org)
4. [OpenCV](https://opencv.org)
5. [libigl](https://libigl.github.io)
6. [Intel MKL](https://www.intel.com/content/www/us/en/developer/tools/oneapi/onemkl.html) (Option)



### Ubuntu

#### Eigen 3.4.0 

Download [Eigen 3.4.0](https://gitlab.com/libeigen/eigen/-/releases/3.4.0) 

```
cd eigen-3.4.0
mkdir build
cd build
cmake ..
sudo make install
```

#### CMake

`sudo apt-get install cmake`

#### OpenMP

```
sudo apt-get install libomp-dev
```

#### OpenCV

```
git clone https://github.com/opencv/opencv.git
mkdir build
cd build
cmake ..
make
sudo make install
```

#### libigl

Download [libigl](https://libigl.github.io) 



## Quickstart

### Download datasets

Download [datasets](https://drive.google.com/file/d/1EDZfsOru4z0j2OEq57DIKOyvudecWrkM/view?usp=share_link), and place the file under the main folder. 

### Modify the path of libigl

Specify the path to libigl in `CMakeLists.txt`.

### Modify the path of MKL or disable

Specify the path to mkl or remove all contents about mkl in `CMakeLists.txt`.

### Compile

```bash
git https://github.com/WANGYINGYU/Occupancy-SLAM.git
cd Occupancy-SLAM
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





## Guidance (For Your Own Datasets)

### Data Format

Preprocess the required data into the following format and store them as txt files.

#### Laser Scan

$m \times n$ matrix, where $m$ is the number of scans and $n$ is the number of beams. The range of the laser beam corresponds to each column of the matrix in the order of the corresponding angle from smallest to largest. 

#### Pose

$m \times 3$ matrix, where $m$ is the number of poses. The three elements of each row of the matrix correspond to x, y and angle.

#### Odometry (Option)

$m \times 3$ matrix with the same format as poses. In addition, this input must be in increments format.

### Parameters Setting

Set parameters in `config.txt`. Refer to the comments in `MyStruct.cpp` for the effect of each parameter.



## Citation

If you find our work useful to your research, please cite the following paper:

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



## Our Results of Occupancy Grid Map



<table border="1" width="100%">
  <tr>
    <th>Dataset</th>
    <th>Initialization</th> 
    <th>Ours</th>
  </tr>
  <tr>
    <td width="0.5%">ACES</td>
    <td width="49%"><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/ACES_Ini.jpg?raw=true"></td> 
    <td width="50.5%"><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/ACES_Our.jpg?raw=true"></td>
  </tr>
  <tr>
    <td>Intel Lab</td>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/Intel_Ini.jpg?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/Intel_Our.jpg?raw=true"></td>
  </tr>
  	<td>C5</td>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/C5_Ini.jpg?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/C5_Our.jpg?raw=true"></td>
  </tr>
    <td>Museum b0</td>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/b0_Ini.jpg?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/b0_Our.jpg?raw=true"></td>
	</tr>
    <td>Freiburg Building 079</td>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/fr079_Ini.jpg?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/fr079_Our.jpg?raw=true"></td>
</tr>
    <td>Garage</td>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/garage_Ini.jpg?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/garage_Our.jpg?raw=true"></td>
</tr>
    <td>MIT</td>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/MIT_Ini.jpg?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/MIT_Our.jpg?raw=true"></td>
	</tr>
    <td>Simu 1</td>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/Simu1_Ini.jpg?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/Simu1_Our.jpg?raw=true"></td> 
	</tr>
    <td>Simu 2</td>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/Simu2_Ini.jpg?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/Simu2_Our.jpg?raw=true"></td>
	</tr>
    <td>Simu 3</td>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/Simu3_Ini.jpg?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/Simu3_Our.jpg?raw=true"></td>
</table>
