Preliminary MATLAB code for 3D Occupancy-SLAM. To get started, run *3D_Occupancy_SLAM* for standard-scale datasets, and *3D_Occupancy_Joining* for large-scale environments. Our approach produces consistent maps and poses without making any assumptions about the environment. Furthermore, it is robust to initialization errors, allowing the use of noisy initial poses. The results below demonstrate its performance in an unstructured environment captured by a UAV.

<table border="1" width="100%">
  <tr>
    <th>Odometry</th> 
    <th>Ours</th>
  </tr>
  <tr>
    <td width="49%"><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Odom_1.png?raw=true"></td> 
    <td width="50.5%"><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Odom_1.png?raw=true"></td>
  </tr>
  <tr>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Odom_2.png?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Ours_2.png?raw=true"></td>
  </tr>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Odom_3.png?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Ours_3.png?raw=true"></td>
  </tr>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Odom_4.png?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Ours_4.png?raw=true"></td>
	</tr>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Odom_5.png?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Ours_5.png?raw=true"></td>
</tr>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Odom_6.png?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Ours_6.png?raw=true"></td>
</tr>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Odom_7.png?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Ours_7.png?raw=true"></td>
	</tr>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Odom_8.png?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Ours_8.png?raw=true"></td> 
	</tr>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Odom_9.png?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Ours_9.png?raw=true"></td>
	</tr>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Odom_10.png?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Ours_10.png?raw=true"></td>
</tr>
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Odom_11.png?raw=true"></td> 
    <td><img src="https://github.com/WANGYINGYU/Occupancy-SLAM/blob/master/images/3D_ Preliminary/Ours_11.png?raw=true"></td>
</table>



A more efficient implementation and C++ code is being developed and will be released in the future.
