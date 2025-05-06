Preliminary MATLAB code for 3D Occupancy-SLAM. To get started, run *3D_Occupancy_SLAM* for standard-scale datasets, and *3D_Occupancy_Joining* for large-scale environments. Our approach produces consistent maps and poses without making any assumptions about the environment. Furthermore, it is robust to initialization errors, allowing the use of noisy initial poses. The results below demonstrate its performance in an unstructured environment captured by a UAV.

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



A more efficient implementation and C++ code is being developed and will be released in the future.
