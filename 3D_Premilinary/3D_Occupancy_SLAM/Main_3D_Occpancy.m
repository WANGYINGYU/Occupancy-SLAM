% Main function of 3D Occupancy SLAM
% Created by Yingyu Wang 24/10/2023

% load dataset
clear all;
close all;
clc;

load ./Data/Real_3D_Lidar_Data/Voxgraph/Voxgraph_Demo.mat;

Pose = Odom;
Odom = FuncCalOdomfromPose(Pose);
PoseGT = Pose;
TrajectoryGT = PoseGT;
Param = FuncLoadParams();

FuncShowPointCloud(Trajectory,Scan,3);

FuncDrawTrajectory(Pose,TrajectoryGT,Pose,4);
 
FuncEvaluatePose(Pose,PoseGT,Param);

FuncShowOccupancyMap(Scan,Pose,5,Param.MaxRange,1);

if ~Param.PreProcess
    tic
    Obs = FuncEqualProcessObs(Scan,Param);
    toc
end
[Map,Param] = FuncInit3DOccupancyMapNorm(Pose,Obs,Param);
[Pose,Iter] = FuncLeastSquares(Map,Pose,Odom,PoseGT,Obs,Scan,TrajectoryGT,Timestamps,1,Param);