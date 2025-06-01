% Main function of 3D Occupancy SLAM
% Created by Yingyu Wang 24/10/2023

% load dataset
clear all;
close all;
clc;

% ***************************** Data Preparation ***************************
% Prepare your own data as the inputs including initial poses and corresponding
% point clouds
% Initial Poses: n times 6 matrix (x,y,z,eular angles(r,p,y)), where n is the 
% total number of LiDAR scan and corresponding poses
% Point clouds: 1 times n cells, each cell stores point clouds at a unique
% timestamp corresponding to initial poses, the dimenssion is 3 times m
% ***************************** Data Preparation ***************************


load ./Data/Demo.mat;

Odom = FuncCalOdomfromPose(Pose);
PoseGT = Pose; % load ground truth poses if you have
Param = FuncLoadParams();

FuncShowPointCloud(Pose,Scan,0,Param);
% Time consuming, turn off
if Param.Visualization 
    FuncShowOccupancyMap(Scan,Pose,5,Param.MaxRange,1);
end

% Evaluate the poses and draw trajectory comparison if you have ground
% truth
if Param.Evaluation
    FuncEvaluatePose(Pose,PoseGT,Param);
    FuncDrawTrajectory(Pose,PoseGT,4);
end

% Enale Preprocess if you don't load processed observation directly
if Param.PreProcess
    Obs = FuncEqualProcessObs(Scan,Param);
end

[Map,Param] = FuncInit3DOccupancyMap(Pose,Obs,Param);
[Pose,Iter] = FuncLeastSquares(Map,Pose,Odom,PoseGT,Obs,Scan,1,Param);