% Main Function of 3D Occupancy Map Joining
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


% ***************************** Note ***************************************
% This code is submap joining which is not used to optimize poses within
% individual submap.
% You should export optimized poses within individual submap from Occunpancy-SLAM 
% or your own method first. You can also consider modify this code to make
% it call Occupancy-SLAM to optimize internal poses within individual
% submap first.
% ***************************** Note ***************************************

load Data/Voxgraph_Demo.mat;
PoseGT = Pose; % Load ground truth trajectory if you have

% Set Parameters
Param = FuncLoadParams();

if Param.Evaluation
    FuncDrawTrajectory(Pose,PoseGT,4);
end

% Divid Data to Submap
[SubMapScan,SubMapPose,CoordinatePose] = FuncDivideData(Scan,Pose,Param);
SubMap= FuncInitLocalOccupancyMap(SubMapPose,SubMapScan,Param);
% Initialize Global Map
[GlobalMap,LocalObs,Param] = FuncInitGlobalOccupancyMap(CoordinatePose,SubMap,Param);
FuncMapJoiningGlobal2LocalNLLS(GlobalMap,SubMap,CoordinatePose,SubMapPose,LocalObs,PoseGT,Param);
