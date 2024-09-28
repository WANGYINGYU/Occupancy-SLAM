% Main Function of 3D Occupancy Map Joining
% Load Data
clear all;
close all;
clc;

load Data/Voxgraph_Demo.mat;

Odom = Pose;
FuncDrawTrajectory(Pose,PoseOur,Odom,4);

% Set Parameters
Param = FuncLoadParams();

FuncShowOccupancyMap(OriginScan,AllPose,5,Param.MaxRange,1);

PoseOdom = Odom;

GT = AllPose;

% Divid Data to Submap
[SubMapScan,SubMapPose,CoordinatePose] = FuncDivideData(Scan,AllPose,Param);
SubMap= FuncInitLocalUnevenOccupancyMap(SubMapPose,SubMapScan,Param);
% Initialize Global Map
[GlobalMap,LocalObs,Param] = FuncInit3DGlobalOccupancyMapUneven(CoordinatePose,SubMap,Param);
FuncMapJoiningGlobal2LocalNLLS(GlobalMap,SubMap,CoordinatePose,SubMapPose,PoseOdom,LocalObs,OriginScan,GT,Timestamps,Param);
