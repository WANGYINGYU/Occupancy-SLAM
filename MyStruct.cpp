//
// Created by Yingyu Wang on 12/4/2023.
//
#include "MyStruct.h"


ParamStruct SetParam(){
    ParamStruct ValParam;
    ValParam.WeightO = 0; // Weight for the odometry term
    ValParam.EvaluateGT = false; // If evaluate the ground truth
    ValParam.PosefromOdom = true; // If use odometry as the initial pose
    ValParam.ModeOdom = true; // If exists odometry inputs
    ValParam.ModeMulti = true; // If use multi-resolution mode
    ValParam.ModeKeyFrame = false; // If use the key-frames mode

    // The map resolution
//    ValParam.Sizei = 1500;
//    ValParam.Sizej = 1500;
//    ValParam.Scale = 0.05;
//    ValParam.OriginX = -35;
//    ValParam.OriginY = -35;

//    ValParam.Sizei = 900; // The map size
//    ValParam.Sizej = 900; // The map size
//    ValParam.Scale = 0.1; // The map resolution
//    ValParam.OriginX = -60; // The origin
//    ValParam.OriginY = -50; // The origin

//    ValParam.Sizei = 1500; // The map size in v(vertical) direction
//    ValParam.Sizej = 1500; // The map size in u(horizontal) direction
//    ValParam.Scale = 0.1; // The map resolution
//    ValParam.OriginX = 80; // The origin of the map in u direction
//    ValParam.OriginY = -120; // The origin of the map in v direction

    ValParam.Sizei = 1200/2; // The map size
    ValParam.Sizej = 1000/2; // The map size
    ValParam.Scale = 0.1; // The map resolution
    ValParam.OriginX = -15; // The origin
    ValParam.OriginY = -20; // The origin

    // Multi-resolution mode
    ValParam.DownTime = 15; // The maximum times of first stage
    ValParam.DownRate = 10; // The downsample rate between two stages
    ValParam.SelectDistance = 0.5; // The selection distance from objects' boundary
    ValParam.SelectKernelSize = 3; // The selection kernel size

    //Key-frame mode
    ValParam.KeyframeRate = 10; // The key-frame rate

    // Paramters for Iteration Algorithm
    ValParam.MaxIter = ValParam.DownTime + 3; // The maximum iteration times
    ValParam.MinMeanDeltaFirst = 2000; // The threshold of minimum mean delta for the first stage
    ValParam.MinMeanDeltaPoseFirst = 0.0006; // The threshold of minimum mean delta w.r.t. poses for the first stage
    ValParam.MinDelta = 100; // The threshold of minimum delta
    ValParam.MinDeltaPose = 0.0002; // The threshold of minimum delta w.r.t. poses
    ValParam.WeightSmoothN = 1; // Weight for the smoothing term of Hit Map N
    ValParam.MapSmoothingWeightFirst = 1e-7; // Weight for the smoothing term of Occupancy Map for the first stage
    ValParam.MapSmoothingWeightSecond = 1e-7; // Weight for the smoothing term of Occupancy Map for the second stage

    //
    ValParam.NumBeam = 1081;
    ValParam.MinAngle = -135.0/180.0*M_PI;
    ValParam.MaxAngle = 135.0/180.0*M_PI;
//    ValParam.NumBeam = 1079; // The number of beams
//    ValParam.MinAngle = -2.35183119774; // The minimum angle of beams
//    ValParam.MaxAngle = 2.35183119774; // The maximum angle of beams

    ValParam.MaxRange = 30.0; // The maximum range of beams
    ValParam.MinRange = 0.023; // The minimum range of beams

    // Show Map Mode
    ValParam.ModeShowMap = false; // If show the map during the iteration, suggestion: false
    // Sparse System Solver in the First Stage
    ValParam.ModeSparseSolver = true; //Iterative Solver for 'true' and Direct Solver for 'false'

    // For Iterative Solver
    // First Stage
    ValParam.SolverFirstMaxIter = 200; // The maximum iteration times of iterative solver for the first stage
    ValParam.SolverFirstTolerance = 0.05; // The converge tolerance of iterative solver for the first stage
    // Second Stage
    ValParam.SolverSecondMaxIter = 50; // The maximum iteration times of iterative solver for the second stage
    ValParam.SolverSecondTolerance = 0.01; // The converge tolerance of iterative solver for the second stage

    return ValParam;
}


