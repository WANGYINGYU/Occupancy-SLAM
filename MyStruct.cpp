//
// Created by Yingyu Wang on 12/4/2023.
//
#include "MyStruct.h"


ParamStruct SetParam(){
    ParamStruct ValParam;
    ValParam.FileDict = "./";
    ValParam.WeightO = 0;
    ValParam.EvaluateGT = true;
    ValParam.PosefromOdom = false;
    ValParam.ModeOdom = false; // If exists odometry inputs
    ValParam.ModeMulti = true; // If use multi-resolution mode
    ValParam.ModeKeyFrame = false; // If use the key-frames mode

    // The map resolution
    ValParam.Sizei = 1500;
    ValParam.Sizej = 1500;
    ValParam.Scale = 0.05;
    ValParam.OriginX = -35;
    ValParam.OriginY = -35;

    // Multi-resolution mode
    ValParam.DownTime = 15;
    ValParam.DownRate = 10;
    ValParam.SelectDistance = 0.5;
    ValParam.SelectKernelSize = 3;

    //Key-frame mode
    ValParam.KeyframeRate = 10;

    // Paramters for Iteration Algorithm
    ValParam.MaxIter = ValParam.DownTime + 5;
    ValParam.MinMeanDeltaFirst = 2000;
    ValParam.MinMeanDeltaPoseFirst = 0.0006;
    ValParam.MinDelta = 100;
    ValParam.MinDeltaPose = 0.0002;
    ValParam.WeightSmoothN = 1;
    ValParam.MapSmoothingWeightFirst = 1e-7;
    ValParam.MapSmoothingWeightSecond = 1e-7;

    //
    ValParam.NumBeam = 1081;
    ValParam.MinAngle = -135.0/180.0*M_PI;
    ValParam.MaxAngle = 135.0/180.0*M_PI;;
    ValParam.ValOccupied = 0.847297860387203;
    ValParam.ValFree = -0.405465108108164;

    ValParam.MaxRange = 60.0;
    ValParam.MinRange = 0.0;

    // Show Map Mode
    ValParam.ModeShowMap = false;
    // Sparse System Solver in the First Stage
    ValParam.ModeSparseSolver = true; //Iterative Solver for 'true' and Direct Solver for 'false'

    // For Iterative Solver
    // First Stage
    ValParam.SolverFirstMaxIter = 200;
    ValParam.SolverFirstTolerance = 0.05;
    // Second Stage
    ValParam.SolverSecondMaxIter = 100;
    ValParam.SolverSecondTolerance = 0.01;



    return ValParam;
}


