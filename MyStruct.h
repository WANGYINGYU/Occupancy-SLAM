//
// Created by Yingyu Wang on 12/4/2023.

#include <string>
#ifndef FASTOCC_FUNCS_H
#define FASTOCC_FUNCS_H

#include <Eigen/Dense>
#include <vector>
#include <tuple>



struct ParamStruct{
    float WeightO;
    float WeightSmoothN;
    double WeightSmoothNSecond = 1.;
    bool EvaluateGT;
    bool PosefromOdom;
    bool ModeOdom; // If exists odometry inputs
    bool ModeMulti; // If use multi-resolution mode
    bool ModeKeyFrame; //If use the key-frames mode

    int Sizei;
    int Sizej;
    double Scale;
    bool FixedGrid = false;
    bool InitialMapRound = false;
    bool NormalizationRound = true;
    double OriginX;
    double OriginY;

    // Multi-resolution mode
    int DownTime;
    int DownRate;
    float SelectDistance;
    int SelectKernelSize;

    //Key-frame mode
    int KeyframeRate;

    // Paramters for Iteration Algorithm
    int MaxIter;
    double MinDelta;
    double MinDeltaPose;
    double MinMeanDeltaFirst;
    double MinMeanDeltaPoseFirst;
    double MapSmoothingWeightFirst;
    double MapSmoothingWeightSecond;


    //Convert Observations Parameters
    int NumBeam;
    double MinAngle;
    double MaxAngle;
    const double ValOccupied = 0.847297860387203;
    const double ValFree = -0.405465108108164;
    double MaxRange;
    double MinRange;

    //If Show Map
    bool ModeShowMap;

    // Sparse System Solver in the First Stage
    bool ModeSparseSolver;
    // For Iterative Solver
    //First Stage
    int SolverFirstMaxIter;
    double SolverFirstTolerance;
    //Second Stage
    int SolverSecondMaxIter;
    double SolverSecondTolerance;


};



ParamStruct SetParam();
#endif // FASTOCC_FUNCS_H
