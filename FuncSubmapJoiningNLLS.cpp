//
// Created by Yingyu Wang on 9/10/2024.
//
#include "iostream"
#include "MyStruct.h"
#include "SubFuncs.h"
#include <vector>
#include <algorithm>

void FuncSubmapJoiningNLLS(const std::vector<MapStruct>& SubMaps,MapStruct& GlobalMap,Eigen::MatrixXd& StatePoses,ParamStruct& ValParam)
{
    auto [JP,JD,IS,ErrorS,SumError,MeanError] = FuncDiffJoiningJacobianGlobal2Local(GlobalMap, StatePoses, SubMaps, ValParam);
    auto [DeltaP,DeltaD,SumDelta,MeanDelta,SumDeltaP,MeanDeltaP] = FuncDelta(JP, JD, ErrorS, IS, ValParam);
    std::cout<<"Mean Delta is "<< MeanDelta<< std::endl;
    std::cout<<"Mean Delta of Pose is "<< MeanDeltaP<< std::endl;
    FuncUpdate(GlobalMap.Grid, StatePoses, DeltaP, DeltaD);
    }
