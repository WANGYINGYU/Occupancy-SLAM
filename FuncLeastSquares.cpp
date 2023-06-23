//
// Created by Yingyu Wang on 30/4/2023.
//

#include "iostream"
#include "MyStruct.h"
#include "SubFuncs.h"
#include <vector>
#include <algorithm>

void FuncLeastSquaresOdom(Eigen::MatrixXd& Map, Eigen::MatrixXd& N, Eigen::MatrixXd& Pose, const Eigen::MatrixXd& PoseGT, const Eigen::MatrixXd& Odom, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd,
                      const std::vector<Eigen::ArrayXd>& LowScanXY, const std::vector<Eigen::ArrayXd>& LowScanOdd, const Eigen::SparseMatrix<double>& HH, ParamStruct& ValParam)
{
    int Iter = 1;
    double MeanDelta = 100000;
    double MeanDeltaP = 100;
    Eigen::MatrixXd SelectMap(ValParam.Sizei * ValParam.DownRate, ValParam.Sizej * ValParam.DownRate), SelectN(ValParam.Sizei * ValParam.DownRate, ValParam.Sizej * ValParam.DownRate);
    std::vector<Eigen::ArrayXd> SelectScanXY(ScanXY.size()), SelectScanOdd(ScanXY.size());
    Eigen::MatrixXi IdSelect, IdSelectVar;
    Eigen::SparseMatrix<double> HHSelect, WeightHHSelect;
    Eigen::SparseMatrix<double> WeightHH = HH * ValParam.MapSmoothingWeightFirst;
    while (MeanDeltaP > ValParam.MinDeltaPose && MeanDelta > ValParam.MinDelta && Iter <= ValParam.MaxIter ){
        std::cout<<"Iter"<<"  "<<Iter<<std::endl;
        if(Iter <= ValParam.DownTime)
        {
            auto [JP,JD,JO,IS,IO,ErrorS,ErrorO,SumError,MeanError] = FuncDiffJacobian(Map, N, Pose, Odom, LowScanXY, LowScanOdd, ValParam);
            auto [DeltaP,DeltaD,SumDelta,MeanDelta,SumDeltaP,MeanDeltaP] = FuncDelta(Map, Odom, JP, JD, JO, ErrorS, ErrorO, IS, IO, WeightHH, ValParam);
            std::cout<<"Mean Delta is "<< MeanDelta<< std::endl;
            std::cout<<"Mean Delta of Pose is "<< MeanDeltaP<< std::endl;
            FuncUpdate(Map, Pose, DeltaP, DeltaD);
            if (ValParam.ModeShowMap){
                FuncShowMap(Map);
            }
            FuncUpdateMapN(N, Pose, LowScanXY, ValParam);
            FuncEval(Pose, PoseGT, ValParam);
            if (MeanDelta < ValParam.MinMeanDeltaFirst){
                ValParam.DownTime = Iter;
                ValParam.MaxIter = ValParam.DownTime + 3;
            }
            FuncSmoothN2(N, HH, ValParam);
        }
        else if ((Iter >= ValParam.DownTime + 1) && ValParam.ModeMulti)
        {
            if (Iter == ValParam.DownTime + 1) {
                std::cout<<"Selection Stage"<< std::endl;
                ValParam.Sizei = ValParam.Sizei * ValParam.DownRate;
                ValParam.Sizej = ValParam.Sizej * ValParam.DownRate;
                ValParam.Scale = ValParam.Scale / ValParam.DownRate;
                auto [HighMap, HighN] = FuncInitialiseGridMap(Pose,ScanXY,ScanOdd,ValParam);
                FuncCalBound(HighMap, IdSelect, IdSelectVar, ValParam);
                FuncSelectScan(Map, Pose, ScanXY, ScanOdd, SelectScanXY, SelectScanOdd, IdSelect, ValParam);
                FuncInitialSelectMap(SelectMap, SelectN, Pose, SelectScanXY, SelectScanOdd, ValParam);
                FuncSelectMapConst(SelectMap, IdSelectVar, HHSelect, ValParam);
                WeightHHSelect = HHSelect * ValParam.MapSmoothingWeightSecond;
            }
            FuncSmoothSelectN2(SelectN, WeightHHSelect, IdSelectVar, ValParam);
            auto [JP, JD, JO, IS, IO, ErrorS, ErrorO, SumError, MeanError] = FuncDiffSelectJacobian(SelectMap, SelectN, Pose, Odom, SelectScanXY, SelectScanOdd, IdSelectVar, ValParam);
            auto [DeltaP,DeltaD,SumDelta,SecondMeanDelta,SumDeltaP,SecondMeanDeltaP] = FuncSelectMapDelta(SelectMap, Odom, JP, JD, JO, ErrorS, ErrorO, IS, IO, WeightHHSelect, IdSelectVar, ValParam);
            MeanDelta = SecondMeanDelta;
            MeanDeltaP = SecondMeanDeltaP;
            std::cout<<"Mean Delta is "<< SecondMeanDelta << std::endl;
            std::cout<<"Mean Delta of Pose is "<< SecondMeanDeltaP << std::endl;
            FuncSelectMapUpdate(SelectMap, Pose, DeltaP, DeltaD, IdSelectVar);
            if (ValParam.ModeShowMap){
                FuncShowMap(SelectMap);
            }
            FuncUpdateSelectN(SelectN, Pose, SelectScanXY, ValParam);
            FuncEval(Pose, PoseGT, ValParam);
            FuncSmoothSelectN2(SelectN, WeightHHSelect, IdSelectVar, ValParam);
        }
    Iter++;
    }
}



// No odometry inputs

void FuncLeastSquares(Eigen::MatrixXd& Map, Eigen::MatrixXd& N, Eigen::MatrixXd& Pose, const Eigen::MatrixXd& PoseGT, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd,
                      const std::vector<Eigen::ArrayXd>& LowScanXY, const std::vector<Eigen::ArrayXd>& LowScanOdd, const Eigen::SparseMatrix<double>& HH, ParamStruct& ValParam)
{
    int Iter = 1;
    double MeanDelta = 100000;
    double MeanDeltaP = 100;
    Eigen::MatrixXd SelectMap(ValParam.Sizei * ValParam.DownRate, ValParam.Sizej * ValParam.DownRate), SelectN(ValParam.Sizei * ValParam.DownRate, ValParam.Sizej * ValParam.DownRate);
    std::vector<Eigen::ArrayXd> SelectScanXY(ScanXY.size()), SelectScanOdd(ScanXY.size());
    Eigen::MatrixXi IdSelect, IdSelectVar;
    Eigen::SparseMatrix<double> HHSelect, WeightHHSelect;
    Eigen::SparseMatrix<double> WeightHH = HH * ValParam.MapSmoothingWeightFirst;
    while (MeanDeltaP > ValParam.MinDeltaPose && MeanDelta > ValParam.MinDelta && Iter <= ValParam.MaxIter ){
        std::cout<<"Iter"<<"  "<<Iter<<std::endl;
        if(Iter <= ValParam.DownTime)
        {
            auto [JP,JD,IS,ErrorS,SumError,MeanError] = FuncDiffJacobian(Map, N, Pose, LowScanXY, LowScanOdd, ValParam);
            auto [DeltaP,DeltaD,SumDelta,FirstMeanDelta,SumDeltaP,FirstMeanDeltaP] = FuncDelta(Map, JP, JD, ErrorS, IS, WeightHH, ValParam);
            std::cout<<"Mean Delta is "<< FirstMeanDelta<< std::endl;
            std::cout<<"Mean Delta of Pose is "<< FirstMeanDeltaP<< std::endl;
            FuncUpdate(Map, Pose, DeltaP, DeltaD);
            if (ValParam.ModeShowMap){
                FuncShowMap(Map);
            }
            FuncUpdateMapN(N, Pose, LowScanXY, ValParam);
            FuncEval(Pose, PoseGT, ValParam);
            if (FirstMeanDelta < ValParam.MinMeanDeltaFirst || FirstMeanDeltaP < ValParam.MinMeanDeltaPoseFirst){
                ValParam.DownTime = Iter;
                ValParam.MaxIter = ValParam.DownTime + 3;
            }
            FuncSmoothN2(N, HH, ValParam);
        }
        else if ((Iter >= ValParam.DownTime + 1) && ValParam.ModeMulti)
        {
            if (Iter == ValParam.DownTime + 1) {
                std::cout<<"Selection Stage"<< std::endl;
                ValParam.Sizei = ValParam.Sizei * ValParam.DownRate;
                ValParam.Sizej = ValParam.Sizej * ValParam.DownRate;
                ValParam.Scale = ValParam.Scale / ValParam.DownRate;
                auto [HighMap, HighN] = FuncInitialiseGridMap(Pose,ScanXY,ScanOdd,ValParam);
                FuncCalBound(HighMap, IdSelect, IdSelectVar, ValParam);
                FuncSelectScan(Map, Pose, ScanXY, ScanOdd, SelectScanXY, SelectScanOdd, IdSelect, ValParam);
                FuncInitialSelectMap(SelectMap, SelectN, Pose, SelectScanXY, SelectScanOdd, ValParam);
                FuncSelectMapConst(SelectMap, IdSelectVar, HHSelect, ValParam);
                WeightHHSelect = HHSelect * ValParam.MapSmoothingWeightSecond;
            }
            FuncSmoothSelectN2(SelectN, WeightHHSelect, IdSelectVar, ValParam);
            auto [JP, JD, IS, ErrorS, SumError, MeanError] = FuncDiffSelectJacobian(SelectMap, SelectN, Pose, SelectScanXY, SelectScanOdd, IdSelectVar, ValParam);
            auto [DeltaP,DeltaD,SumDelta,SecondMeanDelta,SumDeltaP,SecondMeanDeltaP] = FuncSelectMapDelta(SelectMap, JP, JD, ErrorS, IS, WeightHHSelect, IdSelectVar, ValParam);
            MeanDelta = SecondMeanDelta;
            MeanDeltaP = SecondMeanDeltaP;
            std::cout<<"Mean Delta is "<< SecondMeanDelta << std::endl;
            std::cout<<"Mean Delta of Pose is "<< SecondMeanDeltaP << std::endl;
            FuncSelectMapUpdate(SelectMap, Pose, DeltaP, DeltaD, IdSelectVar);
            if (ValParam.ModeShowMap){
                FuncShowMap(SelectMap);
            }
            FuncUpdateSelectN(SelectN, Pose, SelectScanXY, ValParam);
            FuncEval(Pose, PoseGT, ValParam);
            FuncSmoothSelectN2(SelectN, WeightHHSelect, IdSelectVar, ValParam);
        }
        Iter++;
    }
}




void FuncLeastSquaresOdom(Eigen::MatrixXd& Map, Eigen::MatrixXd& N, Eigen::MatrixXd& Pose, const Eigen::MatrixXd& Odom, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd,
                          const std::vector<Eigen::ArrayXd>& LowScanXY, const std::vector<Eigen::ArrayXd>& LowScanOdd, const Eigen::SparseMatrix<double>& HH, ParamStruct& ValParam)
{
    int Iter = 1;
    double MeanDelta = 100000;
    double MeanDeltaP = 100;
    Eigen::MatrixXd SelectMap(ValParam.Sizei * ValParam.DownRate, ValParam.Sizej * ValParam.DownRate), SelectN(ValParam.Sizei * ValParam.DownRate, ValParam.Sizej * ValParam.DownRate);
    std::vector<Eigen::ArrayXd> SelectScanXY(ScanXY.size()), SelectScanOdd(ScanXY.size());
    Eigen::MatrixXi IdSelect, IdSelectVar;
    Eigen::SparseMatrix<double> HHSelect, WeightHHSelect;
    Eigen::SparseMatrix<double> WeightHH = HH * ValParam.MapSmoothingWeightFirst;
    while (MeanDeltaP > ValParam.MinDeltaPose && MeanDelta > ValParam.MinDelta && Iter <= ValParam.MaxIter ){
        std::cout<<"Iter"<<"  "<<Iter<<std::endl;
        if(Iter <= ValParam.DownTime)
        {
            auto [JP,JD,JO,IS,IO,ErrorS,ErrorO,SumError,MeanError] = FuncDiffJacobian(Map, N, Pose, Odom, LowScanXY, LowScanOdd, ValParam);
            auto [DeltaP,DeltaD,SumDelta,MeanDelta,SumDeltaP,MeanDeltaP] = FuncDelta(Map, Odom, JP, JD, JO, ErrorS, ErrorO, IS, IO, WeightHH, ValParam);
            std::cout<<"Mean Delta is "<< MeanDelta<< std::endl;
            std::cout<<"Mean Delta of Pose is "<< MeanDeltaP<< std::endl;
            FuncUpdate(Map, Pose, DeltaP, DeltaD);
            if (ValParam.ModeShowMap){
                FuncShowMap(Map);
            }
            FuncUpdateMapN(N, Pose, LowScanXY, ValParam);
            if (MeanDelta < ValParam.MinMeanDeltaFirst){
                ValParam.DownTime = Iter;
                ValParam.MaxIter = ValParam.DownTime + 3;
            }
            FuncSmoothN2(N, HH, ValParam);
        }
        else if ((Iter >= ValParam.DownTime + 1) && ValParam.ModeMulti)
        {
            if (Iter == ValParam.DownTime + 1) {
                std::cout<<"Selection Stage"<< std::endl;
                ValParam.Sizei = ValParam.Sizei * ValParam.DownRate;
                ValParam.Sizej = ValParam.Sizej * ValParam.DownRate;
                ValParam.Scale = ValParam.Scale / ValParam.DownRate;
                auto [HighMap, HighN] = FuncInitialiseGridMap(Pose,ScanXY,ScanOdd,ValParam);
                FuncCalBound(HighMap, IdSelect, IdSelectVar, ValParam);
                FuncSelectScan(Map, Pose, ScanXY, ScanOdd, SelectScanXY, SelectScanOdd, IdSelect, ValParam);
                FuncInitialSelectMap(SelectMap, SelectN, Pose, SelectScanXY, SelectScanOdd, ValParam);
                FuncSelectMapConst(SelectMap, IdSelectVar, HHSelect, ValParam);
                WeightHHSelect = HHSelect * ValParam.MapSmoothingWeightSecond;
            }
            FuncSmoothSelectN2(SelectN, WeightHHSelect, IdSelectVar, ValParam);
            auto [JP, JD, JO, IS, IO, ErrorS, ErrorO, SumError, MeanError] = FuncDiffSelectJacobian(SelectMap, SelectN, Pose, Odom, SelectScanXY, SelectScanOdd, IdSelectVar, ValParam);
            auto [DeltaP,DeltaD,SumDelta,SecondMeanDelta,SumDeltaP,SecondMeanDeltaP] = FuncSelectMapDelta(SelectMap, Odom, JP, JD, JO, ErrorS, ErrorO, IS, IO, WeightHHSelect, IdSelectVar, ValParam);
            MeanDelta = SecondMeanDelta;
            MeanDeltaP = SecondMeanDeltaP;
            FuncSelectMapUpdate(SelectMap, Pose, DeltaP, DeltaD, IdSelectVar);
            if (ValParam.ModeShowMap){
                FuncShowMap(SelectMap);
            }
            FuncUpdateSelectN(SelectN, Pose, SelectScanXY, ValParam);
            FuncSmoothSelectN2(SelectN, WeightHHSelect, IdSelectVar, ValParam);
        }
        Iter++;
    }
}



// No odometry inputs

void FuncLeastSquares(Eigen::MatrixXd& Map, Eigen::MatrixXd& N, Eigen::MatrixXd& Pose, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd,
                      const std::vector<Eigen::ArrayXd>& LowScanXY, const std::vector<Eigen::ArrayXd>& LowScanOdd, const Eigen::SparseMatrix<double>& HH, ParamStruct& ValParam)
{
    int Iter = 1;
    double MeanDelta = 100000;
    double MeanDeltaP = 100;
    Eigen::MatrixXd SelectMap(ValParam.Sizei * ValParam.DownRate, ValParam.Sizej * ValParam.DownRate), SelectN(ValParam.Sizei * ValParam.DownRate, ValParam.Sizej * ValParam.DownRate);
    std::vector<Eigen::ArrayXd> SelectScanXY(ScanXY.size()), SelectScanOdd(ScanXY.size());
    Eigen::MatrixXi IdSelect, IdSelectVar;
    Eigen::SparseMatrix<double> HHSelect, WeightHHSelect;
    Eigen::SparseMatrix<double> WeightHH = HH * ValParam.MapSmoothingWeightFirst;
    while (MeanDeltaP > ValParam.MinDeltaPose && MeanDelta > ValParam.MinDelta && Iter <= ValParam.MaxIter ){
        std::cout<<"Iter"<<"  "<<Iter<<std::endl;
        if(Iter <= ValParam.DownTime)
        {
            auto [JP,JD,IS,ErrorS,SumError,MeanError] = FuncDiffJacobian(Map, N, Pose, LowScanXY, LowScanOdd, ValParam);
            auto [DeltaP,DeltaD,SumDelta,FirstMeanDelta,SumDeltaP,FirstMeanDeltaP] = FuncDelta(Map, JP, JD, ErrorS, IS, WeightHH, ValParam);
            std::cout<<"Mean Delta is "<< FirstMeanDelta<< std::endl;
            std::cout<<"Mean Delta of Pose is "<< FirstMeanDeltaP<< std::endl;
            FuncUpdate(Map, Pose, DeltaP, DeltaD);
            if (ValParam.ModeShowMap){
                FuncShowMap(Map);
            }
            FuncUpdateMapN(N, Pose, LowScanXY, ValParam);
            if (FirstMeanDelta < ValParam.MinMeanDeltaFirst || FirstMeanDeltaP < ValParam.MinMeanDeltaPoseFirst){
                ValParam.DownTime = Iter;
                ValParam.MaxIter = ValParam.DownTime + 3;
            }
            FuncSmoothN2(N, HH, ValParam);
        }
        else if ((Iter >= ValParam.DownTime + 1) && ValParam.ModeMulti)
        {
            if (Iter == ValParam.DownTime + 1) {
                std::cout<<"Selection Stage"<< std::endl;
                ValParam.Sizei = ValParam.Sizei * ValParam.DownRate;
                ValParam.Sizej = ValParam.Sizej * ValParam.DownRate;
                ValParam.Scale = ValParam.Scale / ValParam.DownRate;
                auto [HighMap, HighN] = FuncInitialiseGridMap(Pose,ScanXY,ScanOdd,ValParam);
                FuncCalBound(HighMap, IdSelect, IdSelectVar, ValParam);
                FuncSelectScan(Map, Pose, ScanXY, ScanOdd, SelectScanXY, SelectScanOdd, IdSelect, ValParam);
                FuncInitialSelectMap(SelectMap, SelectN, Pose, SelectScanXY, SelectScanOdd, ValParam);
                FuncSelectMapConst(SelectMap, IdSelectVar, HHSelect, ValParam);
                WeightHHSelect = HHSelect * ValParam.MapSmoothingWeightSecond;
            }
            FuncSmoothSelectN2(SelectN, WeightHHSelect, IdSelectVar, ValParam);
            auto [JP, JD, IS, ErrorS, SumError, MeanError] = FuncDiffSelectJacobian(SelectMap, SelectN, Pose, SelectScanXY, SelectScanOdd, IdSelectVar, ValParam);
            auto [DeltaP,DeltaD,SumDelta,SecondMeanDelta,SumDeltaP,SecondMeanDeltaP] = FuncSelectMapDelta(SelectMap, JP, JD, ErrorS, IS, WeightHHSelect, IdSelectVar, ValParam);
            MeanDelta = SecondMeanDelta;
            MeanDeltaP = SecondMeanDeltaP;
            std::cout<<"Mean Delta is "<< SecondMeanDelta << std::endl;
            std::cout<<"Mean Delta of Pose is "<< SecondMeanDeltaP << std::endl;
            FuncSelectMapUpdate(SelectMap, Pose, DeltaP, DeltaD, IdSelectVar);
            if (ValParam.ModeShowMap){
                FuncShowMap(SelectMap);
            }
            FuncUpdateSelectN(SelectN, Pose, SelectScanXY, ValParam);
            FuncSmoothSelectN2(SelectN, WeightHHSelect, IdSelectVar, ValParam);
        }
        Iter++;
    }
}
