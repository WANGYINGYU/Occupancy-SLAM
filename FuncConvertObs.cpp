//
// Created by Yingyu Wang on 22/5/2023.
//
#include "MyStruct.h"
#include "SubFuncs.h"

void FuncConvertObs(Eigen::MatrixXd& Range, std::vector<Eigen::ArrayXd>& ScanXY, std::vector<Eigen::ArrayXd>& ScanOdd, const ParamStruct& ValParam){
    Eigen::ArrayXd Bearing = Eigen::ArrayXd::LinSpaced(ValParam.NumBeam, ValParam.MinAngle, ValParam.MaxAngle);
    int NumPose = Range.rows();
    #pragma omp parallel for
    for (int i = 0; i < NumPose; ++i) {
        Eigen::ArrayXd ScanXYj;
        Eigen::ArrayXd ScanOddj;
        for (int j = 0; j < ValParam.NumBeam; ++j) {
            if (Range(i,j)>=ValParam.MaxRange | Range(i,j)<=ValParam.MinRange){Range(i,j)=0.0;}
            int NumP = floor((Range(i,j) / ValParam.Scale));
            Eigen::ArrayXd ScanXYij(2*NumP);
            Eigen::ArrayXd ScanOddij(NumP);
            for (int k = 0; k < NumP; ++k) {
                double HitX = Range(i,j) * cos(Bearing(j));
                double HitY = Range(i,j) * sin(Bearing(j));
                if (k==0){
                    ScanXYij(2*k) = HitX;
                    ScanXYij(2*k+1) = HitY;
                    ScanOddij(k) = ValParam.ValOccupied;
                }
                else{
                    double Xk = (HitX/NumP)*(NumP-k);
                    double Yk = (HitY/NumP)*(NumP-k);
                    ScanXYij(2*k) = Xk;
                    ScanXYij(2*k+1) = Yk;
                    ScanOddij(k) = ValParam.ValFree;
                }
            }
            ScanXYj.conservativeResize(ScanXYj.size() + ScanXYij.size());
            ScanXYj.tail(ScanXYij.size()) = ScanXYij;

            ScanOddj.conservativeResize(ScanOddj.size() + ScanOddij.size());
            ScanOddj.tail(ScanOddij.size()) = ScanOddij;
        }
        ScanXY[i] = ScanXYj;
        ScanOdd[i] = ScanOddj;
    }
}