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
        Eigen::Index count=0;
        for(int j=0;j<ValParam.NumBeam;++j) if(std::isfinite(Range(i,j))&&Range(i,j)>ValParam.MinRange&&Range(i,j)<ValParam.MaxRange) count+=std::floor(Range(i,j)/ValParam.Scale);
        Eigen::ArrayXd ScanXYj(2*count),ScanOddj(count);Eigen::Index offset=0;
        for (int j = 0; j < ValParam.NumBeam; ++j) {
            if (Range(i,j)>=ValParam.MaxRange || Range(i,j)<=ValParam.MinRange || std::isnan(Range(i,j)))
            {
                Range(i,j) = 0.0;
            }

            if (Range(i,j)!=0.0)
            {
                int NumP = floor((Range(i, j) / ValParam.Scale));
                Eigen::ArrayXd ScanXYij(2 * NumP);
                Eigen::ArrayXd ScanOddij(NumP);
                for (int k = 0; k < NumP; ++k) {
                    double HitX = Range(i, j) * cos(Bearing(j));
                    double HitY = Range(i, j) * sin(Bearing(j));
                    if (k == 0) {
                        ScanXYij(2 * k) = HitX;
                        ScanXYij(2 * k + 1) = HitY;
                        ScanOddij(k) = ValParam.ValOccupied;
                    } else {
                        double Xk = (HitX / NumP) * (NumP - k);
                        double Yk = (HitY / NumP) * (NumP - k);
                        ScanXYij(2 * k) = Xk;
                        ScanXYij(2 * k + 1) = Yk;
                        ScanOddij(k) = ValParam.ValFree;
                    }
                }
                ScanXYj.segment(2*offset,ScanXYij.size())=ScanXYij;
                ScanOddj.segment(offset,ScanOddij.size())=ScanOddij;
                offset+=NumP;
            }
        }
        ScanXY[i] = ScanXYj;
        ScanOdd[i] = ScanOddj;
    }
}




