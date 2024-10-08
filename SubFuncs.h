//
// Created by Yingyu Wang on 19/4/2023.
//

#ifndef FASTOCC_SUBFUNCS_H
#define FASTOCC_SUBFUNCS_H
#include <thread>
#include <random>
#include <iomanip>
#include <fstream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#include <Eigen/SparseCore>
#include <Eigen/OrderingMethods>
#include <Eigen/IterativeLinearSolvers>
#include <algorithm>
#include <utility>
#include <omp.h>
#include <igl/find.h>
#include <igl/sparse.h>
#include <igl/grad.h>
#include <igl/cat.h>
#include <igl/slice.h>
#include <igl/unique_rows.h>
#include <igl/ismember.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


void loadFileData(const std::vector<std::string>& filenames, std::vector<std::vector<double>>& fileData, const ParamStruct& ValParam);
std::vector<double> linspace(double start, double end, int num);
double FuncWrap(double alpha);
Eigen::VectorXd VectorToEigenVec (const std::vector<double>& data);
void FuncLowSampleScan(const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, std::vector<Eigen::ArrayXd>& LowScanXY, std::vector<Eigen::ArrayXd>& LowScanOdd, const ParamStruct& ValParam);
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> FuncInitialiseGridMap(const Eigen::MatrixXd& Pose, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, const ParamStruct& ValParam);
Eigen::SparseMatrix<double> FuncMapConst(const ParamStruct& ValParam);
void FuncSmoothN2(Eigen::MatrixXd& N, const Eigen::SparseMatrix<double>& HH, const ParamStruct& ValParam);
std::tuple<Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::VectorXd,Eigen::VectorXd,double,double> FuncDiffJacobian(const Eigen::MatrixXd& Map, const Eigen::MatrixXd& N, const Eigen::MatrixXd& Pose, const Eigen::MatrixXd& Odom, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, const ParamStruct& ValParam);
Eigen::ArrayXd FuncBilinearInterpolation(const Eigen::MatrixXd& Map, const Eigen::MatrixXd& ObjectXY);
template<typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> FuncGradient(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& M, const T& h);
Eigen::Matrix2d FuncdR2D(const double& phi);
std::tuple<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>> FuncGetI(const Eigen::MatrixXd& Odom, const Eigen::VectorXd& ErrorS);
std::tuple<Eigen::VectorXd, Eigen::VectorXd, double, double, double, double> FuncDelta(const Eigen::MatrixXd& Map, const Eigen::MatrixXd& Odom, const Eigen::SparseMatrix<double>& JP, Eigen::SparseMatrix<double>& JD, const Eigen::SparseMatrix<double>& JO, const Eigen::VectorXd& ErrorS, const Eigen::VectorXd& ErrorO, const Eigen::SparseMatrix<double>& IS, const Eigen::SparseMatrix<double>& IO, const Eigen::SparseMatrix<double>& WeightHH, const ParamStruct& ValParam);
void FuncUpdate(Eigen::MatrixXd& Map, Eigen::MatrixXd& Pose, const Eigen::VectorXd& DeltaP, const Eigen::VectorXd& DeltaD);
void FuncUpdateMapN(Eigen::MatrixXd& N, const Eigen::MatrixXd& Pose, const std::vector<Eigen::ArrayXd>& ScanXY, const ParamStruct& ValParam);
void FuncLeastSquaresOdom(Eigen::MatrixXd& Map, Eigen::MatrixXd& N, Eigen::MatrixXd& Pose, const Eigen::MatrixXd& PoseGT, const Eigen::MatrixXd& Odom, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, const std::vector<Eigen::ArrayXd>& LowScanXY, const std::vector<Eigen::ArrayXd>& LowScanOdd, const Eigen::SparseMatrix<double>& HH, ParamStruct& ValParam);
void FuncExpMap(const Eigen::MatrixXd& Map, Eigen::MatrixXd& ExpMap);
void FuncCalBound(const Eigen::MatrixXd& Map, Eigen::MatrixXi& IdSelect, Eigen::MatrixXi& IdSelectVar, const ParamStruct& ValParam);
void FuncSelectScan(const Eigen::MatrixXd& Map, const Eigen::MatrixXd& Pose, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, std::vector<Eigen::ArrayXd>& SelectScanXY, std::vector<Eigen::ArrayXd>& SelectScanOdd, const Eigen::MatrixXi& IdSelect, const ParamStruct& ValParam);
void FuncInitialSelectMap(Eigen::MatrixXd&Map, Eigen::MatrixXd& N, Eigen::MatrixXd& Pose, std::vector<Eigen::ArrayXd>& SelectScanXY, std::vector<Eigen::ArrayXd>& SelectScanOdd, ParamStruct& ValParam);
void FuncSelectMapConst(const Eigen::MatrixXd& SelectMap, const Eigen::MatrixXi& IdSelectVar, Eigen::SparseMatrix<double>& HH, const ParamStruct& ValParam);
void FuncSmoothSelectN2(Eigen::MatrixXd& SelectN, const Eigen::SparseMatrix<double>& WeightHHSelect, const Eigen::MatrixXi IdSelectVar, const ParamStruct& ValParam);
std::tuple<Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::VectorXd,Eigen::VectorXd,double,double> FuncDiffSelectJacobian(const Eigen::MatrixXd& SelectMap, const Eigen::MatrixXd& SelectN, const Eigen::MatrixXd& Pose, const Eigen::MatrixXd& Odom, const std::vector<Eigen::ArrayXd>& SelectScanXY, const std::vector<Eigen::ArrayXd>& SelectScanOdd, const Eigen::MatrixXi& IdSelectVar, const ParamStruct& ValParam);
std::tuple<Eigen::VectorXd, Eigen::VectorXd, double, double, double, double> FuncSelectMapDelta(const Eigen::MatrixXd& SelectMap, const Eigen::MatrixXd& Odom, const Eigen::SparseMatrix<double>& JP, const Eigen::SparseMatrix<double>& JD, const Eigen::SparseMatrix<double>& JO, const Eigen::VectorXd& ErrorS, const Eigen::VectorXd& ErrorO, const Eigen::SparseMatrix<double>& IS, const Eigen::SparseMatrix<double>& IO, const Eigen::SparseMatrix<double>& WeightHH, const Eigen::MatrixXi& IdSelectVar, const ParamStruct& ValParam);
void FuncSelectMapUpdate(Eigen::MatrixXd& SelectMap, Eigen::MatrixXd& Pose, const Eigen::VectorXd& DeltaP, const Eigen::VectorXd& DeltaD, const Eigen::MatrixXi& IdSelectVar);
void FuncUpdateSelectN(Eigen::MatrixXd& SelectN, const Eigen::MatrixXd& Pose, const std::vector<Eigen::ArrayXd>& SelectScanXY, const ParamStruct& ValParam);
void RemoveColumn(Eigen::MatrixXd& matrix, const Eigen::ArrayXi& B);
void RemoveArrayIndex(Eigen::ArrayXi& A, const Eigen::ArrayXi& B);
void RemoveArrayIndex(Eigen::ArrayXd& A, const Eigen::ArrayXi& B);
void FuncEval(const Eigen::MatrixXd& Pose, const Eigen::MatrixXd& PoseGT, const ParamStruct& ValParam);
void FuncNoisePose(Eigen::MatrixXd& Pose);
void loadTxTData(const std::string& filenames, std::vector<double>& fileData, const ParamStruct& ValParam);
void FuncBatchWrap(Eigen::ArrayXd& alpha);
void FuncLeastSquares(Eigen::MatrixXd& Map, Eigen::MatrixXd& N, Eigen::MatrixXd& Pose, const Eigen::MatrixXd& PoseGT, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd,
                      const std::vector<Eigen::ArrayXd>& LowScanXY, const std::vector<Eigen::ArrayXd>& LowScanOdd, const Eigen::SparseMatrix<double>& HH, ParamStruct& ValParam);
Eigen::SparseMatrix<double> FuncGetI(const Eigen::VectorXd& ErrorS);
std::tuple<Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::VectorXd,double,double> FuncDiffJacobian(const Eigen::MatrixXd& Map, const Eigen::MatrixXd& N, const Eigen::MatrixXd& Pose, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, const ParamStruct& ValParam);
std::tuple<Eigen::VectorXd, Eigen::VectorXd, double, double, double, double> FuncDelta(const Eigen::MatrixXd& Map, const Eigen::SparseMatrix<double>& JP, const Eigen::SparseMatrix<double>& JD, const Eigen::VectorXd& ErrorS, const Eigen::SparseMatrix<double>& IS, const Eigen::SparseMatrix<double>& WeightHH, const ParamStruct& ValParam);
std::tuple<Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::SparseMatrix<double>,Eigen::VectorXd,double,double> FuncDiffSelectJacobian(const Eigen::MatrixXd& SelectMap, const Eigen::MatrixXd& SelectN, const Eigen::MatrixXd& Pose, const std::vector<Eigen::ArrayXd>& SelectScanXY, const std::vector<Eigen::ArrayXd>& SelectScanOdd, const Eigen::MatrixXi& IdSelectVar, const ParamStruct& ValParam);
std::tuple<Eigen::VectorXd, Eigen::VectorXd, double, double, double, double> FuncSelectMapDelta(const Eigen::MatrixXd& SelectMap, const Eigen::SparseMatrix<double>& JP, const Eigen::SparseMatrix<double>& JD, const Eigen::VectorXd& ErrorS, const Eigen::SparseMatrix<double>& IS, const Eigen::SparseMatrix<double>& WeightHH, const Eigen::MatrixXi& IdSelectVar, const ParamStruct& ValParam);
void FuncConvertObs(Eigen::MatrixXd& Range, std::vector<Eigen::ArrayXd>& ScanXY, std::vector<Eigen::ArrayXd>& ScanOdd, const ParamStruct& ValParam);
void FuncLeastSquares(Eigen::MatrixXd& Map, Eigen::MatrixXd& N, Eigen::MatrixXd& Pose, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd,
                      const std::vector<Eigen::ArrayXd>& LowScanXY, const std::vector<Eigen::ArrayXd>& LowScanOdd, const Eigen::SparseMatrix<double>& HH, ParamStruct& ValParam);
void FuncLeastSquaresOdom(Eigen::MatrixXd& Map, Eigen::MatrixXd& N, Eigen::MatrixXd& Pose, const Eigen::MatrixXd& Odom, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd,
                          const std::vector<Eigen::ArrayXd>& LowScanXY, const std::vector<Eigen::ArrayXd>& LowScanOdd, const Eigen::SparseMatrix<double>& HH, ParamStruct& ValParam);
Eigen::Matrix2d FuncTheta2R(const double& phi);
void FuncKeyFrameSelectionOdom(const Eigen::MatrixXd& Pose, Eigen::MatrixXd& PoseTem, const Eigen::MatrixXd& PoseGT, Eigen::MatrixXd& PoseGTTem, const Eigen::MatrixXd& Odom, Eigen::MatrixXd& OdomTem, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, std::vector<Eigen::ArrayXd>& ScanXYTem, std::vector<Eigen::ArrayXd>& ScanOddTem, const ParamStruct& ValParam);
void FuncKeyFrameSelectionOdom(const Eigen::MatrixXd& Pose, Eigen::MatrixXd& PoseTem, const Eigen::MatrixXd& Odom, Eigen::MatrixXd& OdomTem, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, std::vector<Eigen::ArrayXd>& ScanXYTem, std::vector<Eigen::ArrayXd>& ScanOddTem, const ParamStruct& ValParam);
void FuncKeyFrameSelection(const Eigen::MatrixXd& Pose, Eigen::MatrixXd& PoseTem, const Eigen::MatrixXd& PoseGT, Eigen::MatrixXd& PoseGTTem, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, std::vector<Eigen::ArrayXd>& ScanXYTem, std::vector<Eigen::ArrayXd>& ScanOddTem, const ParamStruct& ValParam);
void FuncKeyFrameSelection(const Eigen::MatrixXd& Pose, Eigen::MatrixXd& PoseTem, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, std::vector<Eigen::ArrayXd>& ScanXYTem, std::vector<Eigen::ArrayXd>& ScanOddTem, const ParamStruct& ValParam);
void FuncShowMap(Eigen::MatrixXd& Map);
void FuncShowMapPress(Eigen::MatrixXd& Map);
Eigen::MatrixXd FuncInitialiseGridMapToShow(const Eigen::MatrixXd& Pose, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, ParamStruct& ValParam);
Eigen::MatrixXd FuncInitialiseGridMapToShowFinal(const Eigen::MatrixXd& Pose, const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, const ParamStruct& ValParam);

void FuncPosefromOdom(const Eigen::MatrixXd& Odom, Eigen::MatrixXd& Pose);
void RemoveArrayIndex(Eigen::ArrayXi& arr, const std::vector<int>& indicesToRemove);
void RemoveArrayIndex(Eigen::ArrayXd& arr, const std::vector<int>& indicesToRemove);
void SetParametersFromFile(const std::string& fileName, ParamStruct& params);

std::tuple<std::vector<Eigen::MatrixXd>, std::vector<std::vector<Eigen::ArrayXd>>, std::vector<std::vector<Eigen::ArrayXd>>> FuncSubmapsDevision(const Eigen::MatrixXd& Pose,const std::vector<Eigen::ArrayXd>& ScanXY,const std::vector<Eigen::ArrayXd>& ScanOdd,const ParamStruct& ValParam);
Eigen::MatrixXd FuncGetSubmapPose(const std::vector<Eigen::MatrixXd>& PoseSubmaps);
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::Vector2d> FuncInitialiseLocalMap(const Eigen::MatrixXd& Pose,const std::vector<Eigen::ArrayXd>& ScanXY, const std::vector<Eigen::ArrayXd>& ScanOdd, const ParamStruct& ValParam);
Eigen::MatrixXd TransformToLocalFrame(const Eigen::MatrixXd& Pose);
std::vector<SubMap> FuncBuildSubMaps(const std::vector<Eigen::MatrixXd>& PoseSubmaps,const std::vector<std::vector<Eigen::ArrayXd>>& ScanXYSubmaps,const std::vector<std::vector<Eigen::ArrayXd>>& ScanOddSubmaps,ParamStruct& ValParam);
#endif //FASTOCC_SUBFUNCS_H