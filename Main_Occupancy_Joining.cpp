//
// Created by Yingyu Wang on 28/9/2024.
//
#include <iostream>
#include <string>
#include <vector>
#include "MyStruct.h"
#include "SubFuncs.h"

ParamStruct SetParam();

int main() {
    ParamStruct ValParam = SetParam();
    SetParametersFromFile("../config.txt", ValParam);
    std::cout << "Enter the names of the input files (range of scan (must), initialization poses (option, if not using odometry inputs as initialization), and odometry inputs (option, if exit), separated by spaces: ";
    std::string input;
    std::getline(std::cin, input);
    std::vector<std::string> filenames;
    std::string delimiter = " ";
    size_t pos = 0;
    while ((pos = input.find(delimiter)) != std::string::npos) {
        std::string filename = input.substr(0, pos);
        filenames.push_back(filename);
        input.erase(0, pos + delimiter.length());
    }
    filenames.push_back(input);


    if (ValParam.ModeOdom){
        if (ValParam.PosefromOdom){
            if (filenames.size() != 2) {
                std::cout << "The number of input files is not 2 (range and odometry inputs). Please check the input files." << std::endl;
                return 0;}
        } else {
            if (filenames.size() != 3) {
                std::cout << "The number of input files is not 3 (range, initialization poses, and odometry inputs). Please check the input files." << std::endl;
                return 0;}
        }
    } else {
        if (filenames.size() != 2) {
            std::cout << "The number of input files is not 2 (range and initialization poses). Please check the input files." << std::endl;
            return 0;}
    }

    std::vector<double> VecRange;
    loadTxTData(filenames[0],VecRange,ValParam);
    Eigen::VectorXd VecRangeT = VectorToEigenVec(VecRange);
    Eigen::MatrixXd Range = VecRangeT.reshaped(ValParam.NumBeam,VecRangeT.size()/ValParam.NumBeam).transpose();
    auto start = std::chrono::high_resolution_clock::now();  // Start
    std::vector<Eigen::ArrayXd> ScanXY(Range.rows());
    std::vector<Eigen::ArrayXd> ScanOdd(Range.rows());

    FuncConvertObs(Range, ScanXY, ScanOdd, ValParam);
    auto end = std::chrono::high_resolution_clock::now();  // End
    double elapsed_time_sec = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
    std::cout << "Preprocessing, Elapsed time: " << elapsed_time_sec << " seconds " << std::endl;
    // Load the data from the input files and store it in vectors
    std::vector<std::vector<double>> fileData;

    filenames.erase(filenames.begin(), filenames.begin()+1);
    loadFileData(filenames, fileData, ValParam);

    Eigen::MatrixXd MatrixOdom;
    Eigen::MatrixXd Pose;
    if (filenames.size()==1){
        if (ValParam.ModeOdom){
            Eigen::VectorXd VecOdom = VectorToEigenVec(fileData[0]);
            MatrixOdom = VecOdom.reshaped(3,VecOdom.size()/3).transpose();
            FuncPosefromOdom(MatrixOdom, Pose);
        } else{
            Eigen::VectorXd VecPose = VectorToEigenVec(fileData[0]);
            Pose = VecPose.reshaped(3,VecPose.size()/3).transpose();
        }
    } else if (filenames.size()==2){
        Eigen::VectorXd VecPose = VectorToEigenVec(fileData[0]);
        Pose = VecPose.reshaped(3,VecPose.size()/3).transpose();
        Eigen::VectorXd VecOdom = VectorToEigenVec(fileData[1]);
        MatrixOdom = VecOdom.reshaped(3,VecOdom.size()/3).transpose();
    } else{
        std::cout << "The number of input files is wrong. Please check the input files." << std::endl;
        return 0;
    }

    Eigen::MatrixXd PoseGT;
    if (ValParam.EvaluateGT){
        std::cout << "Enter the names of the PoseGT: ";
        std::string PoseGTInput;
        std::getline(std::cin, PoseGTInput);
        std::vector<double> PoseGTData;
        loadTxTData(PoseGTInput,PoseGTData,ValParam);
        Eigen::VectorXd VecPoseGT = VectorToEigenVec(PoseGTData);
        PoseGT = VecPoseGT.reshaped(3,VecPoseGT.size()/3).transpose();
        FuncEval(Pose, PoseGT, ValParam);
    }

    if (ValParam.ModeKeyFrame){
        int NumPose = Pose.rows();
        int NumSelect = floor((NumPose/ValParam.KeyframeRate)) + 1;
        std::vector<Eigen::ArrayXd> ScanXYTem(NumSelect);
        std::vector<Eigen::ArrayXd> ScanOddTem(NumSelect);
        Eigen::MatrixXd PoseTem(NumSelect,3), PoseGTTem(NumSelect,3);
        Eigen::MatrixXd OdomTem(NumSelect,3);
        if (ValParam.ModeOdom){
            if (ValParam.EvaluateGT){
                FuncKeyFrameSelectionOdom(Pose, PoseTem, PoseGT, PoseGTTem, MatrixOdom, OdomTem, ScanXY, ScanOdd, ScanXYTem, ScanOddTem, ValParam);
                PoseGT.resize(NumSelect, 3);
                PoseGT = PoseGTTem;
            }
            else{
                FuncKeyFrameSelectionOdom(Pose, PoseTem, MatrixOdom, OdomTem, ScanXY, ScanOdd, ScanXYTem, ScanOddTem, ValParam);
            }
            MatrixOdom.resize(NumSelect, 3);
            MatrixOdom = OdomTem;
        }
        else{
            if (ValParam.EvaluateGT){
                FuncKeyFrameSelection(Pose, PoseTem, PoseGT, PoseGTTem, ScanXY, ScanOdd, ScanXYTem, ScanOddTem, ValParam);
                PoseGT.resize(NumSelect, 3);
                PoseGT = PoseGTTem;
            }
            else{
                FuncKeyFrameSelection(Pose, PoseTem, ScanXY, ScanOdd, ScanXYTem, ScanOddTem, ValParam);
            }
        }
        ScanXY.resize(NumSelect);
        ScanOdd.resize(NumSelect);
        ScanXY = ScanXYTem;
        ScanOdd = ScanOddTem;
        Pose.resize(NumSelect, 3);
        Pose = PoseTem;
    }

    auto [PoseSubmaps, ScanXYSubmaps, ScanOddSubmaps] = FuncSubmapsDevision(Pose,ScanXY,ScanOdd,ValParam);

    std::vector<MapStruct> SubMaps =  FuncBuildSubMaps(PoseSubmaps,ScanXYSubmaps,ScanOddSubmaps,ValParam);
    MapStruct GlobalMap = FuncBuildGlobalMapfromLocalMaps(SubMaps,PoseSubmaps,ValParam);
    Eigen::MatrixXd StatePoses = FuncGetSubmapPose(PoseSubmaps);
    FuncSubmapJoiningNLLS(SubMaps,GlobalMap,StatePoses,ValParam);
    return 0;
}




