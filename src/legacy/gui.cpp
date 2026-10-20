#include "SubFuncs.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
void FuncShowMap(Eigen::MatrixXd& Map){
    Eigen::MatrixXd ab = Map.array().exp().matrix();
    Eigen::MatrixXd ExpMap = (1 - ab.array() / (ab.array() + 1)).matrix();
    cv::Mat img = cv::Mat(ExpMap.cols(), ExpMap.rows(), CV_64F, ExpMap.data());
    cv::imshow("Map", img);
    cv::waitKey(1000);
    cv::destroyWindow("Map");
    cv::waitKey(1);
}


void FuncShowMapPress(Eigen::MatrixXd& Map){
    Eigen::MatrixXd ab = Map.array().exp().matrix();
    Eigen::MatrixXd ExpMap = (1 - ab.array() / (ab.array() + 1)).matrix();
    cv::Mat img = cv::Mat(ExpMap.cols(), ExpMap.rows(), CV_64F, ExpMap.data());
    cv::imshow("Map", img);
    cv::waitKey(0);
    cv::destroyWindow("Map");
    cv::waitKey(1);
}
