#pragma once
#include "MyStruct.h"
#include <filesystem>
namespace occupancy {
void saveMap2D(const std::filesystem::path& dir,const Eigen::MatrixXd& poses,
 const Eigen::MatrixXd& field,const Eigen::MatrixXd& normalization,
 const Eigen::MatrixXd* integration_counts,const Eigen::MatrixXd& times,const ParamStruct& parameters);
}
