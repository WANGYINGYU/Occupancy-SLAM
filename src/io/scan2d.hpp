#pragma once
#include "MyStruct.h"
#include <filesystem>
#include <vector>
namespace occupancy {
void readSamples2D(const std::filesystem::path& path, int frames, int start_frame, uint64_t budget, std::vector<Eigen::ArrayXd>& samples, std::vector<Eigen::ArrayXd>& labels);
void readEndpoints2D(const std::filesystem::path& path, int frames, const ParamStruct& parameters,
                     uint64_t max_observations, std::vector<Eigen::ArrayXd>& samples,
                     std::vector<Eigen::ArrayXd>& labels);
}
