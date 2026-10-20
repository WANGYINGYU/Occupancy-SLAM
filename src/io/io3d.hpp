#pragma once
#include "occupancy/backend3d.hpp"
namespace occupancy::three_d {
void loadFramePoints(Frame& frame,double voxel,double min_range,double max_range,int stride);
void saveMap3D(const std::filesystem::path& dir,const std::vector<Pose>& poses,const Eigen::VectorXd& field,
 const Eigen::VectorXd* weights,const std::vector<Frame>& frames,const Grid& grid,const Pose& world);
}
