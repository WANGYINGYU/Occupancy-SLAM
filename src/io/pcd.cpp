#include "../io/io3d.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
namespace occupancy::three_d {
void loadFramePoints(Frame& f,double voxel,double min_range,double max_range,int stride) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>),clean(new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile(f.path.string(),*raw)<0) throw std::runtime_error("Cannot read PCD: "+f.path.string());
  for(const auto& p:*raw) {V3 v(p.x,p.y,p.z);if(v.allFinite()&&v.norm()>=min_range&&v.norm()<=max_range)clean->push_back(p);}
  if(voxel>0&&!clean->empty()) {pcl::VoxelGrid<pcl::PointXYZ> filter;filter.setInputCloud(clean);filter.setLeafSize(voxel,voxel,voxel);auto down=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);filter.filter(*down);clean=down;}
  f.points.reserve((clean->size()+stride-1)/stride);
  for(size_t k=0;k<clean->size();k+=stride){const auto& p=(*clean)[k];f.points.emplace_back(p.x,p.y,p.z);}
  if(f.points.empty())throw std::runtime_error("PCD has no valid in-range points: "+f.path.string());
}
}
