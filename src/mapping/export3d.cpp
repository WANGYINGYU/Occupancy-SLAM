#include "../io/io3d.hpp"
#include "cli.h"
#include <pcl/io/pcd_io.h>
namespace occupancy::three_d {
void saveMap3D(const fs::path& dir,const std::vector<Pose>& pp,const Eigen::VectorXd& field,const Eigen::VectorXd* weights,const std::vector<Frame>& frames,const Grid& grid,const Pose& world) {
    fs::create_directories(dir);
    auto trajectory=output(dir/"trajectory.tum"),selected=output(dir/"selected_scans.txt");
    pcl::PointCloud<pcl::PointXYZ> merged;
    for(int i=0;i<int(frames.size());++i) {
      Pose p=world*pp[i]; Eigen::Quaterniond q(p.linear());
      trajectory<<frames[i].time<<' '<<p.translation().transpose()<<' '<<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<q.w()<<'\n';
      selected<<frames[i].time<<' '<<fs::absolute(frames[i].path).string()<<'\n';
      for(const V3& v:frames[i].points) {V3 w=p*v; merged.emplace_back(w.x(),w.y(),w.z());}
    }
    if(pcl::io::savePCDFileBinary((dir/"optimized_points.pcd").string(),merged)<0) throw std::runtime_error("Cannot save merged PCD");
    // Retain double-precision actual field variables as well as probability-colored PCD.
    auto variables=output(dir/"map_variables.txt"); variables<<"# key_x key_y key_z log_odds\n";
    auto grid_pose=output(dir/"world_from_grid.txt"); grid_pose<<world.matrix()<<'\n';
    auto meta=output(dir/"grid.json"); meta<<"{\"voxel\":"<<grid.h<<",\"vertices\":"<<grid.keys.size()<<",\"world_from_grid\":\"world_from_grid.txt\",\"probability\":\"sigmoid(log_odds)\"}\n";
    if(weights) {auto counts=output(dir/"integration_weights.txt"); counts<<*weights<<'\n';}
    pcl::PointCloud<pcl::PointXYZI> vertices;
    for(int i=0;i<field.size();++i) {
      auto k=grid.keys[i]; V3 p=world*(grid.h*V3(k.x,k.y,k.z)); pcl::PointXYZI v;
      variables<<k.x<<' '<<k.y<<' '<<k.z<<' '<<field[i]<<'\n';
      v.x=p.x();v.y=p.y();v.z=p.z();v.intensity=1/(1+std::exp(-std::clamp(field[i],-30.,30.))); vertices.push_back(v);
    }
    if(pcl::io::savePCDFileBinary((dir/"occupancy_vertices.pcd").string(),vertices)<0) throw std::runtime_error("Cannot save occupancy PCD");
}
}
