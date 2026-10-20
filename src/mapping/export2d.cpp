#include "export2d.hpp"
#include "cli.h"
#include <algorithm>
namespace occupancy {
void saveMap2D(const fs::path& dir,const Eigen::MatrixXd& pp,const Eigen::MatrixXd& field,const Eigen::MatrixXd& normalization,const Eigen::MatrixXd* integration_counts,const Eigen::MatrixXd& times,const ParamStruct& p) {
    fs::create_directories(dir);
    auto posefile=output(dir/"poses.txt"); posefile<<pp<<'\n';
    if(times.size()) {auto tum=output(dir/"trajectory.tum"); for(int i=0;i<pp.rows();++i) tum<<times(i,0)<<' '<<pp(i,0)<<' '<<pp(i,1)<<" 0 0 0 "<<sin(pp(i,2)/2)<<' '<<cos(pp(i,2)/2)<<'\n';}
    auto values=output(dir/"occupancy.txt"); values<<field<<'\n';
    auto counts=output(dir/"normalization.txt"); counts<<normalization<<'\n';
    if(integration_counts) {auto support=output(dir/"integration_counts.txt"); support<<*integration_counts<<'\n';}
    auto image=output(dir/"map.pgm"); image<<"P5\n"<<field.cols()<<' '<<field.rows()<<"\n255\n";
    for(int y=field.rows()-1;y>=0;--y) for(int x=0;x<field.cols();++x) {
      double mean=field(y,x)/std::max(normalization(y,x),1e-9);
      unsigned char value=std::lround(255/(1+std::exp(std::clamp(mean,-30.,30.))));
      image.write(reinterpret_cast<const char*>(&value),1);
    }
    auto meta=output(dir/"map.yaml"); meta<<"image: map.pgm\nresolution: "<<p.Scale<<"\norigin: ["<<p.OriginX<<", "<<p.OriginY<<", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.35\n";
}
}
