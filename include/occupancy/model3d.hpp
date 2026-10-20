#pragma once
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <filesystem>
#include <vector>
#include <array>
#include <unordered_map>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <iostream>
namespace occupancy::three_d {
namespace fs=std::filesystem;
using V3=Eigen::Vector3d;
using M3=Eigen::Matrix3d;
using V6=Eigen::Matrix<double,6,1>;
using Pose=Eigen::Isometry3d;
using Sparse=Eigen::SparseMatrix<double>;
using Triplet=Eigen::Triplet<double>;
struct Key {
  int x,y,z;
  bool operator==(const Key& b) const {return x==b.x && y==b.y && z==b.z;}
};
struct Hash {size_t operator()(const Key& k) const {return size_t(uint32_t(k.x))*73856093u ^ size_t(uint32_t(k.y))*19349663u ^ size_t(uint32_t(k.z))*83492791u;}};
struct Observation {int frame; V3 local; double target,weight;};
struct Frame {double time; fs::path path; Pose pose; std::vector<V3> points;};
inline M3 hat(const V3& v) {M3 m; m<<0,-v.z(),v.y(), v.z(),0,-v.x(), -v.y(),v.x(),0; return m;}
inline Pose retract(const Pose& p,const V6& d) {
  Pose q=p; q.translation()+=p.linear()*d.head<3>();
  double theta=d.tail<3>().norm(); if(theta>1e-15) q.linear()=p.linear()*Eigen::AngleAxisd(theta,d.tail<3>()/theta).toRotationMatrix();
  return q;
}
inline V6 odomError(const Pose& a,const Pose& b,const Pose& measured,double wt,double wr) {
  const Pose error=measured.inverse()*(a.inverse()*b);
  Eigen::AngleAxisd aa(error.linear()); V6 r;
  r.head<3>()=wt*error.translation(); r.tail<3>()=wr*aa.angle()*aa.axis(); return r;
}
inline Pose fromTum(const Eigen::MatrixXd& m,int i) {
  Eigen::Quaterniond q(m(i,7),m(i,4),m(i,5),m(i,6));
  if(q.norm()<1e-8 || std::abs(q.norm()-1)>0.05) throw std::runtime_error("Invalid TUM quaternion at row "+std::to_string(i));
  Pose p=Pose::Identity(); p.translation()=m.row(i).segment<3>(1); p.linear()=q.normalized().toRotationMatrix(); return p;
}
struct Grid {
  double h; std::unordered_map<Key,int,Hash> index; std::vector<Key> keys;
  Eigen::VectorXd value; std::vector<std::pair<int,int>> edges;
  Key base(const V3& p) const {
    V3 a=(p/h).array().floor();
    if(!a.allFinite() || a.cwiseAbs().maxCoeff()>1e8) throw std::runtime_error("Point outside supported grid coordinates");
    return {int(a.x()),int(a.y()),int(a.z())};
  }
  void insert(Key k) {auto [it,added]=index.emplace(k,keys.size()); if(added) keys.push_back(k);}
  bool interpolate(const V3& p,std::array<int,8>& ids,std::array<double,8>& weights,double& val,V3& grad,const Eigen::VectorXd& data) const {
    Key b=base(p); V3 f=p/h-V3(b.x,b.y,b.z); val=0; grad.setZero(); int i=0;
    for(int z=0;z<2;++z) for(int y=0;y<2;++y) for(int x=0;x<2;++x,++i) {
      auto it=index.find({b.x+x,b.y+y,b.z+z}); if(it==index.end()) return false;
      ids[i]=it->second; V3 w(x?f.x():1-f.x(),y?f.y():1-f.y(),z?f.z():1-f.z());
      weights[i]=w.prod(); double v=data[ids[i]]; val+=weights[i]*v;
      grad+=v/h*V3((x?1:-1)*w.y()*w.z(),(y?1:-1)*w.x()*w.z(),(z?1:-1)*w.x()*w.y());
    }
    return true;
  }
  void initialize(const std::vector<Observation>& obs,const std::vector<Pose>& poses,int max_vertices) {
    for(const auto& o:obs) {
      Key b=base(poses[o.frame]*o.local);
      for(int z=-1;z<=2;++z) for(int y=-1;y<=2;++y) for(int x=-1;x<=2;++x) insert({b.x+x,b.y+y,b.z+z});
      if(int(keys.size())>max_vertices) throw std::runtime_error("Sparse map exceeds --max-vertices; increase voxel/point stride or shorten window");
    }
    auto accumulated=reintegrate(obs,poses); value=std::move(accumulated.first);
    for(int i=0;i<int(keys.size());++i) {auto k=keys[i]; for(Key n:{Key{k.x+1,k.y,k.z},Key{k.x,k.y+1,k.z},Key{k.x,k.y,k.z+1}}) {auto it=index.find(n); if(it!=index.end()) edges.emplace_back(i,it->second);}}
  }
  std::pair<Eigen::VectorXd,Eigen::VectorXd> reintegrate(const std::vector<Observation>& obs,const std::vector<Pose>& poses) const {
    Eigen::VectorXd sums=Eigen::VectorXd::Zero(keys.size()),count=sums;
    for(const auto& o:obs) {
      std::array<int,8> ids; std::array<double,8> w; double val; V3 g;
      if(!interpolate(poses[o.frame]*o.local,ids,w,val,g,sums)) throw std::runtime_error("Replay observation outside shared grid");
      for(int j=0;j<8;++j) {sums[ids[j]]+=w[j]*o.weight*o.target; count[ids[j]]+=w[j]*o.weight;}
    }
    for(int i=0;i<sums.size();++i) if(count[i]>0) sums[i]/=count[i];
    return {std::move(sums),std::move(count)};
  }

};
struct Problem {
  Grid grid; std::vector<Observation> obs; std::vector<Pose> poses,relative;
  double smooth=0.01,prior=1e-6,trans_weight=5,rot_weight=10;
  int poseDim() const {return 6*(poses.size()-1);}
  // All residuals are kept during trial evaluation. Leaving support rejects a step;
  // residual dropping must never manufacture an apparent objective improvement.
  double evaluate(const std::vector<Pose>& pp,const Eigen::VectorXd& mm,Sparse* jac=nullptr,Eigen::VectorXd* residual=nullptr) const {
    int pd=poseDim(), rows=obs.size()+6*relative.size()+grid.edges.size()+mm.size();
    std::vector<Triplet> entries; Eigen::VectorXd r(rows); int row=0;
    if(jac) entries.reserve(obs.size()*14+relative.size()*72+grid.edges.size()*2+mm.size());
    for(const auto& o:obs) {
      std::array<int,8> ids; std::array<double,8>w; double v; V3 grad;
      if(!grid.interpolate(pp[o.frame]*o.local,ids,w,v,grad,mm)) return INFINITY;
      double s=std::sqrt(o.weight); r[row]=s*(v-o.target);
      if(jac) {
        for(int j=0;j<8;++j) entries.emplace_back(row,pd+ids[j],s*w[j]);
        if(o.frame) {
          Eigen::Matrix<double,3,6> d; d.leftCols<3>()=pp[o.frame].linear(); d.rightCols<3>()=-pp[o.frame].linear()*hat(o.local);
          Eigen::Matrix<double,1,6> jp=s*grad.transpose()*d;
          for(int j=0;j<6;++j) entries.emplace_back(row,6*(o.frame-1)+j,jp[j]);
        }
      }
      ++row;
    }
    for(int i=0;i<int(relative.size());++i) {
      V6 e=odomError(pp[i],pp[i+1],relative[i],trans_weight,rot_weight); r.segment<6>(row)=e;
      if(jac) for(int f:{i,i+1}) if(f) for(int j=0;j<6;++j) {
        V6 delta=V6::Zero(); delta[j]=1e-6; Pose plus=retract(pp[f],delta),minus=retract(pp[f],-delta);
        V6 a=f==i?odomError(plus,pp[i+1],relative[i],trans_weight,rot_weight):odomError(pp[i],plus,relative[i],trans_weight,rot_weight);
        V6 b=f==i?odomError(minus,pp[i+1],relative[i],trans_weight,rot_weight):odomError(pp[i],minus,relative[i],trans_weight,rot_weight);
        V6 deriv=(a-b)/2e-6; for(int k=0;k<6;++k) entries.emplace_back(row+k,6*(f-1)+j,deriv[k]);
      }
      row+=6;
    }
    double ws=std::sqrt(smooth),wp=std::sqrt(prior);
    for(auto [i,j]:grid.edges) {r[row]=ws*(mm[i]-mm[j]); if(jac) {entries.emplace_back(row,pd+i,ws); entries.emplace_back(row,pd+j,-ws);} ++row;}
    for(int i=0;i<mm.size();++i,++row) {r[row]=wp*mm[i]; if(jac) entries.emplace_back(row,pd+i,wp);}
    double cost=r.squaredNorm();
    if(jac) {jac->resize(rows,pd+mm.size()); jac->setFromTriplets(entries.begin(),entries.end());}
    if(residual) *residual=std::move(r);
    return cost;
  }
  void checkJacobian() const {
    Sparse j; Eigen::VectorXd r; evaluate(poses,grid.value,&j,&r);
    for(int col=0;col<std::min(poseDim(),12);++col) {
      auto plus=poses,minus=poses; V6 d=V6::Zero(); d[col%6]=1e-7;
      plus[col/6+1]=retract(plus[col/6+1],d); minus[col/6+1]=retract(minus[col/6+1],-d);
      Eigen::VectorXd rp,rm; evaluate(plus,grid.value,nullptr,&rp); evaluate(minus,grid.value,nullptr,&rm);
      Eigen::VectorXd analytic=j.col(col),numeric=(rp-rm)/2e-7;
      // At grid faces trilinear derivatives have distinct one-sided limits. Check
      // interior rows and the odometry block; skip exact face observations only.
      for(int k=0;k<int(obs.size());++k) {
        V3 u=(poses[obs[k].frame]*obs[k].local)/grid.h;
        if((u-u.array().round().matrix()).cwiseAbs().minCoeff()<1e-5) {analytic[k]=numeric[k]=0;}
      }
      if((analytic-numeric).norm()>2e-4*(1+numeric.norm())) throw std::runtime_error("Pose Jacobian finite-difference check failed");
    }
    for(int col=poseDim();col<std::min<int>(j.cols(),poseDim()+16);++col) {
      auto plus=grid.value,minus=grid.value; plus[col-poseDim()]+=1e-7; minus[col-poseDim()]-=1e-7;
      Eigen::VectorXd rp,rm; evaluate(poses,plus,nullptr,&rp); evaluate(poses,minus,nullptr,&rm);
      Eigen::VectorXd analytic=j.col(col),numeric=(rp-rm)/2e-7;
      if((analytic-numeric).norm()>1e-5*(1+numeric.norm())) throw std::runtime_error("Map Jacobian finite-difference check failed");
    }
  }
};

}
