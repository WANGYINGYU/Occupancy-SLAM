#include "cli.h"
#include "apps.hpp"
#include "../io/io3d.hpp"
using namespace occupancy::three_d;
int occupancy::run3D(int argc,char**argv) try {
  if(argc==1 || (argc==2 && std::string(argv[1])=="--help")) {
    std::cout<<"occupancy_slam_3d --dataset NC20_SEQUENCE --output DIR [--poses odometry.txt] [--manifest scans.txt] [--start 0] [--stride 1] [--max-frames 0] [--voxel 0.5] [--iterations 10] [--point-voxel 0.3] [--point-stride 1] [--max-range 60] [--free-samples 4] [--odom-translation-weight 5] [--odom-rotation-weight 10] [--check-jacobians 1] [--linear-iterations 500] [--linear-tolerance 1e-6] [--max-observations 20000000]\n"
      <<"Alternatively: --poses trajectory.tum --manifest scans.txt (timestamp path; PCD in local sensor frame).\n"; return 0;
  }
  Args a(argc,argv,{"--dataset","--poses","--manifest","--output","--start","--stride","--max-frames","--voxel","--iterations","--point-voxel","--point-stride","--max-range","--min-range","--free-samples","--smooth","--odom-translation-weight","--odom-rotation-weight","--max-vertices","--max-time-diff","--check-jacobians","--linear-iterations","--linear-tolerance","--max-observations"});
  const auto out=newOutput(a); RunState state(out);
  fs::path dataset=a.get("--dataset",".");
  auto tum=matrix(a.get("--poses",(dataset/"odometry.txt").string()),8);
  for(int i=1;i<tum.rows();++i) if(tum(i,0)<=tum(i-1,0)) throw std::runtime_error("TUM timestamps must be strictly increasing");
  int start=a.integer("--start",0),stride=a.integer("--stride",1),maxframes=a.integer("--max-frames",0),pointstride=a.integer("--point-stride",1),free_samples=a.integer("--free-samples",4);
  double pv=a.num("--point-voxel",0.3),minrange=a.num("--min-range",0.3),maxrange=a.num("--max-range",60),gap=a.num("--max-time-diff",0.05);
  Problem problem; problem.grid.h=a.num("--voxel",0.5); problem.smooth=a.num("--smooth",0.01);
  problem.trans_weight=a.num("--odom-translation-weight",5); problem.rot_weight=a.num("--odom-rotation-weight",10);
  if(stride==0 || pointstride==0 || pv<0 || minrange<0 || maxrange<=minrange || gap<0 || problem.grid.h<=0 || problem.smooth<0 || problem.trans_weight<=0 || problem.rot_weight<=0 || free_samples>1000) throw std::runtime_error("Invalid 3D parameter");
  std::vector<Frame> all;
  if(a.values.count("--manifest")) {
    fs::path manifest=a.get("--manifest"); std::ifstream f(manifest); if(!f) throw std::runtime_error("Cannot read manifest");
    std::string line; int last=-1; double lasttime=-INFINITY;
    while(std::getline(f,line)) {
      if(line.empty()||line[0]=='#') continue;
      std::istringstream ss(line); double time; std::string path;
      if(!(ss>>time) || !std::isfinite(time)) throw std::runtime_error("Invalid manifest timestamp");
      std::getline(ss>>std::ws,path); if(path.empty() || time<=lasttime) throw std::runtime_error("Manifest needs ordered timestamp and path");
      lasttime=time; int best=0; double diff=INFINITY;
      int lo=0,hi=tum.rows();
      while(lo<hi){int mid=lo+(hi-lo)/2;if(tum(mid,0)<time)lo=mid+1;else hi=mid;}
      best=std::min<int>(lo,tum.rows()-1);
      if(best>0 && std::abs(tum(best-1,0)-time)<=std::abs(tum(best,0)-time))--best;
      diff=std::abs(tum(best,0)-time);
      if(diff>gap || best<=last) throw std::runtime_error("Missing or reused trajectory timestamp for scan: "+path); last=best;
      fs::path p(path); if(p.is_relative()) p=manifest.parent_path()/p;
      all.push_back({time,p,fromTum(tum,best),{}});
    }
  } else for(int i=0;i<tum.rows();++i) all.push_back({tum(i,0),dataset/"PCD"/("full"+std::to_string(i)+".pcd"),fromTum(tum,i),{}});
  std::vector<Frame> frames;
  for(int i=start;i<int(all.size()) && (!maxframes || int(frames.size())<maxframes);i+=stride) frames.push_back(all[i]);
  if(frames.size()<2) throw std::runtime_error("Need at least two selected PCD/pose pairs");
  const Pose world=frames.front().pose; // optimize near the first scan to retain numeric precision
  for(int i=0;i<int(frames.size());++i) {
    auto& f=frames[i];loadFramePoints(f,pv,minrange,maxrange,pointstride);
    size_t budget=a.integer("--max-observations",20000000);
    if(f.points.size()>budget/(1+free_samples) || problem.obs.size()>budget-f.points.size()*(1+free_samples)) throw std::runtime_error("Observation budget exceeded before sampling");
    for(const V3& v:f.points) {
      problem.obs.push_back({i,v,std::log(0.7/0.3),1.});
      // Fixed number of free-space observations per ray prevents long rays dominating.
      // Leave one voxel near the hit free of negative labels.
      double free_end=std::max(0.,v.norm()-problem.grid.h);
      for(int j=1;j<=free_samples && free_end>minrange;++j) problem.obs.push_back({i,v*(free_end/v.norm())*(double(j)/(free_samples+1)),std::log(0.4/0.6),1./std::max(1,free_samples)});
    }
    if(f.points.empty()) throw std::runtime_error("PCD has no valid in-range points: "+f.path.string());
    problem.poses.push_back(world.inverse()*f.pose);
    if(i) problem.relative.push_back(problem.poses[i-1].inverse()*problem.poses[i]);
    std::cout<<"frame="<<i<<" points="<<f.points.size()<<std::endl;
  }
  problem.grid.initialize(problem.obs,problem.poses,a.integer("--max-vertices",2000000));
  const auto initial_poses=problem.poses;
  const Eigen::VectorXd initial_values=problem.grid.value;
  if(a.integer("--check-jacobians",0)) {problem.checkJacobian(); std::cout<<"Jacobian checks passed\n";}
  auto trace=output(out/"iterations.csv");trace<<"iteration,cost_before,cost_after,lambda,step_norm,accepted,linear_status,linear_iterations,linear_relative_residual\n";
  auto solved=optimize3D(problem,a.integer("--iterations",10),a.integer("--linear-iterations",500),a.num("--linear-tolerance",1e-6),trace);
  // Use the same support and sensor samples for all three fields. No solver runs in replay.
  auto [reintegrated,reintegrated_weights]=problem.grid.reintegrate(problem.obs,problem.poses);
  auto initial_replay=problem.grid.reintegrate(problem.obs,initial_poses);
  auto save=[&](const fs::path& dir,const std::vector<Pose>& pp,const Eigen::VectorXd& field,const Eigen::VectorXd* weights) {
    saveMap3D(dir,pp,field,weights,frames,problem.grid,world);
  };
  save(out/"initial",initial_poses,initial_values,&initial_replay.second);
  save(out/"joint",problem.poses,problem.grid.value,nullptr);
  save(out/"reintegrated",problem.poses,reintegrated,&reintegrated_weights);
  for(const auto& entry:fs::directory_iterator(out/"joint")) fs::copy_file(entry.path(),out/entry.path().filename());
  auto exports=output(out/"map_outputs.json");
  exports<<"{\"schema_version\":2,\"mode\":\"3d\",\"initial\":\"initial\",\"joint\":\"joint\",\"reintegrated\":\"reintegrated\",\"same_grid\":true,\"fixed_final_poses\":true,\"rebuild_method\":\"fresh trilinear hit/free weighted accumulation on shared grid\",\"joint_variable\":\"map_variables.txt log_odds\",\"point_voxel\":"<<pv<<",\"point_stride\":"<<pointstride<<",\"min_range\":"<<minrange<<",\"max_range\":"<<maxrange<<",\"free_samples\":"<<free_samples<<"}\n";
  auto report=output(out/"summary.json"); report<<"{\"mode\":\"3d_sparse_joint_occupancy\",\"frames\":"<<frames.size()<<",\"observations\":"<<problem.obs.size()<<",\"vertices\":"<<problem.grid.keys.size()<<",\"initial_cost\":"<<solved.initial<<",\"final_cost\":"<<solved.final<<",\"accepted_steps\":"<<solved.accepted<<",\"voxel\":"<<problem.grid.h<<",\"stop_reason\":\""<<solved.stop_reason<<"\",\"inexact_linear_solves\":"<<solved.inexact_linear_solves<<",\"completed\":true}\n";
  state.finish(solved.stop_reason);
  return 0;
} catch(const std::exception& e) {std::cerr<<"occupancy_slam_3d: "<<e.what()<<'\n'; return 1;}
