// Diagnostic adapter for the original selected-cell second stage.
// No derivative of N is introduced; N is rebuilt after each joint update.
#include "MyStruct.h"
#include "SubFuncs.h"
#include "cli.h"
#include "src/io/scan2d.hpp"
#include "src/mapping/export2d.hpp"
#include <fstream>
#include "occupancy/linear_solver.hpp"
int main(int argc,char** argv) try {
  Args a(argc,argv,{"--endpoints","--poses","--config","--output","--iterations"});
  auto out=newOutput(a);RunState state(out);
  auto p=SetParam();SetParametersFromFile(a.get("--config"),p);
  omp_set_num_threads(1);Eigen::setNbThreads(1);
  auto poses=matrix(a.get("--poses"),3);
  std::vector<Eigen::ArrayXd> samples,labels;
  occupancy::readEndpoints2D(a.get("--endpoints"),poses.rows(),p,60000000,samples,labels);
  Eigen::Vector2d lo=Eigen::Vector2d::Constant(INFINITY),hi=-lo;
  for(int i=0;i<poses.rows();++i) for(int j=0;j<labels[i].size();++j) {
    Eigen::Vector2d q=Eigen::Rotation2Dd(poses(i,2))*samples[i].segment<2>(2*j).matrix()+poses.row(i).head<2>().transpose();
    lo=lo.cwiseMin(q);hi=hi.cwiseMax(q);
  }
  lo.array()-=2.;hi.array()+=2.;p.OriginX=lo.x();p.OriginY=lo.y();
  p.Sizej=std::ceil((hi.x()-lo.x())/p.Scale)+2;p.Sizei=std::ceil((hi.y()-lo.y())/p.Scale)+2;
  auto [full,counts]=FuncInitialiseGridMap(poses,samples,labels,p);
  Eigen::MatrixXi selected,variables;FuncCalBound(full,selected,variables,p);
  std::vector<Eigen::ArrayXd> xy(poses.rows()),odd(poses.rows());
  FuncSelectScan(full,poses,samples,labels,xy,odd,selected,p);
  samples.clear();labels.clear();
  Eigen::MatrixXd map,n;FuncInitialSelectMap(map,n,poses,xy,odd,p);
  Eigen::SparseMatrix<double> hh;FuncSelectMapConst(map,variables,hh,p);
  Eigen::SparseMatrix<double> weight=hh*p.MapSmoothingWeightSecond;
  FuncSmoothSelectN2(n,weight,variables,p);
  Eigen::MatrixXd times;
  occupancy::saveMap2D(out/"initial",poses,map,n,nullptr,times,p);
  auto log=output(out/"iterations.csv");log<<"iteration,scan_sse,mean_joint_step_squared,mean_pose_step_squared,linear_iterations,linear_relative_residual,linear_status\n";
  int iterations=a.integer("--iterations",5);
  for(int i=0;i<iterations;++i) {
    auto [jp,jd,is,error,sse,mean]=FuncDiffSelectJacobian(map,n,poses,xy,odd,variables,p);
    // Same selected-stage normal equations; honor the requested solve tolerance
    // and report the actual residual instead of silently capping at sqrt(n).
    Eigen::SparseMatrix<double> j=jp.block(0,3,jp.rows(),jp.cols()-3);
    Eigen::SparseMatrix<double> u=j.transpose()*j,v=jd.transpose()*jd+weight,w=j.transpose()*jd,wt=w.transpose();
    Eigen::SparseMatrix<double> top=igl::cat(2,u,w),bottom=igl::cat(2,wt,v),h=igl::cat(1,top,bottom);
    Eigen::ArrayXi ids=variables.col(0).array()*p.Sizej+variables.col(1).array();std::sort(ids.data(),ids.data()+ids.size());
    Eigen::ArrayXd field=map.transpose().reshaped(map.size(),1);
    Eigen::VectorXd selected_field=field(ids),bp=-j.transpose()*error,bm=-jd.transpose()*error-weight*selected_field,b=igl::cat(1,bp,bm);
    auto [delta,linear]=occupancy::solveLinear(h,b,true,p.SolverSecondMaxIter,p.SolverSecondTolerance);
    Eigen::VectorXd dp=delta.head(j.cols()),dm=delta.tail(jd.cols());
    double step=delta.squaredNorm()/delta.size(),stepp=dp.squaredNorm()/dp.size();
    if(!dp.allFinite()||!dm.allFinite()) throw std::runtime_error("Invalid selected-stage update");
    log<<i<<','<<sse<<','<<step<<','<<stepp<<','<<linear.iterations<<','<<linear.relative_residual<<','<<linear.status<<'\n';log.flush();
    FuncSelectMapUpdate(map,poses,dp,dm,variables);
    // Recompute means fresh accumulation. The original helper adds into its argument.
    n.setZero();FuncUpdateSelectN(n,poses,xy,p);FuncSmoothSelectN2(n,weight,variables,p);
    occupancy::saveMap2D(out/("step_"+std::to_string(i+1)),poses,map,n,nullptr,times,p);
    std::cout<<"iteration="<<i<<" scan_sse="<<sse<<" pose_step="<<stepp<<std::endl;
  }
  occupancy::saveMap2D(out/"joint",poses,map,n,nullptr,times,p);
  state.finish("diagnostic_iteration_limit");return 0;
} catch(const std::exception& e) {std::cerr<<e.what()<<std::endl;return 1;}
