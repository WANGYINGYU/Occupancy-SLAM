#include "occupancy/backend2d.hpp"
#include "SubFuncs.h"
#include "workspace2d.hpp"
#include <stdexcept>
#include <iostream>
namespace occupancy {
Cost2D detail::evaluate2D(const Problem2D& b,const detail::VariableLayout2D& layout) {
  const auto& p=b.parameters;Cost2D c;
  const auto& selected=layout.index;
  for(int i=0;i<b.poses.rows();++i) {
    double co=cos(b.poses(i,2)),si=sin(b.poses(i,2));
    for(int j=0;j<b.labels[i].size();++j) {
      double lx=b.samples[i](2*j),ly=b.samples[i](2*j+1);
      double x=(co*lx-si*ly+b.poses(i,0)-p.OriginX)/p.Scale,y=(si*lx+co*ly+b.poses(i,1)-p.OriginY)/p.Scale;
      int u=floor(x),v=floor(y);if(u<0||v<0||u+1>=p.Sizej||v+1>=p.Sizei)throw std::runtime_error("Observation outside evaluation grid");
      if(selected[v*p.Sizej+u]<0||selected[v*p.Sizej+u+1]<0||selected[(v+1)*p.Sizej+u]<0||selected[(v+1)*p.Sizej+u+1]<0)continue;
      double a=x-u,d=y-v;
      auto interp=[&](const Eigen::MatrixXd& m){return (1-a)*(1-d)*m(v,u)+a*(1-d)*m(v,u+1)+(1-a)*d*m(v+1,u)+a*d*m(v+1,u+1);};
      double n=interp(b.normalization);if(!(n>0))throw std::runtime_error("Invalid normalization");
      double e=interp(b.map)/n-b.labels[i](j);c.scan+=e*e;
    }
  }
  if(p.WeightO>0) for(int i=1;i<b.poses.rows();++i) {
    Eigen::Vector2d dt=Eigen::Rotation2Dd(-b.poses(i-1,2))*(b.poses.row(i).head<2>()-b.poses.row(i-1).head<2>()).transpose();
    dt-=b.odometry.row(i).head<2>().transpose();
    double yaw=FuncWrap(b.poses(i,2)-b.poses(i-1,2)-b.odometry(i,2));
    c.odometry+=p.WeightO*(dt.squaredNorm()/(.05*.05)+yaw*yaw/(.02*.02));
  }
  Eigen::VectorXd field;
  if(b.selected_variables.size()) {
    const auto& ids=layout.ids;
    field.resize(ids.size());for(int i=0;i<field.size();++i)field(i)=b.map(ids[i]/p.Sizej,ids[i]%p.Sizej);
  } else field=b.map.transpose().reshaped(b.map.size(),1);
  double lambda=b.selected_variables.size()?p.MapSmoothingWeightSecond:p.MapSmoothingWeightFirst;
  c.smoothness=lambda*field.dot(b.smoothness*field);
  if(!std::isfinite(c.total()))throw std::runtime_error("Nonfinite 2D objective");return c;
}
Cost2D evaluate2D(const Problem2D& b) {return detail::evaluate2D(b,detail::variableLayout2D(b));}
static bool inside(const Problem2D& b) {
  const auto& p=b.parameters;
  for(int i=0;i<b.poses.rows();++i)for(int j=0;j<b.labels[i].size();++j) {
    Eigen::Vector2d q=Eigen::Rotation2Dd(b.poses(i,2))*b.samples[i].segment<2>(2*j).matrix()+b.poses.row(i).head<2>().transpose();
    q=(q-Eigen::Vector2d(p.OriginX,p.OriginY))/p.Scale;
    if(!q.allFinite()||q.x()<1||q.y()<1||q.x()>p.Sizej-3||q.y()>p.Sizei-3)return false;
  }return true;
}
SolverReport2D optimize2D(Problem2D& b,const std::string& policy,std::ostream& trace,
 const std::function<void(int,const Problem2D&)>& snapshot) {
  if(policy!="legacy"&&policy!="decrease")throw std::runtime_error("step-policy must be legacy or decrease");
  const auto layout=detail::variableLayout2D(b);
  SolverReport2D report;report.step_policy=policy;report.initial=detail::evaluate2D(b,layout);report.final=report.initial;auto& p=b.parameters;
  if(p.MaxIter==0)report.stop_reason="not_run";
  trace<<"iteration,scan_sse,mean_pose_step_squared,total_before,frozen_n_total_after,total_after_n_refresh,linear_status,linear_iterations,linear_relative_residual,alpha,accepted,mean_joint_step_squared,observations\n";
  for(int iter=0;iter<p.MaxIter;++iter) {
    auto system=detail::linearize2D(b,layout);const auto before=system.cost;
    bool fine=b.selected_variables.size();
    auto solution=solveLinear(system.hessian,system.rhs,p.ModeSparseSolver,fine?p.SolverSecondMaxIter:p.SolverFirstMaxIter,fine?p.SolverSecondTolerance:p.SolverFirstTolerance,system.storage);
    report.linear=solution.second;Eigen::VectorXd dp=solution.first.head(3*(b.poses.rows()-1)),dm=solution.first.tail(b.smoothness.rows());
    if(report.linear.status!="converged")++report.inexact_linear_solves;
    const Eigen::MatrixXd old_pose=b.poses,old_map=b.map;
    double alpha=1;bool accepted=false;Cost2D candidate=before;
    for(int trial=0;trial<20;++trial,alpha*=.5) {
      b.poses=old_pose;b.map=old_map;
      if(fine)FuncSelectMapUpdate(b.map,b.poses,alpha*dp,alpha*dm,b.selected_variables);else FuncUpdate(b.map,b.poses,alpha*dp,alpha*dm);
      if(!inside(b))continue;candidate=detail::evaluate2D(b,layout);
      if(policy=="legacy"||candidate.total()<=before.total()){accepted=true;break;}
    }
    if(!accepted){b.poses=old_pose;b.map=old_map;report.stop_reason="no_acceptable_step";break;}
    report.mean_joint_step_squared=solution.first.squaredNorm()*alpha*alpha/solution.first.size();
    report.mean_pose_step_squared=dp.squaredNorm()*alpha*alpha/dp.size();
    detail::refreshNormalization2D(b,layout);++report.normalization_updates;++report.iterations;report.final=detail::evaluate2D(b,layout);
    trace<<iter<<','<<before.scan<<','<<report.mean_pose_step_squared<<','<<before.total()<<','<<candidate.total()<<','<<report.final.total()<<','<<report.linear.status<<','<<report.linear.iterations<<','<<report.linear.relative_residual<<','<<alpha<<",1,"<<report.mean_joint_step_squared<<','<<system.observations<<'\n';trace.flush();
    if(snapshot)snapshot(iter+1,b);
    std::cout<<"iteration="<<iter<<" total_after_n_refresh="<<report.final.total()<<" linear="<<report.linear.status<<std::endl;
    if(b.coarse_stage&&(report.mean_joint_step_squared<p.MinMeanDeltaFirst||report.mean_pose_step_squared<p.MinMeanDeltaPoseFirst)){report.stop_reason="coarse_threshold";break;}
    if(!b.coarse_stage&&(report.mean_joint_step_squared<p.MinDelta||report.mean_pose_step_squared<p.MinDeltaPose)){report.stop_reason="small_joint_step";break;}
  }return report;
}
}
