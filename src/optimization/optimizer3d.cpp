#include "occupancy/backend3d.hpp"
namespace occupancy::three_d {
SolverReport3D optimize3D(Problem& problem,int max_iterations,int linear_iterations,double linear_tolerance,std::ostream& trace) {
  SolverReport3D report;
  if(max_iterations==0) report.stop_reason="not_run";
  double initial=problem.evaluate(problem.poses,problem.grid.value),cost=initial,lambda=1e-3; int accepted=0;
  if(!std::isfinite(cost)) throw std::runtime_error("Invalid initial objective");
  for(int iteration=0;iteration<max_iterations;++iteration) {
    Sparse j; Eigen::VectorXd r; problem.evaluate(problem.poses,problem.grid.value,&j,&r);
    Sparse h=j.transpose()*j; Eigen::VectorXd gradient=j.transpose()*r,diagonal=h.diagonal().cwiseMax(1.);
    if(gradient.lpNorm<Eigen::Infinity>()<1e-8) {report.stop_reason="small_gradient";break;}
    bool ok=false; double norm=0,before=cost;
    for(int trial=0;trial<10;++trial) {
      Sparse system=h; for(int i=0;i<system.rows();++i) system.coeffRef(i,i)+=lambda*diagonal[i]; system.makeCompressed();
      auto solved=occupancy::solveLinear(system,-gradient,true,linear_iterations,linear_tolerance);
      Eigen::VectorXd step=std::move(solved.first);report.linear=solved.second;
      if(report.linear.status!="converged") ++report.inexact_linear_solves;
      auto pp=problem.poses; for(int i=1;i<int(pp.size());++i) pp[i]=retract(pp[i],step.segment<6>(6*(i-1)));
      Eigen::VectorXd mm=problem.grid.value+step.tail(problem.grid.value.size());
      double candidate=problem.evaluate(pp,mm); norm=step.norm();
      if(std::isfinite(candidate) && candidate<cost) {problem.poses=std::move(pp); problem.grid.value=std::move(mm); cost=candidate; ok=true; ++accepted; break;}
      lambda*=10;
    }
    trace<<iteration<<','<<before<<','<<cost<<','<<lambda<<','<<norm<<','<<ok<<','<<report.linear.status<<','<<report.linear.iterations<<','<<report.linear.relative_residual<<'\n'; trace.flush();
    std::cout<<"iteration="<<iteration<<" cost="<<cost<<" accepted="<<ok<<" vertices="<<problem.grid.keys.size()<<std::endl;
    if(!ok) {report.stop_reason="no_acceptable_step";break;}
    if(norm<1e-6) {report.stop_reason="small_step";break;}
    if((before-cost)<1e-8*(1+before)) {report.stop_reason="small_cost_change";break;}
    lambda=std::max(1e-9,lambda*0.3);
  }
  report.initial=initial;report.final=cost;report.accepted=accepted;return report;
}
}
