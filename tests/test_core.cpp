#include "MyStruct.h"
#include "SubFuncs.h"
#include "occupancy/backend2d.hpp"
#include <iostream>
void require(bool good,const char* why){if(!good)throw std::runtime_error(why);}
int main() try {
  omp_set_num_threads(1);auto p=SetParam();p.Sizei=8;p.Sizej=8;p.Scale=1;p.OriginX=0;p.OriginY=0;
  Eigen::MatrixXd map=Eigen::MatrixXd::Ones(8,8),n(8,8),poses=Eigen::MatrixXd::Zero(2,3);
  for(int y=0;y<8;++y)for(int x=0;x<8;++x)n(y,x)=2+.1*x+.2*y;
  poses.row(1)<<.1,.2,.05;
  std::vector<Eigen::ArrayXd> xy(2),labels(2);
  for(int i=0;i<2;++i){xy[i].resize(2);xy[i]<<2.2,3.3;labels[i]=Eigen::ArrayXd::Constant(1,p.ValOccupied);}
  auto [jp,jd,is,es,sum,mean]=FuncDiffJacobian(map,n,poses,xy,labels,p);
  require(jp.norm()<1e-14,"Paper approximation must omit the pose derivative of N");
  occupancy::Problem2D approximation{p,poses,map,n,Eigen::MatrixXd::Zero(2,3),xy,labels,FuncMapConst(p)};
  approximation.parameters.WeightO=0.;
  auto frozen_n=occupancy::linearize2D(approximation);
  require(frozen_n.hessian.topLeftCorner(3,3).norm()<1e-14 && frozen_n.rhs.head(3).norm()<1e-14,
          "Streaming normal equations must also omit the pose derivative of N");
  // Nonconstant N still changes the residual. This is intentional, not an FD failure.
  auto moved=poses;moved(1,0)+=.01;
  auto result=FuncDiffJacobian(map,n,moved,xy,labels,p);
  require(std::abs(std::get<3>(result)[1]-es[1])>1e-6,"Residual must still evaluate interpolated N");
  Eigen::SparseMatrix<double> a(12,12);
  for(int i=0;i<12;++i){a.insert(i,i)=2+i*.1;if(i){a.insert(i,i-1)=-.5;a.insert(i-1,i)=-.5;}}
  Eigen::VectorXd b=Eigen::VectorXd::Ones(12);
  auto low=occupancy::solveLinear(a,b,true,1,1e-10),high=occupancy::solveLinear(a,b,true,100,1e-10),direct=occupancy::solveLinear(a,b,false,100,1e-10);
  require(low.second.status=="iteration_limit" && low.second.iterations==1,"CG limit must be applied and reported");
  require(high.second.relative_residual<1e-9 && (high.first-direct.first).norm()<1e-8,"Configured CG/direct solves disagree");
  Eigen::SparseMatrix<double> lower=a.triangularView<Eigen::Lower>();
  for(bool iterative:{false,true}) {
    auto result=occupancy::solveLinear(lower,b,iterative,100,1e-10,occupancy::SymmetricStorage::Lower);
    require((result.first-direct.first).norm()<1e-8 && result.second.relative_residual<1e-9,
            "Lower storage must solve the same symmetric system, including CG/residual reporting");
  }
  // On a bilinear field the interpolated central gradient is exact. A constant
  // N isolates pose/map indexing and rotation signs from the intentional N approximation.
  n.setConstant(2.);
  for(int y=0;y<8;++y) for(int x=0;x<8;++x) map(y,x)=.1+.2*x-.3*y+.01*x*y;
  auto [ap,ad,ai,ae,as,am]=FuncDiffJacobian(map,n,poses,xy,labels,p);
  const double epsilon=1e-6;
  for(int column=0;column<ap.cols();++column) {
    auto plus=poses,minus=poses;plus(column/3,column%3)+=epsilon;minus(column/3,column%3)-=epsilon;
    Eigen::VectorXd numerical=(std::get<3>(FuncDiffJacobian(map,n,plus,xy,labels,p))-std::get<3>(FuncDiffJacobian(map,n,minus,xy,labels,p)))/(2*epsilon);
    require((numerical-Eigen::VectorXd(ap.col(column))).norm()<1e-8,"Pose Jacobian coordinate/rotation mismatch on exact bilinear field");
  }
  for(int column=0;column<ad.cols();++column) {
    auto plus=map,minus=map;plus(column/p.Sizej,column%p.Sizej)+=epsilon;minus(column/p.Sizej,column%p.Sizej)-=epsilon;
    Eigen::VectorXd numerical=(std::get<3>(FuncDiffJacobian(plus,n,poses,xy,labels,p))-std::get<3>(FuncDiffJacobian(minus,n,poses,xy,labels,p)))/(2*epsilon);
    require((numerical-Eigen::VectorXd(ad.col(column))).norm()<1e-8,"Map Jacobian variable ordering mismatch");
  }
  // Streaming normal equations must equal the original sparse Jacobian products,
  // including the fixed first pose, map smoothness and optional odometry factors.
  for(int y=0;y<8;++y)for(int x=0;x<8;++x)n(y,x)=2+.1*x+.2*y;
  auto hh=FuncMapConst(p);Eigen::MatrixXd odom=Eigen::MatrixXd::Zero(2,3);
  occupancy::Problem2D problem{p,poses,map,n,odom,xy,labels,hh};
  for(bool use_odom:{false,true}) {
    problem.parameters.WeightO=use_odom?.3:0.;
    auto [pp,dd,oo,ss,ii,ee,eo,se,me]=FuncDiffJacobian(map,n,poses,odom,xy,labels,problem.parameters);
    Eigen::SparseMatrix<double> j=pp.block(0,3,pp.rows(),pp.cols()-3),o=oo.block(0,3,oo.rows(),oo.cols()-3);
    Eigen::SparseMatrix<double> hpp=j.transpose()*j+problem.parameters.WeightO*o.transpose()*ii*o;
    Eigen::SparseMatrix<double> hdd=dd.transpose()*dd+p.MapSmoothingWeightFirst*hh,hpd=j.transpose()*dd,hpd_t=hpd.transpose();
    Eigen::SparseMatrix<double> top=igl::cat(2,hpp,hpd),bottom=igl::cat(2,hpd_t,hdd),expected_h=igl::cat(1,top,bottom);
    Eigen::VectorXd f=map.transpose().reshaped(map.size(),1),bp=-j.transpose()*ee-problem.parameters.WeightO*o.transpose()*ii*eo,bd=-dd.transpose()*ee-p.MapSmoothingWeightFirst*hh*f,expected_b=igl::cat(1,bp,bd);
    auto system=occupancy::linearize2D(problem);
    require((Eigen::SparseMatrix<double>(system.hessian.selfadjointView<Eigen::Lower>())-expected_h).norm()<1e-9*std::max(1.,expected_h.norm()),"Streaming Hessian differs from original Jacobian products");
    require((system.rhs-expected_b).norm()<1e-9*std::max(1.,expected_b.norm()),"Streaming RHS differs from original Jacobian products");
    require(std::abs(system.cost.total()-occupancy::evaluate2D(problem).total())<1e-9,"Streaming objective differs from evaluator");
  }
  problem.parameters.WeightO=0.;problem.selected_variables.resize(36,2);
  for(int y=1,k=0;y<7;++y)for(int x=1;x<7;++x){problem.selected_variables.row(k++)<<y,x;}
  FuncSelectMapConst(map,problem.selected_variables,problem.smoothness,problem.parameters);
  auto [sp,sd,si,se,ss,sm]=FuncDiffSelectJacobian(map,n,poses,xy,labels,problem.selected_variables,problem.parameters);
  Eigen::SparseMatrix<double> sj=sp.block(0,3,sp.rows(),sp.cols()-3),su=sj.transpose()*sj,sv=sd.transpose()*sd+p.MapSmoothingWeightSecond*problem.smoothness,sw=sj.transpose()*sd,swt=sw.transpose();
  Eigen::SparseMatrix<double> stop=igl::cat(2,su,sw),sbottom=igl::cat(2,swt,sv),sh=igl::cat(1,stop,sbottom);
  problem.parameters.ModeSparseSolver=false;
  auto selected_system=occupancy::linearize2D(problem);
  require((Eigen::SparseMatrix<double>(selected_system.hessian.selfadjointView<Eigen::Lower>())-sh).norm()<1e-9,"Selected streaming Hessian differs from original Jacobian products");
  problem.parameters.NormalizationRound=false;problem.normalization.setConstant(999.);
  occupancy::refreshNormalization2D(problem);auto first_n=problem.normalization;
  occupancy::refreshNormalization2D(problem);
  require((problem.normalization-first_n).norm()<1e-12,"Selected N must be rebuilt, not accumulated across iterations");
  auto floor_counts=occupancy::integrate2D(poses,xy,labels,p,false).second;
  auto round_counts=occupancy::integrate2D(poses,xy,labels,p,true).second;
  require((occupancy::integrateCounts2D(poses,xy,labels,p,false)-floor_counts).norm()==0 &&
          (occupancy::integrateCounts2D(poses,xy,labels,p,true)-round_counts).norm()==0,
          "Counts-only integration changed sample accumulation or rounding");
  for(bool rounded:{false,true}) {
    auto both=occupancy::integrate2D(poses,xy,labels,p,rounded);
    require((occupancy::integrateField2D(poses,xy,labels,p,rounded)-both.first).norm()==0,
            "Field-only integration changed accumulation");
    problem.parameters.NormalizationRound=rounded;
    problem.normalization=occupancy::integrateCounts2D(poses,xy,labels,p,rounded);
    occupancy::smoothNormalization2D(problem);auto reused=problem.normalization;
    occupancy::refreshNormalization2D(problem);
    require((reused-problem.normalization).norm()==0,"Fresh count reuse changes smoothed N");
  }
  for(int col=0;col<selected_system.hessian.outerSize();++col)
    for(Eigen::SparseMatrix<double>::InnerIterator it(selected_system.hessian,col);it;++it)
      require(it.row()>=it.col(),"2D normal equations must store only the lower triangle");
  require(floor_counts.sum()==2&&round_counts.sum()==2&&(floor_counts-round_counts).norm()>0,"Explicit initial-map binning ignored");
  // Original N update bins by round; preserve it separately from floor integration.
  Eigen::MatrixXd updated;FuncUpdateMapN(updated,poses,xy,p);
  require(updated.sum()==2 && updated(3,2)==1 && updated(4,2)==1,"N refresh changed its original round rule");
  std::cout<<"PASS: paper N approximation, N refresh, CG settings/status, direct solve, bilinear pose/map Jacobian, public headers\n";
  return 0;
} catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
