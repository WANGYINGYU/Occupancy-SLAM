#include "MyStruct.h"
#include "SubFuncs.h"
#include "cli.h"
#include "occupancy/backend2d.hpp"
#include "../mapping/export2d.hpp"
#include "apps.hpp"
#include "../io/scan2d.hpp"
int occupancy::run2D(int argc,char**argv) try {
  if(argc==1||(argc==2&&std::string(argv[1])=="--help")) {
    std::cout<<"occupancy_slam_2d (--ranges Range.txt | --endpoints Endpoints.txt | --samples Samples.bin) --poses Pose.txt --config config.txt --output DIR [--odom Odom.txt] [--timestamps times.txt] [--iterations 10] [--resolution 0.1] [--step-policy legacy|decrease] [--max-observations 20000000] [--save-iterations true]\n";return 0;
  }
  Args a(argc,argv,{"--ranges","--endpoints","--samples","--poses","--config","--output","--odom","--timestamps","--iterations","--resolution","--step-policy","--max-observations","--save-iterations","--sample-start"});
  const auto out=newOutput(a);RunState state(out);auto p=SetParam();SetParametersFromFile(a.get("--config"),p);
  p.MaxIter=a.integer("--iterations",p.MaxIter);p.Scale=a.num("--resolution",p.Scale);
  if(p.Scale<=0||p.NumBeam<2||p.MaxRange<=p.MinRange||p.MinRange<0||p.MaxAngle<=p.MinAngle||p.WeightO<0||p.WeightSmoothN<=0||p.MapSmoothingWeightFirst<=0||p.SolverFirstMaxIter<=0||p.SolverFirstTolerance<=0||p.SolverFirstTolerance>=1)throw std::runtime_error("Invalid 2D configuration");
  if(p.ModeMulti&&(p.DownRate<1||p.DownTime<0||p.MapSmoothingWeightSecond<=0||p.SolverSecondMaxIter<1||p.SolverSecondTolerance<=0||p.SolverSecondTolerance>=1))throw std::runtime_error("Invalid multiresolution settings");
  const int coarse_budget=p.ModeMulti?std::min(p.DownTime,std::max(0,p.MaxIter-1)):0;
  p.ModeShowMap=false;p.EvaluateGT=false;
  omp_set_num_threads(1);Eigen::setNbThreads(1);
  int inputs=a.values.count("--ranges")+a.values.count("--endpoints")+a.values.count("--samples");
  if(inputs!=1)throw std::runtime_error("Supply exactly one of --ranges, --endpoints or --samples");
  auto poses=matrix(a.get("--poses"),3);if(poses.rows()<2)throw std::runtime_error("Need >=2 matched scans and poses");
  Eigen::MatrixXd odom,times;
  if(a.values.count("--odom")){odom=matrix(a.get("--odom"),3);if(odom.rows()!=poses.rows())throw std::runtime_error("Odometry row count mismatch");}
  if(p.WeightO>0&&!odom.size())throw std::runtime_error("WeightO > 0 requires --odom");
  if(a.values.count("--timestamps")){times=matrix(a.get("--timestamps"),1);if(times.rows()!=poses.rows())throw std::runtime_error("Timestamp row count mismatch");for(int i=1;i<times.rows();++i)if(times(i,0)<=times(i-1,0))throw std::runtime_error("Timestamps must increase");}
  std::vector<Eigen::ArrayXd> xy(poses.rows()),occ(poses.rows());uint64_t observations=0,budget=a.integer("--max-observations",20000000);
  if(a.values.count("--samples"))readSamples2D(a.get("--samples"),poses.rows(),a.integer("--sample-start",0),budget,xy,occ);
  else if(a.values.count("--endpoints"))readEndpoints2D(a.get("--endpoints"),poses.rows(),p,budget,xy,occ);
  else {
    auto ranges=matrix(a.get("--ranges"),p.NumBeam);if(ranges.rows()!=poses.rows())throw std::runtime_error("Scan row count mismatch");
    for(int i=0;i<ranges.size();++i)if(ranges.data()[i]>p.MinRange&&ranges.data()[i]<p.MaxRange){double n=floor(ranges.data()[i]/p.Scale);if(n>budget-observations)throw std::runtime_error("Observation budget exceeded");observations+=n;}
    FuncConvertObs(ranges,xy,occ,p);
  }
  for(const auto& row:occ)if(!row.size())throw std::runtime_error("Scan has no valid observations");
  if(!p.FixedGrid) {
    Eigen::Vector2d lo=Eigen::Vector2d::Constant(INFINITY),hi=-lo;
    for(int i=0;i<poses.rows();++i)for(int j=0;j<occ[i].size();++j){Eigen::Vector2d q=Eigen::Rotation2Dd(poses(i,2))*xy[i].segment<2>(2*j).matrix()+poses.row(i).head<2>().transpose();lo=lo.cwiseMin(q);hi=hi.cwiseMax(q);}
    double padding=std::max(2.,p.Scale*5.);lo.array()-=padding;hi.array()+=padding;p.OriginX=lo.x();p.OriginY=lo.y();
    double w=ceil((hi.x()-lo.x())/p.Scale)+2,h=ceil((hi.y()-lo.y())/p.Scale)+2;if(!std::isfinite(w*h)||w*h>10000000)throw std::runtime_error("Map exceeds 10M cells");p.Sizej=w;p.Sizei=h;
  }
  if(p.Sizei<5||p.Sizej<5||int64_t(p.Sizei)*p.Sizej>10000000)throw std::runtime_error("Invalid map dimensions");
  const auto fine_parameters=p;const Eigen::MatrixXd initial_poses=poses;
  auto [map,counts]=integrate2D(poses,xy,occ,p,p.InitialMapRound);auto hh=FuncMapConst(p);Eigen::MatrixXd n=counts;FuncSmoothN2(n,hh,p);
  saveMap2D(out/"initial",poses,map,n,&counts,times,p);
  SolverReport2D coarse_report;bool coarse_ran=false;
  auto solve=[&](Problem2D& problem,const std::string& stage){
    auto trace=output(out/(stage+"_iterations.csv"));
    auto snapshot=[&](int i,const Problem2D& b){
      const auto mode=a.get("--save-iterations","false");auto dir=out/"steps"/stage/std::to_string(i);
      if(mode=="true")saveMap2D(dir,b.poses,b.map,b.normalization,nullptr,times,b.parameters);
      else if(mode=="poses"){fs::create_directories(dir);auto f=output(dir/"poses.txt");f<<b.poses<<'\n';}
      else if(mode!="false")throw std::runtime_error("save-iterations must be false, true or poses");
    };
    return optimize2D(problem,a.get("--step-policy","legacy"),trace,snapshot);
  };
  if(p.ModeMulti&&p.MaxIter>0&&coarse_budget>0) {
    auto coarse=p;coarse.Scale*=p.DownRate;coarse.Sizei=(p.Sizei+p.DownRate-1)/p.DownRate;coarse.Sizej=(p.Sizej+p.DownRate-1)/p.DownRate;coarse.MaxIter=coarse_budget;
    std::vector<Eigen::ArrayXd> low_xy(poses.rows()),low_occ(poses.rows());FuncLowSampleScan(xy,occ,low_xy,low_occ,p);
    auto [cm,cn]=integrate2D(poses,low_xy,low_occ,coarse,coarse.InitialMapRound);auto ch=FuncMapConst(coarse);FuncSmoothN2(cn,ch,coarse);
    Problem2D b{coarse,poses,std::move(cm),std::move(cn),odom,std::move(low_xy),std::move(low_occ),ch};b.coarse_stage=true;
    coarse_report=solve(b,"coarse");coarse_ran=true;poses=b.poses;saveMap2D(out/"coarse",poses,b.map,b.normalization,nullptr,times,coarse);
    p.MaxIter=fine_parameters.MaxIter-coarse_budget;
  }
  Problem2D problem{p,poses,{}, {},odom,{}, {},{}};
  if(p.ModeMulti&&p.MaxIter>0) {
    auto high=integrateField2D(poses,xy,occ,p,p.InitialMapRound);Eigen::MatrixXi selected,variables;FuncCalBound(high,selected,variables,p);
    auto clip=[&](Eigen::MatrixXi& cells){int n=0;for(int k=0;k<cells.rows();++k)if(cells(k,0)>=0&&cells(k,0)<p.Sizei&&cells(k,1)>=0&&cells(k,1)<p.Sizej)cells.row(n++)=cells.row(k);cells.conservativeResize(n,2);};clip(selected);clip(variables);
    if(!variables.rows())throw std::runtime_error("No selected variables in fine stage");
    std::vector<Eigen::ArrayXd> selected_xy(poses.rows()),selected_occ(poses.rows());FuncSelectScan(high,poses,xy,occ,selected_xy,selected_occ,selected,p);
    auto [sm,sn]=integrate2D(poses,selected_xy,selected_occ,p,false);Eigen::SparseMatrix<double> sh;FuncSelectMapConst(sm,variables,sh,p);
    problem.map=std::move(sm);problem.normalization=std::move(sn);problem.samples=std::move(selected_xy);problem.labels=std::move(selected_occ);problem.smoothness=sh;problem.selected_variables=variables;problem.parameters.WeightSmoothN=p.WeightSmoothNSecond;
    // The selected integration above uses floor bins. Reuse its fresh counts
    // only when the iterative N rule agrees; rounded N still needs a new pass.
    if(p.NormalizationRound)refreshNormalization2D(problem);
    else smoothNormalization2D(problem);
  } else {problem.map=std::move(map);problem.normalization=std::move(n);problem.samples=std::move(xy);problem.labels=std::move(occ);problem.smoothness=hh;}
  auto solved=solve(problem,"fine");poses=problem.poses;map=problem.map;n=problem.normalization;
  if(!problem.selected_variables.size()){xy=std::move(problem.samples);occ=std::move(problem.labels);}
  auto [rebuilt,rebuilt_counts]=integrate2D(poses,xy,occ,fine_parameters,fine_parameters.InitialMapRound);Eigen::MatrixXd rebuilt_n=rebuilt_counts;FuncSmoothN2(rebuilt_n,hh,fine_parameters);
  saveMap2D(out/"joint",poses,map,n,nullptr,times,p);saveMap2D(out/"reintegrated",poses,rebuilt,rebuilt_n,&rebuilt_counts,times,fine_parameters);
  for(const auto& entry:fs::directory_iterator(out/"joint"))fs::copy_file(entry.path(),out/entry.path().filename());
  fs::copy_file(out/"fine_iterations.csv",out/"iterations.csv");
  auto exports=output(out/"map_outputs.json");
  exports<<"{\"schema_version\":2,\"mode\":\"2d\",\"initial\":\"initial\",\"joint\":\"joint\",\"reintegrated\":\"reintegrated\",\"normalization_pose_derivative\":\"omitted_paper_approximation\",\"normalization_refresh\":\"fresh accumulation then smooth after each accepted iteration\",\"same_grid\":true,\"fixed_final_poses\":true,\"rebuild_method\":\"fresh original samples at final poses\",\"joint_variable\":\"occupancy.txt is M; probability=sigmoid(M/N)\",\"hit_log_odds\":"<<p.ValOccupied<<",\"free_log_odds\":"<<p.ValFree<<"}\n";
  auto report=output(out/"summary.json");
  report<<"{\"mode\":\""<<(p.ModeMulti?"2d_coarse_to_fine":"2d_full_grid")<<"\",\"frames\":"<<poses.rows()<<",\"cells\":"<<map.size()<<",\"resolution\":"<<p.Scale<<",\"iterations\":"<<solved.iterations+coarse_report.iterations<<",\"coarse_iterations\":"<<coarse_report.iterations<<",\"fine_iterations\":"<<solved.iterations<<",\"coarse_stop_reason\":\""<<(coarse_ran?coarse_report.stop_reason:"not_run")<<"\",\"cost_domain\":\"last optimization stage, before and after\",\"initial_scan_sse\":"<<solved.initial.scan<<",\"final_scan_sse\":"<<solved.final.scan<<",\"initial_total_cost\":"<<solved.initial.total()<<",\"final_total_cost\":"<<solved.final.total()<<",\"solver_max_iterations\":"<<p.SolverFirstMaxIter<<",\"solver_tolerance\":"<<p.SolverFirstTolerance<<",\"stop_reason\":\""<<solved.stop_reason<<"\",\"step_policy\":\""<<solved.step_policy<<"\",\"normalization_updates\":"<<solved.normalization_updates+coarse_report.normalization_updates<<",\"normalization_model\":\"paper approximation: omit N pose derivative; recompute after each accepted iteration\",\"inexact_linear_solves\":"<<solved.inexact_linear_solves+coarse_report.inexact_linear_solves<<",\"objective_increased\":"<<(solved.final.total()>solved.initial.total()?"true":"false")<<",\"completed\":true}\n";
  state.finish(solved.stop_reason);return 0;
} catch(const std::exception& e){std::cerr<<"occupancy_slam_2d: "<<e.what()<<'\n';return 1;}
