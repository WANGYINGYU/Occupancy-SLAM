#pragma once
#include "MyStruct.h"
#include "occupancy/linear_solver.hpp"
#include <Eigen/Sparse>
#include <string>
#include <vector>
#include <ostream>
#include <functional>
namespace occupancy {
struct Cost2D { double scan=0, odometry=0, smoothness=0;
  double total() const { return scan+odometry+smoothness; }
};
struct Problem2D {
  ParamStruct parameters;
  Eigen::MatrixXd poses, map, normalization, odometry;
  std::vector<Eigen::ArrayXd> samples, labels;
  Eigen::SparseMatrix<double> smoothness;
  Eigen::MatrixXi selected_variables; // Empty means the full grid.
  bool coarse_stage=false;
};
struct SolverReport2D {
  std::string stop_reason="iteration_limit", step_policy="legacy";
  int iterations=0, normalization_updates=0, inexact_linear_solves=0;
  Cost2D initial, final;
  LinearReport linear;
  double mean_joint_step_squared=0, mean_pose_step_squared=0;
};
struct NormalSystem2D {
  Eigen::SparseMatrix<double> hessian; // Lower for direct LDLT; full for CG traversal compatibility.
  SymmetricStorage storage = SymmetricStorage::Lower;
  Eigen::VectorXd rhs;
  Cost2D cost;
  uint64_t observations=0;
};
// Assemble the same frozen-N Jacobians by accumulating their small row products.
// Memory depends on the grid/poses rather than the number of ray samples.
NormalSystem2D linearize2D(const Problem2D& problem);
std::pair<Eigen::MatrixXd,Eigen::MatrixXd> integrate2D(const Eigen::MatrixXd& poses,
    const std::vector<Eigen::ArrayXd>& samples,const std::vector<Eigen::ArrayXd>& labels,
    const ParamStruct& parameters,bool round_cells);
// Selective accumulation preserves the same samples, order and rounding rules.
Eigen::MatrixXd integrateCounts2D(const Eigen::MatrixXd& poses,
    const std::vector<Eigen::ArrayXd>& samples,const std::vector<Eigen::ArrayXd>& labels,
    const ParamStruct& parameters,bool round_cells);
Eigen::MatrixXd integrateField2D(const Eigen::MatrixXd& poses,
    const std::vector<Eigen::ArrayXd>& samples,const std::vector<Eigen::ArrayXd>& labels,
    const ParamStruct& parameters,bool round_cells);
// Smooth a fresh raw count grid; refresh additionally rebuilds it at current poses.
void smoothNormalization2D(Problem2D& problem);
void refreshNormalization2D(Problem2D& problem);
Cost2D evaluate2D(const Problem2D& problem);
SolverReport2D optimize2D(Problem2D& problem, const std::string& step_policy, std::ostream& trace, const std::function<void(int,const Problem2D&)>& snapshot = {});
}
