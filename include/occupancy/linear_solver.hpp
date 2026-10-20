#pragma once
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <string>
#include <utility>
namespace occupancy {
// Full retains the existing symmetric full-matrix API (including the 3D backend).
// Lower stores only the lower triangle; all operations use its self-adjoint view.
enum class SymmetricStorage { Full, Lower };
struct LinearReport {
  std::string method, status;
  int iterations = 0, max_iterations = 0;
  double tolerance = 0, relative_residual = 0;
};
std::pair<Eigen::VectorXd, LinearReport> solveLinear(
    const Eigen::SparseMatrix<double>& a, const Eigen::VectorXd& b,
    bool iterative, int max_iterations, double tolerance,
    SymmetricStorage storage = SymmetricStorage::Full);
}
