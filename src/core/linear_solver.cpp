#include "occupancy/linear_solver.hpp"
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCholesky>
#include <stdexcept>
#include <cmath>
namespace occupancy {
std::pair<Eigen::VectorXd, LinearReport> solveLinear(
    const Eigen::SparseMatrix<double>& a, const Eigen::VectorXd& b,
    bool iterative, int max_iterations, double tolerance, SymmetricStorage storage) {
  if (max_iterations <= 0 || !std::isfinite(tolerance) || tolerance <= 0 || tolerance >= 1)
    throw std::runtime_error("Invalid linear solver settings");
  LinearReport report; report.method = iterative ? "conjugate_gradient" : "simplicial_ldlt";
  report.max_iterations = max_iterations; report.tolerance = tolerance;
  Eigen::VectorXd x; Eigen::ComputationInfo info;
  if (iterative && storage == SymmetricStorage::Lower) {
    // Preserve the established CG product/reduction order and stopping history.
    // Direct LDLT still works on the compact lower triangle without expansion.
    Eigen::SparseMatrix<double> full = a.selfadjointView<Eigen::Lower>();
    return solveLinear(full,b,true,max_iterations,tolerance,SymmetricStorage::Full);
  }
  if (iterative) {
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower|Eigen::Upper> solver;
    solver.setMaxIterations(max_iterations); solver.setTolerance(tolerance); solver.compute(a);
    if (solver.info() != Eigen::Success) throw std::runtime_error("Linear system setup failed");
    x = solver.solve(b); info = solver.info(); report.iterations = solver.iterations();
  } else {
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver; solver.compute(a);
    if (solver.info() != Eigen::Success) throw std::runtime_error("LDLT factorization failed");
    x = solver.solve(b); info = solver.info(); report.iterations = 1;
  }
  if (!x.allFinite() || info == Eigen::NumericalIssue || info == Eigen::InvalidInput)
    throw std::runtime_error("Linear solve produced an invalid result");
  report.relative_residual = (storage == SymmetricStorage::Lower
      ? (a.selfadjointView<Eigen::Lower>()*x-b).norm() : (a*x-b).norm()) / std::max(b.norm(),1e-30);
  report.status = info == Eigen::Success ? "converged" : "iteration_limit";
  return {std::move(x), report};
}
}
