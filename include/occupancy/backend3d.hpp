#pragma once
#include "occupancy/model3d.hpp"
#include "occupancy/linear_solver.hpp"
namespace occupancy::three_d {
struct SolverReport3D {
  double initial=0, final=0; int accepted=0,inexact_linear_solves=0;
  std::string stop_reason="iteration_limit"; LinearReport linear;
};
SolverReport3D optimize3D(Problem& problem,int max_iterations,int linear_iterations,double linear_tolerance,std::ostream& trace);
}
