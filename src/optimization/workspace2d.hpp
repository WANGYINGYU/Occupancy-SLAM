#pragma once
#include "occupancy/backend2d.hpp"
#include <algorithm>
#include <numeric>
namespace occupancy::detail {
// Immutable for a single optimization stage. Public one-shot entrypoints build
// their own layout so callers may freely replace selected_variables between calls.
struct VariableLayout2D { std::vector<int> ids, index; };
inline VariableLayout2D variableLayout2D(const Problem2D& b) {
  VariableLayout2D layout;
  if (!b.selected_variables.size()) {
    layout.ids.resize(b.map.size());
    std::iota(layout.ids.begin(),layout.ids.end(),0);
  } else {
    layout.ids.reserve(b.selected_variables.rows());
    for(int i=0;i<b.selected_variables.rows();++i)
      layout.ids.push_back(b.selected_variables(i,0)*b.parameters.Sizej+b.selected_variables(i,1));
    std::sort(layout.ids.begin(),layout.ids.end());
  }
  layout.index.assign(b.map.size(),-1);
  for(int k=0;k<int(layout.ids.size());++k)layout.index.at(layout.ids[k])=k;
  return layout;
}
NormalSystem2D linearize2D(const Problem2D&,const VariableLayout2D&);
Cost2D evaluate2D(const Problem2D&,const VariableLayout2D&);
void smoothNormalization2D(Problem2D&,const VariableLayout2D&);
void refreshNormalization2D(Problem2D&,const VariableLayout2D&);
}
