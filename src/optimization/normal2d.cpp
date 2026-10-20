#include "occupancy/backend2d.hpp"
#include "SubFuncs.h"
#include "workspace2d.hpp"
#include <array>
#include <stdexcept>
#include <numeric>
namespace occupancy {
namespace {
double interpolate(const Eigen::MatrixXd& m,int x,int y,const std::array<double,4>& w) {
  return w[0]*m(y,x)+w[1]*m(y,x+1)+w[2]*m(y+1,x)+w[3]*m(y+1,x+1);
}
}
template<bool Field, bool Counts>
static std::pair<Eigen::MatrixXd,Eigen::MatrixXd> integrateImpl(const Eigen::MatrixXd& poses,
 const std::vector<Eigen::ArrayXd>& xy,const std::vector<Eigen::ArrayXd>& labels,
 const ParamStruct& p,bool rounded) {
  Eigen::MatrixXd m,n;
  if constexpr(Field)m=Eigen::MatrixXd::Zero(p.Sizei,p.Sizej);
  if constexpr(Counts)n=Eigen::MatrixXd::Zero(p.Sizei,p.Sizej);
  for(int i=0;i<poses.rows();++i) {
    const double c=std::cos(poses(i,2)),s=std::sin(poses(i,2));
    for(int j=0;j<labels[i].size();++j) {
      const double x=(c*xy[i](2*j)-s*xy[i](2*j+1)+poses(i,0)-p.OriginX)/p.Scale;
      const double y=(s*xy[i](2*j)+c*xy[i](2*j+1)+poses(i,1)-p.OriginY)/p.Scale;
      int u=rounded?std::lround(x):std::floor(x),v=rounded?std::lround(y):std::floor(y);
      if(u<0||v<0||u>=p.Sizej||v>=p.Sizei) throw std::runtime_error("Observation outside configured grid");
      if constexpr(Field)m(v,u)+=labels[i](j);
      if constexpr(Counts)n(v,u)+=1;
    }
  }
  return {std::move(m),std::move(n)};
}
std::pair<Eigen::MatrixXd,Eigen::MatrixXd> integrate2D(const Eigen::MatrixXd& poses,
 const std::vector<Eigen::ArrayXd>& xy,const std::vector<Eigen::ArrayXd>& labels,
 const ParamStruct& p,bool rounded) {return integrateImpl<true,true>(poses,xy,labels,p,rounded);}
Eigen::MatrixXd integrateCounts2D(const Eigen::MatrixXd& poses,
 const std::vector<Eigen::ArrayXd>& xy,const std::vector<Eigen::ArrayXd>& labels,
 const ParamStruct& p,bool rounded) {return integrateImpl<false,true>(poses,xy,labels,p,rounded).second;}
Eigen::MatrixXd integrateField2D(const Eigen::MatrixXd& poses,
 const std::vector<Eigen::ArrayXd>& xy,const std::vector<Eigen::ArrayXd>& labels,
 const ParamStruct& p,bool rounded) {return integrateImpl<true,false>(poses,xy,labels,p,rounded).first;}
void detail::smoothNormalization2D(Problem2D& b,const detail::VariableLayout2D& layout) {
  if(b.selected_variables.size()) {
    const auto& ids=layout.ids;const int size=ids.size();
    std::vector<Eigen::Triplet<double>> t;Eigen::VectorXd rhs=Eigen::VectorXd::Zero(size);
    for(int k=0;k<size;++k) {double n=b.normalization(ids[k]/b.parameters.Sizej,ids[k]%b.parameters.Sizej);if(n>0){t.emplace_back(k,k,1.);rhs(k)=n;}}
    Eigen::SparseMatrix<double> observed(size,size);observed.setFromTriplets(t.begin(),t.end());
    Eigen::SparseMatrix<double> h=observed+b.parameters.WeightSmoothN*b.smoothness;
    auto solved=solveLinear(h,rhs,false,b.parameters.SolverSecondMaxIter,b.parameters.SolverSecondTolerance);
    b.normalization.setZero();for(int k=0;k<size;++k)b.normalization(ids[k]/b.parameters.Sizej,ids[k]%b.parameters.Sizej)=solved.first(k);
  } else FuncSmoothN2(b.normalization,b.smoothness,b.parameters);
}
NormalSystem2D detail::linearize2D(const Problem2D& b,const detail::VariableLayout2D& layout) {
  const auto& p=b.parameters;const auto& ids=layout.ids;
  const int nv=ids.size(),np=3*(b.poses.rows()-1),dim=np+nv;
  const auto& index=layout.index;
  // Each bilinear residual couples only the 3x3 neighborhood around a cell.
  std::vector<std::array<double,9>> mm(nv);
  std::vector<Eigen::Triplet<double>> triplets;
  // Full CG assembly preserves both independently accumulated triangles,
  // including roundoff in weighted odometry terms and its stopping history.
  const bool compact=!p.ModeSparseSolver;
  NormalSystem2D result;result.storage=compact?SymmetricStorage::Lower:SymmetricStorage::Full;
  result.rhs=Eigen::VectorXd::Zero(dim);
  Eigen::MatrixXd gradient=FuncGradient(b.map,1.);
  Eigen::MatrixXd gv=gradient.leftCols(p.Sizej),gu=gradient.rightCols(p.Sizej);
  Eigen::Matrix<double,3,Eigen::Dynamic> cross(3,nv);
  std::vector<unsigned char> touched(nv,0);std::vector<int> active;
  const std::array<int,4> dx{0,1,0,1},dy{0,0,1,1};
  for(int frame=0;frame<b.poses.rows();++frame) {
    cross.setZero();std::fill(touched.begin(),touched.end(),0);active.clear();
    Eigen::Matrix3d pp=Eigen::Matrix3d::Zero();Eigen::Vector3d bp=Eigen::Vector3d::Zero();
    double c=std::cos(b.poses(frame,2)),s=std::sin(b.poses(frame,2));
    for(int j=0;j<b.labels[frame].size();++j) {
      double lx=b.samples[frame](2*j),ly=b.samples[frame](2*j+1);
      double x=(c*lx-s*ly+b.poses(frame,0)-p.OriginX)/p.Scale;
      double y=(s*lx+c*ly+b.poses(frame,1)-p.OriginY)/p.Scale;
      int u=std::floor(x),v=std::floor(y);
      if(u<0||v<0||u+1>=p.Sizej||v+1>=p.Sizei) throw std::runtime_error("Projected sample outside bilinear support");
      std::array<int,4> k{index[v*p.Sizej+u],index[v*p.Sizej+u+1],index[(v+1)*p.Sizej+u],index[(v+1)*p.Sizej+u+1]};
      if(*std::min_element(k.begin(),k.end())<0)continue;
      double a=x-u,d=y-v;std::array<double,4> w{(1-a)*(1-d),a*(1-d),(1-a)*d,a*d};
      double n=interpolate(b.normalization,u,v,w);
      if(!(n>0)||!std::isfinite(n)) throw std::runtime_error("Invalid normalization at observation");
      double e=interpolate(b.map,u,v,w)/n-b.labels[frame](j);
      const double gx=interpolate(gu,u,v,w)/n/p.Scale,gy=interpolate(gv,u,v,w)/n/p.Scale;
      Eigen::Vector3d jp(gx,gy,gx*(-s*lx-c*ly)+gy*(c*lx-s*ly));
      for(auto& value:w)value/=n;
      ++result.observations;result.cost.scan+=e*e;
      if(frame) {pp.noalias()+=jp*jp.transpose();bp.noalias()-=jp*e;}
      for(int q=0;q<4;++q) {
        result.rhs(np+k[q])-=w[q]*e;
        for(int z=0;z<4;++z)mm[k[q]][(dy[z]-dy[q]+1)*3+dx[z]-dx[q]+1]+=w[q]*w[z];
        if(frame) {cross.col(k[q]).noalias()+=jp*w[q];if(!touched[k[q]]){touched[k[q]]=1;active.push_back(k[q]);}}
      }
    }
    if(frame) {
      const int offset=3*(frame-1);result.rhs.segment<3>(offset)+=bp;
      for(int a=0;a<3;++a)for(int d=0;d<3;++d)if(!compact||d<=a)triplets.emplace_back(offset+a,offset+d,pp(a,d));
      for(int k:active)for(int a=0;a<3;++a) {double value=cross(a,k);if(value){if(!compact)triplets.emplace_back(offset+a,np+k,value);triplets.emplace_back(np+k,offset+a,value);}}
    }
  }
  for(int k=0;k<nv;++k) {
    int u=ids[k]%p.Sizej,v=ids[k]/p.Sizej;
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
      double value=mm[k][(dy+1)*3+dx+1];if(!value)continue;
      int x=u+dx,y=v+dy;if(x<0||x>=p.Sizej||y<0||y>=p.Sizei)throw std::runtime_error("Invalid map stencil");
      int other=index[y*p.Sizej+x];if(other<0)throw std::runtime_error("Invalid selected map stencil");
      if(!compact||k>=other)triplets.emplace_back(np+k,np+other,value);
    }
  }
  if(p.WeightO>0) for(int frame=1;frame<b.poses.rows();++frame) {
    Eigen::Matrix2d rt=Eigen::Rotation2Dd(-b.poses(frame-1,2)).toRotationMatrix();
    Eigen::Vector2d t=rt*(b.poses.row(frame).head<2>()-b.poses.row(frame-1).head<2>()).transpose();
    Eigen::Vector3d e(t.x()-b.odometry(frame,0),t.y()-b.odometry(frame,1),FuncWrap(b.poses(frame,2)-b.poses(frame-1,2)-b.odometry(frame,2)));
    Eigen::Matrix<double,3,6> j=Eigen::Matrix<double,3,6>::Zero();
    j.block<2,2>(0,0)=-rt;j.block<2,2>(0,3)=rt;j(0,2)=t.y();j(1,2)=-t.x();j(2,2)=-1;j(2,5)=1;
    Eigen::Vector3d weight(p.WeightO/(.05*.05),p.WeightO/(.05*.05),p.WeightO/(.02*.02));
    Eigen::Matrix<double,6,6> h=j.transpose()*weight.asDiagonal()*j;
    Eigen::Matrix<double,6,1> rhs=-j.transpose()*weight.asDiagonal()*e;
    result.cost.odometry+=e.dot(weight.asDiagonal()*e);
    for(int a=0;a<6;++a) {int ia=3*(frame-2)+a;if(ia<0)continue;result.rhs(ia)+=rhs(a);for(int d=0;d<6;++d){int id=3*(frame-2)+d;if(id>=0&&(!compact||ia>=id))triplets.emplace_back(ia,id,h(a,d));}}
  }
  Eigen::VectorXd field(nv);for(int k=0;k<nv;++k)field(k)=b.map(ids[k]/p.Sizej,ids[k]%p.Sizej);
  const double lambda=b.selected_variables.size()?p.MapSmoothingWeightSecond:p.MapSmoothingWeightFirst;
  Eigen::VectorXd smooth=lambda*(b.smoothness*field);result.rhs.tail(nv)-=smooth;result.cost.smoothness=field.dot(smooth);
  for(int c=0;c<b.smoothness.outerSize();++c)for(Eigen::SparseMatrix<double>::InnerIterator it(b.smoothness,c);it;++it)if(!compact||it.row()>=it.col())triplets.emplace_back(np+it.row(),np+it.col(),lambda*it.value());
  result.hessian.resize(dim,dim);result.hessian.setFromTriplets(triplets.begin(),triplets.end());
  if(!result.rhs.allFinite()||!std::isfinite(result.cost.total()))throw std::runtime_error("Nonfinite normal equations");
  return result;
}
void detail::refreshNormalization2D(Problem2D& b,const detail::VariableLayout2D& layout) {
  // Rebuild N from zero after every accepted pose update. No N pose derivative.
  b.normalization=integrateCounts2D(b.poses,b.samples,b.labels,b.parameters,b.parameters.NormalizationRound);
  detail::smoothNormalization2D(b,layout);
}
NormalSystem2D linearize2D(const Problem2D& b) {return detail::linearize2D(b,detail::variableLayout2D(b));}
void refreshNormalization2D(Problem2D& b) {detail::refreshNormalization2D(b,detail::variableLayout2D(b));}
void smoothNormalization2D(Problem2D& b) {detail::smoothNormalization2D(b,detail::variableLayout2D(b));}

}
