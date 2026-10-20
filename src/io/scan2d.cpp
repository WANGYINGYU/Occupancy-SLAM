#include "scan2d.hpp"
#include "cli.h"
namespace occupancy {
void readEndpoints2D(const fs::path& path, int frames, const ParamStruct& p,
                     uint64_t budget, std::vector<Eigen::ArrayXd>& samples,
                     std::vector<Eigen::ArrayXd>& labels) {
  auto points=matrix(path,3); std::vector<uint64_t> counts(frames,0); uint64_t total=0;
  int previous=-1;
  for(Eigen::Index i=0;i<points.rows();++i) {
    double f=points(i,0);
    if(f!=std::floor(f)||f<0||f>=frames||f<previous) throw std::runtime_error("Endpoint frame indices must be ordered integers in [0, frames)");
    int frame=static_cast<int>(f);previous=frame;
    double range=points.row(i).tail<2>().norm();
    if(range<=p.MinRange||range>=p.MaxRange) continue;
    double n=std::floor(range/p.Scale);
    if(n>budget-total) throw std::runtime_error("Endpoint observation budget exceeded before allocation");
    counts[frame]+=static_cast<uint64_t>(n);total+=static_cast<uint64_t>(n);
  }
  samples.resize(frames);labels.resize(frames);
  for(int i=0;i<frames;++i) {
    if(!counts[i]) throw std::runtime_error("Endpoint scan has no valid observations at frame "+std::to_string(i));
    samples[i].resize(2*counts[i]);labels[i].resize(counts[i]);
  }
  std::vector<uint64_t> offset(frames,0);
  for(Eigen::Index i=0;i<points.rows();++i) {
    int frame=static_cast<int>(points(i,0));double range=points.row(i).tail<2>().norm();
    if(range<=p.MinRange||range>=p.MaxRange) continue;
    int n=static_cast<int>(std::floor(range/p.Scale));
    for(int k=0;k<n;++k) {
      const auto index=offset[frame]++;
      for(int axis=0;axis<2;++axis) samples[frame](2*index+axis)=k==0?points(i,axis+1):(points(i,axis+1)/n)*(n-k);
      labels[frame](index)=k==0?p.ValOccupied:p.ValFree;
    }
  }
}
}

namespace occupancy {
void readSamples2D(const fs::path& path,int frames,int start_frame,uint64_t budget,
                   std::vector<Eigen::ArrayXd>& samples,std::vector<Eigen::ArrayXd>& labels) {
  // Little-endian uint64 frame count; per frame: uint64 n, interleaved XY f64, labels f64.
  const uint16_t endian=1;
  if(*reinterpret_cast<const unsigned char*>(&endian)!=1) throw std::runtime_error("Sample binary requires a little-endian host");
  std::ifstream f(path,std::ios::binary); if(!f) throw std::runtime_error("Cannot open samples");
  auto read=[&](void* to,size_t bytes){if(!f.read(static_cast<char*>(to),bytes)) throw std::runtime_error("Truncated sample binary");};
  uint64_t count=0,total=0;read(&count,8);
  if(count!=static_cast<uint64_t>(frames+start_frame)) throw std::runtime_error("Sample frame count mismatch");
  for(int i=0;i<start_frame;++i){uint64_t n=0;read(&n,8);if(n>100000000)throw std::runtime_error("Invalid skipped sample count");f.seekg(24*n,std::ios::cur);}
  samples.resize(frames);labels.resize(frames);
  for(int i=0;i<frames;++i) {
    uint64_t n=0;read(&n,8);
    if(!n||n>budget-total) throw std::runtime_error("Sample observation budget exceeded");
    total+=n;samples[i].resize(2*n);labels[i].resize(n);
    read(samples[i].data(),16*n);read(labels[i].data(),8*n);
    if(!samples[i].allFinite()||!labels[i].allFinite()) throw std::runtime_error("Nonfinite samples");
  }
  if(f.peek()!=std::char_traits<char>::eof()) throw std::runtime_error("Trailing data in sample binary");
}
}
