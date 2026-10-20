#pragma once
#include <Eigen/Core>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <set>
#include <stdexcept>
#include <cmath>
#include <iomanip>
namespace fs = std::filesystem;
struct Args {
  std::map<std::string,std::string> values;
  Args(int argc, char** argv, const std::set<std::string>& allowed) {
    for(int i=1;i<argc;i+=2) {
      std::string k=argv[i];
      if(!allowed.count(k) || i+1==argc || values.count(k)) throw std::runtime_error("Unknown, duplicate or incomplete option: "+k);
      values[k]=argv[i+1];
    }
  }
  std::string get(std::string k, std::string fallback="") const {
    auto it=values.find(k); if(it!=values.end()) return it->second;
    if(fallback.empty()) throw std::runtime_error("Missing option: "+k);
    return fallback;
  }
  double num(std::string k, double d) const {
    auto it=values.find(k); if(it==values.end()) return d;
    size_t pos=0; double x=std::stod(it->second,&pos);
    if(pos!=it->second.size() || !std::isfinite(x)) throw std::runtime_error("Invalid numeric option: "+k);
    return x;
  }
  int integer(std::string k,int d) const {
    double x=num(k,d); if(x!=std::floor(x) || x<0 || x>100000000) throw std::runtime_error("Invalid integer option: "+k);
    return static_cast<int>(x);
  }
};
inline std::ofstream output(const fs::path& p) {
  std::ofstream f(p); if(!f) throw std::runtime_error("Cannot write: "+p.string());
  f.exceptions(std::ios::failbit|std::ios::badbit); f<<std::setprecision(17); return f;
}
inline Eigen::MatrixXd matrix(const fs::path& p, int columns=0) {
  std::ifstream f(p); if(!f) throw std::runtime_error("Cannot read: "+p.string());
  std::vector<std::vector<double>> rows; std::string line;
  while(std::getline(f,line)) {
    line=line.substr(0,line.find('#')); std::istringstream ss(line); std::string token; std::vector<double> row;
    while(ss>>token) {size_t used; double v=std::stod(token,&used); if(used!=token.size()||!std::isfinite(v)) throw std::runtime_error("Nonfinite/invalid number in "+p.string()); row.push_back(v);}
    if(row.empty()) continue;
    if(columns==0) columns=row.size();
    if(static_cast<int>(row.size())!=columns) throw std::runtime_error("Wrong column count in "+p.string());
    rows.push_back(std::move(row));
  }
  if(rows.empty()) throw std::runtime_error("Empty input: "+p.string());
  Eigen::MatrixXd m(rows.size(),columns);
  for(size_t i=0;i<rows.size();++i) for(int j=0;j<columns;++j) m(i,j)=rows[i][j];
  return m;
}
inline fs::path newOutput(const Args& a) {
  fs::path p=fs::absolute(a.get("--output"));
  if(fs::exists(p) && !fs::is_empty(p)) throw std::runtime_error("Output directory must be empty: "+p.string());
  fs::create_directories(p); return p;
}

// A completion state accompanies partial native outputs, including exceptions.
struct RunState {
  fs::path directory; bool completed=false;
  explicit RunState(fs::path p):directory(std::move(p)) { output(directory/"run_status.json")<<"{\"status\":\"running\"}\n"; }
  void finish(const std::string& reason) {
    output(directory/"run_status.json")<<"{\"status\":\"completed\",\"solver_stop_reason\":\""<<reason<<"\"}\n";
    completed=true;
  }
  ~RunState() noexcept {if(!completed)try{output(directory/"run_status.json")<<"{\"status\":\"failed\"}\n";}catch(...) {}}
};
