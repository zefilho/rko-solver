#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>

namespace rkolib::core {

struct TSol {
  std::vector<double> rk;
  double ofv = std::numeric_limits<double>::infinity();
  double best_time = 0.0;
  std::string nameMH;
  std::vector<double> objs;
  TSol() = default;
};

class IProblem {
public:
  virtual ~IProblem() = default;
  virtual void load(const std::string &filename) = 0;
  virtual void decode(TSol &sol) const = 0;
  virtual int getDimension() const = 0;
  virtual int getNumObjectives() const = 0;
};
} // namespace rkolib::core


class TemplateProblem : public rkolib::core::IProblem {
private:
  int nObj; // number of objectives
  int n; // number of variables for decoder

public:
  TemplateProblem() : nObj(1), n(0) {}
  ~TemplateProblem() override = default;

  void load(const std::string &nomeArquivo) override {
    // Define the loader code
  }

  // =======================================================
  // MÉTODO DECODE
  // =======================================================
  void decode(rkolib::core::TSol &s) const override {
    // Define the decoder code
    // s.rk has size n, and s.rk[i] is the value of the i-th random key
    // You need to set s.objs[j] to the value of the j-th objective
  }

  int getDimension() const override { return n; }
  int getNumObjectives() const override { return nObj; }
};

extern "C" {
  #ifdef _WIN32
    __declspec(dllexport) rkolib::core::IProblem *create_problem() { return new TemplateProblem(); }
    __declspec(dllexport) void destroy_problem(rkolib::core::IProblem *p) { delete p; }
  #else
    __attribute__((visibility("default"))) rkolib::core::IProblem *create_problem() { return new TemplateProblem(); }
    __attribute__((visibility("default"))) void destroy_problem(rkolib::core::IProblem *p) { delete p; }
  #endif
}