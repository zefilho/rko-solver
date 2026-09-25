#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>

#include "rkolib/core/problem.hpp"

class TemplateProblem : public rkolib::core::IProblem {
private:
  int nObj; // number of objectives
  int n; // number of variables for decoder

public:
  TemplateProblem() : nObj(1), n(0) {}
  ~TemplateProblem() override = default;

  void load(const std::string &nomeArquivo) override {
    (void)nomeArquivo;
    // Define the loader code
  }

  // =======================================================
  // MÉTODO DECODE
  // =======================================================
  void decode(rkolib::core::TSol &s) const override {
    (void)s;
    // Define the decoder code
    // s.rk has size n, and s.rk[i] is the value of the i-th random key
    // You need to set s.objs[j] to the value of the j-th objective
  }

  int getDimension() const override { return n; }
  int getNumObjectives() const override { return nObj; }
};

REGISTER_RKO_PROBLEM(TemplateProblem)