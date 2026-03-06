#pragma once

#include "rkolib/core/data.hpp"

namespace rkolib::core {

// Strategy Interface
class IScalarizer {
public:
  virtual ~IScalarizer() = default;
  virtual double scalarize(const TSol &s, const std::vector<double> &lambda,
                           const std::vector<double> &idealPoint) = 0;
  virtual std::string getName() const = 0;
};

// Strategy 1: Tchebycheff (Minimize the distance to the ideal)
class TchebycheffScalarizer : public IScalarizer {
public:
  double scalarize(const TSol &s, const std::vector<double> &lambda,
                   const std::vector<double> &idealPoint) override {
    if (s.objs.empty())
      return 1e15; // Valor ruim

    double max_dist = -1.0;
    size_t nObj = s.objs.size();

    // Pesos padrão se vetor vazio
    bool use_default = (lambda.size() != nObj);
    double default_w = 1.0 / (double)nObj;

    for (size_t k = 0; k < nObj; ++k) {
      double w = use_default ? default_w : lambda[k];
      // | z* - f(x) |
      double diff = std::abs(idealPoint[k] - s.objs[k]);
      double val = w * diff;
      if (val > max_dist)
        max_dist = val;
    }
    return max_dist;
  }
  std::string getName() const override { return "Tchebycheff"; }
};

// Strategy 2: Weighted Sum
class WeightedSumScalarizer : public IScalarizer {
public:
  double scalarize(const TSol &s, const std::vector<double> &lambda,
                   const std::vector<double> & /*ideal*/) override {
    if (s.objs.empty())
      return 1e15;
    double sum = 0.0;
    size_t nObj = s.objs.size();
    bool use_default = (lambda.size() != nObj);
    double default_w = 1.0 / (double)nObj;

    for (size_t k = 0; k < nObj; ++k) {
      double w = use_default ? default_w : lambda[k];
      // Invert sign if it's profit maximization -> cost minimization
      sum += s.objs[k] * w * -1.0;
    }
    return sum;
  }
  std::string getName() const override { return "WeightedSum"; }
};
} // namespace rkolib::core