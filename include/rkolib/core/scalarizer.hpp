#pragma once

#include "rkolib/core/data.hpp"

namespace rkolib::core {

// Strategy Interface
class IScalarizer {
public:
  virtual ~IScalarizer() = default;
  virtual double scalarize(const TSol &s, const std::vector<double> &lambda,
                           const std::vector<double> &idealPoint,
                           const std::vector<double> &nadirPoint) = 0;
  virtual std::string getName() const = 0;
};

// Strategy 1: Tchebycheff (Minimize the distance to the ideal)
class TchebycheffScalarizer : public IScalarizer {
public:
  double scalarize(const TSol &s, const std::vector<double> &lambda,
                   const std::vector<double> &idealPoint, 
                   const std::vector<double> &nadirPoint) override {
    if (s.objs.empty())
      return 1e15; // Valor ruim

    double max_dist = -1.0;
    size_t nObj = s.objs.size();

    // Pesos padrão se vetor vazio
    bool use_default = (lambda.size() != nObj);
    double default_w = 1.0 / (double)nObj;

    for (size_t k = 0; k < nObj; ++k) {
      double w = use_default ? default_w : lambda[k];
      // // | z* - f(x) |
      // double diff = std::abs(idealPoint[k] - s.objs[k]);
      // double val = w * diff;
      // if (val > max_dist)
      //   max_dist = val;
      double diff = std::abs(idealPoint[k] - s.objs[k]);
      double range = std::abs(nadirPoint[k] - idealPoint[k]);
      
      // Evita divisão por zero se Ideal e Nadir forem iguais
      if (range < 1e-9) range = 1e-9; 

      // Normalized distance formula
      double val = w * (diff / range);
      
      if (val > max_dist) {
        max_dist = val;
      }
    }
    return max_dist;
  }
  std::string getName() const override { return "Tchebycheff"; }
};

// Strategy 2: Weighted Sum
class WeightedSumScalarizer : public IScalarizer {
public:
  double scalarize(const TSol &s, const std::vector<double> &lambda,
                   const std::vector<double> & /*ideal*/,
                   const std::vector<double> & /*nadir*/) override {
    if (s.objs.empty())
      return 1e15;
    double sum = 0.0;
    size_t nObj = s.objs.size();
    bool use_default = (lambda.size() != nObj);
    double default_w = 1.0 / (double)nObj;

    for (size_t k = 0; k < nObj; ++k) {
      double w = use_default ? default_w : lambda[k];
      // Invert sign if it's profit maximization -> cost minimization
      sum += s.objs[k] * w;
    }
    return sum;
  }
  std::string getName() const override { return "WeightedSum"; }
};

// Fairness Methods

class GiniScalarizer : public IScalarizer {
public:
    double scalarize(const TSol &s, const std::vector<double> &/*lambda*/,
                     const std::vector<double> &/*ideal*/,
                     const std::vector<double> &/*nadir*/) override {
        if (s.objs.empty()) return 1e15;

        double sum_diffs = 0.0;
        double sum = 0.0;
        int n = s.objs.size();

        for (int i = 0; i < n; ++i) {
            sum += s.objs[i];
            for (int j = 0; j < n; ++j) {
                sum_diffs += std::abs(s.objs[i] - s.objs[j]);
            }
        }

        if (sum < 1e-9) return 0.0; // All resources are zero

        double mean = sum / n;
        double gini = sum_diffs / (2.0 * n * n * mean);
        
        return gini; // The engine will minimize the inequality
    }
    
    std::string getName() const override { return "Gini_Coefficient"; }
};


} // namespace rkolib::core