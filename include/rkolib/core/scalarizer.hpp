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
class NormalizedWeightedTchebycheff : public IScalarizer {
public:
  double scalarize(const TSol &s, const std::vector<double> &lambda,
                   const std::vector<double> &idealPoint, 
                   const std::vector<double> &nadirPoint) override {
    if (s.objs.empty()){
      //fprintf(stderr, "[TRACE CORE] Fim de scalarize. OFV:%f %d \n", s.ofv, s.objs.size());
      return 1e15; // Valor ruim
    }

    for (double w : lambda) {
      if (w < 0.0) {
        throw std::invalid_argument("Tchebycheff weights must be non-negative");
      }
    }

    double max_dist = -1.0;
    size_t nObj = s.objs.size();

    if (!lambda.empty() && lambda.size() != nObj) {
      throw std::invalid_argument("Lambda dimension does not match number of objectives");
    } 

    // Pesos padrão se vetor vazio
    //bool use_default = (lambda.size() != nObj);
    bool use_default = lambda.empty();
    double default_w = 1.0 / (double)nObj;
    double sum_dist = 0.0;

    for (size_t k = 0; k < nObj; ++k) {
      double w = use_default ? default_w : lambda[k];
      // // | z* - f(x) |
      // double diff = std::abs(idealPoint[k] - s.objs[k]);
      // double val = w * diff;
      // if (val > max_dist)
      //   max_dist = val;
      //double diff = std::abs(idealPoint[k] - s.objs[k]);
      double diff = std::max(0.0, s.objs[k] - idealPoint[k]);
      double range = std::abs(nadirPoint[k] - idealPoint[k]);
      
      // Evita divisão por zero se Ideal e Nadir forem iguais
      if (range < 1e-9) range = 1e-9; 

      // Normalized distance formula
      double norm_dist = diff / range;
      double val = w * norm_dist;

      // std::cout << "[TRACE CORE] Tchebycheff Scalarizer Avaliacao" << std::endl;
      // std::cout << "w: " << w << " diff: " << diff << " range: " << range << " norm_dist: " << norm_dist << " val: " << val << std::endl;
      sum_dist += norm_dist;

      
      if (val > max_dist) {
        max_dist = val;
      }
    }

    return max_dist + (0.0001 * sum_dist);
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

// ============================================================================
// Strategy: Lexicographic Min-Max (Assume convenção canônica: tudo é Minimização)
// ============================================================================
class LexicographicMinMaxScalarizer : public IScalarizer {
private:
  double decay_factor_;

public:
  explicit LexicographicMinMaxScalarizer(double decay_factor = 0.05)
      : decay_factor_(decay_factor) {
    if (decay_factor_ <= 0.0 || decay_factor_ >= 1.0) {
      throw std::invalid_argument("Decay factor must be strictly in (0, 1)");
    }
  }

  double scalarize(const TSol &s, const std::vector<double> &lambda,
                   const std::vector<double> &idealPoint,
                   const std::vector<double> &nadirPoint) override {
    if (s.objs.empty()) {
      return 1e15;
    }

    const size_t nObj = s.objs.size();

    if (!lambda.empty() && lambda.size() != nObj) {
      throw std::invalid_argument("Lambda dimension does not match number of objectives");
    }

    const bool use_default_w = lambda.empty();
    const bool has_bounds = (idealPoint.size() == nObj) && (nadirPoint.size() == nObj);

    // Em problemas com <= 16 objetivos, evita alocação no heap se desejar otimizar depois,
    // mas std::vector mantém compatibilidade geral.
    std::vector<double> regret_objs(nObj);

    for (size_t k = 0; k < nObj; ++k) {
      const double w = use_default_w ? 1.0 : lambda[k];
      if (w < 0.0) {
        throw std::invalid_argument("Lexicographic Min-Max weights must be non-negative");
      }

      if (has_bounds) {
        // Na convenção de minimização: idealPoint[k] <= s.objs[k] <= nadirPoint[k]
        double range = nadirPoint[k] - idealPoint[k];
        if (range < 1e-9) range = 1e-9;

        // Usa diferença com sinal em vez de std::abs para não penalizar soluções
        // que eventualmente superem um idealPoint estimado (s.objs[k] < idealPoint[k])
        double norm_dist = std::max(0.0, (s.objs[k] - idealPoint[k]) / range);
        regret_objs[k] = w * norm_dist;
      } else {
        regret_objs[k] = w * s.objs[k];
      }
    }

    // Ordena do PIOR desvio (maior valor) para o MELHOR desvio (menor valor)
    std::sort(regret_objs.begin(), regret_objs.end(), std::greater<double>());

    // Escalarização hierárquica OWA + termo de regularização de Pareto
    double lex_score = regret_objs[0];
    double hierarchy_weight = decay_factor_;
    double tail_sum = 0.0;

    for (size_t k = 1; k < nObj; ++k) {
      if (hierarchy_weight > 1e-13) {
        lex_score += hierarchy_weight * regret_objs[k];
        hierarchy_weight *= decay_factor_;
      }
      tail_sum += regret_objs[k];
    }

    return lex_score + (1e-6 * (tail_sum / static_cast<double>(nObj)));
  }

  std::string getName() const override { return "Lexicographic_MinMax"; }
};


} // namespace rkolib::core