#pragma once

#include "rkolib/core/data.hpp"

namespace rkolib::core {

    // Interface da Estratégia
    class IScalarizer {
    public:
        virtual ~IScalarizer() = default;
        virtual double scalarize(const TSol& s, 
                                 const std::vector<double>& lambda,
                                 const std::vector<double>& idealPoint) = 0;
        virtual std::string getName() const = 0;
    };

    // Estratégia 1: Tchebycheff (Minimizar a distância ao ideal)
    class TchebycheffScalarizer : public IScalarizer {
    public:
        double scalarize(const TSol& s, 
                         const std::vector<double>& lambda,
                         const std::vector<double>& idealPoint) override 
        {
            if (s.objs.empty()) return 1e15; // Valor ruim
            
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
                if (val > max_dist) max_dist = val;
            }
            return max_dist; 
        }
        std::string getName() const override { return "Tchebycheff"; }
    };

    // Estratégia 2: Soma Ponderada
    class WeightedSumScalarizer : public IScalarizer {
    public:
        double scalarize(const TSol& s, 
                         const std::vector<double>& lambda,
                         const std::vector<double>& /*ideal*/) override 
        {
            if (s.objs.empty()) return 1e15;
            double sum = 0.0;
            size_t nObj = s.objs.size();
            bool use_default = (lambda.size() != nObj);
            double default_w = 1.0 / (double)nObj;

            for (size_t k = 0; k < nObj; ++k) {
                double w = use_default ? default_w : lambda[k];
                // Inverte sinal se for maximização de lucro -> minimização de custo
                sum += s.objs[k] * w * -1.0; 
            }
            return sum;
        }
        std::string getName() const override { return "WeightedSum"; }
    };
}