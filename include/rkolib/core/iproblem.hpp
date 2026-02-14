#pragma once
#include "rkolib/core/data.hpp"

namespace rkolib::core {

    // Interface Abstrata
    class IProblem {
        public:
            // Destrutor virtual é OBRIGATÓRIO para polimorfismo seguro
            virtual ~IProblem() = default;

            // Métodos puramente virtuais (= 0) que o usuário DEVE implementar
            virtual void load(const std::string& filename) = 0;
            
            // O Decoder agora é um método membro
            virtual double evaluate(const TSol& s) const = 0;

            virtual int getDimension() const = 0;
        };
        
    // Declaração de uma "Fábrica" que o usuário vai implementar
    std::shared_ptr<IProblem> createProblem();

}