#pragma once
#include "rkolib/core/data.hpp"

namespace rkolib::core {

    // Interface Abstrata
    class IProblem {
        public:
            virtual ~IProblem() = default;

            // Loads data from file
            virtual void load(const std::string& filename) = 0;

            //Calculates raw objectives and populates s.objs[]
            // Does NOT apply weights here (that's Solver's job)
            virtual void evaluate(TSol& s) const = 0;

            // Getters essenciais
            virtual int getDimension() const = 0;     // Number of variables
            virtual int getNumObjectives() const = 0; // 1 = Mono, >1 = Multi
    };
        
    // Declaration of a "factory" that user will implement
    std::shared_ptr<IProblem> createProblem();

}