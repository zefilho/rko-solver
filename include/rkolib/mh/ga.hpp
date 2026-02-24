#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/solver.hpp"
#include "rkolib/core/context.hpp"

// Forward declaration
//struct IProblem; 

namespace rkolib::mh {

    /**
     * Method: GA
     * Description: Search process of the Genetic Algorithm.
     * Enhanced with Q-Learning for adaptive population size and operator rates.
     */
    void GA(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver);

} // namespace rkolib::mh