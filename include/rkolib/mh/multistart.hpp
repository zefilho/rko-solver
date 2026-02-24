#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/solver.hpp"
#include "rkolib/core/context.hpp"

// Forward declaration
//struct IProblem; 

namespace rkolib::mh {

    /**
     * Method: MultiStart
     * Description: A simple metaheuristic that iteratively generates random solutions 
     * and keeps the best one found.
     */
    void MultiStart(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver);

} // namespace rkolib::mh