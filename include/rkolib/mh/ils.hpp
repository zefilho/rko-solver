#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/solver.hpp"
#include "rkolib/core/context.hpp"

// Forward declaration
//struct IProblem; 

namespace rkolib::mh {

    /**
     * Method: ILS
     * Description: Search process of the Iterated Local Search.
     * Enhanced with Q-Learning for adaptive perturbation control.
     */
    void ILS(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver);

} // namespace rkolib::mh