#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/iproblem.hpp"
#include "rkolib/core/context.hpp"

// Forward declaration
//struct IProblem; 

namespace rkolib::mh {

    /**
     * Method: GRASP
     * Description: Metaheuristic Greedy Randomized Adaptive Search Procedure.
     * Enhanced with Q-Learning (known as C-GRASP when combined with Continuous Line Search).
     */
    void GRASP(const rkolib::core::TRunData &runData, const rkolib::core::IProblem &problem);

} // namespace rkolib::mh