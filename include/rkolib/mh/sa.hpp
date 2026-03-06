#pragma once

#include "rkolib/core/context.hpp"
#include "rkolib/core/data.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

/**
 * Method: SA
 * Description: Search process of the Simulated Annealing (SA).
 * Enhanced with Q-Learning for adaptive parameter control.
 */
void SA(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver);

} // namespace rkolib::mh