#pragma once

#include "rkolib/core/context.hpp"
#include "rkolib/core/data.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

/**
 * Method: ACO
 * Description: Search process of the Ant Colony Optimization (ACOR) metaheuristic.
 * Enhanced with Q-Learning for adaptive archive size, ant count, q, and xi parameters.
 */
void ACO(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver);

} // namespace rkolib::mh

