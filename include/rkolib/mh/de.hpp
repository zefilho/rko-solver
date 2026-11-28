#pragma once

#include "rkolib/core/context.hpp"
#include "rkolib/core/data.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

/**
 * Method: DE
 * Description: Search process of the Differential Evolution metaheuristic.
 * Enhanced with Q-Learning for adaptive population size, crossover (Cr), and scaling factor (F).
 */
void DE(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver);

} // namespace rkolib::mh

