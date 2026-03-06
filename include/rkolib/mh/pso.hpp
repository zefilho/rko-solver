#pragma once

#include "rkolib/core/context.hpp"
#include "rkolib/core/data.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

/**
 * Method: PSO
 * Description: Search process of the Particle Swarm Optimization.
 * Enhanced with Q-Learning for adaptive population sizing and parameter
 * control.
 */
void PSO(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver);

} // namespace rkolib::mh