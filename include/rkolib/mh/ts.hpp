#pragma once

#include "rkolib/core/context.hpp"
#include "rkolib/core/data.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

/**
 * Method: TS
 * Description: Generic Random-Key Tabu Search for the RKO framework.
 * Enhanced with Q-Learning for adaptive tabu tenure, neighborhood sampling, and diversification.
 */
void TS(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver);

} // namespace rkolib::mh

