#pragma once

#include "rkolib/core/context.hpp"
#include "rkolib/core/data.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

/**
 * Method: VNS
 * Description: Search process of the Variable Neighborhood Search (VNS)
 * Enhanced with Q-Learning for parameter tuning.
 */
void VNS(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver);

} // namespace rkolib::mh