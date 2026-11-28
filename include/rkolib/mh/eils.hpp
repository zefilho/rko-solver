#pragma once

#include "rkolib/core/context.hpp"
#include "rkolib/core/data.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

/**
 * Method: EILS
 * Description: Search process of the Enhanced Iterated Local Search (E-ILS).
 * Controls perturbation intensity across discrete levels with cycle-based Q-Learning.
 */
void EILS(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver);

} // namespace rkolib::mh

