#pragma once

#include "rkolib/core/context.hpp"
#include "rkolib/core/data.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

/**
 * Method: IPR
 * Description: Apply the Implicit Path-Relinking method with 6 trajectory strategies
 * (One-way, Back-and-Forth, Mixed, GRAPR, Truncated, External) and online Q-Learning
 * control over block size fraction (alpha) and Path-Relinking strategy.
 */
void IPR(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver);

} // namespace rkolib::mh