#pragma once

#include "rkolib/core/context.hpp"
#include "rkolib/core/data.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

/**
 * Method: IPR
 * Description: Apply the Implicit Path Relinking method.
 * Explores the trajectory between random elite solutions from the pool.
 */
void IPR(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver);

} // namespace rkolib::mh