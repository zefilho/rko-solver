#pragma once

#include "rkolib/core/context.hpp"
#include "rkolib/core/data.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

/**
 * Method: BRKGA_CS
 * Description: The evolutionary process of the BRKGA-CS (Biased Random-Key
 * Genetic Algorithm with Community Structure). Uses Label Propagation to
 * maintain population diversity.
 */
void BRKGA_CS(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver);

} // namespace rkolib::mh