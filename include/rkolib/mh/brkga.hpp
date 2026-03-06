#pragma once

#include "rkolib/core/context.hpp"
#include "rkolib/core/data.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

/**
 * Method: BRKGA
 * Description: Search process of the Biased Random-Key Genetic Algorithm.
 * Features:
 * - Classification into Elite and Non-Elite sets.
 * - Introduction of Mutants (Random Immigrants) for diversity.
 * - Parametric Uniform Crossover (Biased towards Elite).
 * - Enhanced with Q-Learning for adaptive parameter control.
 */
void BRKGA(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver);

} // namespace rkolib::mh