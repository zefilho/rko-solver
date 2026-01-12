#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/problem.hpp"

// Forward declaration
//struct TProblemData; 

namespace rkolib::mh {

    /**
     * Method: GA
     * Description: Search process of the Genetic Algorithm.
     * Enhanced with Q-Learning for adaptive population size and operator rates.
     */
    void GA(const rkolib::core::TRunData &runData, const rkolib::core::TProblemData &data);

} // namespace rkolib::mh