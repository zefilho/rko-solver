#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/problem.hpp"

// Forward declaration
//struct TProblemData; 

namespace rkolib::mh {

    /**
     * Method: MultiStart
     * Description: A simple metaheuristic that iteratively generates random solutions 
     * and keeps the best one found.
     */
    void MultiStart(const rkolib::core::TRunData &runData, const rkolib::core::TProblemData &data);

} // namespace rkolib::mh