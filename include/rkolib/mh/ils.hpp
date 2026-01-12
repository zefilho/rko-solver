#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/problem.hpp"

// Forward declaration
//struct TProblemData; 

namespace rkolib::mh {

    /**
     * Method: ILS
     * Description: Search process of the Iterated Local Search.
     * Enhanced with Q-Learning for adaptive perturbation control.
     */
    void ILS(const rkolib::core::TRunData &runData, const rkolib::core::TProblemData &data);

} // namespace rkolib::mh