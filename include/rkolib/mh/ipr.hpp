#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/iproblem.hpp"
#include "rkolib/core/context.hpp"

// Forward declaration
//struct IProblem; 

namespace rkolib::mh {

    /**
     * Method: IPR
     * Description: Apply the Implicit Path Relinking method.
     * Explores the trajectory between random elite solutions from the pool.
     */
    void IPR(const rkolib::core::TRunData &runData, const rkolib::core::IProblem &problem);

} // namespace rkolib::mh