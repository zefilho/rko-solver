#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/problem.hpp"

// Forward declaration
//struct TProblemData; 

namespace rkolib::mh {

    /**
     * Method: IPR
     * Description: Apply the Implicit Path Relinking method.
     * Explores the trajectory between random elite solutions from the pool.
     */
    void IPR(const rkolib::core::TRunData &runData, const rkolib::core::TProblemData &data);

} // namespace rkolib::mh