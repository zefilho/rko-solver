#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/iproblem.hpp"
#include "rkolib/core/context.hpp"

//Forward declaration
//struct IProblem; 

namespace rkolib::mh {

    /**
     * Method: BRKGA_CS
     * Description: The evolutionary process of the BRKGA-CS (Biased Random-Key Genetic Algorithm with Community Structure).
     * Uses Label Propagation to maintain population diversity.
     */
    void BRKGA_CS(const rkolib::core::TRunData &runData, const rkolib::core::IProblem &problem);

} // namespace rkolib::mh