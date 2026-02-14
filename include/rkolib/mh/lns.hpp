#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/iproblem.hpp"
#include "rkolib/core/context.hpp"

// Forward declaration
//struct IProblem; 

namespace rkolib::mh {

    /**s
     * Method: LNS
     * Description: Search process of the Large Neighborhood Search.
     * Uses a specific Destruction/Repair mechanism based on Farey Sequences.
     * Enhanced with Q-Learning.
     */
    void LNS(const rkolib::core::TRunData &runData, const rkolib::core::IProblem &problem);

} // namespace rkolib::mh