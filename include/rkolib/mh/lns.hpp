#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/problem.hpp"

// Forward declaration
//struct TProblemData; 

namespace rkolib::mh {

    /**s
     * Method: LNS
     * Description: Search process of the Large Neighborhood Search.
     * Uses a specific Destruction/Repair mechanism based on Farey Sequences.
     * Enhanced with Q-Learning.
     */
    void LNS(const rkolib::core::TRunData &runData, const rkolib::core::TProblemData &data);

} // namespace rkolib::mh