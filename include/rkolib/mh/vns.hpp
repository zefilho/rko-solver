#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/iproblem.hpp"
#include "rkolib/core/context.hpp"

// Forward declaration para evitar include circular se necessário, 
// mas aqui precisamos da definição completa de IProblem no .cpp
//struct IProblem; 

namespace rkolib::mh {

    /**
     * Method: VNS
     * Description: Search process of the Variable Neighborhood Search (VNS)
     * Enhanced with Q-Learning for parameter tuning.
     */
    void VNS(const rkolib::core::TRunData &runData, const rkolib::core::IProblem &problem);

} // namespace rkolib::mh