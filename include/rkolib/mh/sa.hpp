#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/problem.hpp"

// Forward declaration para evitar include circular se necessário, 
// mas aqui precisamos da definição completa de TProblemData no .cpp
//struct TProblemData; 

namespace rkolib::mh {

    /**
     * Method: SA
     * Description: Search process of the Simulated Annealing (SA).
     * Enhanced with Q-Learning for adaptive parameter control.
     */
    void SA(const rkolib::core::TRunData &runData, const rkolib::core::TProblemData &data);

} // namespace rkolib::mh