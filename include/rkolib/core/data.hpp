#pragma once // Substitui o #ifndef antigo

#include "rkolib/core/common.hpp"

namespace rkolib::core {

    //--------------------------------------------------------------------------
    // Struct: TSol
    // Description: Represents a solution within the Random-Key Optimization context
    //--------------------------------------------------------------------------
    struct TSol
    {
        std::vector<double> rk;                        // Random-key vector (decoder input)
        double ofv = std::numeric_limits<double>::infinity(); // Objective function value
        double best_time = 0.0;                        // Time to find this specific solution
        std::string nameMH;                            // Name of the metaheuristic/algorithm
        
        // Construtor padrão para garantir inicialização limpa
        TSol() = default;
    };

    //--------------------------------------------------------------------------
    // Struct: TRunData
    // Description: Configuration variables for the search process
    //--------------------------------------------------------------------------
    struct TRunData
    {
        int strategy;                           // define the local search strategy (1 = first improvement; 2 = best improvement)
        int control;                            // define the control parameters (0 - offline; 1 - online)
        int MAXTIME;                            // define the maximum running time (stop condiction)
        int MAXRUNS;                            // maximum number of runs of the method
        int debug;                              // define the run mode (0 - save results in files; 1 - print results in screen)
        float restart;                          // define the restart strategy (0 - without restart; 1 - with restart)
        int sizePool;                           // define the size of the elite pool solutions
    };

    //--------------------------------------------------------------------------
    // Struct: TQ
    // Description: Represents an entry in the Q-Learning quality matrix
    //--------------------------------------------------------------------------
    struct TQ
    {
        int S = 0;          // State ID
        double pVar = 0.0;  // Value associated with the state parameter
        double q = 0.0;     // Q-value (learned quality)
        int k = 0;          // Number of times this state was visited
        int kImp = 0;       // Number of times it led to improvement
    };

    //--------------------------------------------------------------------------
    // Struct: TState
    // Description: Represents a full State in the Markov Decision Process (MDP)
    //--------------------------------------------------------------------------
    struct TState
    {
        int label;                 // ID of the state
        std::vector<double> par;        // Vector of parameters defining the state
        double ci;                // Immediate cost/reward of state i
        int numN;                   // Visit counter
        
        std::vector<int> Ai;            // Available actions (indices) from this state
        std::vector<double> Qa;         // Q(s,a) values corresponding to actions Ai
        
        double sumQ;              // Sum of Q values (helper for probabilities)
        double maxQ; // Max Q value
        
        int minA;                  // Index of action with min Q
        int maxA;                  // Index of action with max Q
    };

} // namespace rkolib::core