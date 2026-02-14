#include "rkolib/mh/sa.hpp"

// Dependências internas
#include "rkolib/core/method.hpp"
#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/iproblem.hpp" // Para definição completa de IProblem e problem.getDimension()

namespace rkolib::mh {

    void SA(const rkolib::core::TRunData &runData, const rkolib::core::IProblem &problem)
    {
        using namespace rkolib::core; // Facilita o uso de TSol, TState, funções auxiliares

        const char* method = "SA";
        TSol s;                         // current solution
        TSol sViz;                      // neighbor solution
        TSol sBest;                     // best solution of SA

        double delta = 0;               // difference between solutions
        double bestOFV;                 // value of the best solution in the current iteration

        double T0 = 0;                  // initial temperature
        double T = 0;                   // current temperature
        double alphaSA = 0;             // cool rate
        int SAmax = 0;                  // number of iterations in a temperature T
        int IterT = 0;                  // iteracao corrente
        float betaMin = 0;              // minimum perturbation
        float betaMax = 0;              // maximum perturbation
        int reanneling = 0;             // reanneling flag
        int improv = 0;                 // improvement flag
        float currentTime = 0;          // computational time of the search process

        double start_timeMH = get_time_in_seconds();    // start computational time
        double end_timeMH = get_time_in_seconds();      // end computational time

        std::vector<int> RKorder(problem.getDimension());    // define a order for the neighors
        std::iota(RKorder.begin(), RKorder.end(), 0);

        // ---------------------------------------------------------------------
        // Q-Learning Parameters
        // ---------------------------------------------------------------------
        std::vector<TState> S;              // finite state space
        int numPar = 0;                     // number of parameters
        int numStates = 0;                  // number of states
        int iCurr = 0;                      // current (initial) state
        double epsilon = 0;                 // greed choice possibility
        double lf = 0;                      // learning factor
        double df = 0;                      // discount factor
        double R = 0;                       // reward
        
        // std::vector <std::vector <TQ> > Q; // (Nota: Não utilizado no original)
        // std::vector<int> ai;               // (Nota: Não utilizado no original)
        
        float epsilon_max = 1.0;            // maximum epsilon 
        float epsilon_min = 0.1;            // minimum epsilon
        int Ti = 1;                         // number of epochs performed
        int restartEpsilon = 1;             // number of restart epsilon
        int st = 0;                         // current state
        int at = 0;                         // current action

        // ** read file with parameter values
        numPar = 5; // SA usually has more params than VNS
        std::vector<std::vector<double>> parameters;
        parameters.resize(numPar);

        readParameters(method, runData.control, parameters, numPar);

        // offline control
        if (runData.control == 0){
            // define the parameters of the SA
            // Verificações de segurança para evitar crash se arquivo estiver vazio
            if (!parameters[0].empty()) SAmax   = (int)parameters[0][0];
            if (!parameters[1].empty()) alphaSA = parameters[1][0];
            if (!parameters[2].empty()) betaMin = parameters[2][0];
            if (!parameters[3].empty()) betaMax = parameters[3][0];
            if (!parameters[4].empty()) T0      = parameters[4][0];
        }

        // online control
        else 
        {
            // Q-Learning 
            if (runData.control == 1){
                // create possible states of the Markov chain
                CreateStates(parameters, numStates, numPar, S);

                // number of restart epsilon
                restartEpsilon = 1;  

                // maximum epsilon  
                epsilon_max = 1.0;  

                // current state
                iCurr = irandomico(0, numStates - 1);

                // define the initial parameters of the SA based on State
                // T0 inicial fixo no código original, mas sobrescrito logo abaixo pelo parametro
                // T0 = 1000000; 
                if (!S.empty()) {
                    SAmax   = (int)S[iCurr].par[0];
                    alphaSA = S[iCurr].par[1];
                    betaMin = S[iCurr].par[2];
                    betaMax = S[iCurr].par[3];
                    T0      = S[iCurr].par[4];
                }
            }
        }

        // ---------------------------------------------------------------------
        // Initialization
        // ---------------------------------------------------------------------
        // Create the initial solution with random keys
        CreateInitialSolutions(s, problem.getDimension()); 
        s.ofv = problem.evaluate(s);
        sBest = s;

        // ---------------------------------------------------------------------
        // Main Loop
        // ---------------------------------------------------------------------
        // run the search process until stop criterion
        while (currentTime < runData.MAXTIME * runData.restart)
        {
            IterT = 0;
            if (!reanneling) T = T0;
            else T = T0 * 0.3;

            // Temperature Loop
            while (T > 0.0001 && currentTime < runData.MAXTIME * runData.restart)
            {
                // Q-Learning Update (Pre-Action)
                if (runData.control == 1 && !S.empty()){
                    // set Q-Learning parameters  
                    SetQLParameter(currentTime, Ti, restartEpsilon, epsilon_max, epsilon_min, epsilon, lf, df, (int)(runData.MAXTIME * runData.restart)); 

                    // choose a action at for current state st
                    at = ChooseAction(S, st, epsilon);

                    // execute action at of st (Safe Check)
                    if (at >= 0 && at < (int)S[st].Ai.size()) {
                        iCurr = S[st].Ai[at];

                        // define the parameters according of the current state
                        SAmax   = (int)S[iCurr].par[0];
                        alphaSA = S[iCurr].par[1];      
                        betaMin = S[iCurr].par[2];
                        betaMax = S[iCurr].par[3];            
                    }
                }
                
                bestOFV = INFINITY;

                // Metropolis Loop
                while (IterT < SAmax && currentTime < runData.MAXTIME * runData.restart)
                {
                    if (SOLVER_SHOULD_STOP) return;      
                    
                    IterT++;

                    // Shake the current solution
                    sViz = s;
                    ShakeSolution(sViz, betaMin, betaMax, problem.getDimension());

                    // calculate the OFV
                    sViz.ofv = problem.evaluate(sViz);
                    
                    // value function is the best solution found in this iteration
                    if (sViz.ofv < bestOFV)
                        bestOFV = sViz.ofv;
                    
                    // calculate the delta SA
                    delta = sViz.ofv - s.ofv;

                    // define from which solution to continue the search
                    if (delta < 0)
                    {
                        // update current solution
                        s = sViz;

                        // update the best solution found by SA
                        if (s.ofv < sBest.ofv)
                        {
                            sBest = s;
                            improv = 1;

                            // update the SOLVER_POOL of solutions
                            UpdatePoolSolutions(s, method, runData.debug);
                        }
                    }
                    else
                    {
                        // metropolis criterion
                        double x = randomico(0, 1);
                        if ( x < (std::exp(-delta / T)) )        
                            s = sViz;
                    }
                } // End-SAmax

                // Q-Learning Reward (Post-Action)
                if (runData.control == 1 && !S.empty()){
                    // The reward function is based on improvement of the current best fitness and binary reward
                    if (improv){
                        R = 1;
                        improv = 0;
                    }
                    else{
                        if (std::abs(bestOFV) > 1e-9)
                            R = (sBest.ofv - bestOFV) / bestOFV;
                        else 
                            R = 0;
                    }

                    // index of the next state & Update Q-Table
                    if (at >= 0 && at < (int)S[st].Ai.size()) {
                        int st_1 = S[st].Ai[at];

                        if (at < (int)S[st].Qa.size()) {
                            S[st].Qa[at] = S[st].Qa[at] + lf * (R + df * S[st_1].maxQ - S[st].Qa[at]); 

                            if (S[st].Qa[at] > S[st].maxQ)
                            {
                                S[st].maxQ = S[st].Qa[at];
                                S[st].maxA = at;
                            }
                        }
                        // Define the new current state st
                        st = st_1;
                    }
                }
                // *************************************************************

                T = T * alphaSA;
                IterT = 0;

                // apply local search (Nelder-Mead)
                sViz = s;
                NelderMeadSearch(sViz, problem);

                // update the best solution found by SA
                if (sViz.ofv < sBest.ofv)
                {
                    sBest = sViz;

                    // update the SOLVER_POOL of solutions
                    UpdatePoolSolutions(sBest, method, runData.debug);
                }

                // terminate the search process in MAXTIME
                end_timeMH = get_time_in_seconds();
                currentTime = (float)(end_timeMH - start_timeMH);
                
            } // Fim-T

            // reanneling
            reanneling = 1;
        }

        // print policy
        // if (runData.debug and runData.control == 1)
        //     PrintPolicy(S, st);
    }

} // namespace rkolib::mh