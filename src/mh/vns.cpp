#include "rkolib/mh/vns.hpp"

// Dependências internas da Lib
#include "rkolib/core/method.hpp"   // Utils (randomico, get_time, stop_execution, RVND, Shake)
#include "rkolib/core/qlearning.hpp" // Lógica de Q-Learning
#include "rkolib/core/problem.hpp" // Acesso à estrutura TProblemData (data.n)

namespace rkolib::mh {

    void VNS(const rkolib::core::TRunData &runData, const rkolib::core::TProblemData &data)
    {
        using namespace rkolib::core; // Facilita acesso a TSol, TState, etc.

        const char* method = "VNS";
        double beta = 0;                    // perturbation rate
        int Iter = 0;                       // current iteration
        int IterMelhora = 0;                // iteration that found the best solution
        int kMax = 0;                       // number of neighborhood strutures
        double betaMin = 0.0;               // intensity of perturbation
        int improv = 0;                     // improvement flag

        TSol s,                             // current solution
             sBest,                         // best solution of VNS
             sLine,                         // neighborhood solution
             sBestLine;                     // best neighborhood solution

        
        float currentTime = 0;              // computational time of the search process

        double start_timeMH = get_time_in_seconds();    // start computational time
        double end_timeMH = get_time_in_seconds();      // end computational time

        std::vector<int> RKorder(data.n);   // define a order for the neighors
        std::iota(RKorder.begin(), RKorder.end(), 0);

        // ---------------------------------------------------------------------
        // Q-Learning parameters initialization
        // ---------------------------------------------------------------------
        std::vector<TState> S;              // finite state space
        int numPar = 0;                     // number of parameters
        int numStates = 0;                  // number of states
        int iCurr = 0;                      // current (initial) state
        double epsilon = 0;                 // greed choice possibility
        double lf = 0;                      // learning factor
        double df = 0;                      // discount factor
        double R = 0;                       // reward
        
        // std::vector <std::vector <TQ> > Q; // (Nota Senior: Variável declarada mas não utilizada no original, mantida comentada)
        // std::vector<int> ai;               // (Nota Senior: Não utilizada no original)
        
        float epsilon_max = 1.0;            // maximum epsilon 
        float epsilon_min = 0.1;            // minimum epsilon
        int Ti = 1;                         // number of epochs performed
        int restartEpsilon = 1;             // number of restart epsilon
        int st = 0;                         // current state
        int at = 0;                         // current action

        // ** read file with parameter values
        numPar = 2;
        std::vector<std::vector<double>> parameters;
        parameters.resize(numPar);

        // Lê parâmetros do arquivo (Método definido em method.hpp)
        readParameters(method, runData.control, parameters, numPar);

        // offline control
        if (runData.control == 0){
            if (!parameters[0].empty()) kMax    = (int)parameters[0][0];
            if (!parameters[1].empty()) betaMin = parameters[1][0];
        }

        // online control
        else{
            // Q-Learning 
            if (runData.control == 1){
                // create possible states of the Markov chain (Defined in qlearning.hpp)
                CreateStates(parameters, numStates, numPar, S);

                // number of restart epsilon
                restartEpsilon = 1;  

                // maximum epsilon  
                epsilon_max = 1.0;  

                // current state
                iCurr = irandomico(0, numStates - 1);

                // define parameters of VNS based on initial state
                if (!S.empty()) {
                    kMax    = (int)S[iCurr].par[0];
                    betaMin = S[iCurr].par[1];
                }
            }
        }   

        // ---------------------------------------------------------------------
        // Initialization
        // ---------------------------------------------------------------------
        // Create the initial solution with random keys
        sBest.ofv = INFINITY;
        
        // Loop de 1 iteração apenas para gerar solução inicial (mantido do original)
        for (int i=0; i<1; i++)
        {
            CreateInitialSolutions(s, data.n); 
            s.ofv = Decoder(s, data);
            if (s.ofv < sBest.ofv)
                sBest = s;
        }

        // current solution
        s = sBest;

        // ---------------------------------------------------------------------
        // Main Loop
        // ---------------------------------------------------------------------
        // run the search process until stop criterion            
        while (currentTime < runData.MAXTIME * runData.restart)
        {
            // Q-Learning Update Phase (Pre-Action)
            if (runData.control == 1 && !S.empty()){
                // set Q-Learning parameters  
                SetQLParameter(currentTime, Ti, restartEpsilon, epsilon_max, epsilon_min, epsilon, lf, df, (int)(runData.MAXTIME * runData.restart)); 

                // choose a action at for current state st
                at = ChooseAction(S, st, epsilon);

                // execute action at of st
                if (at >= 0 && at < (int)S[st].Ai.size()) {
                    iCurr = S[st].Ai[at];

                    // define the parameters according of the current state
                    kMax    = (int)S[iCurr].par[0];
                    betaMin = S[iCurr].par[1];                      
                }
            }

            // VNS Loop: current neighborhood
            int k = 1;
            while (k <= kMax && currentTime < runData.MAXTIME * runData.restart)
            {
                if (stop_execution.load()) return;      
                
                Iter++;
                
                //s' <- perturb the best solution in the neighborhood k
                beta = randomico(k * betaMin, (k + 1) * betaMin);

                // perturb the current solution (s)
                sLine = s;
                ShakeSolution(sLine, beta, beta, data.n);

                // calculate OFV
                sLine.ofv = Decoder(sLine, data);

                //s*' <- local search (s')
                sBestLine = sLine; 
                RVND(sBestLine, data, (int)runData.strategy, RKorder);

                //s <- acceptance criterion (s,s*', historico)
                if (sBestLine.ofv < s.ofv)
                {
                    s = sBestLine;
                    
                    // update the best solution found in VNS
                    if (s.ofv < sBest.ofv){   
                        sBest = s;   

                        // return to the first neighborhood structure      
                        k = 1;
                        IterMelhora = Iter;
                        improv = 1;

                        // update the pool of solutions (Global Method)
                        UpdatePoolSolutions(sBestLine, method, (int)runData.debug);
                    }
                }
                else
                {
                    // next neighborhood structure
                    k++; 
                }

                // terminate the search process in MAXTIME
                end_timeMH = get_time_in_seconds();
                currentTime = (float)(end_timeMH - start_timeMH);
            }

            // Q-Learning Reward Phase (Post-Action)
            if (runData.control == 1 && !S.empty()){
                // The reward function is based on improvement of the current best fitness and binary reward
                if (improv){
                    R = 1;
                    improv = 0;
                }
                else{
                    // Evita divisão por zero
                    if (std::abs(s.ofv) > 1e-9)
                        R = (sBest.ofv - s.ofv) / s.ofv;
                    else 
                        R = 0;
                }

                // index of the next state
                // Garante segurança no acesso ao vetor
                if (at >= 0 && at < (int)S[st].Ai.size()) {
                    int st_1 = S[st].Ai[at];

                    // Update the Q-Table value
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
        }

        // print policy (commented in original)
        // if (runData.debug and runData.control == 1)
        //     PrintPolicy(S, st);
    }

} // namespace rkolib::mh