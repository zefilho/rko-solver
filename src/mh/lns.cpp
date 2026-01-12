#include "rkolib/mh/lns.hpp"

// Dependências internas
#include "rkolib/core/method.hpp"
#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/problem.hpp" // Para data.n

namespace rkolib::mh {

    using namespace rkolib::core;

    // -------------------------------------------------------------------------
    // Helper Function: fareySequence
    // Description: generate the farey sequence F of order num (Internal use)
    // -------------------------------------------------------------------------
    static void fareySequence(int num, std::vector<double> &F) {
        int a = 0, b = 1, c = 1, d = num;
        
        F.push_back((double)a / b);

        while (c <= num) {
            int k = (num + b) / d;
            int temp_a = a, temp_b = b;
            a = c;
            b = d;
            c = k * c - temp_a;
            d = k * d - temp_b;
            
            if (a != b){
                F.push_back((double)a / b);
            }
            else{
                F.push_back(((double)a / b) - 0.00001);
            }   
        }
    }

    // -------------------------------------------------------------------------
    // Main Algorithm: LNS
    // -------------------------------------------------------------------------
    void LNS(const TRunData &runData, const TProblemData &data)
    {
        const char* method = "LNS";
        double T0 = 0;                       // initial temperature
        double T;                            // current temperature
        double alphaLNS = 0;                 // cool rate
        double betaMin = 0.0;                // minimum perturbation
        double betaMax = 0.0;                // maximum perturbation
        int reanneling = 0;                  // reanneling flag

        TSol sLine,                          // neighborhood solution
             sLineBest,                      // best neighborhood solution
             s,                              // current solution
             sBest;                          // best solution of LNS
        
        float currentTime = 0;               // computational time of the search process  
        int improv = 0;                      // improvement flag

        double start_timeMH = get_time_in_seconds();    // start computational time
        double end_timeMH = get_time_in_seconds();      // end computational time

        std::vector<int> RKorder(data.n);    // define a order for the neighors
        std::iota(RKorder.begin(), RKorder.end(), 0);

        // ---------------------------------------------------------------------
        // Q-Learning parameters
        // ---------------------------------------------------------------------
        std::vector<TState> S;              // finite state space
        int numPar = 0;                     // number of parameters
        int numStates = 0;                  // number of states
        int iCurr = 0;                      // current (initial) state
        double epsilon = 0;                 // greed choice possibility
        double lf = 0;                      // learning factor
        double df = 0;                      // discount factor
        double R = 0;                       // reward
        
        float epsilon_max = 1.0;            // maximum epsilon 
        float epsilon_min = 0.1;            // minimum epsilon
        int Ti = 1;                         // number of epochs performed
        int restartEpsilon = 1;             // number of restart epsilon
        int st = 0;                         // current state
        int at = 0;                         // current action

        // ** read file with parameter values
        numPar = 4;
        std::vector<std::vector<double>> parameters;
        parameters.resize(numPar);

        readParameters(method, runData.control, parameters, numPar);

        // create a Farey sequence for the Repair operator
        std::vector<double> F;
        fareySequence(7, F);

        // offline control
        if (runData.control == 0){
            if (!parameters[0].empty()) betaMin  = parameters[0][0];
            if (!parameters[1].empty()) betaMax  = parameters[1][0];
            if (!parameters[2].empty()) T0       = parameters[2][0];
            if (!parameters[3].empty()) alphaLNS = parameters[3][0];
        }

        // online control
        else{
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

                // define parameters of LNS based on state
                if (!S.empty()) {
                    betaMin  = S[iCurr].par[0];
                    betaMax  = S[iCurr].par[1];
                    T0       = S[iCurr].par[2];
                    alphaLNS = S[iCurr].par[3];
                }
            }
        }   

        // Create the initial solution with random keys
        CreateInitialSolutions(s, data.n); 
        s.ofv = Decoder(s, data);    
        sBest = s;

        // run the search process until stop criterion
        while (currentTime < runData.MAXTIME * runData.restart)
        {
            if (!reanneling) T = T0;
            else T = T0 * 0.3;

            while (T > 0.01 && currentTime < runData.MAXTIME * runData.restart)
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
                        betaMin  = S[iCurr].par[0];
                        betaMax  = S[iCurr].par[1];
                        T0       = S[iCurr].par[2];
                        alphaLNS = S[iCurr].par[3];
                    }
                }

                // -------------------------------------------------------------
                // LNS DESTRUCTION PHASE (Ruina)
                // -------------------------------------------------------------
                sLine = s;
                int intensity = irandomico((int)(betaMin * data.n), (int)(betaMax * data.n));
                if (intensity < 1) intensity = 1; // Segurança mínima
                
                // define which rk will be deleted - Random Removal
                // 'rng' vem de Methods.hpp (extern)
                std::shuffle(RKorder.begin(), RKorder.end(), rng); 

                // -------------------------------------------------------------
                // LNS REPAIR PHASE (Reconstrução)
                // -------------------------------------------------------------
                // Repair the current solution using Farey intervals
                for (int k = 0; k < intensity; k++)
                {
                    int pos = RKorder[k];
                    double OFVbest = INFINITY;
                    double rkBest = 0;

                    // Testa valores baseados na sequência de Farey
                    for (int j = 0; j < (int)F.size() - 1; j++)
                    {
                        if (stop_execution.load()) return;      
                
                        // generate a random value between two intervals of the Farey sequence
                        sLine.rk[pos] = randomico(F[j], F[j+1]);
                        sLine.ofv = Decoder(sLine, data);

                        if (sLine.ofv < OFVbest)
                        {
                            OFVbest = sLine.ofv;
                            rkBest = sLine.rk[pos];
                        }
                    }

                    // continue the search from the best random key value found for the rk pos
                    sLine.ofv = OFVbest;
                    sLine.rk[pos] = rkBest;
                }

                // local search to improve the repaired solution
                sLineBest = sLine;
                NelderMeadSearch(sLineBest, data);
                
                // calculate delta
                double delta = sLineBest.ofv - s.ofv;

                // acceptance criterion (Metropolis-like)
                if (delta <= 0)
                {
                    s = sLineBest;

                    if (s.ofv < sBest.ofv){
                        sBest = s;
                        improv = 1;

                        // update the pool of solutions
                        UpdatePoolSolutions(s, method, runData.debug);
                    }
                }
                else
                {
                    double x = randomico(0, 1);
                    if ( x < (std::exp(-delta / T)) )        
                        s = sLineBest;
                }

                // -------------------------------------------------------------
                // Q-Learning Reward Phase (Post-Action)
                // -------------------------------------------------------------
                if (runData.control == 1 && !S.empty()){
                    // The reward function is based on improvement of the current best fitness and binary reward
                    if (improv){
                        R = 1;
                        improv = 0;
                    }
                    else{
                         if (std::abs(s.ofv) > 1e-9)
                            R = (sBest.ofv - s.ofv) / s.ofv;
                        else 
                            R = 0;
                    }

                    // index of the next state
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
                
                // Cooling schedule
                T = T * alphaLNS;

                // terminate the search process in MAXTIME
                end_timeMH = get_time_in_seconds();
                currentTime = (float)(end_timeMH - start_timeMH);

            } //Fim-T

            // reanneling
            reanneling = 1;
        }
    }

} // namespace rkolib::mh