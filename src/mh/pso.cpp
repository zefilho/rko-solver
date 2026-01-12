#include "rkolib/mh/pso.hpp"

// Dependências internas
#include "rkolib/core/method.hpp"
#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/problem.hpp" // Para data.n e definição completa de TProblemData

namespace rkolib::mh {

    using namespace rkolib::core;

    // -------------------------------------------------------------------------
    // Helper Function: UpdateParticleSize
    // Description: Internal logic to resize population based on Q-Learning
    // -------------------------------------------------------------------------
    static void UpdateParticleSize(std::vector<TSol> &X, std::vector<TSol> &Pbest, 
                                   std::vector<std::vector<float>> &V, 
                                   const TSol &Gbest, int Psize,  
                                   float c1, float c2, float w, const TProblemData &data)
    {
        // size of the current population
        int oldPsize = (int)X.size();

        // pruning 
        if (oldPsize > Psize){
            X.resize(Psize);
            Pbest.resize(Psize);
            V.resize(Psize);
        }

        // generate new particles 
        else if (oldPsize < Psize){
            X.resize(Psize);
            Pbest.resize(Psize);
            // Resize V and ensure new inner vectors are sized correctly
            V.resize(Psize, std::vector<float>(data.n));

            // Create the initial particles with random keys 
            for (int i = oldPsize; i < Psize; i++)
            {
                // initialize X[i]
                CreateInitialSolutions(X[i], data.n); 

                // initialize Pbest
                Pbest[i] = X[i];    

                // initialize V[i][j]
                // V[i] já foi alocado pelo resize acima
                for (int j = 0; j < data.n; j++)
                    V[i][j] = (float)randomico(0, 1);

                for (int j = 0; j < data.n; j++)
                {
                    float r1 = (float)randomico(0, 1);
                    float r2 = (float)randomico(0, 1);

                    // update v[i][j] logic (using constriction-like formula from original code)
                    V[i][j] = w * (V[i][j] + (c1 * r1 * ((float)Pbest[i].rk[j] - (float)X[i].rk[j])) + 
                                            (c2 * r2 * ((float)Gbest.rk[j] - (float)X[i].rk[j])));
                    
                    // update X[i][j]
                    double oldrk = X[i].rk[j];
                    X[i].rk[j] = X[i].rk[j] + V[i][j];  

                    if (X[i].rk[j] < 0.0 || X[i].rk[j] >= 1.0) {
                        X[i].rk[j] = oldrk; 
                        V[i][j] = 0;
                    }
                }

                // fitness of X[i]
                X[i].ofv = Decoder(X[i], data);

                // update Pbest (initial)
                Pbest[i] = X[i];    
            }
        }
    }

    // -------------------------------------------------------------------------
    // Main Algorithm: PSO
    // -------------------------------------------------------------------------
    void PSO(const TRunData &runData, const TProblemData &data)
    {
        const char* method = "PSO";
        int Psize = 0;                           // number of particles
        float c1 = 0.0;
        float c2 = 0.0;
        float w = 0.0;

        std::vector<TSol> X;                     // current solutions
        std::vector<TSol> Pbest;                 // best solutions 
        std::vector<std::vector<float>> V;       // particle velocity

        TSol Gbest;                              // global best solution
        double bestOFcurrent = 0;                // best ofv found in the current generation

        // local variables
        int numGenerations = 0;                  // number of generations
        float currentTime = 0;                   // computational time of the search process
        int bestGeneration = 0;                  // number of generation that found the best solution
        int improv = 0;                          // improvement flag

        double start_timeMH = get_time_in_seconds();    // start computational time
        double end_timeMH = get_time_in_seconds();      // end computational time

        std::vector<int> RKorder(data.n);        // define a order for the neighors
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

        // offline control
        if (runData.control == 0){
            // define parameters of PSO (offline)
            if (!parameters[0].empty()) Psize = (int)parameters[0][0];
            if (!parameters[1].empty()) c1    = (float)parameters[1][0];
            if (!parameters[2].empty()) c2    = (float)parameters[2][0];
            if (!parameters[3].empty()) w     = (float)parameters[3][0];
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

                // define parameters of PSO based on state
                if (!S.empty()) {
                    Psize = (int)S[iCurr].par[0];
                    c1    = (float)S[iCurr].par[1];
                    c2    = (float)S[iCurr].par[2];
                    w     = (float)S[iCurr].par[3];
                }
            }
        }   

        // initialize population
        X.clear(); 
        Pbest.clear();
        V.clear(); 

        X.resize(Psize);
        Pbest.resize(Psize);
        V.resize(Psize, std::vector<float>(data.n));

        // Create the initial particles with random keys 
        Gbest.ofv = INFINITY;
        
        for (int i=0; i<Psize; i++)
        {
            // initialize X[i]
            CreateInitialSolutions(X[i], data.n); 

            // fitness of X[i]
            X[i].ofv = Decoder(X[i], data);

            // initialize V[i][j]
            for (int j=0; j<data.n; j++)
                V[i][j] = (float)randomico(0, 1);

            // initialize Gbest
            if (X[i].ofv < Gbest.ofv)
                Gbest = X[i];

            // initialize Pbest
            Pbest[i] = X[i];    
        }
        
        // run the evolutionary process until stop criterion
        while (currentTime < runData.MAXTIME * runData.restart)
        {
            // number of generations
            numGenerations++;

            // -----------------------------------------------------------------
            // Q-Learning Update Phase (Pre-Action)
            // -----------------------------------------------------------------
            if (runData.control == 1 && !S.empty()){
                // set Q-Learning parameters  
                SetQLParameter(currentTime, Ti, restartEpsilon, epsilon_max, epsilon_min, epsilon, lf, df, (int)(runData.MAXTIME * runData.restart)); 

                // choose a action at for current state st
                at = ChooseAction(S, st, epsilon);

                // execute action at of st (Bounds Check)
                if (at >= 0 && at < (int)S[st].Ai.size()) {
                    iCurr = S[st].Ai[at];

                    // define the parameters according of the current state
                    Psize = (int)S[iCurr].par[0];
                    c1    = (float)S[iCurr].par[1];
                    c2    = (float)S[iCurr].par[2];
                    w     = (float)S[iCurr].par[3];          

                    // Update population size based on new parameters
                    UpdateParticleSize(X, Pbest, V, Gbest, Psize, c1, c2, w, data);
                }
            }

            // -----------------------------------------------------------------
            // PSO Evolution Phase
            // -----------------------------------------------------------------
            bestOFcurrent = INFINITY;
            // double media = 0; // Unused in logic, removed to avoid warning

            // Garante que loop respeite tamanho atual (que pode ter mudado)
            int currentPsize = (int)X.size(); 

            for (int i = 0; i < currentPsize; i++)
            {
                if (stop_execution.load()) return;      

                double probUpdate = 1.0; 

                // update particles X[i]
                for (int j = 0; j < data.n; j++)
                {
                    float r1 = (float)randomico(0, 1);
                    float r2 = (float)randomico(0, 1);

                    // update v[i][j] com fator de constricao (logica original)
                    V[i][j] = w * (V[i][j] + (c1 * r1 * ((float)Pbest[i].rk[j] - (float)X[i].rk[j])) + 
                                            (c2 * r2 * ((float)Gbest.rk[j] - (float)X[i].rk[j])));
                    
                    if (randomico(0, 1) < probUpdate){
                        // update X[i][j]
                        double oldrk = X[i].rk[j];
                        X[i].rk[j] = X[i].rk[j] + V[i][j];  

                        if (X[i].rk[j] < 0.0 || X[i].rk[j] >= 1.0) {
                            X[i].rk[j] = oldrk; 
                            V[i][j] = 0;
                        }
                    }
                }

                // fitness
                X[i].ofv = Decoder(X[i], data);

                // set the best ofv found in this generation
                if (X[i].ofv < bestOFcurrent){
                    bestOFcurrent = X[i].ofv;
                }

                // set Pbest
                if (X[i].ofv < Pbest[i].ofv){
                    Pbest[i] = X[i];
                }

                // set Gbest 
                if (X[i].ofv < Gbest.ofv){
                    Gbest = X[i];
                    bestGeneration = numGenerations;    
                    improv = 1;
                }

                // media += X[i].ofv;
            } 

            // local search
            double oldGbest = Gbest.ofv;
            if (currentPsize > 0) {
                int chosen = irandomico(0, currentPsize - 1);
                NelderMeadSearch(Pbest[chosen], data);
                
                // update global best particle from LS result
                if (Pbest[chosen].ofv < Gbest.ofv){
                    Gbest = Pbest[chosen];
                    bestGeneration = numGenerations; 
                    improv = 1;   
                }
            }

            if (bestGeneration == numGenerations || Gbest.ofv < oldGbest){
                // update the pool of solutions
                UpdatePoolSolutions(Gbest, method, runData.debug);
            }

            // -----------------------------------------------------------------
            // Q-Learning Reward Phase (Post-Action)
            // -----------------------------------------------------------------
            if (runData.control == 1 && !S.empty()){
                // The reward function is based on improvement of the current best fitness and binary reward
                if (improv){
                    R = 1;
                    improv = 0;
                }
                else{
                    if (std::abs(bestOFcurrent) > 1e-9)
                        R = (Gbest.ofv - bestOFcurrent) / bestOFcurrent;
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
            
            // terminate the evolutionary process in MAXTIME
            end_timeMH = get_time_in_seconds();
            currentTime = (float)(end_timeMH - start_timeMH);
        }

        // free memory of PSO components
        // Vectors clean themselves in C++, but clear() is fine
        X.clear();
        V.clear();
        Pbest.clear();
    }

} // namespace rkolib::mh