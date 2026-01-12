#include "rkolib/mh/grasp.hpp"

// Dependências internas
#include "rkolib/core/method.hpp"         // Utils (randomico, Decoder, RVND, etc.)
#include "rkolib/core/qlearning.hpp"       // Lógica de Q-Learning
#include "rkolib/core/problem.hpp" // Para data.n

namespace rkolib::mh {

    using namespace rkolib::core;

    // -------------------------------------------------------------------------
    // Helper Function: LineSearch (Internal)
    // -------------------------------------------------------------------------
    static void LineSearch(TSol s, float h, int i, double &bestZ, double &bestF, const TProblemData &data)
    {
        // find the best solution in line
        bestZ = 0;
        bestF = INFINITY;

        // generate k as possible random keys for position i, k = rk_i * tau | tau = {0, 1, -1, 2, -2, ...}
        double tau = 0;
        std::vector<double> rk;
        rk.reserve((int)(1.0/h) + 2); // Pre-allocate memory

        // Add base point
        rk.push_back(s.rk[i] + tau * h);
        
        // Add neighbors
        int steps = (int)(1.0/h) + 1;
        for (int j = 0; j < steps; j += 2){
            tau++;

            double val_pos = s.rk[i] + tau * h;
            if (val_pos >= 0.0 && val_pos < 1.0)
                rk.push_back(val_pos);

            double val_neg = s.rk[i] - tau * h;
            if (val_neg >= 0.0 && val_neg < 1.0)
                rk.push_back(val_neg);
        }

        // (sample greedy) 
        // q < m possible insertions (chosen uniformly at random).
        // Avoid log2(0) or negative
        int q = 1;
        if (h > 1e-9) {
            q = (int)ceil(log2(1.0/h)) + 1; 
        }
        
        if (q > (int)rk.size())
            q = (int)rk.size();

        // choose a subset with q rks to calculate the decoder
        // Nota: Idealmente usaríamos o 'rng' do contexto, mas aqui usamos random_device local 
        // para manter compatibilidade com a lógica interna da função original sem passar 'rng' por toda cadeia.
        // Se possível, refatorar para receber 'rng'.
        static std::mt19937 local_rng(std::random_device{}());
        std::shuffle(rk.begin(), rk.end(), local_rng);

        // calculate the quality of solution s with rk j
        for (int j = 0; j < q; j++)
        {  
            if (stop_execution.load()) return;      
            
            s.rk[i] = rk[j];   
            s.ofv = Decoder(s, data);

            if (s.ofv < bestF){
                bestZ = s.rk[i];
                bestF = s.ofv;
            }
        }
        rk.clear();
    }

    // -------------------------------------------------------------------------
    // Helper Function: ConstrutiveGreedyRandomized (Internal)
    // -------------------------------------------------------------------------
    static void ConstrutiveGreedyRandomized(TSol &s, float h, float alpha, const TProblemData &data)
    {
        std::vector<int> UnFixed(data.n);                // store the random-keys not yet fixed
        std::vector<int> chosenRK;                       // store the random-keys that will be search in the line search
        std::vector<double> z(data.n);                   // store the best value of the random-key i
        std::vector<double> g(data.n, INFINITY);         // store the value of the ofv with a random-key z_i

        double min; // Removed unused 'max'

        // minimum and maximum intensity of the construction phase
        double betaMin = 0.3, 
               betaMax = 0.7;

        // initialize the points of the solution that can be changed
        std::iota(UnFixed.begin(), UnFixed.end(), 0);

        // construct a solution by perturbing the current solution and choosing one of the best
        double intensity = randomico(betaMin, betaMax);

        // index of the random key to be set
        int kBest = 0;  
        
        int limit = (int)(data.n * intensity);
        for (int j = 0; j < limit; j++)
        {
            if (stop_execution.load()) return;

            // create a list of candidate solutions by perturbing a (not yet 'fixed') rk of the current solution
            min = INFINITY;
            // max = -INFINITY; // Unused

            // choose the subset of random keys that will be searched
            chosenRK.clear();
            
            int kMax = (int)(UnFixed.size() * 0.1);
            if (kMax < 2) 
                kMax = (int)UnFixed.size();

            chosenRK = UnFixed;
            // 'rng' vem de Methods.hpp (extern)
            std::shuffle(chosenRK.begin(), chosenRK.end(), rng);
            if (kMax < (int)chosenRK.size()) {
                chosenRK.resize(kMax);
            }

            // line search
            // z e g são reutilizados, mas precisam ser resetados conceitualmente? 
            // Na verdade, só acessamos os índices 'i' que estão em chosenRK.

            for (int k = 0; k < kMax; k++) 
            {
                int i = chosenRK[k];

                TSol sAux = s;
                
                // linear search
                // Atualiza z[i] e g[i] por referência/acesso direto se g fosse passado
                // A função LineSearch original atualizava variaveis passadas por referencia.
                // Aqui: z[i] e g[i] são passados
                LineSearch(sAux, h, i, z[i], g[i], data);

                // store the best g[i] and the rk that found this g
                if (min > g[i])
                {
                    min = g[i];
                    kBest = i;
                }
            }

            // select the best candidate to continue building                  
                
            // update the current solution
            s.rk[kBest] = z[kBest];
            s.ofv = g[kBest];

            // remove rk k from the UnFixed set
            for (int i = 0; i < (int)UnFixed.size(); i++)
            {
                if (UnFixed[i] == kBest) 
                {
                    UnFixed.erase(UnFixed.begin() + i);
                    break;
                }
            }
        }

        // update the solution found in the constructive phase
        s.ofv = Decoder(s, data);
    }

    // -------------------------------------------------------------------------
    // Main Algorithm: GRASP
    // -------------------------------------------------------------------------
    void GRASP(const TRunData &runData, const TProblemData &data)
    {
        const char* method = "GRASP";
        // GRASP parameters
        TSol s;                                         // current solution
        TSol sLine;                                     // constructive solution
        TSol sLineBest;                                 // local search solution
        TSol sBest;                                     // best solution of GRASP

        float currentTime = 0;                          // computational time of the search process
        int improv = 0;                                 // improvement flag
        
        double start_timeMH = get_time_in_seconds();    // start computational time
        double end_timeMH = get_time_in_seconds();      // end computational time

        std::vector<int> RKorder(data.n);               // define a order for the neighors
        std::iota(RKorder.begin(), RKorder.end(), 0);

        float alphaGrasp = 0.1;                         // greedy rate
        float h = 0;                                    // grid dense
        float hs = 0;                                   // start grid dense
        float he = 0;                                   // end grid dense

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
        numPar = 3;
        std::vector<std::vector<double>> parameters;
        parameters.resize(numPar);

        readParameters(method, runData.control, parameters, numPar);

        // offline control
        if (runData.control == 0){
            // define parameters of GRASP
            if (!parameters[0].empty()) alphaGrasp = (float)parameters[0][0];
            if (!parameters[1].empty()) hs         = (float)parameters[1][0];
            if (!parameters[2].empty()) he         = (float)parameters[2][0];
        }

        // online control
        else {
            if (runData.control == 1){
                // create possible states of the Markov chain
                CreateStates(parameters, numStates, numPar, S);

                // number of restart epsilon
                restartEpsilon = 1;  

                // maximum epsilon  
                epsilon_max = 1.0;  

                // current state
                iCurr = irandomico(0, numStates - 1);

                // define parameters of GRASP
                if (!S.empty()) {
                    alphaGrasp = (float)S[iCurr].par[0];
                    hs = (float)S[iCurr].par[1];
                    he = (float)S[iCurr].par[2];
                }
            }
        }

        // create an initial solution
        CreateInitialSolutions(s, data.n);
        s.ofv = Decoder(s, data);
        sBest = s;

        int iter = 0;
        // run the search process until stop criterion (maxTime)
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
                    alphaGrasp = (float)S[iCurr].par[0];  
                    hs = (float)S[iCurr].par[1];
                    he = (float)S[iCurr].par[2];          
                }
            }

            h = hs;
            // noImprov = 0;
            while (h >= he && currentTime < runData.MAXTIME * runData.restart)
            {
                if (stop_execution.load()) return; 

                iter++;

                // offline control
                if (runData.control == 0){
                    alphaGrasp = (float)randomico(0.1, 0.9);
                }

                // construct a greedy randomized solution
                sLine = s;
                ConstrutiveGreedyRandomized(sLine, h, alphaGrasp, data);
                
                // apply local search in current solution
                sLineBest = sLine;
                RVND(sLineBest, data, runData.strategy, RKorder);

                // update the best solution found by GRASP
                if (sLineBest.ofv < sBest.ofv){
                    sBest = sLineBest;
                    improv = 1;

                    // update the pool of solutions
                    UpdatePoolSolutions(sLineBest, method, runData.debug);
                }
                // make grid more dense
                else{
                    h = h / 2.0f;
                }

                // accept criterion
                if (sLineBest.ofv < s.ofv){
                    s = sLineBest;
                }
                else{
                    // Metropolis criterion
                    double x = randomico(0.0, 1.0);
                    // Avoid division by zero in time factor
                    double timeFactor = 100.0 - 100.0 * (currentTime / (runData.MAXTIME * runData.restart + 1e-9));
                    if (timeFactor < 1.0) timeFactor = 1.0;

                    if ( x < (exp(-(sLineBest.ofv - s.ofv) / timeFactor)) ){ 
                        s = sLineBest;
                    }
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
                    if (std::abs(sLineBest.ofv) > 1e-9)
                        R = (sBest.ofv - sLineBest.ofv) / sLineBest.ofv;
                    else
                        R = 0;
                }

                // index of the next state
                if (at >= 0 && at < (int)S[st].Ai.size()) {
                    int st_1 = S[st].Ai[at];

                    // update the Q-Table value
                    if (at < (int)S[st].Qa.size()) {
                        S[st].Qa[at] = S[st].Qa[at] + lf * (R + df * S[st_1].maxQ - S[st].Qa[at]); 

                        if (S[st].Qa[at] > S[st].maxQ)
                        {
                            S[st].maxQ = S[st].Qa[at];
                            S[st].maxA = at;
                        }
                    }
                    // define the new current state st
                    st = st_1;
                }
            }

            // terminate the search process in MAXTIME
            end_timeMH = get_time_in_seconds();
            currentTime = (float)(end_timeMH - start_timeMH);
        }
    }

} // namespace rkolib::mh