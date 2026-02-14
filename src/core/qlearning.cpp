#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/method.hpp" // Necessário para randomico() e irandomico()

#include <omp.h>

namespace rkolib::core {

    void PrintPolicy(const std::vector<TState>& S, int st)
    {
        
        std::cout << std::format("Policy for state {}:", st) << std::endl;
    
        for (int i = 0; i < (int)S.size(); i++)
        {
            std::cout << std::format("\n i = {}: \t", i);
            for (int j = 0; j < (int)S[i].Ai.size(); j++)
            {
                std::cout << std::format("{} ({:.2f}) \t", S[i].Ai[j], S[i].Qa[j]);
            }
        }
    }

    // void CreateStates(const std::vector<std::vector<double>>& parameters, int &numStates, int numPar, std::vector<TState> &S)
    // {
    //     #pragma omp critical
    //     { 
    //         // generate possible configurations
    //         if (!parameters.empty()) {
    //             numStates = parameters[0].size();
    //             for (int i = 1; i < numPar; i++){
    //                 numStates = numStates * parameters[i].size();
    //             }
    //         }
            
    //         // create states
    //         for (int i = 0; i < numStates; i++)
    //         {
    //             TState sAux;
    //             sAux.label = i; 
    //             sAux.par.resize(numPar);
    //             sAux.ci = 0;
    //             sAux.numN = 0;
    //             sAux.Ai.clear();
    //             sAux.Qa.clear();
    //             sAux.sumQ = 0;
    //             sAux.maxQ = 0;
    //             sAux.minA = 0;
    //             sAux.maxA = 0;
                
    //             S.push_back(sAux);
    //         }

    //         // define the parameter configuration of each state
    //         int nC = 1;
    //         for (int j = 0; j < numPar; j++)
    //         {
    //             nC = nC * parameters[j].size();

    //             for (int i = 0; i < numStates; i++)
    //             {
    //                 int den = (numStates / nC);
    //                 if (den == 0) den = 1; // Proteção contra divisão por zero
                    
    //                 int columnIndex = i / den;
    //                 columnIndex = columnIndex % parameters[j].size();

    //                 S[i].par[j] = parameters[j][columnIndex];
    //             }
    //         }

    //         // set of feasible control actions at each state
    //         for (int i = 0; i < (int)S.size(); i++)
    //         {
    //             for (int j = 0; j < (int)S.size(); j++)
    //             {
    //                 // Calculate Hamming distance
    //                 int distance = 0;
    //                 for (int k = 0; k < (int)S[i].par.size(); k++) {
    //                     if (S[i].par[k] != S[j].par[k]) {
    //                         distance++;
    //                     }
    //                 }

    //                 // We have an action from s_i if the new state is different from s_i by a maximmum of one parameter
    //                 if (distance <= 1){
    //                     S[i].Ai.push_back(j); // define actions ai (index of the new state) from s_i
                        
    //                     // Nota: randomico(0.05, 0.01) parece invertido (min > max). 
    //                     // Verifique se sua implementação de randomico trata swap automatico.
    //                     double q0 = randomico(0.05, 0.01); 
    //                     S[i].Qa.push_back(q0); // initialize Q(si,ai)

    //                     if (q0 > S[i].maxQ)
    //                     {
    //                         S[i].maxQ = q0;
    //                         S[i].maxA = (int)S[i].Ai.size() - 1;
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }

    // Helper 1: Calculates the total number of combinations
    int CalculateTotalStates(const std::vector<std::vector<double>>& parameters, int numPar) {
        if (parameters.empty()) return 0;
        
        int total = parameters[0].size();
        for (int i = 1; i < numPar; i++) {
            total *= parameters[i].size();
        }
        return total;
    }

    // Helper 2: Initializes the vector with empty states
    void InitializeStateVector(std::vector<TState>& S, int numStates, int numPar) {
        S.clear(); // Good practice to clear before pushing
        S.reserve(numStates); // Optimization
        
        for (int i = 0; i < numStates; i++) {
            TState sAux;
            sAux.label = i;
            sAux.par.resize(numPar);
            sAux.ci = 0;
            sAux.numN = 0;
            sAux.Ai.clear();
            sAux.Qa.clear();
            sAux.sumQ = 0;
            sAux.maxQ = 0;
            sAux.minA = 0;
            sAux.maxA = 0;
            // Vectors Ai and Qa are empty by default, no need to clear explicitly on new object
            S.push_back(sAux);
        }
    }

    // Helper 3: Maps the Cartesian product of parameters to states
    void MapParametersToStates(std::vector<TState>& S, const std::vector<std::vector<double>>& parameters, int numPar, int numStates) {
        int nC = 1;
        for (int j = 0; j < numPar; j++) {
            nC *= parameters[j].size();

            // Calculate divisor once per column group to reduce inner complexity
            int den = (numStates / nC);
            if (den == 0) den = 1; 

            for (int i = 0; i < numStates; i++) {
                int columnIndex = (i / den) % parameters[j].size();
                S[i].par[j] = parameters[j][columnIndex];
            }
        }
    }

    // Helper 4: Calculates Hamming distance (removes inner loop nesting)
    int GetHammingDistance(const TState& s1, const TState& s2) {
        int distance = 0;
        size_t size = s1.par.size();
        for (size_t k = 0; k < size; k++) {
            if (s1.par[k] != s2.par[k]) {
                distance++;
            }
        }
        return distance;
    }

    // Helper 5: Establishes connections between states (Q-Table initialization)
    void BuildStateConnections(std::vector<TState>& S) {
        int sSize = (int)S.size();
        
        for (int i = 0; i < sSize; i++) {
            for (int j = 0; j < sSize; j++) {
                
                // Reduced nesting by extracting distance logic
                if (GetHammingDistance(S[i], S[j]) <= 1) {
                    S[i].Ai.push_back(j);
                    
                    double q0 = randomico(0.05, 0.01);
                    S[i].Qa.push_back(q0);

                    if (q0 > S[i].maxQ) {
                        S[i].maxQ = q0;
                        S[i].maxA = (int)S[i].Ai.size() - 1;
                    }
                }
            }
        }
    }
    void CreateStates(const std::vector<std::vector<double>>& parameters, int &numStates, int numPar, std::vector<TState> &S)
    {
        #pragma omp critical
        { 
            // 1. Calculate dimensions
            numStates = CalculateTotalStates(parameters, numPar);
            
            // 2. Allocate memory and defaults
            InitializeStateVector(S, numStates, numPar);

            // 3. Fill parameter configurations
            MapParametersToStates(S, parameters, numPar, numStates);

            // 4. Create topology (Edges and Q-Values)
            BuildStateConnections(S);
        }
    }

    int ChooseAction(const std::vector<TState>& S, int st, double epsilon)
    {
        // ** choose action for current state from Q-Table using epsilon-Greedy policy
        int at;
                      
        // epsilon-greedy policy
        if (randomico(0, 1) <= 1.0 - epsilon) 
        {
            // choose the action with highest Q value
            at = S[st].maxA;
        }
        else
        {
            // choose a randomly selected action
            if (!S[st].Ai.empty()) {
                at = irandomico(0, (int)S[st].Ai.size() - 1);
            } else {
                at = 0; // Fallback se não houver ações
            }
        }

        // return the choose action
        return at;
    }

    void SetQLParameter(float currentTime, int &Ti, int &restartEpsilon, float epsilon_max, float epsilon_min, double &epsilon, double &lf, double &df, int MAXTIME)
    {
        // **** define epsilon ****
        static const double PI = 3.14159265;          

        // restart epsilon once Ti epochs are performed (Ti is 10% of the runtime)
        Ti = (int)(MAXTIME * 0.1);
        if (Ti == 0) Ti = 1; // Proteção contra divisão por zero

        if (currentTime >= restartEpsilon * Ti){
            restartEpsilon++;

            // cosine decay with warm restart
            epsilon_max = epsilon_max - 0.1f;
            if (epsilon_max < epsilon_min)
                epsilon_max = epsilon_min;
            epsilon = epsilon_max;
        }
        else {
            epsilon = epsilon_min + 0.5 * (epsilon_max - epsilon_min) * (1.0 + cos((((int)currentTime % Ti) / (float)(Ti)) * PI));
        }
        
        // *** define learning rate ***

        // initialy, a higher priority is given to the newly gained information (exploration mode)
        // then, we decrement lf and have a higher priority for the existing information in Q-Table (exploitation mode)
        lf = 1.0 - (0.9 * currentTime / MAXTIME); 

        // *** define discount rate ***

        // we look for a higher, long-term reward
        df = 0.8;
    }

} // namespace rkolib::core