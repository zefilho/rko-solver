#pragma once

#include "rkolib/core/data.hpp" // Necessário para TState

namespace rkolib::core {

    /**
     * Method: PrintPolicy
     * Description: Print the policy in the screen
     */
    void PrintPolicy(const std::vector<TState>& S, int st);

    /**
     * Method: CreateStates
     * Description: Select all states s in S and specify actions a in A
     * Note: Uses OpenMP critical section internally.
     */
    void CreateStates(const std::vector<std::vector<double>>& parameters, 
                      int &numStates, 
                      int numPar, 
                      std::vector<TState> &S);

    /**
     * Method: ChooseAction
     * Description: Choose actions based on epsilon-greedy policy
     */
    int ChooseAction(const std::vector<TState>& S, int st, double epsilon);

    /**
     * Method: SetQLParameter
     * Description: Update the parameters of the Q-Learning method (epsilon decay, learning rate)
     */
    void SetQLParameter(float currentTime, int &Ti, int &restartEpsilon, 
                        float epsilon_max, float epsilon_min, 
                        double &epsilon, double &lf, double &df, int MAXTIME);

} // namespace rkolib::core