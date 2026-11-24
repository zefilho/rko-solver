#pragma once

#include "rkolib/core/data.hpp"

namespace rkolib::core {

    /**
     * Method: PrintPolicy
     * Description: Print the policy in the screen
     */
    void PrintPolicy(const std::vector<TState>& S, int st);

    /**
     * Method: SaveQTable
     * Description: Persist the Q-Table to disk (thread-safe via OpenMP critical)
     */
    bool SaveQTable(const std::vector<TState>& S, int st, const std::string& method,
                    const std::string& outDir = "results");

    /**
     * Method: LoadQTable
     * Description: Load a previously saved Q-Table if topology matches (thread-safe)
     */
    bool LoadQTable(std::vector<TState>& S, const std::string& method,
                    const std::string& outDir = "results");

    /**
     * Method: CreateStates
     * Description: Select all states s in S and specify actions a in A (M/D/A adjacent topology)
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
     * Method: UpdateMaxQ
     * Description: Recompute maxQ and maxA for a given state st
     */
    void UpdateMaxQ(std::vector<TState> &S, int st);

    /**
     * Method: UpdateQValue
     * Description: Update Q-Table value, conditionally recalculate maxQ/maxA, and transition to next state
     */
    void UpdateQValue(std::vector<TState> &S, int &st, int at, double R, double lf, double df);

    /**
     * Method: ComputeDelta
     * Description: Compute capped relative improvement in [0, delta_max]
     */
    double ComputeDelta(double oldBest, double newBest, double delta_max = 0.1);

    /**
     * Method: SetQLParameter (Cosine annealing with warm restarts)
     * Description: Update the parameters of the Q-Learning method (epsilon_max by reference)
     */
    void SetQLParameter(float currentTime, int &Ti, int &restartEpsilon,
                        float &epsilon_max, float epsilon_min,
                        double &epsilon, double &lf, double &df, int MAXTIME);

    /**
     * Method: SetQLParameter (Improvement-based decay)
     * Description: Update epsilon with floor epsilon_min, learning rate lf, and discount factor df
     */
    void SetQLParameter(double &epsilon, double &lf, double &df, int MAXTIME,
                        float currentTime, float delta, double epsilon_min = 0.1);

} // namespace rkolib::core