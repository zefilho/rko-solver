#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/context.hpp"

namespace rkolib {
    class RkoSolver;
}

namespace rkolib::core {

    // -----------------------------------------------------------------------------
    // General Utilities
    // -----------------------------------------------------------------------------
    bool sortByFitness(const TSol &lhs, const TSol &rhs);

    double randomico(double min, double max);
    int irandomico(int min, int max);
    double get_time_in_seconds();

    // -----------------------------------------------------------------------------
    // Solution & Pool Management
    // -----------------------------------------------------------------------------
    void CreateInitialSolutions(TSol &s, const int n);
    void CreatePoolSolutions(rkolib::RkoSolver &solver, const int sizePool);
    void UpdatePoolSolutions(TSol s, const char* mh, const int debug);

    // -----------------------------------------------------------------------------
    // Local Searches & Metaheuristics Components
    // -----------------------------------------------------------------------------
    void ShakeSolution(TSol &s, float betaMin, float betaMax, const int n);
    
    TSol Blending(TSol &s1, TSol &s2, double factor, const int n);
    
    void NelderMeadSearch(TSol &x1, rkolib::RkoSolver &solver);

    void SwapLS(TSol &s, rkolib::RkoSolver &solver, const int &strategy, std::vector<int> &RKorder);
    void InvertLS(TSol &s, rkolib::RkoSolver &solver, const int &strategy, std::vector<int> &RKorder);
    void FareyLS(TSol &s, rkolib::RkoSolver &solver, const int &strategy, std::vector<int> &RKorder);
    
    void RVND(TSol &s, rkolib::RkoSolver &solver, const int &strategy, std::vector<int> &RKorder);

    // -----------------------------------------------------------------------------
    // IO / Config
    // -----------------------------------------------------------------------------

    void readParameters(const std::string& method, int control, 
                     std::vector<std::vector<double>> &parameters, int numPar);                 

    void readParametersYaml(const char* method, int control, 
                    std::vector<std::vector<double>> &parameters, int numPar);

} // namespace rkolib::core