#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/iproblem.hpp"
#include "rkolib/core/context.hpp"

//MELHORIA a Fazer
// -----------------------------------------------------------------------------
// DEPENDÊNCIAS EXTERNAS (Globals)
// Para transformar isso em uma lib pura, essas variáveis deveriam ser passadas 
// como argumentos ou classes de contexto. Mantive extern para compatibilidade.
// -----------------------------------------------------------------------------

// extern std::mt19937 rng;
// extern std::vector<rkolib::core::TSol> pool;
// extern std::atomic<bool> stop_execution;

// Função Decoder deve ser definida no problema específico (ex: KnapsackProblem.cpp)
// Declaramos aqui para que os métodos de busca possam chamá-la.
//double Decoder(const rkolib::core::TSol &s, const rkolib::core::IProblem &problem);

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
    void CreatePoolSolutions(const IProblem &problem, const int sizePool);
    void UpdatePoolSolutions(TSol s, const char* mh, const int debug);

    // -----------------------------------------------------------------------------
    // Local Searches & Metaheuristics Components
    // -----------------------------------------------------------------------------
    void ShakeSolution(TSol &s, float betaMin, float betaMax, const int n);
    
    TSol Blending(TSol &s1, TSol &s2, double factor, const int n);
    
    void NelderMeadSearch(TSol &x1, const IProblem &problem);

    void SwapLS(TSol &s, const IProblem &problem, const int &strategy, std::vector<int> &RKorder);
    void InvertLS(TSol &s, const IProblem &problem, const int &strategy, std::vector<int> &RKorder);
    void FareyLS(TSol &s, const IProblem &problem, const int &strategy, std::vector<int> &RKorder);
    
    void RVND(TSol &s, const IProblem &problem, const int &strategy, std::vector<int> &RKorder);

    // -----------------------------------------------------------------------------
    // IO / Config
    // -----------------------------------------------------------------------------
    //void readParameters(const char* method, int control, 
      //                   std::vector<std::vector<double>> &parameters, int numPar);

    void readParameters(const std::string& method, int control, 
                     std::vector<std::vector<double>> &parameters, int numPar);                 

    void readParametersYaml(const char* method, int control, 
                    std::vector<std::vector<double>> &parameters, int numPar);

} // namespace rkolib::core