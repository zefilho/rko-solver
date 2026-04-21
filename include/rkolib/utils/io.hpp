#pragma once

#include "rkolib/core/data.hpp"

namespace rkolib::utils {

    // Imprime o sumário no terminal
    void WriteSolutionScreen(const std::vector<std::string>& algorithms, 
                             const core::TSol& s, 
                             double timeBest, 
                             double timeTotal, 
                             const std::string& instance, 
                             int dimension, 
                             const std::vector<core::TSol>& pool);

    // Salva o log detalhado da melhor solução (Random Keys incluídas)
    void WriteSolution(const std::vector<std::string>& algorithms, 
                       const core::TSol& s, 
                       double timeBest, 
                       double timeTotal, 
                       const std::string& instance, 
                       int dimension, 
                       const std::string& outDir = "../results");

    // Salva a linha de estatísticas consolidadas em CSV
    void WriteResults(const std::vector<std::string>& algorithms, 
                      double ofv, 
                      double ofvAverage, 
                      const std::vector<double>& ofvs, 
                      double timeBest, 
                      double timeTotal, 
                      const std::string& instance, 
                      const std::string& outDir = "../results");
    
    void WriteConvergenceLog(const std::vector<core::ConvergencePoint>& history, const std::string& outDir);

} // namespace rkolib::utils