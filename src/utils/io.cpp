#include "rkolib/utils/io.hpp"
#include <filesystem>
#include <numeric>

namespace fs = std::filesystem;

namespace rkolib::utils {

    // Helper: Formats the list of algorithms as a single string (ex: "SA | VNS | ILS | ")
    std::string JoinAlgorithms(const std::vector<std::string>& algorithms) {
        std::string result;
        for (size_t i = 0; i < algorithms.size(); ++i) {
            result += algorithms[i];
            if (i < algorithms.size() - 1) {
                result += " | "; // Only adds the bar if it's not the last one
            }
        }
        return result;
    }

    // Helper: Ensures that the destination directory exists
    void EnsureDirectoryExists(const std::string& path) {
        if (!fs::exists(path)) {
            fs::create_directories(path);
        }
    }

    void PrintPoolSolutions(const std::vector<core::TSol>& pool, int dimension) {
        std::cout << "\n╔════════════════════════════════════════════════════════╗\n"
                  << "║                 POOL SOLUTIONS (ELITE)                 ║\n"
                  << "╚════════════════════════════════════════════════════════╝\n";
        std::cout << "Pool Size: " << pool.size() << "\n";

        for (size_t idx = 0; idx < pool.size(); ++idx) {
            const auto& sol = pool[idx];
            std::string mh = sol.nameMH.empty() ? "Init" : sol.nameMH;

            std::cout << std::format("\n[#{:02d}] OFV: {:.6f} | MH: {} | Best time: {:.3f}s\n",
                                     idx + 1, sol.ofv, mh, sol.best_time);

            std::cout << "  Objs: [ ";
            for (double val : sol.objs) {
                std::cout << std::format("{:.6f} ", val);
            }
            std::cout << "]\n";

            std::cout << "  RK:   ";
            int rkLimit = std::min(dimension, static_cast<int>(sol.rk.size()));
            for (int i = 0; i < rkLimit; ++i) {
                std::cout << std::format("{:.3f} ", sol.rk[i]);
            }
            std::cout << "\n";
        }
        std::cout << "══════════════════════════════════════════════════════════\n";
    }

    void WritePoolSolutions(const std::vector<std::string>& algorithms,
                            const std::vector<core::TSol>& pool,
                            const std::string& instance,
                            int dimension,
                            const std::string& outDir) {
        EnsureDirectoryExists(outDir);
        std::string filepath = outDir + "/Pool_Solutions_RKO.txt";

        std::ofstream poolFile(filepath, std::ios::app);
        if (!poolFile.is_open()) {
            std::cerr << "\n[Erro FATAL] Nao foi possivel abrir o arquivo: " << filepath << "\n";
            return;
        }

        poolFile << "\n\n==========================================================\n";
        poolFile << "Instance: " << instance << "\n";
        poolFile << "RKO: " << JoinAlgorithms(algorithms) << "\n";
        poolFile << "Pool Size: " << pool.size() << "\n";
        poolFile << "----------------------------------------------------------\n";

        for (size_t idx = 0; idx < pool.size(); ++idx) {
            const auto& sol = pool[idx];
            std::string mh = sol.nameMH.empty() ? "Init" : sol.nameMH;

            poolFile << std::format("[#{:02d}] OFV: {:.6f} | MH: {} | Best time: {:.3f}s\n",
                                    idx + 1, sol.ofv, mh, sol.best_time);

            poolFile << "Objs: [ ";
            for (double val : sol.objs) {
                poolFile << std::format("{:.6f} ", val);
            }
            poolFile << "]\n";

            poolFile << "RK: ";
            int rkLimit = std::min(dimension, static_cast<int>(sol.rk.size()));
            for (int i = 0; i < rkLimit; ++i) {
                poolFile << std::format("{:.3f} ", sol.rk[i]);
            }
            poolFile << "\n----------------------------------------------------------\n";
        }
    }

    void WriteSolutionScreen(const std::vector<std::string>& algorithms, const core::TSol& s, 
                             double timeBest, double timeTotal, const std::string& instance, 
                             int dimension, [[maybe_unused]] const std::vector<core::TSol>& pool) {
        
        std::cout << "\n\n========================================\n";
        std::cout << "RKO: " << JoinAlgorithms(algorithms) << "\n";
        std::cout << "Best MH: " << s.nameMH << "\n";
        std::cout << "Instance: " << instance << "\n";
        
        std::cout << "Sol (RK): ";
        for (int i = 0; i < dimension; i++) {
            std::cout << std::format("{:.3f} ", s.rk[i]);
        }

        std::cout << std::format("\nOFV: {:.5f}", s.ofv);
        std::cout << std::format("\nTotal time: {:.3f}s", timeTotal);
        std::cout << std::format("\nBest time: {:.3f}s\n", timeBest);
        std::cout << "========================================\n\n";
    }

    void WriteSolution(const std::vector<std::string>& algorithms, const core::TSol& s, 
                       double timeBest, double timeTotal, const std::string& instance, 
                       int dimension, const std::string& outDir) {
        
        EnsureDirectoryExists(outDir);
        std::string filepath = outDir + "/Solutions_RKO.txt";
        
        // std::ios::app opens in "append" mode (adds to the end of the file without overwriting)
        std::ofstream solFile(filepath, std::ios::app);
        
        if (!solFile.is_open()) {
            std::cerr << "\n[Erro FATAL] Nao foi possivel abrir o arquivo: " << filepath << "\n";
            return;
        }

        solFile << "\n\nInstance: " << instance << "\n";
        solFile << "RKO: " << JoinAlgorithms(algorithms) << "\n";
        
        solFile << "Sol: ";
        for (int i = 0; i < dimension; i++) {
            solFile << std::format("{:.3f} ", s.rk[i]);
        }

        solFile << std::format("\nOFV: {:.6f}\n", s.ofv);
        solFile << std::format("Best time: {:.3f}s\n", timeBest);
        solFile << std::format("Total time: {:.3f}s\n", timeTotal);
    }

    void WriteResults(const std::vector<std::string>& algorithms, double ofv, 
                      double ofvAverage, const std::vector<double>& ofvs, 
                      double timeBest, double timeTotal, const std::string& instance, 
                      const std::string& outDir) {
        
        EnsureDirectoryExists(outDir);
        std::string filepath = outDir + "/Results_RKO.csv";
        
        bool isNewFile = !fs::exists(filepath);
        std::ofstream file(filepath, std::ios::app);

        if (!file.is_open()) {
            std::cerr << "\n[Erro FATAL] Nao foi possivel abrir o arquivo: " << filepath << "\n";
            return;
        }

        // If the file was just created, inject the CSV header  
        if (isNewFile) {
            file << "Instance,Algorithms,Num_Runs,All_OFVs...,Best_OFV,Average_OFV,Best_Time,Total_Time\n";
        }

        // CSV format (using comma or semicolon as separator)
        file << instance << "," << JoinAlgorithms(algorithms) << "," << ofvs.size() << ",";
        
        // Concatenate all the results of the rounds (independent runs)
        for (size_t i = 0; i < ofvs.size(); ++i) {
            file << std::format("{:.6f}", ofvs[i]);
            if (i < ofvs.size() - 1) {
                file << "|";
            }
        }

        file << std::format(",{:.6f},{:.6f},{:.3f},{:.3f}\n", ofv, ofvAverage, timeBest, timeTotal);
    }

    void WriteMultiObjectiveResults(const std::vector<std::string>& algorithms, 
                                double scalarOfv, 
                                const std::vector<double>& realObjs, // <-- Adicionado o vetor de objetivos reais
                                double ofvAverage, 
                                const std::vector<double>& ofvs, 
                                double timeBest, double timeTotal, 
                                const std::string& instance, 
                                const std::string& outDir) {
    
    EnsureDirectoryExists(outDir);
    // Cria um arquivo separado para não quebrar a formatação das instâncias Mono-objetivo
    std::string filepath = outDir + "/Results_MO_RKO.csv";
    
    bool isNewFile = !std::filesystem::exists(filepath);
    std::ofstream file(filepath, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "\n[Erro FATAL] Nao foi possivel abrir o arquivo: " << filepath << "\n";
        return;
    }

    // Se o arquivo for novo, injetamos o cabeçalho CSV com a coluna Real_Objectives
    if (isNewFile) {
        file << "Instance,Algorithms,Num_Runs,All_Scalar_OFVs,Best_Scalar_OFV,Real_Objectives,Average_Scalar_OFV,Best_Time,Total_Time\n";
    }

    file << instance << "," << JoinAlgorithms(algorithms) << "," << ofvs.size() << ",";
    
    // Concatena todos os resultados das rodadas (OFV Escalarizado)
    for (size_t i = 0; i < ofvs.size(); ++i) {
        file << std::format("{:.6f}", ofvs[i]);
        if (i < ofvs.size() - 1) {
            file << "|";
        }
    }

    // Grava o Melhor OFV Escalarizado
    file << "," << std::format("{:.6f}", scalarOfv) << ",";

    // Concatena os Objetivos Reais separados por pipe (|)
    // Exemplo: 120.50|0.45 (Custo | Gini)
    for (size_t i = 0; i < realObjs.size(); ++i) {
        file << std::format("{:.6f}", realObjs[i]);
        if (i < realObjs.size() - 1) {
            file << "|";
        }
    }

    // Finaliza com Média e Tempos
    file << std::format(",{:.6f},{:.3f},{:.3f}\n", ofvAverage, timeBest, timeTotal);
}

    // Em rkolib::utils
    void WriteConvergenceLog(const std::vector<core::ConvergencePoint>& history, const std::string& outDir) {
        std::cout << "[Info] Escrevendo log de convergencia...\n";
        EnsureDirectoryExists(outDir);
        std::ofstream file(outDir + "/Convergence_Log.csv");
        
        file << "Time,MH,OFV\n";
        for (const auto& point : history) {
            file << std::format("{:.4f},{},{:.6f}\n", point.timestamp, point.mh_name, point.ofv);
        }
    }

} // namespace rkolib::utils