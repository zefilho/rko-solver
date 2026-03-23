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

    void WriteSolutionScreen(const std::vector<std::string>& algorithms, const core::TSol& s, 
                             double timeBest, double timeTotal, const std::string& instance, 
                             int dimension, const std::vector<core::TSol>& pool) {
        
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

        std::cout << "\nSolution Pool (Top Elite):\n";
        for (const auto& sol : pool) {
            std::cout << std::format("{:.5f} [{}]\n", sol.ofv, sol.nameMH);
        }
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

} // namespace rkolib::utils