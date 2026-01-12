#pragma once

#include <memory>

// Dependências internas
#include "rkolib/core/data.hpp"
#include "rkolib/core/problem.hpp"

namespace rkolib {

    // Estrutura para armazenar configurações da execução
    struct AppConfig {
        std::string instancePath;
        std::string configPath = "config/config-tests.txt";
        int maxTime = 60;
        int seed = -1; 
        
        // Dados internos
        std::vector<std::string> activeAlgorithms;
        rkolib::core::TRunData runData;
    };

    class RKOSolver {
    public:
        // Construtor
        RKOSolver();
        ~RKOSolver();

        // 1. Configura via linha de comando e carrega arquivos
        int init(int argc, char* argv[]);

        // 2. O Entry-Point principal solicitado
        void run();

    private:
        // Métodos auxiliares
        void parseConfigFile();
        void loadProblemData();
        void registerAlgorithms();

        // Membros de Estado
        AppConfig config;
        rkolib::core::TProblemData problemData;
        
        // Estado compartilhado (Thread-safe logic)
        std::vector<rkolib::core::TSol> pool;
        std::atomic<bool> stop_execution;

        // Registro de Algoritmos (Nome -> Função)
        using MetaheuristicFunc = std::function<void(const rkolib::core::TRunData &, const rkolib::core::TProblemData &)>;
        std::map<std::string, MetaheuristicFunc> algo_registry;
    };

} // namespace rkolib