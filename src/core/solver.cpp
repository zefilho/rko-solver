#include "rkolib/core/solver.hpp"
#include "rkolib/core/method.hpp" 

// CLI11 e OpenMP
#include <CLI/CLI.hpp>
#include <omp.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <atomic>
#include <mutex>
// Includes das Metaheurísticas
#include "rkolib/mh/brkga.hpp"
#include "rkolib/mh/sa.hpp"
#include "rkolib/mh/grasp.hpp"
#include "rkolib/mh/ils.hpp"
#include "rkolib/mh/vns.hpp"
#include "rkolib/mh/pso.hpp"
#include "rkolib/mh/ga.hpp"
#include "rkolib/mh/brkga_cs.hpp"
#include "rkolib/mh/lns.hpp"
#include "rkolib/mh/ipr.hpp"
#include "rkolib/mh/multistart.hpp"

// RNG thread-local: cada thread tem sua própria instância isolada
extern std::mt19937 rng;

// Pool compartilhado com proteção via mutex
extern std::vector<rkolib::core::TSol> pool;

// Flag atômica para controle de parada
extern std::atomic<bool> stop_execution;

namespace rkolib {

    RKOSolver::RKOSolver() {
        registerAlgorithms();
        stop_execution.store(false);
    }

    RKOSolver::~RKOSolver() {
        FreeMemoryProblem(problemData);
    }

    void RKOSolver::registerAlgorithms() {
        algo_registry["BRKGA"]      = rkolib::mh::BRKGA;
        algo_registry["SA"]         = rkolib::mh::SA;
        algo_registry["GRASP"]      = rkolib::mh::GRASP;
        algo_registry["ILS"]        = rkolib::mh::ILS;
        algo_registry["VNS"]        = rkolib::mh::VNS;
        algo_registry["PSO"]        = rkolib::mh::PSO;
        algo_registry["GA"]         = rkolib::mh::GA;
        algo_registry["LNS"]        = rkolib::mh::LNS;
        algo_registry["BRKGA-CS"]   = rkolib::mh::BRKGA_CS;
        algo_registry["MultiStart"] = rkolib::mh::MultiStart;
        algo_registry["IPR"]        = rkolib::mh::IPR;
    }

    int RKOSolver::init(int argc, char* argv[]) {
        CLI::App app{"RKO Library - Optimization Solver"};

        app.add_option("-i,--instance", config.instancePath, "Path to instance file")->required()->check(CLI::ExistingFile);
        app.add_option("-t,--time", config.maxTime, "Max execution time (seconds)")->default_val(60);
        app.add_option("-c,--config", config.configPath, "Path to configuration file")->default_val("config/config-tests.txt")->check(CLI::ExistingFile);
        app.add_option("-s,--seed", config.seed, "RNG Seed (default: random)");

        try {
            CLI11_PARSE(app, argc, argv);
            parseConfigFile();
            loadProblemData();
            config.runData.MAXTIME = config.maxTime; 
            return 0;
        } catch (const CLI::ParseError &e) {
            return app.exit(e);
        } catch (const std::exception &e) {
            std::cerr << "Initialization Error: " << e.what() << std::endl;
            return 1;
        }
    }

    void RKOSolver::parseConfigFile() {
        std::ifstream file(config.configPath);
        if (!file.is_open()) throw std::runtime_error("Config file not found.");

        std::string line, key;
        config.runData.MAXRUNS = 1;
        
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            ss >> key;

            if (algo_registry.count(key)) {
                config.activeAlgorithms.push_back(key);
            }
            else if (key == "MAXRUNS")  ss >> config.runData.MAXRUNS;
            else if (key == "debug")    ss >> config.runData.debug;
            else if (key == "control")  ss >> config.runData.control;
            else if (key == "sizePool") ss >> config.runData.sizePool;
        }
    }

    void RKOSolver::loadProblemData() {
        ReadData(config.instancePath.c_str(), problemData);
    }

    void RKOSolver::run() {
        if (config.activeAlgorithms.empty()) {
            std::cerr << "Error: No algorithms selected.\n";
            return;
        }

        rkolib::core::TSol bestSolutionGlobal;
        bestSolutionGlobal.ofv = std::numeric_limits<double>::infinity();

        std::cout << "Instance: " << config.instancePath << "\nRuns: ";

        // Base seed global
        unsigned int GLOBAL_SEED = (config.seed == -1) 
            ? std::chrono::steady_clock::now().time_since_epoch().count() 
            : (unsigned int)config.seed;

        for (int run = 0; run < config.runData.MAXRUNS; run++)
        {
            std::cout << (run + 1) << " " << std::flush;
            
            // Seed base para esta run
            unsigned int runSeed = GLOBAL_SEED + run;
            
            // Stats locais da run
            rkolib::core::TSol bestSolutionRun;
            bestSolutionRun.ofv = std::numeric_limits<double>::infinity();
            
            double start_time = rkolib::core::get_time_in_seconds();
            stop_execution.store(false);

            // Preparação Inicial do Pool (Execução única)
            ::pool.clear();
            ::pool.resize(config.runData.sizePool);
            
            // Garante seed local para a criação do pool
            rng.seed(runSeed); 
            rkolib::core::CreatePoolSolutions(problemData, config.runData.sizePool);
            
            if (!::pool.empty()) bestSolutionRun = ::pool[0];

            // Define número de threads igual ao número de algoritmos ativos
            omp_set_num_threads(config.activeAlgorithms.size());
            
            // REMOVIDO: private(rng) -> rng agora é thread_local global
            // REMOVIDO: shared(stop_execution) -> atomic é shared por padrão
            #pragma omp parallel private(rng) shared(pool, stop_execution)
            {
                // 1. Inicializa RNG Local da Thread
                // Cada thread (algoritmo) recebe uma seed única e determinística baseada no ID
                rng.seed(runSeed + omp_get_thread_num() + 1000);

                // Loop de Restart
                while ((rkolib::core::get_time_in_seconds() - start_time) < config.maxTime)
                {
                    // Reset flag apenas pela thread master
                    #pragma omp single
                    stop_execution.store(false);
                    
                    // Barreira implícita garante que todos viram o reset
                    #pragma omp barrier // Wait for reset
                    // Distribui os algoritmos entre as threads
                    // Usamos schedule(static, 1) para garantir que cada thread pegue 1 algoritmo fixo se num_threads == num_algos
                    #pragma omp for schedule(static, 1)
                    for (size_t i = 0; i < config.activeAlgorithms.size(); ++i) {
                        
                        // Debug seguro
                        if (config.runData.debug) {
                            #pragma omp critical
                            std::cout << "\n[T" << omp_get_thread_num() << "] Start: " << config.activeAlgorithms[i];
                        }

                        // EXECUTA O ALGORITMO
                        // Nota: O algoritmo deve checar 'stop_execution' internamente se quiser parar cedo
                        std::string name = config.activeAlgorithms[i];
                        algo_registry[name](config.runData, problemData);

                        // Se terminou, sinaliza (opcional, depende se a estratégia é "first to finish stops all")
                        stop_execution.store(true); 
                        #pragma omp cancel for
                    }
                    // Barreira Implícita aqui: Ninguém sai do 'omp for' até o mais lento terminar
                    // Isso evita que uma thread reinicie o pool enquanto outra ainda roda.

                    // Atualiza Melhor Solução da Run
                    #pragma omp critical
                    {
                         // Assume-se que o algoritmo atualizou a posição 0 do pool ou retornou algo.
                         // Como é legado, verificamos se pool[0] mudou.
                         if (!::pool.empty() && ::pool[0].ofv < bestSolutionRun.ofv) {
                             bestSolutionRun = ::pool[0];
                             bestSolutionRun.best_time = rkolib::core::get_time_in_seconds() - start_time;
                         }
                    }

                    // Se o tempo acabou, sai do while
                    if ((rkolib::core::get_time_in_seconds() - start_time) >= config.maxTime) break;

                    // Se ainda tem tempo, REINICIA O POOL (Restart Strategy)
                    // Feito apenas por uma thread enquanto as outras esperam
                    #pragma omp single
                    {
                        if (config.runData.debug) std::cout << " [Restarting Pool] ";
                        // Nova semente para o restart não ser idêntico
                        rng.seed(runSeed + (unsigned int)rkolib::core::get_time_in_seconds());
                        rkolib::core::CreatePoolSolutions(problemData, config.runData.sizePool);
                    }
                } 
            } // Fim Parallel

            // Atualiza Global Stats
            if (bestSolutionRun.ofv < bestSolutionGlobal.ofv) {
                bestSolutionGlobal = bestSolutionRun;
            }
        }

        std::cout << "\n\n=== FINAL RESULT ===\n";
        std::cout << "Best OFV: " << bestSolutionGlobal.ofv << "\n";
    }

} // namespace rkolib