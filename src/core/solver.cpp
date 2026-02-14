/**
 * RKO Library - Elegant Solver Implementation
 */

#include "rkolib/core/solver.hpp"
#include "rkolib/core/context.hpp"

// Includes das Meta-heurísticas
// Certifique-se de que estas funções agora aceitam (TRunData, const IProblem&)
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

#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <omp.h>
#include <yaml-cpp/yaml.h>

namespace rkolib {

    // -------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR
    // -------------------------------------------------------------------------
    RkoSolver::RkoSolver() 
        : instancePath_("")
        , configPath_("config/yaml/config.yaml")
        , problemInstance_(nullptr)
        , numActiveMethods_(0)
        , bestObjective_(std::numeric_limits<double>::infinity())
        , averageObjective_(0.0)
        , bestTime_(0.0)
        , totalTime_(0.0)
    {
        initializeRegistry();
    }

    RkoSolver::~RkoSolver() {
        cleanup();
    }

    // -------------------------------------------------------------------------
    // INITIALIZATION
    // -------------------------------------------------------------------------
    void RkoSolver::initializeRegistry() {
        // Mapeia nomes (strings) para as funções das meta-heurísticas.
        // As funções devem ter a assinatura: void func(const TRunData&, const IProblem&)
        algorithmRegistry_ = {
            {"BRKGA",      mh::BRKGA},
            {"SA",         mh::SA},
            {"GRASP",      mh::GRASP},
            {"ILS",        mh::ILS},
            {"VNS",        mh::VNS},
            {"PSO",        mh::PSO},
            {"GA",         mh::GA},
            {"LNS",        mh::LNS},
            {"BRKGA-CS",   mh::BRKGA_CS},
            {"MultiStart", mh::MultiStart},
            {"IPR",        mh::IPR}
        };
    }

    // -------------------------------------------------------------------------
    // ARGUMENT PARSING
    // -------------------------------------------------------------------------
    void RkoSolver::parseArguments(int argc, char* argv[]) {
        CLI::App app{"RKO Library - Optimization Solver"};

        app.add_option("-i,--instance", instancePath_, "Path to instance file")
           ->required()
           ->check(CLI::ExistingFile);

        app.add_option("-t,--time", runData_.MAXTIME, "Max execution time (seconds)")
           ->required();

        app.add_option("-c,--config", configPath_, "Path to configuration file")
           ->check(CLI::ExistingFile);

        try {
            app.parse(argc, argv);
        } catch (const CLI::ParseError& e) {
            app.exit(e);
            throw; // Relança para interromper a execução no main
        }
    }

    // -------------------------------------------------------------------------
    // CONFIGURATION LOADING
    // -------------------------------------------------------------------------
    void RkoSolver::loadConfiguration(const std::string& configFile) {
        std::string targetConfig = configFile.empty() ? configPath_ : configFile;
        
        YAML::Node config;
        try {
            config = YAML::LoadFile(targetConfig);
        } catch (const std::exception& e) {
            throw std::runtime_error("Erro ao carregar YAML (" + targetConfig + "): " + e.what());
        }

        // 1. Limpa seleção anterior
        activateMethods_.clear();
        numActiveMethods_ = 0;

        // 2. Lê Meta-heurísticas
        if (config["metaheuristics"]) {
            for (const auto& mh_node : config["metaheuristics"]) {
                std::string name = mh_node.as<std::string>();
                
                bool found = false;
                for (size_t i = 0; i < algorithmRegistry_.size(); ++i) {
                    if (name == algorithmRegistry_[i].name) {
                        activateMethods_.push_back(static_cast<int>(i));
                        numActiveMethods_++;
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    std::cerr << "[Warning] Algoritmo desconhecido no YAML: " << name << std::endl;
                }
            }
        }

        // 3. Lê Configurações de Execução
        if (auto settings = config["execution_settings"]) {
            runData_.MAXRUNS = settings["max_runs"].as<int>(1);
            runData_.debug   = settings["debug_mode"].as<int>(0);
            runData_.control = settings["control_mode"].as<int>(0);
        }

        // 4. Lê Parâmetros de Busca
        if (auto params = config["search_parameters"]) {
            runData_.strategy = params["local_search_strategy"].as<int>(1);
            runData_.restart  = params["restart_threshold"].as<float>(1.0f);
            runData_.sizePool = params["elite_pool_size"].as<int>(10);
        }
    }

    // -------------------------------------------------------------------------
    // MAIN EXECUTION LOOP
    // -------------------------------------------------------------------------
    void RkoSolver::run() {
        validateConfiguration();
        
        // Carrega o problema (Via Factory/Singleton)
        loadProblemData();
        
        initializeStatistics();
        
        // Banner Informativo
        std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
        std::cout << "║              RKO Solver - Optimization Framework       ║\n";
        std::cout << "╚════════════════════════════════════════════════════════╝\n";
        std::cout << "\n[Info] Instance: " << instancePath_
                  << "\n[Info] Methods:  " << numActiveMethods_
                  << "\n[Info] Runs:     " << runData_.MAXRUNS
                  << "\n[Info] Time Limit: " << runData_.MAXTIME << "s"
                  << "\n\nProgress: " << std::flush;

        for (int run = 0; run < runData_.MAXRUNS; ++run) {
            std::cout << "[" << (run + 1) << "] " << std::flush;
            executeRun(run);
        }

        computeFinalStatistics();
        displayResults();
    }

    void RkoSolver::validateConfiguration() {
        if (numActiveMethods_ == 0) {
            throw std::runtime_error("Nenhum método de otimização selecionado.");
        }
        if (instancePath_.empty()) {
            throw std::runtime_error("Caminho da instância não fornecido.");
        }
    }

    // -------------------------------------------------------------------------
    // PROBLEM LOADING (SINGLETON/FACTORY INTEGRATION)
    // -------------------------------------------------------------------------
    void RkoSolver::loadProblemData() {
        // Chama a fábrica global definida pelo usuário (via problem_interface.hpp)
        problemInstance_ = core::createProblem();

        if (!problemInstance_) {
            throw std::runtime_error("Falha fatal: createProblem() retornou nulo.");
        }

        try {
            problemInstance_->load(instancePath_);
        } catch (const std::exception& e) {
            throw std::runtime_error("Erro ao carregar dados do problema: " + std::string(e.what()));
        }
    }

    // -------------------------------------------------------------------------
    // RUN EXECUTION
    // -------------------------------------------------------------------------
    void RkoSolver::executeRun(int runIndex) {
        auto& ctx = core::SolverContext::instance();
        
        // Define Semente
        unsigned int seed = (runData_.debug) 
            ? (1234 + runIndex) 
            : static_cast<unsigned int>(std::chrono::steady_clock::now().time_since_epoch().count());
        
        ctx.setSeed(seed);

        // Inicializa melhor solução local da run
        core::TSol bestSolutionRun;
        bestSolutionRun.ofv = std::numeric_limits<double>::infinity();
        
        double startTime = omp_get_wtime(); // Usando timer do OpenMP ou utils::get_time
        
        ctx.resetStopFlag();
        
        // Inicializa Pool de Soluções
        // IMPORTANTE: Agora passamos *problemInstance_ (interface)
        // Certifique-se que CreatePoolSolutions aceita (const IProblem&, int)
        ctx.initializePool(runData_.sizePool);
        core::CreatePoolSolutions(*problemInstance_, runData_.sizePool); 
        
        // Recupera a melhor do pool inicial
        bestSolutionRun = ctx.getBestSolution();

        // Executa Meta-heurísticas em Paralelo
        executeParallelMethods(startTime, bestSolutionRun, seed);

        double endTime = omp_get_wtime();
        updateStatistics(bestSolutionRun, startTime, endTime);
    }

    void RkoSolver::executeParallelMethods(double startTime, core::TSol& bestSolutionRun, unsigned int baseSeed) {
        auto& ctx = core::SolverContext::instance();
        
        // Configura número de threads
        omp_set_num_threads(numActiveMethods_);

        #pragma omp parallel
        {
            // Loop principal de tempo
            while ((omp_get_wtime() - startTime) < runData_.MAXTIME) {
                
                #pragma omp single
                { ctx.resetStopFlag(); }
                
                // Barreira implícita no single, ou explícita aqui se necessário
                #pragma omp barrier

                // Distribui os métodos entre as threads
                #pragma omp for
                for (int i = 0; i < numActiveMethods_; ++i) {
                    
                    // Ponto de cancelamento (se suportado pelo compilador/env)
                    #pragma omp cancellation point for

                    // EXECUÇÃO DO MÉTODO
                    // Recupera a função do registro e executa passando a Interface
                    try {
                        getActiveFunction(i)(runData_, *problemInstance_);
                    } catch (...) {
                        // Captura exceções para não derrubar todas as threads
                        #pragma omp critical
                        std::cerr << "Erro na thread " << omp_get_thread_num() << std::endl;
                    }

                    // Sinaliza para outros pararem (se lógica cooperativa existir)
                    ctx.signalStop();
                    
                    #pragma omp cancel for
                }

                // Zona Crítica/Single para atualizar melhor solução e reiniciar
                #pragma omp single
                {
                    // Atualiza a melhor local da run
                    core::TSol ctxBest = ctx.getBestSolution();
                    if (ctxBest.ofv < bestSolutionRun.ofv) {
                        bestSolutionRun = ctxBest;
                    }

                    // Se ainda tem tempo, reinicia o pool (Multi-Start behavior)
                    if ((omp_get_wtime() - startTime) < runData_.MAXTIME) {
                        core::CreatePoolSolutions(*problemInstance_, runData_.sizePool);
                    }
                }
            }
        }
        ctx.resetStopFlag();
    }

    // -------------------------------------------------------------------------
    // STATISTICS & RESULTS
    // -------------------------------------------------------------------------
    void RkoSolver::initializeStatistics() {
        bestObjective_ = std::numeric_limits<double>::infinity();
        averageObjective_ = 0.0;
        totalTime_ = 0.0;
        bestTime_ = 0.0;
        bestSolutionGlobal_.ofv = std::numeric_limits<double>::infinity();
        objectiveValues_.clear();
        objectiveValues_.reserve(runData_.MAXRUNS);
    }

    void RkoSolver::updateStatistics(const core::TSol& runSolution, double startTime, double endTime) {
        double runTime = endTime - startTime;

        if (runSolution.ofv < bestSolutionGlobal_.ofv) {
            bestSolutionGlobal_ = runSolution;
        }
        
        if (runSolution.ofv < bestObjective_) {
            bestObjective_ = runSolution.ofv;
            bestTime_ = runTime; // Tempo em que a melhor global foi encontrada (aprox)
        }

        averageObjective_ += runSolution.ofv;
        objectiveValues_.push_back(runSolution.ofv);
        totalTime_ += runTime;
    }

    void RkoSolver::computeFinalStatistics() {
        if (runData_.MAXRUNS > 0) {
            averageObjective_ /= runData_.MAXRUNS;
            // totalTime_ aqui representa a soma dos tempos de execução.
            // Se quiser a média de tempo por run:
            totalTime_ /= runData_.MAXRUNS; 
        }
    }

    void RkoSolver::displayResults() {
        std::cout << "\n\n╔════════════════════════════════════════════════════════╗\n"
                  << "║                    RESULTS SUMMARY                     ║\n"
                  << "╚════════════════════════════════════════════════════════╝\n";
        
        std::cout << std::fixed << std::setprecision(5);
        std::cout << "Best Objective:    " << bestObjective_ << "\n"
                  << "Average Objective: " << averageObjective_ << "\n"
                  << "Avg Run Time:      " << totalTime_ << "s\n";
        
        // Exibir solução detalhada se desejar
        // std::cout << "Solution Vector: ";
        // for(auto v : bestSolutionGlobal_.rk) std::cout << v << " ";
        // std::cout << "\n";
    }

    void RkoSolver::cleanup() {
        // Com shared_ptr, a limpeza do problemInstance_ é automática.
        problemInstance_.reset();
    }

    // -------------------------------------------------------------------------
    // HELPER ACCESSORS
    // -------------------------------------------------------------------------
    std::string RkoSolver::getActiveName(int index) const {
        if (index < 0 || index >= numActiveMethods_) return "Unknown";
        int registryIndex = activateMethods_[index];
        return algorithmRegistry_[registryIndex].name;
    }

    std::function<void(const core::TRunData&, const core::IProblem&)> 
    RkoSolver::getActiveFunction(int index) const {
        int registryIndex = activateMethods_[index];
        return algorithmRegistry_[registryIndex].function;
    }

} // namespace rkolib