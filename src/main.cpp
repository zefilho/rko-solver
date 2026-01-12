/**
 * RKO Library - Main Entry Point
 * Organized by: Senior C++ Assistant
 */

// -----------------------------------------------------------------------------
// 1. SYSTEM HEADERS (Cross-Platform)
// -----------------------------------------------------------------------------
#if defined(_WIN32) || defined(_WIN64)
    #include <windows.h>
#else
    #include <sys/time.h>
    #include <time.h>
#endif

// CLI
#include <CLI/CLI.hpp>

// Parallelism
#include <omp.h>

// -----------------------------------------------------------------------------
// 2. PROJECT HEADERS
// -----------------------------------------------------------------------------
#include "rkolib/core/data.hpp" // Definitions of rkolib::core::TRunData, TSol, etc.
#include "rkolib/core/problem.hpp"
#include "rkolib/core/method.hpp" 
//#include "rkolib/utils/io.hpp"

// Metaheuristics
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

// -----------------------------------------------------------------------------
// 3. GLOBAL CONTEXT (Thread-Safe Shared State)
// -----------------------------------------------------------------------------
std::mt19937 rng;
std::atomic<bool> stop_execution(false);
std::vector<rkolib::core::TSol> pool;

// -----------------------------------------------------------------------------
// 4. ALGORITHM REGISTRY
// -----------------------------------------------------------------------------
#define TOTAL_MH 11

// Function pointer type alias for cleanliness
using MetaheuristicFunc = void (*)(const rkolib::core::TRunData &, const rkolib::core::TProblemData &);

// Registry arrays
const char* ALL_ALGO_NAMES[TOTAL_MH] = {
    "BRKGA", "SA", "GRASP", "ILS", "VNS", "PSO", 
    "GA", "LNS", "BRKGA-CS", "MultiStart", "IPR"
};

MetaheuristicFunc ALL_ALGO_FUNCS[TOTAL_MH] = {
    rkolib::mh::BRKGA,    rkolib::mh::SA, 
    rkolib::mh::GRASP,    rkolib::mh::ILS, 
    rkolib::mh::VNS,      rkolib::mh::PSO,
    rkolib::mh::GA,       rkolib::mh::LNS, 
    rkolib::mh::BRKGA_CS, rkolib::mh::MultiStart, 
    rkolib::mh::IPR
};

// Active algorithms for current run
#define MAX_ACTIVE_MH 100
MetaheuristicFunc active_functions[MAX_ACTIVE_MH];
const char* active_names[MAX_ACTIVE_MH];
int num_active_mh = 0;

// -----------------------------------------------------------------------------
// 5. HELPER FUNCTIONS
// -----------------------------------------------------------------------------
void LoadConfiguration(const std::string& configFile, rkolib::core::TRunData& runData) {
    std::ifstream file(configFile);
    if (!file.is_open()) {
        std::cerr << "\nERROR: Configuration file " << configFile << " not found.\n";
        exit(EXIT_FAILURE);
    }

    std::string line, key;
    
    // Defaults
    runData.MAXRUNS = 1;
    runData.debug = 0;
    runData.control = 0;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue; // Skip comments/empty

        std::stringstream ss(line);
        ss >> key;

        // Check for Algorithms
        for (int i = 0; i < TOTAL_MH; ++i) {
            if (key == ALL_ALGO_NAMES[i]) {
                if (num_active_mh < MAX_ACTIVE_MH) {
                    active_functions[num_active_mh] = ALL_ALGO_FUNCS[i];
                    active_names[num_active_mh] = ALL_ALGO_NAMES[i];
                    num_active_mh++;
                }
            }
        }

        // Check for Parameters
        if (key == "MAXRUNS")  ss >> runData.MAXRUNS;
        else if (key == "debug")    ss >> runData.debug;
        else if (key == "control")  ss >> runData.control;
        else if (key == "strategy") ss >> runData.strategy;
        else if (key == "restart")  ss >> runData.restart;
        else if (key == "sizePool") ss >> runData.sizePool;
    }
}

// -----------------------------------------------------------------------------
// 6. MAIN EXECUTION
// -----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    // 1. Argument Validation
    // if (argc < 3) {
    //     std::cerr << "Usage: " << argv[0] << " <instance_path> <max_time_seconds>\n";
    //     return 1;
    // }

    // // 2. Setup Run Data
    // rkolib::core::TRunData runData;
    // std::string instanceName = argv[1];
    // runData.MAXTIME = std::stoi(argv[2]);
    // -------------------------------------------------------------------------
    // 1. CLI11 SETUP (Substitui a validação manual de argc)
    // -------------------------------------------------------------------------
    CLI::App app{"RKO Library - Optimization Solver"};

    // Variáveis temporárias para capturar os argumentos
    std::string instanceName;
    int maxTimeArg = 60;
    std::string configPath = "config/config-tests.txt"; // Valor default original

    // Definição das opções
    // -i ou --instance: Obrigatório
    app.add_option("-i,--instance", instanceName, "Path to instance file")
       ->required()
       ->check(CLI::ExistingFile);

    // -t ou --time: Obrigatório (conforme seu código original pedia argv[2])
    app.add_option("-t,--time", maxTimeArg, "Max execution time (seconds)")
       ->required();

    // -c ou --config: Opcional (adicionei para flexibilidade, mas mantém o default)
    app.add_option("-c,--config", configPath, "Path to configuration file")
       ->check(CLI::ExistingFile);

    // Parse (Trata erros e help automaticamente)
    CLI11_PARSE(app, argc, argv);

    // -------------------------------------------------------------------------
    // 2. Setup Run Data (Mantendo variáveis originais)
    // -------------------------------------------------------------------------
    rkolib::core::TRunData runData;
    
    // Passando os valores capturados pelo CLI11 para a estrutura original
    runData.MAXTIME = maxTimeArg;

    // 3. Load Configuration
    LoadConfiguration("config/config-tests.txt", runData);

    // 4. Load Problem Data
    rkolib::core::TProblemData data;
    ReadData(instanceName.c_str(), data); // Assuming ReadData takes char*

    // 5. Statistics tracking
    double foBest = std::numeric_limits<double>::infinity();
    double foAverage = 0.0;
    float timeBest = 0.0;
    float timeTotal = 0.0;
    std::vector<double> ofvs;

    rkolib::core::TSol bestSolutionGlobal;
    bestSolutionGlobal.ofv = std::numeric_limits<double>::infinity();

    std::cout << "\n\nInstance: " << instanceName << "\nRun: ";

    // -------------------------------------------------------------------------
    // MAIN LOOP (MAXRUNS)
    // -------------------------------------------------------------------------
    for (int run = 0; run < runData.MAXRUNS; run++)
    {
        std::cout << (run + 1) << " " << std::flush;

        // A. Seed Initialization
        unsigned int RSEED = std::chrono::steady_clock::now().time_since_epoch().count();
        if (runData.debug) RSEED = 1234; // Deterministic debug
        rng.seed(RSEED);

        // B. Run Initialization
        rkolib::core::TSol bestSolutionRun;
        bestSolutionRun.ofv = std::numeric_limits<double>::infinity();
        
        double start_time = rkolib::core::get_time_in_seconds();
        double end_time = start_time;
        
        // C. Parallel Metaheuristic Execution (OpenMP)
        stop_execution.store(false);

        if (num_active_mh > 0) {
            // Initialize Shared Pool
            pool.clear();
            pool.resize(runData.sizePool);
            CreatePoolSolutions(data, runData.sizePool);
            bestSolutionRun = pool[0]; // Assume initial best

            // Parallel Region
            omp_set_num_threads(num_active_mh);
            
            #pragma omp parallel private(rng) shared(pool, stop_execution)
            {
                // Each thread gets its own RNG seed based on thread ID + Global Seed
                rng.seed(RSEED + omp_get_thread_num());

                while ((rkolib::core::get_time_in_seconds() - start_time) < runData.MAXTIME)
                {
                    // Reset cancellation flag for restart loop
                    if (omp_get_thread_num() == 0) {
                        stop_execution.store(false);
                    }
                    #pragma omp barrier // Wait for reset

                    #pragma omp for
                    for (int i = 0; i < num_active_mh; ++i) {
                        
                        // Check for external cancellation
                        #pragma omp cancellation point for

                        if (runData.debug) {
                            #pragma omp critical
                            std::cout << "\nThread " << omp_get_thread_num() 
                                      << " executing " << active_names[i] 
                                      << " [" << (rkolib::core::get_time_in_seconds() - start_time) << "s]";
                        }

                        // EXECUTE ALGORITHM
                        active_functions[i](runData, data);

                        // If an algorithm finishes (converged or time out internally), 
                        // signal others to stop this iteration
                        stop_execution.store(true);
                        
                        #pragma omp cancel for
                    }

                    // Iteration cleanup
                    double current_time = rkolib::core::get_time_in_seconds();
                    
                    // Update run best (Critical section implicit if updating shared best via pool logic, 
                    // but here we read safely after barrier or rely on atomic updates in MHs)
                    if (pool[0].ofv < bestSolutionRun.ofv) {
                        bestSolutionRun = pool[0];
                    }

                    // Restart Logic (If time remains)
                    if ((current_time - start_time) < runData.MAXTIME) {
                        #pragma omp single
                        {
                            CreatePoolSolutions(data, runData.sizePool);
                        }
                    }
                }
            } // End Parallel
            
            // Final flag reset
            stop_execution.store(false);
        } 
        else {
            std::cerr << "\nERROR: No solver implemented or selected in config.\n";
            exit(1);
        }

        // D. Update Global Stats
        end_time = rkolib::core::get_time_in_seconds();
        
        if (bestSolutionRun.ofv < bestSolutionGlobal.ofv) {
            bestSolutionGlobal = bestSolutionRun;
        }

        if (bestSolutionRun.ofv < foBest) {
            foBest = bestSolutionRun.ofv;
        }

        foAverage += bestSolutionRun.ofv;
        ofvs.push_back(bestSolutionRun.ofv);
        
        timeBest += (bestSolutionRun.best_time - start_time); // Assuming best_time is absolute timestamp
        timeTotal += (end_time - start_time);
    }

    // -------------------------------------------------------------------------
    // 7. OUTPUT & CLEANUP
    // -------------------------------------------------------------------------
    foAverage /= runData.MAXRUNS;
    timeBest /= runData.MAXRUNS;
    timeTotal /= runData.MAXRUNS;

    // if (!runData.debug) {
    //     rkolib::utils::WriteSolution(active_names, num_active_mh, bestSolutionGlobal, timeBest, timeTotal, instanceName.c_str(), data);
    //     rkolib::utils::WriteResults(active_names, num_active_mh, foBest, foAverage, ofvs, timeBest, timeTotal, instanceName.c_str());
    // } else {
    //     rkolib::utils::WriteSolutionScreen(active_names, num_active_mh, bestSolutionGlobal, timeBest, timeTotal, instanceName.c_str(), data, pool);
    // }

    FreeMemoryProblem(data);

    return 0;
}