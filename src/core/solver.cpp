/**
 * RKO Library - Elegant Solver Implementation
 */

#include "rkolib/core/solver.hpp"
#include "rkolib/core/context.hpp"
#include "rkolib/utils/io.hpp"

// Includes das Meta-heurísticas
#include "rkolib/mh/brkga.hpp"
#include "rkolib/mh/brkga_cs.hpp"
#include "rkolib/mh/ga.hpp"
#include "rkolib/mh/grasp.hpp"
#include "rkolib/mh/ils.hpp"
#include "rkolib/mh/ipr.hpp"
#include "rkolib/mh/lns.hpp"
#include "rkolib/mh/multistart.hpp"
#include "rkolib/mh/pso.hpp"
#include "rkolib/mh/sa.hpp"
#include "rkolib/mh/vns.hpp"

#include <CLI/CLI.hpp>
#include <iostream>
#include <numeric>
#include <algorithm> // Necessário para std::min
#include <omp.h>
#include <yaml-cpp/yaml.h>

// Headers for dynamic library loading
#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace rkolib {

// -------------------------------------------------------------------------
// CONSTRUCTOR & DESTRUCTOR
// -------------------------------------------------------------------------
RkoSolver::RkoSolver()
    : instancePath_(""), configPath_("config/yaml/config.yaml"),
      problemLibPath_(""), // Path to the plugin
      libHandle_(nullptr), // Handle of the plugin in memory
      problemInstance_(nullptr), numActiveMethods_(0),
      bestObjective_(std::numeric_limits<double>::infinity()),
      averageObjective_(0.0), bestTime_(0.0), totalTime_(0.0),
      scalarizer_(std::make_unique<core::TchebycheffScalarizer>()) {
  initializeRegistry();
}

RkoSolver::~RkoSolver() {
  cleanup();
  unloadProblemLibrary(); // Ensures the DLL/.so is unloaded
}

// -------------------------------------------------------------------------
// INITIALIZATION
// -------------------------------------------------------------------------
void RkoSolver::initializeRegistry() {
  algorithmRegistry_ = {{"BRKGA", mh::BRKGA},
                        {"SA", mh::SA},
                        {"GRASP", mh::GRASP},
                        {"ILS", mh::ILS},
                        {"VNS", mh::VNS},
                        {"PSO", mh::PSO},
                        {"GA", mh::GA},
                        {"LNS", mh::LNS},
                        {"BRKGA-CS", mh::BRKGA_CS},
                        {"MultiStart", mh::MultiStart},
                        {"IPR", mh::IPR}};
}

// -------------------------------------------------------------------------
// ARGUMENT PARSING
// -------------------------------------------------------------------------
void RkoSolver::parseArguments(int argc, char *argv[]) {
  CLI::App app{"RKO Library - Optimization Solver"};

  app.add_option("-i,--instance", instancePath_, "Path to instance file")
      ->required()
      ->check(CLI::ExistingFile);

  app.add_option("-t,--time", runData_.MAXTIME, "Max execution time (seconds)")
      ->required();

  app.add_option("-c,--config", configPath_, "Path to configuration file")
      ->check(CLI::ExistingFile);

  // Argument for the problem plugin
  app.add_option("-p,--plugin", problemLibPath_,
                 "Path to problem plugin (.so / .dll)")
      ->required()
      ->check(CLI::ExistingFile);

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError &e) {
    app.exit(e);
    throw;
  }
}

// -------------------------------------------------------------------------
// CONFIGURATION LOADING
// -------------------------------------------------------------------------
void RkoSolver::loadConfiguration(const std::string &configFile) {
  std::string targetConfig = configFile.empty() ? configPath_ : configFile;
  YAML::Node config;
  try {
    config = YAML::LoadFile(targetConfig);
  } catch (const std::exception &e) {
    throw std::runtime_error("Erro ao carregar YAML (" + targetConfig +
                             "): " + e.what());
  }

  activateMethods_.clear();
  numActiveMethods_ = 0;

  if (config["metaheuristics"]) {
    for (const auto &mh_node : config["metaheuristics"]) {
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
        std::cerr << "[Warning] Algoritmo desconhecido no YAML: " << name
                  << std::endl;
      }
    }
  }

  if (auto settings = config["execution_settings"]) {
    runData_.MAXRUNS = settings["max_runs"].as<int>(1);
    runData_.debug = settings["debug_mode"].as<int>(0);
    runData_.control = settings["control_mode"].as<int>(0);
  }

  if (auto params = config["search_parameters"]) {
    runData_.strategy = params["local_search_strategy"].as<int>(1);
    runData_.restart = params["restart_threshold"].as<float>(1.0f);
    runData_.sizePool = params["elite_pool_size"].as<int>(10);
  }

  std::string scalarizationMethod = "Tchebycheff";
  if (config["scalarization"]) {
    scalarizationMethod = config["scalarization"].as<std::string>();
  }

  if (scalarizationMethod == "WeightedSum") {
    scalarizer_ = std::make_unique<core::WeightedSumScalarizer>();
  } else if (scalarizationMethod == "Gini_Coefficient") {
    scalarizer_ = std::make_unique<core::GiniScalarizer>();
  } else {
    scalarizer_ = std::make_unique<core::TchebycheffScalarizer>();
  }

  if (config["default_weights"]) {
    defaultWeights_.clear();
    for (const auto &w : config["default_weights"]) {
      defaultWeights_.push_back(w.as<double>());
    }
  }

  std::cout << "Scalarization Strategy: " << scalarizer_->getName()
            << std::endl;
}

// -------------------------------------------------------------------------
// PLUGIN LOADING (DYNAMIC LIBRARY)
// -------------------------------------------------------------------------
void RkoSolver::loadProblemData() {
  std::cout << "[Info] Carregando plugin do problema: " << problemLibPath_
            << "...\n";

  typedef core::IProblem *(*CreateProblemFunc)();
  typedef void (*DestroyProblemFunc)(core::IProblem *);

  CreateProblemFunc createFunc = nullptr;
  DestroyProblemFunc destroyFunc = nullptr;

#ifdef _WIN32
  libHandle_ = LoadLibraryA(problemLibPath_.c_str());
  if (!libHandle_)
    throw std::runtime_error("Falha ao carregar a DLL do problema.");

  createFunc =
      (CreateProblemFunc)GetProcAddress((HMODULE)libHandle_, "create_problem");
  destroyFunc = (DestroyProblemFunc)GetProcAddress((HMODULE)libHandle_,
                                                   "destroy_problem");
#else
  libHandle_ = dlopen(problemLibPath_.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!libHandle_) {
    throw std::runtime_error("Falha ao carregar o .so: " +
                             std::string(dlerror()));
  }

  createFunc = (CreateProblemFunc)dlsym(libHandle_, "create_problem");
  destroyFunc = (DestroyProblemFunc)dlsym(libHandle_, "destroy_problem");
#endif

  if (!createFunc || !destroyFunc) {
    unloadProblemLibrary();
    throw std::runtime_error("Falha: Funções 'create_problem' ou "
                             "'destroy_problem' não encontradas no plugin.");
  }

  problemInstance_ = std::shared_ptr<core::IProblem>(createFunc(), destroyFunc);

  std::cout << "[Info] Lendo instância: " << instancePath_ << "...\n";
  problemInstance_->load(instancePath_);
}

void RkoSolver::unloadProblemLibrary() {
  if (problemInstance_) {
    problemInstance_.reset();
  }

  if (libHandle_) {
#ifdef _WIN32
    FreeLibrary((HMODULE)libHandle_);
#else
    dlclose(libHandle_);
#endif
    libHandle_ = nullptr;
  }
}

// -------------------------------------------------------------------------
// MAIN EXECUTION LOOP
// -------------------------------------------------------------------------
void RkoSolver::run() {
  validateConfiguration();
  loadProblemData();
  initializeStatistics();

  std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
  std::cout << "║              RKO Solver - Optimization Framework       ║\n";
  std::cout << "╚════════════════════════════════════════════════════════╝\n";
  std::cout << "\n[Info] Instance: " << instancePath_
            << "\n[Info] Methods:  " << numActiveMethods_
            << "\n[Info] Runs:     " << runData_.MAXRUNS
            << "\n[Info] Time Limit: " << runData_.MAXTIME << "s" << std::endl;

  std::cout << "\nProcessing: " << std::flush;
  char spinner[] = {'|', '/', '-', '\\'};

  for (int run = 0; run < runData_.MAXRUNS; ++run) {
      std::cout << "\rProcessing: " << spinner[run % 4] << " [" << (run + 1) << "]" << std::flush;
      executeRun(run);
  }
  std::cout << "\rProcessing: Pronto!       \n" << std::endl;

  computeFinalStatistics();
  displayResults();

  std::vector<std::string> activeAlgorithms;
  activeAlgorithms.reserve(numActiveMethods_);
  for (int i = 0; i < numActiveMethods_; ++i) {
      activeAlgorithms.push_back(getActiveName(i));
  }

  int dimension = getProblemDimension();

  if (runData_.debug == 0) { 
      utils::WriteSolution(activeAlgorithms, bestSolutionGlobal_, bestTime_, totalTime_, instancePath_, dimension);
      
      if(problemInstance_->getNumObjectives() > 1){
        utils::WriteMultiObjectiveResults(activeAlgorithms, bestObjective_, bestSolutionGlobal_.objs, averageObjective_, objectiveValues_, bestTime_, totalTime_, instancePath_);
      } else {
        utils::WriteResults(activeAlgorithms, bestObjective_, averageObjective_, objectiveValues_, bestTime_, totalTime_, instancePath_);
      }
      
  } else {
      auto& ctx = core::SolverContext::instance();
      utils::WriteSolutionScreen(activeAlgorithms, bestSolutionGlobal_, bestTime_, totalTime_, instancePath_, dimension, ctx.getPool());
  }

  utils::WriteConvergenceLog(convergenceHistory_, "../results");
}

void RkoSolver::validateConfiguration() {
  if (numActiveMethods_ == 0) {
    throw std::runtime_error("No optimization method selected.");
  }
  if (instancePath_.empty()) {
    throw std::runtime_error("Instance path not provided.");
  }
}

// -------------------------------------------------------------------------
// RUN EXECUTION
// -------------------------------------------------------------------------
void RkoSolver::executeRun(int runIndex) {
  
  auto &ctx = core::SolverContext::instance();

  unsigned int seed =
      (runData_.debug)
          ? (1234 + runIndex)
          : static_cast<unsigned int>(
                std::chrono::steady_clock::now().time_since_epoch().count());

  ctx.setSeed(seed);

  int nObj = problemInstance_->getNumObjectives();
  initReferencePoints(nObj);

  core::TSol bestSolutionRun;
  bestSolutionRun.ofv = std::numeric_limits<double>::infinity();

  double startTime = omp_get_wtime();

  ctx.resetStopFlag();

  ctx.initializePool(runData_.sizePool);
  core::CreatePoolSolutions(*this, runData_.sizePool);

  bestSolutionRun = ctx.getBestSolution();

  executeParallelMethods(startTime, bestSolutionRun, seed);

  double endTime = omp_get_wtime();
  updateStatistics(bestSolutionRun, startTime, endTime);
}

// -------------------------------------------------------------------------
// EXECUÇÃO PARALELA 
// -------------------------------------------------------------------------
void RkoSolver::executeParallelMethods(double startTime,
                                       core::TSol &bestSolutionRun,
                                       unsigned int /*baseSeed*/) {
  auto &ctx = core::SolverContext::instance();
  omp_set_num_threads(numActiveMethods_);

  // Only one parallel region. Threads are born here and die at the end of the round.
  #pragma omp parallel
  {
    while ((omp_get_wtime() - startTime) < runData_.MAXTIME) {

      #pragma omp single
      {
        ctx.resetStopFlag();
      }
      // Ensures the stopFlag is cleared for all before starting
      #pragma omp barrier 

      // 1. Manual distribution (Much safer and lighter than #pragma omp for)
      int tid = omp_get_thread_num();
      
      if (tid < numActiveMethods_) {
        try {
          // Thread roda sua MH correspondente.
          getActiveFunction(tid)(runData_, *this);
        } catch (const std::exception& e) {
          #pragma omp critical
          std::cerr << "[T-" << tid << "] Exception: " << e.what() << std::endl;
        } catch (...) {
          #pragma omp critical
          std::cerr << "[T-" << tid << "] Unknown Error" << std::endl;
        }
        
        // 2. The first MH that meets the stopping criterion (or finds the optimum) 
        // raises the flag for the others to stop peacefully.
        ctx.signalStop();
      }

      // 3. CRITICAL BARRIER: Wait for ALL MHs to see the flag and exit their functions
      #pragma omp barrier 

      // 4. Only one thread updates the global best of the round
      #pragma omp single
      {
        core::TSol ctxBest = ctx.getBestSolution();
        if (ctxBest.ofv < bestSolutionRun.ofv) {
          bestSolutionRun = ctxBest;

          double currentTime = omp_get_wtime() - startTime;
          convergenceHistory_.push_back({currentTime, ctxBest.nameMH, ctxBest.ofv});
        }

        if ((omp_get_wtime() - startTime) < runData_.MAXTIME) {
          core::CreatePoolSolutions(*this, runData_.sizePool);
        }
      }
      // No thread advances while the update does not finish
      #pragma omp barrier 

    } // End of while(MAXTIME)
  } // End of parallel block. The threads are returned clean to the OS.

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
  convergenceHistory_.clear();
}

void RkoSolver::updateStatistics(const core::TSol &runSolution,
                                 double startTime, double endTime) {
  double runTime = endTime - startTime;

  // 1. Unificação da checagem do Melhor Global (Garante sincronia de estado)
  if (runSolution.ofv < bestSolutionGlobal_.ofv) {
    
    bestSolutionGlobal_ = runSolution; // Cópia profunda da solução completa
    bestObjective_ = runSolution.ofv;  // Cache rápido da função objetivo
    
    // 2. Correção da precisão do tempo
    // Se a struct TSol registrou o momento exato do "achado" (best_time), usamos ele.
    // Caso contrário, fazemos o fallback para o tempo total da rodada.
    if (runSolution.best_time > 0.0) {
        bestTime_ = runSolution.best_time; 
    } else {
        bestTime_ = runTime;
    }
  }

  // 3. Acumuladores globais para cálculo de média no final
  averageObjective_ += runSolution.ofv;
  objectiveValues_.push_back(runSolution.ofv);
  totalTime_ += runTime; 
}

void RkoSolver::computeFinalStatistics() {
  if (runData_.MAXRUNS > 0) { averageObjective_ /= runData_.MAXRUNS; totalTime_ /= runData_.MAXRUNS; }
}

void RkoSolver::displayResults() {
  std::cout << "\n\n╔════════════════════════════════════════════════════════╗\n"
            << "║                    RESULTS SUMMARY                     ║\n"
            << "╚════════════════════════════════════════════════════════╝\n";
  std::cout << std::fixed << std::setprecision(5);

  if (problemInstance_->getNumObjectives() <= 1) {
    std::cout << "Best Objective:    " << std::abs(bestObjective_) << "\n"
              << "Average Objective: " << std::abs(averageObjective_) << "\n"
              << "Avg Run Time:      " << totalTime_ << "s\n";
    return;
  }

  std::cout << "Scalarized Fitness (Dist): " << bestObjective_ << " (Minimization)\n";
  std::cout << "Real Objectives Values:    [ ";
  for (double val : bestSolutionGlobal_.objs) { std::cout << val << " "; }
  std::cout << "]\nAvg Run Time:              " << totalTime_ << "s\n";
  std::cout << "Ideal Point Found:         [ ";
  for (double v : idealPoint_) std::cout << v << " ";
  std::cout << "]\n";
}

void RkoSolver::cleanup() {}

// -------------------------------------------------------------------------
// EVALUATION CORE 
// -------------------------------------------------------------------------
void RkoSolver::decodeSolution(core::TSol &sol, const std::vector<double> &lambda) {
 // fprintf(stderr, "[TRACE CORE] Inicio de decodeSolution. OFV: \n");

  problemInstance_->decode(sol);

  if (problemInstance_->getNumObjectives() <= 1) return;

  std::vector<double> activeLambda = lambda;
  if (activeLambda.empty()) {
    if (!defaultWeights_.empty()) { 
        activeLambda = defaultWeights_; 
    } else { 
        activeLambda.assign(sol.objs.size(), 1.0 / sol.objs.size()); 
    }
  }

  #pragma omp critical(scalarization_lock)
  {
    
    if (sol.objs.size() > idealPoint_.size()) {
        idealPoint_.resize(sol.objs.size(), 1.0e15);
        nadirPoint_.resize(sol.objs.size(), -1.0e15);
    }

    for (size_t k = 0; k < sol.objs.size(); ++k) {
      // Atualiza Ideal: Queremos a MENOR distância possível
      if (sol.objs[k] < idealPoint_[k]) { 
          idealPoint_[k] = sol.objs[k]; 
      }
      // Atualiza Nadir: Captura a PIOR (maior) distância encontrada
      if (sol.objs[k] > nadirPoint_[k]) { 
          nadirPoint_[k] = sol.objs[k]; 
      }
    }

    //fprintf(stderr, "[TRACE CORE] Inicio de scalarize. OFV:%f %d \n", sol.ofv, sol.objs.size());
    // Calcula o OFV com segurança total de memória
    sol.ofv = scalarizer_->scalarize(sol, activeLambda, idealPoint_, nadirPoint_);
  }

  //fprintf(stderr, "[TRACE CORE] Fim de decodeSolution. OFV: \n");
}


int RkoSolver::getProblemDimension() const {
  return problemInstance_->getDimension();
}

int RkoSolver::getNumObjectives() const {
  return problemInstance_->getNumObjectives();
}

void RkoSolver::initReferencePoints(int nObj) {
  if (nObj <= 1)
    return;

  //std::cout << "MultiObjetivo - Inicializando Pontos de Referência" << std::endl;
  idealPoint_.assign(nObj, 1.0e15); 
  nadirPoint_.assign(nObj, -1.0e15);  
  
  //std::cout << "[DEBUG TRACE] Pontos de Referência inicializados e alocados. Tamanho idealPoint_: " << idealPoint_.size() << std::endl;
}

// -------------------------------------------------------------------------
// HELPER ACCESSORS
// -------------------------------------------------------------------------
std::string RkoSolver::getActiveName(int index) const {
  if (index < 0 || index >= numActiveMethods_) return "Unknown";
  int registryIndex = activateMethods_[index];
  return algorithmRegistry_[registryIndex].name;
}

std::function<void(const core::TRunData &, RkoSolver &)>
RkoSolver::getActiveFunction(int index) const {
  int registryIndex = activateMethods_[index];
  return algorithmRegistry_[registryIndex].function;
}

} // namespace rkolib