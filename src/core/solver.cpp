/**
 * RKO Library - Elegant Solver Implementation
 */

#include "rkolib/core/solver.hpp"
#include "rkolib/core/context.hpp"
#include "rkolib/core/method.hpp"
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
      problemInstance_(nullptr),
      problemLibPath_(""), // Path to the plugin
      libHandle_(nullptr), // Handle of the plugin in memory
      numActiveMethods_(0),
      bestObjective_(std::numeric_limits<double>::infinity()),
      averageObjective_(0.0), bestTime_(0.0), totalTime_(0.0),
      scalarizer_(std::make_unique<core::NormalizedWeightedTchebycheff>()) {
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

static std::unique_ptr<core::IScalarizer> createScalarizerFromConfig(
    const std::string &method, const YAML::Node &config) {
  if (method == "WeightedSum") {
    return std::make_unique<core::WeightedSumScalarizer>();
  }
  if (method == "Gini_Coefficient") {
    return std::make_unique<core::GiniScalarizer>();
  }
  if (method == "Lexicographic_MinMax") {
    double decay = 0.05;
    if (config["lexicographic_decay"]) {
      decay = config["lexicographic_decay"].as<double>(0.05);
    }
    return std::make_unique<core::LexicographicMinMaxScalarizer>(decay);
  }
  return std::make_unique<core::NormalizedWeightedTchebycheff>();
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
      auto it = std::find_if(algorithmRegistry_.begin(), algorithmRegistry_.end(),
                             [&name](const AlgorithmEntry &entry) {
                               return entry.name == name;
                             });
      if (it != algorithmRegistry_.end()) {
        activateMethods_.push_back(
            static_cast<int>(std::distance(algorithmRegistry_.begin(), it)));
        numActiveMethods_++;
      } else {
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
  scalarizer_ = createScalarizerFromConfig(scalarizationMethod, config);

  if (config["default_weights"]) {
    defaultWeights_.clear();
    for (const auto &w : config["default_weights"]) {
      defaultWeights_.push_back(w.as<double>());
    }
  }

  if (config["pool_update_method"]) {
    runData_.poolUpdateMethod = config["pool_update_method"].as<int>();
  }

  if (config["ideal_point"]) {
    userIdealPoint_.clear();
    for (const auto &v : config["ideal_point"]) {
      userIdealPoint_.push_back(v.as<double>());
    }
  }

  if (config["nadir_point"]) {
    userNadirPoint_.clear();
    for (const auto &v : config["nadir_point"]) {
      userNadirPoint_.push_back(v.as<double>());
    }
  }

  if (config["fixed_ideal_point"]) {
    fixedIdealPoint_ = config["fixed_ideal_point"].as<bool>();
  }

  if (config["fixed_nadir_point"]) {
    fixedNadirPoint_ = config["fixed_nadir_point"].as<bool>(false);
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

  problemInstance_->setDebugMode(runData_.debug);

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
  auto& ctx = core::SolverContext::instance();

  if (runData_.debug == 0) { 
      utils::WriteSolution(activeAlgorithms, bestSolutionGlobal_, bestTime_, totalTime_, instancePath_, dimension);
      utils::WritePoolSolutions(activeAlgorithms, ctx.getPool(), instancePath_, dimension);
      
      if(problemInstance_->getNumObjectives() > 1){
        utils::WriteMultiObjectiveResults(activeAlgorithms, bestObjective_, bestSolutionGlobal_.objs, averageObjective_, objectiveValues_, bestTime_, totalTime_, instancePath_);
      } else {
        utils::WriteResults(activeAlgorithms, bestObjective_, averageObjective_, objectiveValues_, bestTime_, totalTime_, instancePath_);
      }
      
  } else {
      utils::WriteSolutionScreen(activeAlgorithms, bestSolutionGlobal_, bestTime_, totalTime_, instancePath_, dimension, ctx.getPool());
  }

  utils::WriteConvergenceLog(convergenceHistory_, "results");
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

  int max_threads = std::max(omp_get_max_threads(), numActiveMethods_);
  ctx.setSeed(seed, max_threads);

  int nObj = problemInstance_->getNumObjectives();
  initReferencePoints(nObj);

  core::TSol bestSolutionRun;
  bestSolutionRun.ofv = std::numeric_limits<double>::infinity();

  double startTime = omp_get_wtime();
  ctx.setRunStartTime(core::get_time_in_seconds());

  ctx.resetStopFlag();

  ctx.initializePool(runData_.sizePool);
  core::CreatePoolSolutions(*this, runData_.sizePool);

  // CRÍTICO: Após criar o Pool inicial, recalcula todos os OFVs do Pool
  // usando o Ideal/Nadir real do Pool (elimina o falso ofv = 0.0 da 1ª solução!)
  syncReferencePointsAndPool(nullptr);

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
        // 1. Antes de comparar quem ganhou a rodada, sincroniza a régua (Ideal/Nadir)
        // e re-escalariza tanto o ctxBest quanto o bestSolutionRun na mesma régua!
        syncReferencePointsAndPool(&bestSolutionRun);

        core::TSol ctxBest = ctx.getBestSolution();
        if (ctxBest.ofv < bestSolutionRun.ofv) {
          bestSolutionRun = ctxBest;

          double currentTime = omp_get_wtime() - startTime;
          convergenceHistory_.push_back({currentTime, ctxBest.nameMH, ctxBest.ofv});
        }

        if ((omp_get_wtime() - startTime) < runData_.MAXTIME) {
          core::CreatePoolSolutions(*this, runData_.sizePool);
          syncReferencePointsAndPool(&bestSolutionRun);
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

  core::TSol validRunSol = runSolution;
  if (problemInstance_->getNumObjectives() > 1 && scalarizer_) {
    std::vector<double> weights = defaultWeights_;
    if (weights.empty() && !validRunSol.objs.empty()) {
      weights.assign(validRunSol.objs.size(), 1.0 / static_cast<double>(validRunSol.objs.size()));
    }

    // Garante que o Ideal/Nadir global incorpore os melhores limites desta Run
    for (size_t k = 0; k < validRunSol.objs.size(); ++k) {
      if (!fixedIdealPoint_ && validRunSol.objs[k] < idealPoint_[k]) {
        idealPoint_[k] = validRunSol.objs[k];
      }
    }

    validRunSol.ofv = scalarizer_->scalarize(validRunSol, weights, idealPoint_, nadirPoint_);

    // CRÍTICO: Re-avalia o bestSolutionGlobal_ na mesma régua (idealPoint_/nadirPoint_)
    // antes da comparação entre Runs!
    if (!bestSolutionGlobal_.objs.empty()) {
      bestSolutionGlobal_.ofv =
          scalarizer_->scalarize(bestSolutionGlobal_, weights, idealPoint_, nadirPoint_);
      bestObjective_ = bestSolutionGlobal_.ofv;
    }
  }

  // 1. Unificação da checagem do Melhor Global (Garante sincronia de estado)
  if (validRunSol.ofv < bestSolutionGlobal_.ofv) {
    
    bestSolutionGlobal_ = validRunSol; // Cópia profunda da solução completa
    bestObjective_ = validRunSol.ofv;  // Cache rápido da função objetivo
    
    // 2. Correção da precisão do tempo
    // Se a struct TSol registrou o momento exato do "achado" (best_time), usamos ele.
    // Caso contrário, fazemos o fallback para o tempo total da rodada.
    if (validRunSol.best_time > 0.0) {
        bestTime_ = validRunSol.best_time; 
    } else {
        bestTime_ = runTime;
    }
  }

  // 3. Acumuladores globais para cálculo de média no final
  averageObjective_ += validRunSol.ofv;
  objectiveValues_.push_back(validRunSol.ofv);
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
  } else {
    std::cout << "Scalarized Fitness (Dist): " << bestObjective_ << " (Minimization)\n";
    std::cout << "Real Objectives Values:    [ ";
    for (double val : bestSolutionGlobal_.objs) { std::cout << val << " "; }
    std::cout << "]\nAvg Run Time:              " << totalTime_ << "s\n";
    std::cout << "Ideal Point Found:         [ ";
    for (double v : idealPoint_) std::cout << v << " ";
    std::cout << "]\n";
  }

  auto &ctx = core::SolverContext::instance();
  utils::PrintPoolSolutions(ctx.getPool(), getProblemDimension());
}

void RkoSolver::cleanup() {}

// -------------------------------------------------------------------------
// EVALUATION CORE 
// -------------------------------------------------------------------------
void RkoSolver::decodeSolution(core::TSol &sol, const std::vector<double> &lambda) {
  problemInstance_->decode(sol);

  if (problemInstance_->getNumObjectives() <= 1) {
    if (!sol.objs.empty()) {
      sol.ofv = sol.objs[0];
    }
    return;
  }

  std::vector<double> activeLambda = lambda;
  if (activeLambda.empty()) {
    if (!defaultWeights_.empty()) {
      activeLambda = defaultWeights_;
    } else {
      activeLambda.assign(sol.objs.size(), 1.0 / static_cast<double>(sol.objs.size()));
    }
  }

  #pragma omp critical(scalarization_lock)
  {
    const size_t m = sol.objs.size();
    if (m > idealPoint_.size()) {
      idealPoint_.resize(m, 1.0e15);
      candIdealPoint_.resize(m, 1.0e15);
      nadirPoint_.resize(m, -1.0e15);
    }

    for (size_t k = 0; k < m; ++k) {
      // Registra candidato a Ideal
      if (!fixedIdealPoint_ && sol.objs[k] < candIdealPoint_[k]) {
        candIdealPoint_[k] = sol.objs[k];
        // Na fase de aquecimento (CreatePoolSolutions), já reflete no idealPoint_
        if (!boundsInitialized_ && sol.objs[k] < idealPoint_[k]) {
          idealPoint_[k] = sol.objs[k];
        }
      }

      // ATENÇÃO: Só expande nadirPoint_ diretamente durante o aquecimento inicial (!boundsInitialized_)
      // e ignorando soluções penalizadas (ex: > 1e12). Após o Pool existir, syncReferencePointsAndPool cuida do Nadir!
      if (!fixedNadirPoint_ && !boundsInitialized_ && sol.objs[k] < 1.0e12) {
        if (sol.objs[k] > nadirPoint_[k]) {
          nadirPoint_[k] = sol.objs[k];
        }
      }

      // Proteção contra a armadilha da 1ª solução (Ideal == Nadir):
      // Se ainda não há amplitude conhecida para o objetivo k, garante um range base proporcional
      if (nadirPoint_[k] <= idealPoint_[k] + 1e-7) {
        nadirPoint_[k] = idealPoint_[k] + std::max(1.0, std::abs(idealPoint_[k]) * 0.1);
      }
    }

    sol.ofv = scalarizer_->scalarize(sol, activeLambda, idealPoint_, nadirPoint_);
  }
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

  candIdealPoint_.assign(nObj, 1.0e15);
  boundsInitialized_ = false;

  if (!userIdealPoint_.empty() && static_cast<int>(userIdealPoint_.size()) == nObj) {
    idealPoint_ = userIdealPoint_;
    candIdealPoint_ = userIdealPoint_;
  } else {
    idealPoint_.assign(nObj, 1.0e15);
  }

  if (!userNadirPoint_.empty() && static_cast<int>(userNadirPoint_.size()) == nObj) {
    nadirPoint_ = userNadirPoint_;
    boundsInitialized_ = true;
  } else {
    nadirPoint_.assign(nObj, -1.0e15);
  }
}

static void extractPoolNadir(const std::vector<core::TSol> &pool, int nObj,
                             std::vector<double> &poolNadir, bool &validPool) {
  for (const auto &s : pool) {
    if (static_cast<int>(s.objs.size()) != nObj) {
      continue;
    }
    validPool = true;
    for (int k = 0; k < nObj; ++k) {
      poolNadir[k] = std::max(poolNadir[k], s.objs[k]);
    }
  }
}

static bool updateNadirFromElitePool(const std::vector<core::TSol> &pool, int nObj,
                                    const std::vector<double> &idealPoint,
                                    std::vector<double> &nadirPoint,
                                    bool &boundsInitialized) {
  if (pool.empty()) {
    return false;
  }

  std::vector<double> poolNadir(nObj, -1.0e15);
  bool validPool = false;
  extractPoolNadir(pool, nObj, poolNadir, validPool);
  if (!validPool) {
    return false;
  }

  bool changed = false;
  for (int k = 0; k < nObj; ++k) {
    if (poolNadir[k] <= idealPoint[k] + 1e-6) {
      poolNadir[k] = idealPoint[k] + 1.0;
    }
    double range = std::max(1e-9, std::abs(nadirPoint[k] - idealPoint[k]));
    if (!boundsInitialized || (std::abs(poolNadir[k] - nadirPoint[k]) / range) > 0.005) {
      nadirPoint[k] = boundsInitialized
                          ? (0.8 * nadirPoint[k] + 0.2 * poolNadir[k])
                          : poolNadir[k];
      changed = true;
    }
  }
  boundsInitialized = true;
  return changed;
}

void RkoSolver::syncReferencePointsAndPool(core::TSol *bestRunSol) {
  int nObj = problemInstance_->getNumObjectives();
  if (nObj <= 1 || !scalarizer_)
    return;

  auto &ctx = core::SolverContext::instance();
  auto &pool = ctx.getPool();

  bool boundsChanged = false;

  // 1. Atualiza o IdealPoint com o melhor visto pelas threads até agora
  if (!fixedIdealPoint_) {
    for (int k = 0; k < nObj; ++k) {
      if (candIdealPoint_[k] < idealPoint_[k] - 1e-7) {
        idealPoint_[k] = candIdealPoint_[k];
        boundsChanged = true;
      }
    }
  }

  // 2. Calcula o NadirPoint a partir do Pool de Elite (Evita contaminação por soluções lixo!)
  if (!fixedNadirPoint_ &&
      updateNadirFromElitePool(pool, nObj, idealPoint_, nadirPoint_, boundsInitialized_)) {
    boundsChanged = true;
  }

  if (!boundsChanged)
    return;

  // 3. Se a régua mudou, RE-ESCALARIZA todo o Pool e os Incumbentes na MESMA régua!
  // Isso elimina 100% do bug da 1ª solução com ofv = 0.0 e o Fitness Drift.
  std::vector<double> weights = defaultWeights_;
  if (weights.empty()) {
    weights.assign(nObj, 1.0 / static_cast<double>(nObj));
  }

  // 3. Re-escalariza todo o Pool e reordena (garantindo que pool_[0] seja o verdadeiro líder!)
  ctx.rescalarizePool(*scalarizer_, weights, idealPoint_, nadirPoint_);

  // 4. Re-escalariza também o bestSolutionRun para comparação justa no #pragma omp single
  if (bestRunSol != nullptr && !bestRunSol->objs.empty()) {
    bestRunSol->ofv = scalarizer_->scalarize(*bestRunSol, weights, idealPoint_, nadirPoint_);
  }
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