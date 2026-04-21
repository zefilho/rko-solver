/**
 * RKO Library - Elegant Solver Interface
 * Main orchestrator for optimization algorithms
 */

#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/method.hpp"
#include "rkolib/core/problem.hpp" // Certifique-se que o nome é iproblem.hpp
#include "rkolib/core/scalarizer.hpp"

namespace rkolib {

/**
 * @brief Main solver class - orchestrates optimization process
 */
class RkoSolver {
public:
  // -------------------------------------------------------------------------
  // CONSTRUCTOR & DESTRUCTOR
  // -------------------------------------------------------------------------
  RkoSolver();
  ~RkoSolver();

  // -------------------------------------------------------------------------
  // PUBLIC INTERFACE
  // -------------------------------------------------------------------------
  void parseArguments(int argc, char *argv[]);
  void loadConfiguration(const std::string &configFile = "");
  void run();

  // -------------------------------------------------------------------------
  // ACCESSORS
  // -------------------------------------------------------------------------
  const core::TSol &getBestSolution() const { return bestSolutionGlobal_; }
  const core::TRunData &getRunData() const { return runData_; }
  double getBestObjective() const { return bestObjective_; }
  double getAverageObjective() const { return averageObjective_; }

  void decodeSolution(core::TSol &sol, const std::vector<double> &lambda = {});

  int getProblemDimension() const;
  const std::vector<double> &getIdealPoint() const { return idealPoint_; }
  const std::vector<double> &getNadirPoint() const { return nadirPoint_; }

  // Helper para inicializar antes do loop paralelo
  void initReferencePoints(int nObj);

private:
  struct AlgorithmEntry {
    std::string name;
    std::function<void(const core::TRunData &, RkoSolver &)> function;
  };

  // Private Initialization & Execution Helpers
  void initializeRegistry();
  void validateConfiguration();
  void loadProblemData();
  void unloadProblemLibrary();
  void initializeStatistics();
  void executeRun(int runIndex);
  void executeParallelMethods(double startTime, core::TSol &bestSolutionRun,
                              unsigned int baseSeed);
  void updateStatistics(const core::TSol &runSolution, double startTime,
                        double endTime);
  void computeFinalStatistics();
  void displayResults();
  void cleanup();

  // Helper Accessors
  std::string getActiveName(int index) const;
  std::function<void(const core::TRunData &, RkoSolver &)>
  getActiveFunction(int index) const;

  // -------------------------------------------------------------------------
  // MEMBER VARIABLES
  // -------------------------------------------------------------------------
  std::string instancePath_;
  std::string configPath_;
  core::TRunData runData_;

  // Instance of the problem loaded dynamically
  std::shared_ptr<core::IProblem> problemInstance_;

  // Path to the problem library (.so / .dll)
  std::string problemLibPath_;

  // Handle of the library loaded in the OS (dlopen/LoadLibrary)
  void *libHandle_;

  std::vector<AlgorithmEntry> algorithmRegistry_;
  std::vector<int> activateMethods_;
  int numActiveMethods_;

  core::TSol bestSolutionGlobal_;
  double bestObjective_;
  double averageObjective_;
  double bestTime_;
  double totalTime_;
  std::vector<double> objectiveValues_;

  // Scalarization strategy (Tchebycheff, WeightedSum, etc)
  std::unique_ptr<core::IScalarizer> scalarizer_;

  // Thread synchronization for multiobjective
  std::mutex mtx_;

  // Global reference points for multiobjective
  std::vector<double> idealPoint_;
  std::vector<double> nadirPoint_; // Added for normalization
  std::vector<double> defaultWeights_;

  // Convergence points
  std::vector<core::ConvergencePoint> convergenceHistory_;
};

} // namespace rkolib