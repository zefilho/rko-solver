/**
 * RKO Library - Elegant Solver Interface
 * Main orchestrator for optimization algorithms
 */

#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/iproblem.hpp"
#include "rkolib/core/method.hpp"
#include <CLI/CLI.hpp>


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
        void parseArguments(int argc, char* argv[]);
        void loadConfiguration(const std::string& configFile = "");
        void run();

        // -------------------------------------------------------------------------
        // ACCESSORS
        // -------------------------------------------------------------------------
        const core::TSol& getBestSolution() const { return bestSolutionGlobal_; }
        const core::TRunData& getRunData() const { return runData_; }
        double getBestObjective() const { return bestObjective_; }
        double getAverageObjective() const { return averageObjective_; }

    private:
        struct AlgorithmEntry {
            std::string name;
            std::function<void(const core::TRunData&, const core::IProblem&)> function;
        };

        // Private Initialization & Execution Helpers
        void initializeRegistry();
        void validateConfiguration();
        void loadProblemData();
        void initializeStatistics();
        void executeRun(int runIndex);
        void executeParallelMethods(double startTime, core::TSol& bestSolutionRun, unsigned int baseSeed);
        void updateStatistics(const core::TSol& runSolution, double startTime, double endTime);
        void computeFinalStatistics();
        void displayResults();
        void cleanup();

        // Helper Accessors
        std::string getActiveName(int index) const;
        std::function<void(const core::TRunData&, const core::IProblem&)> getActiveFunction(int index) const;

        // -------------------------------------------------------------------------
        // MEMBER VARIABLES
        // -------------------------------------------------------------------------
        std::string instancePath_;
        std::string configPath_;
        core::TRunData runData_;
        std::shared_ptr<core::IProblem> problemInstance_;

        std::vector<AlgorithmEntry> algorithmRegistry_;
        std::vector<int> activateMethods_;
        int numActiveMethods_;

        core::TSol bestSolutionGlobal_;
        double bestObjective_;
        double averageObjective_;
        double bestTime_;
        double totalTime_;
        std::vector<double> objectiveValues_;
    };

} // namespace rkolib