/**
 * RKO Library - Solver Context Singleton
 * Centralizes shared state without global variables
 */

#pragma once

#include <random>
#include <atomic>
#include <vector>
#include "rkolib/core/data.hpp"

namespace rkolib {
    namespace core {

    /**
    * @brief Thread-safe singleton that holds shared solver state
    * 
    * Replaces global variables (rng, stop_execution, pool) with a
    * centralized context accessible via SolverContext::instance()
    */
    class SolverContext {
      public:
          // -------------------------------------------------------------------------
          // SINGLETON ACCESS
          // -------------------------------------------------------------------------
          static SolverContext& instance() {
              static SolverContext instance;
              return instance;
          }

          // Delete copy/move constructors
          SolverContext(const SolverContext&) = delete;
          SolverContext& operator=(const SolverContext&) = delete;
          SolverContext(SolverContext&&) = delete;
          SolverContext& operator=(SolverContext&&) = delete;

          // -------------------------------------------------------------------------
          // SHARED STATE ACCESSORS
          // -------------------------------------------------------------------------
          
          std::mt19937& getRng() { return rng_; }
          
          std::atomic<bool>& getStopFlag() { return stopExecution_; }
          
          std::vector<TSol>& getPool() { return pool_; }
          
          void setSeed(unsigned int seed) { rng_.seed(seed); }
          
          void resetStopFlag() { stopExecution_.store(false); }
          
          void signalStop() { stopExecution_.store(true); }
          
          bool shouldStop() const { return stopExecution_.load(); }

          // -------------------------------------------------------------------------
          // POOL MANAGEMENT
          // -------------------------------------------------------------------------
          
          void initializePool(size_t size) {
              pool_.clear();
              pool_.resize(size);
          }
          
          void clearPool() {
              pool_.clear();
          }
          
          TSol& getBestSolution() {
              return pool_[0];
          }

      private:
          SolverContext() 
              : stopExecution_(false) 
          {}

          ~SolverContext() = default;

          // Shared state
          std::mt19937 rng_;
          std::atomic<bool> stopExecution_;
          std::vector<TSol> pool_;
      };

      // -------------------------------------------------------------------------
      // CONVENIENCE MACROS (Optional - for backward compatibility)
      // -------------------------------------------------------------------------
      #define SOLVER_RNG        rkolib::core::SolverContext::instance().getRng()
      #define SOLVER_STOP       rkolib::core::SolverContext::instance().getStopFlag()
      #define SOLVER_POOL       rkolib::core::SolverContext::instance().getPool()
      #define SOLVER_SHOULD_STOP rkolib::core::SolverContext::instance().shouldStop()

      } // namespace core
} // namespace rkolib