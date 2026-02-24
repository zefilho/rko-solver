/**
 * RKO Library - Solver Context Singleton
 * Centralizes shared state without global variables
 */

#pragma once

#include <random>
#include <atomic>
#include <vector>
#include <omp.h>
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
          
          std::mt19937& getRng() { 
              int thread_id = omp_get_thread_num();
              
              // Proteção: se o vetor não foi inicializado ou thread_id é estranho
              if (thread_id >= (int)rngs_.size()) {
                  return rngs_[0]; // Fallback para a thread mestre
              }
              return rngs_[thread_id]; 
          }

          void setSeed(unsigned int baseSeed) { 
              int max_threads = omp_get_max_threads();
              
              // Garante que temos RNGs suficientes
              if (rngs_.size() < (size_t)max_threads) {
                  rngs_.resize(max_threads);
              }

              // Semeia cada gerador com um valor diferente para evitar 
              // que todas as threads gerem a mesma sequência
              for(int i = 0; i < max_threads; ++i) {
                  // baseSeed + i garante diversidade
                  // (baseSeed + i * 1000) separa mais as sementes se preferir
                  rngs_[i].seed(baseSeed + i); 
              }
          }
          
          std::atomic<bool>& getStopFlag() { return stopExecution_; }
          
          std::vector<TSol>& getPool() { return pool_; }
          
          //void setSeed(unsigned int seed) { rng_.seed(seed); }
          
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
              if (pool_.empty()) throw std::runtime_error("Pool vazio!");
              return pool_[0];
          }

      private:
          SolverContext() 
              : stopExecution_(false) 
          {}

          ~SolverContext() = default;

          // Shared state
          std::vector<std::mt19937> rngs_;
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