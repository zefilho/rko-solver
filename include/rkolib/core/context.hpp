/**
 * RKO Library - Solver Context Singleton
 * Centralizes shared state without global variables
 */

#pragma once

#include "rkolib/core/data.hpp"
#include "rkolib/core/scalarizer.hpp" // Para IScalarizer
#include <algorithm>
#include <atomic>
#include <omp.h>
#include <random>
#include <stdexcept>
#include <vector>

namespace rkolib {
namespace core {

/**
 * @brief Wrapper alinhado em 64 bytes (Cache Line) para evitar False Sharing
 * entre threads OpenMP que sorteiam chaves aleatórias simultaneamente.
 */
struct alignas(64) PaddedRng {
  std::mt19937 engine;
};

/**
 * @brief Thread-safe singleton that holds shared solver state
 */
class SolverContext {
public:
  // -------------------------------------------------------------------------
  // SINGLETON ACCESS
  // -------------------------------------------------------------------------
  static SolverContext &instance() {
    static SolverContext instance;
    return instance;
  }

  SolverContext(const SolverContext &) = delete;
  SolverContext &operator=(const SolverContext &) = delete;
  SolverContext(SolverContext &&) = delete;
  SolverContext &operator=(SolverContext &&) = delete;

  // -------------------------------------------------------------------------
  // SHARED STATE ACCESSORS
  // -------------------------------------------------------------------------

  std::mt19937 &getRng() {
    int thread_id = omp_get_thread_num();

    // Agora rngs_ é dimensionado com folga em setSeed, evitando fallback concorrente em rngs_[0]
    if (thread_id >= 0 && thread_id < static_cast<int>(rngs_.size())) {
      return rngs_[thread_id].engine;
    }
    return rngs_[0].engine;
  }

  void setSeed(unsigned int baseSeed, int min_threads = 32) {
    // Garante cobertura mesmo se omp_set_num_threads(numActiveMethods_)
    // for maior que omp_get_max_threads()
    int max_threads = std::max(omp_get_max_threads(), min_threads);

    if (rngs_.size() < static_cast<size_t>(max_threads)) {
      rngs_.resize(max_threads);
    }

    for (int i = 0; i < max_threads; ++i) {
      // Multiplicador primo afasta as sequências de Mersenne Twister entre MHs
      rngs_[i].engine.seed(baseSeed + static_cast<unsigned int>(i * 104729));
    }
  }

  std::atomic<bool> &getStopFlag() { return stopExecution_; }

  std::vector<TSol> &getPool() { return pool_; }
  const std::vector<TSol> &getPool() const { return pool_; }

  void resetStopFlag() { stopExecution_.store(false, std::memory_order_relaxed); }

  void signalStop() { stopExecution_.store(true, std::memory_order_relaxed); }

  bool shouldStop() const { return stopExecution_.load(std::memory_order_relaxed); }

  // -------------------------------------------------------------------------
  // POOL MANAGEMENT & MULTI-OBJECTIVE RESCALARIZATION
  // -------------------------------------------------------------------------

  void initializePool(size_t size) {
    pool_.clear();
    pool_.resize(size);
  }

  void clearPool() { pool_.clear(); }

  // Ordena o Pool garantindo o invariante: pool_[0] é sempre a melhor solução (menor ofv)
  void sortPool() {
    std::sort(pool_.begin(), pool_.end(),
              [](const TSol &a, const TSol &b) { return a.ofv < b.ofv; });
  }

  // Re-avalia todo o Pool com o novo Ideal/Nadir sem re-executar o Decoder
  // e já reordena para que pool_[0] reflita o verdadeiro líder na nova régua!
  void rescalarizePool(IScalarizer &scalarizer,
                       const std::vector<double> &lambda,
                       const std::vector<double> &idealPoint,
                       const std::vector<double> &nadirPoint) {
    if (pool_.empty()) {
      return;
    }

    for (auto &s : pool_) {
      if (!s.objs.empty()) {
        s.ofv = scalarizer.scalarize(s, lambda, idealPoint, nadirPoint);
      }
    }
    sortPool();
  }

  TSol getBestSolution() const {
    if (pool_.empty()) {
      throw std::runtime_error("Pool vazio!");
    }
    return pool_[0];
  }

private:
  SolverContext() : stopExecution_(false) {}
  ~SolverContext() = default;

  std::vector<PaddedRng> rngs_;
  std::atomic<bool> stopExecution_;
  std::vector<TSol> pool_;
};

// -------------------------------------------------------------------------
// CONVENIENCE MACROS (Backward compatibility)
// -------------------------------------------------------------------------
#define SOLVER_RNG rkolib::core::SolverContext::instance().getRng()
#define SOLVER_STOP rkolib::core::SolverContext::instance().getStopFlag()
#define SOLVER_POOL rkolib::core::SolverContext::instance().getPool()
#define SOLVER_SHOULD_STOP rkolib::core::SolverContext::instance().shouldStop()

} // namespace core
} // namespace rkolib