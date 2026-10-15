#include "rkolib/mh/ipr.hpp"

#include <algorithm>
#include "rkolib/core/method.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

using namespace rkolib::core;

// -------------------------------------------------------------------------
// Helper: SubstituteBlock
// -------------------------------------------------------------------------
static void SubstituteBlock(TSol &sViz, const TSol &guia, int initialBlock,
                            int finalBlock, int dim, int direction) {
  for (int k = initialBlock; k < finalBlock && k < dim; k++) {
    if (direction == 1) {
      sViz.rk[k] = guia.rk[k];
    } else if (direction == -1) {
      sViz.rk[k] = std::clamp(1.0 - guia.rk[k], 0.0, 0.999999);
    }
  }
}

// -------------------------------------------------------------------------
// Helper: EvaluateCandidateBlocks
// -------------------------------------------------------------------------
static void EvaluateCandidateBlocks(const TSol &sCurrent, const TSol &guia,
                                    const std::vector<int> &fixedBlock,
                                    int numBlock, int blockSize, int dim,
                                    int direction, RkoSolver &solver,
                                    TSol &bestIteration, int &bestBlock) {
  for (int i = 0; i < numBlock; i++) {
    if (fixedBlock[i] != 0) {
      continue;
    }

    int initialBlock = i * blockSize;
    int finalBlock = initialBlock + blockSize;

    TSol sViz = sCurrent;
    SubstituteBlock(sViz, guia, initialBlock, finalBlock, dim, direction);
    solver.decodeSolution(sViz);

    if (sViz.ofv < bestIteration.ofv) {
      bestIteration = sViz;
      bestBlock = i;
    }
  }
}

// -------------------------------------------------------------------------
// Helper: PathRelink
// -------------------------------------------------------------------------
static TSol PathRelink(const TSol &atual, const TSol &guia, RkoSolver &solver,
                       const TRunData &runData) {
  TSol bestPath = atual;
  TSol sCurrent = atual;

  int direction = 1;
  int dim = solver.getProblemDimension();
  int blockSize = std::max(1, (int)(dim * 0.10));
  int numBlock = dim / blockSize;

  std::vector<int> fixedBlock(numBlock, 1);
  int dist = 0;

  for (int i = 0; i < dim; i++) {
    if (std::abs(atual.rk[i] - guia.rk[i]) > 1e-9) {
      int blockIdx = i % numBlock;
      if (fixedBlock[blockIdx] != 0) {
        fixedBlock[blockIdx] = 0;
        dist++;
      }
    }
  }

  if (dist == 0) {
    return bestPath;
  }

  bestPath.ofv = std::numeric_limits<double>::infinity();
  int numIteration = 0;

  while (numIteration < numBlock - 1) {
    if (SOLVER_SHOULD_STOP) {
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] IPR interrompido via SOLVER_SHOULD_STOP." << std::endl;
      }
      return bestPath;
    }

    numIteration++;
    int bestBlock = -1;
    TSol bestIteration = sCurrent;
    bestIteration.ofv = std::numeric_limits<double>::infinity();

    EvaluateCandidateBlocks(sCurrent, guia, fixedBlock, numBlock, blockSize,
                            dim, direction, solver, bestIteration, bestBlock);

    if (bestIteration.ofv < bestPath.ofv) {
      bestPath = bestIteration;
    }

    if (bestBlock < 0) {
      break;
    }

    sCurrent = bestIteration;
    fixedBlock[bestBlock] = 1;
  }

  return bestPath;
}

// -------------------------------------------------------------------------
// Main Algorithm: IPR
// -------------------------------------------------------------------------
void IPR(const TRunData &runData, RkoSolver &solver) {
  const char *method = "IPR";

  float currentTime = 0;
  double start_timeMH = get_time_in_seconds();
  double end_timeMH = get_time_in_seconds();

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] IPR iniciado." << std::endl;
  }

  while (currentTime < runData.MAXTIME * runData.restart) {
    TSol atual;
    TSol guia;
    bool pool_ok = false;

    #pragma omp critical
    {
      if (SOLVER_POOL.size() >= 2) {
        int k1, k2;
        do {
          k1 = irandomico(0, (int)SOLVER_POOL.size() - 1);
          k2 = irandomico(0, (int)SOLVER_POOL.size() - 1);
        } while (k1 == k2);

        atual = SOLVER_POOL[k1];
        guia = SOLVER_POOL[k2];
        pool_ok = true;
      }
    }

    if (!pool_ok) {
      break;
    }

    TSol bestPath = PathRelink(atual, guia, solver, runData);
    if (SOLVER_SHOULD_STOP) {
      return;
    }

    end_timeMH = get_time_in_seconds();
    currentTime = (float)(end_timeMH - start_timeMH);

    UpdatePoolSolutions(bestPath, method, runData.debug, runData.poolUpdateMethod);
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] IPR finalizado." << std::endl;
  }
}

} // namespace rkolib::mh