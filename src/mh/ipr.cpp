#include "rkolib/mh/ipr.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "rkolib/core/method.hpp"
#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

using namespace rkolib::core;

enum PRType {
  PR_ONE_WAY = 0,
  PR_BACK_FORTH = 1,
  PR_MIXED = 2,
  PR_GRAPR = 3,
  PR_TRUNCATED = 4,
  PR_EXTERNAL = 5
};

static constexpr double IPR_TRUNC_FRAC = 0.5; // truncated PR: fraction of path explored
static constexpr double IPR_RCL_FRAC   = 0.3; // GRAPR: fraction of candidate moves in RCL

static bool IPRTimeIsOver(const TRunData &runData, double start_timeMH) {
  return SOLVER_SHOULD_STOP ||
         (get_time_in_seconds() - start_timeMH) >= runData.MAXTIME * runData.restart;
}

static std::vector<int> IPRDifferentBlocks(const TSol &a, const TSol &b,
                                           int blockSize, int numBlock, int n) {
  std::vector<int> blocks;
  blocks.reserve(numBlock);
  for (int bl = 0; bl < numBlock; bl++) {
    int ini = bl * blockSize;
    int fim = std::min(ini + blockSize, n);
    for (int k = ini; k < fim; k++) {
      if (a.rk[k] != b.rk[k]) {
        blocks.push_back(bl);
        break;
      }
    }
  }
  return blocks;
}

static void IPRApplyBlock(TSol &s, const TSol &guide, int bl,
                          int blockSize, int n, bool external) {
  int ini = bl * blockSize;
  int fim = std::min(ini + blockSize, n);
  for (int k = ini; k < fim; k++) {
    if (!external) {
      s.rk[k] = guide.rk[k];
    } else {
      s.rk[k] = std::clamp(1.0 - guide.rk[k], 0.0, 0.999999);
    }
  }
}

static int IPRSelectMove(const TSol &s, const TSol &guide,
                         const std::vector<int> &cand, int blockSize, int n,
                         bool external, bool randomized, TSol &chosen,
                         RkoSolver &solver) {
  int numCand = static_cast<int>(cand.size());
  std::vector<TSol> neigh(numCand);
  std::vector<int> order(numCand);

  for (int c = 0; c < numCand; c++) {
    neigh[c] = s;
    IPRApplyBlock(neigh[c], guide, cand[c], blockSize, n, external);
    solver.decodeSolution(neigh[c]);
    order[c] = c;
  }

  int pick = 0;
  if (!randomized) {
    for (int c = 1; c < numCand; c++) {
      if (neigh[c].ofv < neigh[pick].ofv) {
        pick = c;
      }
    }
  } else {
    std::sort(order.begin(), order.end(),
              [&neigh](int x, int y) { return neigh[x].ofv < neigh[y].ofv; });

    int rclSize = std::max(1, static_cast<int>(std::ceil(IPR_RCL_FRAC * numCand)));
    pick = order[irandomico(0, rclSize - 1)];
  }

  chosen = neigh[pick];
  return pick;
}

static TSol IPRPathOneWay(const TSol &start, const TSol &guide, int blockSize,
                          int numBlock, int maxSteps, bool external,
                          bool randomized, const TRunData &runData,
                          RkoSolver &solver, double start_timeMH) {
  int n = solver.getProblemDimension();
  TSol bestPath = start;
  bestPath.ofv = std::numeric_limits<double>::infinity();

  std::vector<int> cand = IPRDifferentBlocks(start, guide, blockSize, numBlock, n);

  int numSteps = external ? static_cast<int>(cand.size())
                          : static_cast<int>(cand.size()) - 1;
  if (numSteps > maxSteps) {
    numSteps = maxSteps;
  }

  TSol sCurrent = start;
  TSol sNext;
  for (int step = 0; step < numSteps; step++) {
    int pick = IPRSelectMove(sCurrent, guide, cand, blockSize, n, external,
                             randomized, sNext, solver);

    sCurrent = sNext;
    cand.erase(cand.begin() + pick);

    if (sCurrent.ofv < bestPath.ofv) {
      bestPath = sCurrent;
    }

    if (IPRTimeIsOver(runData, start_timeMH)) {
      break;
    }
  }

  return bestPath;
}

static TSol IPRPathMixed(const TSol &atual, const TSol &guia, int blockSize,
                         int numBlock, const TRunData &runData,
                         RkoSolver &solver, double start_timeMH) {
  int n = solver.getProblemDimension();
  TSol bestPath = atual;
  bestPath.ofv = std::numeric_limits<double>::infinity();

  TSol sA = atual;
  TSol sB = guia;
  TSol sNext;

  std::vector<int> cand = IPRDifferentBlocks(sA, sB, blockSize, numBlock, n);

  bool forward = true;
  while (cand.size() > 1) {
    TSol *I = forward ? &sA : &sB;
    TSol *G = forward ? &sB : &sA;

    int pick = IPRSelectMove(*I, *G, cand, blockSize, n, false, false, sNext, solver);

    *I = sNext;
    cand.erase(cand.begin() + pick);

    if (I->ofv < bestPath.ofv) {
      bestPath = *I;
    }

    forward = !forward;

    if (IPRTimeIsOver(runData, start_timeMH)) {
      break;
    }
  }

  return bestPath;
}

// -------------------------------------------------------------------------
// Main Algorithm: IPR
// -------------------------------------------------------------------------
void IPR(const TRunData &runData, RkoSolver &solver) {
  const char *method = "IPR";

  float currentTime = 0.0f;
  double start_timeMH = get_time_in_seconds();
  double end_timeMH = get_time_in_seconds();

  TSol bestIPR;
  bestIPR.ofv = std::numeric_limits<double>::infinity();

  // Parâmetros offline padrão
  double alphaBlock = 0.10;
  int typePR = PR_ONE_WAY;

  // Parâmetros do Q-Learning
  std::vector<TState> S;
  int numPar = 2;
  int numStates = 0;
  int iCurr = 0;
  double epsilon = 0.0;
  double lf = 0.0;
  double df = 0.0;
  double R = 0.0;
  float delta = 0.0f;
  double epsilon_min = 0.1;
  int st = 0;
  int at = 0;

  // Espaço de estados ordenado: alpha x typePR (3 x 6 = 18 estados)
  std::vector<std::vector<double>> parameters(numPar);
  parameters[0] = {0.05, 0.10, 0.20};
  parameters[1] = {PR_ONE_WAY, PR_BACK_FORTH, PR_MIXED,
                   PR_GRAPR,   PR_TRUNCATED,  PR_EXTERNAL};

  if (runData.control == 1) {
    CreateStates(parameters, numStates, numPar, S);
    LoadQTable(S, method);

    epsilon = 0.95;
    delta = 0.0f;

    st = irandomico(0, numStates - 1);
    iCurr = st;
    if (!S.empty()) {
      alphaBlock = S[iCurr].par[0];
      typePR = static_cast<int>(std::lround(S[iCurr].par[1]));
    }
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] IPR iniciado." << std::endl;
  }

  int n = solver.getProblemDimension();

  while (currentTime < runData.MAXTIME * runData.restart && !SOLVER_SHOULD_STOP) {
    TSol atual;
    TSol guia;
    bool pool_ok = false;

    #pragma omp critical(pool_lock)
    {
      int poolSize = static_cast<int>(SOLVER_POOL.size());
      if (poolSize >= 2) {
        int k1, k2;
        do {
          k1 = irandomico(0, poolSize - 1);
          k2 = irandomico(0, poolSize - 1);
        } while (k1 == k2);

        atual = SOLVER_POOL[k1];
        guia = SOLVER_POOL[k2];
        pool_ok = true;
      }
    }

    // Aguarda o pool ter ao menos 2 soluções sem encerrar a thread
    if (!pool_ok) {
      end_timeMH = get_time_in_seconds();
      currentTime = static_cast<float>(end_timeMH - start_timeMH);
      continue;
    }

    // Q-Learning: escolha online de (alphaBlock, typePR)
    if (runData.control == 1 && !S.empty()) {
      SetQLParameter(epsilon, lf, df,
                     static_cast<int>(runData.MAXTIME * runData.restart),
                     currentTime, delta, epsilon_min);
      delta = 0.0f;

      at = ChooseAction(S, st, epsilon);
      if (at >= 0 && at < static_cast<int>(S[st].Ai.size())) {
        iCurr = S[st].Ai[at];
        alphaBlock = S[iCurr].par[0];
        typePR = static_cast<int>(std::lround(S[iCurr].par[1]));
      }
    }

    int blockSize = std::max(1, static_cast<int>(n * alphaBlock));
    int numBlock = (n + blockSize - 1) / blockSize;
    int numDiff = static_cast<int>(
        IPRDifferentBlocks(atual, guia, blockSize, numBlock, n).size());

    TSol bestPath = atual;
    bestPath.ofv = std::numeric_limits<double>::infinity();

    if (numDiff > 0) {
      switch (typePR) {
      case PR_ONE_WAY:
        bestPath = IPRPathOneWay(atual, guia, blockSize, numBlock, numDiff,
                                 false, false, runData, solver, start_timeMH);
        break;
      case PR_BACK_FORTH: {
        TSol xF = IPRPathOneWay(atual, guia, blockSize, numBlock, numDiff,
                                false, false, runData, solver, start_timeMH);
        if (!IPRTimeIsOver(runData, start_timeMH)) {
          TSol xB = IPRPathOneWay(guia, atual, blockSize, numBlock, numDiff,
                                  false, false, runData, solver, start_timeMH);
          bestPath = (xF.ofv <= xB.ofv) ? xF : xB;
        } else {
          bestPath = xF;
        }
        break;
      }
      case PR_MIXED:
        bestPath = IPRPathMixed(atual, guia, blockSize, numBlock, runData,
                                solver, start_timeMH);
        break;
      case PR_GRAPR:
        bestPath = IPRPathOneWay(atual, guia, blockSize, numBlock, numDiff,
                                 false, true, runData, solver, start_timeMH);
        break;
      case PR_TRUNCATED: {
        int maxSteps = std::max(1, static_cast<int>(std::ceil(IPR_TRUNC_FRAC * numDiff)));
        bestPath = IPRPathOneWay(atual, guia, blockSize, numBlock, maxSteps,
                                 false, false, runData, solver, start_timeMH);
        break;
      }
      case PR_EXTERNAL:
        bestPath = IPRPathOneWay(atual, guia, blockSize, numBlock, numDiff,
                                 true, false, runData, solver, start_timeMH);
        break;
      default:
        break;
      }
    }

    if (SOLVER_SHOULD_STOP) {
      break;
    }

    if (bestPath.ofv < std::numeric_limits<double>::infinity()) {
      if (bestPath.ofv < bestIPR.ofv) {
        if (bestIPR.ofv < std::numeric_limits<double>::infinity()) {
          delta = static_cast<float>(ComputeDelta(bestIPR.ofv, bestPath.ofv));
        }
        bestIPR = bestPath;

        if (runData.debug > 0) {
          std::cout << "[DEBUG][MH] IPR (type=" << typePR << ", alpha=" << alphaBlock
                    << ") encontrou nova melhor solucao: ofv=" << bestIPR.ofv << std::endl;
        }
      }
      UpdatePoolSolutions(bestPath, method, runData.debug, runData.poolUpdateMethod);
    }

    // Q-Learning: recompensa binária e atualização da Q-Table (somente se numDiff > 0)
    if (runData.control == 1 && !S.empty() && numDiff > 0) {
      R = (bestPath.ofv < std::min(atual.ofv, guia.ofv)) ? 1.0 : 0.0;
      UpdateQValue(S, st, at, R, lf, df);
    }

    end_timeMH = get_time_in_seconds();
    currentTime = static_cast<float>(end_timeMH - start_timeMH);
  }

  if (runData.control == 1 && !S.empty()) {
    SaveQTable(S, st, method);
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] IPR finalizado. Melhor OFV=" << bestIPR.ofv << std::endl;
  }
}

} // namespace rkolib::mh