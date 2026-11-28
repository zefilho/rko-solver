#include "rkolib/mh/ts.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <vector>

#include "rkolib/core/method.hpp"
#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/solver.hpp"
#include "rkolib/utils/io.hpp"

namespace rkolib::mh {

using namespace rkolib::core;

struct TTabuMove {
  int i;
  int j;
  int expiration;
};

// -------------------------------------------------------------------------
// Helper Function: IsMoveTabu
// -------------------------------------------------------------------------
static bool IsMoveTabu(const std::vector<TTabuMove> &tabuList, int i, int j,
                       int iteration) {
  for (const auto &t : tabuList) {
    if (((t.i == i && t.j == j) || (t.i == j && t.j == i)) &&
        iteration < t.expiration) {
      return true;
    }
  }
  return false;
}

// -------------------------------------------------------------------------
// Helper Function: ExploreNeighborhoodTS
// -------------------------------------------------------------------------
static bool ExploreNeighborhoodTS(const TSol &s, const TSol &sBest,
                                  TSol &sBestViz, int &bestI, int &bestJ,
                                  float neighborhoodSize, int iteration,
                                  const std::vector<TTabuMove> &tabuList,
                                  RkoSolver &solver, const TRunData &runData) {
  int dim = solver.getProblemDimension();
  sBestViz.ofv = std::numeric_limits<double>::infinity();
  bestI = -1;
  bestJ = -1;

  int maxSamples =
      static_cast<int>(static_cast<double>(dim) * dim * neighborhoodSize);
  for (int k = 0; k < maxSamples; k++) {
    if (SOLVER_SHOULD_STOP) {
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] TS interrompido via SOLVER_SHOULD_STOP."
                  << std::endl;
      }
      return false;
    }

    TSol sViz = s;
    int i = irandomico(0, dim - 1);
    int j = irandomico(0, dim - 1);
    while (i == j && dim > 1) {
      j = irandomico(0, dim - 1);
    }

    std::swap(sViz.rk[i], sViz.rk[j]);
    solver.decodeSolution(sViz);

    bool isTabu = IsMoveTabu(tabuList, i, j, iteration);

    // Aspiration criterion
    if (sViz.ofv < sBest.ofv) {
      isTabu = false;
    }

    if (!isTabu && sViz.ofv < sBestViz.ofv) {
      sBestViz = sViz;
      bestI = i;
      bestJ = j;
    }
  }
  return true;
}

// -------------------------------------------------------------------------
// Main Algorithm: TS
// -------------------------------------------------------------------------
void TS(const TRunData &runData, RkoSolver &solver) {
  const char *method = "TS";

  TSol s;
  TSol sBest;
  TSol sBestViz;

  float currentTime = 0.0f;
  double start_timeMH = get_time_in_seconds();
  double end_timeMH = get_time_in_seconds();

  int dim = solver.getProblemDimension();
  std::vector<int> RKorder(dim);
  std::iota(RKorder.begin(), RKorder.end(), 0);

  // Tabu Search parameters
  int tabuTenure = 10;
  float neighborhoodSize = 0.5f;
  int maxNoImprovement = 100;
  int noImprovement = 0;
  float betaMin = 0.05f;
  float betaMax = 0.25f;

  std::vector<TTabuMove> tabuList;
  int iteration = 0;

  // Q-Learning parameters
  std::vector<TState> S;
  int numPar = 5;
  int numStates = 0;
  int iCurr = 0;
  double epsilon = 0.0;
  double lf = 0.0;
  double df = 0.0;
  double R = 0.0;
  double delta = 0.0;
  const double epsilon_min = 0.1;
  int st = 0;
  int at = 0;

  std::vector<std::vector<double>> parameters(numPar);
  readParametersYaml(method, runData.control, parameters, numPar);

  if (runData.control == 0) {
    if (!parameters[0].empty())
      maxNoImprovement = static_cast<int>(parameters[0][0]);
    if (!parameters[1].empty())
      tabuTenure = static_cast<int>(parameters[1][0]);
    if (!parameters[2].empty())
      neighborhoodSize = static_cast<float>(parameters[2][0]);
    if (!parameters[3].empty())
      betaMin = static_cast<float>(parameters[3][0]);
    if (!parameters[4].empty())
      betaMax = static_cast<float>(parameters[4][0]);
  } else if (runData.control == 1) {
    CreateStates(parameters, numStates, numPar, S);
    LoadQTable(S, method);

    epsilon = 0.95;
    delta = 0.0;
    if (!S.empty()) {
      iCurr = irandomico(0, numStates - 1);
      st = iCurr;
      maxNoImprovement = static_cast<int>(S[iCurr].par[0]);
      tabuTenure = static_cast<int>(S[iCurr].par[1]);
      neighborhoodSize = static_cast<float>(S[iCurr].par[2]);
      betaMin = static_cast<float>(S[iCurr].par[3]);
      betaMax = static_cast<float>(S[iCurr].par[4]);
    }
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] TS iniciado. maxNoImp=" << maxNoImprovement
              << ", tabuTenure=" << tabuTenure
              << ", neighSize=" << neighborhoodSize << std::endl;
  }

  // Initial solution
  CreateInitialSolutions(s, dim);
  solver.decodeSolution(s);
  sBest = s;
  UpdatePoolSolutions(sBest, method, runData.debug, runData.poolUpdateMethod);

  double lastNM_time = 0.0;
  const double tauMinNM_sec =
      static_cast<double>(runData.tauMinNM) * runData.MAXTIME;
  const double tauMaxNM_sec =
      static_cast<double>(runData.tauMaxNM) * runData.MAXTIME;

  while (currentTime < runData.MAXTIME * runData.restart &&
         !SOLVER_SHOULD_STOP) {
    iteration++;
    int improv = 0;

    // Q-Learning Pre-Action
    if (runData.control == 1 && !S.empty()) {
      SetQLParameter(epsilon, lf, df,
                     static_cast<int>(runData.MAXTIME * runData.restart),
                     currentTime, static_cast<float>(delta), epsilon_min);
      delta = 0.0;

      at = ChooseAction(S, st, epsilon);
      if (at >= 0 && at < static_cast<int>(S[st].Ai.size())) {
        iCurr = S[st].Ai[at];
        maxNoImprovement = static_cast<int>(S[iCurr].par[0]);
        tabuTenure = static_cast<int>(S[iCurr].par[1]);
        neighborhoodSize = static_cast<float>(S[iCurr].par[2]);
        betaMin = static_cast<float>(S[iCurr].par[3]);
        betaMax = static_cast<float>(S[iCurr].par[4]);
      }
    }

    // Generate neighborhood
    int bestI = -1;
    int bestJ = -1;
    if (!ExploreNeighborhoodTS(s, sBest, sBestViz, bestI, bestJ,
                               neighborhoodSize, iteration, tabuList, solver,
                               runData)) {
      break;
    }

    // Apply best move
    if (bestI != -1) {
      s = sBestViz;

      TTabuMove move{bestI, bestJ, iteration + tabuTenure};
      tabuList.push_back(move);

      tabuList.erase(
          std::remove_if(tabuList.begin(), tabuList.end(),
                         [iteration](const TTabuMove &m) {
                           return iteration >= m.expiration;
                         }),
          tabuList.end());

      if (s.ofv < sBest.ofv) {
        delta = std::max(delta, ComputeDelta(sBest.ofv, s.ofv));
        sBest = s;
        noImprovement = 0;
        improv = 1;
        UpdatePoolSolutions(sBest, method, runData.debug,
                            runData.poolUpdateMethod);
      } else {
        noImprovement++;
      }
    }

    // Diversification
    if (noImprovement >= maxNoImprovement) {
      ShakeSolution(s, betaMin, betaMax, dim);
      solver.decodeSolution(s);
      noImprovement = 0;
    }

    // Local Search (RVND)
    RVND(s, solver, runData.strategy, RKorder);
    if (s.ofv < sBest.ofv) {
      delta = std::max(delta, ComputeDelta(sBest.ofv, s.ofv));
      sBest = s;
      improv = 1;
      UpdatePoolSolutions(sBest, method, runData.debug,
                          runData.poolUpdateMethod);
    }

    // Deterministic condition to call Nelder-Mead
    currentTime = static_cast<float>(get_time_in_seconds() - start_timeMH);
    double deltaT_NM = currentTime - lastNM_time;
    if ((improv && deltaT_NM >= tauMinNM_sec) || (deltaT_NM >= tauMaxNM_sec)) {
      NelderMeadSearch(s, solver);
      currentTime = static_cast<float>(get_time_in_seconds() - start_timeMH);
      lastNM_time = currentTime;

      if (s.ofv < sBest.ofv) {
        delta = std::max(delta, ComputeDelta(sBest.ofv, s.ofv));
        sBest = s;
        improv = 1;
        if (runData.debug > 0) {
          std::cout << "[DEBUG][MH] TS (LS) encontrou nova melhor solucao: ofv="
                    << sBest.ofv << std::endl;
        }
        UpdatePoolSolutions(sBest, method, runData.debug,
                            runData.poolUpdateMethod);
      }
    }

    // Q-Learning Update
    if (runData.control == 1 && !S.empty()) {
      R = improv ? 1.0 : -0.01;
      UpdateQValue(S, st, at, R, lf, df);
    }

    end_timeMH = get_time_in_seconds();
    currentTime = static_cast<float>(end_timeMH - start_timeMH);
  }

  if (runData.control == 1 && !S.empty()) {
    SaveQTable(S, st, method);
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] TS finalizado. Iteracoes=" << iteration
              << ", Melhor OFV=" << sBest.ofv << std::endl;
  }
}

} // namespace rkolib::mh

