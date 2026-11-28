#include "rkolib/mh/eils.hpp"

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

static constexpr int EILS_NUM_LEVELS = 5;

// -------------------------------------------------------------------------
// Helper Function: EILSPerturbationLevel
// -------------------------------------------------------------------------
static double EILSPerturbationLevel(int level, double betaMin, double betaMax) {
  if (EILS_NUM_LEVELS <= 1) {
    return betaMin;
  }
  return betaMin + level * (betaMax - betaMin) / (EILS_NUM_LEVELS - 1);
}

// -------------------------------------------------------------------------
// Main Algorithm: EILS
// -------------------------------------------------------------------------
void EILS(const TRunData &runData, RkoSolver &solver) {
  const char *method = "EILS";
  int Iter = 0;

  TSol sBest,     // best solution of the E-ILS (s*)
      sLine,      // perturbed solution (s')
      sBestLine;  // local optimum of the perturbed solution (s'')

  double betaMin = 0.10;
  double betaMax = 0.20;
  int Nmax = 3;

  int level = 0;
  int numFail = 0;
  double beta = 0.0;
  bool newCycle = true;
  int cycleImprov = 0;
  int improv = 0;

  float currentTime = 0.0f;
  double start_timeMH = get_time_in_seconds();
  double end_timeMH = get_time_in_seconds();

  int dim = solver.getProblemDimension();
  std::vector<int> RKorder(dim);
  std::iota(RKorder.begin(), RKorder.end(), 0);

  // Q-Learning parameters
  std::vector<TState> S;
  int numPar = 3;
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
      betaMin = parameters[0][0];
    if (!parameters[1].empty())
      betaMax = parameters[1][0];
    if (!parameters[2].empty())
      Nmax = static_cast<int>(parameters[2][0]);
  } else if (runData.control == 1) {
    CreateStates(parameters, numStates, numPar, S);
    LoadQTable(S, method);

    epsilon = 0.95;
    delta = 0.0;
    if (!S.empty()) {
      iCurr = irandomico(0, numStates - 1);
      st = iCurr;
      betaMin = S[iCurr].par[0];
      betaMax = S[iCurr].par[1];
      Nmax = static_cast<int>(S[iCurr].par[2]);
    }
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] EILS iniciado. betaMin=" << betaMin
              << ", betaMax=" << betaMax << ", Nmax=" << Nmax << std::endl;
  }

  // Create initial solution and apply local search (first local optimum s*)
  CreateInitialSolutions(sBest, dim);
  solver.decodeSolution(sBest);
  RVND(sBest, solver, runData.strategy, RKorder);
  UpdatePoolSolutions(sBest, method, runData.debug, runData.poolUpdateMethod);

  end_timeMH = get_time_in_seconds();
  currentTime = static_cast<float>(end_timeMH - start_timeMH);

  bool firstCycle = true;
  double lastNM_time = 0.0;
  const double tauMinNM_sec =
      static_cast<double>(runData.tauMinNM) * runData.MAXTIME;
  const double tauMaxNM_sec =
      static_cast<double>(runData.tauMaxNM) * runData.MAXTIME;

  while (currentTime < runData.MAXTIME * runData.restart) {
    if (SOLVER_SHOULD_STOP) {
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] EILS interrompido via SOLVER_SHOULD_STOP."
                  << std::endl;
      }
      break;
    }

    // Start of a cycle of levels: agent chooses (betaMin, betaMax, Nmax)
    if (newCycle) {
      if (runData.control == 1 && !S.empty() && !firstCycle) {
        SetQLParameter(epsilon, lf, df,
                       static_cast<int>(runData.MAXTIME * runData.restart),
                       currentTime, static_cast<float>(delta), epsilon_min);
        delta = 0.0;

        at = ChooseAction(S, st, epsilon);
        if (at >= 0 && at < static_cast<int>(S[st].Ai.size())) {
          iCurr = S[st].Ai[at];
          betaMin = S[iCurr].par[0];
          betaMax = S[iCurr].par[1];
          Nmax = static_cast<int>(S[iCurr].par[2]);
        }
      } else if (runData.control == 1 && !S.empty()) {
        // First cycle: agent stays in its initial state (action "maintain")
        SetQLParameter(epsilon, lf, df,
                       static_cast<int>(runData.MAXTIME * runData.restart),
                       currentTime, static_cast<float>(delta), epsilon_min);
        at = 0;
        for (int a = 0; a < static_cast<int>(S[st].Ai.size()); a++) {
          if (S[st].Ai[a] == st) {
            at = a;
            break;
          }
        }
      }
      firstCycle = false;

      if (betaMin > betaMax) {
        std::swap(betaMin, betaMax);
      }
      if (Nmax < 1) {
        Nmax = 1;
      }

      level = 0;
      numFail = 0;
      cycleImprov = 0;
      newCycle = false;
    }

    Iter++;
    improv = 0;

    // Perturbation with the intensity of the current level (always from sBest)
    beta = EILSPerturbationLevel(level, betaMin, betaMax);

    sLine = sBest;
    ShakeSolution(sLine, beta, beta, dim);
    solver.decodeSolution(sLine);

    // Local search: s'' <- RVND(s')
    sBestLine = sLine;
    RVND(sBestLine, solver, runData.strategy, RKorder);

    // Acceptance criterion: s'' replaces s* only if it is better
    if (sBestLine.ofv < sBest.ofv) {
      delta = std::max(delta, ComputeDelta(sBest.ofv, sBestLine.ofv));
      sBest = sBestLine;
      improv = 1;

      if (runData.debug > 0) {
        std::cout
            << "[DEBUG][MH] EILS encontrou nova melhor solucao na iteracao "
            << Iter << " (nivel " << level << "): ofv=" << sBest.ofv
            << std::endl;
      }
      UpdatePoolSolutions(sBest, method, runData.debug,
                          runData.poolUpdateMethod);
    }

    // Deterministic condition to call Nelder-Mead
    end_timeMH = get_time_in_seconds();
    currentTime = static_cast<float>(end_timeMH - start_timeMH);
    double deltaT_NM = currentTime - lastNM_time;
    if ((improv && deltaT_NM >= tauMinNM_sec) || (deltaT_NM >= tauMaxNM_sec)) {
      double bestOFV = sBest.ofv;
      NelderMeadSearch(sBest, solver);
      currentTime = static_cast<float>(get_time_in_seconds() - start_timeMH);
      lastNM_time = currentTime;

      if (sBest.ofv < bestOFV) {
        delta = std::max(delta, ComputeDelta(bestOFV, sBest.ofv));
        improv = 1;

        if (runData.debug > 0) {
          std::cout
              << "[DEBUG][MH] EILS (LS) encontrou nova melhor solucao na iteracao "
              << Iter << ": ofv=" << sBest.ofv << std::endl;
        }
        UpdatePoolSolutions(sBest, method, runData.debug,
                            runData.poolUpdateMethod);
      }
    }

    // Update level schedule
    if (improv) {
      cycleImprov = 1;
      newCycle = true;
    } else {
      numFail++;
      if (numFail >= Nmax) {
        numFail = 0;
        level++;
        if (level >= EILS_NUM_LEVELS) {
          newCycle = true;
        }
      }
    }

    // Q-Learning: at the end of a cycle, reward the configuration used in it
    if (runData.control == 1 && !S.empty() && newCycle) {
      R = cycleImprov ? 1.0 : 0.0;
      UpdateQValue(S, st, at, R, lf, df);
    }

    end_timeMH = get_time_in_seconds();
    currentTime = static_cast<float>(end_timeMH - start_timeMH);
  }

  if (runData.control == 1 && !S.empty()) {
    SaveQTable(S, st, method);
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] EILS finalizado. Iteracoes=" << Iter
              << ", Melhor OFV=" << sBest.ofv << std::endl;
  }
}

} // namespace rkolib::mh

