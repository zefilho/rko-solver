#include "rkolib/mh/ils.hpp"

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

enum {
  ILS_ACCEPT_RANDOM_WALK = 0,
  ILS_ACCEPT_BETTER = 1,
  ILS_ACCEPT_RESTART = 2
};
static constexpr int ILS_ACCEPT = ILS_ACCEPT_RESTART;
static constexpr int ILS_MAX_NO_IMPROV = 10;

void ILS(const TRunData &runData, RkoSolver &solver) {
  const char *method = "ILS";
  int Iter = 0;
  int IterImprov = 0;

  TSol sCurrent, // current solution (accepted local optimum)
      sBest,     // best solution found by the ILS
      sLine,     // perturbed solution
      sBestLine; // local optimum of the perturbed solution

  double betaMin = 0.0;
  double betaMax = 0.0;

  float currentTime = 0.0f;
  int improv = 0;

  double start_timeMH = get_time_in_seconds();
  double end_timeMH = get_time_in_seconds();

  int dim = solver.getProblemDimension();
  std::vector<int> RKorder(dim);
  std::iota(RKorder.begin(), RKorder.end(), 0);

  // Q-Learning parameters
  std::vector<TState> S;
  int numPar = 2;
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
    }
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] ILS iniciado. betaMin=" << betaMin
              << ", betaMax=" << betaMax << std::endl;
  }

  // Create initial solution and apply local search
  CreateInitialSolutions(sBest, dim);
  solver.decodeSolution(sBest);
  RVND(sBest, solver, runData.strategy, RKorder);
  UpdatePoolSolutions(sBest, method, runData.debug, runData.poolUpdateMethod);

  // Search starts from the initial local optimum
  sCurrent = sBest;

  end_timeMH = get_time_in_seconds();
  currentTime = static_cast<float>(end_timeMH - start_timeMH);

  double lastNM_time = 0.0;
  const double tauMinNM_sec =
      static_cast<double>(runData.tauMinNM) * runData.MAXTIME;
  const double tauMaxNM_sec =
      static_cast<double>(runData.tauMaxNM) * runData.MAXTIME;

  while (currentTime < runData.MAXTIME * runData.restart) {
    if (SOLVER_SHOULD_STOP) {
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] ILS interrompido via SOLVER_SHOULD_STOP."
                  << std::endl;
      }
      break;
    }

    Iter++;
    improv = 0;

    // Q-Learning Pre-Action
    if (runData.control == 1 && !S.empty()) {
      SetQLParameter(epsilon, lf, df,
                     static_cast<int>(runData.MAXTIME * runData.restart),
                     currentTime, static_cast<float>(delta), epsilon_min);
      delta = 0.0;

      at = ChooseAction(S, st, epsilon);
      if (at >= 0 && at < static_cast<int>(S[st].Ai.size())) {
        iCurr = S[st].Ai[at];
        betaMin = S[iCurr].par[0];
        betaMax = S[iCurr].par[1];
      }
    }

    // Perturb current solution sCurrent
    sLine = sCurrent;
    ShakeSolution(sLine, betaMin, betaMax, dim);
    solver.decodeSolution(sLine);

    // Local search on perturbed solution
    sBestLine = sLine;
    RVND(sBestLine, solver, runData.strategy, RKorder);

    // Acceptance criterion for sCurrent
    bool accept = false;
    if (ILS_ACCEPT == ILS_ACCEPT_RANDOM_WALK) {
      accept = true;
    } else if (ILS_ACCEPT == ILS_ACCEPT_BETTER) {
      accept = (sBestLine.ofv < sCurrent.ofv);
    } else {
      accept = (sBestLine.ofv < sCurrent.ofv) ||
               (Iter - IterImprov >= ILS_MAX_NO_IMPROV);
    }

    if (accept) {
      if (sBestLine.ofv < sCurrent.ofv || ILS_ACCEPT == ILS_ACCEPT_RESTART) {
        IterImprov = Iter;
      }
      sCurrent = sBestLine;
    }

    // Update global best solution sBest
    if (sBestLine.ofv < sBest.ofv) {
      delta = std::max(delta, ComputeDelta(sBest.ofv, sBestLine.ofv));
      sBest = sBestLine;
      improv = 1;

      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] ILS encontrou nova melhor solucao na iteracao "
                  << Iter << ": ofv=" << sBest.ofv << std::endl;
      }
      UpdatePoolSolutions(sBest, method, runData.debug,
                          runData.poolUpdateMethod);
    }

    // Deterministic condition to call Nelder-Mead (on sCurrent)
    currentTime = static_cast<float>(get_time_in_seconds() - start_timeMH);
    double deltaT_NM = currentTime - lastNM_time;
    if ((improv && deltaT_NM >= tauMinNM_sec) || (deltaT_NM >= tauMaxNM_sec)) {
      double currentOFV = sCurrent.ofv;
      NelderMeadSearch(sCurrent, solver);
      currentTime = static_cast<float>(get_time_in_seconds() - start_timeMH);
      lastNM_time = currentTime;

      if (sCurrent.ofv < currentOFV) {
        IterImprov = Iter;
      }

      if (sCurrent.ofv < sBest.ofv) {
        delta = std::max(delta, ComputeDelta(sBest.ofv, sCurrent.ofv));
        sBest = sCurrent;
        improv = 1;

        if (runData.debug > 0) {
          std::cout
              << "[DEBUG][MH] ILS (LS) encontrou nova melhor solucao na iteracao "
              << Iter << ": ofv=" << sBest.ofv << std::endl;
        }
        UpdatePoolSolutions(sBest, method, runData.debug,
                            runData.poolUpdateMethod);
      }
    }

    // Q-Learning Reward Phase (binary reward)
    if (runData.control == 1 && !S.empty()) {
      R = improv ? 1.0 : 0.0;
      UpdateQValue(S, st, at, R, lf, df);
    }

    end_timeMH = get_time_in_seconds();
    currentTime = static_cast<float>(end_timeMH - start_timeMH);
  }

  if (runData.control == 1 && !S.empty()) {
    SaveQTable(S, st, method);
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] ILS finalizado. Iteracoes=" << Iter
              << ", Melhor OFV=" << sBest.ofv << std::endl;
  }
}

} // namespace rkolib::mh
