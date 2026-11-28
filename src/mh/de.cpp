#include "rkolib/mh/de.hpp"

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

// -------------------------------------------------------------------------
// Helper Function: UpdatePopulationDE
// -------------------------------------------------------------------------
static void UpdatePopulationDE(int sizePop, std::vector<TSol> &Pop,
                               RkoSolver &solver) {
  int oldPsize = static_cast<int>(Pop.size());

  if (oldPsize > sizePop) {
    Pop.resize(sizePop);
  } else if (oldPsize < sizePop) {
    Pop.resize(sizePop);
    int dim = solver.getProblemDimension();
    for (int k = oldPsize; k < sizePop; k++) {
      CreateInitialSolutions(Pop[k], dim);
      solver.decodeSolution(Pop[k]);
    }
  }
}

// -------------------------------------------------------------------------
// Helper Function: GenerateTrialDE
// -------------------------------------------------------------------------
static TSol GenerateTrialDE(int targetIdx, int sizePop, double Cr, double F,
                            const std::vector<TSol> &Pop, const TSol &bestInd,
                            RkoSolver &solver) {
  int r1, r2, r3;
  do {
    r1 = irandomico(0, sizePop - 1);
  } while (r1 == targetIdx);
  do {
    r2 = irandomico(0, sizePop - 1);
  } while (r2 == targetIdx || r2 == r1);
  do {
    r3 = irandomico(0, sizePop - 1);
  } while (r3 == targetIdx || r3 == r1 || r3 == r2);

  int dim = solver.getProblemDimension();
  TSol trial;
  trial.rk.resize(dim);
  int jrand = irandomico(0, dim - 1);

  for (int j = 0; j < dim; ++j) {
    if (randomico(0, 1) <= Cr || j == jrand) {
      double v;
      if (randomico(0, 1) < 0.5) {
        v = Pop[r1].rk[j] + (F * (Pop[r2].rk[j] - Pop[r3].rk[j]));
      } else {
        v = Pop[targetIdx].rk[j] + F * (bestInd.rk[j] - Pop[r1].rk[j]) +
            F * (Pop[r2].rk[j] - Pop[r3].rk[j]);
      }
      // Wrap-around repair to [0, 1)
      trial.rk[j] = std::clamp(v - std::floor(v), 0.0, 0.9999999);
    } else {
      trial.rk[j] = Pop[targetIdx].rk[j];
    }
  }

  solver.decodeSolution(trial);
  return trial;
}

// -------------------------------------------------------------------------
// Helper Function: EvolveGenerationDE
// -------------------------------------------------------------------------
static bool EvolveGenerationDE(int sizePop, double Cr, double F,
                               const std::vector<TSol> &Pop,
                               std::vector<TSol> &PopNew, TSol &bestInd,
                               double &bestOFcurrent, int &bestGeneration,
                               int numGenerations, int &improv, double &delta,
                               float currentTime, std::vector<int> &RKorder,
                               RkoSolver &solver, const TRunData &runData,
                               const char *method) {
  double maxTimeRestart =
      std::max(1e-9, static_cast<double>(runData.MAXTIME) * runData.restart);

  for (int i = 0; i < sizePop; ++i) {
    if (SOLVER_SHOULD_STOP) {
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] DE interrompido via SOLVER_SHOULD_STOP."
                  << std::endl;
      }
      return false;
    }

    TSol trial = GenerateTrialDE(i, sizePop, Cr, F, Pop, bestInd, solver);

    if (randomico(0, 1) < 0.05) {
      RVND(trial, solver, runData.strategy, RKorder);
    }

    if (trial.ofv <= Pop[i].ofv) {
      PopNew[i] = trial;
    } else {
      double denom =
          std::max(1e-9, 1000.0 - 1000.0 * (currentTime / maxTimeRestart));
      if (randomico(0, 1) < std::exp(-(trial.ofv - Pop[i].ofv) / denom)) {
        PopNew[i] = trial;
      } else {
        PopNew[i] = Pop[i];
      }
    }

    if (PopNew[i].ofv < bestOFcurrent) {
      bestOFcurrent = PopNew[i].ofv;
    }

    if (PopNew[i].ofv < bestInd.ofv) {
      delta = std::max(delta, ComputeDelta(bestInd.ofv, PopNew[i].ofv));
      bestInd = PopNew[i];
      bestGeneration = numGenerations;
      improv = 1;
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] DE encontrou novo melhor individuo na geracao "
                  << numGenerations << ": ofv=" << bestInd.ofv << std::endl;
      }
      UpdatePoolSolutions(bestInd, method, runData.debug,
                          runData.poolUpdateMethod);
    }
  }
  return true;
}

// -------------------------------------------------------------------------
// Main Algorithm: DE
// -------------------------------------------------------------------------
void DE(const TRunData &runData, RkoSolver &solver) {
  const char *method = "DE";
  int sizePop = 200;
  double Cr = 0.50;
  double F = 0.50;

  std::vector<TSol> Pop;
  std::vector<TSol> PopNew;
  TSol bestInd;

  int numGenerations = 0;
  float currentTime = 0.0f;
  int bestGeneration = 0;
  int improv = 0;

  double start_timeMH = get_time_in_seconds();
  double end_timeMH = get_time_in_seconds();

  std::vector<int> RKorder(solver.getProblemDimension());
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
  const double math_eps = 1e-9;
  double delta = 0.0;
  const double epsilon_min = 0.1;
  int st = 0;
  int at = 0;

  std::vector<std::vector<double>> parameters(numPar);
  readParametersYaml(method, runData.control, parameters, numPar);

  if (runData.control == 0) {
    if (!parameters[0].empty())
      sizePop = static_cast<int>(parameters[0][0]);
    if (!parameters[1].empty())
      Cr = parameters[1][0];
    if (!parameters[2].empty())
      F = parameters[2][0];
  } else if (runData.control == 1) {
    CreateStates(parameters, numStates, numPar, S);
    LoadQTable(S, method);

    epsilon = 0.95;
    delta = 0.0;
    if (!S.empty()) {
      iCurr = irandomico(0, numStates - 1);
      st = iCurr;
      sizePop = static_cast<int>(S[iCurr].par[0]);
      Cr = S[iCurr].par[1];
      F = S[iCurr].par[2];
    }
  }

  sizePop = std::max(4, sizePop);

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] DE iniciado. sizePop=" << sizePop
              << ", Cr=" << Cr << ", F=" << F << std::endl;
  }

  Pop.resize(sizePop);
  PopNew.resize(sizePop);
  bestInd.ofv = std::numeric_limits<double>::infinity();

  int dim = solver.getProblemDimension();
  for (int i = 0; i < sizePop; ++i) {
    CreateInitialSolutions(Pop[i], dim);
    solver.decodeSolution(Pop[i]);
    if (Pop[i].ofv < bestInd.ofv) {
      bestInd = Pop[i];
    }
  }

  double lastNM_time = 0.0;
  const double tauMinNM_sec =
      static_cast<double>(runData.tauMinNM) * runData.MAXTIME;
  const double tauMaxNM_sec =
      static_cast<double>(runData.tauMaxNM) * runData.MAXTIME;

  while (currentTime < runData.MAXTIME * runData.restart &&
         !SOLVER_SHOULD_STOP) {
    numGenerations++;
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
        sizePop = std::max(4, static_cast<int>(S[iCurr].par[0]));
        Cr = S[iCurr].par[1];
        F = S[iCurr].par[2];

        UpdatePopulationDE(sizePop, Pop, solver);
        PopNew.resize(sizePop);
      }
    }

    double bestOFcurrent = std::numeric_limits<double>::infinity();

    if (!EvolveGenerationDE(sizePop, Cr, F, Pop, PopNew, bestInd, bestOFcurrent,
                            bestGeneration, numGenerations, improv, delta,
                            currentTime, RKorder, solver, runData, method)) {
      break;
    }

    // Deterministic condition to call Nelder-Mead
    currentTime = static_cast<float>(get_time_in_seconds() - start_timeMH);
    double deltaT_NM = currentTime - lastNM_time;
    if ((improv && deltaT_NM >= tauMinNM_sec) || (deltaT_NM >= tauMaxNM_sec)) {
      double bestOFV = bestInd.ofv;
      NelderMeadSearch(bestInd, solver);
      currentTime = static_cast<float>(get_time_in_seconds() - start_timeMH);
      lastNM_time = currentTime;

      if (bestInd.ofv < bestOFV) {
        delta = std::max(delta, ComputeDelta(bestOFV, bestInd.ofv));
        bestGeneration = numGenerations;
        improv = 1;
        if (!PopNew.empty()) {
          int pos = irandomico(0, static_cast<int>(PopNew.size()) - 1);
          PopNew[pos] = bestInd;
        }
        if (runData.debug > 0) {
          std::cout << "[DEBUG][MH] DE (LS) encontrou novo melhor individuo: ofv="
                    << bestInd.ofv << std::endl;
        }
        UpdatePoolSolutions(bestInd, method, runData.debug,
                            runData.poolUpdateMethod);
      }
    }

    Pop = PopNew;

    // Q-Learning Reward Phase (Post-Action)
    if (runData.control == 1 && !S.empty()) {
      if (improv) {
        R = 1.0;
        improv = 0;
      } else {
        R = (std::abs(bestOFcurrent) > math_eps)
                ? (bestInd.ofv - bestOFcurrent) /
                      (std::abs(bestOFcurrent) + math_eps)
                : 0.0;
      }
      UpdateQValue(S, st, at, R, lf, df);
    }

    end_timeMH = get_time_in_seconds();
    currentTime = static_cast<float>(end_timeMH - start_timeMH);
  }

  if (runData.control == 1 && !S.empty()) {
    SaveQTable(S, st, method);
  }

  Pop.clear();
  PopNew.clear();

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] DE finalizado. Geracoes=" << numGenerations
              << ", Melhor Geracao=" << bestGeneration
              << ", Melhor OFV=" << bestInd.ofv << std::endl;
  }
}

} // namespace rkolib::mh

