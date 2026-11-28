#include "rkolib/mh/grasp.hpp"

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
// Helper Function: LineSearch (Internal)
// -------------------------------------------------------------------------
static void LineSearch(TSol s, float h, int i, double &bestZ, double &bestF,
                       RkoSolver &solver) {
  bestZ = 0.0;
  bestF = std::numeric_limits<double>::infinity();

  std::vector<double> rk;
  rk.push_back(s.rk[i]);

  int maxTau = (h > 1e-9f) ? static_cast<int>(std::ceil(1.0 / h)) : 1;
  for (int tau = 1; tau <= maxTau; tau++) {
    double up = s.rk[i] + tau * h;
    double down = s.rk[i] - tau * h;

    if (up >= 0.0 && up < 1.0) {
      rk.push_back(up);
    }
    if (down >= 0.0 && down < 1.0) {
      rk.push_back(down);
    }
  }

  int q = 1;
  if (h > 1e-9f) {
    q = static_cast<int>(std::ceil(std::log2(static_cast<int>(1.0 / h)))) + 1;
  }
  if (q > static_cast<int>(rk.size())) {
    q = static_cast<int>(rk.size());
  }

  std::shuffle(rk.begin(), rk.end(), SOLVER_RNG);

  for (int j = 0; j < q; j++) {
    if (SOLVER_SHOULD_STOP) {
      break;
    }

    s.rk[i] = rk[j];
    solver.decodeSolution(s);

    if (s.ofv < bestF) {
      bestZ = s.rk[i];
      bestF = s.ofv;
    }
  }
}

// -------------------------------------------------------------------------
// Helper Function: ConstrutiveGreedyRandomized (Internal)
// -------------------------------------------------------------------------
static void ConstrutiveGreedyRandomized(TSol &s, float h, float alpha,
                                        RkoSolver &solver) {
  int n = solver.getProblemDimension();
  std::vector<int> UnFixed(n);
  std::vector<int> chosenRK;
  std::vector<int> RCL;
  std::vector<double> z(n);
  std::vector<double> g(n, std::numeric_limits<double>::infinity());

  double betaMin = 0.3;
  double betaMax = 0.7;
  double intensity = randomico(betaMin, betaMax);

  std::iota(UnFixed.begin(), UnFixed.end(), 0);
  std::shuffle(UnFixed.begin(), UnFixed.end(), SOLVER_RNG);

  int numRebuild = std::min(n, static_cast<int>(std::ceil(n * intensity)));
  UnFixed.resize(numRebuild);

  for (int k = 0; k < numRebuild; k++) {
    s.rk[UnFixed[k]] = randomico(0.0, 0.999999);
  }
  solver.decodeSolution(s);

  while (!UnFixed.empty()) {
    if (SOLVER_SHOULD_STOP) {
      return;
    }

    double minG = std::numeric_limits<double>::infinity();
    double maxG = -std::numeric_limits<double>::infinity();

    int kMax = static_cast<int>(UnFixed.size() * 0.1);
    if (kMax < 2) {
      kMax = 2;
    }
    if (kMax > static_cast<int>(UnFixed.size())) {
      kMax = static_cast<int>(UnFixed.size());
    }

    chosenRK = UnFixed;
    std::shuffle(chosenRK.begin(), chosenRK.end(), SOLVER_RNG);
    chosenRK.resize(kMax);

    std::fill(z.begin(), z.end(), 0.0);
    std::fill(g.begin(), g.end(), std::numeric_limits<double>::infinity());

    for (int k = 0; k < kMax; k++) {
      int i = chosenRK[k];
      TSol sAux = s;

      LineSearch(sAux, h, i, z[i], g[i], solver);

      if (g[i] < minG) {
        minG = g[i];
      }
      if (g[i] > maxG && g[i] < std::numeric_limits<double>::infinity()) {
        maxG = g[i];
      }
    }

    if (minG == std::numeric_limits<double>::infinity()) {
      break;
    }

    RCL.clear();
    double threshold = minG + alpha * (maxG - minG);
    for (int k = 0; k < kMax; k++) {
      if (g[chosenRK[k]] <= threshold) {
        RCL.push_back(chosenRK[k]);
      }
    }

    if (RCL.empty()) {
      break;
    }

    int kBest = RCL[irandomico(0, static_cast<int>(RCL.size()) - 1)];
    s.rk[kBest] = z[kBest];
    s.ofv = g[kBest];

    auto it = std::find(UnFixed.begin(), UnFixed.end(), kBest);
    if (it != UnFixed.end()) {
      UnFixed.erase(it);
    }
  }

  solver.decodeSolution(s);
}

// -------------------------------------------------------------------------
// Helper Function: GraspInnerStep
// -------------------------------------------------------------------------
static bool GraspInnerStep(TSol &s, TSol &sBest, float &h, float alphaGrasp,
                           int &iter, int &improv, double &delta,
                           float &currentTime, double start_timeMH,
                           double &lastNM_time, double tauMinNM_sec,
                           double tauMaxNM_sec, std::vector<int> &RKorder,
                           RkoSolver &solver, const TRunData &runData,
                           const char *method, TSol &sLineBest) {
  if (SOLVER_SHOULD_STOP) {
    if (runData.debug > 0) {
      std::cout << "[DEBUG][MH] GRASP interrompido via SOLVER_SHOULD_STOP."
                << std::endl;
    }
    return false;
  }

  iter++;

  TSol sLine = s;
  ConstrutiveGreedyRandomized(sLine, h, alphaGrasp, solver);

  sLineBest = sLine;
  RVND(sLineBest, solver, runData.strategy, RKorder);

  if (sLineBest.ofv < sBest.ofv) {
    delta = std::max(delta, ComputeDelta(sBest.ofv, sLineBest.ofv));
    sBest = sLineBest;
    improv = 1;

    if (runData.debug > 0) {
      std::cout << "[DEBUG][MH] GRASP encontrou nova melhor solucao na iteracao "
                << iter << ": ofv=" << sBest.ofv << std::endl;
    }
    UpdatePoolSolutions(sLineBest, method, runData.debug,
                        runData.poolUpdateMethod);
  } else {
    h = h / 2.0f;
  }

  if (sLineBest.ofv < s.ofv) {
    s = sLineBest;
  } else {
    double T = 100.0 * (1.0 - currentTime / (runData.MAXTIME * runData.restart));
    if (T > 0.0 &&
        randomico(0.0, 1.0) < std::exp(-(sLineBest.ofv - s.ofv) / T)) {
      s = sLineBest;
    }
  }

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
        std::cout
            << "[DEBUG][MH] GRASP (LS) encontrou nova melhor solucao na iteracao "
            << iter << ": ofv=" << sBest.ofv << std::endl;
      }
    }
  }

  UpdatePoolSolutions(s, method, runData.debug, runData.poolUpdateMethod);

  currentTime = static_cast<float>(get_time_in_seconds() - start_timeMH);
  return true;
}

// -------------------------------------------------------------------------
// Main Algorithm: GRASP
// -------------------------------------------------------------------------
void GRASP(const TRunData &runData, RkoSolver &solver) {
  const char *method = "GRASP";
  TSol s;
  TSol sLineBest;
  TSol sBest;

  float currentTime = 0.0f;
  int improv = 0;

  double start_timeMH = get_time_in_seconds();
  double end_timeMH = get_time_in_seconds();

  int dim = solver.getProblemDimension();
  std::vector<int> RKorder(dim);
  std::iota(RKorder.begin(), RKorder.end(), 0);

  float alphaGrasp = 0.1f;
  float h = 0.0f;
  float hs = 0.125f;
  float he = 0.00098f;

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
      alphaGrasp = static_cast<float>(parameters[0][0]);
    if (!parameters[1].empty())
      hs = static_cast<float>(parameters[1][0]);
    if (!parameters[2].empty())
      he = static_cast<float>(parameters[2][0]);
  } else if (runData.control == 1) {
    CreateStates(parameters, numStates, numPar, S);
    LoadQTable(S, method);
    epsilon = 0.95;
    delta = 0.0;

    if (!S.empty()) {
      iCurr = irandomico(0, numStates - 1);
      st = iCurr;
      alphaGrasp = static_cast<float>(S[iCurr].par[0]);
      hs = static_cast<float>(S[iCurr].par[1]);
      he = static_cast<float>(S[iCurr].par[2]);
    }
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] GRASP iniciado. alphaGrasp=" << alphaGrasp
              << ", hs=" << hs << ", he=" << he << std::endl;
  }

  CreateInitialSolutions(s, dim);
  solver.decodeSolution(s);
  sBest = s;
  sLineBest = s;

  int iter = 0;
  double lastNM_time = 0.0;
  const double tauMinNM_sec =
      static_cast<double>(runData.tauMinNM) * runData.MAXTIME;
  const double tauMaxNM_sec =
      static_cast<double>(runData.tauMaxNM) * runData.MAXTIME;

  while (currentTime < runData.MAXTIME * runData.restart &&
         !SOLVER_SHOULD_STOP) {
    improv = 0;

    if (runData.control == 1 && !S.empty()) {
      SetQLParameter(epsilon, lf, df,
                     static_cast<int>(runData.MAXTIME * runData.restart),
                     currentTime, static_cast<float>(delta), epsilon_min);
      delta = 0.0;
      at = ChooseAction(S, st, epsilon);

      if (at >= 0 && at < static_cast<int>(S[st].Ai.size())) {
        iCurr = S[st].Ai[at];
        alphaGrasp = static_cast<float>(S[iCurr].par[0]);
        hs = static_cast<float>(S[iCurr].par[1]);
        he = static_cast<float>(S[iCurr].par[2]);
      }
    }

    if (hs < he) {
      std::swap(hs, he);
    }

    h = hs;
    bool stopped = false;
    while (h >= he && h > 1e-9f &&
           currentTime < runData.MAXTIME * runData.restart &&
           !SOLVER_SHOULD_STOP) {
      if (runData.control == 0) {
        alphaGrasp = static_cast<float>(randomico(0.1, 0.9));
      }

      if (!GraspInnerStep(s, sBest, h, alphaGrasp, iter, improv, delta,
                          currentTime, start_timeMH, lastNM_time, tauMinNM_sec,
                          tauMaxNM_sec, RKorder, solver, runData, method,
                          sLineBest)) {
        stopped = true;
        break;
      }
    }
    if (stopped) {
      break;
    }

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
    std::cout << "[DEBUG][MH] GRASP finalizado. Iteracoes=" << iter
              << ", Melhor OFV=" << sBest.ofv << std::endl;
  }
}

} // namespace rkolib::mh
