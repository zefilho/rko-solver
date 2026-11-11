#include "rkolib/mh/lns.hpp"

#include <algorithm>
#include "rkolib/core/method.hpp"
#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

using namespace rkolib::core;

// -------------------------------------------------------------------------
// Helper Function: fareySequence
// -------------------------------------------------------------------------
static void fareySequence(int num, std::vector<double> &F) {
  int a = 0, b = 1, c = 1, d = num;

  F.push_back((double)a / b);

  while (c <= num) {
    int k = (num + b) / d;
    int temp_a = a, temp_b = b;
    a = c;
    b = d;
    c = k * c - temp_a;
    d = k * d - temp_b;

    if (a != b) {
      F.push_back((double)a / b);
    } else {
      F.push_back(((double)a / b) - 0.00001);
    }
  }
}

// -------------------------------------------------------------------------
// Helper Function: FindBestFareyInterval
// -------------------------------------------------------------------------
static bool FindBestFareyInterval(TSol &sLine, int pos, const std::vector<double> &F,
                                  double &OFVbest, double &rkBest,
                                  RkoSolver &solver, const TRunData &runData) {
  for (int j = 0; j < (int)F.size() - 1; j++) {
    if (SOLVER_SHOULD_STOP) {
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] LNS interrompido via SOLVER_SHOULD_STOP." << std::endl;
      }
      return false;
    }

    sLine.rk[pos] = randomico(F[j], F[j + 1]);
    solver.decodeSolution(sLine);

    if (sLine.ofv < OFVbest) {
      OFVbest = sLine.ofv;
      rkBest = sLine.rk[pos];
    }
  }
  return true;
}

// -------------------------------------------------------------------------
// Helper Function: RepairSolution
// -------------------------------------------------------------------------
static bool RepairSolution(TSol &sLine, int intensity,
                           const std::vector<int> &RKorder,
                           const std::vector<double> &F,
                           RkoSolver &solver, const TRunData &runData) {
  for (int k = 0; k < intensity; k++) {
    int pos = RKorder[k];
    double OFVbest = INFINITY;
    double rkBest = 0;

    if (!FindBestFareyInterval(sLine, pos, F, OFVbest, rkBest, solver, runData)) {
      return false;
    }

    sLine.ofv = OFVbest;
    sLine.rk[pos] = rkBest;
  }
  return true;
}

// -------------------------------------------------------------------------
// Helper Function: AcceptLnsSolution
// -------------------------------------------------------------------------
static void AcceptLnsSolution(TSol &s, TSol &sBest, const TSol &sLineBest,
                              double T, int &improv, double &delta,
                              const double math_eps, const TRunData &runData,
                              const char *method) {
  double diff = sLineBest.ofv - s.ofv;
  if (diff <= 0) {
    s = sLineBest;

    if (s.ofv < sBest.ofv) {
      delta = (sBest.ofv - s.ofv) / (sBest.ofv + math_eps);
      sBest = s;
      improv = 1;

      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] LNS encontrou nova melhor solucao: ofv=" 
                  << sBest.ofv << std::endl;
      }

      UpdatePoolSolutions(s, method, runData.debug, runData.poolUpdateMethod);
    }
  } else {
    double x = randomico(0, 1);
    if (x < std::exp(-diff / T)) {
      s = sLineBest;
    }
  }
}

// -------------------------------------------------------------------------
// Helper Function: RunLnsCoolingCycle
// -------------------------------------------------------------------------
static bool RunLnsCoolingCycle(TSol &s, TSol &sBest, double &T, double alphaLNS,
                               float betaMin, float betaMax,
                               std::vector<int> &RKorder, const std::vector<double> &F,
                               int &improv, double &delta, int &st, int &at,
                               std::vector<TState> &S, double &epsilon, double lf,
                               double df, float &currentTime, double start_timeMH,
                               const double math_eps, RkoSolver &solver,
                               const TRunData &runData, const char *method) {
  while (T > 0.01 && currentTime < runData.MAXTIME * runData.restart) {
    if (runData.control == 1 && !S.empty()) {
      SetQLParameter(epsilon, lf, df, (int)(runData.MAXTIME * runData.restart),
                     currentTime, delta);
      delta = 0.0;
      at = ChooseAction(S, st, epsilon);

      if (at >= 0 && at < (int)S[st].Ai.size()) {
        int iCurr = S[st].Ai[at];
        betaMin = S[iCurr].par[0];
        betaMax = S[iCurr].par[1];
        alphaLNS = S[iCurr].par[3];
      }
    }

    TSol sLine = s;
    int intensity = irandomico((int)(betaMin * solver.getProblemDimension()),
                               (int)(betaMax * solver.getProblemDimension()));
    intensity = std::clamp(intensity, 1, solver.getProblemDimension());

    std::shuffle(RKorder.begin(), RKorder.end(), SOLVER_RNG);

    if (!RepairSolution(sLine, intensity, RKorder, F, solver, runData)) {
      return false;
    }

    TSol sLineBest = sLine;
    NelderMeadSearch(sLineBest, solver);

    AcceptLnsSolution(s, sBest, sLineBest, T, improv, delta, math_eps, runData, method);

    if (runData.control == 1 && !S.empty()) {
      double R = 0.0;
      if (improv) {
        R = 1;
        improv = 0;
      } else {
        R = (std::abs(s.ofv) > 1e-9) ? (sBest.ofv - s.ofv) / (std::abs(s.ofv) + 1e-9) : 0;
      }

      UpdateQValue(S, st, at, R, lf, df);
    }

    T = T * alphaLNS;

    double end_timeMH = get_time_in_seconds();
    currentTime = (float)(end_timeMH - start_timeMH);
  }
  return true;
}

// -------------------------------------------------------------------------
// Main Algorithm: LNS
// -------------------------------------------------------------------------
void LNS(const TRunData &runData, RkoSolver &solver) {
  const char *method = "LNS";
  double T0 = 0;        // initial temperature
  double T;             // current temperature
  double alphaLNS = 0;  // cool rate
  double betaMin = 0.0; // minimum perturbation
  double betaMax = 0.0; // maximum perturbation
  int reanneling = 0;   // reanneling flag

  TSol s, sBest;

  float currentTime = 0;
  int improv = 0;

  double start_timeMH = get_time_in_seconds();

  std::vector<int> RKorder(solver.getProblemDimension());
  std::iota(RKorder.begin(), RKorder.end(), 0);

  // ---------------------------------------------------------------------
  // Q-Learning parameters
  // ---------------------------------------------------------------------
  std::vector<TState> S;
  int numPar = 0;
  int numStates = 0;
  int iCurr = 0;
  double epsilon = 0;
  double lf = 0;
  double df = 0;

  const double math_eps = 1e-9;
  double delta = 0.0;
  int st = 0;
  int at = 0;

  numPar = 4;
  std::vector<std::vector<double>> parameters;
  parameters.resize(numPar);

  readParametersYaml(method, runData.control, parameters, numPar);

  std::vector<double> F;
  fareySequence(7, F);

  // offline control
  if (runData.control == 0) {
    if (!parameters[0].empty()) { betaMin = parameters[0][0]; }
    if (!parameters[1].empty()) { betaMax = parameters[1][0]; }
    if (!parameters[2].empty()) { T0 = parameters[2][0]; }
    if (!parameters[3].empty()) { alphaLNS = parameters[3][0]; }
  } else if (runData.control == 1) {
    CreateStates(parameters, numStates, numPar, S);
    epsilon = 0.95;
    delta = 0.0;

    iCurr = irandomico(0, numStates - 1);
    st = iCurr;

    if (!S.empty()) {
      betaMin = S[iCurr].par[0];
      betaMax = S[iCurr].par[1];
      T0 = S[iCurr].par[2];
      alphaLNS = S[iCurr].par[3];
    }
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] LNS iniciado. betaMin=" << betaMin << ", betaMax=" << betaMax 
              << ", T0=" << T0 << ", alphaLNS=" << alphaLNS << std::endl;
  }

  CreateInitialSolutions(s, solver.getProblemDimension());
  solver.decodeSolution(s);
  sBest = s;

  while (currentTime < runData.MAXTIME * runData.restart) {
    T = (!reanneling) ? T0 : (T0 * 0.3);

    if (!RunLnsCoolingCycle(s, sBest, T, alphaLNS, betaMin, betaMax, RKorder,
                            F, improv, delta, st, at, S, epsilon, lf, df,
                            currentTime, start_timeMH, math_eps, solver,
                            runData, method)) {
      return;
    }

    reanneling = 1;
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] LNS finalizado. Melhor OFV=" << sBest.ofv << std::endl;
  }
}

} // namespace rkolib::mh