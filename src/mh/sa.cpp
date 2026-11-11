#include "rkolib/mh/sa.hpp"

#include "rkolib/core/method.hpp"
#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

static bool MetropolisStep(rkolib::core::TSol &s, rkolib::core::TSol &sBest, double T,
                           float betaMin, float betaMax, double &bestOFV,
                           rkolib::RkoSolver &solver,
                           const rkolib::core::TRunData &runData, const char *method) {
  using namespace rkolib::core;
  TSol sViz = s;
  ShakeSolution(sViz, betaMin, betaMax, solver.getProblemDimension());
  solver.decodeSolution(sViz);

  if (sViz.ofv < bestOFV) {
    bestOFV = sViz.ofv;
  }

  double delta = sViz.ofv - s.ofv;
  bool improvedBest = false;
  if (delta < 0) {
    s = sViz;
    if (s.ofv < sBest.ofv) {
      sBest = s;
      improvedBest = true;
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] SA encontrou nova melhor solucao: ofv=" << sBest.ofv
                  << " (Temp=" << T << ")" << std::endl;
      }
      UpdatePoolSolutions(s, method, runData.debug, runData.poolUpdateMethod);
    }
  } else {
    double x = randomico(0, 1);
    if (x < std::exp(-delta / T)) {
      s = sViz;
    }
  }
  return improvedBest;
}

static bool RunMetropolisEpoch(rkolib::core::TSol &s, rkolib::core::TSol &sBest, double T,
                               int SAmax, int &IterT, float betaMin, float betaMax,
                               double &bestOFV, float currentTime,
                               rkolib::RkoSolver &solver,
                               const rkolib::core::TRunData &runData, const char *method) {
  bool anyImprovement = false;
  while (IterT < SAmax && currentTime < runData.MAXTIME * runData.restart) {
    if (SOLVER_SHOULD_STOP) {
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] SA interrompido via SOLVER_SHOULD_STOP." << std::endl;
      }
      return anyImprovement;
    }

    IterT++;
    if (MetropolisStep(s, sBest, T, betaMin, betaMax, bestOFV, solver, runData, method)) {
      anyImprovement = true;
    }
  }
  return anyImprovement;
}

static bool RunTemperatureLoop(rkolib::core::TSol &s, rkolib::core::TSol &sBest, double &T,
                               double alphaSA, float betaMin, float betaMax,
                               int SAmax, int &IterT, int &improv, double &delta_q,
                               int &st, int &at, std::vector<rkolib::core::TState> &S,
                               double &epsilon, double lf, double df,
                               float &currentTime, double start_timeMH,
                               const double math_eps, rkolib::RkoSolver &solver,
                               const rkolib::core::TRunData &runData, const char *method) {
  using namespace rkolib::core;

  while (T > 0.0001 && currentTime < runData.MAXTIME * runData.restart) {
    double bestOfvStartGen = sBest.ofv;

    if (runData.control == 1 && !S.empty()) {
      SetQLParameter(epsilon, lf, df, (int)(runData.MAXTIME * runData.restart),
                     currentTime, delta_q);
      delta_q = 0.0;
      at = ChooseAction(S, st, epsilon);

      if (at >= 0 && at < (int)S[st].Ai.size()) {
        int iCurr = S[st].Ai[at];
        SAmax = (int)S[iCurr].par[0];
        alphaSA = S[iCurr].par[1];
        betaMin = S[iCurr].par[2];
        betaMax = S[iCurr].par[3];
      }
    }

    double bestOFV = INFINITY;

    if (RunMetropolisEpoch(s, sBest, T, SAmax, IterT, betaMin, betaMax,
                           bestOFV, currentTime, solver, runData, method)) {
      improv = 1;
    }

    if (SOLVER_SHOULD_STOP) {
      return false;
    }

    if (runData.control == 1 && !S.empty()) {
      double R = 0.0;
      if (improv) {
        R = 1;
        improv = 0;
        delta_q = (bestOfvStartGen - sBest.ofv) / (bestOfvStartGen + math_eps);
      } else {
        R = (std::abs(bestOFV) > math_eps) ? (sBest.ofv - bestOFV) / (std::abs(bestOFV) + math_eps) : 0;
      }

      UpdateQValue(S, st, at, R, lf, df);
    }

    T = T * alphaSA;
    IterT = 0;

    TSol sViz = s;
    NelderMeadSearch(sViz, solver);

    if (sViz.ofv < sBest.ofv) {
      sBest = sViz;

      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] SA (LS) encontrou nova melhor solucao: ofv=" << sBest.ofv << std::endl;
      }

      UpdatePoolSolutions(sBest, method, runData.debug, runData.poolUpdateMethod);
    }

    double end_timeMH = get_time_in_seconds();
    currentTime = (float)(end_timeMH - start_timeMH);
  }
  return true;
}

void SA(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver) {
  using namespace rkolib::core;

  const char *method = "SA";
  TSol s;     // current solution
  TSol sBest; // best solution of SA

  double T0 = 0;         // initial temperature
  double T = 0;          // current temperature
  double alphaSA = 0;    // cool rate
  int SAmax = 0;         // number of iterations in a temperature T
  int IterT = 0;         // iteracao corrente
  float betaMin = 0;     // minimum perturbation
  float betaMax = 0;     // maximum perturbation
  int reanneling = 0;    // reanneling flag
  int improv = 0;        // improvement flag
  float currentTime = 0; // computational time of the search process

  double start_timeMH = get_time_in_seconds();

  std::vector<int> RKorder(solver.getProblemDimension());
  std::iota(RKorder.begin(), RKorder.end(), 0);

  // ---------------------------------------------------------------------
  // Q-Learning Parameters
  // ---------------------------------------------------------------------
  std::vector<TState> S;
  int numPar = 0;
  int numStates = 0;
  int iCurr = 0;
  double epsilon = 0;
  double lf = 0;
  double df = 0;

  const double math_eps = 1e-9;
  double delta_q = 0.0;
  int st = 0;
  int at = 0;

  numPar = 5;
  std::vector<std::vector<double>> parameters;
  parameters.resize(numPar);

  readParametersYaml(method, runData.control, parameters, numPar);

  // offline control
  if (runData.control == 0) {
    if (!parameters[0].empty()) { SAmax = (int)parameters[0][0]; }
    if (!parameters[1].empty()) { alphaSA = parameters[1][0]; }
    if (!parameters[2].empty()) { betaMin = parameters[2][0]; }
    if (!parameters[3].empty()) { betaMax = parameters[3][0]; }
    if (!parameters[4].empty()) { T0 = parameters[4][0]; }
  } else if (runData.control == 1) {
    CreateStates(parameters, numStates, numPar, S);
    epsilon = 0.95;
    delta_q = 0.0;

    iCurr = irandomico(0, numStates - 1);
    st = iCurr;

    if (!S.empty()) {
      SAmax = (int)S[iCurr].par[0];
      alphaSA = S[iCurr].par[1];
      betaMin = S[iCurr].par[2];
      betaMax = S[iCurr].par[3];
      T0 = S[iCurr].par[4];
    }
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] SA iniciado. SAmax=" << SAmax << ", alphaSA=" << alphaSA 
              << ", betaMin=" << betaMin << ", betaMax=" << betaMax << ", T0=" << T0 << std::endl;
  }

  CreateInitialSolutions(s, solver.getProblemDimension());
  solver.decodeSolution(s);
  sBest = s;

  // ---------------------------------------------------------------------
  // Main Loop
  // ---------------------------------------------------------------------
  while (currentTime < runData.MAXTIME * runData.restart) {
    IterT = 0;
    T = (!reanneling) ? T0 : (T0 * 0.3);

    if (!RunTemperatureLoop(s, sBest, T, alphaSA, betaMin, betaMax, SAmax,
                            IterT, improv, delta_q, st, at, S, epsilon, lf,
                            df, currentTime, start_timeMH, math_eps, solver,
                            runData, method)) {
      return;
    }

    reanneling = 1;
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] SA finalizado. Melhor OFV=" << sBest.ofv << std::endl;
  }
}

} // namespace rkolib::mh