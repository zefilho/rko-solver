#include "rkolib/mh/grasp.hpp"

#include "rkolib/core/method.hpp"
#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

using namespace rkolib::core;

// -------------------------------------------------------------------------
// Helper Function: LineSearch (Internal)
// -------------------------------------------------------------------------
static void LineSearch(TSol s, float h, int i, double &bestZ, double &bestF,
                       RkoSolver &solver) {
  bestZ = 0;
  bestF = INFINITY;

  double tau = 0;
  std::vector<double> rk;
  rk.reserve((int)(1.0 / h) + 2);

  rk.push_back(s.rk[i] + tau * h);

  int steps = (int)(1.0 / h) + 1;
  for (int j = 0; j < steps; j += 2) {
    tau++;

    double val_pos = s.rk[i] + tau * h;
    if (val_pos >= 0.0 && val_pos < 1.0) {
      rk.push_back(val_pos);
    }

    double val_neg = s.rk[i] - tau * h;
    if (val_neg >= 0.0 && val_neg < 1.0) {
      rk.push_back(val_neg);
    }
  }

  int q = 1;
  if (h > 1e-9) {
    q = (int)ceil(log2(1.0 / h)) + 1;
  }

  if (q > (int)rk.size()) {
    q = (int)rk.size();
  }

  static std::mt19937 local_rng(std::random_device{}());
  std::shuffle(rk.begin(), rk.end(), local_rng);

  for (int j = 0; j < q; j++) {
    if (SOLVER_SHOULD_STOP) {
      return;
    }

    s.rk[i] = rk[j];
    solver.decodeSolution(s);

    if (s.ofv < bestF) {
      bestZ = s.rk[i];
      bestF = s.ofv;
    }
  }
  rk.clear();
}

// -------------------------------------------------------------------------
// Helper Function: ConstrutiveGreedyRandomized (Internal)
// -------------------------------------------------------------------------
static void ConstrutiveGreedyRandomized(TSol &s, float h, float alpha,
                                        RkoSolver &solver) {
  std::vector<int> UnFixed(solver.getProblemDimension());
  std::vector<int> chosenRK;
  std::vector<double> z(solver.getProblemDimension());
  std::vector<double> g(solver.getProblemDimension(), INFINITY);

  double min;
  double betaMin = 0.3;
  double betaMax = 0.7;

  std::iota(UnFixed.begin(), UnFixed.end(), 0);
  double intensity = randomico(betaMin, betaMax);
  int kBest = 0;

  int limit = (int)(solver.getProblemDimension() * intensity);
  for (int j = 0; j < limit; j++) {
    if (SOLVER_SHOULD_STOP) {
      return;
    }

    min = INFINITY;
    chosenRK.clear();

    int kMax = (int)(UnFixed.size() * 0.1);
    if (kMax < 2) {
      kMax = (int)UnFixed.size();
    }

    chosenRK = UnFixed;
    std::shuffle(chosenRK.begin(), chosenRK.end(), SOLVER_RNG);
    if (kMax < (int)chosenRK.size()) {
      chosenRK.resize(kMax);
    }

    for (int k = 0; k < kMax; k++) {
      int i = chosenRK[k];
      TSol sAux = s;

      LineSearch(sAux, h, i, z[i], g[i], solver);

      if (min > g[i]) {
        min = g[i];
        kBest = i;
      }
    }

    s.rk[kBest] = z[kBest];
    s.ofv = g[kBest];

    for (int i = 0; i < (int)UnFixed.size(); i++) {
      if (UnFixed[i] == kBest) {
        UnFixed.erase(UnFixed.begin() + i);
        break;
      }
    }
  }

  solver.decodeSolution(s);
}

// -------------------------------------------------------------------------
// Helper Function: GraspInnerStep
// -------------------------------------------------------------------------
static bool GraspInnerStep(TSol &s, TSol &sBest, float &h, float alphaGrasp,
                           int &iter, int &improv, double &delta,
                           const double math_eps, float currentTime,
                           std::vector<int> &RKorder, RkoSolver &solver,
                           const TRunData &runData, const char *method,
                           TSol &sLineBest) {
  if (SOLVER_SHOULD_STOP) {
    if (runData.debug > 0) {
      std::cout << "[DEBUG][MH] GRASP interrompido via SOLVER_SHOULD_STOP." << std::endl;
    }
    return false;
  }

  iter++;

  TSol sLine = s;
  ConstrutiveGreedyRandomized(sLine, h, alphaGrasp, solver);

  sLineBest = sLine;
  RVND(sLineBest, solver, runData.strategy, RKorder);

  if (sLineBest.ofv < sBest.ofv) {
    delta = (sBest.ofv - sLineBest.ofv) / (sBest.ofv + math_eps);
    sBest = sLineBest;
    improv = 1;

    if (runData.debug > 0) {
      std::cout << "[DEBUG][MH] GRASP encontrou nova melhor solucao na iteracao " 
                << iter << ": ofv=" << sBest.ofv << std::endl;
    }

    UpdatePoolSolutions(sLineBest, method, runData.debug, runData.poolUpdateMethod);
  } else {
    h = h / 2.0f;
  }

  if (sLineBest.ofv < s.ofv) {
    s = sLineBest;
  } else {
    double x = randomico(0.0, 1.0);
    double timeFactor = 100.0 - 100.0 * (currentTime / (runData.MAXTIME * runData.restart + math_eps));
    if (timeFactor < 1.0) {
      timeFactor = 1.0;
    }

    if (x < exp(-(sLineBest.ofv - s.ofv) / timeFactor)) {
      s = sLineBest;
    }
  }

  return true;
}

// -------------------------------------------------------------------------
// Main Algorithm: GRASP
// -------------------------------------------------------------------------
void GRASP(const TRunData &runData, RkoSolver &solver) {
  const char *method = "GRASP";
  TSol s;         // current solution
  TSol sLineBest; // local search solution
  TSol sBest;     // best solution of GRASP

  float currentTime = 0; // computational time of the search process
  int improv = 0;        // improvement flag

  double start_timeMH = get_time_in_seconds();
  double end_timeMH = get_time_in_seconds();

  std::vector<int> RKorder(solver.getProblemDimension());
  std::iota(RKorder.begin(), RKorder.end(), 0);

  float alphaGrasp = 0.1f;
  float h = 0;
  float hs = 0;
  float he = 0;

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
  double R = 0;

  const double math_eps = 1e-9;
  double delta = 0.0;
  int st = 0;
  int at = 0;

  numPar = 3;
  std::vector<std::vector<double>> parameters;
  parameters.resize(numPar);

  readParametersYaml(method, runData.control, parameters, numPar);

  // offline control
  if (runData.control == 0) {
    if (!parameters[0].empty()) { alphaGrasp = (float)parameters[0][0]; }
    if (!parameters[1].empty()) { hs = (float)parameters[1][0]; }
    if (!parameters[2].empty()) { he = (float)parameters[2][0]; }
  } else if (runData.control == 1) {
    CreateStates(parameters, numStates, numPar, S);
    epsilon = 0.95;
    delta = 0.0;

    iCurr = irandomico(0, numStates - 1);
    st = iCurr;

    if (!S.empty()) {
      alphaGrasp = (float)S[iCurr].par[0];
      hs = (float)S[iCurr].par[1];
      he = (float)S[iCurr].par[2];
    }
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] GRASP iniciado. alphaGrasp=" << alphaGrasp << ", hs=" << hs 
              << ", he=" << he << std::endl;
  }

  CreateInitialSolutions(s, solver.getProblemDimension());
  solver.decodeSolution(s);
  sBest = s;

  int iter = 0;

  while (currentTime < runData.MAXTIME * runData.restart) {
    if (runData.control == 1 && !S.empty()) {
      SetQLParameter(epsilon, lf, df, (int)(runData.MAXTIME * runData.restart),
                     currentTime, delta);
      delta = 0.0;
      at = ChooseAction(S, st, epsilon);

      if (at >= 0 && at < (int)S[st].Ai.size()) {
        iCurr = S[st].Ai[at];
        alphaGrasp = (float)S[iCurr].par[0];
        hs = (float)S[iCurr].par[1];
        he = (float)S[iCurr].par[2];
      }
    }

    h = hs;
    while (h >= he && currentTime < runData.MAXTIME * runData.restart) {
      if (runData.control == 0) {
        alphaGrasp = (float)randomico(0.1, 0.9);
      }

      if (!GraspInnerStep(s, sBest, h, alphaGrasp, iter, improv, delta,
                          math_eps, currentTime, RKorder, solver, runData,
                          method, sLineBest)) {
        return;
      }

      end_timeMH = get_time_in_seconds();
      currentTime = (float)(end_timeMH - start_timeMH);
    }

    if (runData.control == 1 && !S.empty()) {
      if (improv) {
        R = 1;
        improv = 0;
      } else {
        R = (std::abs(sLineBest.ofv) > math_eps) ? (sBest.ofv - sLineBest.ofv) / (std::abs(sLineBest.ofv) + math_eps) : 0;
      }

      UpdateQValue(S, st, at, R, lf, df);
    }

    end_timeMH = get_time_in_seconds();
    currentTime = (float)(end_timeMH - start_timeMH);
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] GRASP finalizado. Iteracoes=" << iter
              << ", Melhor OFV=" << sBest.ofv << std::endl;
  }
}

} // namespace rkolib::mh