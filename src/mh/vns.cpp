#include "rkolib/mh/vns.hpp"

#include "rkolib/core/method.hpp"
#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

static void AcceptNeighborhoodSolution(core::TSol &s, core::TSol &sBest, const core::TSol &sBestLine,
                                        int &k, int Iter, int &IterMelhora, int &improv,
                                        const char *method, const rkolib::core::TRunData &runData) {
  if (sBestLine.ofv >= s.ofv) {
    k++;
    return;
  }

  s = sBestLine;
  if (s.ofv < sBest.ofv) {
    sBest = s;
    k = 1;
    IterMelhora = Iter;
    improv = 1;

    if (runData.debug > 0) {
      std::cout << "[DEBUG][MH] VNS encontrou nova melhor solucao na iteracao " 
                << Iter << ": ofv=" << sBest.ofv << std::endl;
    }

    UpdatePoolSolutions(sBestLine, method, (int)runData.debug, runData.poolUpdateMethod);
  }
}

void VNS(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver) {
  using namespace rkolib::core; // Facilita acesso a TSol, TState, etc.

  const char *method = "VNS";
  double beta = 0;      // perturbation rate
  int Iter = 0;         // current iteration
  int IterMelhora = 0;  // iteration that found the best solution
  int kMax = 0;         // number of neighborhood strutures
  double betaMin = 0.0; // intensity of perturbation
  int improv = 0;       // improvement flag

  TSol s,        // current solution
      sBest,     // best solution of VNS
      sLine,     // neighborhood solution
      sBestLine; // best neighborhood solution

  float currentTime = 0; // computational time of the search process

  double start_timeMH = get_time_in_seconds(); // start computational time
  double end_timeMH = get_time_in_seconds();   // end computational time

  std::vector<int> RKorder(
      solver.getProblemDimension()); // define a order for the neighors
  std::iota(RKorder.begin(), RKorder.end(), 0);

  // ---------------------------------------------------------------------
  // Q-Learning parameters initialization
  // ---------------------------------------------------------------------
  std::vector<TState> S; // finite state space
  int numPar = 0;        // number of parameters
  int numStates = 0;     // number of states
  int iCurr = 0;         // current (initial) state
  double epsilon = 0;    // greed choice possibility
  double lf = 0;         // learning factor
  double df = 0;         // discount factor
  double R = 0;          // reward

  const double math_eps = 1e-9; // security margin for floating point denominator
  double delta = 0.0; // variance used to update the parameters of the Q-Learning method
  float epsilon_max = 1.0; // maximum epsilon
  float epsilon_min = 0.1; // minimum epsilon
  int restartEpsilon = 1;  // number of restart epsilon
  int st = 0;              // current state
  int at = 0;              // current action

  // ** read file with parameter values
  numPar = 2;
  std::vector<std::vector<double>> parameters;
  parameters.resize(numPar);

  // read parameters from txt or yaml
  readParametersYaml(method, runData.control, parameters, numPar);

  // offline control
  if (runData.control == 0) {
    if (!parameters[0].empty()) {
      kMax = (int)parameters[0][0];
    }
    if (!parameters[1].empty()) {
      betaMin = parameters[1][0];
    }
  } else if (runData.control == 1) {
    // Q-Learning: create possible states of the Markov chain (Defined in qlearning.hpp)
    CreateStates(parameters, numStates, numPar, S);

    restartEpsilon = 1;
    epsilon_max = 1.0;
    epsilon = 0.95;
    delta = 0.0;

    iCurr = irandomico(0, numStates - 1);
    st = iCurr;

    if (!S.empty()) {
      kMax = (int)S[iCurr].par[0];
      betaMin = S[iCurr].par[1];
    }
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] VNS iniciado. kMax=" << kMax << ", betaMin=" << betaMin << std::endl;
  }

  // ---------------------------------------------------------------------
  // Initialization
  // ---------------------------------------------------------------------
  sBest.ofv = INFINITY;

  CreateInitialSolutions(s, solver.getProblemDimension());
  solver.decodeSolution(s);
  sBest = s;

  // ---------------------------------------------------------------------
  // Main Loop
  // ---------------------------------------------------------------------
  while (currentTime < runData.MAXTIME * runData.restart) {
    double bestOfvStartGen = sBest.ofv;

    // Q-Learning Update Phase (Pre-Action)
    if (runData.control == 1 && !S.empty()) {
      SetQLParameter(epsilon, lf, df, (int)(runData.MAXTIME * runData.restart),
                     currentTime, delta);
      delta = 0.0;

      at = ChooseAction(S, st, epsilon);
      if (at >= 0 && at < (int)S[st].Ai.size()) {
        iCurr = S[st].Ai[at];
        kMax = (int)S[iCurr].par[0];
        betaMin = S[iCurr].par[1];
      }
    }

    // VNS Loop: current neighborhood
    int k = 1;
    while (k <= kMax && currentTime < runData.MAXTIME * runData.restart) {
      if (SOLVER_SHOULD_STOP) {
        return;
      }

      Iter++;

      // s' <- perturb the best solution in the neighborhood k
      beta = randomico(k * betaMin, (k + 1) * betaMin);
      sLine = s;
      ShakeSolution(sLine, beta, beta, solver.getProblemDimension());
      solver.decodeSolution(sLine);

      // s*' <- local search (s')
      sBestLine = sLine;
      RVND(sBestLine, solver, (int)runData.strategy, RKorder);

      // s <- acceptance criterion (s,s*', historico)
      AcceptNeighborhoodSolution(s, sBest, sBestLine, k, Iter, IterMelhora, improv, method, runData);

      end_timeMH = get_time_in_seconds();
      currentTime = (float)(end_timeMH - start_timeMH);
    }

    // Q-Learning Reward Phase (Post-Action)
    if (runData.control == 1 && !S.empty()) {
      if (improv) {
        R = 1;
        improv = 0;
        delta = (bestOfvStartGen - sBest.ofv) / (bestOfvStartGen + math_eps);
      } else {
        R = (std::abs(s.ofv) > math_eps) ? ((sBest.ofv - s.ofv) / (std::abs(s.ofv) + math_eps)) : 0.0;
      }

      UpdateQValue(S, st, at, R, lf, df);
    }
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] VNS finalizado. Iteracoes=" << Iter
              << ", Melhor OFV=" << sBest.ofv << std::endl;
  }
}

} // namespace rkolib::mh