#include "rkolib/mh/ils.hpp"

// Dependências internas
#include "rkolib/core/method.hpp"
#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/solver.hpp" // Para solver.getProblemDimension()

namespace rkolib::mh {

void ILS(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver) {
  using namespace rkolib::core;

  const char *method = "ILS";
  int Iter = 0; // count the number of iterations of the ILS
  int IterImprov = 0; // store the last iteration that improve the current solution

  TSol sBest,    // local optimal solution (current)
      sLine,     // neighborhood solution
      sBestLine; // local optimal of the neighborhood solution

  double betaMin = 0.0; // minimum perturbation
  double betaMax = 0.0; // maximum perturbation

  float currentTime = 0; // computational time of the search process
  int improv = 0;        // improvement flag

  double start_timeMH = get_time_in_seconds(); // start computational time
  double end_timeMH = get_time_in_seconds();   // end computational time

  std::vector<int> RKorder(
      solver.getProblemDimension()); // define a order for the neighors
  std::iota(RKorder.begin(), RKorder.end(), 0);

  // ---------------------------------------------------------------------
  // Q-Learning parameters
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
  int Ti = 1;              // number of epochs performed
  int restartEpsilon = 1;  // number of restart epsilon
  int st = 0;              // current state
  int at = 0;              // current action

  // ** read file with parameter values
  numPar = 2;
  std::vector<std::vector<double>> parameters;
  parameters.resize(numPar);

  readParametersYaml(method, runData.control, parameters, numPar);

  // offline control
  // offline control
  if (runData.control == 0) {
    if (!parameters[0].empty()) {
      betaMin = parameters[0][0];
    }
    if (!parameters[1].empty()) {
      betaMax = parameters[1][0];
    }
  } else if (runData.control == 1) {
    // Q-Learning
    CreateStates(parameters, numStates, numPar, S);
    epsilon = 0.95;
    delta = 0.0;

    iCurr = irandomico(0, numStates - 1);
    st = iCurr;

    if (!S.empty()) {
      betaMin = S[iCurr].par[0];
      betaMax = S[iCurr].par[1];
    }
  }

  Iter = 0;

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] ILS iniciado. betaMin=" << betaMin << ", betaMax=" << betaMax << std::endl;
  }

  // create initial solution
  CreateInitialSolutions(sBest, solver.getProblemDimension());
  solver.decodeSolution(sBest);

  // apply local search
  RVND(sBest, solver, runData.strategy, RKorder);
  UpdatePoolSolutions(sBest, method, runData.debug, runData.poolUpdateMethod);

  // terminate the search process in MAXTIME
  end_timeMH = get_time_in_seconds();
  currentTime = (float)(end_timeMH - start_timeMH);

  // run the search process until stop criterion
  while (currentTime < runData.MAXTIME * runData.restart) {
    if (SOLVER_SHOULD_STOP) {
      if (runData.debug > 0) std::cout << "[DEBUG][MH] ILS interrompido via SOLVER_SHOULD_STOP." << std::endl;
      return;
    }

    Iter++;

    // Q-Learning Update Phase (Pre-Action)
    if (runData.control == 1 && !S.empty()) {
      SetQLParameter(epsilon, lf, df, (int)(runData.MAXTIME * runData.restart),
                     currentTime, delta);
      delta = 0.0;

      at = ChooseAction(S, st, epsilon);
      if (at >= 0 && at < (int)S[st].Ai.size()) {
        iCurr = S[st].Ai[at];
        betaMin = S[iCurr].par[0];
        betaMax = S[iCurr].par[1];
      }
    }

    // new iteration of the ILS
    sLine = sBest;

    // Shake the current solution
    ShakeSolution(sLine, betaMin, betaMax, solver.getProblemDimension());
    solver.decodeSolution(sLine);

    // s*' <- local search (s')
    sBestLine = sLine;
    RVND(sBestLine, solver, runData.strategy, RKorder);

    // s* <- acceptance criterion (s*, s*', historico)
    if (sBestLine.ofv < sBest.ofv) {
      delta = (sBest.ofv - sBestLine.ofv) / (sBest.ofv + math_eps);
      sBest = sBestLine;
      IterImprov = Iter;
      improv = 1;

      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] ILS encontrou nova melhor solucao na iteracao " 
                  << Iter << ": ofv=" << sBest.ofv << std::endl;
      }

      UpdatePoolSolutions(sBest, method, runData.debug, runData.poolUpdateMethod);
    }

    // Q-Learning Reward Phase (Post-Action)
    if (runData.control == 1 && !S.empty()) {
      if (improv) {
        R = 1;
        improv = 0;
      } else {
        R = (std::abs(sBestLine.ofv) > math_eps) ? ((sBest.ofv - sBestLine.ofv) / (std::abs(sBestLine.ofv) + math_eps)) : 0.0;
      }

      UpdateQValue(S, st, at, R, lf, df);
    }

    end_timeMH = get_time_in_seconds();
    currentTime = (float)(end_timeMH - start_timeMH);
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] ILS finalizado. Iteracoes=" << Iter
              << ", Melhor OFV=" << sBest.ofv << std::endl;
  }
}

} // namespace rkolib::mh