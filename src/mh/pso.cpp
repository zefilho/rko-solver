#include "rkolib/mh/pso.hpp"

#include "rkolib/core/method.hpp"
#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

using namespace rkolib::core;

// -------------------------------------------------------------------------
// Helper Function: UpdateParticleVelocityAndPosition
// -------------------------------------------------------------------------
static void UpdateParticleVelocityAndPosition(TSol &x, const TSol &pBest,
                                              const TSol &gBest,
                                              std::vector<float> &v, float c1,
                                              float c2, float w, int dim) {
  for (int j = 0; j < dim; j++) {
    float r1 = (float)randomico(0, 1);
    float r2 = (float)randomico(0, 1);

    v[j] = w * (v[j] +
                (c1 * r1 * ((float)pBest.rk[j] - (float)x.rk[j])) +
                (c2 * r2 * ((float)gBest.rk[j] - (float)x.rk[j])));

    double oldrk = x.rk[j];
    x.rk[j] = x.rk[j] + v[j];

    if (x.rk[j] < 0.0 || x.rk[j] >= 1.0) {
      x.rk[j] = oldrk;
      v[j] = 0;
    }
  }
}

// -------------------------------------------------------------------------
// Helper Function: UpdateParticleSize
// Description: Internal logic to resize population based on Q-Learning
// -------------------------------------------------------------------------
static void UpdateParticleSize(std::vector<TSol> &X, std::vector<TSol> &Pbest,
                               std::vector<std::vector<float>> &V,
                               const TSol &Gbest, int Psize, float c1, float c2,
                               float w, RkoSolver &solver) {
  int oldPsize = (int)X.size();
  if (oldPsize == Psize) {
    return;
  }

  if (oldPsize > Psize) {
    X.resize(Psize);
    Pbest.resize(Psize);
    V.resize(Psize);
    return;
  }

  X.resize(Psize);
  Pbest.resize(Psize);
  V.resize(Psize, std::vector<float>(solver.getProblemDimension()));

  int dim = solver.getProblemDimension();
  for (int i = oldPsize; i < Psize; i++) {
    CreateInitialSolutions(X[i], dim);
    Pbest[i] = X[i];

    for (int j = 0; j < dim; j++) {
      V[i][j] = (float)randomico(0, 1);
    }

    UpdateParticleVelocityAndPosition(X[i], Pbest[i], Gbest, V[i], c1, c2, w, dim);
    solver.decodeSolution(X[i]);
    Pbest[i] = X[i];
  }
}

// -------------------------------------------------------------------------
// Helper Function: ApplyPsoLocalSearch
// -------------------------------------------------------------------------
static void ApplyPsoLocalSearch(std::vector<TSol> &Pbest, TSol &Gbest,
                                int &bestGeneration, int numGenerations,
                                int &improv, RkoSolver &solver,
                                const TRunData &runData) {
  int currentPsize = (int)Pbest.size();
  if (currentPsize <= 0) {
    return;
  }

  int chosen = irandomico(0, currentPsize - 1);
  NelderMeadSearch(Pbest[chosen], solver);

  if (Pbest[chosen].ofv < Gbest.ofv) {
    Gbest = Pbest[chosen];
    bestGeneration = numGenerations;
    improv = 1;

    if (runData.debug > 0) {
      std::cout << "[DEBUG][MH] PSO (LS) encontrou novo Gbest: ofv="
                << Gbest.ofv << std::endl;
    }
  }
}

// -------------------------------------------------------------------------
// Helper Function: EvolveParticles
// -------------------------------------------------------------------------
static bool EvolveParticles(std::vector<TSol> &X, std::vector<TSol> &Pbest,
                            std::vector<std::vector<float>> &V, TSol &Gbest,
                            double &bestOFcurrent, int &bestGeneration,
                            int numGenerations, float c1, float c2, float w,
                            RkoSolver &solver, const TRunData &runData) {
  int currentPsize = (int)X.size();
  bool improved = false;
  int dim = solver.getProblemDimension();

  for (int i = 0; i < currentPsize; i++) {
    if (SOLVER_SHOULD_STOP) {
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] PSO interrompido via SOLVER_SHOULD_STOP." << std::endl;
      }
      return improved;
    }

    UpdateParticleVelocityAndPosition(X[i], Pbest[i], Gbest, V[i], c1, c2, w, dim);
    solver.decodeSolution(X[i]);

    if (X[i].ofv < bestOFcurrent) {
      bestOFcurrent = X[i].ofv;
    }

    if (X[i].ofv < Pbest[i].ofv) {
      Pbest[i] = X[i];
    }

    if (X[i].ofv < Gbest.ofv) {
      Gbest = X[i];
      bestGeneration = numGenerations;
      improved = true;

      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] PSO encontrou novo Gbest na geracao " 
                  << numGenerations << ": ofv=" << Gbest.ofv << std::endl;
      }
    }
  }
  return improved;
}

// -------------------------------------------------------------------------
// Main Algorithm: PSO
// -------------------------------------------------------------------------
void PSO(const TRunData &runData, RkoSolver &solver) {
  const char *method = "PSO";
  int Psize = 0; // number of particles
  float c1 = 0.0;
  float c2 = 0.0;
  float w = 0.0;

  std::vector<TSol> X;               // current solutions
  std::vector<TSol> Pbest;           // best solutions
  std::vector<std::vector<float>> V; // particle velocity

  TSol Gbest;               // global best solution
  double bestOFcurrent = 0; // best ofv found in the current generation

  int numGenerations = 0; // number of generations
  float currentTime = 0;  // computational time of the search process
  int bestGeneration = 0; // number of generation that found the best solution
  int improv = 0;         // improvement flag

  double start_timeMH = get_time_in_seconds(); // start computational time
  double end_timeMH = get_time_in_seconds();   // end computational time

  std::vector<int> RKorder(solver.getProblemDimension());
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

  const double math_eps = 1e-9;
  double delta = 0.0;
  int st = 0;
  int at = 0;

  numPar = 4;
  std::vector<std::vector<double>> parameters;
  parameters.resize(numPar);

  readParametersYaml(method, runData.control, parameters, numPar);

  // offline control
  if (runData.control == 0) {
    if (!parameters[0].empty()) { Psize = (int)parameters[0][0]; }
    if (!parameters[1].empty()) { c1 = (float)parameters[1][0]; }
    if (!parameters[2].empty()) { c2 = (float)parameters[2][0]; }
    if (!parameters[3].empty()) { w = (float)parameters[3][0]; }
  } else if (runData.control == 1) {
    CreateStates(parameters, numStates, numPar, S);
    epsilon = 0.95;
    delta = 0.0;

    iCurr = irandomico(0, numStates - 1);
    st = iCurr;

    if (!S.empty()) {
      Psize = (int)S[iCurr].par[0];
      c1 = (float)S[iCurr].par[1];
      c2 = (float)S[iCurr].par[2];
      w = (float)S[iCurr].par[3];
    }
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] PSO iniciado. Psize=" << Psize << ", c1=" << c1 
              << ", c2=" << c2 << ", w=" << w << std::endl;
  }

  // initialize population
  X.clear();
  Pbest.clear();
  V.clear();

  X.resize(Psize);
  Pbest.resize(Psize);
  V.resize(Psize, std::vector<float>(solver.getProblemDimension()));

  Gbest.ofv = INFINITY;

  for (int i = 0; i < Psize; i++) {
    CreateInitialSolutions(X[i], solver.getProblemDimension());
    solver.decodeSolution(X[i]);

    for (int j = 0; j < solver.getProblemDimension(); j++) {
      V[i][j] = (float)randomico(0, 1);
    }

    if (X[i].ofv < Gbest.ofv) {
      Gbest = X[i];
    }
    Pbest[i] = X[i];
  }

  // run the evolutionary process until stop criterion
  while (currentTime < runData.MAXTIME * runData.restart) {
    numGenerations++;
    double bestOfvStartGen = Gbest.ofv;

    // -----------------------------------------------------------------
    // Q-Learning Update Phase (Pre-Action)
    // -----------------------------------------------------------------
    if (runData.control == 1 && !S.empty()) {
      SetQLParameter(epsilon, lf, df, (int)(runData.MAXTIME * runData.restart),
                     currentTime, delta);
      delta = 0.0;
      at = ChooseAction(S, st, epsilon);

      if (at >= 0 && at < (int)S[st].Ai.size()) {
        iCurr = S[st].Ai[at];
        Psize = (int)S[iCurr].par[0];
        c1 = (float)S[iCurr].par[1];
        c2 = (float)S[iCurr].par[2];
        w = (float)S[iCurr].par[3];

        UpdateParticleSize(X, Pbest, V, Gbest, Psize, c1, c2, w, solver);
      }
    }

    // -----------------------------------------------------------------
    // PSO Evolution Phase
    // -----------------------------------------------------------------
    bestOFcurrent = INFINITY;

    if (EvolveParticles(X, Pbest, V, Gbest, bestOFcurrent, bestGeneration,
                        numGenerations, c1, c2, w, solver, runData)) {
      improv = 1;
    }

    if (SOLVER_SHOULD_STOP) {
      return;
    }

    double oldGbest = Gbest.ofv;
    ApplyPsoLocalSearch(Pbest, Gbest, bestGeneration, numGenerations, improv, solver, runData);

    if (bestGeneration == numGenerations || Gbest.ofv < oldGbest) {
      UpdatePoolSolutions(Gbest, method, runData.debug, runData.poolUpdateMethod);
    }

    // -----------------------------------------------------------------
    // Q-Learning Reward Phase (Post-Action)
    // -----------------------------------------------------------------
    if (runData.control == 1 && !S.empty()) {
      if (improv) {
        R = 1;
        improv = 0;
        delta = (bestOfvStartGen - Gbest.ofv) / (bestOfvStartGen + math_eps);
      } else {
        R = (std::abs(bestOFcurrent) > 1e-9) ? (Gbest.ofv - bestOFcurrent) / (std::abs(bestOFcurrent) + 1e-9) : 0;
      }

      UpdateQValue(S, st, at, R, lf, df);
    }

    end_timeMH = get_time_in_seconds();
    currentTime = (float)(end_timeMH - start_timeMH);
  }

  X.clear();
  V.clear();
  Pbest.clear();

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] PSO finalizado. Geraçoes=" << numGenerations
              << ", Melhor OFV=" << Gbest.ofv << std::endl;
  }
}

} // namespace rkolib::mh