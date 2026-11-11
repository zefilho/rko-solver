#include "rkolib/mh/brkga.hpp"

#include "rkolib/core/method.hpp"
#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/solver.hpp"
#include <cstdlib>

namespace rkolib::mh {

using namespace rkolib::core;

// =========================================================================
// HELPER FUNCTIONS (INTERNAL)
// =========================================================================

// -------------------------------------------------------------------------
// Method: PUX (Parametric Uniform Crossover)
// -------------------------------------------------------------------------
static TSol PUX(int eliteSize, int popSize, double rhoe,
                const std::vector<TSol> &Pop, const int n) {
  int eliteParent = irandomico(0, eliteSize - 1); 
  int nonEliteParent = irandomico(eliteSize, popSize - 1);

  TSol s = Pop[eliteParent]; 

  for (int j = 0; j < n; j++) {
    if (randomico(0, 1) >= rhoe) {
      s.rk[j] = Pop[nonEliteParent].rk[j];
    }
  }

  return s;
}

// -------------------------------------------------------------------------
// Method: UpdatePopSize
// -------------------------------------------------------------------------
static void UpdatePopSize(int p, double pe, double pm, double rhoe,
                          std::vector<TSol> &Pop, std::vector<TSol> &PopInter,
                          RkoSolver &solver) {
  int oldPsize = (int)Pop.size();
  if (oldPsize == p) {
    return;
  }

  if (oldPsize > p) {
    PopInter = Pop;
    Pop.resize(p);

    int eliteLimit = (int)(p * pe);
    for (int i = 0; i < eliteLimit; i++) {
      Pop[i] = PopInter[i];
    }

    int pos = (int)(pe * oldPsize);
    for (int i = eliteLimit; i < p; i++) {
      if (pos < oldPsize) {
        Pop[i] = PopInter[pos++];
      } else {
        CreateInitialSolutions(Pop[i], solver.getProblemDimension());
        solver.decodeSolution(Pop[i]);
      }
    }
    PopInter.resize(p);
    return;
  }

  Pop.resize(p);
  PopInter.resize(p);

  for (int k = oldPsize; k < p; k++) {
    if (SOLVER_SHOULD_STOP) {
      return;
    }

    Pop[k] = PUX((int)(oldPsize * pe), oldPsize, rhoe, Pop,
                 solver.getProblemDimension());
    solver.decodeSolution(Pop[k]);
  }

  std::sort(Pop.begin(), Pop.end(), sortByFitness);
}

// -------------------------------------------------------------------------
// Helper: GenerateOffspring
// -------------------------------------------------------------------------
static bool GenerateOffspring(int eliteCount, int crossoverLimit, int p, double rhoe,
                              std::vector<TSol> &PopInter, const std::vector<TSol> &Pop,
                              TSol &bestOff, RkoSolver &solver, const TRunData &runData) {
  for (int i = eliteCount; i < crossoverLimit; i++) {
    if (SOLVER_SHOULD_STOP) {
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] BRKGA interrompido via SOLVER_SHOULD_STOP." << std::endl;
      }
      return false;
    }

    PopInter[i] = PUX(eliteCount, p, rhoe, Pop, solver.getProblemDimension());
    solver.decodeSolution(PopInter[i]);

    if (PopInter[i].ofv < bestOff.ofv) {
      bestOff = PopInter[i];
    }
  }
  return true;
}

// -------------------------------------------------------------------------
// Helper: GenerateMutants
// -------------------------------------------------------------------------
static bool GenerateMutants(int crossoverLimit, int p, std::vector<TSol> &PopInter,
                            RkoSolver &solver, const TRunData &runData) {
  for (int i = crossoverLimit; i < p; i++) {
    if (SOLVER_SHOULD_STOP) {
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] BRKGA interrompido via SOLVER_SHOULD_STOP." << std::endl;
      }
      return false;
    }

    CreateInitialSolutions(PopInter[i], solver.getProblemDimension());
    solver.decodeSolution(PopInter[i]);
  }
  return true;
}

// =========================================================================
// MAIN ALGORITHM IMPLEMENTATION
// =========================================================================

void BRKGA(const TRunData &runData, RkoSolver &solver) {
  const char *method = "BRKGA";
  int p = 0;       // size of population
  double pe = 0.0; // fraction of population to be the elite-set
  double pm = 0.0; // fraction of population to be replaced by mutants
  double rhoe = 0.0; // probability that offspring inherit an allele from elite parent

  std::vector<TSol> Pop;      // current population
  std::vector<TSol> PopInter; // intermediary population
  TSol bestInd;               // best individual found in past generation

  int numGenerations = 0; // number of generations
  int bestGeneration = 0; // generation in which found the best solution
  float currentTime = 0;  // computational time of the search process
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
    if (!parameters[0].empty()) { p = (int)parameters[0][0]; }
    if (!parameters[1].empty()) { pe = parameters[1][0]; }
    if (!parameters[2].empty()) { pm = parameters[2][0]; }
    if (!parameters[3].empty()) { rhoe = parameters[3][0]; }
  } else if (runData.control == 1) {
    CreateStates(parameters, numStates, numPar, S);
    epsilon = 0.95;
    delta = 0.0;

    iCurr = irandomico(0, numStates - 1);
    st = iCurr;

    if (!S.empty()) {
      p = (int)S[iCurr].par[0];
      pe = S[iCurr].par[1];
      pm = S[iCurr].par[2];
      rhoe = S[iCurr].par[3];
    }
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] BRKGA iniciado. PopSize=" << p << ", pe=" << pe 
              << ", pm=" << pm << ", rhoe=" << rhoe << std::endl;
  }

  Pop.clear();
  PopInter.clear();
  Pop.resize(p);
  PopInter.resize(p);

  for (int i = 0; i < p; i++) {
    CreateInitialSolutions(Pop[i], solver.getProblemDimension());
    solver.decodeSolution(Pop[i]);
    PopInter[i] = Pop[i];
  }

  std::sort(Pop.begin(), Pop.end(), sortByFitness);
  bestInd = Pop[0];

  // run the evolutionary process until stop criterion
  while (currentTime < runData.MAXTIME * runData.restart) {
    numGenerations++;

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
        p = (int)S[iCurr].par[0];
        pe = S[iCurr].par[1];
        pm = S[iCurr].par[2];
        rhoe = S[iCurr].par[3];

        UpdatePopSize(p, pe, pm, rhoe, Pop, PopInter, solver);
      }
    }

    if ((int)PopInter.size() != p) {
      PopInter.resize(p);
    }

    // 1. ELITE
    int eliteCount = (int)(p * pe);
    for (int i = 0; i < eliteCount; i++) {
      PopInter[i] = Pop[i];
    }

    // 2. CROSSOVER
    int mutantCount = (int)(p * pm);
    int crossoverLimit = p - mutantCount;

    TSol bestOff;
    bestOff.ofv = std::numeric_limits<double>::infinity();

    if (!GenerateOffspring(eliteCount, crossoverLimit, p, rhoe, PopInter, Pop,
                          bestOff, solver, runData)) {
      return;
    }

    // 3. MUTANTS
    if (!GenerateMutants(crossoverLimit, p, PopInter, solver, runData)) {
      return;
    }

    Pop = PopInter;

    if (eliteCount > 0) {
      int pos = irandomico(0, eliteCount - 1);
      NelderMeadSearch(Pop[pos], solver);
    }

    std::sort(Pop.begin(), Pop.end(), sortByFitness);

    if (Pop[0].ofv < bestInd.ofv) {
      bestGeneration = numGenerations;
      improv = 1;
      delta = (bestInd.ofv - Pop[0].ofv) / (bestInd.ofv + math_eps);
      bestInd = Pop[0];

      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] BRKGA encontrou novo melhor individuo na geracao "
                  << numGenerations << ": ofv=" << bestInd.ofv << std::endl;
      }

      UpdatePoolSolutions(Pop[0], method, runData.debug, runData.poolUpdateMethod);
    }

    // -----------------------------------------------------------------
    // Q-Learning Reward Phase (Post-Action)
    // -----------------------------------------------------------------
    if (runData.control == 1 && !S.empty()) {
      if (improv) {
        R = 1.0 + 1.0 / (double)p;
        improv = 0;
      } else {
        R = (std::abs(bestInd.ofv) > 1e-9) ? (bestInd.ofv - bestOff.ofv) / (std::abs(bestInd.ofv) + 1e-9) : 0;
      }

      UpdateQValue(S, st, at, R, lf, df);
    }

    end_timeMH = get_time_in_seconds();
    currentTime = (float)(end_timeMH - start_timeMH);
  }

  Pop.clear();
  PopInter.clear();

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] BRKGA finalizado. Geraçoes=" << numGenerations
              << ", Melhor OFV=" << bestInd.ofv << std::endl;
  }
}

} // namespace rkolib::mh