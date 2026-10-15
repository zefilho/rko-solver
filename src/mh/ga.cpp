#include "rkolib/mh/ga.hpp"

#include "rkolib/core/method.hpp"
#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

using namespace rkolib::core;

// -------------------------------------------------------------------------
// Helper Function: UpdatePopulation (Internal)
// -------------------------------------------------------------------------
static void UpdatePopulation(int sizePop, std::vector<TSol> &Pop,
                             RkoSolver &solver) {
  int oldPsize = (int)Pop.size();

  if (oldPsize > sizePop) {
    Pop.resize(sizePop);
  } else if (oldPsize < sizePop) {
    Pop.resize(sizePop);
    for (int k = oldPsize; k < sizePop; k++) {
      CreateInitialSolutions(Pop[k], solver.getProblemDimension());
      solver.decodeSolution(Pop[k]);
    }
  }
}

// -------------------------------------------------------------------------
// Helper Function: TournamentSelection
// -------------------------------------------------------------------------
static void TournamentSelection(const std::vector<TSol> &pop,
                                std::vector<TSol> &popInter, int sizePop) {
  for (int j = 0; j < sizePop; j++) {
    int p1, p2, p3;
    do {
      p1 = irandomico(0, sizePop - 1);
      p2 = irandomico(0, sizePop - 1);
      p3 = irandomico(0, sizePop - 1);
    } while (p1 == p2 || p1 == p3 || p2 == p3);

    int melhor = p1;
    if (pop[p2].ofv < pop[melhor].ofv) {
      melhor = p2;
    }
    if (pop[p3].ofv < pop[melhor].ofv) {
      melhor = p3;
    }
    popInter[j] = pop[melhor];
  }
}

// -------------------------------------------------------------------------
// Helper Function: CrossAndMutatePair
// -------------------------------------------------------------------------
static void CrossAndMutatePair(TSol &child1, TSol &child2,
                               const TSol &parent1, const TSol &parent2,
                               float probCros, float probMut,
                               RkoSolver &solver) {
  child1 = parent1;
  child2 = parent2;

  if (randomico(0, 1) >= probCros) {
    return;
  }

  int dim = solver.getProblemDimension();
  for (int j = 0; j < dim; j++) {
    if (randomico(0, 1) < 0.5) {
      child1.rk[j] = parent2.rk[j];
    }
    if (randomico(0, 1) < 0.5) {
      child2.rk[j] = parent1.rk[j];
    }
    if (randomico(0, 1) <= probMut) {
      child1.rk[j] = randomico(0, 1);
    }
    if (randomico(0, 1) <= probMut) {
      child2.rk[j] = randomico(0, 1);
    }
  }

  solver.decodeSolution(child1);
  solver.decodeSolution(child2);
}

// -------------------------------------------------------------------------
// Helper Function: EvaluateOffspring
// -------------------------------------------------------------------------
static void EvaluateOffspring(TSol &child, TSol &bestInd, double &bestOFcurrent,
                              int &bestGeneration, int numGenerations,
                              int &improv, const char *method,
                              const TRunData &runData) {
  if (child.ofv < bestInd.ofv) {
    bestInd = child;
    bestGeneration = numGenerations;
    improv = 1;
    if (runData.debug > 0) {
      std::cout << "[DEBUG][MH] GA encontrou novo melhor individuo na geracao " 
                << numGenerations << ": ofv=" << bestInd.ofv << std::endl;
    }
    UpdatePoolSolutions(bestInd, method, runData.debug, runData.poolUpdateMethod);
  }

  if (child.ofv < bestOFcurrent) {
    bestOFcurrent = child.ofv;
  }
}
// -------------------------------------------------------------------------
// Helper Function: ReproduceGeneration
// -------------------------------------------------------------------------
static bool ReproduceGeneration(int sizePop, std::vector<TSol> &PopNew,
                                const std::vector<TSol> &PopInter, TSol &bestInd,
                                double &bestOFcurrent, int &bestGeneration,
                                int numGenerations, int &improv, float probCros,
                                float probMut, RkoSolver &solver,
                                const TRunData &runData, const char *method) {
  for (int i = 0; i < sizePop - 1; i += 2) {
    if (SOLVER_SHOULD_STOP) {
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] GA interrompido via SOLVER_SHOULD_STOP." << std::endl;
      }
      return false;
    }

    CrossAndMutatePair(PopNew[i], PopNew[i + 1], PopInter[i], PopInter[i + 1],
                       probCros, probMut, solver);
    EvaluateOffspring(PopNew[i], bestInd, bestOFcurrent, bestGeneration,
                      numGenerations, improv, method, runData);
    EvaluateOffspring(PopNew[i + 1], bestInd, bestOFcurrent, bestGeneration,
                      numGenerations, improv, method, runData);
  }
  return true;
}

// -------------------------------------------------------------------------
// Main Algorithm: GA
// -------------------------------------------------------------------------
void GA(const TRunData &runData, RkoSolver &solver) {
  const char *method = "GA";
  int sizePop = 0;      // population size
  float probCros = 0.0; // crossover probability
  float probMut = 0.0;  // mutation probability

  std::vector<TSol> Pop;      // current population
  std::vector<TSol> PopInter; // selected individuals to evolutionary process
  std::vector<TSol> PopNew;   // offsprings

  TSol bestInd; // best solution found in SGA

  int numGenerations = 0; // number of generations
  float currentTime = 0;  // computational time of the search process
  int bestGeneration = 0; // generation in which found the best solution
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

  numPar = 3;
  std::vector<std::vector<double>> parameters;
  parameters.resize(numPar);

  readParametersYaml(method, runData.control, parameters, numPar);

  // offline control
  if (runData.control == 0) {
    if (!parameters[0].empty()) { sizePop = (int)parameters[0][0]; }
    if (!parameters[1].empty()) { probCros = (float)parameters[1][0]; }
    if (!parameters[2].empty()) { probMut = (float)parameters[2][0]; }
  } else if (runData.control == 1) {
    CreateStates(parameters, numStates, numPar, S);
    epsilon = 0.95;
    delta = 0.0;

    iCurr = irandomico(0, numStates - 1);
    st = iCurr;

    if (!S.empty()) {
      sizePop = (int)S[iCurr].par[0];
      probCros = (float)S[iCurr].par[1];
      probMut = (float)S[iCurr].par[2];
    }
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] GA iniciado. sizePop=" << sizePop << ", probCros=" << probCros 
              << ", probMut=" << probMut << std::endl;
  }

  // initialize population
  Pop.resize(sizePop);
  PopNew.resize(sizePop);
  PopInter.resize(sizePop);

  bestInd.ofv = std::numeric_limits<double>::infinity();

  for (int i = 0; i < sizePop; i++) {
    CreateInitialSolutions(Pop[i], solver.getProblemDimension());
    solver.decodeSolution(Pop[i]);

    if (Pop[i].ofv < bestInd.ofv) {
      bestInd = Pop[i];
    }
  }

  // run the evolutionary process until stop criterion
  while (currentTime < runData.MAXTIME * runData.restart) {
    numGenerations++;
    double bestOfvStartGen = bestInd.ofv;

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
        sizePop = (int)S[iCurr].par[0];
        probCros = (float)S[iCurr].par[1];
        probMut = (float)S[iCurr].par[2];

        UpdatePopulation(sizePop, Pop, solver);
      }
    }

    if ((int)PopNew.size() != sizePop) {
      PopNew.resize(sizePop);
    }
    if ((int)PopInter.size() != sizePop) {
      PopInter.resize(sizePop);
    }

    // -----------------------------------------------------------------
    // Selection (Tournament)
    // -----------------------------------------------------------------
    TournamentSelection(Pop, PopInter, sizePop);

    // -----------------------------------------------------------------
    // Crossover & Mutation
    // -----------------------------------------------------------------
    double bestOFcurrent = INFINITY;

    if (!ReproduceGeneration(sizePop, PopNew, PopInter, bestInd, bestOFcurrent,
                            bestGeneration, numGenerations, improv, probCros,
                            probMut, solver, runData, method)) {
      return;
    }

    // -----------------------------------------------------------------
    // Local Search
    // -----------------------------------------------------------------
    int pos1 = irandomico(0, sizePop - 1);
    NelderMeadSearch(PopNew[pos1], solver);

    if (PopNew[pos1].ofv < bestInd.ofv) {
      bestInd = PopNew[pos1];
      bestGeneration = numGenerations;
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] GA (LS) encontrou novo melhor individuo: ofv=" << bestInd.ofv << std::endl;
      }
      UpdatePoolSolutions(bestInd, method, runData.debug, runData.poolUpdateMethod);
    }

    // replace the population with offspring
    Pop = PopNew;

    // -----------------------------------------------------------------
    // Q-Learning Reward Phase (Post-Action)
    // -----------------------------------------------------------------
    if (runData.control == 1 && !S.empty()) {
      if (improv) {
        R = 1;
        improv = 0;
        delta = (bestOfvStartGen - bestInd.ofv) / (bestOfvStartGen + math_eps);
      } else {
        R = (std::abs(bestOFcurrent) > math_eps) ? (bestInd.ofv - bestOFcurrent) / (std::abs(bestOFcurrent) + math_eps) : 0;
      }

      UpdateQValue(S, st, at, R, lf, df);
    }

    end_timeMH = get_time_in_seconds();
    currentTime = (float)(end_timeMH - start_timeMH);
  }

  Pop.clear();
  PopNew.clear();
  PopInter.clear();

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] GA finalizado. Geraçoes=" << numGenerations
              << ", Melhor OFV=" << bestInd.ofv << std::endl;
  }
}

} // namespace rkolib::mh