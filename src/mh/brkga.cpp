#include "rkolib/mh/brkga.hpp"

// Dependências internas
#include "rkolib/core/method.hpp"
#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/solver.hpp" // Para solver.getProblemDimension()
#include <cstdlib>

namespace rkolib::mh {

using namespace rkolib::core;

// =========================================================================
// HELPER FUNCTIONS (INTERNAL)
// =========================================================================

// -------------------------------------------------------------------------
// Method: PUX (Parametric Uniform Crossover)
// Optimization: Arguments passed by const reference.
// -------------------------------------------------------------------------
static TSol PUX(int eliteSize, int popSize, double rhoe,
                const std::vector<TSol> &Pop, const int n) {
                
  int eliteParent = irandomico(0, eliteSize - 1); 
  int nonEliteParent = irandomico(eliteSize, popSize - 1);

  // Cean copy of the elite parent! This ensures that 
  // all other fields of TSol remain consistent.
  TSol s = Pop[eliteParent]; 

  for (int j = 0; j < n; j++) {
    // If not inherit from elite, replace with non-elite
    if (randomico(0, 1) >= rhoe) {
      s.rk[j] = Pop[nonEliteParent].rk[j];
    }
  }

  return s;
}

// -------------------------------------------------------------------------
// Method: UpdatePopSize
// Description: Logic to resize population dynamically
// -------------------------------------------------------------------------
static void UpdatePopSize(int p, double pe, double pm, double rhoe,
                          std::vector<TSol> &Pop, std::vector<TSol> &PopInter,
                          RkoSolver &solver) {
  int oldPsize = (int)Pop.size();

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
        Pop[i] = PopInter[pos];
        pos++;
      } else {
        CreateInitialSolutions(Pop[i], solver.getProblemDimension());
        solver.decodeSolution(Pop[i]);
      }
    }
    
    // Remova PopInter.clear();
    // Apenas redimensione
    PopInter.resize(p); 
  }
  else if (oldPsize < p) {
    Pop.resize(p);
    PopInter.resize(p); // Garanta que PopInter acompanhe o crescimento

    // generate new chromosomes using PUX from existing SOLVER_POOL
    for (int k = oldPsize; k < p; k++) {
      if (SOLVER_SHOULD_STOP)
        return;

      Pop[k] = PUX((int)(oldPsize * pe), oldPsize, rhoe, Pop,
                   solver.getProblemDimension());
      solver.decodeSolution(Pop[k]);
    }

    // sort new population
    std::sort(Pop.begin(), Pop.end(), sortByFitness);

    // clean intermediate population
    //PopInter.clear();
   // PopInter.resize(p);
  }
}

// =========================================================================
// MAIN ALGORITHM IMPLEMENTATION
// =========================================================================

void BRKGA(const TRunData &runData, RkoSolver &solver) {
  const char *method = "BRKGA";
  int p = 0;       // size of population
  double pe = 0.0; // fraction of population to be the elite-set
  double pm = 0.0; // fraction of population to be replaced by mutants
  double rhoe =
      0.0; // probability that offspring inherit an allele from elite parent

  std::vector<TSol> Pop;      // current population
  std::vector<TSol> PopInter; // intermediary population
  TSol bestInd;               // best individual found in past generation

  int numGenerations = 0; // number of generations
  int bestGeneration = 0; // generation in which found the best solution
  float currentTime = 0;  // computational time of the search process
  int improv = 0;         // improvement flag

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
  double delta; // variance used to update the parameters of the Q-Learning method
  float epsilon_max = 1.0; // maximum epsilon
  float epsilon_min = 0.1; // minimum epsilon
  int Ti = 1;              // number of epochs performed
  int restartEpsilon = 1;  // number of restart epsilon
  int st = 0;              // current state
  int at = 0;              // current action

  // ** read file with parameter values
  numPar = 4;
  std::vector<std::vector<double>> parameters;
  parameters.resize(numPar);

  readParametersYaml(method, runData.control, parameters, numPar);

  // offline control
  if (runData.control == 0) {
    if (!parameters[0].empty())
      p = (int)parameters[0][0];
    if (!parameters[1].empty())
      pe = parameters[1][0];
    if (!parameters[2].empty())
      pm = parameters[2][0];
    if (!parameters[3].empty())
      rhoe = parameters[3][0];
  }

  // online control
  else {
    // Q-Learning
    if (runData.control == 1) {
      // create possible states of the Markov chain
      CreateStates(parameters, numStates, numPar, S);

      // number of restart epsilon
      restartEpsilon = 1;

      // maximum epsilon
      epsilon_max = 1.0;

      //variance and epison
      epsilon = 0.95;
      delta = 0.0;

      // current state
      iCurr = irandomico(0, numStates - 1);
      st = iCurr;

      // define the initial parameters of the BRGKA
      if (!S.empty()) {
        p = (int)S[iCurr].par[0];
        pe = S[iCurr].par[1];
        pm = S[iCurr].par[2];
        rhoe = S[iCurr].par[3];
      }
    }
  }
  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] BRKGA iniciado. PopSize=" << p << ", pe=" << pe 
              << ", pm=" << pm << ", rhoe=" << rhoe << std::endl;
  }

  // initialize population
  Pop.clear();
  PopInter.clear();
  Pop.resize(p);
  PopInter.resize(p);

  // Create the initial chromosomes with random keys
  for (int i = 0; i < p; i++) {
    CreateInitialSolutions(Pop[i], solver.getProblemDimension());
    solver.decodeSolution(Pop[i]);
    PopInter[i] = Pop[i];
  }

  // sort population in increase order of fitness
  std::sort(Pop.begin(), Pop.end(), sortByFitness);

  // save the best solution found
  bestInd = Pop[0];

  // run the evolutionary process until stop criterion
  while (currentTime < runData.MAXTIME * runData.restart) {
    // number of generations
    numGenerations++;

    // -----------------------------------------------------------------
    // Q-Learning Update Phase (Pre-Action)
    // -----------------------------------------------------------------
    if (runData.control == 1 && !S.empty()) {
      // set Q-Learning parameters
      if (0) {
      SetQLParameter(currentTime, Ti, restartEpsilon, epsilon_max, epsilon_min,
                     epsilon, lf, df, (int)(runData.MAXTIME * runData.restart));
      } else {
        SetQLParameter(epsilon, lf, df, (int)(runData.MAXTIME * runData.restart),
          currentTime, delta);
      }

      // 
      delta = 0.0;

      // choose a action at for current state st
      at = ChooseAction(S, st, epsilon);

      // execute action at
      if (at >= 0 && at < (int)S[st].Ai.size()) {
        iCurr = S[st].Ai[at];

        // define the parameters of the BRGKA according of the current state
        p = (int)S[iCurr].par[0];
        pe = S[iCurr].par[1];
        pm = S[iCurr].par[2];
        rhoe = S[iCurr].par[3];

        // update population size
        UpdatePopSize(p, pe, pm, rhoe, Pop, PopInter, solver);
      }
    }

    // Ensure PopInter matches current p
    if ((int)PopInter.size() != p)
      PopInter.resize(p);

    // 1. ELITE: The 'Pe' best chromosomes are maintained
    int eliteCount = (int)(p * pe);
    for (int i = 0; i < eliteCount; i++) {
      PopInter[i] = Pop[i];
    }

    // 2. CROSSOVER: We'll mate 'p - pe - pm' pairs
    // Note: Fixed logic. Standard BRKGA fills from eliteCount up to (p -
    // mutantCount)
    int mutantCount = (int)(p * pm);
    int crossoverLimit = p - mutantCount;

    TSol bestOff;
    bestOff.ofv = std::numeric_limits<double>::infinity();

    for (int i = eliteCount; i < crossoverLimit; i++) {
      if (SOLVER_SHOULD_STOP) {
        if (runData.debug > 0) std::cout << "[DEBUG][MH] BRKGA interrompido via SOLVER_SHOULD_STOP." << std::endl;
        return;
      }

      // Parametric uniform crossover
      PopInter[i] = PUX(eliteCount, p, rhoe, Pop, solver.getProblemDimension());

      // Calculate the fitness of new chromosomes
      solver.decodeSolution(PopInter[i]);

      if (PopInter[i].ofv < bestOff.ofv) {
        bestOff = PopInter[i];
      }
    }

    // 3. MUTANTS: We'll introduce 'pm' mutants
    // Fix: Standard BRKGA places mutants at the end.
    // Original code index math was ambiguous/overlapping.
    // Now we iterate from 'crossoverLimit' to 'p'.
    for (int i = crossoverLimit; i < p; i++) {
      if (SOLVER_SHOULD_STOP) {
        if (runData.debug > 0) std::cout << "[DEBUG][MH] BRKGA interrompido via SOLVER_SHOULD_STOP." << std::endl;
        return;
      }

      CreateInitialSolutions(PopInter[i], solver.getProblemDimension());
      solver.decodeSolution(PopInter[i]);
    }

    // Update the current population
    Pop = PopInter;

    // Appy local search in one elite solution (Optional enhancement present in
    // code)
    if (eliteCount > 0) {
      int pos = irandomico(0, eliteCount - 1);
      NelderMeadSearch(Pop[pos], solver);
    }

    // Sort population in increase order of fitness
    std::sort(Pop.begin(), Pop.end(), sortByFitness);

    // We improve the best fitness in the current population
    if (Pop[0].ofv < bestInd.ofv) {
      bestGeneration = numGenerations;
      improv = 1;

      delta = (bestInd.ofv - Pop[0].ofv) / (bestInd.ofv + math_eps);

      bestInd = Pop[0];

      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] BRKGA encontrou novo melhor individuo na geracao "
                  << numGenerations << ": ofv=" << bestInd.ofv << std::endl;
      }

      // Update SOLVER_POOL of solutions
      UpdatePoolSolutions(Pop[0], method, runData.debug, runData.poolUpdateMethod);
    }

    // -----------------------------------------------------------------
    // Q-Learning Reward Phase (Post-Action)
    // -----------------------------------------------------------------
    if (runData.control == 1 && !S.empty()) {
      // We improve the best fitness in the current population
      if (improv) {
        // Fix: Force floating point division. 1/p is 0 in integer arithmetic.
        R = 1.0 + 1.0 / (double)p;
        improv = 0;
      } else {
        if (std::abs(bestInd.ofv) > 1e-9)
          R = (bestInd.ofv - bestOff.ofv) / (std::abs(bestInd.ofv) + 1e-9);
        else
          R = 0;
      }

      // index of the next state
      if (at >= 0 && at < (int)S[st].Ai.size()) {
        int st_1 = S[st].Ai[at];

        // Update the Q-Table value
        if (at < (int)S[st].Qa.size()) {
          S[st].Qa[at] =
              S[st].Qa[at] + lf * (R + df * S[st_1].maxQ - S[st].Qa[at]);

          // Recalculate true maxQ and maxA for state st
          S[st].maxQ = S[st].Qa[0];
          S[st].maxA = 0;
          for (size_t k = 1; k < S[st].Qa.size(); ++k) {
            if (S[st].Qa[k] > S[st].maxQ) {
              S[st].maxQ = S[st].Qa[k];
              S[st].maxA = (int)k;
            }
          }
        }
        // Define the new current state st
        st = st_1;
      }
    }

    // terminate the evolutionary process in MAXTIME
    end_timeMH = get_time_in_seconds();
    currentTime = (float)(end_timeMH - start_timeMH);
  }

  // free memory of BRKGA components
  Pop.clear();
  PopInter.clear();

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] BRKGA finalizado. Geraçoes=" << numGenerations
              << ", Melhor OFV=" << bestInd.ofv << std::endl;
  }
}

} // namespace rkolib::mh