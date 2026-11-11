#include "rkolib/mh/brkga_cs.hpp"

#include "rkolib/core/method.hpp"
#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

using namespace rkolib::core;

// =========================================================================
// HELPER FUNCTIONS (INTERNAL)
// =========================================================================

// -------------------------------------------------------------------------
// Method: PearsonCorrelation
// -------------------------------------------------------------------------
static double PearsonCorrelation(const std::vector<double> &X,
                                 const std::vector<double> &Y, const int n) {
  double sumXY = 0;
  double sumX2 = 0;
  double sumY2 = 0;
  double sumX = 0;
  double sumY = 0;

  for (int j = 0; j < n; j++) {
    sumX += X[j];
    sumX2 += X[j] * X[j];
    sumXY += X[j] * Y[j];
    sumY += Y[j];
    sumY2 += Y[j] * Y[j];
  }

  double denominator =
      sqrt((n * sumX2 - sumX * sumX) * (n * sumY2 - sumY * sumY));

  if (denominator == 0) {
    return 0.0;
  }

  return ((n * sumXY) - (sumX * sumY)) / denominator;
}

// -------------------------------------------------------------------------
// Method: ParametricUniformCrossover
// -------------------------------------------------------------------------
static TSol ParametricUniformCrossover(int eliteSize, int popSize, double pm,
                                       double rhoe,
                                       const std::vector<TSol> &Pop,
                                       const int n) {
  int eliteParent = irandomico(0, eliteSize - 1);
  int nonEliteParent = irandomico(eliteSize, popSize - 1);

  TSol s = Pop[eliteParent];

  for (int j = 0; j < n; j++) {
    if (randomico(0, 1) < pm) {
      s.rk[j] = randomico(0, 1);
    } else {
      if (randomico(0, 1) < rhoe) {
        s.rk[j] = Pop[eliteParent].rk[j];
      } else {
        s.rk[j] = Pop[nonEliteParent].rk[j];
      }
    }
  }

  return s;
}

// -------------------------------------------------------------------------
// Method: ChaoticInd
// -------------------------------------------------------------------------
static void ChaoticInd(TSol &s, int rhoe, const int n) {
  for (int k = 0; k < n; k++) {
    if (randomico(0, 1) > (double)rhoe) {
      s.rk[k] = randomico(0, 1);
    }
  }
}

// -------------------------------------------------------------------------
// Method: UpdatePopulationSize
// -------------------------------------------------------------------------
static void UpdatePopulationSize(int p, double pe, double pm, double rhoe,
                                 std::vector<TSol> &Pop,
                                 std::vector<TSol> &PopInter,
                                 RkoSolver &solver) {
  int oldPsize = (int)Pop.size();
  if (oldPsize == p) {
    return;
  }

  if (oldPsize > p) {
    PopInter = Pop;
    Pop.resize(p);

    for (int i = 0; i < (int)(p * pe); i++) {
      Pop[i] = PopInter[i];
    }

    int pos = (int)(pe * oldPsize);
    for (int i = (int)(p * pe); i < p; i++) {
      if (pos < (int)PopInter.size()) {
        Pop[i] = PopInter[pos++];
      } else {
        CreateInitialSolutions(Pop[i], solver.getProblemDimension());
        solver.decodeSolution(Pop[i]);
      }
    }

    PopInter.clear();
    PopInter.resize(p);
    return;
  }

  Pop.resize(p);

  for (int k = oldPsize; k < p; k++) {
    if (SOLVER_SHOULD_STOP) {
      return;
    }

    Pop[k] = ParametricUniformCrossover((int)(oldPsize * pe), oldPsize - 1, pm,
                                        rhoe, Pop, solver.getProblemDimension());
    solver.decodeSolution(Pop[k]);
  }

  std::sort(Pop.begin(), Pop.end(), sortByFitness);

  PopInter.clear();
  PopInter.resize(p);
}

// -------------------------------------------------------------------------
// Helper: GetBestNeighborLabel
// -------------------------------------------------------------------------
static int GetBestNeighborLabel(int currentLabel,
                                const std::map<int, double> &totalLabels) {
  if (totalLabels.empty()) {
    return currentLabel;
  }

  double melhorPeso = std::numeric_limits<double>::lowest();
  auto it = totalLabels.find(currentLabel);
  if (it != totalLabels.end()) {
    melhorPeso = it->second;
  }

  int melhorLabel = currentLabel;
  for (const auto &pair : totalLabels) {
    if (pair.second > melhorPeso) {
      melhorLabel = pair.first;
      melhorPeso = pair.second;
    }
  }
  return melhorLabel;
}

// -------------------------------------------------------------------------
// Method: LP (Label Propagation)
// -------------------------------------------------------------------------
static void LP(const std::vector<std::vector<std::pair<int, double>>> &listaArestas,
               std::vector<int> &label) {
  int nk = (int)listaArestas.size();

  std::vector<int> ordemVisita(nk);
  std::iota(ordemVisita.begin(), ordemVisita.end(), 0);

  for (int i = 0; i < nk; i++) {
    label[i] = i;
  }

  int movimentos = 1;
  std::mt19937 g(std::random_device{}());

  while (movimentos) {
    movimentos = 0;
    std::shuffle(ordemVisita.begin(), ordemVisita.end(), g);

    for (std::vector<int>::size_type idVertice = 0;
         idVertice < ordemVisita.size(); idVertice++) {
      int u = ordemVisita[idVertice];

      std::map<int, double> totalLabels;
      for (const auto &edge : listaArestas[u]) {
        int labelVizinho = label[edge.first];
        totalLabels[labelVizinho] += edge.second;
      }

      int melhorLabel = GetBestNeighborLabel(label[u], totalLabels);
      if (melhorLabel != label[u]) {
        label[u] = melhorLabel;
        movimentos = 1;
      }
    }
  }
}

// -------------------------------------------------------------------------
// Method: PromisingLP
// -------------------------------------------------------------------------
static void PromisingLP(int p, double pe, std::vector<int> &label,
                        std::vector<int> &promising) {
  int Tpe = (int)(p * pe);
  std::vector<int> grupos;

  for (int i = 0; i < Tpe; i++) {
    bool found = false;
    for (unsigned int j = 0; j < grupos.size(); j++) {
      if (label[i] == grupos[j]) {
        found = true;
        break;
      }
    }
    if (!found) {
      grupos.push_back(label[i]);
    }
  }

  for (unsigned int j = 0; j < grupos.size(); j++) {
    int numTry = 0;
    while (numTry < Tpe * 2) {
      numTry++;
      int pos = irandomico(0, Tpe - 1);
      if (label[pos] == grupos[j]) {
        promising[pos] = 1;
        break;
      }
    }
  }
}

// -------------------------------------------------------------------------
// Method: IC (Initial Clustering)
// -------------------------------------------------------------------------
static void IC(int p, double pe, const std::vector<TSol> &Pop,
               std::vector<int> &promising, const int n) {
  int Tpe = (int)(p * pe);
  std::vector<std::vector<std::pair<int, double>>> listaArestas(Tpe);
  double sigma = 0.6;

  for (int i = 0; i < Tpe - 1; i++) {
    for (int j = i + 1; j < Tpe; j++) {
      double pearson = PearsonCorrelation(Pop[i].rk, Pop[j].rk, n);
      if (pearson > sigma) {
        listaArestas[i].push_back(std::make_pair(j, pearson));
        listaArestas[j].push_back(std::make_pair(i, pearson));
      }
    }
  }

  std::vector<int> label(Tpe, 0);
  LP(listaArestas, label);
  PromisingLP(p, pe, label, promising);
}

// -------------------------------------------------------------------------
// Helper: CrossoverMating
// -------------------------------------------------------------------------
static bool CrossoverMating(int p, double pe, double pm, double rhoe,
                            std::vector<TSol> &PopInter,
                            const std::vector<TSol> &Pop, double &bestOFV,
                            RkoSolver &solver, const TRunData &runData) {
  int eliteCount = (int)(p * pe);
  int dim = solver.getProblemDimension();

  for (int i = eliteCount; i < p; i++) {
    if (SOLVER_SHOULD_STOP) {
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] BRKGA-CS interrompido via SOLVER_SHOULD_STOP." << std::endl;
      }
      return false;
    }

    PopInter[i] = ParametricUniformCrossover(eliteCount, p, pm, rhoe, Pop, dim);
    solver.decodeSolution(PopInter[i]);

    if (PopInter[i].ofv < bestOFV) {
      bestOFV = PopInter[i].ofv;
    }
  }
  return true;
}

// -------------------------------------------------------------------------
// Helper: ApplyCommunityLocalSearch
// -------------------------------------------------------------------------
static bool ApplyCommunityLocalSearch(std::vector<TSol> &Pop, TSol &bestInd,
                                      int p, double pe, double rhoe,
                                      int &bestGeneration, int numGenerations,
                                      int &noImprovBRKGA,
                                      std::vector<int> &RKorder,
                                      RkoSolver &solver,
                                      const TRunData &runData,
                                      const char *method) {
  noImprovBRKGA = 0;
  int limitElite = (int)(p * pe);
  std::vector<int> promising(limitElite, 0);

  IC(p, pe, Pop, promising, solver.getProblemDimension());

  std::vector<int> promisingSol;
  for (int i = 0; i < limitElite; i++) {
    if (SOLVER_SHOULD_STOP) {
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] BRKGA-CS interrompido via SOLVER_SHOULD_STOP." << std::endl;
      }
      return false;
    }

    if (promising[i] == 1) {
      promisingSol.push_back(i);
    } else {
      ChaoticInd(Pop[i], (int)rhoe, solver.getProblemDimension());
      solver.decodeSolution(Pop[i]);
    }
  }

  for (unsigned int i = 0; i < promisingSol.size(); i++) {
    if (SOLVER_SHOULD_STOP) {
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] BRKGA-CS interrompido via SOLVER_SHOULD_STOP." << std::endl;
      }
      return false;
    }

    int solIndex = promisingSol[i];
    if (i < 1) {
      RVND(Pop[solIndex], solver, runData.strategy, RKorder);
    } else {
      NelderMeadSearch(Pop[solIndex], solver);
    }

    if (Pop[solIndex].ofv < bestInd.ofv) {
      bestInd = Pop[solIndex];
      bestGeneration = numGenerations;
      noImprovBRKGA = 0;

      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] BRKGA-CS (LS) encontrou nova melhor solucao: ofv=" 
                  << bestInd.ofv << std::endl;
      }

      UpdatePoolSolutions(bestInd, method, runData.debug, runData.poolUpdateMethod);
    }
  }

  std::sort(Pop.begin(), Pop.end(), sortByFitness);
  return true;
}

// =========================================================================
// MAIN ALGORITHM IMPLEMENTATION
// =========================================================================

void BRKGA_CS(const TRunData &runData, RkoSolver &solver) {
  const char *method = "BRKGA-CS";
  int p = 1597;     // size of population
  double pe = 0.20; // fraction of population to be the elite-set
  double pm = 0.03; // fraction of population to be replaced by mutants
  double rhoe = 0.70; // probability that offspring inherit an allele from elite parent

  std::vector<TSol> Pop;      // current population
  std::vector<TSol> PopInter; // intermediary population
  TSol bestInd;               // best solution of the BRKGA-CS

  double start_timeMH = get_time_in_seconds(); // start computational time
  double end_timeMH = get_time_in_seconds();   // end computational time

  int numGenerations = 0; // number of generations
  int bestGeneration = 0; // generation in which found the best solution
  float currentTime = 0;  // computational time of the search process
  int noImprovBRKGA = 0; // number of generations without improvement
  int improv = 0; // improvement flag

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
    std::cout << "[DEBUG][MH] BRKGA-CS iniciado. PopSize=" << p << ", pe=" << pe 
              << ", pm=" << pm << ", rhoe=" << rhoe << std::endl;
  }

  Pop.clear();
  PopInter.clear();
  Pop.resize(p);
  PopInter.resize(p);

  bestInd.ofv = std::numeric_limits<double>::infinity();

  for (int i = 0; i < p; i++) {
    CreateInitialSolutions(Pop[i], solver.getProblemDimension());
    solver.decodeSolution(Pop[i]);
    PopInter[i] = Pop[i];

    if (Pop[i].ofv < bestInd.ofv) {
      bestInd = Pop[i];
    }
  }

  std::sort(Pop.begin(), Pop.end(), sortByFitness);

  // run the evolutionary process until stop criterion
  while (currentTime < runData.MAXTIME * runData.restart) {
    numGenerations++;
    noImprovBRKGA++;
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
        p = (int)S[iCurr].par[0];
        pe = S[iCurr].par[1];
        pm = S[iCurr].par[2];
        rhoe = S[iCurr].par[3];

        UpdatePopulationSize(p, pe, pm, rhoe, Pop, PopInter, solver);
      }
    }

    if ((int)PopInter.size() != p) {
      PopInter.resize(p);
    }

    for (int i = 0; i < (int)(p * pe); i++) {
      PopInter[i] = Pop[i];
    }

    double bestOFV = INFINITY;
    if (!CrossoverMating(p, pe, pm, rhoe, PopInter, Pop, bestOFV, solver, runData)) {
      return;
    }

    Pop = PopInter;
    std::sort(Pop.begin(), Pop.end(), sortByFitness);

    if (Pop[0].ofv < bestInd.ofv) {
      bestInd = Pop[0];
      bestGeneration = numGenerations;
      noImprovBRKGA = 0;
      improv = 1;

      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] BRKGA-CS encontrou nova melhor solucao na geracao "
                  << numGenerations << ": ofv=" << bestInd.ofv << std::endl;
      }

      UpdatePoolSolutions(bestInd, method, runData.debug, runData.poolUpdateMethod);
    }

    // -----------------------------------------------------------------
    // Q-Learning Reward Phase (Post-Action)
    // -----------------------------------------------------------------
    if (runData.control == 1 && !S.empty()) {
      if (improv) {
        R = 1.0 + 1.0 / (double)p;
        improv = 0;
        delta = (bestOfvStartGen - bestInd.ofv) / (bestOfvStartGen + math_eps);
      } else {
        R = (std::abs(bestOFV) > math_eps) ? (bestInd.ofv - bestOFV) / (std::abs(bestOFV) + math_eps) : 0;
      }

      UpdateQValue(S, st, at, R, lf, df);
    }

    // Local Search in Communities
    if (noImprovBRKGA == 0 || noImprovBRKGA > 10) {
      if (!ApplyCommunityLocalSearch(Pop, bestInd, p, pe, rhoe, bestGeneration,
                                    numGenerations, noImprovBRKGA, RKorder,
                                    solver, runData, method)) {
        return;
      }
    }

    end_timeMH = get_time_in_seconds();
    currentTime = (float)(end_timeMH - start_timeMH);
  }

  Pop.clear();
  PopInter.clear();

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] BRKGA-CS finalizado. Geraçoes=" << numGenerations
              << ", Melhor OFV=" << bestInd.ofv << std::endl;
  }
}

} // namespace rkolib::mh