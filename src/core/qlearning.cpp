#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/method.hpp" // Necessário para randomico() e irandomico()

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <omp.h>
#include <sstream>

namespace rkolib::core {

void PrintPolicy(const std::vector<TState> &S, int st) {

  std::cout << std::format("Policy for state {}:", st) << std::endl;

  for (int i = 0; i < (int)S.size(); i++) {
    std::cout << std::format("\n i = {}: \t", i);
    for (int j = 0; j < (int)S[i].Ai.size(); j++) {
      std::cout << std::format("{} ({:.2f}) \t", S[i].Ai[j], S[i].Qa[j]);
    }
  }
}

bool SaveQTable(const std::vector<TState> &S, int st, const std::string &method,
                const std::string &outDir) {
  if (S.empty() || S[0].par.empty()) {
    return false;
  }

  bool success = false;
#pragma omp critical(qtable_io_lock)
  {
    try {
      if (!outDir.empty() && !std::filesystem::exists(outDir)) {
        std::filesystem::create_directories(outDir);
      }

      std::string filepath =
          outDir.empty() ? ("QTable_" + method + ".txt")
                         : (outDir + "/QTable_" + method + ".txt");

      std::ofstream out(filepath, std::ios::trunc);
      if (out.is_open()) {
        out << std::setprecision(10);
        out << "numStates " << S.size() << "\n";
        out << "numPar " << S[0].par.size() << "\n";
        out << "lastState " << st << "\n";

        for (size_t i = 0; i < S.size(); ++i) {
          out << "S " << i;
          for (double pVal : S[i].par) {
            out << " " << pVal;
          }
          out << "\n";

          out << "A " << S[i].Ai.size();
          for (size_t k = 0; k < S[i].Ai.size() && k < S[i].Qa.size(); ++k) {
            out << " " << S[i].Ai[k] << ":" << S[i].Qa[k];
          }
          out << "\n";
        }

        success = out.good();
      }
    } catch (...) {
      success = false;
    }
  }
  return success;
}

bool LoadQTable(std::vector<TState> &S, const std::string &method,
                const std::string &outDir) {
  if (S.empty() || S[0].par.empty()) {
    return false;
  }

  bool loaded = false;
#pragma omp critical(qtable_io_lock)
  {
    std::string filepath =
        outDir.empty() ? ("QTable_" + method + ".txt")
                       : (outDir + "/QTable_" + method + ".txt");

    std::ifstream in(filepath);
    if (in.is_open()) {
      std::string tag;
      size_t fileNumStates = 0;
      size_t fileNumPar = 0;
      int fileLastState = 0;

      bool headerOk = true;
      if (!(in >> tag >> fileNumStates) || tag != "numStates") headerOk = false;
      if (headerOk && (!(in >> tag >> fileNumPar) || tag != "numPar")) headerOk = false;
      if (headerOk && (!(in >> tag >> fileLastState) || tag != "lastState")) headerOk = false;

      if (headerOk && fileNumStates == S.size() && fileNumPar == S[0].par.size()) {
        std::vector<std::vector<double>> loadedQa(S.size());
        bool valid = true;

        for (size_t i = 0; i < S.size() && valid; ++i) {
          size_t stateIdx = 0;
          if (!(in >> tag >> stateIdx) || tag != "S" || stateIdx != i) {
            valid = false;
            break;
          }

          for (size_t j = 0; j < fileNumPar; ++j) {
            double pVal = 0.0;
            if (!(in >> pVal) || std::fabs(pVal - S[i].par[j]) > 1e-6) {
              valid = false;
              break;
            }
          }
          if (!valid) break;

          size_t numActions = 0;
          if (!(in >> tag >> numActions) || tag != "A" || numActions != S[i].Ai.size()) {
            valid = false;
            break;
          }

          loadedQa[i].resize(numActions, 0.0);
          for (size_t k = 0; k < numActions; ++k) {
            std::string pairToken;
            if (!(in >> pairToken)) {
              valid = false;
              break;
            }
            size_t colonPos = pairToken.find(':');
            if (colonPos == std::string::npos) {
              valid = false;
              break;
            }
            try {
              int target = std::stoi(pairToken.substr(0, colonPos));
              double qVal = std::stod(pairToken.substr(colonPos + 1));
              if (target != S[i].Ai[k] || !std::isfinite(qVal)) {
                valid = false;
                break;
              }
              loadedQa[i][k] = qVal;
            } catch (...) {
              valid = false;
              break;
            }
          }
        }

        if (valid) {
          for (size_t i = 0; i < S.size(); ++i) {
            S[i].Qa = std::move(loadedQa[i]);
            UpdateMaxQ(S, static_cast<int>(i));
          }
          loaded = true;
        }
      }
    }
  }
  return loaded;
}

// Helper 1: Calculates the total number of combinations
int CalculateTotalStates(const std::vector<std::vector<double>> &parameters,
                         int numPar) {
  if (parameters.empty())
    return 0;

  int total = static_cast<int>(parameters[0].size());
  for (int i = 1; i < numPar; i++) {
    total *= static_cast<int>(parameters[i].size());
  }
  return total;
}

// Helper 2: Initializes the vector with empty states
void InitializeStateVector(std::vector<TState> &S, int numStates, int numPar) {
  S.clear();            // Good practice to clear before pushing
  S.reserve(numStates); // Optimization

  for (int i = 0; i < numStates; i++) {
    TState sAux;
    sAux.label = i;
    sAux.par.resize(numPar);
    sAux.ci = 0;
    sAux.numN = 0;
    sAux.Ai.clear();
    sAux.Qa.clear();
    sAux.sumQ = 0;
    sAux.maxQ = -std::numeric_limits<double>::infinity();
    sAux.minA = 0;
    sAux.maxA = 0;
    S.push_back(sAux);
  }
}

// Helper 3: Maps the Cartesian product of parameters to states
void MapParametersToStates(std::vector<TState> &S,
                           const std::vector<std::vector<double>> &parameters,
                           int numPar, int numStates) {
  int nC = 1;
  for (int j = 0; j < numPar; j++) {
    nC *= static_cast<int>(parameters[j].size());

    // Calculate divisor once per column group to reduce inner complexity
    int den = (numStates / nC);
    if (den == 0)
      den = 1;

    for (int i = 0; i < numStates; i++) {
      int columnIndex = (i / den) % static_cast<int>(parameters[j].size());
      S[i].par[j] = parameters[j][columnIndex];
    }
  }
}

// Helper 4: Establishes adjacent M/D/A connections between states via stride in O(|S| * m)
void BuildStateConnections(std::vector<TState> &S,
                           const std::vector<std::vector<double>> &parameters,
                           int numPar, int numStates) {
  if (numPar <= 0 || numStates <= 0 || S.empty())
    return;

  std::vector<int> stride(numPar);
  int prod = 1;
  for (int j = 0; j < numPar; j++) {
    prod *= static_cast<int>(parameters[j].size());
    stride[j] = (prod > 0) ? (numStates / prod) : 1;
  }

  for (int i = 0; i < static_cast<int>(S.size()); i++) {
    std::vector<int> targets;
    targets.reserve(1 + 2 * numPar);

    // Action M: keep current configuration
    targets.push_back(i);

    for (int j = 0; j < numPar; j++) {
      int size = static_cast<int>(parameters[j].size());
      int idx = (i / stride[j]) % size;

      // Action D: previous value of parameter j
      if (idx > 0) {
        targets.push_back(i - stride[j]);
      }
      // Action A: next value of parameter j
      if (idx < size - 1) {
        targets.push_back(i + stride[j]);
      }
    }

    for (int target : targets) {
      S[i].Ai.push_back(target);
      double q0 = randomico(0.005, 0.01);
      S[i].Qa.push_back(q0);

      if (q0 > S[i].maxQ) {
        S[i].maxQ = q0;
        S[i].maxA = static_cast<int>(S[i].Ai.size()) - 1;
      }
    }
  }
}

void CreateStates(const std::vector<std::vector<double>> &parameters,
                  int &numStates, int numPar, std::vector<TState> &S) {
#pragma omp critical
  {
    // 1. Calculate dimensions
    numStates = CalculateTotalStates(parameters, numPar);

    // 2. Allocate memory and defaults
    InitializeStateVector(S, numStates, numPar);

    // 3. Fill parameter configurations
    MapParametersToStates(S, parameters, numPar, numStates);

    // 4. Create adjacent M/D/A topology (Edges and Q-Values)
    BuildStateConnections(S, parameters, numPar, numStates);
  }
}

int ChooseAction(const std::vector<TState> &S, int st, double epsilon) {
  // ** choose action for current state from Q-Table using epsilon-Greedy policy
  int at;

  // epsilon-greedy policy
  if (randomico(0, 1) <= 1.0 - epsilon) {
    // choose the action with highest Q value
    at = S[st].maxA;
  } else {
    // choose a randomly selected action
    if (!S[st].Ai.empty()) {
      at = irandomico(0, (int)S[st].Ai.size() - 1);
    } else {
      at = 0; // Fallback se não houver ações
    }
  }

  // return the chosen action
  return at;
}

double ComputeDelta(double oldBest, double newBest, double delta_max) {
  if (newBest >= oldBest)
    return 0.0;

  double den = std::fabs(oldBest);
  if (den < 1e-12)
    return delta_max;

  double d = (oldBest - newBest) / den;
  return (d > delta_max) ? delta_max : d;
}

void SetQLParameter(float currentTime, int &Ti, int &restartEpsilon,
                    float &epsilon_max, float epsilon_min, double &epsilon,
                    double &lf, double &df, int MAXTIME) {
  // **** define epsilon ****
  static const double PI = 3.14159265;

  // restart epsilon once Ti epochs are performed (Ti is 10% of the runtime)
  Ti = (int)(MAXTIME * 0.1);
  if (Ti == 0)
    Ti = 1; // Proteção contra divisão por zero

  if (currentTime >= restartEpsilon * Ti) {
    restartEpsilon++;

    // cosine decay with warm restart
    epsilon_max = epsilon_max - 0.1f;
    if (epsilon_max < epsilon_min)
      epsilon_max = epsilon_min;
    epsilon = epsilon_max;
  } else {
    epsilon = epsilon_min +
              0.5 * (epsilon_max - epsilon_min) *
                  (1.0 + cos((((int)currentTime % Ti) / (float)(Ti)) * PI));
  }

  // *** define learning rate ***
  lf = std::max(0.01, 1.0 - (0.9 * currentTime / (double)MAXTIME));

  // *** define discount rate ***
  df = 0.8;
}

void SetQLParameter(double &epsilon, double &lf, double &df, int MAXTIME,
                    float currentTime, float delta, double epsilon_min) {
  double d = std::clamp(static_cast<double>(delta), 0.0, 1.0);

  epsilon = epsilon * (1.0 - d);
  if (epsilon < epsilon_min) {
    epsilon = epsilon_min;
  }

  // *** define learning rate ***
  lf = std::max(0.01,
                1.0 - (0.9 * static_cast<double>(currentTime) /
                       static_cast<double>(MAXTIME)));

  // *** define discount rate ***
  df = 0.8;
}

void UpdateMaxQ(std::vector<TState> &S, int st) {
  if (st < 0 || st >= static_cast<int>(S.size()) || S[st].Qa.empty())
    return;

  S[st].maxA = 0;
  S[st].maxQ = S[st].Qa[0];
  for (size_t k = 1; k < S[st].Qa.size(); ++k) {
    if (S[st].Qa[k] > S[st].maxQ) {
      S[st].maxQ = S[st].Qa[k];
      S[st].maxA = static_cast<int>(k);
    }
  }
}

void UpdateQValue(std::vector<TState> &S, int &st, int at, double R, double lf,
                  double df) {
  // Validação completa de índices para evitar acesso fora dos limites
  if (st < 0 || st >= static_cast<int>(S.size()))
    return;
  if (at < 0 || at >= static_cast<int>(S[st].Ai.size()) ||
      at >= static_cast<int>(S[st].Qa.size()))
    return;

  const int st_1 = S[st].Ai[at];
  if (st_1 < 0 || st_1 >= static_cast<int>(S.size()))
    return;

  // Equação de Bellman padrão
  S[st].Qa[at] += lf * (R + df * S[st_1].maxQ - S[st].Qa[at]);

  // Atualização condicional de maxQ / maxA em O(1) no caso comum
  if (S[st].Qa[at] > S[st].maxQ) {
    S[st].maxQ = S[st].Qa[at];
    S[st].maxA = at;
  } else if (at == S[st].maxA) {
    UpdateMaxQ(S, st);
  }

  // Transição para o próximo estado
  st = st_1;
}

} // namespace rkolib::core