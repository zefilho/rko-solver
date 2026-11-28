#include "rkolib/mh/aco.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <vector>

#include "rkolib/core/method.hpp"
#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/solver.hpp"
#include "rkolib/utils/io.hpp"

namespace rkolib::mh {

using namespace rkolib::core;

// -------------------------------------------------------------------------
// Helper Function: UpdateArchiveSize
// -------------------------------------------------------------------------
static void UpdateArchiveSize(std::vector<TSol> &T, int archive_size,
                              RkoSolver &solver) {
  int oldPsize = static_cast<int>(T.size());

  if (oldPsize > archive_size) {
    T.resize(archive_size);
  } else if (oldPsize < archive_size) {
    T.resize(archive_size);
    int dim = solver.getProblemDimension();
    for (int i = oldPsize; i < archive_size; i++) {
      CreateInitialSolutions(T[i], dim);
      solver.decodeSolution(T[i]);
    }
    std::sort(T.begin(), T.end(), sortByFitness);
  }
}

// -------------------------------------------------------------------------
// Helper Function: ACO_calculate_weight
// -------------------------------------------------------------------------
static double ACO_calculate_weight(int archive_size, double q_param, int rank) {
  double l_minus_1 = static_cast<double>(rank - 1);
  double qk_product = q_param * archive_size;

  if (qk_product < std::numeric_limits<double>::epsilon()) {
    return (l_minus_1 == 0.0) ? 1.0 : 0.0;
  }

  constexpr double kPi = 3.14159265358979323846;
  double denominator = qk_product * std::sqrt(2.0 * kPi);
  double exponent = -(l_minus_1 * l_minus_1) / (2.0 * qk_product * qk_product);
  return (1.0 / denominator) * std::exp(exponent);
}

// -------------------------------------------------------------------------
// Helper Function: ACO_calculate_std_dev
// -------------------------------------------------------------------------
static double ACO_calculate_std_dev(int archive_size, double xi_param,
                                    const std::vector<TSol> &T,
                                    int dimension_idx,
                                    int chosen_solution_idx) {
  int k_minus_1 = archive_size - 1;
  if (k_minus_1 <= 0) {
    return 0.0;
  }

  double sum_distances = 0.0;
  double s_l_i = T[chosen_solution_idx].rk[dimension_idx];

  for (int e = 0; e < archive_size; ++e) {
    if (e == chosen_solution_idx)
      continue;
    double s_e_i = T[e].rk[dimension_idx];
    sum_distances += std::abs(s_e_i - s_l_i);
  }

  return xi_param * (sum_distances / k_minus_1);
}

// -------------------------------------------------------------------------
// Helper Function: ACO_construct_solution
// -------------------------------------------------------------------------
static TSol ACO_construct_solution(int archive_size, double q_param,
                                   double xi_param, const std::vector<TSol> &T,
                                   int n) {
  TSol new_sol;
  new_sol.rk.resize(n);

  std::vector<double> weights(archive_size);
  double total_weight = 0.0;
  for (int l = 0; l < archive_size; ++l) {
    weights[l] = ACO_calculate_weight(archive_size, q_param, l + 1);
    total_weight += weights[l];
  }

  int chosen_solution_idx = -1;
  double rand_val = randomico(0, 1);
  double cumulative_prob = 0.0;
  for (int l = 0; l < archive_size; ++l) {
    cumulative_prob += (total_weight > 0.0) ? (weights[l] / total_weight)
                                            : (1.0 / archive_size);
    if (rand_val <= cumulative_prob) {
      chosen_solution_idx = l;
      break;
    }
  }
  if (chosen_solution_idx == -1) {
    chosen_solution_idx = archive_size - 1;
  }

  for (int d = 0; d < n; ++d) {
    double mu_val = T[chosen_solution_idx].rk[d];
    double sigma_val =
        ACO_calculate_std_dev(archive_size, xi_param, T, d, chosen_solution_idx);

    if (sigma_val < std::numeric_limits<double>::epsilon()) {
      sigma_val = 0.9999;
    }

    std::normal_distribution<double> normal_dist(mu_val, sigma_val);
    double sampled_value = normal_dist(SOLVER_RNG);

    int attempts = 0;
    while ((sampled_value < 0.0 || sampled_value >= 1.0) && attempts < 100) {
      sampled_value = normal_dist(SOLVER_RNG);
      attempts++;
    }
    if (sampled_value < 0.0 || sampled_value >= 1.0) {
      sampled_value = randomico(0, 1);
    }

    new_sol.rk[d] = sampled_value;
  }

  return new_sol;
}

// -------------------------------------------------------------------------
// Helper Function: ACO_update_archive
// -------------------------------------------------------------------------
static void ACO_update_archive(int archive_size, std::vector<TSol> &T,
                               const std::vector<TSol> &new_solutions) {
  for (const auto &sol : new_solutions) {
    T.push_back(sol);
  }

  std::sort(T.begin(), T.end(), sortByFitness);

  if (static_cast<int>(T.size()) > archive_size) {
    T.resize(archive_size);
  }
}

// -------------------------------------------------------------------------
// Main Algorithm: ACO
// -------------------------------------------------------------------------
void ACO(const TRunData &runData, RkoSolver &solver) {
  const char *method = "ACO";

  int archive_size = 50;
  int num_ants = 2;
  double q_param = 0.01;
  double xi_param = 0.85;

  std::vector<TSol> T;
  TSol bestSolution;

  int numGenerations = 0;
  float currentTime = 0.0f;
  int bestGeneration = 0;
  int improv = 0;

  double start_timeMH = get_time_in_seconds();
  double end_timeMH = get_time_in_seconds();

  int dim = solver.getProblemDimension();

  // Q-Learning parameters
  std::vector<TState> S;
  int numPar = 4;
  int numStates = 0;
  int iCurr = 0;
  double epsilon = 0.0;
  double lf = 0.0;
  double df = 0.0;
  double R = 0.0;
  double delta = 0.0;
  const double epsilon_min = 0.05; // ACO.h:211
  int st = 0;
  int at = 0;

  std::vector<std::vector<double>> parameters(numPar);
  readParametersYaml(method, runData.control, parameters, numPar);

  if (runData.control == 0) {
    if (!parameters[0].empty())
      archive_size = static_cast<int>(parameters[0][0]);
    if (!parameters[1].empty())
      num_ants = static_cast<int>(parameters[1][0]);
    if (!parameters[2].empty())
      q_param = parameters[2][0];
    if (!parameters[3].empty())
      xi_param = parameters[3][0];
  } else if (runData.control == 1) {
    CreateStates(parameters, numStates, numPar, S);
    LoadQTable(S, method);

    epsilon = 0.95;
    delta = 0.0;
    if (!S.empty()) {
      iCurr = irandomico(0, numStates - 1);
      st = iCurr;
      archive_size = static_cast<int>(S[iCurr].par[0]);
      num_ants = static_cast<int>(S[iCurr].par[1]);
      q_param = S[iCurr].par[2];
      xi_param = S[iCurr].par[3];
    }
  }

  archive_size = std::max(2, archive_size);
  num_ants = std::max(1, num_ants);

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] ACO iniciado. archive_size=" << archive_size
              << ", num_ants=" << num_ants << ", q=" << q_param
              << ", xi=" << xi_param << std::endl;
  }

  T.resize(archive_size);
  bestSolution.ofv = std::numeric_limits<double>::infinity();

  for (int i = 0; i < archive_size; ++i) {
    CreateInitialSolutions(T[i], dim);
    solver.decodeSolution(T[i]);
    if (T[i].ofv < bestSolution.ofv) {
      bestSolution = T[i];
    }
  }
  std::sort(T.begin(), T.end(), sortByFitness);

  double lastNM_time = 0.0;
  const double tauMinNM_sec =
      static_cast<double>(runData.tauMinNM) * runData.MAXTIME;
  const double tauMaxNM_sec =
      static_cast<double>(runData.tauMaxNM) * runData.MAXTIME;

  while (currentTime < runData.MAXTIME * runData.restart &&
         !SOLVER_SHOULD_STOP) {
    numGenerations++;
    improv = 0;

    // Q-Learning Pre-Action
    if (runData.control == 1 && !S.empty()) {
      SetQLParameter(epsilon, lf, df,
                     static_cast<int>(runData.MAXTIME * runData.restart),
                     currentTime, static_cast<float>(delta), epsilon_min);
      delta = 0.0;

      at = ChooseAction(S, st, epsilon);
      if (at >= 0 && at < static_cast<int>(S[st].Ai.size())) {
        iCurr = S[st].Ai[at];
        archive_size = std::max(2, static_cast<int>(S[iCurr].par[0]));
        num_ants = std::max(1, static_cast<int>(S[iCurr].par[1]));
        q_param = S[iCurr].par[2];
        xi_param = S[iCurr].par[3];

        UpdateArchiveSize(T, archive_size, solver);
      }
    }

    std::vector<TSol> newSolutions;
    newSolutions.reserve(num_ants);

    for (int i = 0; i < num_ants; ++i) {
      if (SOLVER_SHOULD_STOP) {
        break;
      }

      TSol newSol =
          ACO_construct_solution(archive_size, q_param, xi_param, T, dim);
      solver.decodeSolution(newSol);
      newSolutions.push_back(newSol);

      if (newSol.ofv < bestSolution.ofv) {
        delta = std::max(delta, ComputeDelta(bestSolution.ofv, newSol.ofv));
        bestSolution = newSol;
        bestGeneration = numGenerations;
        improv = 1;
        if (runData.debug > 0) {
          std::cout
              << "[DEBUG][MH] ACO encontrou nova melhor solucao na geracao "
              << numGenerations << ": ofv=" << bestSolution.ofv << std::endl;
        }
        UpdatePoolSolutions(bestSolution, method, runData.debug,
                            runData.poolUpdateMethod);
      }
    }

    if (SOLVER_SHOULD_STOP) {
      break;
    }

    ACO_update_archive(archive_size, T, newSolutions);

    // Deterministic condition to call Nelder-Mead
    currentTime = static_cast<float>(get_time_in_seconds() - start_timeMH);
    double deltaT_NM = currentTime - lastNM_time;
    if ((improv && deltaT_NM >= tauMinNM_sec) || (deltaT_NM >= tauMaxNM_sec)) {
      NelderMeadSearch(T[0], solver);
      currentTime = static_cast<float>(get_time_in_seconds() - start_timeMH);
      lastNM_time = currentTime;

      if (T[0].ofv < bestSolution.ofv) {
        delta = std::max(delta, ComputeDelta(bestSolution.ofv, T[0].ofv));
        bestSolution = T[0];
        bestGeneration = numGenerations;
        improv = 1;
        if (runData.debug > 0) {
          std::cout << "[DEBUG][MH] ACO (LS) encontrou nova melhor solucao: ofv="
                    << bestSolution.ofv << std::endl;
        }
        UpdatePoolSolutions(bestSolution, method, runData.debug,
                            runData.poolUpdateMethod);
      }
    }

    // Q-Learning Reward Phase
    if (runData.control == 1 && !S.empty()) {
      if (improv) {
        R = 1.0;
        improv = 0;
      } else {
        R = 0.0;
      }
      UpdateQValue(S, st, at, R, lf, df);
    }

    end_timeMH = get_time_in_seconds();
    currentTime = static_cast<float>(end_timeMH - start_timeMH);
  }

  if (runData.control == 1 && !S.empty()) {
    SaveQTable(S, st, method);
  }

  T.clear();
  bestSolution.rk.clear();

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] ACO finalizado. Geracoes=" << numGenerations
              << ", Melhor Geracao=" << bestGeneration
              << ", Melhor OFV=" << bestSolution.ofv << std::endl;
  }
}

} // namespace rkolib::mh

