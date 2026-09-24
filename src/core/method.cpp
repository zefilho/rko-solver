#include "rkolib/core/method.hpp"
#include "rkolib/core/solver.hpp"
#include <omp.h> // OpenMP
#include <yaml-cpp/yaml.h>

namespace rkolib::core {

// -----------------------------------------------------------------------------
// General Utilities
// -----------------------------------------------------------------------------

bool sortByFitness(const TSol &lhs, const TSol &rhs) {
  return lhs.ofv < rhs.ofv;
}

double randomico(double min, double max) {
  // Thread-Safe now!
  // SOLVER_RNG calls getRng(), which uses omp_get_thread_num()
  std::uniform_real_distribution<double> dist(min, max);
  return dist(SOLVER_RNG);
}

int irandomico(int min, int max) {
  // Thread-Safe now!
  std::uniform_int_distribution<int> dist(min, max);
  return dist(SOLVER_RNG);
}

double get_time_in_seconds() {
#if defined(_WIN32) || defined(_WIN64)
  LARGE_INTEGER frequency;
  LARGE_INTEGER timeCur;
  QueryPerformanceFrequency(&frequency);
  QueryPerformanceCounter(&timeCur);
  return static_cast<double>(timeCur.QuadPart) / frequency.QuadPart;
#else
  struct timespec timeCur;
  clock_gettime(CLOCK_MONOTONIC, &timeCur);
  return timeCur.tv_sec + timeCur.tv_nsec / 1e9;
#endif
}

// -----------------------------------------------------------------------------
// Solution & Pool Management
// -----------------------------------------------------------------------------

void CreateInitialSolutions(TSol &s, const int n) {
  s.rk.resize(n);
  // create a random-key solution
  for (int j = 0; j < n; j++) {
    s.rk[j] = randomico(0, 1); // random value between [0,1)
  }
}

void CreatePoolSolutions(rkolib::RkoSolver &solver, const int sizePool) {

  // 1. Definição Dinâmica do Tamanho do Pool
  int actualSizePool = sizePool;
  if (solver.getNumObjectives() > 1) { 
      actualSizePool = sizePool * 2;
  }

  #pragma omp critical
  {
    auto &ctx = SolverContext::instance();
    if (ctx.getPool().size() != (size_t)actualSizePool) {
      ctx.initializePool(actualSizePool);
    }

    // Pass 1: Decode solutions to initialize random keys and establish consolidated reference points (ideal/nadir)
    for (int i = 0; i < actualSizePool; i++) {
      CreateInitialSolutions(SOLVER_POOL[i], solver.getProblemDimension());
      solver.decodeSolution(SOLVER_POOL[i]);
    }

    // Pass 2: Re-scalarize all pool solutions using the consolidated ideal/nadir reference points
    for (int i = 0; i < actualSizePool; i++) {
      solver.decodeSolution(SOLVER_POOL[i]);
      SOLVER_POOL[i].best_time = get_time_in_seconds();
    }

    // sort SOLVER_POOL in increasing order of fitness
    std::sort(SOLVER_POOL.begin(), SOLVER_POOL.begin() + actualSizePool,
              sortByFitness);

    // verify if similar solutions exist in the SOLVER_POOL
    int clone = 0;
    // Note: original reverse loop maintained, but check if actualSizePool <=
    // SOLVER_POOL.size()
    for (int i = actualSizePool - 1; i > 0; i--) {
      if (std::abs(SOLVER_POOL[i].ofv - SOLVER_POOL[i - 1].ofv) <
          1e-9) { // Safe double comparison
        for (int j = 0; j < 0.2 * solver.getProblemDimension(); j++) {
          int pos = irandomico(0, solver.getProblemDimension() - 1);
          SOLVER_POOL[i].rk[pos] = randomico(0, 1);
        }
        solver.decodeSolution(SOLVER_POOL[i]);
        clone = 1;
      }
    }

    // sort SOLVER_POOL again if clones were mutated
    if (clone) {
      for (int i = 0; i < actualSizePool; i++) {
        solver.decodeSolution(SOLVER_POOL[i]);
      }
      std::sort(SOLVER_POOL.begin(), SOLVER_POOL.begin() + actualSizePool,
                sortByFitness);
    }
  }
}

// Return a value between -1.0 and 1.0. The higher, the more similar the vectors are.
inline double CosineSimilarity(const std::vector<double>& a, const std::vector<double>& b) {
    double dot = 0.0, norm_a = 0.0, norm_b = 0.0;
    
    // Assuming a.size() == b.size() (same problem dimension)
    for (size_t i = 0; i < a.size(); ++i) {
        dot += a[i] * b[i];
        norm_a += a[i] * a[i];
        norm_b += b[i] * b[i];
    }
    
    if (norm_a == 0.0 || norm_b == 0.0) return 0.0;
    return dot / (std::sqrt(norm_a) * std::sqrt(norm_b));
}

// void UpdatePoolSolutions(TSol s, const char *mh, const int debug) {
  
//   #pragma omp critical(pool_lock)
//   {
//     int pool_size = static_cast<int>(SOLVER_POOL.size());
    
//     // Only proceed if the pool is initialized
//     if (pool_size > 0) {
        
//         // ====================================================================
//         // 1. GUARDIÃO MULTI-OBJETIVO: Filtro de Dominância de Pareto
//         // ====================================================================
//         if (s.objs.size() > 1) {
//             bool is_dominated = false;
//             for (const auto& pool_sol : SOLVER_POOL) {
//                 // Checagens para Minimização
//                 bool strictly_better = false;
//                 bool worse_in_any = false;
                
//                 for (size_t k = 0; k < s.objs.size(); ++k) {
//                     // Tolerância epsilon (1e-9) para erros de precisão
//                     if (pool_sol.objs[k] > s.objs[k] + 1e-9) worse_in_any = true;
//                     if (pool_sol.objs[k] < s.objs[k] - 1e-9) strictly_better = true;
//                 }
                
//                 if (!worse_in_any && strictly_better) {
//                     is_dominated = true;
//                     break; 
//                 }
//             }
            
//             // Rejeição imediata se for dominada
//             if (is_dominated) return; 
//         }

//         // ====================================================================
//         // 2. Filtro de Existência (Evita clones perfeitos de OFV escalarizado)
//         // ====================================================================
//         bool exists = std::ranges::any_of(SOLVER_POOL, [&](const auto &entry) {
//             return std::abs(entry.ofv - s.ofv) < 1e-9;
//         });

//         // 3. Entry Filter: Only accept if it doesn't exist AND is strictly better than the worst
//         if (!exists && s.ofv < SOLVER_POOL.back().ofv) {
            
//             // The solution is accepted! Update its metadata.
//             s.best_time = get_time_in_seconds();
//             s.nameMH = mh;

//             // Log globally if a new overall best solution is found
//             if (s.ofv < SOLVER_POOL[0].ofv && debug) {
//                 int thread_id = omp_get_thread_num();
//                 std::cout << std::format("\nBest solution: {:.10f} (Thread: {} - MH: {})",
//                                          s.ofv, thread_id, mh);
//             }

//             // ====================================================================
//             // 4. Estratégia de Diversidade (Cosine Similarity)
//             // ====================================================================
//             int start_idx = std::max(1, pool_size / 2); 
//             int target_idx = pool_size - 1; // Default victim is the absolute worst
//             double max_sim = -2.0;          

//             for (int i = start_idx; i < pool_size; i++) {
//                 // Extra safeguard: the victim MUST have a worse objective function value
//                 if (SOLVER_POOL[i].ofv > s.ofv) {
//                     double sim = rkolib::core::CosineSimilarity(s.rk, SOLVER_POOL[i].rk);
                    
//                     if (sim > max_sim) {
//                         max_sim = sim;
//                         target_idx = i;
//                     }
//                 }
//             }

//             // 5. Substituição tática
//             SOLVER_POOL[target_idx] = s;

//             // 6. Reordena o pool para manter o invariante 
//             // (Ordemos pelo Tchebycheff/OFV Escalarizado para apoiar o MOEA/D)
//             std::ranges::sort(SOLVER_POOL, [](const TSol& a, const TSol& b) {
//                 return a.ofv < b.ofv; // Ascending order
//             });
            
//         } // End of conditional entry
//     } // End of pool_size check
//   } // End of #pragma omp critical(pool_lock)
// }

// void UpdatePoolSolutions(core::TSol s, const char *mh, const int debug) {
//   #pragma omp critical(pool_lock)
//   {
//     int pool_size = static_cast<int>(SOLVER_POOL.size());
//     if (pool_size == 0) return;

//     bool is_mo = s.objs.size() > 1;

//     // ====================================================================
//     // 1. Filtro de Clones Exatos (Objetivos Reais)
//     // ====================================================================
//     bool exists = std::ranges::any_of(SOLVER_POOL, [&](const auto &entry) {
//         if (is_mo) {
//             for (size_t k = 0; k < s.objs.size(); ++k)
//                 if (std::abs(entry.objs[k] - s.objs[k]) > 1e-9) return false;
//             return true;
//         }
//         return std::abs(entry.ofv - s.ofv) < 1e-9;
//     });

//     if (exists) return; // Descarte imediato

//     // Atualiza metadados
//     s.best_time = get_time_in_seconds();
//     s.nameMH = mh;

//     // Se for Mono-Objetivo, mantém a lógica antiga simplificada
//     if (!is_mo) {
//         if (s.ofv < SOLVER_POOL.back().ofv) {
//             SOLVER_POOL.back() = s;
//             std::ranges::sort(SOLVER_POOL, [](const auto& a, const auto& b) { return a.ofv < b.ofv; });
            
//             if (s.ofv < SOLVER_POOL[0].ofv && debug) {
//                 std::cout << std::format("\nBest (Mono): {:.6f} (T-{} | MH: {})", s.ofv, omp_get_thread_num(), mh);
//             }
//         }
//         return;
//     }

//     // ====================================================================
//     // 2. LÓGICA NSGA-II (Fast Non-Dominated Sorting & Crowding Distance)
//     // ====================================================================
    
//     // Cria população unida N+1
//     std::vector<core::TSol> combined = SOLVER_POOL;
//     combined.push_back(s);
//     int N_plus_1 = combined.size();

//     // Estruturas auxiliares para o NSGA-II
//     std::vector<int> rank(N_plus_1, 0);
//     std::vector<double> cd(N_plus_1, 0.0);
//     std::vector<std::vector<int>> S_dom(N_plus_1); // Soluções dominadas por i
//     std::vector<int> n_dom(N_plus_1, 0); // Quantas dominam i
//     std::vector<std::vector<int>> fronts(1);

//     // Passo 2.1: Identificação de Fronteiras (Fast Non-Dominated Sort)
//     for (int p = 0; p < N_plus_1; p++) {
//         for (int q = 0; q < N_plus_1; q++) {
//             if (p == q) continue;
            
//             bool p_dominates_q = false;
//             bool q_dominates_p = false;
//             bool p_strictly_better = false;
//             bool q_strictly_better = false;

//             for (size_t k = 0; k < s.objs.size(); k++) {
//                 if (combined[p].objs[k] < combined[q].objs[k] - 1e-9) p_strictly_better = true;
//                 if (combined[q].objs[k] < combined[p].objs[k] - 1e-9) q_strictly_better = true;
//             }

//             if (p_strictly_better && !q_strictly_better) p_dominates_q = true;
//             if (q_strictly_better && !p_strictly_better) q_dominates_p = true;

//             if (p_dominates_q) S_dom[p].push_back(q);
//             else if (q_dominates_p) n_dom[p]++;
//         }

//         if (n_dom[p] == 0) {
//             rank[p] = 1;
//             fronts[0].push_back(p);
//         }
//     }

//     // Define fronteiras subsequentes
//     int i = 0;
//     while (!fronts[i].empty()) {
//         std::vector<int> next_front;
//         for (int p : fronts[i]) {
//             for (int q : S_dom[p]) {
//                 n_dom[q]--;
//                 if (n_dom[q] == 0) {
//                     rank[q] = i + 2;
//                     next_front.push_back(q);
//                 }
//             }
//         }
//         i++;
//         if (!next_front.empty()) fronts.push_back(next_front);
//     }

//     // Passo 2.2: Cálculo do Crowding Distance
//     int num_objs = s.objs.size();
//     for (const auto& front : fronts) {
//         if (front.empty()) continue;
        
//         int l = front.size();
//         for (int idx : front) cd[idx] = 0.0; // Reseta distâncias
        
//         for (int m = 0; m < num_objs; m++) {
//             // Ordena a fronteira pelo objetivo 'm'
//             std::vector<int> sorted_front = front;
//             std::ranges::sort(sorted_front, [&](int a, int b) {
//                 return combined[a].objs[m] < combined[b].objs[m];
//             });

//             // Extremos recebem distância infinita para sempre serem preservados
//             cd[sorted_front[0]] = std::numeric_limits<double>::infinity();
//             cd[sorted_front[l - 1]] = std::numeric_limits<double>::infinity();

//             double obj_min = combined[sorted_front[0]].objs[m];
//             double obj_max = combined[sorted_front[l - 1]].objs[m];
//             double delta = obj_max - obj_min;
//             if (delta < 1e-9) delta = 1.0; // Previne divisão por zero

//             for (int j = 1; j < l - 1; j++) {
//                 if (cd[sorted_front[j]] != std::numeric_limits<double>::infinity()) {
//                     cd[sorted_front[j]] += (combined[sorted_front[j + 1]].objs[m] - 
//                                             combined[sorted_front[j - 1]].objs[m]) / delta;
//                 }
//             }
//         }
//     }

//     // Passo 2.3: Operador de Comparação (Crowded-Comparison)
//     std::vector<int> indices(N_plus_1);
//     std::iota(indices.begin(), indices.end(), 0);

//     // O melhor é quem tem o Menor Rank. 
//     // Em caso de empate no Rank, o melhor é quem tem MAIOR Crowding Distance.
//     std::ranges::sort(indices, [&](int a, int b) {
//         if (rank[a] != rank[b]) return rank[a] < rank[b];
//         return cd[a] > cd[b];
//     });

//     // Passo 2.4: Trunca para o tamanho original e salva de volta no Pool
//     for (int k = 0; k < pool_size; k++) {
//         SOLVER_POOL[k] = combined[indices[k]];
//     }

//     if (debug && rank[N_plus_1 - 1] == 1) { 
//         // Se a nova solução (índice N_plus_1-1 original) entrou na Fronteira 1
//         std::cout << std::format("\nNova solucao Pareto-Otima (T-{} | MH: {})", omp_get_thread_num(), mh);
//     }
//   } // End #pragma omp critical(pool_lock)
// }

void UpdatePoolSolutions(core::TSol s, const char *mh, const int debug, int updateMethod) {
  #pragma omp critical(pool_lock)
  {
    int pool_size = static_cast<int>(SOLVER_POOL.size());
    
    // REGRA OPENMP: Só executa se o pool for maior que 0 (Sem 'return' antecipado)
    if (pool_size > 0) {
        bool is_mo = s.objs.size() > 1;

        // ====================================================================
        // FILTRO GLOBAL: Guardião de Clones e Dominância Restrita
        // ====================================================================
        bool exists = false;
        bool is_dominated = false;

        for (const auto& pool_sol : SOLVER_POOL) {
            if (is_mo) {
                // Verifica Clone Exato
                bool clone = true;
                for (size_t k = 0; k < s.objs.size(); ++k) {
                    if (std::abs(pool_sol.objs[k] - s.objs[k]) > 1e-9) clone = false;
                }
                if (clone) { exists = true; break; }

                // Verifica Dominância
                bool strictly_better = false;
                bool worse_in_any = false;
                for (size_t k = 0; k < s.objs.size(); ++k) {
                    if (pool_sol.objs[k] > s.objs[k] + 1e-9) worse_in_any = true;
                    if (pool_sol.objs[k] < s.objs[k] - 1e-9) strictly_better = true;
                }
                if (!worse_in_any && strictly_better) {
                    is_dominated = true;
                    break;
                }
            } else {
                if (std::abs(pool_sol.ofv - s.ofv) < 1e-9) { exists = true; break; }
            }
        }

        // REGRA OPENMP: Só procede se a solução passou nos filtros
        if (!exists && !is_dominated) {
            
            // Atualiza metadados da nova solução
            s.best_time = get_time_in_seconds();
            s.nameMH = mh;

            // ====================================================================
            // MÉTODO 0: SUBSTITUIÇÃO TÁTICA (Cosseno) OU MONO-OBJETIVO
            // ====================================================================
            if (updateMethod == 0 || !is_mo) { 
                if (s.ofv < SOLVER_POOL.back().ofv) {
                    bool new_best = (s.ofv < SOLVER_POOL[0].ofv);
                    int start_idx = std::max(1, pool_size / 2); 
                    int target_idx = pool_size - 1; 
                    double max_sim = -2.0;          

                    for (int i = start_idx; i < pool_size; i++) {
                        if (SOLVER_POOL[i].ofv > s.ofv) {
                            double sim = rkolib::core::CosineSimilarity(s.rk, SOLVER_POOL[i].rk);
                            if (sim > max_sim) {
                                max_sim = sim;
                                target_idx = i;
                            }
                        }
                    }
                    SOLVER_POOL[target_idx] = s;
                    std::ranges::sort(SOLVER_POOL, [](const auto& a, const auto& b) { return a.ofv < b.ofv; });

                    if (debug && new_best) {
                        std::cout << "\n[DEBUG][pool] Nova melhor solucao pela MH (" << mh 
                                  << "): ofv=" << s.ofv << std::endl;
                    }
                }
            }
            // ====================================================================
            // MÉTODO 1: NSGA-II (Rank + Crowding Distance)
            // ====================================================================
            else if (updateMethod == 1 && is_mo) {
                std::vector<core::TSol> combined = SOLVER_POOL;
                combined.push_back(s);
                int N_plus_1 = combined.size();

                std::vector<int> rank(N_plus_1, 0);
                std::vector<double> cd(N_plus_1, 0.0);
                std::vector<std::vector<int>> S_dom(N_plus_1); // Soluções dominadas por i
                std::vector<int> n_dom(N_plus_1, 0); // Quantas dominam i
                std::vector<std::vector<int>> fronts(1);

                // Passo 2.1: Identificação de Fronteiras (Fast Non-Dominated Sort)
                for (int p = 0; p < N_plus_1; p++) {
                    for (int q = 0; q < N_plus_1; q++) {
                        if (p == q) continue;
                        
                        bool p_dominates_q = false;
                        bool q_dominates_p = false;
                        bool p_strictly_better = false;
                        bool q_strictly_better = false;

                        for (size_t k = 0; k < s.objs.size(); k++) {
                            if (combined[p].objs[k] < combined[q].objs[k] - 1e-9) p_strictly_better = true;
                            if (combined[q].objs[k] < combined[p].objs[k] - 1e-9) q_strictly_better = true;
                        }

                        if (p_strictly_better && !q_strictly_better) p_dominates_q = true;
                        if (q_strictly_better && !p_strictly_better) q_dominates_p = true;

                        if (p_dominates_q) S_dom[p].push_back(q);
                        else if (q_dominates_p) n_dom[p]++;
                    }

                    if (n_dom[p] == 0) {
                        rank[p] = 1;
                        fronts[0].push_back(p);
                    }
                }

                // Define fronteiras subsequentes
                int i = 0;
                while (!fronts[i].empty()) {
                    std::vector<int> next_front;
                    for (int p : fronts[i]) {
                        for (int q : S_dom[p]) {
                            n_dom[q]--;
                            if (n_dom[q] == 0) {
                                rank[q] = i + 2;
                                next_front.push_back(q);
                            }
                        }
                    }
                    i++;
                    if (!next_front.empty()) fronts.push_back(next_front);
                }

                // Passo 2.2: Cálculo do Crowding Distance
                int num_objs = s.objs.size();
                for (const auto& front : fronts) {
                    if (front.empty()) continue;
                    
                    int l = front.size();
                    for (int idx : front) cd[idx] = 0.0; // Reseta distâncias
                    
                    for (int m = 0; m < num_objs; m++) {
                        // Ordena a fronteira pelo objetivo 'm'
                        std::vector<int> sorted_front = front;
                        std::ranges::sort(sorted_front, [&](int a, int b) {
                            return combined[a].objs[m] < combined[b].objs[m];
                        });

                        // Extremos recebem distância infinita para sempre serem preservados
                        cd[sorted_front[0]] = std::numeric_limits<double>::infinity();
                        cd[sorted_front[l - 1]] = std::numeric_limits<double>::infinity();

                        double obj_min = combined[sorted_front[0]].objs[m];
                        double obj_max = combined[sorted_front[l - 1]].objs[m];
                        double delta = obj_max - obj_min;
                        if (delta < 1e-9) delta = 1.0; // Previne divisão por zero

                        for (int j = 1; j < l - 1; j++) {
                            if (cd[sorted_front[j]] != std::numeric_limits<double>::infinity()) {
                                cd[sorted_front[j]] += (combined[sorted_front[j + 1]].objs[m] - 
                                                        combined[sorted_front[j - 1]].objs[m]) / delta;
                            }
                        }
                    }
                }

                // Passo 2.3: Operador de Comparação (Crowded-Comparison)
                std::vector<int> indices(N_plus_1);
                std::iota(indices.begin(), indices.end(), 0);

                // O melhor é quem tem o Menor Rank. 
                // Em caso de empate no Rank, o melhor é quem tem MAIOR Crowding Distance.
                std::ranges::sort(indices, [&](int a, int b) {
                    if (rank[a] != rank[b]) return rank[a] < rank[b];
                    return cd[a] > cd[b];
                });

                // Passo 2.4: Trunca para o tamanho original e salva de volta no Pool
                for (int k = 0; k < pool_size; k++) {
                    SOLVER_POOL[k] = combined[indices[k]];
                }

                if (debug && rank[N_plus_1 - 1] == 1) { 
                    // Se a nova solução (índice N_plus_1-1 original) entrou na Fronteira 1
                    std::cout << std::format("\nNova solucao Pareto-Otima (T-{} | MH: {})", omp_get_thread_num(), mh);
                }
            }
            // ====================================================================
            // MÉTODO 2: MEMÓRIA ESPACIAL DA FRONTEIRA (Geometria de Nichos)
            // ====================================================================
            else if (updateMethod == 2 && is_mo && s.objs.size() >= 2) {
                
                const int K = 5; 
                int capacity = std::max(1, (pool_size + K - 1) / K); 

                // Passo 1: Encontrar Extremos da Fronteira Atual (E0 e E1)
                int idx_E0 = 0, idx_E1 = 0;
                for (int i = 1; i < pool_size; i++) {
                    if (SOLVER_POOL[i].objs[1] < SOLVER_POOL[idx_E0].objs[1]) idx_E0 = i;
                    if (SOLVER_POOL[i].objs[0] < SOLVER_POOL[idx_E1].objs[0]) idx_E1 = i;
                }

                double E0_z1 = SOLVER_POOL[idx_E0].objs[0];
                double E0_z2 = SOLVER_POOL[idx_E0].objs[1];
                double E1_z1 = SOLVER_POOL[idx_E1].objs[0];
                double E1_z2 = SOLVER_POOL[idx_E1].objs[1];

                double v1 = E1_z1 - E0_z1;
                double v2 = E1_z2 - E0_z2;
                double v_norm_sq = v1 * v1 + v2 * v2;

                // Fallback de colapso: Reta nula (Fronteira com apenas 1 ponto válido)
                if (v_norm_sq < 1e-9) {
                    if (s.ofv < SOLVER_POOL.back().ofv) {
                        SOLVER_POOL.back() = s;
                        std::ranges::sort(SOLVER_POOL, [](const auto& a, const auto& b) { return a.ofv < b.ofv; });
                    }
                } 
                // Fluxo Normal do Método Espacial
                else {
                    auto calc_signature = [&](double z1, double z2, double& t, double& d, int& R) {
                        double u1 = z1 - E0_z1;
                        double u2 = z2 - E0_z2;
                        
                        t = (u1 * v1 + u2 * v2) / v_norm_sq;
                        t = std::clamp(t, 0.0, 1.0);
                        
                        double p1 = E0_z1 + t * v1;
                        double p2 = E0_z2 + t * v2;
                        
                        d = std::sqrt((z1 - p1)*(z1 - p1) + (z2 - p2)*(z2 - p2));
                        
                        R = static_cast<int>(t * K);
                        if (R >= K) R = K - 1; 
                    };

                    std::vector<double> pool_t(pool_size), pool_d(pool_size);
                    std::vector<int> pool_R(pool_size);
                    std::vector<int> count_R(K, 0);

                    for (int i = 0; i < pool_size; i++) {
                        calc_signature(SOLVER_POOL[i].objs[0], SOLVER_POOL[i].objs[1], pool_t[i], pool_d[i], pool_R[i]);
                        count_R[pool_R[i]]++;
                    }

                    double s_t, s_d;
                    int s_R;
                    calc_signature(s.objs[0], s.objs[1], s_t, s_d, s_R);

                    bool inserted = false;

                    if (count_R[s_R] >= capacity) {
                        int worst_idx = -1;
                        double max_d = -1.0;
                        for (int i = 0; i < pool_size; i++) {
                            if (pool_R[i] == s_R && pool_d[i] > max_d) {
                                max_d = pool_d[i];
                                worst_idx = i;
                            }
                        }
                        if (worst_idx != -1 && s_d < max_d) {
                            SOLVER_POOL[worst_idx] = s;
                            inserted = true;
                        }
                    } else {
                        int max_count = -1;
                        int crowded_R = -1;
                        for (int r = 0; r < K; r++) {
                            if (r != s_R && count_R[r] > max_count) {
                                max_count = count_R[r];
                                crowded_R = r;
                            }
                        }
                        if (crowded_R != -1) {
                            int worst_idx = -1;
                            double max_d = -1.0;
                            for (int i = 0; i < pool_size; i++) {
                                if (pool_R[i] == crowded_R && pool_d[i] > max_d) {
                                    max_d = pool_d[i];
                                    worst_idx = i;
                                }
                            }
                            if (worst_idx != -1) {
                                SOLVER_POOL[worst_idx] = s;
                                inserted = true;
                            }
                        }
                    }

                    if (inserted) {
                        std::ranges::sort(SOLVER_POOL, [](const auto& a, const auto& b) { return a.ofv < b.ofv; });
                        if (debug && s.ofv < SOLVER_POOL[0].ofv) {
                            std::cout << std::format("\nNovo recorde via Memoria Espacial! (T-{} | MH: {})", omp_get_thread_num(), mh);
                        }
                    }
                } // Fim Else Fluxo Normal
            }
        } // Fim IF (!exists && !is_dominated)
    } // Fim IF (pool_size > 0)
  } // Fim #pragma omp critical(pool_lock) - Tudo sai por aqui com segurança!
}

// void UpdatePoolSolutions(TSol s, const char *mh, const int debug) {
//   #pragma omp critical
//   {
//     int pool_size = static_cast<int>(SOLVER_POOL.size());
    
//     // Only proceed if the pool is initialized
//     if (pool_size > 0) {
        
//         // 1. Check if the solution already exists in the pool (exact fitness match)
//         bool exists = std::ranges::any_of(SOLVER_POOL, [&](const auto &entry) {
//             return std::abs(entry.ofv - s.ofv) < 1e-9;
//         });

//         // 2. Entry Filter: Only accept if it doesn't exist AND is strictly better than the worst
//         if (!exists && s.ofv < SOLVER_POOL.back().ofv) {
            
//             // The solution is accepted! Update its metadata.
//             s.best_time = get_time_in_seconds();
//             s.nameMH = mh;

//             // Log globally if a new overall best solution is found
//             if (s.ofv < SOLVER_POOL[0].ofv && debug) {
//                 int thread_id = omp_get_thread_num();
//                 std::cout << std::format("\nBest solution: {:.10f} (Thread: {} - MH: {})",
//                                          s.ofv, thread_id, mh);
//             }

//             // 3. Diversity Strategy (Cosine Similarity Replacement)
//             int start_idx = std::max(1, pool_size / 2); 
//             int target_idx = pool_size - 1; // Default victim is the absolute worst
//             double max_sim = -2.0;          

//             for (int i = start_idx; i < pool_size; i++) {
//                 // Extra safeguard: the victim MUST have a worse objective function value
//                 if (SOLVER_POOL[i].ofv > s.ofv) {
//                     double sim = rkolib::core::CosineSimilarity(s.rk, SOLVER_POOL[i].rk);
                    
//                     if (sim > max_sim) {
//                         max_sim = sim;
//                         target_idx = i;
//                     }
//                 }
//             }

//             // 4. Perform the tactical replacement
//             SOLVER_POOL[target_idx] = s;

//             // 5. Re-sort the entire pool to maintain the ordering invariant
//             std::ranges::sort(SOLVER_POOL, [](const TSol& a, const TSol& b) {
//                 return a.ofv < b.ofv; // Ascending order
//             });
            
//         } // End of conditional entry
//     } // End of pool_size check
//   } // End of #pragma omp critical (All threads exit here cleanly, unlocking the mutex)
// }

// void UpdatePoolSolutions(TSol s, const char *mh, const int debug) {
// #pragma omp critical
//   {
//     // Checks if it already exists in the SOLVER_POOL
//     bool exists = std::ranges::any_of(SOLVER_POOL, [&](const auto &entry) {
//       return std::abs(entry.ofv - s.ofv) < 1e-9;
//     });

//     // print that a new best solution was found
//     if (!SOLVER_POOL.empty() && s.ofv < SOLVER_POOL[0].ofv && debug) {
//       int thread_id = omp_get_thread_num();
//       std::cout << std::format("\nBest solution: {:.10f} (Thread: {} - MH: {})",
//                                s.ofv, thread_id, mh);
//     }

//     // Goes from back to front
//     if (!exists) {
//       s.best_time = get_time_in_seconds();
//       s.nameMH = mh; // std::string assignment

//       // insert the new solution preserving order (Insertion Sort logic)
//       int i;
//       for (i = static_cast<int>(SOLVER_POOL.size()) - 1;
//            i > 0 && SOLVER_POOL[i - 1].ofv > s.ofv; i--) {
//         SOLVER_POOL[i] = SOLVER_POOL[i - 1]; // Push to the right
//       }
//       SOLVER_POOL[i] = s;
//     }
//   }
// }

// -----------------------------------------------------------------------------
// Local Searches & Components
// -----------------------------------------------------------------------------

void ShakeSolution(TSol &s, float betaMin, float betaMax, const int n) {
  int intensity = (int)(n * randomico(betaMin, betaMax)) + 1;
  if (intensity < 1)
    intensity = 1;

  for (int k = 0; k < intensity; k++) {
    int shaking_type = irandomico(1, 4);
    int i = irandomico(0, n - 1);

    if (shaking_type == 1) {
      s.rk[i] = randomico(0, 1);
    } else if (shaking_type == 2) {
      if (s.rk[i] > 0.0001)
        s.rk[i] = 1.0 - s.rk[i];
      else
        s.rk[i] = 0.9999;
    } else if (shaking_type == 3) {
      int j = irandomico(0, n - 1);
      std::swap(s.rk[i], s.rk[j]);
    }

    if (shaking_type == 4 && n > 1) {
      // Swap with neighbor (cuidado com limites)
      int idx = irandomico(0, n - 2);
      std::swap(s.rk[idx], s.rk[idx + 1]);
    }
  }
}

TSol Blending(TSol &s1, TSol &s2, double factor, const int n) {
  TSol s;
  s.rk.resize(n);

  // =================================================================
  // VACINA: Proteção contra vetores vazios (Slots vazios do Pool)
  // Se qualquer uma das soluções de entrada estiver malformada, 
  // nós ignoramos a mistura e geramos chaves aleatórias seguras.
  // =================================================================
  if (s1.rk.size() < static_cast<size_t>(n) || s2.rk.size() < static_cast<size_t>(n)) {
      for (int j = 0; j < n; j++) {
          s.rk[j] = randomico(0, 1);
      }
      return s;
  }

  for (int j = 0; j < n; j++) {
    double value;
    if (randomico(0, 1) < 0.02) { // mutation
      value = randomico(0, 1);
    } else if (randomico(0, 1) < 0.5) {
      value = s1.rk[j];
      continue;
    } else if (factor == -1) { // Nelder-Mead reflection logic implicitly
      value = std::clamp(1.0 - s2.rk[j], 0.0, 0.9999999);
    } else {
      value = s2.rk[j];
    }
    s.rk[j] = value;
  }
  return s;
}

void NelderMeadSearch(TSol &x1, rkolib::RkoSolver &solver) {
  TSol x2, x3;
  int poolSize = 0;

  // =================================================================
  // VACINA 1: LEITURA PROTEGIDA DO POOL
  // =================================================================
  #pragma omp critical
  {
    poolSize = (int)SOLVER_POOL.size();
    
    // Só sorteia e faz a cópia se o pool for seguro
    if (poolSize >= 2) {
      int k1, k2;
      do {
        k1 = irandomico(0, poolSize - 1);
        k2 = irandomico(0, poolSize - 1);
      } while (k1 == k2);

      // Cópia profunda e 100% protegida das soluções do Pool
      x2 = SOLVER_POOL[k1];
      x3 = SOLVER_POOL[k2];
    }
  }

  // Se o pool for pequeno demais, aborta a busca local em segurança
  if (poolSize < 2) return;

  TSol x1Origem = x1;
  TSol xBest = x1;

  bool improved = 0;
  bool improvedX1 = 0; // Tracks if x1 was improved

  // Helper: Evaluates a candidate solution and updates xBest if necessary
  auto decodeAndUpdate = [&](TSol &candidate) {
    solver.decodeSolution(candidate);
    if (candidate.ofv < xBest.ofv) {
      xBest = candidate;
      improved = true;
      improvedX1 = true;
    }
  };

  auto sortSimplex = [](TSol &a, TSol &b, TSol &c) {
    if (a.ofv > b.ofv)
      std::swap(a, b);
    if (a.ofv > c.ofv)
      std::swap(a, c);
    if (b.ofv > c.ofv)
      std::swap(b, c);
  };

  // // Random selection of x2 and x3
  // int k1, k2;
  // do {
  //   k1 = irandomico(0, poolSize - 1);
  //   k2 = irandomico(0, poolSize - 1);
  // } while (k1 == k2);

  // TSol x2 = SOLVER_POOL[k1];
  // TSol x3 = SOLVER_POOL[k2];

  // Sort x1, x2, x3
  sortSimplex(x1, x2, x3);

  // Centroid
  TSol x0 = Blending(x1, x2, 1, solver.getProblemDimension());
  decodeAndUpdate(x0);

  int iter_count = 1;
  int maxIter = std::max(
      10, static_cast<int>(solver.getProblemDimension() * std::exp(-2)));

  while (iter_count <= maxIter) {
    bool shrink = false;

    // Reflection
    TSol x_r = Blending(x0, x3, -1, solver.getProblemDimension());
    decodeAndUpdate(x_r);

    if (x_r.ofv < x1.ofv) {
      // Expansion
      TSol x_e = Blending(x_r, x0, -1, solver.getProblemDimension());
      decodeAndUpdate(x_e);
      x3 = (x_e.ofv < x_r.ofv) ? x_e : x_r;
    } else if (x_r.ofv < x2.ofv) {
      x3 = x_r;
    } else {
      bool outsite = x_r.ofv < x3.ofv;
      TSol x_c = outsite ? Blending(x_r, x0, 1, solver.getProblemDimension())
                         : Blending(x0, x3, 1, solver.getProblemDimension());
      decodeAndUpdate(x_c);
      if (x_c.ofv < (outsite ? x_r.ofv : x3.ofv)) {
        x3 = x_c;
      } else {
        shrink = true;
      }
    }

    if (shrink) {
      x2 = Blending(x1, x2, 1, solver.getProblemDimension());
      decodeAndUpdate(x2);

      x3 = Blending(x1, x3, 1, solver.getProblemDimension());
      decodeAndUpdate(x3);
    }

    // Sort again
    sortSimplex(x1, x2, x3);

    x0 = Blending(x1, x2, 1, solver.getProblemDimension());
    decodeAndUpdate(x0);

    if (improved) {
      improved = 0;
      iter_count = 0;
    } else {
      iter_count++;
    }

    if (SOLVER_SHOULD_STOP)
      return;
  }

  x1 = improvedX1 ? xBest : x1Origem;
}

void runFirstImprovement(rkolib::core::TSol &s, rkolib::core::TSol &sBest,
                         rkolib::RkoSolver &solver,
                         const std::vector<int> &RKorder, int limitI,
                         int limitJ) {
  for (int i = 0; i < limitI; ++i) {
    for (int j = i + 1; j < limitJ; ++j) {
      std::swap(s.rk[RKorder[i]], s.rk[RKorder[j]]);
      solver.decodeSolution(s);

      if (s.ofv < sBest.ofv) {
        sBest = s;
      } else {
        s = sBest; // Reverte
      }

      if (SOLVER_SHOULD_STOP)
        return;
    }
  }
}

void runBestImprovement(rkolib::core::TSol &s, rkolib::core::TSol &sBest,
                        rkolib::RkoSolver &solver,
                        const std::vector<int> &RKorder, int limitI, int n) {
  bool improved = false;
  rkolib::core::TSol sCurrent = s;

  for (int i = 0; i < limitI; ++i) {
    for (int j = i + 1; j < n; ++j) {
      std::swap(s.rk[RKorder[i]], s.rk[RKorder[j]]);
      solver.decodeSolution(s);

      if (s.ofv < sBest.ofv) {
        sBest = s;
        improved = true;
      }
      s = sCurrent; // Always revert to test the next neighbot

      if (SOLVER_SHOULD_STOP)
        return;
    }
  }
  if (improved)
    s = sBest;
}

void SwapLS(TSol &s, rkolib::RkoSolver &solver, const int &strategy,
            std::vector<int> &RKorder) {
  std::shuffle(RKorder.begin(), RKorder.end(), SOLVER_RNG);
  rkolib::core::TSol sBest = s;
  const float rate = 1.0f;
  const int n = static_cast<int>(solver.getProblemDimension());
  const int limitI = static_cast<int>((n - 1) * rate);
  const int limitJ = static_cast<int>(n * rate);

  if (strategy == 1) {
    runFirstImprovement(s, sBest, solver, RKorder, limitI, limitJ);
  } else if (strategy == 2) {
    runBestImprovement(s, sBest, solver, RKorder, limitI, n);
  }
}

void InvertLS(TSol &s, rkolib::RkoSolver &solver, const int &strategy,
              std::vector<int> &RKorder) {
  std::shuffle(RKorder.begin(), RKorder.end(), SOLVER_RNG);
  float rate = 1.0;
  int limit = (int)(solver.getProblemDimension() * rate);

  TSol sBest = s;
  TSol sCurrent = s;
  bool improved = false;

  for (int i = 0; i < limit; i++) {
    // invert logic
    if (s.rk[RKorder[i]] > 0.00001)
      s.rk[RKorder[i]] = 1.0 - s.rk[RKorder[i]];
    else
      s.rk[RKorder[i]] = 0.99999;

    solver.decodeSolution(s);

    if (s.ofv < sBest.ofv) {
      sBest = s;
      if (strategy == 1) {
        // s = sBest already true here
      } else {
        improved = true;
      }
    } else {
      if (strategy == 1)
        s = sBest; // Reverts if first impr failed
    }

    if (strategy == 2)
      s = sCurrent; // Always reverts in best impr
    if (SOLVER_SHOULD_STOP)
      return;
  }

  if (strategy == 2 && improved)
    s = sBest;
}

void FareyLS(TSol &s, rkolib::RkoSolver &solver, const int &strategy,
             std::vector<int> &RKorder) {
  std::shuffle(RKorder.begin(), RKorder.end(), SOLVER_RNG);

  static const std::vector<double> F = {
      0.00, 0.142857, 0.166667, 0.20,     0.25, 0.285714, 0.333333,
      0.40, 0.428571, 0.50,     0.571429, 0.60, 0.666667, 0.714286,
      0.75, 0.80,     0.833333, 0.857143, 1.0};

  float rate = 1.0;
  int limit = static_cast<int>(solver.getProblemDimension() * rate);

  TSol sBest = s;
  TSol sCurrent = s;
  bool improved = false;

  for (int i = 0; i < limit; i++) {
    for (size_t j = 0; j < F.size() - 1; j++) {
      s.rk[RKorder[i]] = randomico(F[j], F[j + 1]);
      solver.decodeSolution(s);

      if (s.ofv < sBest.ofv) {
        sBest = s;
        if (strategy == 2)
          improved = true;
      } else {
        if (strategy == 1)
          s = sBest;
      }

      if (strategy == 2)
        s = sCurrent;
      if (SOLVER_SHOULD_STOP)
        return;
    }
  }
  if (strategy == 2 && improved)
    s = sBest;
}

void RVND(TSol &s, rkolib::RkoSolver &solver, const int &strategy,
          std::vector<int> &RKorder) {
  // Define the list of neighborhood structures
  const int numLS = 4;
  enum class LS { SWAP = 1, INVERT, NELDERMEAD, FAREY };

  std::vector<int> NSL;
  std::iota(NSL.begin(), NSL.end(), 1); // Push with values 1 to numLS

  while (!NSL.empty()) {
    if (SOLVER_SHOULD_STOP)
      return;

    double foCurrent = s.ofv;

    std::uniform_int_distribution<size_t> dist(0, numLS - 1);
    size_t pos = dist(SOLVER_RNG);
    auto k = (LS)NSL[pos];

    switch (k) {
    case LS::SWAP:
      SwapLS(s, solver, strategy, RKorder);
      break;
    case LS::INVERT:
      InvertLS(s, solver, strategy, RKorder);
      break;
    case LS::NELDERMEAD:
      NelderMeadSearch(s, solver);
      break;
    default:
      FareyLS(s, solver, strategy, RKorder);
      break;
    }

    if (s.ofv < foCurrent) {
      NSL.resize(numLS);
      std::iota(NSL.begin(), NSL.end(), 1); // Push with values 1 to numLS
    } else {
      NSL.erase(NSL.begin() + pos);
    }
  }
}

void readParameters(const std::string &method, int control,
                    std::vector<std::vector<double>> &parameters, int numPar) {
#pragma omp critical
  {
    std::string paramFile =
        (control == 0) ? "config/param-offline.txt" : "config/param-online.txt";

    std::ifstream file(paramFile);
    if (!file.is_open()) {
      std::cerr << "Error in open file " << paramFile << ".\n";
      std::exit(1);
    }

    std::string line;
    while (std::getline(file, line)) {
      if (line == method) {
        for (int i = 0; i < numPar; i++) {
          if (std::getline(file, line)) {
            // Remove characters: { } , = and spaces
            std::string cleaned;
            for (char c : line) {
              if (c != '{' && c != '}' && c != ',' && c != '=' && c != ' ') {
                cleaned += c;
              } else if (!cleaned.empty() && cleaned.back() != ' ') {
                cleaned += ' '; // Separates tokens with space
              }
            }

            std::istringstream iss(cleaned);
            std::string token;
            while (iss >> token) {
              double aux = 0;
              try {
                size_t pos;
                aux = std::stod(token, &pos);
                if (pos == token.size()) { // Complete conversion
                  parameters[i].push_back(aux);
                }
              } catch (const std::invalid_argument &) {
                // Token is not a valid number, ignore
              } catch (const std::out_of_range &) {
                // Value out of double range, ignore
              }
            }
          }
        }
      }
    }
  }
}

// void readParameters(const char*  method, int control,
//                 std::vector<std::vector<double>> &parameters, int numPar)
// {
//     #pragma omp critical
//     {
//         char paramFile[256];
//         if (control == 0){
//             strncpy(paramFile,"config/param-offline.txt",255);
//         }
//         else{
//             strncpy(paramFile,"config/param-online.txt",255);
//         }

//         FILE *file = fopen(paramFile, "r");
//         if (file == NULL) {
//             printf("Error in open file %s.\n", paramFile);
//             getchar();
//         }

//         char line[100];     // Buffer line

//         // Reading the file line by line
//         while (fgets(line, sizeof(line), file) != NULL) {
//             line[strcspn(line, "\n")] = '\0';  // remove newline

//             if (strcmp(line, method) == 0) {
//                 // Read the parameter values
//                 for (int i = 0; i < numPar; i++) {
//                     double aux = 0;
//                     if (fgets(line, sizeof(line), file) != NULL) {
//                         char *token = strtok(line, "{},= "); // separete line
//                         by delimiters while (token != NULL) {
//                             if (sscanf(token, "%lf", &aux) == 1) {
//                                 parameters[i].push_back(aux);
//                             }
//                             token = strtok(NULL, "{},= "); // next value
//                         }
//                     }
//                 }
//             }
//         }
//         fclose(file);
//     }
// }

void LoadYamlLogic(const std::string &paramFile, const char *method, int numPar,
                   std::vector<std::vector<double>> &parameters) {
  try {
    YAML::Node config;
    try {
      config = YAML::LoadFile(paramFile);
    } catch (const YAML::BadFile &) {
      std::string altFile = paramFile;
      if (altFile.rfind("../", 0) == 0) {
        altFile = altFile.substr(3); // Remove "../"
      } else {
        altFile = "../" + altFile;
      }
      config = YAML::LoadFile(altFile);
    }

    // Guard Clause 1: Method does not exist
    if (!config[method]) {
      std::cerr << "Method '" << method << "' not found in " << paramFile
                << std::endl;
      return; // NOW ALLOWED (exits the auxiliary function, not the critical
              // block directly)
    }

    const YAML::Node &methodNode = config[method];

    // Guard Clause 2: Invalid format
    if (!methodNode.IsSequence()) {
      std::cerr << "Invalid format for method: " << method
                << " (expected a list of lists)." << std::endl;
      return;
    }

    // Filling logic
    int limit = std::min(numPar, static_cast<int>(methodNode.size()));

    for (int i = 0; i < limit; i++) {
      if (!methodNode[i].IsSequence())
        continue;

      for (const auto &val : methodNode[i]) {
        parameters[i].push_back(val.as<double>());
      }
    }

  } catch (const YAML::BadFile &e) {
    std::cerr << "Error opening YAML file: " << paramFile << std::endl;
    exit(1); // Exit is allowed because it kills the entire process
  } catch (const YAML::ParserException &e) {
    std::cerr << "Syntax error in YAML: " << e.what() << std::endl;
    exit(1);
  } catch (const std::exception &e) {
    std::cerr << "Unknown error: " << e.what() << std::endl;
    exit(1);
  }
}

void readParametersYaml(const char *method, int control,
                        std::vector<std::vector<double>> &parameters,
                        int numPar) {
  // Define o nome do arquivo fora da área crítica (operação leve)
  std::string paramFile = (control == 0) ? "../config/yaml/param-offline.yaml"
                                         : "../config/yaml/param-online.yaml";

#pragma omp critical
  {
    // Calls the logic. If it returns, it goes back to this line,
    // reaches the closing brace '}' and releases the Lock correctly.
    LoadYamlLogic(paramFile, method, numPar, parameters);
  }
}

// void readParametersYaml(const char* method, int control,
//                 std::vector<std::vector<double>> &parameters, int numPar)
// {
//     #pragma omp critical
//     {
//         // 1. Define the file (now with .yaml extension)
//         std::string paramFile;
//         if (control == 0) paramFile = "config/yaml/param-offline.yaml";
//         else paramFile = "config/yaml/param-online.yaml";

//         try {

//             // 2. Loads the file using yaml-cpp
//             YAML::Node config = YAML::LoadFile(paramFile);

//             // 3. Verified if the method exists in the YAML
//             if (!config[method]) {
//                 std::cerr << "Método '" << method << "' não encontrado em "
//                 << paramFile << std::endl; return;
//             }

//             const YAML::Node& methodNode = config[method];

//             if (!methodNode.IsSequence()) {
//                 std::cerr << "Formato inválido para o método: " << method <<
//                 " (esperado uma lista de listas)." << std::endl; return;
//             }

//             int limit = std::min(numPar,
//             static_cast<int>(methodNode.size())); for (int i = 0; i < limit;
//             i++) {
//                 // Verified if the inner item is also a sequence (list of
//                 doubles) if (!methodNode[i].IsSequence()) continue;

//                 for (const auto& val : methodNode[i]) {
//                     parameters[i].push_back(val.as<double>());
//                 }
//             }

//         } catch (const YAML::BadFile& e) {
//             std::cerr << "Erro ao abrir o arquivo YAML: " << paramFile <<
//             std::endl; exit(1);
//         } catch (const YAML::ParserException& e) {
//             std::cerr << "Erro de sintaxe no YAML: " << e.what() <<
//             std::endl; exit(1);
//         } catch (const std::exception& e) {
//             std::cerr << "Erro desconhecido: " << e.what() << std::endl;
//             exit(1);
//         }
//     }
// }

} // namespace rkolib::core