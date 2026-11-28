# API & Component Reference (`rkolib`)

This document provides a technical reference for the core classes, data structures, plugin interfaces, scalarizers, Q-Learning routines, IO utilities, and all 15 metaheuristics implemented in **RKOSolver** (`rkolib`).

---

## 1. Core Architecture

### `rkolib::RkoSolver`
*Header: [`include/rkolib/core/solver.hpp`](https://github.com/zefilho/rko-solver/blob/main/include/rkolib/core/solver.hpp)*

Central orchestrator class responsible for CLI parsing, YAML configuration loading, dynamic plugin loading, reference point initialization and synchronization, parallel OpenMP execution, and statistical output.

#### Key Public Methods
```cpp
// Parses CLI flags (-i/--instance, -c/--config, -t/--time, -p/--plugin)
void parseArguments(int argc, char *argv[]);

// Loads configuration parameters from a YAML file
void loadConfiguration(const std::string &configFile = "");

// Executes all MAXRUNS optimization runs
void run();

// Decodes a random-key solution using the active problem plugin and scalarizer
void decodeSolution(core::TSol &sol, const std::vector<double> &lambda = {});

// Retrieves a thread-safe copy of the best solution found so far
core::TSol getBestSolution() const;

// Returns current Ideal and Nadir reference points
const std::vector<double> &getIdealPoint() const;
const std::vector<double> &getNadirPoint() const;
```

---

### `rkolib::core::SolverContext`
*Header: [`include/rkolib/core/context.hpp`](https://github.com/zefilho/rko-solver/blob/main/include/rkolib/core/context.hpp)*

Thread-safe Singleton managing shared solver state across OpenMP threads: the elite solution pool (`SOLVER_POOL`), 64-byte cache-line-aligned per-thread RNGs (`PaddedRng` / `SOLVER_RNG`), run start timestamps, anti-drift pool re-scalarization, and cooperative stop flags (`SOLVER_SHOULD_STOP`).

#### Key Public Methods
```cpp
// Returns singleton instance
static SolverContext &instance();

// Elite Solution Pool Access & Re-Scalarization (protect with #pragma omp critical(pool_lock))
std::vector<TSol> &getPool();
const std::vector<TSol> &getPool() const;
void initializePool(size_t size);
void clearPool();
void sortPool();
void rescalarizePool(IScalarizer &scalarizer,
                     const std::vector<double> &lambda,
                     const std::vector<double> &idealPoint,
                     const std::vector<double> &nadirPoint);
TSol getBestSolution() const;

// Run timing helpers (used for relative best_time tracking)
void setRunStartTime(double t);
double getRunStartTime() const;

// OpenMP Synchronization & Stop Control
void signalStop();
bool shouldStop() const;
void resetStopFlag();

// Thread-Safe Cache-Aligned Random Number Generator (PaddedRng, prime-strided seeds)
std::mt19937 &getRng();
void setSeed(unsigned int baseSeed, int min_threads = 32);
```

---

## 2. Problem Plugin Interface

### `rkolib::core::IProblem`
*Header: [`include/rkolib/core/problem.hpp`](https://github.com/zefilho/rko-solver/blob/main/include/rkolib/core/problem.hpp)*

Abstract base interface implemented by all dynamically loaded problem plugins.

```cpp
namespace rkolib::core {
class IProblem {
public:
  virtual ~IProblem() = default;

  // Loads instance data from file
  virtual void load(const std::string &filename) = 0;

  // Transforms random-key vector (sol.rk) into a solution and populates sol.objs
  virtual void decode(TSol &sol) const = 0;

  // Returns number of random keys required by the decoder
  virtual int getDimension() const = 0;

  // Returns number of target objectives (1 = Single-Objective, >1 = Multi-Objective)
  virtual int getNumObjectives() const = 0;

  // Sets debug mode (0: silent, 1: detailed terminal output)
  virtual void setDebugMode(int debug) { (void)debug; }
};
}
```

### Plugin Registration Macro: `REGISTER_RKO_PROBLEM`
Exports `create_problem` and `destroy_problem` with C linkage for `dlopen`/`dlsym`:

```cpp
#define REGISTER_RKO_PROBLEM(ProblemClass) \
  extern "C" { \
    EXPORT_PLUGIN rkolib::core::IProblem *create_problem() { \
      return new ProblemClass(); \
    } \
    EXPORT_PLUGIN void destroy_problem(rkolib::core::IProblem *p) { \
      delete p; \
    } \
  }
```

---

## 3. Data Structures

### `rkolib::core::TSol`
*Header: [`include/rkolib/core/data.hpp`](https://github.com/zefilho/rko-solver/blob/main/include/rkolib/core/data.hpp)*

```cpp
struct TSol {
  std::vector<double> rk;                               // Random-key vector in [0, 1)
  double ofv = std::numeric_limits<double>::infinity(); // Scalarized Objective Function Value
  double best_time = 0.0;                               // Relative time (s) when discovered
  std::string nameMH;                                   // Metaheuristic that found this solution
  std::vector<double> objs;                             // Raw objective values vector
  TSol() = default;
};
```

### `rkolib::core::TRunData`
*Header: [`include/rkolib/core/data.hpp`](https://github.com/zefilho/rko-solver/blob/main/include/rkolib/core/data.hpp)*

```cpp
struct TRunData {
  int strategy;              // Local search strategy (1 = First Improvement, 2 = Best Improvement)
  int control;               // Parameter control mode (0 = Offline Tuning, 1 = Online Q-Learning)
  int MAXTIME;               // Maximum execution time in seconds
  int MAXRUNS;               // Maximum number of independent runs
  int debug;                 // Debug output level (0 = File output, 1 = Terminal logs)
  float restart;             // Restart threshold multiplier
  int sizePool;              // Base elite solution pool size (doubled when M > 1)
  int poolUpdateMethod = 0;  // Pool policy (0 = Cosine Diversity, 1 = NSGA-II, 2 = Spatial Niche Geometry)
  float tauMinNM = 0.005f;   // Minimum fraction of MAXTIME between Nelder-Mead calls
  float tauMaxNM = 0.20f;    // Maximum fraction of MAXTIME without a Nelder-Mead call
};
```

---

## 4. Multi-Objective Scalarizers

### `rkolib::core::IScalarizer`
*Header: [`include/rkolib/core/scalarizer.hpp`](https://github.com/zefilho/rko-solver/blob/main/include/rkolib/core/scalarizer.hpp)*

```cpp
class IScalarizer {
public:
  virtual ~IScalarizer() = default;
  virtual double scalarize(const TSol &s, const std::vector<double> &lambda,
                           const std::vector<double> &idealPoint,
                           const std::vector<double> &nadirPoint) = 0;
  virtual std::string getName() const = 0;
};
```

### Implementations

| Class | Config Key (`scalarization`) | Formula |
| :--- | :--- | :--- |
| **`NormalizedWeightedTchebycheff`** | `"Tchebycheff"` | $\max_k d_k + 10^{-4} \sum_k \frac{\max(0, f_k(x) - z^*_k)}{z^{\text{nad}}_k - z^*_k}$ |
| **`WeightedSumScalarizer`** | `"WeightedSum"` | $\sum_k w_k \cdot f_k(x)$ |
| **`GiniScalarizer`** | `"Gini_Coefficient"` | $\bar{d} \cdot (1 + G(d)) = \bar{d} + \frac{1}{2 M^2} \sum_i \sum_j \|d_i - d_j\|$ over normalized regrets $d_k \ge 0$ |
| **`LexicographicMinMaxScalarizer`** | `"Lexicographic_MinMax"` | $\sum_{r=0}^{M-1} \gamma^{r} d_{(r)} + \frac{10^{-6}}{M}\sum_{r=1}^{M-1} d_{(r)}$ with sorted $d_{(0)} \ge \dots \ge d_{(M-1)}$ |

---

## 5. Q-Learning Control Engine

*Header: [`include/rkolib/core/qlearning.hpp`](https://github.com/zefilho/rko-solver/blob/main/include/rkolib/core/qlearning.hpp)*  
*Implementation: [`src/core/qlearning.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/core/qlearning.cpp)*

```cpp
namespace rkolib::core {

// Builds the Cartesian state space and O(|S| * m) M/D/A adjacent action topology
void CreateStates(const std::vector<std::vector<double>> &parameters,
                  int &numStates, int numPar, std::vector<TState> &S);

// Selects an action using an epsilon-greedy policy
int ChooseAction(const std::vector<TState> &S, int st, double epsilon);

// Computes capped relative improvement in [0, delta_max] (default delta_max = 0.1)
double ComputeDelta(double oldBest, double newBest, double delta_max = 0.1);

// Improvement-based epsilon decay with floor epsilon_min (default 0.1)
void SetQLParameter(double &epsilon, double &lf, double &df, int MAXTIME,
                    float currentTime, float delta, double epsilon_min = 0.1);

// Cosine annealing with warm restarts (epsilon_max passed by reference)
void SetQLParameter(float currentTime, int &Ti, int &restartEpsilon,
                    float &epsilon_max, float epsilon_min,
                    double &epsilon, double &lf, double &df, int MAXTIME);

// Recomputes maxQ and maxA for state st
void UpdateMaxQ(std::vector<TState> &S, int st);

// Applies Bellman update, conditionally calls UpdateMaxQ, and transitions st -> st_1
void UpdateQValue(std::vector<TState> &S, int &st, int at,
                  double R, double lf, double df);

// Thread-safe Q-Table persistence (results/QTable_<method>.txt)
bool SaveQTable(const std::vector<TState> &S, int st,
                const std::string &method, const std::string &outDir = "results");
bool LoadQTable(std::vector<TState> &S,
                const std::string &method, const std::string &outDir = "results");

void PrintPolicy(const std::vector<TState> &S, int st);

} // namespace rkolib::core
```

---

## 6. Local Search, Pool Management & IO

*Headers: [`include/rkolib/core/method.hpp`](https://github.com/zefilho/rko-solver/blob/main/include/rkolib/core/method.hpp), [`include/rkolib/utils/io.hpp`](https://github.com/zefilho/rko-solver/blob/main/include/rkolib/utils/io.hpp)*

### Core Search & Pool Functions (`rkolib::core`)
- **`CreatePoolSolutions(solver, sizePool)`**: Two-pass pool initialization ensuring consolidated Ideal/Nadir reference points and clone mutation.
- **`UpdatePoolSolutions(sol, method, debug, poolUpdateMethod)`**: Thread-safe (`pool_lock`) elite pool update supporting Cosine Similarity diversity (`0`), NSGA-II (`1`), and Spatial Frontier Niche Geometry (`2`).
- **`RVND(s, solver, strategy, RKorder)`**: Randomized Variable Neighborhood Descent over `SwapLS`, `InvertLS`, and `FareyLS`.
- **`NelderMeadSearch(x1, solver)`**: Simplex search over random keys using thread-safe pool sampling (`pool_lock`).

### IO Utilities (`rkolib::utils`)
- **`WriteResults(...)` / `WriteMultiObjectiveResults(...)`**: Appends single-objective (`Results_RKO.csv`) or multi-objective (`Results_MO_RKO.csv`) run statistics.
- **`WriteSolution(...)` / `WriteSolutionScreen(...)`**: Logs detailed global best solution and terminal summary.
- **`PrintPoolSolutions(pool, dimension)`**: Prints the formatted elite pool summary (`OFV`, `MH`, `Best time`, `Objs`, `RK`) to stdout.
- **`WritePoolSolutions(algorithms, pool, instance, dimension, outDir = "results")`**: Appends the full elite pool to `results/Pool_Solutions_RKO.txt`.
- **`WriteConvergenceLog(history, outDir = "results")`**: Exports timestamped incumbent improvement points to `results/Convergence_Log.csv`.

---

## 7. Registered Metaheuristics (15 Total)

All metaheuristics conform to `std::function<void(const core::TRunData &, RkoSolver &)>` (see [Metaheuristics Catalog](metaheuristics.md) for mathematical details):

| Metaheuristic | Description | Source |
| :--- | :--- | :--- |
| **`SA`** | Simulated Annealing ($T > 10^{-5}$, $0.5 \, T_0$ reannealing, pre-reward `NelderMeadSearch(s)`, Q-Learning) | [`src/mh/sa.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/sa.cpp) |
| **`ILS`** | Iterated Local Search (`sCurrent`/`sBest`, `ILS_ACCEPT_RESTART`, `RVND`, `NelderMeadSearch(sCurrent)`, binary Q-Learning) | [`src/mh/ils.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/ils.cpp) |
| **`EILS`** | Enhanced Iterated Local Search (5 perturbation levels, `Nmax` failures, `RVND`, `NelderMeadSearch`, cycle-based Q-Learning) | [`src/mh/eils.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/eils.cpp) |
| **`VNS`** | Variable Neighborhood Search (best-of-10 init, `RVND`, inner-loop `NelderMeadSearch(s)`, Q-Learning) | [`src/mh/vns.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/vns.cpp) |
| **`GRASP`** | Continuous GRASP (bidirectional grid `LineSearch`, RCL, `RVND`, inner `NelderMeadSearch(s)`, 12-state Q-Learning) | [`src/mh/grasp.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/grasp.cpp) |
| **`LNS`** | Large Neighborhood Search (Farey $\mathcal{F}_7$ repair, SA cooling, `NelderMeadSearch(s)`, Q-Learning) | [`src/mh/lns.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/lns.cpp) |
| **`TS`** | Tabu Search (bidirectional tenure list, aspiration, diversification + `RVND`, `NelderMeadSearch`, Q-Learning) | [`src/mh/ts.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/ts.cpp) |
| **`BRKGA`** | Biased Random-Key Genetic Algorithm (hybrid `PUX` with fitness swap, Q-Learning, `NelderMeadSearch`) | [`src/mh/brkga.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/brkga.cpp) |
| **`BRKGA-CS`** | BRKGA with Community Search (Pearson LP clustering, `double rhoe` `ChaoticInd`, selective `RVND`, `NelderMeadSearch`) | [`src/mh/brkga_cs.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/brkga_cs.cpp) |
| **`PSO`** | Particle Swarm Optimization (Q-Learning, `NelderMeadSearch` with `Pbest` feedback injection) | [`src/mh/pso.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/pso.cpp) |
| **`GA`** | Genetic Algorithm (tournament selection, Q-Learning, `NelderMeadSearch` with `Pop` feedback injection) | [`src/mh/ga.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/ga.cpp) |
| **`DE`** | Differential Evolution (50/50 `rand/1` & `current-to-best/1`, toroidal wrap-around, 5% `RVND`, `NelderMeadSearch`, Q-Learning) | [`src/mh/de.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/de.cpp) |
| **`ACO`** | Continuous Ant Colony Optimization $\text{ACO}_{\mathbb{R}}$ (Gaussian archive, rejection sampling, $\varepsilon_{\min}=0.05$, `NelderMeadSearch`, Q-Learning) | [`src/mh/aco.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/aco.cpp) |
| **`IPR`** | Implicit Path-Relinking (contiguous blocks, 6 strategies `0..5`, 18-state online Q-Learning) | [`src/mh/ipr.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/ipr.cpp) |
| **`MultiStart`** | Parameter-free continuous random-key sampling baseline | [`src/mh/multistart.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/multistart.cpp) |
