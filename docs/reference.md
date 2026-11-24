# API & Component Reference (`rkolib`)

This document provides a technical reference for the core classes, data structures, plugin interfaces, scalarizers, Q-Learning routines, IO utilities, and metaheuristics implemented in **RKOSolver** (`rkolib`).

---

## 1. Core Architecture

### `rkolib::RkoSolver`
*Header: [`include/rkolib/core/solver.hpp`](https://github.com/zefilho/rko-solver/blob/main/include/rkolib/core/solver.hpp)*

Central orchestrator class responsible for CLI parsing, YAML configuration loading, dynamic plugin loading, reference point initialization, parallel OpenMP execution, and statistical output.

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

Thread-safe Singleton managing shared solver state across OpenMP threads: the elite solution pool (`SOLVER_POOL`), 64-byte cache-line-aligned per-thread RNGs (`PaddedRng` / `SOLVER_RNG`), run start timestamps, and cooperative stop flags (`SOLVER_SHOULD_STOP`).

#### Key Public Methods
```cpp
// Returns singleton instance
static SolverContext &instance();

// Elite Solution Pool Access (protect with #pragma omp critical(pool_lock))
std::vector<TSol> &getPool();
void initializePool(size_t size);
TSol getBestSolution();

// Run timing helpers (used for relative best_time tracking)
void setRunStartTime(double startTime);
double getRunStartTime() const;

// OpenMP Synchronization & Stop Control
void signalStop();
bool isStopped() const;
void resetStopFlag();

// Thread-Safe Cache-Aligned Random Number Generator (PaddedRng)
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
  int sizePool;              // Base elite solution pool size
  int poolUpdateMethod = 0;  // Pool update policy (0 = Cosine Diversity, 1 = NSGA-II)
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
| **`NormalizedWeightedTchebycheff`** | `"Tchebycheff"` | $\max_k d_k + \rho \sum_k \frac{f_k(x) - z^*_k}{z^{\text{nad}}_k - z^*_k}$ |
| **`WeightedSumScalarizer`** | `"WeightedSum"` | $\sum_k w_k \cdot f_k(x)$ |
| **`GiniScalarizer`** | `"Gini_Coefficient"` | $\bar{d} \cdot (1 + G(d)) = \bar{d} + \frac{1}{2 n^2} \sum_i \sum_j \|d_i - d_j\|$ over normalized regrets $d_k \ge 0$ |
| **`LexicographicMinMaxScalarizer`** | `"Lexicographic_MinMax"` | $\sum_{k=1}^{n} \gamma^{k-1} d_{(k)}$ with sorted $d_{(1)} \ge \dots \ge d_{(n)}$ and `lexicographic_decay` $\gamma$ |

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
- **`UpdatePoolSolutions(sol, method, debug, poolUpdateMethod)`**: Thread-safe (`pool_lock`) elite pool update supporting Cosine Similarity diversity (`0`) and NSGA-II (`1`).
- **`RVND(s, solver, strategy, RKorder)`**: Randomized Variable Neighborhood Descent over `SwapLS`, `InvertLS`, and `FareyLS`.
- **`NelderMeadSearch(x1, solver)`**: Simplex search over random keys using thread-safe pool sampling (`pool_lock`).

### IO Utilities (`rkolib::utils`)
- **`PrintPoolSolutions(pool, dimension)`**: Prints the formatted elite pool summary (`OFV`, `MH`, `Best time`, `Objs`, `RK`) to stdout.
- **`WritePoolSolutions(algorithms, pool, instance, dimension, outDir = "results")`**: Appends the full elite pool to `results/Pool_Solutions_RKO.txt`.

---

## 7. Integrated Metaheuristics

All metaheuristics conform to `std::function<void(const core::TRunData &, RkoSolver &)>`:

| Metaheuristic | Description | Source |
| :--- | :--- | :--- |
| **`SA`** | Simulated Annealing with geometric cooling, Q-Learning, and time-gated Nelder-Mead | [`src/mh/sa.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/sa.cpp) |
| **`ILS`** | Iterated Local Search with `RVND`, Q-Learning, and time-gated Nelder-Mead | [`src/mh/ils.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/ils.cpp) |
| **`VNS`** | Variable Neighborhood Search with `RVND`, Q-Learning, and time-gated Nelder-Mead | [`src/mh/vns.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/vns.cpp) |
| **`BRKGA`** | Biased Random-Key Genetic Algorithm with Q-Learning and time-gated Nelder-Mead | [`src/mh/brkga.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/brkga.cpp) |
| **`BRKGA-CS`** | BRKGA with Community Search (`RVND`), Q-Learning, and time-gated Nelder-Mead | [`src/mh/brkga_cs.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/brkga_cs.cpp) |
| **`PSO`** | Particle Swarm Optimization with Q-Learning and time-gated Nelder-Mead | [`src/mh/pso.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/pso.cpp) |
| **`GA`** | Genetic Algorithm with tournament selection, Q-Learning, and time-gated Nelder-Mead | [`src/mh/ga.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/ga.cpp) |
| **`LNS`** | Large Neighborhood Search with Farey repair, Q-Learning, and time-gated Nelder-Mead | [`src/mh/lns.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/lns.cpp) |
| **`GRASP`** | Greedy Randomized Adaptive Search with `RVND`, Q-Learning, and time-gated Nelder-Mead | [`src/mh/grasp.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/grasp.cpp) |
| **`IPR`** | Implicit Path-Relinking with 6 strategies (`One-way`, `Back-and-Forth`, `Mixed`, `GRAPR`, `Truncated`, `External`) and 18-state Q-Learning | [`src/mh/ipr.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/ipr.cpp) |
| **`MultiStart`** | Multi-start random-key sampling baseline | [`src/mh/multistart.cpp`](https://github.com/zefilho/rko-solver/blob/main/src/mh/multistart.cpp) |
