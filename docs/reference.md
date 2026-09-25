# API & Component Reference (`rkolib`)

This document provides a comprehensive technical reference for the core components, data structures, plugin interfaces, scalarizers, and metaheuristics implemented in the **RKO Framework** (`rkolib`).

---

## 1. Core Architecture

### `rkolib::RkoSolver`
*Header: [`rkolib/core/solver.hpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/include/rkolib/core/solver.hpp)*

The central orchestrator class that manages CLI parsing, YAML configuration loading, reference point initialization, parallel OpenMP thread execution, and statistical aggregation.

#### Key Public Methods
```cpp
// Parses CLI flags (-i/--instance, -c/--config, -t/--time, -p/--plugin)
void parseArguments(int argc, char *argv[]);

// Loads configuration parameters from a YAML file
void loadConfiguration(const std::string &configFile = "");

// Executes all MAXRUNS optimization runs
void run();

// Decodes a random-key solution using the active problem plugin
void decodeSolution(core::TSol &sol, const std::vector<double> &lambda = {});

// Retrieves a thread-safe copy of the best solution found so far
core::TSol getBestSolution() const;

// Returns current reference points
const std::vector<double> &getIdealPoint() const;
const std::vector<double> &getNadirPoint() const;
```

---

### `rkolib::core::SolverContext`
*Header: [`rkolib/core/context.hpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/include/rkolib/core/context.hpp)*

A thread-safe Singleton that manages global search state across OpenMP threads, including the elite solution pool, thread-isolated Random Number Generators (`SOLVER_RNG`), thread seeds, and cooperative stop flags.

#### Key Public Methods
```cpp
// Returns singleton instance
static SolverContext &instance();

// Elite Solution Pool Access
std::vector<TSol> &getPool();
void initializePool(size_t size);
TSol getBestSolution();

// OpenMP Synchronization & Stop Control
void signalStop();
bool isStopped() const;
void resetStopFlag();

// Thread-Safe Random Number Generator
std::mt19937 &getRng();
void setSeed(unsigned int seed);
```

---

## 2. Problem Plugin Interface

### `rkolib::core::IProblem`
*Header: [`rkolib/core/problem.hpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/include/rkolib/core/problem.hpp)*

Abstract base interface that all problem plugins must inherit from to interface with the `rkosolver` engine.

```cpp
namespace rkolib::core {
class IProblem {
public:
  virtual ~IProblem() = default;

  // Loads instance data from file
  virtual void load(const std::string &filename) = 0;

  // Transforms random-key vector (sol.rk) into problem solution and populates raw objectives (sol.objs)
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
Export macro that registers the plugin factory functions (`create_problem` and `destroy_problem`) using C linkage for dynamic library loading via `dlopen`/`dlsym`:

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
*Header: [`rkolib/core/data.hpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/include/rkolib/core/data.hpp)*

Represents a single solution in the Random-Key Optimization domain.

```cpp
struct TSol {
  std::vector<double> rk;                        // Random-key vector in range [0, 1)
  double ofv = std::numeric_limits<double>::infinity(); // Scalarized Objective Function Value (Fitness)
  double best_time = 0.0;                        // Time (in seconds) when this solution was discovered
  std::string nameMH;                            // Name of the metaheuristic that found this solution
  std::vector<double> objs;                      // Raw objective values vector
  TSol() = default;
};
```

---

### `rkolib::core::TRunData`
*Header: [`rkolib/core/data.hpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/include/rkolib/core/data.hpp)*

Configuration data structure containing global search parameters.

```cpp
struct TRunData {
  int strategy;              // Local search strategy (1 = First Improvement, 2 = Best Improvement)
  int control;               // Parameter control mode (0 = Offline Tuning, 1 = Online Q-Learning)
  int MAXTIME;               // Maximum execution time in seconds
  int MAXRUNS;               // Maximum number of independent optimization runs
  int debug;                 // Debug output level (0 = File output, 1 = Terminal logs)
  float restart;             // Restart threshold multiplier
  int sizePool;              // Elite solution pool size
  int poolUpdateMethod = 0;  // Pool update policy (0 = Standard Cosine Diversity, 1 = NSGA-II)
};
```

---

## 4. Multi-Objective Scalarizers

### `rkolib::core::IScalarizer`
*Header: [`rkolib/core/scalarizer.hpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/include/rkolib/core/scalarizer.hpp)*

Abstract interface for multi-objective scalarization methods.

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

#### 1. `NormalizedWeightedTchebycheff`
Computes the normalized Tchebycheff distance to the ideal point $z^*$ with an augmented $\epsilon$-term:
$$F(x) = \max_{k} \left\{ w_k \cdot \frac{|z_k^* - f_k(x)|}{\text{nadir}_k - z_k^*} \right\} + \epsilon \sum_{k} \frac{|z_k^* - f_k(x)|}{\text{nadir}_k - z_k^*}$$

#### 2. `WeightedSumScalarizer`
Computes the classical weighted linear sum of objective values:
$$F(x) = \sum_{k} w_k \cdot f_k(x)$$

#### 3. `GiniScalarizer`
Minimizes equity inequality across objective components using the Gini Coefficient:
$$G = \frac{\sum_{i} \sum_{j} |f_i(x) - f_j(x)|}{2 n^2 \mu}$$

---

## 5. Q-Learning Control Engine

### `rkolib::core::QLearningControl`
*Header: [`rkolib/core/qlearning.hpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/include/rkolib/core/qlearning.hpp)*

Reinforcement learning engine that adaptively tunes search parameters during execution.

- **Quality Matrix $Q(s, a)$**: Stores quality values for discrete state-action pairs.
- **State Definition ($s$)**: Discretized performance state based on objective improvements.
- **Reward Function ($R$)**: Computed based on relative percentage fitness improvement $\Delta f / f_{\text{best}}$.
- **Action Selection ($a$)**: $\epsilon$-greedy policy balancing exploration and exploitation.

---

## 6. Elite Solution Pool Management

*Header: [`rkolib/core/method.hpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/include/rkolib/core/method.hpp)*

### Functions
- **`CreatePoolSolutions(solver, sizePool)`**: Generates and decodes the initial solution pool in two passes to ensure stable reference points before sorting by fitness.
- **`UpdatePoolSolutions(solver, candidateSol)`**: Evaluates candidate solutions for entry into the elite pool.
- **`CosineSimilarity(vecA, vecB)`**: Calculates diversity between random-key vectors to prevent clone saturation in the pool.

---

## 7. Integrated Metaheuristics

All metaheuristics conform to the standard function signature:
`std::function<void(const core::TRunData &, RkoSolver &)>`

| Metaheuristic | Description | File |
| :--- | :--- | :--- |
| **`SA`** | Simulated Annealing with geometric cooling schedule | [`src/mh/sa.cpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/src/mh/sa.cpp) |
| **`ILS`** | Iterated Local Search with adaptive perturbation | [`src/mh/ils.cpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/src/mh/ils.cpp) |
| **`VNS`** | Variable Neighborhood Search with neighborhood switching | [`src/mh/vns.cpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/src/mh/vns.cpp) |
| **`BRKGA`** | Biased Random-Key Genetic Algorithm | [`src/mh/brkga.cpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/src/mh/brkga.cpp) |
| **`BRKGA-CS`** | BRKGA with integrated local search phases | [`src/mh/brkga_cs.cpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/src/mh/brkga_cs.cpp) |
| **`PSO`** | Particle Swarm Optimization adapted to continuous random keys | [`src/mh/pso.cpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/src/mh/pso.cpp) |
| **`GA`** | Standard Genetic Algorithm with uniform crossover | [`src/mh/ga.cpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/src/mh/ga.cpp) |
| **`LNS`** | Large Neighborhood Search with destroy and repair operators | [`src/mh/lns.cpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/src/mh/lns.cpp) |
| **`GRASP`** | Greedy Randomized Adaptive Search Procedure | [`src/mh/grasp.cpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/src/mh/grasp.cpp) |
| **`IPR`** | Iterative Path Relinking exploring trajectories between elite solutions | [`src/mh/ipr.cpp`](file:///home/jose-lopes/pessoais/01-projetos/posdoc/rko-solver/src/mh/ipr.cpp) |
