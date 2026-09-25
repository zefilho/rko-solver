# RKO - Random-Key Optimizer

Welcome to the documentation for the modernized C++20 implementation of the **RKO framework**.

This algorithm has been heavily refactored to support a **Dynamic Plugin Architecture**. The core optimization engine is now completely decoupled from problem-specific logic. Users no longer need to recompile the solver to test new combinatorial optimization problems; they simply compile their problem as a shared library (`.so` or `.dll`) and inject it into the solver at runtime.

![RKO_pipeline](https://github.com/user-attachments/assets/bb26650b-b5f0-4fc7-9cb1-55f5ca8b6132)

---

## Architectural Scope

This framework is split into two major components:

1. **The Core Engine (`rkosolver`)**: Manages OpenMP multi-threading, 10 integrated metaheuristics, online Q-learning parameter control, multi-objective scalarization strategies, and a thread-safe elite solution pool.
2. **The Problem Plugins (`.so` / `.dll`)**: Independent shared libraries that implement the `IProblem` interface (Decoder and IO).

### Key Features
- **10 Integrated Metaheuristics**: SA, ILS, VNS, BRKGA, BRKGA-CS, PSO, GA, LNS, GRASP, and IPR.
- **Online Q-Learning Parameter Control**: Reinforcement learning engine that dynamically adapts search parameters based on metaheuristic performance (`control_mode: 1`).
- **Multi-Objective Optimization**: Native support for single- and multi-objective problems with normalized Tchebycheff, Weighted Sum, or Gini Coefficient scalarization.
- **Reference Point Customization**: Support for dynamic or fixed ideal (`ideal_point`) and nadir (`nadir_point`) reference points.
- **Elite Solution Pool Management**: Multi-policy pool updates supporting Cosine Similarity diversity verification (`poolUpdateMethod: 0`) and NSGA-II Pareto dominance/crowding distance (`poolUpdateMethod: 1`).

---

## Getting Started

### Building the Core Solver

The project uses **CMake** for the core engine and an optimized **Makefile** for rapid plugin development.

From the root directory, run:
```bash
make build
```
This will create the `build/` directory, resolve dependencies (`yaml-cpp`, `CLI11`), and compile the `rkosolver` binary.

### Building a Problem Plugin

To compile a specific problem (e.g., Tourist, Knapsack Problem, or TSP) without recompiling the core, use the agile plugin command:
```bash
make plugin PROB=tourist
# Or for the Traveling Salesman Problem:
make plugin PROB=tspproblem
```
The compiled plugins will be saved in `build/plugins/`.

---

## Running the Algorithm

The solver is executed via a robust CLI (Command Line Interface). You must provide the instance file, configuration file, maximum execution time, and compiled plugin path.

**Linux Example:**
```bash
./build/bin/rkosolver -i ./instances/tourist/I1.txt -c ./config/yaml/config.yaml -t 10 -p ./build/plugins/tourist.so
```

**CLI Arguments:**
- `-i`, `--instance`: Path to the problem instance file.
- `-t`, `--time`: Maximum execution time in seconds.
- `-c`, `--config`: Path to the YAML configuration file.
- `-p`, `--plugin`: Path to the problem dynamic library (`.so` or `.dll`).

*Note: Results and convergence metrics are automatically exported to the `results/` directory.*

---

## Configuration (`config.yaml`)

The solver is configured using a readable YAML configuration system:

```yaml
# Selected Metaheuristics (Each runs in a dedicated OpenMP thread)
metaheuristics:
  - SA
  - ILS
  - VNS
  - BRKGA
  - BRKGA-CS
  - PSO
  - GA
  - LNS
  - GRASP
  - IPR

# General Execution Settings
execution_settings:
  max_runs: 10         # Maximum number of runs
  debug_mode: 0        # 0: File Output (CSV/TXT), 1: Terminal Output
  control_mode: 1      # 0: Offline Tuning, 1: Online Control (Q-Learning)

# Search Parameters
search_parameters:
  local_search_strategy: 1  # 1: First Improvement, 2: Best Improvement
  restart_threshold: 1.0    # % of max time to trigger restart
  elite_pool_size: 10       # Size of the elite solution pool

# Multi-Objective Scalarization ("Tchebycheff", "WeightedSum", or "Gini_Coefficient")
scalarization: "Tchebycheff"

# Default weights (if not dynamically assigned by algorithm)
default_weights: [0.00, 1.00]

# Optional A Priori Reference Points (if omitted, calculated dynamically)
# ideal_point: [-110.0, 40.0]
# nadir_point: [0.0, 100.0]

# Freeze ideal point during search (true/false)
fixed_ideal_point: false

# Pool Update Strategy: 0 = Standard (Cosine Similarity Diversity), 1 = NSGA-II
poolUpdateMethod: 0
```

---

## Creating a New Problem Plugin

To solve a new problem, implement the `rkolib::core::IProblem` interface:

1. Create a new `.cpp` file in `problems/` (e.g., `myproblem.cpp`).
2. Include `rkolib/core/problem.hpp` and implement `load()` and `decode(TSol &sol) const`:

```cpp
#include "rkolib/core/problem.hpp"

class MyProblem : public rkolib::core::IProblem {
public:
    void load(const std::string &filename) override {
        // Read instance data from file
    }

    void decode(rkolib::core::TSol &sol) const override {
        // Transform random-key vector sol.rk into problem solution
        // Assign target objective values to sol.objs (e.g., sol.objs = {obj1, obj2})
    }

    int getDimension() const override { return 100; }
    int getNumObjectives() const override { return 2; }
};

// Export plugin factory cleanly using macro
REGISTER_RKO_PROBLEM(MyProblem)
```

3. Build your plugin with:
```bash
make plugin PROB=myproblem
```
4. Run `rkosolver` with your new plugin:
```bash
./build/bin/rkosolver -i ./instances/myproblem/inst1.txt -c ./config/yaml/config.yaml -t 10 -p ./build/plugins/myproblem.so
```

---

## Parallel Execution & Q-Learning Control Engine

- **OpenMP Thread Allocation**: Each selected metaheuristic runs independently in a dedicated OpenMP thread.
- **Q-Learning Engine (`control_mode: 1`)**: Maintains a $Q(s, a)$ quality matrix that learns which metaheuristics yield effective objective improvements and updates search intensities dynamically.
- **Elite Solution Sharing**: Threads share solutions through a thread-safe solution pool synchronized with critical sections (`#pragma omp critical(pool_lock)`).

---

## References

When using this framework in academic studies, please cite:

> [1] Chaves, A.A., Resende, M.G.C., Schuetz, M.J.A., Brubaker, J.K., Katzgraber, H.G., Arruda, E.F., Silva, R.M.A. 
> *A Random-Key Optimizer for Combinatorial Optimization*. Journal of Heuristics, v. 31, n. 4, p. 32, 2025.

**DOI:** <a href="https://doi.org/10.1007/s10732-025-09568-z" target="_blank">https://doi.org/10.1007/s10732-025-09568-z</a>  
**Technical Report:** <a href="https://doi.org/10.48550/arXiv.2411.04293" target="_blank">https://doi.org/10.48550/arXiv.2411.04293</a>
