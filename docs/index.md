# RKO - Random-Key Optimizer

Welcome to the documentation for the modernized C++20 implementation of the **RKO framework**.

This framework has been refactored around a **Dynamic Plugin Architecture** that decouples the core optimization engine from problem-specific logic. Users no longer need to recompile the solver to test new combinatorial optimization problems; they compile their problem as a shared library (`.so` or `.dll`) and load it at runtime.

![RKO_pipeline](https://github.com/user-attachments/assets/bb26650b-b5f0-4fc7-9cb1-55f5ca8b6132)

---

## Architectural Scope

The framework is split into two major components:

1. **The Core Engine (`rkosolver`)**: Manages OpenMP multi-threading, 11 registered metaheuristics, online Q-Learning parameter control with M/D/A adjacent topology and Q-Table persistence, multi-objective scalarization strategies, deterministic time-gated Nelder-Mead local search, and a thread-safe elite solution pool.
2. **The Problem Plugins (`.so` / `.dll`)**: Independent shared libraries that implement the `IProblem` interface (Decoder and IO).

### Key Features
- **11 Integrated Metaheuristics**: `SA`, `ILS`, `VNS`, `BRKGA`, `BRKGA-CS`, `PSO`, `GA`, `LNS`, `GRASP`, `IPR`, and `MultiStart`.
- **Online Q-Learning Parameter Control (`control_mode: 1`)**: Reinforcement learning engine using $O(|\mathcal{S}| \cdot m)$ adjacent M/D/A action topology, capped relative improvement (`ComputeDelta`, $\delta_{\max} = 0.1$), exploration floor ($\varepsilon_{\min} = 0.1$), conditional `UpdateMaxQ`, and thread-safe Q-Table persistence (`results/QTable_<method>.txt`).
- **Deterministic Time-Gated Nelder-Mead & Decoupled `RVND`**: `RVND` focuses strictly on `SwapLS`, `InvertLS`, and `FareyLS`, while `NelderMeadSearch` is triggered across all 9 search metaheuristics via configurable `tau_min_nm` and `tau_max_nm` time fractions.
- **Implicit Path-Relinking (`IPR`) with 6 Strategies**: Contiguous block coverage supporting `One-way`, `Back-and-Forth`, `Mixed`, `GRAPR`, `Truncated`, and `External` trajectories, tuned online by an 18-state Q-Learning controller.
- **Multi-Objective Optimization**: Native support for single- and multi-objective problems with `Tchebycheff`, `WeightedSum`, Ideal/Nadir-normalized `Gini_Coefficient`, and `Lexicographic_MinMax` scalarizers.
- **Reference Point Customization**: Dynamic or fixed ideal (`ideal_point`, `fixed_ideal_point`) and nadir (`nadir_point`, `fixed_nadir_point`) reference points.
- **Thread-Safe Elite Solution Pool & IO**: Synchronized via `#pragma omp critical(pool_lock)` with cache-line-aligned `PaddedRng` generators and full pool export to `results/Pool_Solutions_RKO.txt`.

---

## Getting Started

### Building the Core Solver

The project uses **CMake** for the core engine and a **Makefile** for rapid plugin development.

From the repository root, run:
```bash
make build
```
This creates the `build/` directory, resolves dependencies (`yaml-cpp`, `CLI11`), and compiles the `rkosolver` binary in `build/bin/rkosolver`.

### Building a Problem Plugin

To compile a problem plugin (e.g., Tourist, Knapsack Problem, or TSP) without recompiling the core:
```bash
make plugin PROB=tourist
# Or for the Traveling Salesman Problem:
make plugin PROB=tspproblem
```
Compiled plugins are saved to `build/plugins/`.

---

## Running the Algorithm

The solver is executed from the project root via CLI, providing the instance file, configuration file, maximum execution time, and compiled plugin path.

**Linux Example:**
```bash
./build/bin/rkosolver -i ./instances/tourist-1/I1.txt -c ./config/yaml/config.yaml -t 10 -p ./build/plugins/tourist.so
```

**CLI Arguments:**
- `-i`, `--instance`: Path to the problem instance file.
- `-t`, `--time`: Maximum execution time in seconds (`MAXTIME`).
- `-c`, `--config`: Path to the YAML configuration file (defaults to `config/yaml/config.yaml`).
- `-p`, `--plugin`: Path to the problem dynamic library (`.so` or `.dll`).

### Generated Output Files (`results/`)

Default configuration (`config/yaml/param-*.yaml`) and output paths (`results/`) are resolved relative to the project root:

| File | Description |
| :--- | :--- |
| `results/Results_RKO.csv` | Aggregated run statistics (OFV, average/best runtime, objectives). |
| `results/Solutions_RKO.txt` | Best solution per run, random-key vector, and discovery time. |
| `results/Pool_Solutions_RKO.txt` | Full elite pool dump (`OFV`, originating `MH`, relative `Best time`, `Objs`, `RK`). |
| `results/QTable_<method>.txt` | Persisted Q-Table per metaheuristic when `control_mode: 1` (auto-reloaded on compatible subsequent runs). |

---

## Configuration (`config/yaml/config.yaml`)

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
  debug_mode: 0        # 0: File Output (CSV/TXT), 1: Terminal Debug Output
  control_mode: 1      # 0: Offline Tuning, 1: Online Control (Q-Learning)

# Search Parameters
search_parameters:
  local_search_strategy: 1  # 1: First Improvement, 2: Best Improvement
  restart_threshold: 1      # % of max time to trigger restart
  elite_pool_size: 10       # Base size of the elite solution pool (2x for multi-objective)
  tau_min_nm: 0.005         # Minimum fraction of MAXTIME between Nelder-Mead calls
  tau_max_nm: 0.20          # Maximum fraction of MAXTIME without a Nelder-Mead call

# Multi-Objective Scalarization:
# "Tchebycheff", "WeightedSum", "Gini_Coefficient", or "Lexicographic_MinMax"
scalarization: "Tchebycheff"

# Decay factor for Lexicographic_MinMax (default: 0.05)
# lexicographic_decay: 0.05

# Default objective weights
default_weights: [0.50, 0.50]

# Optional A Priori Reference Points (calculated dynamically if omitted)
# ideal_point: [-110.0, 40.0]
# nadir_point: [0.0, 100.0]

# Freeze ideal / nadir points during search (true/false)
fixed_ideal_point: false
fixed_nadir_point: false

# Pool Update Strategy: 0 = Standard (Cosine Similarity Diversity), 1 = NSGA-II
poolUpdateMethod: 0
```

!!! tip "Parameter Ordering for Online Control"
    When using `control_mode: 1`, candidate lists in `config/yaml/param-online.yaml` should be sorted in **ascending order** so that Q-Learning Decrease (`D`) and Increase (`A`) actions correspond to true parameter decrements and increments.

---

## Creating a New Problem Plugin

To solve a new problem, implement the `rkolib::core::IProblem` interface:

1. Create a `.cpp` file in `problems/` (e.g., `myproblem.cpp`).
2. Include `rkolib/core/problem.hpp` and implement `load()`, `decode(TSol &sol) const`, `getDimension()`, and `getNumObjectives()`:

```cpp
#include "rkolib/core/problem.hpp"

class MyProblem : public rkolib::core::IProblem {
public:
    void load(const std::string &filename) override {
        // Read instance data from file
    }

    void decode(rkolib::core::TSol &sol) const override {
        // Transform random-key vector sol.rk into a feasible solution
        // Store raw objective values in sol.objs (canonical minimization convention)
    }

    int getDimension() const override { return 100; }
    int getNumObjectives() const override { return 2; }
};

// Export plugin factory functions via C linkage
REGISTER_RKO_PROBLEM(MyProblem)
```

3. Build and run your plugin:
```bash
make plugin PROB=myproblem
./build/bin/rkosolver -i ./instances/myproblem/inst1.txt -c ./config/yaml/config.yaml -t 10 -p ./build/plugins/myproblem.so
```

---

## Parallel Execution & Synchronization

- **OpenMP Thread Allocation**: Each selected metaheuristic executes independently in its own OpenMP thread.
- **False-Sharing Protection (`PaddedRng`)**: Per-thread `std::mt19937` engines are 64-byte cache-line aligned (`alignas(64)`) in `SolverContext` to prevent cache contention.
- **Dedicated Critical Sections**:
  - `#pragma omp critical(pool_lock)` protects all reads/writes to `SOLVER_POOL` (including `NelderMeadSearch` and `IPR`).
  - `#pragma omp critical(qtable_io_lock)` serializes `SaveQTable` and `LoadQTable` file access.

For deep dives into Q-Learning, Nelder-Mead/`RVND`, `IPR`, and scalarization mathematics, see [Algorithms & Control](algorithms.md).

---

## References

When using this framework in academic studies, please cite:

> [1] Chaves, A.A., Resende, M.G.C., Schuetz, M.J.A., Brubaker, J.K., Katzgraber, H.G., Arruda, E.F., Silva, R.M.A.
> *A Random-Key Optimizer for Combinatorial Optimization*. Journal of Heuristics, v. 31, n. 4, p. 32, 2025.

**DOI:** <a href="https://doi.org/10.1007/s10732-025-09568-z" target="_blank">https://doi.org/10.1007/s10732-025-09568-z</a>  
**Technical Report:** <a href="https://doi.org/10.48550/arXiv.2411.04293" target="_blank">https://doi.org/10.48550/arXiv.2411.04293</a>
