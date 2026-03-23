# RKO - Random-Key Optimizer

Welcome to the documentation for the modernized C++20 implementation of the **RKO framework**.

This algorithm has been heavily refactored to support a **Dynamic Plugin Architecture**. The core optimization engine is now completely decoupled from the problem-specific logic. Users no longer need to recompile the solver to test new combinatorial optimization problems; they simply compile their problem as a shared library (`.so` or `.dll`) and inject it into the solver at runtime.

![RKO_pipeline](https://github.com/user-attachments/assets/bb26650b-b5f0-4fc7-9cb1-55f5ca8b6132)

---

## Architectural Scope

This framework is split into two major components:

1. **The Core Engine (`rkosolver`)**: Handles the metaheuristics, OpenMP parallelization, solution pool (with cosine-similarity diversity), and multi-objective scalarization.
2. **The Problem Plugins (`.so` / `.dll`)**: Independent shared libraries that implement the `IProblem` interface (Decoder and IO).

This code natively supports Single and **Multi-Objective** optimization problems (using Tchebycheff or Weighted Sum scalarization).

---

## Getting Started

### Building the Core Solver

The project uses **CMake** for the core engine and a highly optimized **Makefile** for rapid plugin development.

From the root directory, run:
```bash
make build
```
This will create the `build/` directory, resolve dependencies (yaml-cpp, CLI11), and compile the `rkosolver` binary.

### Building a Problem Plugin

To compile a specific problem (e.g., the Knapsack Problem or TSP) without recompiling the core, use the agile plugin command:
```bash
make plugin PROB=kpproblem
# Or for the Traveling Salesman Problem:
make plugin PROB=tspproblem
```
The compiled plugins will be output to `build/plugins/`.

---

## Running the Algorithm

The solver is executed via a robust CLI (Command Line Interface). You must provide the instance, the configuration file, the max time, and the compiled plugin.

**Linux Example:**
```bash
./build/bin/rkosolver -i ./instances/kp/kp10.txt -c ./config/yaml/config.yaml -t 10 -p ./build/plugins/kpproblem.so
```

**CLI Arguments:**
- `-i`, `--instance`: Path to the problem instance file.
- `-t`, `--time`: Maximum execution time in seconds.
- `-c`, `--config`: Path to the YAML configuration file.
- `-p`, `--plugin`: Path to the problem dynamic library (`.so` or `.dll`).

*Note: Results are automatically exported to the `Results/` directory as `Solutions_RKO.txt` and `Results_RKO.csv`.*

---

## Configuration

The legacy `.conf` files have been replaced by a modern, readable YAML configuration system.
You configure parameters utilizing `config.yaml`.

```yaml
# Selected Metaheuristics (Each runs in a dedicated OpenMP thread)
metaheuristics:
  - SA
  - ILS
  - VNS
  - BRKGA
  - BRKGA-CS

execution_settings:
  max_runs: 10
  debug_mode: 0       # 0: File Output (CSV/TXT), 1: Terminal Output
  control_mode: 1     # 0: Offline Tuning, 1: Online Control (Q-Learning)

search_parameters:
  local_search_strategy: 1
  restart_threshold: 1.0
  elite_pool_size: 10

# Multi-objective handling (Tchebycheff or WeightedSum)
scalarization: Tchebycheff
```

---

## Creating a New Problem

To solve a new problem, you do not need to touch the core code! Follow these steps for Plugin Development:

1. Create a new `.cpp` file in `problems/` (e.g., `myproblem.cpp`).
2. Inherit from `rkolib::core::IProblem` - include `rkolib/core/problem.hpp` - and implement the `load()` and `evaluate(TSol& s)` methods.
3. Export the factory functions:

```cpp
extern "C" {
    EXPORT_PLUGIN rkolib::core::IProblem* create_problem() { return new MyProblem(); }
    EXPORT_PLUGIN void destroy_problem(rkolib::core::IProblem* p) { delete p; }
}
```
*Obs: It is necessary to maintain the same signature and sequence of functions so that the solver can find the functions in the plugin.*

4. Run `make plugin PROB=myproblem` **OR** `g++ -O3 -shared -fPIC -std=c++20 ./problems/myproblem.cpp -o ./build/plugins/myproblem.so`.
5. Execute `myproblem` with:
```bash
./build/bin/rkosolver -i ./examples/kp/kp10.txt -c ./config/yaml/config.yaml --time 2 -p ./build/plugins/myproblem.so
```

---

## OpenMP Parallelization

The code is heavily parallelized using OpenMP directives. In this setup, `#MH` threads are dynamically allocated based on your `config.yaml`. Each thread executes a different metaheuristic independently, sharing elite solutions through a thread-safe, diversity-aware solution pool.

---

## References

When using this algorithm in academic studies, please refer to the following work:

> [1] Chaves, A.A., Resende, M.G.C., Schuetz, M.J.A., Brubaker, J.K., Katzgraber, H.G., Arruda, E.F., Silva, R.M.A. 
> *A Random-Key Optimizer for Combinatorial Optimization*. Journal of Heuristics, v. 31, n. 4, p. 32, 2025.

**DOI:** <a href="https://doi.org/10.1007/s10732-025-09568-z" target="_blank">https://doi.org/10.1007/s10732-025-09568-z</a>  
**Technical Report:** <a href="https://doi.org/10.48550/arXiv.2411.04293" target="_blank">https://doi.org/10.48550/arXiv.2411.04293</a>
