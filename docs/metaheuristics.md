# Metaheuristics Catalog

`RKOSolver` registers **15 metaheuristics** in `RkoSolver::initializeRegistry()` (`src/mh/`), all operating purely on continuous random-key vectors $\mathbf{x} \in [0, 1)^n$ and conforming to the uniform signature:

```cpp
void MH(const rkolib::core::TRunData &runData, rkolib::RkoSolver &solver);
```

Each active metaheuristic listed in `config/yaml/config.yaml` executes asynchronously in its own dedicated OpenMP thread, cooperating through the thread-safe `SOLVER_POOL`.

---

## 1. Trajectory & Neighborhood Metaheuristics

### 1.1 Simulated Annealing (`SA` — `src/mh/sa.cpp`)
- **Controlled Parameters (5)**: $SA_{\max}, \alpha_{\text{SA}}, \beta_{\min}, \beta_{\max}, T_0$.
- **Mechanics**: At each temperature $T$, executes $SA_{\max}$ Metropolis steps perturbing the current solution `s` with `ShakeSolution(betaMin, betaMax)` and accepting worsening moves ($\Delta > 0$) with probability $\exp(-\Delta / T)$.
- **Intensification & Reannealing**: Before Q-Learning reward evaluation and geometric cooling ($T \leftarrow \alpha_{\text{SA}} T$), evaluates the time-gated `NelderMeadSearch(s, solver)` directly on `s` (marking `improv = 1` if `sBest` improves). Cools until $T \le 10^{-5}$ (`0.00001`), then reanneals at $T \leftarrow 0.5 \, T_0$.
- **Online Reward**: $R = 1$ if `sBest` improved in the epoch; otherwise $(f(\mathbf{s}_{\text{Best}}) - f_{\text{bestEpoch}}) / (|f_{\text{bestEpoch}}| + 10^{-9})$.

### 1.2 Iterated Local Search (`ILS` — `src/mh/ils.cpp`)
- **Controlled Parameters (2)**: $\beta_{\min}, \beta_{\max}$.
- **Mechanics**: Generates an initial random-key vector refined immediately by `RVND` to initialize both the accepted local optimum `sCurrent` and the global incumbent `sBest`. Each iteration perturbs `sCurrent` via `ShakeSolution(betaMin, betaMax)` and applies `RVND` to obtain `sBestLine`.
- **Restart Acceptance & Intensification**: Uses `ILS_ACCEPT_RESTART`, updating `sCurrent = sBestLine` whenever $f(\mathbf{s}_{\text{BestLine}}) < f(\mathbf{s}_{\text{Current}})$ or after `ILS_MAX_NO_IMPROV = 10` stagnant iterations (`Iter - IterImprov >= 10`). Time-gated `NelderMeadSearch` acts directly on `sCurrent` (resetting `IterImprov` if improved).
- **Online Reward**: Binary reward $R \in \{0, 1\}$ per iteration.

### 1.3 Enhanced Iterated Local Search (`EILS` — `src/mh/eils.cpp`)
- **Controlled Parameters (3)**: $\beta_{\min}, \beta_{\max}, N_{\max}$.
- **Mechanics**: Maintains an elite local optimum $\mathbf{s}^*$ (`sBest`, initialized via `RVND`) and escalates perturbation across $L = 5$ deterministic linear levels (`EILS_NUM_LEVELS = 5`):

$$\beta_\ell = \beta_{\min} + \ell \cdot \frac{\beta_{\max} - \beta_{\min}}{L - 1}, \qquad \ell \in \{0, 1, 2, 3, 4\}$$

- **Level Escalation Schedule**: Every iteration perturbs $\mathbf{s}^*$ with fixed intensity $\beta_\ell$ (`ShakeSolution(sLine, beta, beta, n)`), applies `RVND`, and checks time-gated `NelderMeadSearch(sBest, solver)`. Any strict improvement over $\mathbf{s}^*$ sets `cycleImprov = 1` and immediately restarts a fresh cycle at $\ell = 0$; otherwise `numFail` increments, advancing $\ell \leftarrow \ell + 1$ after $N_{\max}$ consecutive failures and completing the cycle once $\ell = 5$.
- **Cycle-Synchronized Q-Learning**: Selects $(\beta_{\min}, \beta_{\max}, N_{\max})$ at the start of each level cycle (`Maintain` on the first cycle) and updates $Q(s_t, a_t)$ at cycle completion with binary reward $R = \text{cycleImprov} \in \{0, 1\}$.

### 1.4 Variable Neighborhood Search (`VNS` — `src/mh/vns.cpp`)
- **Controlled Parameters (2)**: $k_{\max}, \beta_{\min}$.
- **Initialization**: Samples **10** random initial solutions and initializes both `sBest` and `s` with the best candidate.
- **Mechanics**: Iterates over neighborhoods $k \in \{1, \dots, k_{\max}\}$, sampling $\beta \sim \mathcal{U}(k\beta_{\min}, (k+1)\beta_{\min})$, perturbing `s` via `ShakeSolution(beta, beta)`, and applying `RVND`. Accepts any improvement over `s` (resetting $k \leftarrow 1$ when `sBest` improves; otherwise $k \leftarrow k + 1$).
- **Inner Intensification & Reward**: Evaluates the deterministic `NelderMeadSearch(s, solver)` trigger **inside** the inner `while (k <= kMax)` loop using per-step `iterImprov`, while accumulating `cycleImprov` for the Q-Learning reward at the end of the neighborhood cycle.

### 1.5 Continuous Greedy Randomized Adaptive Search Procedure (`GRASP` — `src/mh/grasp.cpp`)
- **Controlled Parameters (3)**: $\alpha_{\text{GRASP}}, h_s, h_e$.
- **Bidirectional Grid `LineSearch`**: For coordinate $i$ and step $h$, builds the full candidate grid $\{s_i\} \cup \{s_i \pm \tau h \in [0, 1) : \tau = 1, \dots, \lceil 1/h \rceil\}$, shuffles it via `SOLVER_RNG`, and evaluates $q = \min(|\text{grid}|, \lceil \log_2(\lfloor 1/h \rfloor) \rceil + 1)$ candidates.
- **Randomized Adaptive Reconstruction**: Partially randomizes $\lceil n \cdot \mathcal{U}(0.3, 0.7) \rceil$ keys (`UnFixed`). At each constructive step, evaluates $k_{\max} = \max(2, \lfloor 0.1 |\text{UnFixed}| \rfloor)$ unfixed coordinates via `LineSearch`, builds $\text{RCL} = \{i : g_i \le g_{\min} + \alpha_{\text{GRASP}}(g_{\max} - g_{\min})\}$, and fixes one randomly chosen RCL coordinate.
- **Scale Halving & Inner Intensification**: Applies `RVND` to the constructed solution; halves $h \leftarrow h / 2$ whenever `sBest` does not improve, updates working solution `s` via greedy + linear Metropolis acceptance $T = 100(1 - t / (T_{\max} \cdot \text{restart}))$, and checks time-gated `NelderMeadSearch(s, solver)` at every scale step $h \ge h_e$.
- **Control Modes**: Offline mode randomizes $\alpha_{\text{GRASP}} \sim \mathcal{U}(0.1, 0.9)$ per step; online Q-Learning tunes 12 states with binary reward $R \in \{0, 1\}$.

### 1.6 Large Neighborhood Search (`LNS` — `src/mh/lns.cpp`)
- **Controlled Parameters (4)**: $\beta_{\min}, \beta_{\max}, T_0, \alpha_{\text{LNS}}$.
- **Mechanics**: Destroys $I \in [\lfloor \beta_{\min} n \rfloor, \lfloor \beta_{\max} n \rfloor]$ randomly chosen keys and repairs each key greedily by sampling inside the 18 subintervals of the Farey sequence $\mathcal{F}_7$. Uses SA-style acceptance (cooling until $T \le 0.01$, reannealing at $0.3 \, T_0$) and time-gated `NelderMeadSearch(s, solver)`.

### 1.7 Tabu Search (`TS` — `src/mh/ts.cpp`)
- **Controlled Parameters (5)**: $\text{maxNoImprovement}, \text{tabuTenure}, \text{neighborhoodSize}, \beta_{\min}, \beta_{\max}$.
- **Mechanics**: Evaluates $\lfloor n^2 \cdot \text{neighborhoodSize} \rfloor$ pairwise swap moves $(i, j)$. A swap is forbidden if $(i, j)$ or $(j, i)$ is active in `tabuList` ($\text{iter} < \text{expiration}$), unless it satisfies the **aspiration criterion** ($f(\mathbf{s}_{\text{viz}}) < f(\mathbf{s}_{\text{Best}})$).
- **Diversification & Reward**: Applies the best admissible swap, records $\{i^*, j^*, \text{iter} + \text{tabuTenure}\}$ while purging expired entries, triggers `ShakeSolution(betaMin, betaMax)` when $\text{noImp} \ge \text{maxNoImprovement}$, refines via `RVND`, and checks time-gated `NelderMeadSearch`. Online reward: $R = 1.0$ on global improvement, or $R = -0.01$ otherwise.

---

## 2. Population, Evolutionary & Swarm Metaheuristics

### 2.1 Biased Random-Key Genetic Algorithm (`BRKGA` — `src/mh/brkga.cpp`)
- **Controlled Parameters (4)**: $p, p_e, p_m, \rho_e$ (with dynamic `UpdatePopSize`).
- **Hybrid `PUX` Crossover**: Copies the top $\lfloor p \cdot p_e \rfloor$ elite individuals, generates $\lfloor p(1 - p_e - p_m) \rfloor$ offspring via Parametric Uniform Crossover (`PUX`), and injects $\lfloor p \cdot p_m \rfloor$ random mutants. In `PUX`, `nonEliteParent` is sampled 50% of the time from $[\text{eliteSize}, p - 1]$ and 50% of the time from the full population $[0, p - 1]$, swapping parents if `Pop[eliteParent].ofv > Pop[nonEliteParent].ofv` so $\rho_e$ always favors the fitter parent. Applies time-gated `NelderMeadSearch` to `Pop[0]`.

### 2.2 BRKGA with Community Search (`BRKGA-CS` — `src/mh/brkga_cs.cpp`)
- **Controlled Parameters (4)**: $p, p_e, p_m, \rho_e$.
- **Community Detection & Selective `RVND`**: Whenever an improvement occurs or stagnation exceeds 10 generations, builds a Pearson correlation graph ($\sigma > 0.6$) across the elite set and runs Label Propagation (`LP`) clustering. Non-representative elite members undergo chaotic mutation `ChaoticInd` (re-sampling keys with probability $1 - \rho_e$ using fractional `double rhoe`), while `RVND` refines the top promising community representative (`i < 1`), complemented by time-gated `NelderMeadSearch` on `Pop[0]`.

### 2.3 Genetic Algorithm (`GA` — `src/mh/ga.cpp`)
- **Controlled Parameters (3)**: $\text{sizePop}, p_{\text{cros}}, p_{\text{mut}}$.
- **Mechanics**: Binary tournament selection, uniform crossover ($p_{\text{cros}}$), gene-wise uniform mutation ($p_{\text{mut}}$), and generational replacement. Whenever time-gated `NelderMeadSearch` improves `bestInd`, the refined individual is injected back into a random slot of `Pop` (`Pop[pos1] = bestInd`).

### 2.4 Particle Swarm Optimization (`PSO` — `src/mh/pso.cpp`)
- **Controlled Parameters (4)**: $P_{\text{size}}, c_1, c_2, w$.
- **Mechanics**: Evolves particle positions $\mathbf{X}_i \in [0, 1)^n$ and velocities $\mathbf{V}_i$ toward personal bests $\mathbf{P}_{\text{best}, i}$ and global best $\mathbf{G}_{\text{best}}$. Whenever time-gated `NelderMeadSearch` improves $\mathbf{G}_{\text{best}}$, the refined vector is injected into a random slot of `Pbest` (`Pbest[chosen] = Gbest`) to propagate elite genetic information across the swarm.

### 2.5 Differential Evolution (`DE` — `src/mh/de.cpp`)
- **Controlled Parameters (3)**: $\text{sizePop}, Cr, F$ (with dynamic `UpdatePopulationDE`).
- **Hybrid Mutation & Toroidal Repair**: For each target vector $\mathbf{x}_i$, selects three distinct indices $r_1, r_2, r_3 \ne i$ and $j_{\text{rand}} \in \{0, \dots, n - 1\}$. Under binomial crossover ($u \le Cr \lor j = j_{\text{rand}}$), alternates 50/50 between `DE/rand/1` and `DE/current-to-best/1`, wrapping out-of-bounds keys toroidally via $\text{trial}_j = \text{clamp}(v_j - \lfloor v_j \rfloor, 0, 0.9999999)$.
- **Refinement & Acceptance**: Applies `RVND` to 5% of trial vectors, accepts improvements or Metropolis moves under linear cooling $T = 1000(1 - t / (T_{\max} \cdot \text{restart}))$, and applies time-gated `NelderMeadSearch` on `bestInd`.

### 2.6 Continuous Ant Colony Optimization (`ACO` / $\text{ACO}_{\mathbb{R}}$ — `src/mh/aco.cpp`)
- **Controlled Parameters (4)**: $k = \text{archive\_size}, m = \text{num\_ants}, q, \xi$ ($\varepsilon_{\min} = 0.05$, binary reward $R \in \{0, 1\}$).
- **Gaussian Solution Archive**: Maintains a sorted archive $T = (\mathbf{s}_1, \dots, \mathbf{s}_k)$ with rank-based Gaussian weights $\omega_l = \frac{1}{q k \sqrt{2\pi}} \exp\!\big(-\frac{(l - 1)^2}{2 q^2 k^2}\big)$. Each of the $m$ ants selects a guide $\mathbf{s}_l$ proportionally to $\omega_l$ and samples each coordinate $d$ from $\mathcal{N}(s_{l,d}, \sigma_d^2)$ with rejection sampling in $[0, 1)$, where $\sigma_d = \xi \sum_{e \ne l} \frac{|s_{e,d} - s_{l,d}|}{k - 1}$. Merges the $m$ ants into $T$, retains the best $k$ solutions, and applies time-gated `NelderMeadSearch` to $T[0]$.

---

## 3. Pool Trajectory Hybridization & Baseline

### 3.1 Implicit Path-Relinking (`IPR` — `src/mh/ipr.cpp`)
- **Controlled Parameters (2, embedded)**: $\alpha_{\text{block}} \in \{0.05, 0.10, 0.20\}$ and $\text{typePR} \in \{0, \dots, 5\}$ (18 online states; offline uses $\alpha = 0.10$, `PR_ONE_WAY`). Relinks elite pairs $(\mathbf{x}^s, \mathbf{x}^g)$ from `SOLVER_POOL` over contiguous key blocks across 6 strategies (see [Algorithms & Control — IPR](algorithms.md#3-implicit-path-relinking-ipr)).

### 3.2 MultiStart (`MultiStart` — `src/mh/multistart.cpp`)
- Parameter-free pure random-key sampler in $[0, 1)^n$, serving as an optimization baseline and continuous source of global diversity for `SOLVER_POOL`.

---

## 4. Consolidated Hyperparameter Grid (`param-offline.yaml` & `param-online.yaml`)

| Metaheuristic | Parameters | Offline (`control_mode: 0`) | Online Grid (`control_mode: 1`) | $|\mathcal{S}|$ |
| :--- | :--- | :--- | :--- | :---: |
| **`BRKGA`** | $p, p_e, p_m, \rho_e$ | `987, 0.20, 0.15, 0.65` | `{300, 400, 500} × {0.10, 0.15, 0.20}² × {0.60, 0.65, 0.70}` | `81` |
| **`BRKGA-CS`** | $p, p_e, p_m, \rho_e$ | `610, 0.20, 0.001, 0.65` | `{377, 610, 987} × {0.15, 0.20, 0.25} × {0.01, 0.02, 0.03} × {0.60, 0.65, 0.70}` | `81` |
| **`SA`** | $SA_{\max}, \alpha, \beta_{\min}, \beta_{\max}, T_0$ | `1000, 0.99, 0.01, 0.05, 1e6` | `{300, 500, 1000} × {0.95, 0.98, 0.99} × {0.03, 0.04, 0.05} × {0.08, 0.09, 0.10} × {1e6}` | `81` |
| **`GRASP`** | $\alpha_G, h_s, h_e$ | `0.10, 0.125, 0.00098` | `{0.10, 0.15, 0.20} × {0.0625, 0.125} × {0.00049, 0.00098}` | `12` |
| **`ILS`** | $\beta_{\min}, \beta_{\max}$ | `0.10, 0.20` | `{0.05, 0.10, 0.15} × {0.15, 0.20, 0.25}` | `9` |
| **`EILS`** | $\beta_{\min}, \beta_{\max}, N_{\max}$ | `0.10, 0.20, 3` | `{0.05, 0.07, 0.10} × {0.15, 0.20, 0.25} × {3, 5, 10}` | `27` |
| **`VNS`** | $k_{\max}, \beta_{\min}$ | `5, 0.05` | `{5, 8, 10} × {0.02, 0.04, 0.05}` | `9` |
| **`PSO`** | $P_{\text{size}}, c_1, c_2, w$ | `200, 2.05, 2.05, 0.73` | `{100, 200, 300} × {1.5, 2.0, 2.5}² × {0.5, 0.7, 0.9}` | `81` |
| **`GA`** | $\text{sizePop}, p_c, p_m$ | `400, 0.98, 0.005` | `{200, 400, 600} × {0.90, 0.95, 0.98} × {0.02, 0.03, 0.05}` | `27` |
| **`LNS`** | $\beta_{\min}, \beta_{\max}, T_0, \alpha$ | `0.10, 0.30, 10000, 0.95` | `{0.05, 0.10, 0.15} × {0.15, 0.20, 0.25, 0.30} × {100, 10000} × {0.95, 0.96}` | `48` |
| **`DE`** | $\text{sizePop}, Cr, F$ | `200, 0.50, 0.50` | `{100, 200, 300} × {0.40, 0.50, 0.60} × {0.50, 0.55, 0.60}` | `27` |
| **`TS`** | $\text{maxNoImp}, \text{ten}, \text{ng}, \beta_{\min}, \beta_{\max}$ | `10, 1000, 0.50, 0.05, 0.25` | `{500, 1000} × {200, 300} × {0.70, 0.80, 0.90} × {0.05} × {0.25}` | `12` |
| **`ACO`** | $k, m, q, \xi$ | `50, 2, 0.01, 0.85` | `{50, 75} × {2, 3} × {0.01, 0.1, 0.3} × {0.80, 0.85}` | `24` |
| **`IPR`** | $\alpha_{\text{block}}, \text{typePR}$ | `0.10, 0 (PR_ONE_WAY)` | `{0.05, 0.10, 0.20} × {0, 1, 2, 3, 4, 5}` *(embedded)* | `18` |
| **`MultiStart`** | — | Parameter-free | Parameter-free | — |

