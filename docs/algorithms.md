# Algorithms & Control Mechanisms

This page documents the transversal mathematical and algorithmic mechanisms shared across **RKOSolver**: **Online Q-Learning**, **Local Search (`RVND` & Time-Gated `NelderMeadSearch`)**, **Implicit Path-Relinking (`IPR`)**, **Multi-Objective Scalarization**, and **Elite Pool Diversity Management**.

For individual algorithm mechanics and parameter grids of all 15 registered metaheuristics, see the [Metaheuristics Catalog](metaheuristics.md).

---

## 1. Online Q-Learning Parameter Control

When `control_mode: 1` is enabled in `config/yaml/config.yaml`, each metaheuristic adapts its hyperparameters online by navigating a discrete Markov Decision Process (MDP) over parameter configurations.

### 1.1 M/D/A Adjacent Action Topology

Each parameter $j \in \{0, \dots, m - 1\}$ has an ordered list of candidate values $P_j = \{v_{j,0} < \dots < v_{j,n_j - 1}\}$, forming the Cartesian state space $\mathcal{S} = P_0 \times \dots \times P_{m-1}$ of size $|\mathcal{S}| = \prod_{j=0}^{m-1} n_j$. Rather than connecting every state to all Hamming-distance-1 neighbors in $O(|\mathcal{S}|^2)$, `CreateStates` builds an adjacent transition graph via `stride` in $O(|\mathcal{S}| \cdot m)$:

$$\text{stride}[j] = \frac{|\mathcal{S}|}{\prod_{r=0}^{j} n_r}, \qquad c_j(i) = \left\lfloor \frac{i}{\text{stride}[j]} \right\rfloor \bmod n_j$$

- **Maintain (`M`)**: Stay in the current state $i \to i$.
- **Decrease (`D`)**: Step one parameter $j$ down to $v_{j, c_j(i) - 1}$ (`i - stride[j]`) when $c_j(i) > 0$.
- **Increase (`A`)**: Step one parameter $j$ up to $v_{j, c_j(i) + 1}$ (`i + stride[j]`) when $c_j(i) < n_j - 1$.

Every state therefore has between $1 + m$ (grid corners) and $1 + 2m$ (interior states) feasible actions (initialized with optimistic priors $Q_0(s, a) \sim \mathcal{U}(0.005, 0.01)$), ensuring smooth, physically meaningful parameter transitions.

### 1.2 Capped Relative Improvement (`ComputeDelta`) and Exploration Floor

To prevent a single large jump (e.g., leaving a penalized infeasible region or crossing zero) from collapsing exploration $\varepsilon \to 0$, relative improvement is computed by `ComputeDelta`:

$$\delta = \min\!\left(\frac{f^*_{\text{old}} - f^*_{\text{new}}}{|f^*_{\text{old}}|}, \; \delta_{\max}\right), \quad \delta_{\max} = 0.1$$

- Returns `0.0` if $f^*_{\text{new}} \ge f^*_{\text{old}}$.
- Returns $\delta_{\max} = 0.1$ if $f^*_{\text{new}} < f^*_{\text{old}}$ and $|f^*_{\text{old}}| < 10^{-12}$.

Before choosing the next action, `SetQLParameter` updates $\varepsilon$, learning rate $\alpha_t$ (`lf`), and discount factor $\gamma$ (`df`):

$$\varepsilon \leftarrow \max\!\big(\varepsilon_{\min}, \; \varepsilon \cdot (1 - \text{clamp}(\delta, 0, 1))\big)$$

$$\text{lf} = \max\!\left(0.01, \; 1.0 - 0.9 \cdot \frac{t}{\text{MAXTIME}}\right), \qquad \text{df} = 0.8$$

where $\varepsilon_{\min} = 0.10$ by default (`0.05` in `ACO`). Depending on the metaheuristic's temporal granularity, Q-Learning actions and rewards operate either **per iteration/generation** (e.g., `SA`, `ILS`, `LNS`, `TS`, `BRKGA`, `BRKGA-CS`, `GA`, `PSO`, `DE`, `ACO`, `IPR`) or **per complete multi-step cycle** (`VNS` over $k=1..k_{\max}$, `GRASP` over grid scales $h_s \to h_e$, and `EILS` over its 5 perturbation levels).

### 1.3 Bellman Update & Conditional `UpdateMaxQ`

After observing reward $R$, `UpdateQValue` transitions $s_t \to s_{t+1} = \text{Ai}[s_t][a_t]$ and applies the Bellman update:

$$Q(s_t, a_t) \leftarrow Q(s_t, a_t) + \text{lf} \cdot \big(R + \text{df} \cdot \max_{a'} Q(s_{t+1}, a') - Q(s_t, a_t)\big)$$

- If the updated $Q(s_t, a_t) > \text{maxQ}(s_t)$, both `maxQ` and `maxA` are updated in $O(1)$.
- Only when the previously greedy action (`at == S[st].maxA`) decreases in value does `UpdateMaxQ` rescan `S[st].Qa` in $O(|\text{Ai}|)$.

### 1.4 Q-Table Persistence (`SaveQTable` / `LoadQTable`)

- At startup (right after `CreateStates`), each online metaheuristic calls `LoadQTable(S, method)` to check `results/QTable_<method>.txt`.
- The file is loaded **only if** the saved topology (`numStates`, `numPar`, parameter values per state, and action targets `Ai`) matches the current state space within $10^{-6}$.
- On exit (including time-limit termination via `SOLVER_SHOULD_STOP`), `SaveQTable(S, st, method)` persists the learned Q-Table under `#pragma omp critical(qtable_io_lock)`.

---

## 2. Local Search: `RVND` & Deterministic Nelder-Mead

### 2.1 Decoupled `RVND`

`RVND` (`src/core/method.cpp`) explores three fast random-key neighborhoods over a shuffled permutation `RKorder`, resetting the active neighborhood list whenever an improvement is found and honoring `local_search_strategy` (`1` = First Improvement, `2` = Best Improvement):

1. **`SwapLS`**: Pairwise swap of random-key positions ($O(n^2)$).
2. **`InvertLS`**: Complementary mirror inversion $r_k \leftarrow 1.0 - r_k$ ($O(n)$).
3. **`FareyLS`**: Sampling across the 18 subintervals of the order-7 Farey sequence $\mathcal{F}_7$ ($18n$ evaluations).

`NelderMeadSearch` is decoupled from `RVND` so it never inflates the cost of inner local-search calls.

### 2.2 Time-Gated `NelderMeadSearch` Trigger & Feedback Injection

Across all **13 search metaheuristics** (`SA`, `ILS`, `EILS`, `VNS`, `GRASP`, `LNS`, `TS`, `BRKGA`, `BRKGA-CS`, `GA`, `PSO`, `DE`, `ACO`), `NelderMeadSearch` builds a 3-vertex simplex using two elite solutions sampled under `pool_lock` from `SOLVER_POOL` and executes deterministically based on elapsed time ($\Delta t_{\text{NM}}$):

```cpp
double deltaT_NM = currentTime - lastNM_time;
if ((improv && deltaT_NM >= tauMinNM_sec) || (deltaT_NM >= tauMaxNM_sec)) {
    NelderMeadSearch(targetSol, solver);
    currentTime = static_cast<float>(get_time_in_seconds() - start_timeMH);
    lastNM_time = currentTime;
}
```

- **Per-Iteration `improv = 0` Reset**: Every metaheuristic resets `improv = 0` at the top of each iteration/generation, ensuring that even in offline mode (`control_mode: 0`), `improv == 1` strictly reflects a fresh improvement rather than firing continuously every `tau_min_nm`.
- **Population & Trajectory Feedback**:
  - Trajectory MHs (`SA`, `ILS`, `VNS`, `GRASP`, `LNS`, `TS`) apply `NelderMeadSearch` directly to the active working trajectory (`s` or `sCurrent`) before evaluating the Q-Learning reward.
  - Population/Swarm MHs (`GA`, `PSO`, `BRKGA`, `BRKGA-CS`, `DE`, `ACO`) refine the elite incumbent and feed improvements back into the active population (`Pop[pos1] = bestInd` in `GA`, `Pbest[chosen] = Gbest` in `PSO`, `Pop[0]` in `BRKGA`/`BRKGA-CS`, `archive[0]` in `ACO`).

---

## 3. Implicit Path-Relinking (`IPR`)

`IPR` (`src/mh/ipr.cpp`) runs concurrently on its own OpenMP thread, pairing distinct elite solutions $(x^s, x^g)$ sampled from `SOLVER_POOL` under `#pragma omp critical(pool_lock)`. If the pool has fewer than 2 solutions at startup, the thread waits cooperatively instead of terminating.

### 3.1 Contiguous Block Partitioning

Given dimension $n$ and block fraction $\alpha \in \{0.05, 0.10, 0.20\}$:

$$b = \max(1, \lfloor \alpha n \rfloor), \qquad m = \left\lceil \frac{n}{b} \right\rceil = \frac{n + b - 1}{b}$$

Each block $\ell \in \{0, \dots, m - 1\}$ covers the contiguous slice $[\ell b, \min((\ell + 1)b, n))$, ensuring 100% key coverage even when $n$ is not divisible by $b$.

### 3.2 Six Path-Relinking Strategies

| ID | Enum | Strategy | Behavior |
| :---: | :--- | :--- | :--- |
| `0` | `PR_ONE_WAY` | One-way | Greedy internal path $x^s \to x^g$ ($d - 1$ steps). |
| `1` | `PR_BACK_FORTH` | Back-and-Forth | Evaluates both $x^s \to x^g$ and $x^g \to x^s$, keeping the best visited solution. |
| `2` | `PR_MIXED` | Mixed | Alternates greedy moves from both ends ($s_A \to s_B$, $s_B \to s_A$) until meeting in the middle. |
| `3` | `PR_GRAPR` | GRAPR | Randomized greedy One-way selecting from an RCL of the top $\lceil 0.3 |\Delta| \rceil$ moves. |
| `4` | `PR_TRUNCATED` | Truncated | Greedy One-way stopped after $\max(1, \lceil 0.5 d \rceil)$ moves. |
| `5` | `PR_EXTERNAL` | External | Greedy external path moving away from $x^g$ ($x_k \leftarrow \text{clamp}(1.0 - x_k^g, 0, 0.999999)$) for $d$ steps. |

### 3.3 Online Q-Learning in `IPR`

- **Offline (`control_mode: 0`)**: Uses $\alpha = 0.10$ and `PR_ONE_WAY`.
- **Online (`control_mode: 1`)**: Controls an 18-state space ($\alpha \in \{0.05, 0.10, 0.20\} \times \text{typePR} \in \{0, \dots, 5\}$) with binary reward (evaluated when `numDiff > 0`):

$$r_t = \begin{cases} 1, & \text{if } f(x^*_{\text{path}}) < \min(f(x^s), f(x^g)), \\ 0, & \text{otherwise.} \end{cases}$$

---

## 4. Multi-Objective Scalarization & Reference Point Sync

All multi-objective scalarizers follow the canonical **minimization** convention (maximization objectives are negated by the problem decoder) and normalize raw objective vectors using the Ideal ($z^*$) and Nadir ($z^{\text{nad}}$) reference points:

$$d_i = w_i \cdot \max\!\left(0.0, \; \frac{f_i(x) - z^*_i}{\max(10^{-9}, \; z^{\text{nad}}_i - z^*_i)}\right)$$

1. **`Tchebycheff` (`NormalizedWeightedTchebycheff`)**:
   $$F(x) = \max_{i} d_i + 10^{-4} \sum_{i} \frac{\max(0.0, \; f_i(x) - z^*_i)}{\max(10^{-9}, \; z^{\text{nad}}_i - z^*_i)}$$
2. **`WeightedSum` (`WeightedSumScalarizer`)**:
   $$F(x) = \sum_{i} w_i \cdot f_i(x)$$
3. **`Gini_Coefficient` (`GiniScalarizer`)**:
   Minimizes mean normalized weighted regret combined with Gini inequality across objectives (Sen welfare form, singularity-free and Pareto-monotonic):
   $$F(x) = \bar{d} \cdot (1 + G(d)) = \frac{1}{M}\sum_{i=0}^{M-1} d_i + \frac{1}{2 M^2} \sum_{i=0}^{M-1} \sum_{j=0}^{M-1} |d_i - d_j|$$
4. **`Lexicographic_MinMax` (`LexicographicMinMaxScalarizer`)**:
   Sorts normalized weighted regrets in descending order $d_{(0)} \ge d_{(1)} \ge \dots \ge d_{(M-1)}$ and aggregates them with geometric decay $\gamma = \text{lexicographic\_decay}$ (default `0.05`) plus a Pareto tie-breaking regularization tail:
   $$F(x) = \sum_{r=0}^{M-1} \gamma^{r} d_{(r)} + \frac{10^{-6}}{M} \sum_{r=1}^{M-1} d_{(r)}$$

### Anti-Drift Reference Point Management
- **Two-Pass Pool Initialization**: `CreatePoolSolutions` first decodes all initial candidates to consolidate non-degenerate $z^*$ and $z^{\text{nad}}$ bounds (ignoring penalized outliers $\ge 10^{12}$), and then re-scalarizes all solutions under the unified bounds.
- **Exponential Pool-Nadir Smoothing & `rescalarizePool`**: During optimization, the monitor thread tracks $z^*$ improvements and smooths $z^{\text{nad}}$ from the non-dominated elite pool ($0.8 z^{\text{nad}} + 0.2 z^{\text{nad}}_{\text{pool}}$), invoking `SolverContext::rescalarizePool` whenever reference points shift so no stale `ofv` survives in `SOLVER_POOL`.

---

## 5. Elite Pool Diversity Policies (`poolUpdateMethod`)

Every candidate submitted to `UpdatePoolSolutions` must first pass a **Global Guardian Filter** that rejects objective/OFV clones ($|\Delta| \le 10^{-9}$) and strictly Pareto-dominated solutions ($M > 1$). Accepted candidates update `SOLVER_POOL` (capacity $P = \text{sizePool}$ for $M=1$, or $P = 2 \cdot \text{sizePool}$ for $M > 1$) under one of three policies:

- **`0` — Tactical Cosine Similarity Replacement (Default)**: If $f(s) < f(\text{Pool}[P-1])$, replaces the individual in the **lower half** of the pool ($\lfloor P/2 \rfloor \le i < P$ with $f(\text{Pool}[i]) > f(s)$) that maximizes cosine similarity $\cos(s.\text{rk}, \text{Pool}[i].\text{rk})$, preserving diverse genetic structures.
- **`1` — NSGA-II Pareto Ranking & Crowding Distance**: Merges $\text{Pool} \cup \{s\}$, performs Fast Non-Dominated Sorting ($\mathcal{F}_1, \mathcal{F}_2, \dots$) and Crowding Distance assignment, and retains the top $P$ individuals.
- **`2` — Spatial Frontier Niche Geometry ($M \ge 2$)**: Projects solutions onto the chord connecting extreme Pareto anchors $(E_0, E_1)$, partitioning $[0, 1]$ into $K = 5$ niches of capacity $C = \lceil P / K \rceil$. Under-populated niches evict the worst orthogonal-distance point from overcrowded niches to fill gaps along the Pareto front.
