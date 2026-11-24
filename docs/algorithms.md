# Algorithms & Control Mechanisms

This page documents the mathematical and algorithmic mechanisms behind **Online Q-Learning**, **Local Search (`RVND` & Time-Gated `NelderMeadSearch`)**, **Implicit Path-Relinking (`IPR`)**, and **Multi-Objective Scalarization** in `rkosolver`.

---

## 1. Online Q-Learning Parameter Control

When `control_mode: 1` is enabled in `config/yaml/config.yaml`, each metaheuristic adapts its hyperparameters online by navigating a discrete Markov Decision Process (MDP) over parameter configurations.

### 1.1 M/D/A Adjacent Action Topology

Each parameter $j \in \{1, \dots, m\}$ has an ordered list of candidate values $P_j = \{v_{j,1} < \dots < v_{j,n_j}\}$. Rather than connecting every state to all Hamming-distance-1 neighbors in $O(|\mathcal{S}|^2)$, `CreateStates` builds an adjacent transition graph via `stride` in $O(|\mathcal{S}| \cdot m)$:

- **Maintain (`M`)**: Stay in the current state $i \to i$.
- **Decrease (`D`)**: Step one parameter $j$ down to $v_{j, k_j - 1}$ (`i - stride[j]`) when $k_j > 1$.
- **Increase (`A`)**: Step one parameter $j$ up to $v_{j, k_j + 1}$ (`i + stride[j]`) when $k_j < n_j$.

Every state therefore has between $1 + m$ (grid corners) and $1 + 2m$ (interior states) feasible actions, ensuring smooth, physically meaningful parameter transitions.

### 1.2 Capped Relative Improvement (`ComputeDelta`) and Exploration Floor

To prevent a single large jump (e.g., leaving a penalized infeasible region or crossing zero) from collapsing exploration $\varepsilon \to 0$, relative improvement is computed by `ComputeDelta`:

$$\delta = \min\!\left(\frac{f^*_{\text{old}} - f^*_{\text{new}}}{|f^*_{\text{old}}|}, \; \delta_{\max}\right), \quad \delta_{\max} = 0.1$$

- Returns `0.0` if $f^*_{\text{new}} \ge f^*_{\text{old}}$.
- Returns $\delta_{\max} = 0.1$ if $|f^*_{\text{old}}| < 10^{-12}$.

Before choosing the next action, `SetQLParameter` updates $\varepsilon$, learning rate $\alpha_t$ (`lf`), and discount factor $\gamma$ (`df`):

$$\varepsilon \leftarrow \max\!\big(\varepsilon_{\min}, \; \varepsilon \cdot (1 - \text{clamp}(\delta, 0, 1))\big), \quad \varepsilon_{\min} = 0.1$$

$$\text{lf} = \max\!\left(0.01, \; 1.0 - 0.9 \cdot \frac{t}{\text{MAXTIME}}\right), \qquad \text{df} = 0.8$$

### 1.3 Bellman Update & Conditional `UpdateMaxQ`

After executing an iteration/generation and computing reward $R$, `UpdateQValue` applies the standard Bellman update:

$$Q(s_t, a_t) \leftarrow Q(s_t, a_t) + \text{lf} \cdot \big(R + \text{df} \cdot \max_{a'} Q(s_{t+1}, a') - Q(s_t, a_t)\big)$$

- If the updated $Q(s_t, a_t) > \text{maxQ}(s_t)$, both `maxQ` and `maxA` are updated in $O(1)$.
- Only when the previously greedy action (`at == S[st].maxA`) decreases in value does `UpdateMaxQ` rescan `S[st].Qa` to find the true maximum.

### 1.4 Q-Table Persistence (`SaveQTable` / `LoadQTable`)

- At startup (right after `CreateStates`), each online metaheuristic calls `LoadQTable(S, method)` to check `results/QTable_<method>.txt`.
- The file is loaded **only if** the saved topology (`numStates`, `numPar`, parameter values per state, and action targets `Ai`) matches the current state space exactly.
- On exit (including time-limit termination via `SOLVER_SHOULD_STOP`), `SaveQTable(S, st, method)` persists the learned Q-Table under `#pragma omp critical(qtable_io_lock)`.

---

## 2. Local Search: `RVND` & Deterministic Nelder-Mead

### 2.1 Decoupled `RVND`

`RVND` (`src/core/method.cpp`) explores three fast random-key neighborhoods in randomized order, resetting the active neighborhood list whenever an improvement is found:

1. **`SwapLS`**: Pairwise swap of random-key positions.
2. **`InvertLS`**: Mirror inversion ($r_k \leftarrow 1.0 - r_k$).
3. **`FareyLS`**: Sampling across Farey sequence intervals.

`NelderMeadSearch` has been decoupled from `RVND` so it no longer inflates the cost of every inner local-search call.

### 2.2 Time-Gated `NelderMeadSearch` Trigger

Across all 9 search metaheuristics (`SA`, `BRKGA`, `BRKGA-CS`, `GA`, `LNS`, `PSO`, `GRASP`, `ILS`, `VNS`), `NelderMeadSearch` is triggered deterministically based on elapsed time since the last call ($\Delta t_{\text{NM}}$) and two configurable fractions of `MAXTIME`:

```cpp
double deltaT_NM = currentTime - lastNM_time;
if ((improv && deltaT_NM >= tauMinNM_sec) || (deltaT_NM >= tauMaxNM_sec)) {
    NelderMeadSearch(targetSol, solver);
    lastNM_time = currentTime;
}
```

- **`tau_min_nm` (default `0.005`)**: Minimum time fraction between consecutive Nelder-Mead calls when the metaheuristic is actively finding improvements (`improv == 1`).
- **`tau_max_nm` (default `0.20`)**: Maximum time fraction allowed without a Nelder-Mead intensification step when search stagnates.

Because `runData.MAXTIME` can be overridden via `-t`/`--time` after YAML parsing, `tauMinNM` and `tauMaxNM` are stored as fractions in `[0, 1]` and converted to seconds at runtime.

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

Only finite solutions (`bestPath.ofv < infinity`) are submitted to `UpdatePoolSolutions`.

---

## 4. Multi-Objective Scalarization Strategies

All multi-objective scalarizers follow the canonical **minimization** convention (maximization objectives are negated by the problem decoder) and normalize raw objective vectors using the Ideal ($z^*$) and Nadir ($z^{\text{nad}}$) reference points:

$$d_i = w_i \cdot \max\!\left(0.0, \; \frac{f_i(x) - z^*_i}{z^{\text{nad}}_i - z^*_i}\right)$$

1. **`Tchebycheff` (`NormalizedWeightedTchebycheff`)**:
   $$F(x) = \max_{i} d_i + \rho \sum_{i} \frac{f_i(x) - z^*_i}{z^{\text{nad}}_i - z^*_i}$$
2. **`WeightedSum` (`WeightedSumScalarizer`)**:
   $$F(x) = \sum_{i} w_i \cdot f_i(x)$$
3. **`Gini_Coefficient` (`GiniScalarizer`)**:
   Minimizes mean normalized regret combined with Gini inequality across objectives (Pareto-compliant and safe for negative maximization objectives):
   $$F(x) = \bar{d} \cdot (1 + G(d)) = \bar{d} + \frac{1}{2 n^2} \sum_{i=0}^{n-1} \sum_{j=0}^{n-1} |d_i - d_j|$$
4. **`Lexicographic_MinMax` (`LexicographicMinMaxScalarizer`)**:
   Sorts normalized weighted regrets in descending order $d_{(1)} \ge d_{(2)} \ge \dots \ge d_{(n)}$ and aggregates them with geometric decay $\gamma = \text{lexicographic\_decay}$ (default `0.05`):
   $$F(x) = \sum_{k=1}^{n} \gamma^{k-1} d_{(k)}$$

