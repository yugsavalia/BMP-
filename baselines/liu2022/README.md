# Baseline: Liu et al. (IEEE IoT-J 2022), MPGA, adapted to the fleet-sizing model

**Source paper.** C. Liu, Y. Guo, N. Li, X. Song, "AoI-Minimal Task Assignment and Trajectory Optimization in
Multi-UAV-Assisted IoT Networks," *IEEE Internet of Things Journal*, vol. 9, no. 21, pp. 21777–21791, 2022.
DOI [10.1109/JIOT.2022.3182160](https://doi.org/10.1109/JIOT.2022.3182160). The PDF is in the repo root.

**What this folder contains**

| File | Purpose |
|---|---|
| `liu_mpga.py` | The planner. It contains Liu's improved K-means, the chromosome and operators, the multi-population GA, and the runtime planner `build_liu_planner()` |
| `test_liu.py` | Self-checks on very small environments (plain asserts, runs in about 1 minute) |
| `run_m40.py` | Runs the baseline on exactly the same instances as `data/m40_sa.csv`, so every row pairs with an SA row |
| `results/m40_liu.csv` | Output of `run_m40.py`, in the same schema as every other run CSV |

One change was made outside this folder: [`experiments/run_grid.py`](../../experiments/run_grid.py) has a new
`--planner liu_mpga` branch (6 lines) and the name was added to the list of known planners. Nothing else in `sim/`,
`experiments/` or `analysis/` was modified.

---

## 1. What Liu et al. do (original)

UAVs take off from a data center (DC). Each UAV collects data from its assigned sensor nodes (SNs) in order.
The UAVs then meet at an interaction point (IPT) and share data with each other (TDMA). Each UAV then delivers
data to its assigned users and returns to the DC. The objective is the average AoI of the data the users receive
(eq. 13), subject to each UAV's energy limit (17) and a minimum separation between UAVs (18). The paper
decomposes the problem as follows:

* **P2, task assignment + trajectory** (given the IPT). The paper proposes two solvers:
  * *Hybrid (CD).* An improved global K-means splits the targets between UAVs, then dynamic programming
    (Held–Karp) orders each UAV's targets.
  * *MPGA* (their main method). One chromosome is a permutation of all SNs, followed by all users. It is cut
    into one segment per UAV by breakpoints. The GA uses N_p populations with N_0 individuals each for N_iter
    generations. Selection is roulette on a normalised fitness F; with some probability the population's best
    or the global best is chosen as a parent instead. Crossover is sequential (order) crossover (Fig. 2).
    Mutations are swap, inversion and slide. A breakpoint-update operator balances route lengths. Each
    population has its own p_cr and p_mu. Parameters (Sec. IV): N_p = 8, N_0 = 60, N_iter = 100,
    p_cr ~ U[0.5, 0.95], p_mu ~ U[0.05, 0.3].
* **P3, IPT location.** A Fermat–Weber problem solved with CVX.
* If (17) cannot be met, a *task-reduction* rule drops the lowest-priority SN and repeats until it can.

## 2. What was changed, and why

### Removed (not part of our system model)

| Liu component | Why it was removed |
|---|---|
| Users, the IPT, UAV-to-UAV data sharing, the delivery stage, P3/CVX | Our model has no users. Data is "collected" when the UAV uploads it (age at collection, §I of our paper) |
| Collision avoidance (stop–wait, altitude change) | Our simulator doesn't model collisions for any planner. State this in the paper |
| Acceleration correction ΔT_d | Our model flies at constant speed v. It is the equivalent of ΔT_d = 0 |
| FDMA, channel-dependent rate R = B log2(1 + SNR) | Our model hovers directly above each sensor with a fixed upload rate R_u. Upload time = min(λ·age, B)/R_u |
| CD hybrid and GADP (Held–Karp) | Held–Karp costs O(2ⁿ n²). With 10–40 sensors per territory it's out of reach. MPGA is their scalable main method |

### Changed

| Liu | Here | Why |
|---|---|---|
| Each UAV has its own battery E_max,n | Every sortie gets **U/K** (shared inventory) | Without this, K\* has no meaning (Remark 1 of our paper) |
| One mission that must visit all assigned SNs | Segment k becomes **UAV k's territory and its repeating visit sequence**. At each launch the UAV flies the next contiguous run of its sequence that fits U/K (`cyclic_sched.cut_sortie`, the same rule as `cluster_patrol` and C4) | A whole territory rarely fits in one sortie at U/K\* |
| Infeasible chromosome → fitness 0, then task reduction | Energy is enforced by the sortie-cutting step. A sensor that can't be reached even on its own is skipped, the same as for every other planner | At U/K\* almost every chromosome breaks (17) as a single mission, so every fitness would be 0. Task reduction would also abandon sensors that are still reachable and penalise the baseline for reasons unrelated to its routing |
| Fitness = average user AoI, eq. (22), weighted by τ (how many users want each SN's data) | Fitness = **time-averaged priority-weighted age at collection** of the repeating patrol, computed by the planner-side surrogate `cyclic_sched.Ctx.uav_J` and summed over UAVs | Optimising a one-mission objective for a nonstop deployment would hand the baseline the wrong target, and reviewers would call that unfair. The surrogate simulates the same launch stagger, the same estimated hover time at arrival, the same cutting rule, the same reserve check, and integrates weighted age exactly. It is the surrogate the C4 baseline already uses |
| User-demand weights τ | Sensor priorities w_i | Same objective as every other planner |
| Rendezvous at the IPT | None. Territories don't overlap, and the depot's commitment exclusion is still applied when a sortie is cut | Liu's "information sharing" exchanges collected data for delivery. It is **not** a way to stop UAVs serving the same sensor |
| Number of UAVs N (input) | K (the swept fleet size, including K\*) | This is the point of the comparison |
| Improved K-means: center update pulled toward the IPT, l_s | Pulled toward the **depot** | No IPT exists. The depot is the shared point every UAV returns to |

### Details the paper doesn't give: values used here (agreed before implementation)

| Item | Value | Where |
|---|---|---|
| Initial population | One individual is the improved K-means split, with each cluster ordered by NN + 2-opt (`tour_order`). All others are random permutations with random breakpoints | `seed_individual`, `mpga` |
| Probability that a parent is the population's best a_i^l / the global best a_opt | 0.1 / 0.1. Otherwise roulette on F | `CFG["p_local_best"]`, `CFG["p_global_best"]` |
| Elitism | Each population carries over its best individual | `mpga` |
| Choice of mutation | Whenever mutation fires (probability p_mu), swap, inversion or slide is picked with equal probability. "Slide translation" = move a random block to a random position | `mutate` |
| Breakpoint update | With probability p_mu, move one random breakpoint by 1–3 genes so that the segment with the longer closed depot tour gives genes to its neighbour | `breakpoint_update` |
| ψ in F | 1e-9 | `CFG["psi"]` |
| p_cr, p_mu per population | Drawn once from the paper's ranges with a fixed RNG seed (0) | `mpga` |
| Improved K-means Dr_i | The printed formula Σ_j d(i,j)/Σ_l d(i,l) is identically 1, which is a typo. We use Dr_i = Σ_j d(i,j): the most central target seeds the first cluster | `improved_kmeans` |
| Improved K-means after seeding | Lloyd iterations with the depot-pulled center update until the labels stop changing | `improved_kmeans` |
| Crossover form | OX1 (Davis order crossover). Checked against Liu Fig. 2 child by child in `test_ox1_matches_fig2`. The child keeps parent 1's breakpoints, as in Fig. 2 | `ox1` |

The GA is deterministic for a given instance and K, because the RNG seed is fixed. Every knob is in `CFG`.

### What the planner sees
Only what the other planners see: sensor positions, priorities, the depot, the airframe and energy parameters,
and the **nominal** generation rate. It does not see the true per-sensor rates or the events. The MPGA runs once,
on the first launch of a simulation. After that, each launch costs one call to `cut_sortie`.

## 3. How it fits into the code

```
DynSim ─launch k─▶ planner(req)                      (bind_sim records which UAV k is launching)
                     ├─ first call: ctx = Ctx(pos, depot, w, params, K)
                     │              perm, cuts = mpga(ctx)         ← fitness = Σ_k ctx.uav_J(segment k)
                     │              seq[k] = segment k
                     └─ every call: route = cut_sortie(seq[k], ptr[k], …, U/K, excluded)
```

---

## 4. Testing on very small environments

```bash
python baselines/liu2022/test_liu.py
```

`python baselines/liu2022/test_liu.py fast` skips the two simulator runs. Expected output ends with `ALL PASS`.

| Test | Environment | What it checks |
|---|---|---|
| `test_ox1_matches_fig2` | none | The crossover reproduces both children in Liu's Fig. 2 exactly |
| `test_operators_keep_valid_chromosome` | M = 12, K = 3 | 3,000 random crossover + mutation + breakpoint steps always leave a valid permutation and valid sorted breakpoints |
| `test_improved_kmeans` | M = 40, K = 4 | Labels lie in range, the result is deterministic, and the seed individual is a valid chromosome |
| `test_tiny_bruteforce` | **M = 5, K = 2**, three fields | Enumerates all 5! × 6 = 720 chromosomes to find the exact surrogate optimum. In these fields the K-means seed is 3–31% above the optimum, so the GA must actually search. The MPGA (2×20×40) must reach the optimum exactly and must never be worse than its seed |
| `test_harness_equals_cluster_patrol` | M = 30, K = 3, 4 h sim | With cluster_patrol's own k-means + tour sequences injected (`build_liu_planner(seqs=…)`), the Liu runtime must reproduce cluster_patrol's J **bit for bit**. This proves the deployment wrapper adds no behaviour of its own |
| `test_tiny_end_to_end` | **M = 8, K = 2**, 4 h sim | The full pipeline in DynSim: J is finite, every sensor is served, there are no duplicate visits (territories don't overlap), MPGA surrogate ≤ seed surrogate, and the territories cover every sensor exactly once |

To try a single small run by hand (shows the GA's progress):

```bash
python -c "import sys; sys.path[:0]=['sim','baselines/liu2022']; from dyn_env import DynParams, DynSim; from liu_mpga import build_liu_planner, CFG; p=DynParams(M=10,K=2,Emax=1.5e6,layout='paper',T_horizon=6*3600.,T_burnin=3600.); pl=build_liu_planner(cfg=dict(CFG,Niter=30),verbose=True); s=DynSim(p,pl,seed=1,coordinate=True); pl.bind_sim(s); print(s.run()['J_timeavg'])"
```

## 5. Comparing against our planner (SA) under normal conditions

"Normal conditions" means the setup of the existing controlled M = 40 comparison: paper and ring layouts,
M = 40, E_max = 1.5 MJ, 12 seeds, coordinated launch-time planning, 12 h horizon with 3 h burn-in, and the
same K values as `data/m40_sa.csv`. Every Liu row pairs with an SA row and a cluster-patrol row on the same
instance and K.

**Step 1: smoke test (2 seeds, about 3 minutes on 8 cores)**
```bash
python baselines/liu2022/run_m40.py smoke_liu.csv --seeds 1-2 --procs 8
python analysis/baseline_table.py data/m40_sa.csv data/m40_cluster.csv smoke_liu.csv --by-family
```

**Step 2: full M = 40 set (148 runs; each takes 30–80 s, mostly GA; about 15 minutes on 11 cores; resumable)**
```bash
python baselines/liu2022/run_m40.py --procs 11
python analysis/baseline_table.py data/m40_sa.csv data/m40_cluster.csv baselines/liu2022/results/m40_liu.csv --by-family
```

How to read `baseline_table.py`:
* `J vs SA`: median over instances of (this planner's J at its own best K) / (SA's J at its best K) − 1.
  Negative means the baseline has lower AoI than SA.
* `=SA` / `~SA`: how often this planner's best K equals SA's best K, or is within one UAV of it.
* `=rule` / `~rule`: how often its best K equals, or is within one of, the closed-form point predictor
  ℓ = min(⌊K_reach⌋, round K_commute), with K_commute taken from this planner's own measured r_c.
* `cens`: the fraction of instances whose best K sits at the top of the swept range (not a reliable optimum).

**Step 3: per-K paired view (optional)**
```bash
python -c "import pandas as pd; k=['layout','seed','K']; a=pd.read_csv('data/m40_sa.csv'); c=pd.read_csv('baselines/liu2022/results/m40_liu.csv'); m=a[k+['J']].merge(c[k+['J','n_never']],on=k,suffixes=('_sa','_liu')); m['liu/sa']=m.J_liu/m.J_sa; print(m.groupby(['layout','K'])['liu/sa'].describe()[['count','50%','min','max']])"
```

**What to look for.** The claim this tests is that **K\* doesn't depend on the planner**. Report `~rule` and
`~SA` for Liu, next to SA's own `~rule`. The J level is a separate question: report it, but the fleet-size
argument doesn't depend on SA having the lowest J.

## 6. Results (M = 40, 1.5 MJ, 12 seeds × 2 layouts, 148 runs, from `results/m40_liu.csv`)

```
planner        fam      n   J vs SA   =SA   ~SA  =rule  ~rule   bias
sa             all     24    +0.0%  1.00  1.00   0.62   1.00  +0.00
cluster_patrol all     24    +3.9%  0.58  0.83   0.42   0.96  -0.50
liu_mpga       all     24    -3.7%  0.38  1.00   0.58   1.00  -0.46
liu_mpga       paper   12    -5.5%  0.42  1.00   0.50   1.00  -0.58
liu_mpga       ring    12    -2.9%  0.33  1.00   0.67   1.00  -0.33
```

* **Fleet size.** Liu's best K is **within one UAV of the closed-form point predictor in 24/24 instances**
  (exact in 58%, against SA's 62%), and within one UAV of SA's best K in 24/24. Its best K tends to be about
  half a UAV below SA's (bias −0.46).
* **AoI level.** At each planner's own best K, adapted Liu has **lower J than SA in 19 of 24 instances**
  (median −5.5% on paper, −2.9% on ring). Per K, the gap is large at small fleets (median Liu/SA = 0.66 at
  K = 2 on paper, 0.77 on ring) and disappears at K ≥ 5–6, where stranding dominates both planners.
* **Reading.** The fleet-size claim (K\* holds across planners) is **supported**. The claim that SA has the
  lowest AoI is **not supported at M = 40** against this baseline. See the second caveat in §7: this is Liu's
  search method optimising our objective, a GA-searched territorial patrol of the same kind as C4.

### All three layouts at M = 50 (404 runs, `results/m50_liu.csv`)
Paired with the stored SA (`data/sa_rerun.csv`) and cluster-patrol (`data/cluster_win.csv`) results at
M = 50, 1.5 MJ, without rerunning either:
```bash
python baselines/liu2022/run_m40.py baselines/liu2022/results/m50_liu.csv --ref data/sa_rerun.csv --M 50 --Emax 1.5e6
```
The full write-up (changes, steps, tests, results for paper / ring / core) is in `report/report.pdf`. Regenerate
it with `python baselines/liu2022/report/make_report.py`, then compile `report/report.tex` with Tectonic.

## 7. Caveats to state in the paper

* **This is an adaptation, not the authors' code.** Every change is listed in §2. The two that matter most are
  the fitness (our surrogate) and the nonstop deployment (sortie cutting).
* **Using our surrogate as the fitness makes this a strong baseline.** It is Liu's search method optimising our
  objective. In structure it is the same as the territorial C4 baseline (fixed territories + repeating sequence
  + the same surrogate), but it searches with a GA instead of local search and has no frequency classes.
  Expect it to compete with SA on J. That is fine for the fleet-size claim, but don't present it as a weak
  baseline.
* **Territories are fixed and the sequence ignores live state.** The sequence is fixed at the first launch and
  never reacts to events, actual generation rates, or the fleet's live state. The only live elements are the
  depot's commitment exclusion and the hover-energy estimate when a sortie is cut. This is inherent to Liu's
  offline plan.
* **The single-mission AoI of the original paper isn't reported.** If a reviewer asks for it, it's the age at
  the end of each mission, but it doesn't apply to nonstop operation.
* **The GA seed is fixed (0).** To show the result isn't one lucky seed, rerun a subset with `CFG["seed"]`
  changed.
