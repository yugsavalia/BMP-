# Fleet Sizing for Continuous AoI Data Collection with UAV Swarms

Code and recorded results for the paper in [`paper/Fleet_Sizing_UAV_AoI.pdf`](paper/Fleet_Sizing_UAV_AoI.pdf).

**The question the paper asks:** a fleet of UAVs flies sorties from a central depot, over and over, collecting
data from ground sensors. The goal is to keep the priority-weighted *age of information* (AoI) low.
The fleet shares a fixed energy budget, so every extra UAV means less battery for each of them. How many UAVs
should be in the air?

**The answer it gives:** a closed-form rule, `K = min(K_reach, round(K_commute))`.
- `K_reach` is set by the farthest sensor. Above it, some sensor can no longer be reached.
- `K_commute` is set by the commute radius. It is the fleet size that maximises the visit rate.

The paper checks this rule against fleet sweeps in a discrete-event simulator.

This repository contains:
1. the simulator and the sortie planners (`sim/`),
2. the scripts that ran the fleet-size sweeps (`experiments/`),
3. the raw run results as CSV (`data/`),
4. the scripts that turn those CSVs into the paper's tables, statistics and figures (`analysis/`, `figures/`).

---

## Quick start

```bash
pip install -r requirements.txt
```

Regenerate every table and statistic from the recorded runs. This runs no simulation and takes about a minute:

```bash
python reproduce.py --out results
```

Each step writes `results/<step>.txt`. The step list at the top of `reproduce.py` says which paper element each
step feeds. Its labels follow an earlier draft's table numbering; the mapping below uses the PDF's numbering.

Run the simulator itself (a small smoke test with the greedy planner, K = 2…8):

```bash
python sim/dyn_env.py
```

Run one fleet-size sweep. Example: ring layout, 50 sensors, 1.5 MJ, seeds 1–2. The K range is chosen
automatically around `K_reach`:

```bash
python experiments/run_grid.py my_runs.csv --layouts ring --M 50 --Emax 1.5e6 --seeds 1-2 --procs 4
```

Each run simulates 12 h of operation. A run takes roughly 10–60 s, depending on M and K.

---

## Folder layout

```
reproduce.py      one command: data/*.csv -> every table/statistic/figure in the paper
requirements.txt
paper/            the paper (PDF)
sim/              the model: simulator + all sortie planners (importable library)
experiments/      scripts that RUN simulations and write CSV rows
data/             recorded simulation results (the CSVs the paper's numbers come from)
analysis/         scripts that READ the CSVs and print tables / statistics
figures/          plotting scripts
legacy/           earlier development scripts, not used by the final paper (kept for history)
```

Scripts in `experiments/`, `analysis/`, `figures/` and `legacy/` add `sim/`, `experiments/` and `analysis/` to
`sys.path` in their first import line. You can run any of them from any directory with `python <folder>/<script>.py`.

---

## `sim/`: the model

| File | What it is | Paper |
|---|---|---|
| `dyn_env.py` | **The simulator.** `DynParams` holds every parameter (field size, powers, speed, buffer, event rates, reserve). `SensorField` sets up the three layouts (`paper`, `ring`, `core`), the drop-head buffers, the events and the age accounting. `DynSim` is the global discrete-event loop. It keeps exactly K UAVs airborne, coordinates them through route commitments, and integrates the objective J. It also contains the greedy ratio planner and the instrumentation that computes the per-instance `K_reach`, `K_commute`, `r_c`, `P̄` and `r_max/r_c`. | §III, §IV, §VI, Alg. 2–3 |
| `sa_sortie.py` | **The main planner.** Simulated annealing for one sortie, with the cost-aware eject-and-insert repair move. It is seeded by the greedy ratio rule and maximises Σ w̃·â under the energy budget. | §VI, Alg. 1 |
| `rr_planner.py` | Age-blind baseline planners: tour patrol (`rr_tour`), sweep patrol (`rr_sweep`), and the published k-means cluster patrol of Rahimi & Shafieinejad (`cluster_patrol`). | §VII-K |
| `cyclic_sched.py` | The territorial scheduler "C4". Each UAV gets fixed territories and a cyclic tour, both chosen by local search on a surrogate. | §VII-K |
| `territory_sa.py` | Hybrid planners: SA restricted to territories, and a square-root-due patrol. These are the failed hybrids mentioned in §VII-K. | §VII-K |
| `milp_sortie.py` | CP-SAT model of the per-decision problem (14). Used as an exact or near-exact planner. | §VI-A, §VII-K |
| `mip_gsec.py` | A second exact solver for the same problem, using a MIP with subtour cuts. Used to cross-check CP-SAT. | §VI-A |

Planners all share one interface: `planner(SortieRequest) -> list of sensor indices`. `SortieRequest`, defined in
`dyn_env.py`, holds only the information a real operator would have at launch time.

## `experiments/`: producing the data

| File | What it does |
|---|---|
| `run_grid.py` | **Main sweep runner.** Runs a layout × M × Emax × seed × K grid in parallel and appends one CSV row per run. `--planner` selects SA, the greedy rule, an ablation variant, a patrol, `cluster_patrol`, `cyc` or CP-SAT (`cpsat_d<sec>`). Other flags: `--coord`, `--replan`, `--divert`, `--belief`, `--q`, `--tau`, `--L`, `--Th`. The run is resumable. |
| `run_depot.py` | The same grid with the depot moved off-centre (§VII-F). |
| `run_hetero.py` | Reserved-sortie (unequal) energy split for the core family (§VII-C). |
| `run_mass.py` | Battery-mass-dependent rotor power (Remark 1). |
| `run_pilot.py` | Multi-planner sweeps on pilot seeds, with automatic anti-censoring extension (used to design C4). |
| `fill_k.py`, `fill_tail.py` | Extend sweeps until every optimum is interior. This is the anti-censoring protocol of §VII-A. |
| `certify_sortie.py` | Compares SA and greedy against an exact Held–Karp solver on small fields (Table II). |
| `milp_probe.py` | Checks CP-SAT against Held–Karp and times CP-SAT at M = 40 (§VI-A). |
| `outage_test.py` | Grounds the fleet for 3 h to compare the robust reach cap with the flight-only cap (§VII-B). |
| `sat_check.py` | Checks the no-overflow assumption of Proposition 3. |
| `measure_regularity.py` | Measures hop time, inter-visit CV and visit rate per planner (§V-B, §VII-K). |
| `latency.py` | Planning time per launch decision (§VI-C). |
| `fix_header.py`, `fix_schema.py` | CSV maintenance. They normalise old-schema rows to the current columns. |
| `run_baseline.ps1`, `run_rr_all.sh` | The command recipes that produced the CSVs in `data/`. Run them from the repo root. |

## `data/`: recorded runs

There is one row per simulation run. The key columns are `layout, M, Emax, K, seed, coord, replan, planner, L, Th`,
and the outputs are `J, J_age, J_event, K_reach_i, K_commute_i, r_c, P_bar, n_never, …`. The column list is
`FIELDS` in `experiments/run_grid.py`.

| File | Contents |
|---|---|
| `sa_rerun.csv` | SA, primary grid (paper/ring/core × M ∈ {50,100,200} × {1.5, 3} MJ × 12 seeds), all extensions |
| `grid.csv` | Earlier SA grid. It supplies the cells that `sa_rerun.csv` lacks: paper M = 400, 6 MJ, and the field-size (L) sweep |
| `rr_tour_win.csv`, `rr_sweep_win.csv` | Tour and sweep patrols on the primary grid |
| `cluster_win.csv` | Published cluster patrol on the primary grid |
| `m40_sa.csv`, `m40_rr.csv`, `m40_cluster.csv`, `m40_cp.csv`, `m40_cp15.csv` | The controlled M = 40 comparison (SA, patrol, cluster, CP-SAT default and 3× budget) |
| `depot_q.csv`, `depot_e.csv` | Depot at the quarter point and near an edge |
| `hetero.csv` | Reserved-sortie allocation |
| `apriori_regret.csv` | Per-instance verdicts of the design-time criterion (5) |

`reproduce.py` merges `sa_rerun.csv` and `grid.csv` into `grid_final.csv`, giving 272 instances in 24 cells.
Every analysis script reads that merged file.

The CSVs behind some single-section results are not included: Table III ablation, Table X coordination,
Table XII belief, and the mass model. To get them, run the matching `run_grid.py` flags first, then the analysis script.

## `analysis/`: CSV → paper numbers

Steps run by `reproduce.py`:

| Script | Produces |
|---|---|
| `build_grid_final.py` | Merges the SA grids into `grid_final.csv` |
| `make_table4.py` | **Table IV** (rule vs measured optimum, every cell) and the §VII-B agreement statistics |
| `make_estimators.py` | Appendix B: the per-instance estimator vs the per-cell alternatives |
| `strand_final.py` | **Table V**: the stranding condition, Proposition 6 |
| `criterion_variants.py` | **Table VII**: regret conditional on the design-time criterion (5) |
| `exante_rc.py` | §VII-J: estimates r_c and P̄ before deployment (no flight) |
| `reach_bracket.py` | §V-A / §VII-B: the K_reach–K_flight bracket |
| `baseline_table.py` | **Table XI**: fleet-size optimum under the other planners |
| `paired_diff.py` | §VII-K: paired agreement differences between planners |
| `perturb_inputs.py` | §VII-J: sensitivity to misstated physical inputs |
| `analyze_hetero.py` | §VII-C: reserved sortie vs equal shares |
| `depot_table.py` (uses `analyze_depot.py`) | **Table IX**: off-centre depot |
| `cluster_compare.py` | §VII-K: published cluster patrol vs SA |
| `bootstrap_ci.py` | All 95% intervals (stratified bootstrap, Clopper–Pearson) |

Other analysis scripts, run by hand:

| Script | Produces |
|---|---|
| `analyze.py` | General per-instance law test over any grid CSV, including the L sweep (Table VIII) and the 24 h horizon rows (Table VI) |
| `ablation.py` | Table III, from an ablation CSV |
| `compare_baseline.py`, `own_opt_compare.py` | Generic two-planner comparisons (C4 vs cluster patrol, §VII-K) |
| `analyze_mass.py` | Remark 1: the mass-dependent-power rule |
| `selector.py` | The regime-selector design that failed (§VII-K) |
| `make_table8.py` | Regret table, alternative generator |
| `cv_check.py`, `relcost_drift.py` | §V-B checks on exported per-node visit data (`EXPORT_NODES=1 run_grid.py …`) |
| `alpha_check.py`, `censor_check.py`, `failcells.py`, `rc_all.py`, `rc_check.py` | One-off diagnostics on `data/grid.csv`: r_c(K) (§VII-I), censoring, failure cells, stop counts |

## `figures/`

| Script | Figure |
|---|---|
| `plot_snapshot_pair.py` | Fig. 1: one sortie per UAV at K and K + 1 (the reach mechanism) |
| `make_figures.py` | Fig. 2 (J vs K, three families) and Fig. 3 (regime plot), written to `figs/` |
| `fig_planners.py` | Fig. 4: J(K) under three planners at M = 40 (also run by `reproduce.py`) |
| `plot_routes.py` | A route snapshot for any (M, Emax, K, seed, time) |

---

## Checked against the paper

- The simulator implements §III–VI as written. The checked items are: the energy model and the reserve; the three
  layouts; the drop-head buffers and age = time since last visit (Prop. 2); the event model and the objective (1);
  depot-mediated exclusion coordination; hover budgeted at the arrival time; Alg. 1 SA with repair; and Alg. 3 belief.
- `run_grid.py` re-runs of recorded rows reproduce J bit-for-bit.
- `reproduce.py` regenerates Table IV, Table V and Table VII exactly as printed. It also regenerates the headline
  figures: 56% exact and 78% within one UAV over 240 instances, and 163/164 interval coverage.
- The closed form (7) equals the numerical argmax of Γ(K) in (6).
- A Monte-Carlo run on the paper layout gives the Prop. 5 constants κ = 0.855, 0.893, 0.922 and 0.944, the same as the paper.
