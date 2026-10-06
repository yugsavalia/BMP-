# run_baseline.ps1 (run from the repo root) -- external-baseline work, i9-14900KS. Run blocks one at a time, paste output back.
# Rule: one writer per CSV. Never run two blocks that append to the same file concurrently.

# --- 0. Does Windows reproduce the banked SA grid? (platform FP vs code drift) ~1 min
python experiments/run_grid.py data/repro.csv --layouts ring --M 100 --Emax 1.5e6 --K 3 --seeds 1-3 --procs 3
#   banked:    8.2044e5  8.1674e5  7.4590e5
#   Linux now: 8.4615e5  8.1674e5  7.5987e5

# --- 1. Same-environment paired comparison, SA vs round-robin (primary grid)
python experiments/run_grid.py data/sa_rerun.csv --layouts paper ring core --M 50 100 200 --Emax 1.5e6 3e6 --seeds 1-12 --procs 22
python experiments/run_grid.py data/rr_tour_win.csv --layouts paper ring core --M 50 100 200 --Emax 1.5e6 3e6 --seeds 1-12 --procs 22 --planner rr_tour
python experiments/run_grid.py data/rr_sweep_win.csv --layouts paper ring core --M 50 100 200 --Emax 1.5e6 3e6 --seeds 1-12 --procs 22 --planner rr_sweep
python analysis/compare_baseline.py data/sa_rerun.csv data/rr_tour_win.csv --label rr_tour --per-cell > data/cmp_rr_tour.txt
python analysis/compare_baseline.py data/sa_rerun.csv data/rr_sweep_win.csv --label rr_sweep --per-cell > data/cmp_rr_sweep.txt

# --- 2. Un-censor core M=200 (SA argmin sits at the top of the swept K range in 11/12 instances)
python experiments/run_grid.py data/sa_rerun.csv --layouts core --M 200 --Emax 1.5e6 --seeds 1-12 --procs 22 --K 9 10 11 12 13 14 15 16 17 18
python experiments/run_grid.py data/rr_tour_win.csv --layouts core --M 200 --Emax 1.5e6 --seeds 1-12 --procs 22 --K 9 10 11 12 13 14 15 16 17 18 --planner rr_tour

# --- 3. CP-SAT on the i9: per-decision quality and time, post-burn-in decisions (~5 min)
python experiments/milp_probe.py time 40 1-3 1.5e6 4 10 60 8

# --- 4. M=40 three-planner K-sweep. Time ONE instance first; extrapolate before launching all 12.
python experiments/run_grid.py data/m40_sa.csv --layouts paper ring --M 40 --Emax 1.5e6 --seeds 1-12 --procs 22
python experiments/run_grid.py data/m40_rr.csv --layouts paper ring --M 40 --Emax 1.5e6 --seeds 1-12 --procs 22 --planner rr_tour
python experiments/run_grid.py data/m40_cp.csv --layouts paper --M 40 --Emax 1.5e6 --seeds 1 --procs 5 --planner cpsat_d5
#   cpsat uses 4 threads per run: --procs 5 = 20 threads. Deterministic budget, so oversubscription
#   changes wall time only, never results. Then:
python analysis/compare_baseline.py data/m40_sa.csv data/m40_cp.csv --label cpsat_d5 --per-cell
