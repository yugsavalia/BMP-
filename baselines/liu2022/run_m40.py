"""run_m40.py -- run the Liu MPGA baseline on EXACTLY the (layout, M, Emax, seed, K) keys of
data/m40_sa.csv, so every row pairs with an SA row and a cluster_patrol row. Resumable.

usage: python baselines/liu2022/run_m40.py [out.csv] [--procs N] [--layouts paper ring] [--seeds 1-12]
default out: baselines/liu2022/results/m40_liu.csv
then:  python analysis/baseline_table.py data/m40_sa.csv data/m40_cluster.csv baselines/liu2022/results/m40_liu.csv --by-family
"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(HERE, "..", "..")
sys.path[:0] = [os.path.join(ROOT, d) for d in ("sim", "experiments", "analysis")]
import argparse, csv
from multiprocessing import Pool
from run_grid import one, FIELDS, parse_seeds

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("out", nargs="?", default=os.path.join(HERE, "results", "m40_liu.csv"))
    ap.add_argument("--ref", default=os.path.join(ROOT, "data", "m40_sa.csv"))
    ap.add_argument("--procs", type=int, default=os.cpu_count())
    ap.add_argument("--layouts", nargs="+", default=None)
    ap.add_argument("--seeds", default=None)
    ap.add_argument("--M", type=int, default=None, help="keep only ref rows with this M")
    ap.add_argument("--Emax", type=float, default=None, help="keep only ref rows with this Emax")
    a = ap.parse_args()
    seeds = set(parse_seeds(a.seeds)) if a.seeds else None
    keys = set()
    for r in csv.DictReader(open(a.ref)):
        if a.layouts and r["layout"] not in a.layouts: continue
        if a.M and int(r["M"]) != a.M: continue
        if a.Emax and float(r["Emax"]) != a.Emax: continue
        if seeds and int(r["seed"]) not in seeds: continue
        keys.add((r["layout"], int(r["M"]), float(r["Emax"]), int(r["K"]), int(r["seed"]), float(r["L"]), float(r["Th"])))
    done = set()
    if os.path.exists(a.out) and os.path.getsize(a.out) > 0:
        for r in csv.DictReader(open(a.out)):
            done.add((r["layout"], int(r["M"]), float(r["Emax"]), int(r["K"]), int(r["seed"]), float(r["L"]), float(r["Th"])))
    jobs = [(lay, M, E, K, s, "exclude", "launch", 0, 0.0, "prior", 0.0, "liu_mpga", L, Th, 1200)
            for (lay, M, E, K, s, L, Th) in sorted(keys - done)]
    print(f"{len(jobs)} runs to do ({len(done)} already in {a.out})", flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    new = not os.path.exists(a.out) or os.path.getsize(a.out) == 0
    with open(a.out, "a", newline="") as f, Pool(a.procs) as pool:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new: w.writeheader()
        for i, row in enumerate(pool.imap_unordered(one, jobs)):
            w.writerow(row); f.flush()
            print(f"[{i + 1}/{len(jobs)}] {row['layout']} K={row['K']} s={row['seed']} J={row['J']:.3e} {row['secs']:.0f}s", flush=True)
