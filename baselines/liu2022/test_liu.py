"""test_liu.py -- self-checks for the Liu MPGA baseline. Plain asserts, no framework.

usage: python baselines/liu2022/test_liu.py          (all, ~1-2 min)
       python baselines/liu2022/test_liu.py fast     (skips the two DynSim runs)
"""
import os, sys; sys.path[:0] = [os.path.dirname(os.path.abspath(__file__))] + [os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", d) for d in ("sim",)]
import itertools
import numpy as np
from dyn_env import DynParams, DynSim, SensorField
from rr_planner import _kmeans, tour_order, build_cluster_planner
from cyclic_sched import Ctx
from liu_mpga import (ox1, mutate, breakpoint_update, segments, improved_kmeans, seed_individual,
                      Evaluator, mpga, build_liu_planner, CFG)


def tiny_ctx(M, K, seed=1, Emax=1.5e6, layout="paper"):
    p = DynParams(M=M, K=K, Emax=Emax, layout=layout)
    F = SensorField(p, np.random.default_rng(seed))
    return Ctx(F.pos, p.home, F.wi_base, p, K), p


def test_ox1_matches_fig2():
    # Liu Fig. 2: SN part (positions 3-6 kept from parent 1) and user part (positions 10-11)
    assert ox1([2, 4, 3, 8, 6, 5, 9, 7], [5, 3, 9, 2, 7, 6, 4, 8], 2, 5) == [2, 7, 3, 8, 6, 5, 4, 9]
    assert ox1([11, 12, 14, 13], [14, 11, 13, 12], 1, 2) == [13, 12, 14, 11]


def test_operators_keep_valid_chromosome():
    ctx, _ = tiny_ctx(12, 3)
    rng = np.random.default_rng(0)
    perm, cuts = list(range(12)), [4, 8]
    for _ in range(3000):
        q = [int(x) for x in rng.permutation(12)]
        a, b = sorted(rng.choice(12, 2, replace=False))
        perm = ox1(perm, q, int(a), int(b))
        perm = mutate(perm, rng)
        cuts = breakpoint_update(perm, cuts, ctx, rng)
        assert sorted(perm) == list(range(12)), perm
        assert cuts == sorted(cuts) and 0 <= cuts[0] and cuts[-1] <= 12, cuts
        assert sum(len(s) for s in segments(perm, cuts)) == 12


def test_improved_kmeans():
    ctx, _ = tiny_ctx(40, 4)
    lab = improved_kmeans(ctx.pos, ctx.home, 4)
    assert lab.shape == (40,) and set(lab.tolist()) <= set(range(4))
    assert np.array_equal(lab, improved_kmeans(ctx.pos, ctx.home, 4))          # deterministic
    perm, cuts = seed_individual(ctx)
    assert sorted(perm) == list(range(40)) and len(cuts) == 3


def test_tiny_bruteforce():
    """M=5, K=2: enumerate every (order, breakpoint) = 5! x 6 = 720 chromosomes; MPGA must reach
    the exact surrogate optimum, and can never be worse than its own seed (elitism). Field seeds
    2, 6, 8 are ones where the K-means seed is 3-31% above the optimum, so the GA must search."""
    for s in (2, 6, 8):
        ctx, _ = tiny_ctx(5, 2, seed=s)
        ev = Evaluator(ctx)
        J_opt = min(ev(list(pm), [c]) for pm in itertools.permutations(range(5)) for c in range(6))
        _, _, J_ga, info = mpga(ctx, dict(CFG, Np=2, N0=20, Niter=40))
        print(f"  tiny s={s}: J_opt={J_opt:.6e} J_seed/opt={info['J_seed'] / J_opt:.4f} J_mpga/opt={J_ga / J_opt:.6f}")
        assert info["J_seed"] > J_opt * 1.01                       # the test really exercises the search
        assert J_ga <= info["J_seed"] + 1e-9
        assert J_ga <= J_opt * (1 + 1e-9), "MPGA missed the brute-force optimum on a 720-point space"


def test_harness_equals_cluster_patrol():
    """With the k-means + tour sequences of cluster_patrol injected, the Liu runtime (bind_sim +
    cut_sortie) must reproduce cluster_patrol's J bit-for-bit: the deployment wrapper adds nothing."""
    M, K, s = 30, 3, 2
    def p_():
        return DynParams(M=M, K=K, Emax=1.5e6, layout="paper", coord_mode="exclude",
                         T_horizon=4 * 3600.0, T_burnin=1 * 3600.0)
    F = SensorField(p_(), np.random.default_rng(s))
    lab = _kmeans(F.pos, K, seed=0)
    seqs = [[int(x) for x in np.where(lab == k)[0][tour_order(F.pos[lab == k], p_().home)]] for k in range(K)]
    Js = []
    for pl in (build_cluster_planner(), build_liu_planner(seqs=seqs)):
        sim = DynSim(p_(), pl, seed=s, coordinate=True); pl.bind_sim(sim); Js.append(sim.run()["J_timeavg"])
    print(f"  harness: cluster={Js[0]:.6e} liu(seqs)={Js[1]:.6e}")
    assert Js[0] == Js[1]


def test_tiny_end_to_end():
    """Full pipeline in DynSim on a tiny field: runs, territories are disjoint (no duplicate visits),
    every sensor is served, and MPGA's surrogate is no worse than its seed."""
    p = DynParams(M=8, K=2, Emax=1.5e6, layout="paper", coord_mode="exclude",
                  T_horizon=4 * 3600.0, T_burnin=1 * 3600.0)
    pl = build_liu_planner(cfg=dict(CFG, Np=2, N0=20, Niter=20))
    sim = DynSim(p, pl, seed=3, coordinate=True); pl.bind_sim(sim); m = sim.run()
    inf = pl.state["info"]
    print(f"  e2e: J={m['J_timeavg']:.4e} dup={m['dup_frac']:.3f} never={m['n_never_visited']} "
          f"J_sur seed={inf['J_seed']:.4e} best={inf['J_best']:.4e}")
    assert np.isfinite(m["J_timeavg"]) and m["n_never_visited"] == 0 and m["dup_frac"] == 0.0
    assert inf["J_best"] <= inf["J_seed"] + 1e-9
    assert sorted(sum(pl.state["seq"], [])) == list(range(8))


if __name__ == "__main__":
    fast = len(sys.argv) > 1 and sys.argv[1] == "fast"
    tests = [test_ox1_matches_fig2, test_operators_keep_valid_chromosome, test_improved_kmeans, test_tiny_bruteforce]
    if not fast:
        tests += [test_harness_equals_cluster_patrol, test_tiny_end_to_end]
    for t in tests:
        print(t.__name__); t()
    print("ALL PASS")
