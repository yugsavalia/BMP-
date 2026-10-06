"""make_report.py -- figures, tables and in-text numbers for report.tex, straight from the run CSVs.

usage: python baselines/liu2022/report/make_report.py      (then compile report.tex)
Writes into this folder, for each study S in {fifty, forty}:
  fig_S.pdf (J(K) curves), tab_S.tex (summary rows), perk_S.tex (Liu/SA per K), numbers.tex (macros).
fifty = primary study, M=50, 1.5 MJ, all three layouts (SA and cluster patrol from the existing data/).
forty = secondary check, M=40, 1.5 MJ, paper + ring (the controlled set of the fleet-sizing paper).
"""
import os, math
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(HERE, "..", "..", "..")
DATA, RES = os.path.join(ROOT, "data"), os.path.join(HERE, "..", "results")
COLS = ["layout", "M", "Emax", "seed", "K", "J", "K_reach_i", "K_commute_i", "n_never"]
PLANNERS = ["SA", "Cluster patrol", "Liu MPGA"]
TAG = {"SA": "Sa", "Cluster patrol": "Cl", "Liu MPGA": "Liu"}
COL = {"SA": "#2a78d6", "Cluster patrol": "#eb6834", "Liu MPGA": "#1baf7a"}   # categorical slots 1-3
MK = {"SA": "o", "Cluster patrol": "s", "Liu MPGA": "^"}
NAME = {"paper": "Clustered (paper)", "ring": "Ring", "core": "Core"}


def load(path, M):
    d = pd.read_csv(path)[COLS]
    return d[(d.M == M) & (d.Emax == 1.5e6)]


STUDIES = {
    "fifty": dict(layouts=["paper", "ring", "core"], D={
        "SA": load(os.path.join(DATA, "sa_rerun.csv"), 50),
        "Cluster patrol": load(os.path.join(DATA, "cluster_win.csv"), 50),
        "Liu MPGA": load(os.path.join(RES, "m50_liu.csv"), 50)}),
    "forty": dict(layouts=["paper", "ring"], D={
        "SA": load(os.path.join(DATA, "m40_sa.csv"), 40),
        "Cluster patrol": load(os.path.join(DATA, "m40_cluster.csv"), 40),
        "Liu MPGA": load(os.path.join(RES, "m40_liu.csv"), 40)}),
}


def rule_of(g):
    """Closed-form point predictor l = min(floor K_reach, round K_commute), per instance, from the
    planner's own smallest-K row (as analysis/baseline_table.py does)."""
    r0 = g.loc[g.K.idxmin()]
    kr = math.floor(r0.K_reach_i); law = min(kr, r0.K_commute_i)
    return int(law) if law == kr else int(round(law))


pct = lambda x: f"{100 * x:.0f}\\%"
sgn = lambda x: "--" if not np.isfinite(x) else f"{100 * x:+.1f}\\%".replace("-", "$-$")   # typeset minus
macros = []
plt.rcParams.update({"font.size": 8, "font.family": "serif", "axes.linewidth": 0.6,
                     "axes.edgecolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e"})

for S, cfg in STUDIES.items():
    D, lays = cfg["D"], cfg["layouts"]
    rows, perk = [], []
    for lay in lays:
        sa = D["SA"][D["SA"].layout == lay]
        for pl in PLANNERS:
            d = D[pl][D[pl].layout == lay]
            rec = []
            for s in sorted(set(sa.seed) & set(d.seed)):
                gs, gb = sa[sa.seed == s].set_index("K"), d[d.seed == s].set_index("K")
                Ks = sorted(set(gs.index) & set(gb.index))
                Js, Jb = gs.J[Ks], gb.J[Ks]
                am_s, am_b = Js.idxmin(), Jb.idxmin()
                rl, rl_sa = rule_of(d[d.seed == s].reset_index()), rule_of(sa[sa.seed == s].reset_index())
                rec.append(dict(am_s=am_s, am_b=am_b, rl=rl, gap=Jb[am_b] / Js[am_s],
                                at_rule=(Jb[rl_sa] / Js[rl_sa]) if rl_sa in Ks else np.nan,
                                cens=am_b == max(Ks)))
            r = pd.DataFrame(rec)
            rows.append(dict(layout=lay, planner=pl, n=len(r), am=r.am_b.mean(), rule=r.rl.mean(),
                             ex_rule=(r.am_b == r.rl).mean(), w1_rule=((r.am_b - r.rl).abs() <= 1).mean(),
                             ex_sa=(r.am_b == r.am_s).mean(), w1_sa=((r.am_b - r.am_s).abs() <= 1).mean(),
                             gap=np.median(r.gap) - 1, better=int((r.gap < 1 - 1e-12).sum()),
                             at_rule=np.nanmedian(r.at_rule) - 1, cens=r.cens.mean()))
        m = sa[["seed", "K", "J"]].merge(D["Liu MPGA"][D["Liu MPGA"].layout == lay][["seed", "K", "J"]],
                                         on=["seed", "K"], suffixes=("_sa", "_liu"))
        for K, g in m.groupby("K"):
            perk.append(dict(layout=lay, K=K, n=len(g), med=(g.J_liu / g.J_sa).median()))
    R, P = pd.DataFrame(rows), pd.DataFrame(perk)

    with open(os.path.join(HERE, f"tab_{S}.tex"), "w") as f:
        for lay in lays:
            f.write(f"\\midrule\n\\multicolumn{{10}}{{l}}{{\\textit{{{NAME[lay]}}}}}\\\\\n")
            for _, r in R[R.layout == lay].iterrows():
                base = r.planner == "SA"
                f.write(f"{r.planner} & {r.n} & {r.am:.2f} & {pct(r.ex_rule)} & {pct(r.w1_rule)} & "
                        f"{'--' if base else pct(r.w1_sa)} & {'--' if base else sgn(r.gap)} & "
                        f"{'--' if base else f'{r.better}/{r.n}'} & {'--' if base else sgn(r.at_rule)} & {pct(r.cens)}\\\\\n")
    with open(os.path.join(HERE, f"perk_{S}.tex"), "w") as f:
        for lay in lays:
            p = P[(P.layout == lay) & (P.K <= 10)]
            cells = [f"{v:.2f}" if np.isfinite(v) else "--" for v in p.med]
            f.write(f"{NAME[lay]} & " + ", ".join(f"$K{{=}}{K}$: {c}" for K, c in zip(p.K, cells)) + "\\\\\n")
    for _, r in R.iterrows():
        tag = S + r.layout + TAG[r.planner]
        for k, v in [("WOneRule", pct(r.w1_rule)), ("ExRule", pct(r.ex_rule)), ("WOneSa", pct(r.w1_sa)),
                     ("Gap", sgn(r.gap)), ("Better", f"{r.better}/{r.n}"), ("AtRule", sgn(r.at_rule)),
                     ("Am", f"{r.am:.2f}"), ("Cens", pct(r.cens))]:
            macros.append(f"\\newcommand{{\\{tag}{k}}}{{{v}}}")
    L = R[R.planner == "Liu MPGA"]
    macros.append(f"\\newcommand{{\\{S}LiuBetterAll}}{{{int(L.better.sum())}/{int(L.n.sum())}}}")
    macros.append(f"\\newcommand{{\\{S}LiuWOneRuleAll}}{{{int(round((L.w1_rule * L.n).sum()))}/{int(L.n.sum())}}}")
    tail = P[P.K > 10].med
    if len(tail):
        macros.append(f"\\newcommand{{\\{S}TailLo}}{{{tail.min():.2f}}}\\newcommand{{\\{S}TailHi}}{{{tail.max():.2f}}}")
    for lay in lays:
        k2 = P[(P.layout == lay) & (P.K == 2)].med
        macros.append(f"\\newcommand{{\\{S}{lay}LiuKtwo}}{{{k2.iloc[0]:.2f}}}")

    # figure: J(K) / SA's per-instance minimum, median over seeds with IQR band; each planner on its own K range
    fig, axes = plt.subplots(1, len(lays), figsize=(2.35 * len(lays), 2.4))
    for ax, lay in zip(np.atleast_1d(axes), lays):
        sa = D["SA"][D["SA"].layout == lay]
        seeds = sorted(sa.seed.unique())
        sa_min = {s: sa[sa.seed == s].J.min() for s in seeds}
        for pl in PLANNERS:
            d = D[pl][D[pl].layout == lay]
            Ks = sorted(set.intersection(*[set(d[d.seed == s].K) for s in seeds]))
            norm = np.array([[d[(d.seed == s) & (d.K == K)].J.iloc[0] / sa_min[s] for K in Ks] for s in seeds])
            med = np.median(norm, 0); q1, q3 = np.quantile(norm, [.25, .75], 0)
            ax.fill_between(Ks, q1, q3, color=COL[pl], alpha=0.15, lw=0)
            ax.plot(Ks, med, color=COL[pl], lw=1.4, marker=MK[pl], ms=3.5, label=pl)
        mean_rule = np.mean([rule_of(sa[sa.seed == s].reset_index()) for s in seeds])
        ax.axvline(mean_rule, color="#52514e", ls="--", lw=0.8)
        ax.text(mean_rule, 0.98, r" mean $\ell$", transform=ax.get_xaxis_transform(), va="top", fontsize=7, color="#52514e")
        ams = [D[pl][(D[pl].layout == lay) & (D[pl].seed == s)].set_index("K").J.idxmin() for pl in PLANNERS for s in seeds]
        ax.set_xlim(1.6, max(ams) + 2.4)
        ax.set_yscale("log"); ax.set_title(NAME[lay], fontsize=8.5)
        ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda y, _: f"{y:g}"))
        ax.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        ax.set_xlabel("fleet size $K$"); ax.grid(axis="y", color="#e5e4df", lw=0.5)
        ax.spines[["top", "right"]].set_visible(False)
    first = np.atleast_1d(axes)[0]
    first.set_ylabel("J(K) / SA's minimum J")
    first.legend(frameon=False, fontsize=7, loc="upper left")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, f"fig_{S}.pdf"), bbox_inches="tight")
    print(f"== {S}\n" + R.to_string(float_format=lambda x: f"{x:.3f}"))

with open(os.path.join(HERE, "numbers.tex"), "w") as f:
    f.write("\n".join(macros) + "\n")
