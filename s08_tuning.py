"""
Section 8: Controller gain tuning - Ziegler-Nichols method on G(s) = 1 / (s+1)^3.

Procedure:
1. Increase proportional gain Kp until sustained oscillations occur (Ku, Tu).
2. Apply the classical Ziegler-Nichols tuning formulas.
3. Compare response and apply refined tuning to eliminate excessive overshoot.

Generates: figures/fig08a_ziegler_nichols_find_Ku.png,
           fig08b_ziegler_nichols_result.png, results/s08.json
"""

import numpy as np
from pid_lib import ThirdOrderLag, PID, simulate, step_setpoint, step_metrics
from plot_utils import new_fig, save_fig, save_results, BLUE, ORANGE, GREEN, RED, PURPLE
import config as cfg

TS = cfg.TS_REAL
sp = step_setpoint(1.0)


def p_only(kp, T):
    return simulate(ThirdOrderLag(), PID(kp, ts=TS), sp, T, ts=TS, substeps=5)


def peaks(o):
    y = o["y"]
    idx = [i for i in range(1, len(y) - 1) if y[i] > y[i - 1] and y[i] >= y[i + 1]]
    return idx


def swing_ratio(kp, T=60.0):
    """Ratio of consecutive oscillation amplitudes about steady-state value."""
    o = p_only(kp, T)
    idx = peaks(o)
    yss = kp / (1.0 + kp)
    a = [o["y"][i] - yss for i in idx]
    return a[-1] / a[-2], o["t"][idx[-1]] - o["t"][idx[-2]]


# ---------------- Step A: Determine Ku and Tu ----------------
scan = np.arange(3.0, 12.01, 0.5)
ratios = [swing_ratio(kp)[0] for kp in scan]

lo, hi = 6.0, 10.0
for _ in range(25):
    mid = (lo + hi) / 2.0
    lo, hi = (mid, hi) if swing_ratio(mid)[0] < 1.0 else (lo, mid)

KU = (lo + hi) / 2.0
_, TU = swing_ratio(KU)
KU_THEORY, TU_THEORY = 8.0, 2.0 * np.pi / np.sqrt(3.0)

fig, (a1, a2) = new_fig(2, 2.9)
for kp, col in zip([2, 5, KU, 9], [BLUE, GREEN, ORANGE, RED]):
    o = p_only(kp, 20.0)
    lab = f"Kp = {kp:.2f} (Ku)" if kp == KU else f"Kp = {kp:.0f}"
    a1.plot(o["t"], o["y"], color=col, label=lab, lw=1.8 if kp == KU else 1.2)

o = p_only(KU, 20.0)
idx = peaks(o)[-3:]
a1.annotate("", xy=(o["t"][idx[-1]], 1.8), xytext=(o["t"][idx[-2]], 1.8),
            arrowprops=dict(arrowstyle="<->", color=ORANGE, lw=1.4))
a1.text((o["t"][idx[-1]] + o["t"][idx[-2]]) / 2.0, 1.88, f"Tu = {TU:.2f} s",
        ha="center", color=ORANGE, fontsize=7.5)
a1.set_xlim(0, 20)
a1.set_ylim(-0.5, 2.7)
a1.axhline(1.0, color="k", ls="--", lw=1)
a1.set_xlabel("time [s]")
a1.set_ylabel("output y [-]")
a1.set_title("(a) P-only response on $1/(s+1)^3$: oscillation at Ku")
a1.legend(loc="upper left", ncol=2)

a2.plot(scan, ratios, "o-", color=PURPLE, ms=3, label="amplitude ratio of consecutive swings")
a2.axhline(1.0, color="k", ls="--", lw=1, label="sustained oscillation ratio = 1")
a2.axvline(KU, color=ORANGE, ls=":", label=f"Ku = {KU:.2f}")
a2.set_xlabel("proportional gain Kp [-]")
a2.set_ylabel("swing ratio [-]")
a2.set_title("(b) Identifying Ku where oscillation ratio reaches 1.0")
a2.legend(loc="lower right")

save_fig(fig, "fig08a_ziegler_nichols_find_Ku")

# ---------------- Step B: Ziegler-Nichols tuning table ----------------
rules = {
    "P": dict(kp=0.5 * KU),
    "PI": dict(kp=0.45 * KU, ti=TU / 1.2),
    "PID": dict(kp=0.6 * KU, ti=TU / 2.0, td=TU / 8.0)
}

for r in rules.values():
    r["ki"] = r["kp"] / r["ti"] if "ti" in r else 0.0
    r["kd"] = r["kp"] * r["td"] if "td" in r else 0.0


def run_gains(kp, ki, kd, T=40.0):
    tau = (kd / kp) / 10.0 if kd > 0 else 0.0
    o = simulate(ThirdOrderLag(), PID(kp, ki, kd, ts=TS, d_filter_tau=tau), sp, T, ts=TS, substeps=3)
    mt = step_metrics(o["t"], o["y"], 1.0)
    mt["peak_u"] = float(np.abs(o["u"]).max())
    return o, mt


# ---------------- Step C: Practical refinement ----------------
grid = []
for kp in [1.5, 2.0, 2.5, 3.0, 3.5, 4.0]:
    for ti in [1.5, 2.0, 2.5, 3.0, 4.0]:
        for td in [0.5, 0.8, 1.0, 1.3]:
            _, mt = run_gains(kp, kp / ti, kp * td)
            grid.append((mt["settling_time"] if np.isfinite(mt["settling_time"]) else 1e9,
                         mt["overshoot_pct"], kp, ti, td))

candidates_ok = sorted([g for g in grid if g[1] < 5.0])
_, _, kp_b, ti_b, td_b = candidates_ok[0]
improved = dict(kp=kp_b, ti=ti_b, td=td_b, ki=kp_b / ti_b, kd=kp_b * td_b)

fig, (a1, a2) = new_fig(2, 2.9)
res = dict(Ku=KU, Tu=TU, Ku_theory=KU_THEORY, Tu_theory=TU_THEORY, rules={}, grid_size=len(grid))
runs = [("ZN P", rules["P"], BLUE),
        ("ZN PI", rules["PI"], ORANGE),
        ("ZN PID", rules["PID"], RED),
        ("improved PID", improved, GREEN)]

for name, g, col in runs:
    o, mt = run_gains(g["kp"], g["ki"], g["kd"])
    a1.plot(o["t"], o["y"], color=col, lw=2.0 if "improved" in name else 1.3,
            label=f"{name} (Kp={g['kp']:.2f}, Ki={g['ki']:.2f}, Kd={g['kd']:.2f})")
    if name in ("ZN PID", "improved PID"):
        a2.plot(o["t"], o["u"], color=col, label=name)
    res["rules"][name.replace(" ", "_")] = dict(**g, **mt)

a1.axhline(1.0, color="k", ls="--", lw=1, label="target 1")
a1.set_xlim(0, 25)
a1.set_ylim(0, 2.3)
a1.set_xlabel("time [s]")
a1.set_ylabel("output y [-]")
a1.set_title("(a) Ziegler-Nichols response vs refined tuning")
a1.legend(loc="upper right", fontsize=6.0)

a2.set_xlim(0, 10)
a2.set_xlabel("time [s]")
a2.set_ylabel("control effort u [-]")
a2.set_title("(b) Controller output signal")
a2.legend(loc="upper right")

save_fig(fig, "fig08b_ziegler_nichols_result")
save_results("s08", res)

print(f"Ku = {KU:.3f} (theoretical {KU_THEORY}), Tu = {TU:.3f} s (theoretical {TU_THEORY:.3f} s)")
