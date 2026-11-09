"""
Section 5: Examining P, I, and D actions individually.

- Tests P-only with 5 different strengths and checks steady-state offset against theory:
  x_ss = Kp / (Kp + k)
- Adds integral action (I) to eliminate steady-state offset and show buildup of holding force.
- Adds derivative action (D) to add virtual damping and suppress oscillation.

Generates: figures/fig05a_P_sweep.png, fig05b_I_sweep.png, fig05c_D_sweep.png, results/s05.json
"""

import numpy as np
from pid_lib import SpringMassDamper, PID, simulate, step_setpoint, step_metrics
from plot_utils import new_fig, save_fig, save_results, PALETTE, GREY
import config as cfg

m_, c_, k_ = cfg.M, cfg.C, cfg.K
TS = cfg.TS_FAST
sp = step_setpoint(cfg.TARGET)


def run(kp, ki=0.0, kd=0.0, T=4.0):
    o = simulate(SpringMassDamper(m_, c_, k_), PID(kp, ki, kd, ts=TS), sp, T, ts=TS, substeps=5)
    return o, step_metrics(o["t"], o["y"], cfg.TARGET)


def theory_pd(kp, kd):
    """Closed-loop dynamics: m*x'' + (c + Kd)*x' + (k + Kp)*x = Kp*r."""
    wn = np.sqrt((k_ + kp) / m_)
    zeta = (c_ + kd) / (2.0 * np.sqrt(m_ * (k_ + kp)))
    if zeta < 1.0:
        wd = wn * np.sqrt(1.0 - zeta ** 2)
        bounce = np.exp(-np.pi * zeta / np.sqrt(1.0 - zeta ** 2)) * 100.0
        tpeak = np.pi / wd
    else:
        bounce, tpeak = 0.0, np.nan
    return dict(wn=wn, zeta=zeta, bounce_pct=bounce, t_peak=tpeak,
                x_final=kp / (kp + k_), sse=k_ / (kp + k_))


res = {}

# -----------------------------------------------------------------------------
# 5a: Proportional-only sweep
# -----------------------------------------------------------------------------
KPS = [5, 20, 50, 100, 200]
fig, (a1, a2) = new_fig(2, 2.8)
rows = []

for kp, col in zip(KPS, PALETTE):
    o, mt = run(kp)
    th = theory_pd(kp, 0)
    bounce_sim = (mt["peak"] - mt["final"]) / mt["final"] * 100.0
    a1.plot(o["t"], o["y"], color=col, label=f"Kp = {kp} N/m")
    rows.append(dict(kp=kp, x_final_sim=mt["final"], x_final_th=th["x_final"],
                     sse_sim=mt["sse"], sse_th=th["sse"], zeta=th["zeta"],
                     bounce_th=th["bounce_pct"], bounce_sim=bounce_sim,
                     t_peak_th=th["t_peak"], t_peak_sim=mt["t_peak"],
                     overshoot_vs_target=mt["overshoot_pct"], rise=mt["rise_time"],
                     settle=mt["settling_time"]))

a1.axhline(cfg.TARGET, color="k", ls="--", lw=1, label="target 1 m")
a1.set_xlim(0, 2)
a1.set_ylim(0, 1.5)
a1.set_xlabel("time [s]")
a1.set_ylabel("block position x [m]")
a1.set_title("(a) P only: increasing Kp reduces error but never reaches 1 m")
a1.legend(loc="upper right", ncol=2)

kk = np.linspace(0, 500, 300)
a2.plot(kk, kk / (kk + k_), color=GREY, label="calculation: Kp / (Kp + k)")
a2.plot([r["kp"] for r in rows], [r["x_final_sim"] for r in rows], "o", color=PALETTE[3],
        label="simulation (final position)")
a2.axhline(cfg.TARGET, color="k", ls="--", lw=1, label="target 1 m")
a2.set_xlabel("proportional gain Kp [N/m]")
a2.set_ylabel("final position x [m]")
a2.set_ylim(0, 1.1)
a2.set_xlim(0, 520)
a2.set_title("(b) Steady-state position: calculation vs simulation")
a2.legend(loc="lower right")

save_fig(fig, "fig05a_P_sweep")
res["P"] = rows

# -----------------------------------------------------------------------------
# 5b: Adding Integral action (holding Kp fixed)
# -----------------------------------------------------------------------------
KP0 = cfg.KP
KIS = [0, 25, 50, 100, 200, 400]
fig, (a1, a2) = new_fig(2, 2.8)
rows = []

for ki, col in zip(KIS, PALETTE):
    o, mt = run(KP0, ki, 0, T=10.0)
    a1.plot(o["t"], o["y"], color=col, label=f"Ki = {ki}")
    a2.plot(o["t"], o["i"], color=col, label=f"Ki = {ki}")
    rows.append(dict(ki=ki, sse=mt["sse"], overshoot=mt["overshoot_pct"],
                     settle=mt["settling_time"], rise=mt["rise_time"], i_final=o["i"][-1],
                     final=mt["final"]))

a1.axhline(cfg.TARGET, color="k", ls="--", lw=1, label="target 1 m")
a1.set_xlim(0, 6)
a1.set_ylim(0, 1.6)
a1.set_xlabel("time [s]")
a1.set_ylabel("block position x [m]")
a1.set_title(f"(a) P + I (Kp = {KP0:.0f}): integral action eliminates steady-state error")
a1.legend(loc="lower right", ncol=2)

a2.axhline(cfg.K * cfg.TARGET, color="k", ls="--", lw=1, label="k * target = 20 N")
a2.set_xlim(0, 6)
a2.set_xlabel("time [s]")
a2.set_ylabel("integral force component [N]")
a2.set_title("(b) Buildup of integral holding force")
a2.legend(loc="upper right", ncol=2)

save_fig(fig, "fig05b_I_sweep")
res["I"] = rows

# -----------------------------------------------------------------------------
# 5c: Adding Derivative action
# -----------------------------------------------------------------------------
KDS = [0, 5, 10, 20, 40]
KI0 = cfg.KI
fig, (a1, a2) = new_fig(2, 2.8)
rows_pd, rows_pid = [], []

for kd, col in zip(KDS, PALETTE):
    # PD response
    o, mt = run(KP0, 0, kd, T=4.0)
    th = theory_pd(KP0, kd)
    a1.plot(o["t"], o["y"], color=col, label=f"Kd = {kd}")
    rows_pd.append(dict(kd=kd, zeta=th["zeta"], bounce_th=th["bounce_pct"],
                        bounce_sim=(mt["peak"] - mt["final"]) / mt["final"] * 100.0,
                        t_peak_th=th["t_peak"], t_peak_sim=mt["t_peak"], sse=mt["sse"],
                        rise=mt["rise_time"], final=mt["final"]))

    # Full PID response
    o, mt = run(KP0, KI0, kd, T=6.0)
    a2.plot(o["t"], o["y"], color=col, label=f"Kd = {kd}")
    rows_pid.append(dict(kd=kd, overshoot=mt["overshoot_pct"], settle=mt["settling_time"],
                         rise=mt["rise_time"], sse=mt["sse"], peak=mt["peak"]))

for a in (a1, a2):
    a.axhline(cfg.TARGET, color="k", ls="--", lw=1, label="target 1 m")
    a.set_xlabel("time [s]")
    a.set_ylabel("block position x [m]")

a1.axhline(KP0 / (KP0 + k_), color=GREY, ls=":", lw=1)
a1.set_xlim(0, 1.5)
a1.set_ylim(0, 1.15)
a1.set_title(f"(a) P + D (Kp = {KP0:.0f}): derivative term damps bouncing")
a1.legend(loc="lower right", ncol=2)

a2.set_xlim(0, 3)
a2.set_ylim(0, 1.5)
a2.set_title(f"(b) P + I + D (Kp = {KP0:.0f}, Ki = {KI0:.0f}): Kd reduces integral overshoot")
a2.legend(loc="lower right", ncol=2)

save_fig(fig, "fig05c_D_sweep")
res["D_pd"], res["D_pid"] = rows_pd, rows_pid

res["calc"] = dict(kp_for_1pct=99.0 * k_, kp0=KP0, x_ss_kp0=KP0 / (KP0 + k_), sse_kp0=k_ / (KP0 + k_),
                   wn_kp0=np.sqrt((k_ + KP0) / m_))
save_results("s05", res)

# Print tabular summary to console
print("P only: Kp, x_final (theory/sim), sse (theory/sim), zeta, overshoot% (theory/sim)")
for r in res["P"]:
    print(f"  Kp={r['kp']:3d}: final={r['x_final_th']:.3f}/{r['x_final_sim']:.3f}, "
          f"sse={r['sse_th']:.3f}/{r['sse_sim']:.3f}, zeta={r['zeta']:.3f}, "
          f"bounce%={r['bounce_th']:.1f}/{r['bounce_sim']:.1f}")
