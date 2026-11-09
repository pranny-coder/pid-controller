"""
Section 10: Interview parameter sensitivity study.

Evaluates how single-parameter perturbations (mass, damping, spring stiffness,
sensor noise, force limits, gains) alter the closed-loop system dynamics.

Generates: figures/fig10_whatif.png, results/s10_output.txt
"""

import os
import numpy as np
from pid_lib import SpringMassDamper, PID, simulate, step_setpoint, step_metrics
from plot_utils import new_fig, save_fig, PALETTE, GREY, RES_DIR
import config as cfg

TS = cfg.TS_REAL
base = dict(m=cfg.M, c=cfg.C, k=cfg.K, kp=cfg.KP, ki=cfg.KI, kd=cfg.KD, fmax=cfg.F_MAX,
            noise=cfg.NOISE, tau=cfg.D_FILTER_TAU)

scenarios = [
    ("baseline", {}),
    ("mass 1 -> 2 kg", dict(m=2.0)),
    ("mass 1 -> 0.5 kg", dict(m=0.5)),
    ("spring 20 -> 40 N/m", dict(k=40.0)),
    ("spring 20 -> 10 N/m", dict(k=10.0)),
    ("damping 10 -> 2 N s/m", dict(c=2.0)),
    ("noise +-1 -> +-5 mm", dict(noise=0.005)),
    ("force limit 30 -> 15 N", dict(fmax=15.0)),
    ("force limit 30 -> 60 N", dict(fmax=60.0)),
    ("Kp x2 (200)", dict(kp=200.0)),
    ("Ki x4 (400)", dict(ki=400.0)),
    ("Kd = 0", dict(kd=0.0)),
]


def run(p):
    pid = PID(p["kp"], p["ki"], p["kd"], ts=TS, u_min=-p["fmax"], u_max=p["fmax"],
              anti_windup="backcalc", d_filter_tau=p["tau"], d_on_measurement=True)
    o = simulate(SpringMassDamper(p["m"], p["c"], p["k"]), pid, step_setpoint(cfg.TARGET), 6.0,
                 ts=TS, substeps=5, noise_amp=p["noise"], seed=cfg.SEED)
    mt = step_metrics(o["t"], o["y"], cfg.TARGET)
    mt["jitter"] = float(o["u"][o["t"] > 4.0].std())
    return o, mt


lines = []
P = lambda s="": (print(s), lines.append(s))
P(f"{'Scenario':26}{'Rise[s]':>9}{'Over[%]':>9}{'Settle[s]':>10}{'Final[m]':>10}{'Jitter[N]':>10}")

fig, (a1, a2) = new_fig(2, 2.9)
for i, (name, ch) in enumerate(scenarios):
    p = dict(base)
    p.update(ch)
    o, mt = run(p)
    fmt_val = lambda v, w, nd: f"{'never':>{w}}" if not np.isfinite(v) else f"{v:{w}.{nd}f}"
    P(f"{name:26}{fmt_val(mt['rise_time'], 9, 3)}{mt['overshoot_pct']:9.2f}{fmt_val(mt['settling_time'], 10, 2)}"
      f"{mt['final']:10.3f}{mt['jitter']:10.3f}")
    ax = a1 if i < 6 else a2
    ax.plot(o["t"], o["y"], color=PALETTE[i % 6] if i else "k", lw=2.0 if i == 0 else 1.2, label=name)

for ax, ttl in [(a1, "(a) Changing plant parameters"), (a2, "(b) Changing limits, noise, and gains")]:
    ax.axhline(cfg.TARGET, color=GREY, ls="--", lw=1)
    if ax is a2:
        a2.plot([], [], color="k", lw=2.0, label="baseline")
    ax.set_xlim(0, 3)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("block position x [m]")
    ax.set_title(ttl)
    ax.legend(loc="lower right", fontsize=6.2)

save_fig(fig, "fig10_whatif")
with open(os.path.join(RES_DIR, "s10_output.txt"), "w") as f:
    f.write("\n".join(lines) + "\n")
