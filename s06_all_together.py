"""
Section 6: Comparing P, PI, PD, and full PID; selection of final gains.

Generates: figures/fig06_compare.png, figures/fig06b_candidates.png, results/s06.json
"""

import numpy as np
from pid_lib import SpringMassDamper, PID, simulate, step_setpoint, step_metrics
from plot_utils import new_fig, save_fig, save_results, BLUE, ORANGE, GREEN, RED, PALETTE
import config as cfg

TS = cfg.TS_FAST
sp = step_setpoint(cfg.TARGET)


def run(kp, ki, kd, T=8.0):
    o = simulate(SpringMassDamper(cfg.M, cfg.C, cfg.K), PID(kp, ki, kd, ts=TS), sp, T, ts=TS, substeps=5)
    mt = step_metrics(o["t"], o["y"], cfg.TARGET)
    mt["peak_force"] = float(np.abs(o["u"]).max())
    return o, mt


KP, KI, KD = cfg.KP, cfg.KI, cfg.KD
combos = {
    "P": (KP, 0, 0),
    "PI": (KP, KI, 0),
    "PD": (KP, 0, KD),
    "PID": (KP, KI, KD)
}
cols = {"P": BLUE, "PI": ORANGE, "PD": GREEN, "PID": RED}

fig, (a1, a2) = new_fig(2, 2.8)
res = {"combos": {}}

for name, (kp, ki, kd) in combos.items():
    o, mt = run(kp, ki, kd)
    a1.plot(o["t"], o["y"], color=cols[name], lw=2.0 if name == "PID" else 1.4,
            label=f"{name} (Kp={kp:.0f}, Ki={ki:.0f}, Kd={kd:.0f})")
    a2.plot(o["t"], o["u"], color=cols[name], lw=2.0 if name == "PID" else 1.4, label=name)
    res["combos"][name] = dict(kp=kp, ki=ki, kd=kd, **mt)

a1.axhline(cfg.TARGET, color="k", ls="--", lw=1, label="target 1 m")
a1.set_xlim(0, 3)
a1.set_ylim(0, 1.35)
a1.set_xlabel("time [s]")
a1.set_ylabel("block position x [m]")
a1.set_title("(a) Position response: P, PI, PD, and PID")
a1.legend(loc="lower right")

a2.set_xlim(0, 1.5)
a2.axhline(cfg.K * cfg.TARGET, color="k", ls=":", lw=1, label="20 N holding force")
a2.set_xlabel("time [s]")
a2.set_ylabel("control force u [N]")
a2.set_title("(b) Commanded control force")
a2.legend(loc="upper right")

save_fig(fig, "fig06_compare")

# Candidate gain sets trade-off analysis
candidates = [
    ("Gentle", 50, 50, 8),
    ("CHOSEN", KP, KI, KD),
    ("Fast", 150, 150, 10),
    ("Very fast", 300, 200, 20),
    ("Too much I", KP, 400, KD),
    ("Too much D", KP, KI, 40)
]

res["candidates"] = []
fig, ax = new_fig(1, 2.8, 5.6)

for (nm, kp, ki, kd), col in zip(candidates, PALETTE):
    o, mt = run(kp, ki, kd, T=8.0)
    ax.plot(o["t"], o["y"], color=col, lw=2.2 if nm == "CHOSEN" else 1.3,
            label=f"{nm} ({kp:.0f}, {ki:.0f}, {kd:.0f})")
    res["candidates"].append(dict(name=nm, kp=kp, ki=ki, kd=kd, **mt))

ax.axhline(cfg.TARGET, color="k", ls="--", lw=1, label="target 1 m")
ax.set_xlim(0, 3)
ax.set_ylim(0, 1.5)
ax.set_xlabel("time [s]")
ax.set_ylabel("block position x [m]")
ax.set_title("Candidate gain sets: speed vs overshoot vs actuator effort")
ax.legend(loc="lower right", ncol=2)

save_fig(fig, "fig06b_candidates")
save_results("s06", res)

print(f"{'Controller':12}{'Rise[s]':>9}{'Over[%]':>9}{'Settle[s]':>10}{'SSE[m]':>9}{'PeakF[N]':>9}")
for nm, d in list(res["combos"].items()) + [(c["name"], c) for c in res["candidates"]]:
    settle_str = f"{d['settling_time']:10.3f}" if np.isfinite(d["settling_time"]) else "       nan"
    print(f"{nm:12}{d['rise_time']:9.3f}{d['overshoot_pct']:9.2f}{settle_str}{d['sse']:9.4f}{d['peak_force']:9.1f}")
