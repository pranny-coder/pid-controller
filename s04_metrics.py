"""
Section 4: The four standard step response performance metrics.

Marks rise time, overshoot, settling time, and steady-state error on one graph
using a proportional-only response (Kp = 200 N/m) as an illustrative example.

Generates: figures/fig04_metrics.png, results/s04.json
"""

import numpy as np
from pid_lib import SpringMassDamper, PID, simulate, step_setpoint, step_metrics
from plot_utils import new_fig, save_fig, save_results, BLUE, ORANGE, GREEN, RED, PURPLE, GREY
import config as cfg

KP_DEMO = 200.0
o = simulate(SpringMassDamper(cfg.M, cfg.C, cfg.K), PID(KP_DEMO, ts=cfg.TS_FAST),
             step_setpoint(cfg.TARGET), 3.0, ts=cfg.TS_FAST, substeps=5)
t, y = o["t"], o["y"]
m = step_metrics(t, y, cfg.TARGET)
band = 0.02 * cfg.TARGET

fig, ax = new_fig(1, 3.4, 5.6)
ax.plot(t, y, color=BLUE, label=f"position x(t), P only, Kp = {KP_DEMO:.0f} N/m")
ax.axhline(cfg.TARGET, color="k", ls="--", lw=1, label="target 1 m")
ax.axhline(m["final"], color=GREY, ls=":", lw=1, label=f"final value {m['final']:.3f} m")
ax.fill_between([0, t[-1]], m["final"] - band, m["final"] + band, color=GREEN, alpha=0.15,
                label="settling band: final value \u00b10.02 m")

# 1) Rise time: 10% -> 90%
ax.hlines([0.1, 0.9], 0, m["t90"], color=ORANGE, lw=0.8, ls=":")
ax.annotate("", xy=(m["t90"], 0.1), xytext=(m["t10"], 0.1),
            arrowprops=dict(arrowstyle="<->", color=ORANGE, lw=1.6))
ax.plot([m["t10"], m["t90"]], [0.1, 0.9], "o", color=ORANGE, ms=4)
ax.text(m["t90"] + 0.02, 0.03, f"(1) rise time = {m['rise_time']*1000:.0f} ms\n(10% to 90% of target)",
        color=ORANGE, fontsize=7.5, va="bottom")

# 2) Overshoot
ax.annotate("", xy=(m["t_peak"], m["peak"]), xytext=(m["t_peak"], cfg.TARGET),
            arrowprops=dict(arrowstyle="<->", color=RED, lw=1.6))
ax.text(m["t_peak"] + 0.09, 1.29, f"(2) overshoot = {m['overshoot_pct']:.1f} %\n(peak {m['peak']:.3f} m)",
        color=RED, fontsize=7.5, va="center")

# 3) Settling time
ax.axvline(m["settling_time"], color=GREEN, ls="-.", lw=1.2)
ax.text(m["settling_time"] + 0.02, 0.4, f"(3) settling time\n= {m['settling_time']:.2f} s",
        color=GREEN, fontsize=7.5)

# 4) Steady-state error
xe = 1.42
ax.plot([xe, xe], [m["final"], cfg.TARGET], color=PURPLE, lw=4, solid_capstyle="butt")
ax.annotate(f"(4) steady-state error\n= {m['sse']:.3f} m ({m['sse']/cfg.TARGET*100:.1f} % of target)",
            xy=(xe, 0.955), xytext=(xe - 0.05, 0.66), color=PURPLE, fontsize=7.5,
            ha="right", va="center", arrowprops=dict(arrowstyle="-", color=PURPLE, lw=0.8))

ax.set_xlabel("time [s]")
ax.set_ylabel("block position x [m]")
ax.set_title("The four performance measures of a step response (P-only demo)")
ax.set_ylim(-0.02, 1.35)
ax.set_xlim(0, 1.5)
ax.legend(loc="lower right", fontsize=6.5)

save_fig(fig, "fig04_metrics")
save_results("s04", dict(kp=KP_DEMO, **m))

print("Step response metrics:", {k: round(float(v), 4) for k, v in m.items()})
