"""
Section 9: Complete demonstration of reusable PID on the test system.

Runs the finalized PID controller (incorporating anti-windup, derivative filtering,
and derivative-on-measurement) against a naive uncorrected PID controller under
realistic physical constraints:
- Controller update frequency: 100 Hz (Ts = 0.01 s)
- Actuator limit: +-30 N
- Sensor noise: +-1 mm uniform random noise
- Step profile: 0 -> 1 m (at 0 s), 1 -> 0.4 m (at 3 s), 0.4 -> 1 m (at 5 s)

Generates: figures/fig09_final_demo.png, results/s09.json, results/s09_output.txt
"""

import os
import numpy as np
from pid_lib import SpringMassDamper, PID, simulate, step_metrics
from plot_utils import new_fig, save_fig, save_results, GREEN, RED, GREY, RES_DIR
import config as cfg

TS = cfg.TS_REAL
T_END = 8.0


def target_profile(t):
    if t < 0:
        return 0.0
    return 1.0 if t < 3.0 else (0.4 if t < 5.0 else 1.0)


# Configured PID controller with all practical fixes
pid = PID(kp=cfg.KP, ki=cfg.KI, kd=cfg.KD, ts=TS,
          u_min=-cfg.F_MAX, u_max=cfg.F_MAX,
          anti_windup="backcalc",
          d_filter_tau=cfg.D_FILTER_TAU,
          d_on_measurement=True)

# Baseline naive PID without fixes
naive = PID(kp=cfg.KP, ki=cfg.KI, kd=cfg.KD, ts=TS,
            u_min=-cfg.F_MAX, u_max=cfg.F_MAX,
            anti_windup="none",
            d_filter_tau=0.0,
            d_on_measurement=False)

plant = lambda: SpringMassDamper(cfg.M, cfg.C, cfg.K)
o = simulate(plant(), pid, target_profile, T_END, ts=TS, substeps=5, noise_amp=cfg.NOISE, seed=cfg.SEED)
n = simulate(plant(), naive, target_profile, T_END, ts=TS, substeps=5, noise_amp=cfg.NOISE, seed=cfg.SEED)

lines = []
P = lambda s="": (print(s), lines.append(s))

P(f"PID(kp={cfg.KP:.0f}, ki={cfg.KI:.0f}, kd={cfg.KD:.0f}, ts={TS}, u_limit=\u00b1{cfg.F_MAX:.0f} N, "
  f"anti_windup='backcalc', d_filter_tau={cfg.D_FILTER_TAU}, d_on_measurement=True)")
P(f"Sensor noise = \u00b1{cfg.NOISE*1000:.0f} mm, Seed = {cfg.SEED}")
P()
P(f"{'t [s]':>6} {'target [m]':>11} {'position [m]':>13} {'measured [m]':>13} {'force [N]':>10}")

for t_show in [0, 0.1, 0.25, 0.5, 1.0, 2.0, 2.99, 3.25, 3.5, 4.0, 5.25, 5.5, 6.0, 7.99]:
    i = int(round(t_show / TS))
    P(f"{o['t'][i]:6.2f} {o['r'][i]:11.2f} {o['y'][i]:13.4f} {o['ym'][i]:13.4f} {o['u'][i]:10.2f}")

# First step response metrics (t < 3 s)
sel = o["t"] < 3.0
mt_fix = step_metrics(o["t"][sel], o["y"][sel], 1.0)
mt_naive = step_metrics(n["t"][sel], n["y"][sel], 1.0)

after = o["t"] > 6.5
jit_fix = float(o["u"][after].std())
jit_naive = float(n["u"][after].std())

P()
P(f"{'First step: 0 -> 1 m':26}{'FIXED PID':>14}{'NAIVE PID':>14}")
P(f"{'Rise time [s]':26}{mt_fix['rise_time']:14.3f}{mt_naive['rise_time']:14.3f}")
P(f"{'Overshoot [%]':26}{mt_fix['overshoot_pct']:14.2f}{mt_naive['overshoot_pct']:14.2f}")
P(f"{'Settling time [s]':26}{mt_fix['settling_time']:14.3f}{mt_naive['settling_time']:14.3f}")
P(f"{'Steady-state error [mm]':26}{mt_fix['sse']*1000:14.2f}{mt_naive['sse']*1000:14.2f}")
P(f"{'Force jitter at rest [N]':26}{jit_fix:14.3f}{jit_naive:14.3f}")
P(f"{'Peak force demand [N]':26}{np.abs(o['u']).max():14.1f}{np.abs(n['u']).max():14.1f}")

with open(os.path.join(RES_DIR, "s09_output.txt"), "w") as f:
    f.write("\n".join(lines) + "\n")

fig, (a1, a2) = new_fig(2, 2.9)
tt = np.arange(0, T_END, TS)
a1.plot(tt, [target_profile(t) for t in tt], color="k", ls="--", lw=1, label="target")
a1.plot(o["t"], o["ym"], color=GREY, lw=0.6, alpha=0.8, label="measured (noisy)")
a1.plot(n["t"], n["y"], color=RED, lw=1.2, label="naive PID (no fixes)")
a1.plot(o["t"], o["y"], color=GREEN, lw=1.8, label="PID with fixes")
a1.set_xlabel("time [s]")
a1.set_ylabel("block position x [m]")
a1.set_title("(a) Position tracking under constraints")
a1.legend(loc="lower right")

a2.step(n["t"], n["u"], where="post", color=RED, lw=0.8, label="naive PID")
a2.step(o["t"], o["u"], where="post", color=GREEN, lw=1.0, label="PID with fixes")
a2.axhline(cfg.F_MAX, color=GREY, ls="-.", lw=1, label="motor limit \u00b130 N")
a2.axhline(-cfg.F_MAX, color=GREY, ls="-.", lw=1)
a2.set_ylim(-40, 40)
a2.set_xlabel("time [s]")
a2.set_ylabel("control force u [N]")
a2.set_title("(b) Motor command signal")
a2.legend(loc="lower right", fontsize=6.2)

save_fig(fig, "fig09_final_demo")
save_results("s09", dict(fixed=mt_fix, naive=mt_naive, jitter_fixed=jit_fix, jitter_naive=jit_naive,
                         peak_fixed=float(np.abs(o["u"]).max()), peak_naive=float(np.abs(n["u"]).max())))
