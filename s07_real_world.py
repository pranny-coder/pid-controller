"""
Section 7: Four practical real-world problems and their solutions.

 (a) Actuator force saturation (30 N) -> Integral windup (fix: anti-windup clamping / back-calc)
 (b) Sensor noise (+-1 mm)           -> Derivative jitter (fix: low-pass derivative filter)
 (c) Setpoint step discontinuity     -> Derivative kick   (fix: derivative-on-measurement)
 (d) Controller update rate / delay  -> Instability       (fix: sufficient sampling rate / retuning)

Generates: figures/fig07a_windup.png, fig07b_noise.png, fig07c_kick.png,
           fig07d_sample_time.png, results/s07.json
"""

import numpy as np
from pid_lib import SpringMassDamper, PID, simulate, step_setpoint, step_metrics
from plot_utils import new_fig, save_fig, save_results, BLUE, ORANGE, GREEN, RED, PURPLE, GREY
import config as cfg

KP, KI, KD, TS = cfg.KP, cfg.KI, cfg.KD, cfg.TS_REAL
sp = step_setpoint(cfg.TARGET)
plant = lambda: SpringMassDamper(cfg.M, cfg.C, cfg.K)
res = {}

# =============================================================================
# (a) Actuator Saturation and Anti-Windup
# =============================================================================
fig, (a1, a2) = new_fig(2, 2.8)
res["a"] = {}

for name, aw, col, lab in [("none", "none", RED, "no anti-windup"),
                           ("clamp", "clamp", ORANGE, "fix 1: clamping"),
                           ("backcalc", "backcalc", GREEN, "fix 2: back-calculation")]:
    o = simulate(plant(), PID(KP, KI, KD, ts=TS, u_min=-cfg.F_MAX, u_max=cfg.F_MAX, anti_windup=aw),
                 sp, 8.0, ts=TS, substeps=5)
    mt = step_metrics(o["t"], o["y"], cfg.TARGET)
    sat_time = float(np.sum(np.abs(o["u"]) >= cfg.F_MAX - 1e-9) * TS)
    a1.plot(o["t"], o["y"], color=col, label=lab)
    a2.plot(o["t"], o["i"], color=col, label=lab)
    res["a"][name] = dict(**mt, max_i=float(o["i"].max()), sat_time=sat_time)

a1.axhline(cfg.TARGET, color="k", ls="--", lw=1, label="target 1 m")
a1.set_xlim(0, 3)
a1.set_ylim(0, 1.15)
a1.set_xlabel("time [s]")
a1.set_ylabel("block position x [m]")
a1.set_title("(a) Position under \u00b130 N motor saturation")
a1.legend(loc="lower right")

a2.axhline(cfg.F_MAX, color=GREY, ls="-.", lw=1, label="motor limit 30 N")
a2.axhline(cfg.K * cfg.TARGET, color="k", ls=":", lw=1, label="steady-state hold: 20 N")
a2.set_xlim(0, 3)
a2.set_ylim(-1, 47)
a2.set_xlabel("time [s]")
a2.set_ylabel("integral term contribution [N]")
a2.set_title("(b) Integrator accumulation")
a2.legend(loc="upper right", ncol=2, fontsize=6.2)

save_fig(fig, "fig07a_windup")

# =============================================================================
# (b) Sensor Noise and Derivative Low-Pass Filtering
# =============================================================================
fig, (a1, a2) = new_fig(2, 2.8)
res["b"] = {}

for tau, col, lab in [(0.0, RED, "no filter (tau = 0)"),
                      (0.02, GREEN, "filter tau = 0.02 s"),
                      (0.05, BLUE, "filter tau = 0.05 s")]:
    o = simulate(plant(), PID(KP, KI, KD, ts=TS, d_filter_tau=tau), sp, 4.0, ts=TS, substeps=5,
                 noise_amp=cfg.NOISE, seed=cfg.SEED)
    o_clean = simulate(plant(), PID(KP, KI, KD, ts=TS, d_filter_tau=tau), sp, 4.0, ts=TS, substeps=5)
    mt = step_metrics(o_clean["t"], o_clean["y"], cfg.TARGET)
    sel = o["t"] > 2.0
    a1.step(o["t"], o["u"], where="post", color=col, lw=1.0, label=lab)
    a2.plot(o["t"], o["y"], color=col, label=lab)
    res["b"][f"tau_{tau}"] = dict(u_std=float(o["u"][sel].std()), d_std=float(o["d"][sel].std()),
                                  u_pp=float(o["u"][sel].max() - o["u"][sel].min()),
                                  rise=mt["rise_time"], settle=mt["settling_time"],
                                  overshoot=mt["overshoot_pct"])

a1.set_xlim(2.0, 2.5)
a1.set_ylim(15, 25)
a1.axhline(cfg.K * cfg.TARGET, color="k", ls=":", lw=1)
a1.set_xlabel("time [s]")
a1.set_ylabel("control force u [N]")
a1.set_title("(a) Control force under \u00b11 mm sensor noise")
a1.legend(loc="upper right")

a2.axhline(cfg.TARGET, color="k", ls="--", lw=1, label="target 1 m")
a2.set_xlim(0, 1.5)
a2.set_ylim(0, 1.1)
a2.set_xlabel("time [s]")
a2.set_ylabel("block position x [m]")
a2.set_title("(b) Position response with filtered derivative")
a2.legend(loc="lower right")

save_fig(fig, "fig07b_noise")

# =============================================================================
# (c) Setpoint Step and Derivative Kick
# =============================================================================
fig, (a1, a2) = new_fig(2, 2.8)
res["c"] = {}

for name, dm, tau, col, lab in [("error_raw", False, 0.0, RED, "D on error, no filter"),
                                ("error_filt", False, cfg.D_FILTER_TAU, ORANGE, "D on error + filter"),
                                ("meas", True, cfg.D_FILTER_TAU, GREEN, "fix: D on measurement")]:
    o = simulate(plant(), PID(KP, KI, KD, ts=TS, d_on_measurement=dm, d_filter_tau=tau),
                 sp, 3.0, ts=TS, substeps=5)
    mt = step_metrics(o["t"], o["y"], cfg.TARGET)
    a1.step(o["t"], o["u"], where="post", color=col, label=lab + f" (peak {o['u'].max():.0f} N)")
    a2.plot(o["t"], o["y"], color=col, label=lab)
    res["c"][name] = dict(**mt, peak_u=float(o["u"].max()), peak_d=float(o["d"].max()))

a1.set_xlim(-0.01, 0.12)
a1.set_xlabel("time [s]")
a1.set_ylabel("control force u [N]")
a1.set_title("(a) Initial force spike at setpoint step (0 \u2192 1 m)")
a1.legend(loc="upper right")

a2.axhline(cfg.TARGET, color="k", ls="--", lw=1, label="target 1 m")
a2.set_xlim(0, 1.5)
a2.set_ylim(0, 1.1)
a2.set_xlabel("time [s]")
a2.set_ylabel("block position x [m]")
a2.set_title("(b) Position trajectory comparison")
a2.legend(loc="lower right")

save_fig(fig, "fig07c_kick")

# =============================================================================
# (d) Controller Loop Update Rate (Sampling Time Ts)
# =============================================================================
def run_ts(ts, scale=1.0, T=8.0):
    return simulate(plant(), PID(KP * scale, KI * scale, KD * scale, ts=ts), sp, T, ts=ts,
                    substeps=max(5, int(round(ts / 0.002))))


def is_stable(ts, scale=1.0):
    o = run_ts(ts, scale, T=40.0)
    y = o["y"]
    n = len(y)
    if n < int(40.0 / ts) - 1:
        return False
    a = np.abs(y[n // 2: 3 * n // 4] - 1.0).max()
    b = np.abs(y[3 * n // 4:] - 1.0).max()
    return bool(b <= a * 1.0001 or b < 1e-3)


fig, (a1, a2) = new_fig(2, 2.8)
res["d"] = {"runs": {}}

for ts, col in zip([0.01, 0.05, 0.1, 0.2, 0.3], [GREEN, BLUE, PURPLE, ORANGE, RED]):
    o = run_ts(ts)
    mt = step_metrics(o["t"], o["y"], cfg.TARGET)
    a1.plot(o["t"], o["y"], color=col, label=f"Ts = {ts*1000:.0f} ms ({1/ts:.0f} Hz)")
    res["d"]["runs"][f"{ts}"] = dict(**mt, max_abs_y=float(np.abs(o["y"]).max()),
                                      stable=is_stable(ts))

a1.axhline(cfg.TARGET, color="k", ls="--", lw=1)
a1.set_xlim(0, 6)
a1.set_ylim(-0.5, 2.5)
a1.set_xlabel("time [s]")
a1.set_ylabel("block position x [m]")
a1.set_title("(a) Position response with decreasing sample rate")
a1.legend(loc="upper right", ncol=1)

lo, hi = 0.2, 0.4
for _ in range(10):
    mid = (lo + hi) / 2.0
    lo, hi = (mid, hi) if is_stable(mid) else (lo, mid)
res["d"]["critical_ts"] = (lo + hi) / 2.0

for ts_fix, scale, col, lab in [(0.01, 1.0, GREEN, "fast loop: Ts = 10 ms, nominal gains"),
                                (0.2, 1.0, RED, "slow loop: Ts = 200 ms, nominal gains"),
                                (0.2, 0.5, BLUE, "slow loop: Ts = 200 ms, gains scaled x 0.5")]:
    o = run_ts(ts_fix, scale)
    mt = step_metrics(o["t"], o["y"], cfg.TARGET)
    a2.plot(o["t"], o["y"], color=col, label=lab)
    res["d"][f"fix_{ts_fix}_{scale}"] = dict(**mt)

a2.axhline(cfg.TARGET, color="k", ls="--", lw=1, label="target 1 m")
a2.set_xlim(0, 6)
a2.set_ylim(0, 1.6)
a2.set_xlabel("time [s]")
a2.set_ylabel("block position x [m]")
a2.set_title("(b) Remedy for slow update rates: sample faster or reduce gains")
a2.legend(loc="lower right")

save_fig(fig, "fig07d_sample_time")
save_results("s07", res)
