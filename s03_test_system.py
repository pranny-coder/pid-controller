"""
Section 3: The test system (1 kg block, damper, spring) open-loop dynamics.

Illustrates open-loop response to step forces and demonstrates why a continuous
force of k * target = 20 N is required to hold the block at 1 m against the spring.

Generates: figures/fig03_test_system.png, results/s03.json
"""

import numpy as np
from pid_lib import SpringMassDamper, simulate_open_loop
from plot_utils import new_fig, save_fig, save_results, BLUE, ORANGE, GREEN, RED
import config as cfg

m, c, k = cfg.M, cfg.C, cfg.K
poles = np.roots([m, c, k])
zeta = c / (2.0 * np.sqrt(m * k))
wn = np.sqrt(k / m)

fig, (a1, a2) = new_fig(2, 2.8)

# (a) Response to constant forces held indefinitely
for F, col in zip([5, 10, 20, 30], [BLUE, ORANGE, GREEN, RED]):
    t, y, _ = simulate_open_loop(SpringMassDamper(m, c, k), lambda t, F=F: F, 3.0)
    a1.plot(t, y, color=col, label=f"F = {F} N -> {F / k:.2f} m")
    a1.axhline(F / k, color=col, ls=":", lw=0.8)

a1.axhline(cfg.TARGET, color="k", ls="--", lw=1, label="target 1 m")
a1.set_title("(a) Constant force F held indefinitely")
a1.set_xlabel("time [s]")
a1.set_ylabel("block position x [m]")
a1.set_ylim(-0.05, 2.05)
a1.legend(loc="upper left", ncol=2, columnspacing=1.0)

# (b) Push with 20 N, then release force at t = 2 s
t, y, F = simulate_open_loop(SpringMassDamper(m, c, k), lambda t: 20.0 if t < 2.0 else 0.0, 4.0)
a2.plot(t, y, color=BLUE, label="position x [m]")
a2.set_xlabel("time [s]")
a2.set_ylabel("position x [m]")

a2b = a2.twinx()
a2b.plot(t, F, color=ORANGE, ls="--", label="force F [N]")
a2b.set_ylabel("force F [N]")
a2b.set_ylim(-2, 30)
a2b.grid(False)
a2b.spines["right"].set_visible(True)

l1, n1 = a2.get_legend_handles_labels()
l2, n2 = a2b.get_legend_handles_labels()
a2.legend(l1 + l2, n1 + n2, loc="center right")
a2.set_title("(b) 20 N force applied, released at t = 2 s")

save_fig(fig, "fig03_test_system")

# Time to reach 63.2% and 98% under 20 N force
t, y, _ = simulate_open_loop(SpringMassDamper(m, c, k), lambda t: 20.0, 4.0)
t63 = t[np.argmax(y >= 0.632)]
t98 = t[np.argmax(y >= 0.98)]

save_results("s03", dict(poles=poles, zeta=zeta, wn=wn, F_hold=k * cfg.TARGET,
                         t63=t63, t98=t98, final_20N=y[-1], dc_gain=1.0 / k))

print(f"System poles: {poles}, damping ratio zeta: {zeta:.3f}, natural frequency wn: {wn:.3f} rad/s")
print(f"t_63%: {t63:.3f} s, t_98%: {t98:.3f} s")
