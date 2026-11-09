"""
PID control library and physics simulation routines.

Implemented from first principles using only NumPy (no external control library).

Contents:
- PID: Reusable discrete PID controller with anti-windup, derivative filtering,
       and derivative-on-measurement.
- SpringMassDamper: Numerical simulation of m*x'' + c*x' + k*x = F using RK4.
- ThirdOrderLag: Numerical simulation of G(s) = 1 / (s+1)^3 using RK4.
- simulate: Closed-loop simulation connecting controller and plant with zero-order hold.
- simulate_open_loop: Open-loop plant response to an arbitrary input force.
- step_metrics: Step response evaluation (rise time, overshoot, settling time, steady-state error).
"""

import numpy as np


# -----------------------------------------------------------------------------
# 1. Discrete PID Controller
# -----------------------------------------------------------------------------
class PID:
    """Discrete PID controller with anti-windup, derivative filtering, and kick prevention.

    Control law:
        u_unsat[k] = P[k] + I[k] + D[k]
        u[k] = clip(u_unsat[k], u_min, u_max)

    Parameters:
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        ts: Controller sample time [s]
        u_min, u_max: Actuator saturation bounds (e.g., +-30 N)
        anti_windup: Anti-windup method ("none", "clamp", or "backcalc")
        tt: Back-calculation tracking time constant [s] (default: kp / ki)
        d_filter_tau: Derivative low-pass filter time constant [s] (0 = unfiltered)
        d_on_measurement: If True, differentiate plant output to eliminate kick;
                          if False, differentiate error.
    """

    def __init__(self, kp, ki=0.0, kd=0.0, ts=0.01, u_min=-np.inf, u_max=np.inf,
                 anti_windup="backcalc", tt=None, d_filter_tau=0.0,
                 d_on_measurement=True):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ts = ts
        self.u_min = u_min
        self.u_max = u_max
        self.anti_windup = anti_windup
        self.tt = tt if tt is not None else (kp / ki if ki > 0 else 1.0)
        self.tau = d_filter_tau
        self.d_on_meas = d_on_measurement
        self.reset()

    def reset(self, setpoint=0.0, measurement=0.0):
        """Reset internal integrator and state history."""
        self.integral = 0.0
        self.prev_error = setpoint - measurement
        self.prev_meas = measurement
        self.d_filt = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.u_unsat = 0.0

    def update(self, setpoint, measurement):
        """Compute the control signal u for the current time step."""
        e = setpoint - measurement

        # Proportional term (present error)
        self.p_term = self.kp * e

        # Derivative term (rate of change)
        if self.d_on_meas:
            d_raw = -(measurement - self.prev_meas) / self.ts
        else:
            d_raw = (e - self.prev_error) / self.ts

        # Low-pass filter on derivative (first-order discrete lag)
        if self.tau > 0:
            self.d_filt += (self.ts / (self.tau + self.ts)) * (d_raw - self.d_filt)
        else:
            self.d_filt = d_raw

        self.d_term = self.kd * self.d_filt

        # Integral term integration
        i_new = self.integral + self.ki * e * self.ts

        # Actuator saturation
        u_unsat = self.p_term + i_new + self.d_term
        u = min(max(u_unsat, self.u_min), self.u_max)

        # Anti-windup handling
        if self.anti_windup == "backcalc":
            i_new += (self.ts / self.tt) * (u - u_unsat)
        elif self.anti_windup == "clamp":
            if u != u_unsat and (e * u_unsat > 0):
                i_new = self.integral

        self.integral = i_new
        self.i_term = i_new
        self.u_unsat = u_unsat
        self.prev_error = e
        self.prev_meas = measurement

        return u


# -----------------------------------------------------------------------------
# 2. Plant Models
# -----------------------------------------------------------------------------
class SpringMassDamper:
    """Second-order plant: m*x'' + c*x' + k*x = F.
    Default: m = 1 kg, c = 10 N*s/m, k = 20 N/m. Output is position x [m].
    """

    def __init__(self, m=1.0, c=10.0, k=20.0, x0=0.0, v0=0.0):
        self.m = m
        self.c = c
        self.k = k
        self.x0 = x0
        self.v0 = v0
        self.reset()

    def reset(self):
        self.s = np.array([self.x0, self.v0], dtype=float)

    def output(self):
        return self.s[0]

    def _f(self, s, F):
        # s[0] = x, s[1] = v = x'
        # s'[0] = v, s'[1] = (F - c*v - k*x) / m
        return np.array([s[1], (F - self.c * s[1] - self.k * s[0]) / self.m])

    def step(self, F, dt):
        """RK4 integration step over duration dt."""
        s = self.s
        k1 = self._f(s, F)
        k2 = self._f(s + 0.5 * dt * k1, F)
        k3 = self._f(s + 0.5 * dt * k2, F)
        k4 = self._f(s + dt * k3, F)
        self.s = s + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


class ThirdOrderLag:
    """Third-order plant: G(s) = 1 / (s+1)^3.
    Consists of three identical 1/(s+1) first-order lags in series.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.s = np.zeros(3)

    def output(self):
        return self.s[2]

    @staticmethod
    def _f(s, u):
        return np.array([u - s[0], s[0] - s[1], s[1] - s[2]])

    def step(self, u, dt):
        """RK4 integration step over duration dt."""
        s = self.s
        k1 = self._f(s, u)
        k2 = self._f(s + 0.5 * dt * k1, u)
        k3 = self._f(s + 0.5 * dt * k2, u)
        k4 = self._f(s + dt * k3, u)
        self.s = s + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


# -----------------------------------------------------------------------------
# 3. Simulation Runners
# -----------------------------------------------------------------------------
def step_setpoint(value=1.0, t_step=0.0):
    """Setpoint generator that steps to `value` at time t_step."""
    return lambda t: value if t >= t_step else 0.0


def simulate(plant, ctrl, setpoint, t_end, ts=None, substeps=20,
             noise_amp=0.0, seed=0):
    """Run closed-loop simulation with zero-order hold controller.

    The controller updates at period `ts`. Between controller updates,
    the continuous physical plant is integrated using `substeps` RK4 steps.

    Parameters:
        plant: Plant model instance
        ctrl: PID controller instance
        setpoint: Callable returning target value at time t
        t_end: Total simulation duration [s]
        ts: Controller period [s] (defaults to ctrl.ts)
        substeps: Number of RK4 physics steps per controller interval
        noise_amp: Uniform noise amplitude in [-noise_amp, +noise_amp]
        seed: Random seed for reproducibility

    Returns:
        dict containing arrays: t, r, y, ym, u, p, i, d, u_unsat
    """
    ts = ctrl.ts if ts is None else ts
    n = int(round(t_end / ts))
    rng = np.random.default_rng(seed)
    noise = rng.uniform(-noise_amp, noise_amp, n) if noise_amp > 0 else np.zeros(n)

    plant.reset()
    ctrl.ts = ts
    ctrl.reset(setpoint(-1.0), plant.output())

    out = {k: np.zeros(n) for k in ("t", "r", "y", "ym", "u", "p", "i", "d", "u_unsat")}
    dt = ts / substeps

    for k in range(n):
        t = k * ts
        r = setpoint(t)
        y = plant.output()
        ym = y + noise[k]

        u = ctrl.update(r, ym)

        for key, val in zip(out.keys(), (t, r, y, ym, u, ctrl.p_term, ctrl.i_term,
                                         ctrl.d_term, ctrl.u_unsat)):
            out[key][k] = val

        for _ in range(substeps):
            plant.step(u, dt)

        if not np.isfinite(plant.output()) or abs(plant.output()) > 1e9:
            for key in out:
                out[key] = out[key][:k + 1]
            break

    return out


def simulate_open_loop(plant, force_fn, t_end, dt=0.001):
    """Drive plant open-loop with an input force profile."""
    n = int(round(t_end / dt))
    plant.reset()
    t = np.arange(n) * dt
    y = np.zeros(n)
    F = np.zeros(n)

    for k in range(n):
        F[k] = force_fn(t[k])
        y[k] = plant.output()
        plant.step(F[k], dt)

    return t, y, F


# -----------------------------------------------------------------------------
# 4. Performance Metrics
# -----------------------------------------------------------------------------
def step_metrics(t, y, target=1.0, band=0.02):
    """Calculate the four key step response metrics:
    - rise_time: Time from 10% to 90% of target value [s]
    - overshoot_pct: Peak percentage above target value [%]
    - settling_time: Time after which response stays within +-band of final value [s]
    - sse: Steady-state error (target - final value)
    """
    t, y = np.asarray(t), np.asarray(y)
    final = y[-1]

    def first_cross(level):
        idx = np.where(y >= level)[0]
        if len(idx) == 0:
            return np.nan
        j = idx[0]
        if j == 0:
            return t[0]
        return t[j - 1] + (level - y[j - 1]) / (y[j] - y[j - 1]) * (t[j] - t[j - 1])

    t10, t90 = first_cross(0.1 * target), first_cross(0.9 * target)
    rise = t90 - t10 if (np.isfinite(t10) and np.isfinite(t90)) else np.nan

    peak = y.max()
    overshoot = max(0.0, (peak - target) / target * 100.0)

    outside = np.where(np.abs(y - final) > band * abs(target))[0]
    if len(outside) == 0:
        settle = 0.0
    elif outside[-1] >= len(y) - 1:
        settle = np.nan
    else:
        settle = t[outside[-1] + 1]

    return dict(rise_time=rise, overshoot_pct=overshoot, settling_time=settle,
                sse=target - final, final=final, peak=peak,
                t_peak=t[int(np.argmax(y))], t10=t10, t90=t90)
