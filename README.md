# PID Control - Intern Assignment Submission

Python implementation of a discrete PID controller written from first principles, along with full simulation scripts recreating every graph and numerical result required by the assignment.

Only **NumPy** (matrix and vector mathematics) and **Matplotlib** (figure generation) are used. No external control libraries (such as `scipy.signal` or `control`) are used.

---

## Quick Start

### 1. Installation
Install the required dependencies:
```bash
pip install -r requirements.txt
```

### 2. Run All Simulations
To recreate all figures and numerical results in one pass:
```bash
python run_all.py
```
This runs all section scripts sequentially (typically ~1 minute). Plots are saved to the `figures/` directory and numerical summaries to `results/`.

You can also run any section independently:
```bash
python s05_each_part.py
python s07_real_world.py
python s09_final_demo.py
```

---

## Project Structure & Section Mapping

| Section | Script | Generated Figures | Summary |
|---|---|---|---|
| **1. Basic Idea** | `s01_loop_diagram.py` | `fig01_loop_diagram.png` | Feedback loop diagram with everyday shower example. |
| **2. PID Formula** | `pid_lib.py` (`PID.update`) | — | Discrete PID algorithm with anti-windup, filtering, and kick prevention. |
| **3. Test System** | `s03_test_system.py` | `fig03_test_system.png` | Open-loop response of the 1 kg mass-spring-damper ($m=1$, $c=10$, $k=20$). Shows 20 N holding force. |
| **4. Metrics** | `s04_metrics.py` | `fig04_metrics.png` | Marks the 4 key measures (rise time, overshoot, settling time, steady-state error) on a single graph. |
| **5. Each Part Alone** | `s05_each_part.py` | `fig05a_P_sweep.png`<br>`fig05b_I_sweep.png`<br>`fig05c_D_sweep.png` | P-only sweep (compares $x_{ss} = \frac{K_p}{K_p+k}$ to simulation), adding I to eliminate offset, adding D to damp oscillations. |
| **6. All Together** | `s06_all_together.py` | `fig06_compare.png`<br>`fig06b_candidates.png` | Side-by-side comparison of P, PI, PD, and PID; evaluation of gain sets; selection of final gains ($K_p=100$, $K_i=100$, $K_d=10$). |
| **7. Real-World Issues** | `s07_real_world.py` | `fig07a_windup.png`<br>`fig07b_noise.png`<br>`fig07c_kick.png`<br>`fig07d_sample_time.png` | Demonstrates and resolves (a) 30 N motor saturation/windup, (b) $\pm 1\text{ mm}$ noise, (c) derivative kick on setpoint step, (d) slow sampling rates. |
| **8. Choosing Gains** | `s08_tuning.py` | `fig08a_ziegler_nichols_find_Ku.png`<br>`fig08b_ziegler_nichols_result.png` | Classical Ziegler-Nichols tuning on $G(s) = \frac{1}{(s+1)^3}$ (finds $K_u \approx 8$, $T_u \approx 3.63\text{ s}$), followed by refined tuning. |
| **9. Reusable Code** | `s09_final_demo.py` | `fig09_final_demo.png` | Comprehensive test comparing finalized robust PID against naive PID under noise, saturation, and multi-step inputs. |
| **10. Parameter Study** | `s10_interview_whatif.py` | `fig10_whatif.png` | Sensitivity analysis varying mass, damping, stiffness, noise, saturation limits, and gains. |

### Supporting Modules
* **`pid_lib.py`**: Implementation of `PID`, physical plant models (`SpringMassDamper`, `ThirdOrderLag`), simulation engines (`simulate`, `simulate_open_loop`), and evaluation metrics (`step_metrics`).
* **`config.py`**: Central repository for all system parameters, controller gains, physical constants, and sample times.
* **`plot_utils.py`**: Consistent plotting styles, headless backend setup, figure export, and result serialization routines.

---

## Controller Usage Example

```python
from pid_lib import PID, SpringMassDamper, simulate, step_setpoint

# Instantiate controller with practical fixes enabled
pid = PID(
    kp=100.0,
    ki=100.0,
    kd=10.0,
    ts=0.01,                 # 100 Hz sampling period
    u_min=-30.0,
    u_max=30.0,              # Actuator saturation limit (+-30 N)
    anti_windup="backcalc",  # Back-calculation anti-windup
    d_filter_tau=0.02,       # Derivative low-pass filter time constant
    d_on_measurement=True    # Differentiate output rather than error (prevents kick)
)

# Run closed-loop simulation on the test plant
plant = SpringMassDamper(m=1.0, c=10.0, k=20.0)
out = simulate(plant, pid, step_setpoint(1.0), t_end=5.0)

# Returned dictionary contains time histories:
# out["t"], out["r"], out["y"], out["ym"], out["u"], out["p"], out["i"], out["d"]
```

---

## Simulation Details

* **Test Plant**: Second-order continuous-time mass-spring-damper system:
  $$m \ddot{x} + c \dot{x} + k x = F$$
  with $m = 1.0\text{ kg}$, $c = 10.0\text{ N}\cdot\text{s}/\text{m}$, and $k = 20.0\text{ N}/\text{m}$.
* **Numerical Integration**: Continuous plant states are integrated using fourth-order Runge-Kutta (RK4) with multiple sub-steps per controller interval to ensure accuracy.
* **Discrete Execution**: The controller executes at fixed discrete intervals $T_s$ with a zero-order hold (ZOH) on the output, accurately mimicking real digital microcontroller loops.
* **Sample Times**:
  * Sections 3–6 utilize $T_s = 1\text{ ms}$ (quasi-continuous) so that discrete simulation exactly reflects continuous-time analytical derivations.
  * Sections 7–9 utilize $T_s = 10\text{ ms}$ (100 Hz), reflecting practical embedded controller update rates.

---

## Performance Metrics Definitions

| Metric | Definition |
|---|---|
| **Rise Time** | Time elapsed from 10% to 90% of the target setpoint ($r$). |
| **Overshoot** | Percentage by which peak response exceeds setpoint: $\max(0, \frac{y_{\max} - r}{r} \times 100\%)$. |
| **Settling Time** | The time after which the response remains within a $\pm 2\%$ band around its final steady value. |
| **Steady-State Error** | Final offset between target setpoint and settled value: $e_{ss} = r - y_{\text{final}}$. |
