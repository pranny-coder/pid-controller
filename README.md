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

---

## Technical Write-Up: Approach, What Worked, What Didn't & Next Steps

### 1. Approach
* **First-Principles Modeling**: Developed the physical plants ($m \ddot{x} + c \dot{x} + k x = F$ and $G(s) = \frac{1}{(s+1)^3}$) and simulation engine strictly using Python and NumPy, avoiding third-party control toolboxes. Continuous physics is integrated using 4th-order Runge-Kutta (RK4) with fine sub-stepping ($dt = T_s / 5$).
* **Discrete Zero-Order Hold Architecture**: The PID controller executes at discrete intervals ($T_s = 10\text{ ms}$ for 100 Hz loops) holding output constant between updates, accurately replicating real-world embedded firmware.
* **Systematic Evaluation**: Progressed from analytical baseline derivations (P-only steady-state error $x_{ss} = \frac{K_p}{K_p+k}$) to component sweeps, side-by-side trade-off matrices, and real-world constraint mitigation.

### 2. What Worked
* **Back-Calculation Anti-Windup**: Eliminates integrator buildup during $\pm 30\text{ N}$ actuator saturation. It dropped overshoot from $13.8\%$ to $0.00\%$ and accelerated settling time by $2.26\times$ (from $1.83\text{ s}$ down to $0.81\text{ s}$).
* **Derivative-on-Measurement**: Differentiating the plant output rather than the error signal completely abolished the destructive $1000\text{ N}$ derivative kick spike during setpoint steps without compromising rise time.
* **First-Order Derivative Filtering**: Passing the derivative through a discrete low-pass filter ($\tau = 0.02\text{ s}$) suppressed actuator force jitter by $70\%$ (from $0.946\text{ N}$ down to $0.285\text{ N}$ standard deviation) under $\pm 1\text{ mm}$ sensor noise.
* **Refined Ziegler-Nichols Tuning**: Slashed classical Z-N overshoot on $G(s) = 1/(s+1)^3$ from $55.1\%$ down to $4.8\%$, settling $2.3\times$ faster ($6.1\text{ s}$ vs $14.1\text{ s}$).

### 3. What Didn't (and Challenges Overcome)
* **Naive Textbook PID Formulation**: An unaugmented textbook PID failed when physical constraints were introduced, causing severe motor chatter under sensor noise and massive overshoot when saturated.
* **Classical Ziegler-Nichols Aggressiveness**: Standard Z-N tuning formulas target a quarter-decay ratio ($25\%$ decay per cycle), which produced unacceptable overshoot ($>50\%$) for precision robotics. Parameter refinement (reducing $K_p$, increasing $T_i$, and tuning $T_d$) was necessary to achieve well-damped response.
* **Sampling Rate Limits**: Discovered that when controller update intervals exceed $T_s > 200\text{ ms}$, phase lag induces instability (critical threshold $T_{\text{crit}} \approx 255\text{ ms}$). For slow loops, scaling gains down by $50\%$ was required to restore stable margins.

### 4. What I Would Explore Next With More Time
* **Feedforward Control**: Implement gravity/friction feedforward ($u_{ff} = m \ddot{r} + c \dot{r} + k r$) combined with trapezoidal or S-curve trajectory profiling to eliminate reliance on error-driven feedback alone.
* **State Estimation (Kalman Filter)**: Replace the first-order derivative low-pass filter with a Linear Quadratic Estimator (Kalman Filter) to fuse noisy position measurements and estimate true velocity states.
* **Gain Scheduling & Adaptive Control**: Dynamically adapt $K_p, K_i, K_d$ as operating points change to handle non-linear dynamics (e.g., non-linear spring deflection, aerodynamic drag).
* **Model Predictive Control (MPC)**: Formulate multi-state constrained optimization to systematically handle state and input boundaries across coupled robotic joints.

