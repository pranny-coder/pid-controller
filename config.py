"""
Configuration parameters for the PID controller experiments and simulations.
All system properties, controller gains, and real-world limits are defined here.
"""

# --- Test system (given): 1 kg block, damper, spring ---
M = 1.0            # mass            [kg]
C = 10.0           # damping         [N*s/m]
K = 20.0           # spring constant [N/m]
TARGET = 1.0       # setpoint        [m]

# --- Controller sample times ---
TS_FAST = 0.001    # 1 kHz : continuous-like simulation for sections 3-6
TS_REAL = 0.01     # 100 Hz: realistic microcontroller loop for sections 7-9

# --- Final gains chosen in section 6 ---
KP = 100.0         # proportional gain [N/m]
KI = 100.0         # integral gain     [N/(m*s)]
KD = 10.0          # derivative gain   [N*s/m]
D_FILTER_TAU = 0.02 # derivative filter time constant [s]

# --- Real-world constraints (section 7) ---
F_MAX = 30.0       # actuator saturation limit [+-30 N]
NOISE = 0.001      # sensor measurement noise  [+-1 mm]
SEED = 0           # random seed for reproducible noise
