"""
Execute all section scripts sequentially to recreate all figures and results.

Run with:
    python run_all.py
"""

import runpy
import time

SCRIPTS = [
    "s01_loop_diagram",
    "s03_test_system",
    "s04_metrics",
    "s05_each_part",
    "s06_all_together",
    "s07_real_world",
    "s08_tuning",
    "s09_final_demo",
    "s10_interview_whatif"
]

t0 = time.time()
for name in SCRIPTS:
    print(f"\n===== Running {name}.py =====")
    runpy.run_path(f"{name}.py", run_name="__main__")

print(f"\nAll scripts executed successfully in {time.time() - t0:.1f} s.")
print("Generated plots are saved in 'figures/' and data in 'results/'.")
