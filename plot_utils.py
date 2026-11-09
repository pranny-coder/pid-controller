"""
Plotting and result logging utilities.

Uses Matplotlib (configured with Agg backend for headless operation) and JSON for results.
"""

import json
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.abspath(__file__))
FIG_DIR = os.path.join(ROOT, "figures")
RES_DIR = os.path.join(ROOT, "results")
os.makedirs(FIG_DIR, exist_ok=True)
os.makedirs(RES_DIR, exist_ok=True)

# Standard styling palette
BLUE, ORANGE, GREEN, RED = "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"
PURPLE, BROWN, GREY, BLACK = "#9467bd", "#8c564b", "#7f7f7f", "#222222"
PALETTE = [BLUE, ORANGE, GREEN, RED, PURPLE, BROWN]

plt.rcParams.update({
    "font.size": 8,
    "axes.titlesize": 9,
    "axes.labelsize": 8,
    "legend.fontsize": 6.8,
    "xtick.labelsize": 7.5,
    "ytick.labelsize": 7.5,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "lines.linewidth": 1.5,
    "figure.dpi": 100,
    "axes.spines.top": False,
    "axes.spines.right": False,
})


def new_fig(ncols=2, height=2.7, width=7.2, **kw):
    fig, axes = plt.subplots(1, ncols, figsize=(width, height), **kw)
    return fig, axes


def save_fig(fig, name):
    path = os.path.join(FIG_DIR, name + ".png")
    fig.tight_layout()
    fig.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(fig)
    print("Saved figure:", path)


def _clean(o):
    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, (np.floating, float)):
        return None if not np.isfinite(o) else float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.ndarray):
        return _clean(o.tolist())
    return o


def save_results(name, obj):
    """Save numerical results to JSON in the results directory."""
    path = os.path.join(RES_DIR, name + ".json")
    with open(path, "w") as f:
        json.dump(_clean(obj), f, indent=1)


def fmt(v, nd=2, unit=""):
    """Format floating point values, handling NaN gracefully."""
    return "never" if v is None or not np.isfinite(v) else f"{v:.{nd}f}{unit}"
