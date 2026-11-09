"""
Section 1: The basic feedback loop diagram.

Generates: figures/fig01_loop_diagram.png
"""

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle
from plot_utils import save_fig, BLUE, ORANGE, GREEN, GREY, BLACK, RED


def box(ax, x, y, w, h, text, color):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.08",
                                fc=color, ec=BLACK, lw=1.2, alpha=0.9))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=8.5,
            weight="bold", color="white")


def arrow(ax, p0, p1, color=BLACK):
    ax.annotate("", xy=p1, xytext=p0,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.6, shrinkA=0, shrinkB=0))


fig, ax = plt.subplots(figsize=(7.2, 2.9))
ax.set_xlim(0, 12.2)
ax.set_ylim(-0.2, 4.0)
ax.axis("off")
ax.grid(False)
ax.set_title("The feedback loop: target \u2192 controller \u2192 machine \u2192 sensor \u2192 back to start",
             loc="left", fontsize=9)

cy = 2.5
ax.text(0.3, cy + 0.55, "Setpoint r\n(target)", ha="center", fontsize=8)
arrow(ax, (0.0, cy), (1.55, cy))
ax.add_patch(Circle((1.85, cy), 0.3, fc="white", ec=BLACK, lw=1.4))
ax.text(1.85, cy, "+", ha="center", va="center", fontsize=11, weight="bold")
ax.text(2.1, cy - 0.62, "-", ha="center", va="center", fontsize=16, weight="bold", color=RED)
arrow(ax, (2.15, cy), (3.35, cy))
ax.text(2.75, cy + 0.2, "error\ne = r - y", ha="center", va="bottom", fontsize=7.5, color=RED)

box(ax, 3.4, cy - 0.5, 2.3, 1.0, "CONTROLLER\n(P + I + D)", BLUE)
arrow(ax, (5.7, cy), (7.0, cy))
ax.text(6.35, cy + 0.2, "command\nu (force)", ha="center", va="bottom", fontsize=7.5)

box(ax, 7.0, cy - 0.5, 2.3, 1.0, "MACHINE\n(the plant)", ORANGE)
arrow(ax, (9.3, cy), (11.0, cy))
ax.text(11.1, cy + 0.2, "actual output y\n(position)", ha="left", va="bottom", fontsize=8)

# Feedback loop path
ax.plot([10.2, 10.2], [cy, 0.9], color=BLACK, lw=1.6)
box(ax, 7.0, 0.4, 2.3, 1.0, "SENSOR\n(measures y)", GREEN)
arrow(ax, (10.2, 0.9), (9.3, 0.9))
ax.plot([7.0, 1.85], [0.9, 0.9], color=BLACK, lw=1.6)
arrow(ax, (1.85, 0.9), (1.85, cy - 0.3))
ax.text(4.4, 1.0, "measured value fed back", ha="center", va="bottom", fontsize=7.5, color=GREY)

# Everyday shower example
kw = dict(ha="center", fontsize=7.3, style="italic", color=GREY)
ax.text(0.3, cy - 0.75, "Shower: target\n= 38 \u00b0C", **kw)
ax.text(4.55, 1.6, "brain: 'too cold,\nturn tap more'", **kw)
ax.text(8.15, 3.35, "tap valve + water pipe", **kw)
ax.text(8.15, -0.05, "hand senses\ntemperature", **kw)

save_fig(fig, "fig01_loop_diagram")
