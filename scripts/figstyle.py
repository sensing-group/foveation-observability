"""Shared visual language for every figure in this article.

Both figure scripts import from here, so the panels cannot drift apart. The
idiom is tinted
cards, a coloured accent bar, a bold coloured header, all in the single blue
family of Table I (header #2B4C9B, fill #EFF3FC) so table and figures read as
one theme, and a restrained data
palette inside.

One rule per element:
  card fill / accent  -> position in the figure, never meaning
  BLUE                -> sounded links and estimates
  ORANGE              -> the occupant, and events on a time axis
  RED                 -> what could not be measured
  GREY                -> references, baselines, neutral guides
"""
from __future__ import annotations

import matplotlib.pyplot as plt

# ---- card chrome, in order of appearance across a figure ----
CARD = ["#F7F9FE", "#EFF3FC", "#E3EAFB"]
ACC = ["#2B4C9B", "#2B4C9B", "#2B4C9B"]

# ---- data palette, colourblind-safe (Wong) ----
BLUE = "#0072B2"
ORANGE = "#E69F00"
RED = "#D55E00"
GREY = "#4D4D4D"

# ---- type scale, identical in every figure ----
FS_HEADER = 9.2
FS_LABEL = 8.2
FS_TICK = 7.4
FS_ANNOT = 7.2
FS_LEGEND = 7.0


def rcparams() -> dict:
    return {
        "font.family": "sans-serif", "font.sans-serif": ["DejaVu Sans"],
        "font.size": 8, "axes.linewidth": 0.7,
        "xtick.major.width": 0.7, "ytick.major.width": 0.7,
        "pdf.fonttype": 42,
    }


def card_grid(fig, n, *, margin=0.010, gap=0.016, y0=0.035, h=0.945):
    """Return a function placing card i of n and giving back its axes.

    Card bounds are computed from the grid rather than from axes bounding
    boxes: panels carry different label widths, and deriving the card from the
    axes made them different sizes.
    """
    cw = (1.0 - 2 * margin - (n - 1) * gap) / n

    def card(i, title, *, padl=0.068, padr=0.016, padb=0.165, padt=0.145):
        x0 = margin + i * (cw + gap)
        fig.patches.append(plt.Rectangle(
            (x0, y0), cw, h, transform=fig.transFigure, facecolor=CARD[i % 3],
            edgecolor=ACC[i % 3], linewidth=0.8, zorder=-10))
        fig.patches.append(plt.Rectangle(
            (x0 + 0.012, y0 + h - 0.105), 0.006, 0.062,
            transform=fig.transFigure, facecolor=ACC[i % 3], edgecolor="none",
            zorder=3))
        fig.text(x0 + 0.026, y0 + h - 0.098, title, fontsize=FS_HEADER,
                 fontweight="bold", color=ACC[i % 3], va="bottom", ha="left")
        ax = fig.add_axes([x0 + padl * cw / 0.32, y0 + padb,
                           cw - (padl + padr) * cw / 0.32, h - padb - padt])
        ax.set_zorder(5)
        ax.patch.set_alpha(1.0)
        ax.set_facecolor("white")
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
        for sp in ("left", "bottom"):
            ax.spines[sp].set_color("0.55")
            ax.spines[sp].set_linewidth(0.7)
        ax.tick_params(labelsize=FS_TICK)
        return ax

    return card
