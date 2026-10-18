#!/usr/bin/env python3
"""FIGURE 3: where walking-speed motion is resolvable, against the budget.

Curves: the fraction of the room at which the best allocation reaches the walking
requirement (2 f_D = 46 Hz per required link), as a function of the sounding budget B in
exchanges per second, for three periphery floors:
  none      every node may be zeroed (pure fovea)
  half      every node keeps half its uniform share (periphery at B/T per node)
  uniform   every node keeps its uniform share (no fovea at all)
The deployed budget (50 triggers/s: 20 ms slots over ten nodes) is marked, and a second axis
prices B at the 802.11bf exchange cost (NDPA + NDP + report, 352.9 us).

Model: Fresnel sensitivity and node positions as in Fig. 4 (model.py);
required set = smallest link set carrying REQ_FRACTION of the sensitivity at a room point;
link (i, j) is refreshed whenever node i or node j transmits, so it runs at b_i + b_j;
allocation = max-min linear program over the required links. The rate is linear in B, so
each room point is solved once at B_REF and scaled.

Writes results/feasibility.json with every number the text quotes.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.optimize import linprog
import figstyle as fs
import model

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = ROOT / "results" / "fig_feasibility.pdf"
DATA_OUT = ROOT / "results" / "feasibility.json"

# ---- the numbers of Section III ----
V_WALK_MPS = 1.4
F_D_WALK = 23.0                    # Hz, 2 v / lambda at 2.437 GHz
RATE_WALK = 2 * F_D_WALK           # 46 Hz per required link
REQ_FRACTION = 0.80
B_REF = 200.0                      # LP solved here; rate is linear in B
B_DEPLOYED = 50.0                  # 20 ms slots x 10 nodes
EXCHANGE_US = 352.9                # NDPA + SIFS + NDP + SIFS + report, channel access included
FLOORS = {"no floor": 0.0, "half share kept": 0.5, "uniform": 1.0}
GRID_STEP_M = 0.10
ROOM_W, ROOM_H = 5.0, 4.0
B_MIN, B_MAX = 0.0, 300.0


def load_model():
    mod = model
    _, nodes = mod.load_positions(ROOT / "data" / "positions.json")
    return mod, nodes


def sensitivities(mod, nodes, x):
    lam = mod.C_LIGHT / mod.CARRIER_HZ
    n = len(nodes)
    links, s = [], []
    for i in range(n):
        for j in range(i + 1, n):
            d = np.linalg.norm(nodes[i] - nodes[j])
            sf = mod.FRESNEL_SIGMA_SCALE * np.sqrt(lam * d)
            rho = np.linalg.norm(x - nodes[i]) + np.linalg.norm(x - nodes[j]) - d
            links.append((i, j))
            s.append(np.exp(-rho ** 2 / (2 * sf ** 2)))
    return links, np.array(s)


def required_set(links, s):
    order = np.argsort(-s)
    cum = np.cumsum(s[order]) / s.sum()
    k = int(np.searchsorted(cum, REQ_FRACTION) + 1)
    return [links[o] for o in order[:k]]


def maxmin_rate(req, n, beta, B):
    c = np.zeros(n + 1)
    c[-1] = -1.0
    A, b = [], []
    for (i, j) in req:
        row = np.zeros(n + 1)
        row[i] = row[j] = -1.0
        row[-1] = 1.0
        A.append(row)
        b.append(0.0)
    Aeq = np.zeros((1, n + 1))
    Aeq[0, :n] = 1.0
    bounds = [(beta * B / n, None)] * n + [(0.0, None)]
    res = linprog(c, A_ub=np.array(A), b_ub=b, A_eq=Aeq, b_eq=[B],
                  bounds=bounds, method="highs")
    return float(res.x[-1])


def main() -> int:
    mod, nodes = load_model()
    n = len(nodes)
    pts = [np.array([x, y]) for x in np.arange(GRID_STEP_M, ROOM_W, GRID_STEP_M)
           for y in np.arange(GRID_STEP_M, ROOM_H, GRID_STEP_M)]
    rates = {}
    for name, beta in FLOORS.items():
        r = []
        for p in pts:
            links, s = sensitivities(mod, nodes, p)
            r.append(maxmin_rate(required_set(links, s), n, beta, B_REF))
        rates[name] = np.array(r)

    B = np.linspace(B_MIN, B_MAX, 301)
    curves = {name: np.array([(r * b / B_REF >= RATE_WALK).mean() for b in B])
              for name, r in rates.items()}

    def budget_for(name, frac):
        """Smallest B at which the room fraction reaches frac."""
        c = curves[name]
        idx = np.argmax(c >= frac) if (c >= frac).any() else None
        return None if idx is None else float(B[idx])

    out = {
        "rate_walk_hz": RATE_WALK, "req_fraction": REQ_FRACTION, "b_deployed": B_DEPLOYED,
        "exchange_us": EXCHANGE_US, "n_nodes": n, "uniform_rate_deployed_hz": 2 * B_DEPLOYED / n,
        "b_uniform_pass": n * F_D_WALK,
        "airtime_pct": {str(b): round(b * EXCHANGE_US * 1e-4, 2) for b in (B_DEPLOYED, 92, 184, 200, 230)},
        "pass_fraction_at": {name: {str(b): round(float((r * b / B_REF >= RATE_WALK).mean()), 3)
                                    for b in (B_DEPLOYED, 92, 184, 200, 230)}
                             for name, r in rates.items()},
        "budget_for_half_room": {name: budget_for(name, 0.5) for name in FLOORS},
        "budget_for_90pct_room": {name: budget_for(name, 0.9) for name in FLOORS},
        "median_rate_at_200": {name: float(np.median(r)) for name, r in rates.items()},
    }
    DATA_OUT.write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))

    plt.rcParams.update(fs.rcparams())
    fig = plt.figure(figsize=(3.5, 2.7))
    fig.patch.set_facecolor("white")
    card = fs.card_grid(fig, 1)
    ax = card(0, "Walking-speed tracking against the budget",
              padl=0.052, padr=0.012, padb=0.21, padt=0.27)
    styles = {"no floor": (fs.BLUE, "-"), "half share kept": (fs.ORANGE, "-"),
              "uniform": (fs.GREY, "--")}
    for name, c in curves.items():
        col, ls = styles[name]
        ax.plot(B, 100 * c, color=col, ls=ls, lw=1.4, label=name)
    ax.axvline(B_DEPLOYED, color=fs.RED, lw=0.9, ls=":")
    ax.text(B_DEPLOYED + 4, 92, "deployed\n50/s", fontsize=fs.FS_ANNOT, color=fs.RED, va="top")
    ax.set_xlim(B_MIN, B_MAX)
    ax.set_ylim(0, 100)
    ax.set_xlabel("sounding budget B (exchanges per second)", fontsize=fs.FS_LABEL)
    ax.set_ylabel("room area, %", fontsize=fs.FS_LABEL)
    ax.legend(fontsize=fs.FS_LEGEND, frameon=False, loc="center right")
    top = ax.secondary_xaxis("top", functions=(lambda b: b * EXCHANGE_US * 1e-4,
                                               lambda a: a / (EXCHANGE_US * 1e-4)))
    top.set_xlabel("airtime at the 802.11bf exchange cost, %", fontsize=fs.FS_LABEL)
    top.tick_params(labelsize=fs.FS_TICK)
    fig.savefig(OUT)
    fig.savefig(OUT.with_suffix(".png"), dpi=200)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
