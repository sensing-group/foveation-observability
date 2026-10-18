#!/usr/bin/env python3
"""FIGURE 4: the observability field of the ten-node room, with two contours.

Red: position bound above BLIND_THRESHOLD_M (not localizable).
Orange: presence-test deflection below DEFLECTION_THRESHOLD (not detectable).
The model and measured inputs come from model.py, the single source of the physics. The deflection for a body at a point is
sqrt(sum_l w_l^2) * (SIGNAL_REL / NOISE_REL), one sample per link.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.colors import LogNorm
import figstyle as fs
import model

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = ROOT / "results" / "fig_field.pdf"
DATA_OUT = ROOT / "results" / "field.json"
ROOM_W, ROOM_H = 5.0, 4.0
DEFLECTION_THRESHOLD = 3.0
INTEGRATION_SAMPLES = 40          # one second at the uniform 40 Hz of Section III


def load_model():
    mod = model
    macs, nodes = mod.load_positions(ROOT / "data" / "positions.json")
    return mod, macs, nodes


def deflection(mod, nodes, grid):
    lam = mod.C_LIGHT / mod.CARRIER_HZ
    n = len(nodes)
    acc = np.zeros(grid.shape[0])
    for i in range(n):
        for j in range(i + 1, n):
            d = np.linalg.norm(nodes[i] - nodes[j])
            sf = mod.FRESNEL_SIGMA_SCALE * np.sqrt(lam * d)
            rho = (np.linalg.norm(grid - nodes[i], axis=1)
                   + np.linalg.norm(grid - nodes[j], axis=1) - d)
            acc += np.exp(-rho ** 2 / (2 * sf ** 2)) ** 2
    return np.sqrt(acc) * (mod.SIGNAL_REL / mod.NOISE_REL)


def main() -> int:
    mod, macs, nodes = load_model()
    res = mod.GRID_RES_M
    xs = np.arange(0.0, ROOM_W + 1e-9, res)
    ys = np.arange(0.0, ROOM_H + 1e-9, res)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    grid = np.column_stack([X.ravel(), Y.ravel()])
    R = mod.crlb(mod.fim(grid, nodes)).reshape(len(xs), len(ys))
    D = deflection(mod, nodes, grid).reshape(len(xs), len(ys))
    notloc = R > mod.BLIND_THRESHOLD_M
    notdet = D < DEFLECTION_THRESHOLD
    Rn = R / np.sqrt(INTEGRATION_SAMPLES)
    Dn = D * np.sqrt(INTEGRATION_SAMPLES)
    stats = {
        "median_bound_cm": float(np.median(R) * 100), "p90_bound_cm": float(np.percentile(R, 90) * 100),
        "not_localizable_frac": float(notloc.mean()), "not_detectable_frac": float(notdet.mean()),
        "detectable_not_localizable_frac": float((notloc & ~notdet).mean()),
        "neither_frac": float((notloc & notdet).mean()),
        "one_second": {"not_localizable_frac": float((Rn > mod.BLIND_THRESHOLD_M).mean()),
                       "not_detectable_frac": float((Dn < DEFLECTION_THRESHOLD).mean()),
                       "detectable_not_localizable_frac": float(((Rn > mod.BLIND_THRESHOLD_M) & ~(Dn < DEFLECTION_THRESHOLD)).mean())},
        "deflection_threshold": DEFLECTION_THRESHOLD, "blind_threshold_m": mod.BLIND_THRESHOLD_M,
        "snr": mod.SIGNAL_REL / mod.NOISE_REL,
    }
    DATA_OUT.write_text(json.dumps(stats, indent=2))
    print(json.dumps(stats, indent=2))

    plt.rcParams.update(fs.rcparams())
    fig = plt.figure(figsize=(3.5, 2.75))
    fig.patch.set_facecolor("white")
    card = fs.card_grid(fig, 1)
    ax = card(0, "Observability field", padl=0.055, padr=0.075, padb=0.17, padt=0.15)
    im = ax.pcolormesh(X, Y, np.clip(R, 0.05, 10.0), cmap="viridis_r", shading="auto",
                       norm=LogNorm(vmin=0.1, vmax=10.0), rasterized=True)
    ax.contour(X, Y, notloc.astype(float), levels=[0.5], colors=[fs.RED], linewidths=1.3)
    ax.contour(X, Y, notdet.astype(float), levels=[0.5], colors=[fs.ORANGE],
               linewidths=1.1, linestyles="dashed")
    for m, (x, y) in zip(macs, nodes):
        ax.annotate(m[-5:].lstrip("0"), (x, y), xytext=(3, 3), textcoords="offset points",
                    fontsize=6.2, color="black", zorder=6,
                    path_effects=[pe.withStroke(linewidth=1.6, foreground="white")])
    ax.scatter(nodes[:, 0], nodes[:, 1], s=20, facecolor="white", edgecolor="black",
               linewidth=0.7, zorder=6)
    ax.set_xlim(0, ROOM_W); ax.set_ylim(0, ROOM_H)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)", fontsize=fs.FS_LABEL, labelpad=1)
    ax.set_ylabel("y (m)", fontsize=fs.FS_LABEL, labelpad=1)
    ax.tick_params(labelsize=fs.FS_TICK)
    pos = ax.get_position()
    cax = fig.add_axes([pos.x1 + 0.02, pos.y0, 0.025, pos.height])
    cb = fig.colorbar(im, cax=cax)
    cb.set_label("position bound (m, log)", fontsize=7.0, labelpad=2)
    cb.ax.tick_params(labelsize=6.0, length=2)
    fig.savefig(OUT, dpi=300)
    fig.savefig(OUT.with_suffix(".png"), dpi=200)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
