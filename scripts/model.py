"""Radio tomographic forward model and Cramer-Rao bound used for every figure and number.

A body at point x perturbs link (i, j) by a sensitivity that is a Gaussian in the excess path
length rho = |x - n_i| + |x - n_j| - |n_i - n_j|, with width FRESNEL_SIGMA_SCALE * sqrt(lambda d).
The Fisher information at x sums the squared sensitivity gradients of all links, scaled by the
measured signal-to-noise ratio (SIGNAL_REL / NOISE_REL). Noise is independent across links and
there is one target, so every bound is optimistic.

SIGNAL_REL and NOISE_REL were measured on the ten-node mesh: the fractional amplitude
perturbation of an occupant and the fractional noise of a 144-minute confirmed-empty run.
Each constant can be overridden through the environment variable named after it.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import List, Tuple

import numpy as np

C_LIGHT = 299792458.0
CARRIER_HZ = float(os.environ.get("CRLB_CARRIER_HZ", "2.437e9"))
NOISE_REL = float(os.environ.get("CRLB_NOISE_REL", "0.039"))
SIGNAL_REL = float(os.environ.get("CRLB_SIGNAL_REL", "0.072"))
GRID_RES_M = float(os.environ.get("CRLB_GRID_RES_M", "0.10"))
FRESNEL_SIGMA_SCALE = float(os.environ.get("CRLB_FRESNEL_SIGMA_SCALE", "0.5"))
BLIND_THRESHOLD_M = float(os.environ.get("CRLB_BLIND_THRESHOLD_M", "1.0"))
"""A point whose bound exceeds this is called not localizable."""


def load_positions(path: Path) -> Tuple[List[str], np.ndarray]:
    raw = json.loads(path.read_text())
    ids = [m for m, p in raw.items() if p]
    pts = np.array([[float(p[0]), float(p[1])] for p in raw.values() if p])
    return ids, pts


def fim(grid: np.ndarray, nodes: np.ndarray) -> np.ndarray:
    lam = C_LIGHT / CARRIER_HZ
    J = np.zeros((grid.shape[0], 2, 2))
    n = len(nodes)
    for i in range(n):
        for j in range(i + 1, n):
            ni, nj = nodes[i], nodes[j]
            d_l = float(np.linalg.norm(ni - nj))
            if d_l <= 1e-6:
                continue
            s_f = FRESNEL_SIGMA_SCALE * np.sqrt(lam * d_l)
            vi, vj = grid - ni, grid - nj
            di = np.linalg.norm(vi, axis=1)
            dj = np.linalg.norm(vj, axis=1)
            ok = (di > 1e-6) & (dj > 1e-6)
            rho = di + dj - d_l
            w = np.exp(-(rho ** 2) / (2.0 * s_f ** 2))
            gr = np.zeros_like(grid)
            gr[ok] = vi[ok] / di[ok, None] + vj[ok] / dj[ok, None]
            g = gr * (w * (-rho / (s_f ** 2)))[:, None]
            J[:, 0, 0] += g[:, 0] ** 2
            J[:, 0, 1] += g[:, 0] * g[:, 1]
            J[:, 1, 1] += g[:, 1] ** 2
    J[:, 1, 0] = J[:, 0, 1]
    return J * (SIGNAL_REL ** 2) / (NOISE_REL ** 2)


def crlb(J: np.ndarray) -> np.ndarray:
    """RMS position bound in metres at each grid point (inf where J is singular)."""
    a, b, d = J[:, 0, 0], J[:, 0, 1], J[:, 1, 1]
    det = a * d - b * b
    tr = a + d
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(det > 0, np.sqrt(np.maximum(tr / det, 0.0)), np.inf)
