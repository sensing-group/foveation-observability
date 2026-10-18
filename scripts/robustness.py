"""Numbers of Secs. III and IV that no figure carries: robustness of the field and of the
allocation, the dark links at the room centre, and the airtime price of one exchange.

Writes results/robustness.json.
"""
from __future__ import annotations

import itertools
import json
from pathlib import Path

import numpy as np
from scipy.optimize import linprog

import model

ROOT = Path(__file__).resolve().parent.parent
DATA_OUT = ROOT / "results" / "robustness.json"

B = 200.0                         # exchanges per second
RATE_WALK = 46.0                  # Hz per required link
REQ_FRACTION = 0.80
ROOM_W, ROOM_H = 5.0, 4.0
LP_STEP_M = 0.10                  # room points for the allocation checks, as in Fig. 3
CENTRE = np.array([ROOM_W / 2, ROOM_H / 2])
FLOOR_SHARES = (0.0, 1 / 3, 0.5)  # every node's share as a fraction of uniform
PERTURB_SIGMA_M = 0.5
PERTURB_DRAWS = 40
PERTURB_SEED = 0
SNR_FACTOR = 0.5                  # "if that ratio halves"
ZERO_RATE = 1e-6

# 802.11 sounding exchange, microseconds (channel access, NDPA, NDP, two SIFS, compressed report)
EXCHANGE_PARTS_US = {"channel access": 110.5, "NDPA": 76.0, "NDP": 49.6, "2 x SIFS": 32.0,
                     "compressed report": 84.8}

_, NODES = model.load_positions(ROOT / "data" / "positions.json")
N = len(NODES)
LAM = model.C_LIGHT / model.CARRIER_HZ
LINKS = [(i, j) for i in range(N) for j in range(i + 1, N)]


def sens(x: np.ndarray, nodes: np.ndarray = NODES) -> np.ndarray:
    out = []
    for (i, j) in LINKS:
        d = np.linalg.norm(nodes[i] - nodes[j])
        sf = model.FRESNEL_SIGMA_SCALE * np.sqrt(LAM * d)
        rho = np.linalg.norm(x - nodes[i]) + np.linalg.norm(x - nodes[j]) - d
        out.append(np.exp(-rho ** 2 / (2 * sf ** 2)))
    return np.array(out)


def required_set(x: np.ndarray, frac: float = REQ_FRACTION):
    s = sens(x)
    order = np.argsort(-s)
    k = int(np.searchsorted(np.cumsum(s[order]) / s.sum(), frac) + 1)
    return [LINKS[o] for o in order[:k]]


def allocate(req, beta: float):
    """Max-min rate over the required links; returns (rate, per-node shares)."""
    c = np.zeros(N + 1)
    c[-1] = -1.0
    A, ub = [], []
    for (i, j) in req:
        row = np.zeros(N + 1)
        row[i] = row[j] = -1.0
        row[-1] = 1.0
        A.append(row)
        ub.append(0.0)
    Aeq = np.zeros((1, N + 1))
    Aeq[0, :N] = 1.0
    res = linprog(c, A_ub=np.array(A), b_ub=ub, A_eq=Aeq, b_eq=[B],
                  bounds=[(beta * B / N, None)] * N + [(0.0, None)], method="highs")
    return float(res.x[-1]), res.x[:N]


def room_grid(step: float):
    xs = np.arange(0, ROOM_W + 1e-9, step)
    ys = np.arange(0, ROOM_H + 1e-9, step)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    return np.column_stack([X.ravel(), Y.ravel()])


def fim_weighted(grid: np.ndarray, w: np.ndarray) -> np.ndarray:
    """Fisher information with each link's term weighted by its rate relative to uniform."""
    J = np.zeros((grid.shape[0], 2, 2))
    for (i, j), wl in zip(LINKS, w):
        ni, nj = NODES[i], NODES[j]
        d = np.linalg.norm(ni - nj)
        sf = model.FRESNEL_SIGMA_SCALE * np.sqrt(LAM * d)
        vi, vj = grid - ni, grid - nj
        di, dj = np.linalg.norm(vi, axis=1), np.linalg.norm(vj, axis=1)
        rho = di + dj - d
        g = (vi / di[:, None] + vj / dj[:, None]) * (np.exp(-rho ** 2 / (2 * sf ** 2)) * (-rho / sf ** 2))[:, None]
        J[:, 0, 0] += wl * g[:, 0] ** 2
        J[:, 0, 1] += wl * g[:, 0] * g[:, 1]
        J[:, 1, 1] += wl * g[:, 1] ** 2
    J[:, 1, 0] = J[:, 0, 1]
    return J * (model.SIGNAL_REL ** 2) / (model.NOISE_REL ** 2)


def main() -> int:
    out = {}
    G = room_grid(model.GRID_RES_M)
    blind = lambda R: float((R > model.BLIND_THRESHOLD_M).mean())
    R0 = model.crlb(model.fim(G, NODES))
    out["not_localizable_frac"] = blind(R0)
    # bound scales as 1/SNR: halving the SNR doubles every bound
    out["not_localizable_frac_snr_halved"] = blind(R0 / SNR_FACTOR)

    rng = np.random.default_rng(PERTURB_SEED)
    draws = [blind(model.crlb(model.fim(G, NODES + rng.normal(0, PERTURB_SIGMA_M, NODES.shape))))
             for _ in range(PERTURB_DRAWS)]
    out["positions_perturbed"] = {"sigma_m": PERTURB_SIGMA_M, "p10": float(np.percentile(draws, 10)),
                                  "median": float(np.median(draws)), "p90": float(np.percentile(draws, 90))}

    rate, b = allocate(required_set(CENTRE), 0.0)
    funded = b > ZERO_RATE
    out["room_centre"] = {"required_links": len(required_set(CENTRE)), "rate_hz": rate,
                          "uniform_rate_hz": 2 * B / N, "funded_nodes": int(funded.sum()),
                          "dark_links": int(sum(1 for (i, j) in LINKS if not funded[i] and not funded[j]))}

    field = {}
    for beta in FLOOR_SHARES + (1.0,):
        _, b = allocate(required_set(CENTRE), beta)
        w = np.array([(b[i] + b[j]) / (2 * B / N) for (i, j) in LINKS])
        field[f"{beta:.2f}"] = blind(model.crlb(fim_weighted(G, w)))
    out["field_under_centre_allocation_not_localizable"] = field

    pts = [np.array([x, y]) for x in np.arange(LP_STEP_M, ROOM_W, LP_STEP_M)
           for y in np.arange(LP_STEP_M, ROOM_H, LP_STEP_M)]
    floors = {}
    for beta in FLOOR_SHARES:
        rs, dark = [], []
        for p in pts:
            r, b = allocate(required_set(p), beta)
            rs.append(r)
            dark.append(sum(1 for (i, j) in LINKS if b[i] + b[j] < ZERO_RATE))
        floors[f"{beta:.2f}"] = {"pass_frac": float((np.array(rs) >= RATE_WALK).mean()),
                                 "median_dark_links": float(np.median(dark))}
    out["floor_shares_at_200"] = floors

    # directed links: each direction is its own series, so only the transmitter refreshes it;
    # the cheapest cover funds a node set that touches every required link
    subsets = sorted((s for k in range(1, N + 1) for s in itertools.combinations(range(N), k)), key=len)
    ks = []
    for p in pts:
        req = required_set(p)
        ks.append(next(len(s) for s in subsets if all(i in s or j in s for (i, j) in req)))
    ks = np.array(ks)
    out["directed"] = {"uniform_rate_hz": B / N, "pass_frac": float((B / ks >= RATE_WALK).mean())}

    total = sum(EXCHANGE_PARTS_US.values())
    out["exchange_us"] = {"parts": EXCHANGE_PARTS_US, "total": round(total, 1),
                          "without_report": round(total - EXCHANGE_PARTS_US["compressed report"], 1)}

    DATA_OUT.parent.mkdir(exist_ok=True)
    DATA_OUT.write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
