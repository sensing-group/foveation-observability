#!/usr/bin/env python3
"""FIGURE 5: one evening. The presence estimate against the movement evidence it had.

Episode of 2026-08-04 (local time, UTC+2): entry 19:34, hold through a quiet movement
channel, release 21:50 by accumulated absence. Reads data/episode.json (per-bin rows:
ts, p_present, reported_occupied, z_roll, s_absence).
"""
from __future__ import annotations

import datetime as dt
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
import figstyle as fs

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = ROOT / "results" / "fig_episode.pdf"
DATA_OUT = ROOT / "results" / "episode_stats.json"
TZ = dt.timezone(dt.timedelta(hours=2))
SMOOTH_BINS = 31
GREEN = "#009E73"


def main() -> int:
    ep = json.loads((ROOT / "data" / "episode.json").read_text())
    t0 = ep[0]["ts"]
    hrs = np.array([(r["ts"] - t0) / 3600.0 for r in ep])
    present = np.array([1.0 if (r.get("p_present") or 0) >= 0.5 else 0.0 for r in ep])
    zroll = np.array([float(r.get("z_roll") or 0.0) for r in ep])
    occ = [r for r in ep if r.get("reported_occupied")]
    t_entry, t_release = occ[0]["ts"], occ[-1]["ts"]
    start_label = dt.datetime.fromtimestamp(t0, TZ).strftime("%H:%M")
    entry_label = dt.datetime.fromtimestamp(t_entry, TZ).strftime("%H:%M")
    release_label = dt.datetime.fromtimestamp(t_release, TZ).strftime("%H:%M")
    empty_p90 = float(np.percentile(zroll[present == 0], 90))
    frac_quiet = float((zroll[present == 1] < empty_p90).mean())
    stats = {"start_local": start_label, "entry_local": entry_label, "release_local": release_label,
             "hold_hours": (t_release - t_entry) / 3600.0, "empty_p90": empty_p90,
             "held_bins_below_empty_p90": frac_quiet}
    DATA_OUT.write_text(json.dumps(stats, indent=2))
    print(json.dumps(stats, indent=2))

    plt.rcParams.update(fs.rcparams())
    fig = plt.figure(figsize=(3.5, 2.1))
    fig.patch.set_facecolor("white")
    card = fs.card_grid(fig, 1)
    ax = card(0, "Presence estimate over one evening", padl=0.062, padr=0.085, padb=0.20, padt=0.19)
    ax.fill_between(hrs, 0, present, step="mid", color=fs.BLUE, alpha=0.12, linewidth=0)
    ax.plot(hrs, present, drawstyle="steps-mid", color=fs.BLUE, lw=1.4)
    ax.set_ylim(-0.04, 1.42)
    ax.set_yticks([0, 1]); ax.set_yticklabels(["empty", "present"], fontsize=7.2)
    ax.set_xlabel(f"hours since {start_label} (local)", fontsize=fs.FS_LABEL, labelpad=1)
    ax.tick_params(labelsize=fs.FS_TICK)
    ax.set_xlim(hrs.min(), hrs.max())
    ker = np.ones(SMOOTH_BINS) / SMOOTH_BINS
    zsm = np.convolve(np.clip(zroll, 0, 12), ker, mode="same")
    axm = ax.twinx()
    axm.fill_between(hrs, 0, zsm, color=fs.ORANGE, alpha=0.42, linewidth=0)
    axm.plot(hrs, zsm, color=fs.ORANGE, lw=0.8)
    axm.set_ylabel("movement evidence", fontsize=7.2, color=fs.ORANGE, labelpad=1)
    axm.tick_params(labelsize=6.6, colors=fs.ORANGE)
    axm.axhline(empty_p90, color=fs.GREY, lw=0.8, ls=(0, (4, 2)), zorder=4)
    axm.text(hrs.max() - 0.06, empty_p90 + 0.25, "empty-room p90", fontsize=6.8, color=fs.GREY,
             ha="right", va="bottom", path_effects=[pe.withStroke(linewidth=1.8, foreground="white")])
    axm.set_ylim(0, 9)
    for ts, lab, col, ha, x in [
            (t_entry, f"entry {entry_label}", GREEN, "left", (t_entry - t0) / 3600 + 0.05),
            (t_release, f"released {release_label}", fs.RED, "right", hrs.max() - 0.04)]:
        th = (ts - t0) / 3600.0
        ax.axvline(th, color=col, lw=1.0, ls="--", zorder=5)
        ax.text(x, 1.38, lab, fontsize=7.0, color=col, va="top", ha=ha, zorder=9)
    fig.savefig(OUT, dpi=300)
    fig.savefig(OUT.with_suffix(".png"), dpi=200)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
