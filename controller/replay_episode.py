"""Replay the absence release on the 4 August 2026 evening from the logged per-bin inputs.

The entry is taken from the log (entry detection is not part of the extract). From the bin after
it, the ledger recomputes the absence score and the release from evidence alone, and every bin
is compared with the score and reason the deployed system logged.

Writes results/episode_replay.json.
"""
from __future__ import annotations

import datetime as dt
import json
import sys
from pathlib import Path

from release import AbsenceLedger, Bin

ROOT = Path(__file__).resolve().parent.parent
INPUTS = ROOT / "data" / "episode_inputs.jsonl"
DATA_OUT = ROOT / "results" / "episode_replay.json"
LOCAL = dt.timezone(dt.timedelta(hours=2))      # CEST, the evening's local time
LOGGED_DIGITS = 4                                # the log rounds the score to 4 decimals


def hhmmss(ts: float) -> str:
    return dt.datetime.fromtimestamp(ts, LOCAL).strftime("%H:%M:%S")


def main() -> int:
    rows = [json.loads(l) for l in INPUTS.read_text().splitlines()]
    led = AbsenceLedger()
    mismatches, release_ts, last_zero_ts, entry_ts = [], None, None, None
    for r in rows:
        if r["event"] == "entry":
            led.enter()
            entry_ts = r["ts"]
            continue
        if not led.present:
            continue
        event, why = led.step(Bin(r["ts"], r["z_roll"], r["z_mesh"], r["bfi_agitation_z"],
                                  r["bfi_n_reporting"], r["sounding_density"],
                                  bool(r["illuminated"])))
        if event:
            release_ts = r["ts"]
            if r["event"] != event:
                mismatches.append((hhmmss(r["ts"]), "event", event, r["event"]))
            break
        if round(led.s_absence, LOGGED_DIGITS) != round(r["s_absence"] or 0.0, LOGGED_DIGITS) \
                or why != (r["absence_reason"] or ""):
            mismatches.append((hhmmss(r["ts"]), led.s_absence, why, r["s_absence"], r["absence_reason"]))
        if led.s_absence == 0.0:
            last_zero_ts = r["ts"]
    logged_release = next(r["ts"] for r in rows if r["event"] == "absence_confirmed")
    out = {"entry": hhmmss(entry_ts), "release_replayed": hhmmss(release_ts),
           "release_logged": hhmmss(logged_release), "last_zero": hhmmss(last_zero_ts),
           "climb_minutes": round((release_ts - last_zero_ts) / 60, 1),
           "hold_hours": round((release_ts - entry_ts) / 3600, 2),
           "bins_compared": sum(1 for r in rows if entry_ts < r["ts"] <= release_ts),
           "mismatches": len(mismatches)}
    DATA_OUT.parent.mkdir(exist_ok=True)
    DATA_OUT.write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))
    for m in mismatches[:20]:
        print("  mismatch", m)
    return 0 if not mismatches and release_ts == logged_release else 1


if __name__ == "__main__":
    sys.exit(main())
