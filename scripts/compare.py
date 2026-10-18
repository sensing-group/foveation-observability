"""Compare results/*.json against reference/*.json; exit 1 on any difference beyond TOLERANCE."""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TOLERANCE = 1e-9


def walk(a, b, path=""):
    if isinstance(a, dict):
        if set(a) != set(b):
            yield f"{path}: keys differ"
            return
        for k in a:
            yield from walk(a[k], b[k], f"{path}.{k}")
    elif isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if abs(a - b) > TOLERANCE * max(1.0, abs(b)):
            yield f"{path}: {a} vs reference {b}"
    elif a != b:
        yield f"{path}: {a!r} vs reference {b!r}"


def main() -> int:
    bad = []
    for ref in sorted((ROOT / "reference").glob("*.json")):
        res = ROOT / "results" / ref.name
        if not res.exists():
            bad.append(f"{ref.name}: missing from results/")
            continue
        diffs = list(walk(json.loads(res.read_text()), json.loads(ref.read_text())))
        bad += [f"{ref.name}{d}" for d in diffs]
        print(f"{ref.name}: {'match' if not diffs else f'{len(diffs)} differences'}")
    for line in bad:
        print("  " + line)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
