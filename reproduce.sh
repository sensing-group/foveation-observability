#!/usr/bin/env sh
# Regenerates every figure and number of the article into results/, replays the case-study
# release from its logged inputs, and compares every number against reference/, the files the
# article was written from.
set -e
cd "$(dirname "$0")"
mkdir -p results
for s in make_fig_feasibility make_fig_field make_fig_episode robustness; do
  echo "== $s"
  python scripts/$s.py > results/$s.log
done
python scripts/required_set_lp.py > results/required_set_lp.log
python scripts/allocation_floor_sweep.py > results/allocation_floor_sweep.log
echo "== replay_episode"
(cd controller && python replay_episode.py > ../results/replay_episode.log)
python scripts/compare.py
