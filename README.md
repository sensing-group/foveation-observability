# Foveation and observability: code and data

Code and data behind the figures and numbers of K. Khamaisi and B. Rodrigues, "Foveation and
Observability in Next-Generation Wireless Sensing Networks", IEEE Communications Magazine
(under revision). Every number the article quotes from geometry or from the case study is
regenerated here from two input files.

## Reproduce

```
pip install -r requirements.txt
./reproduce.sh
```

`reproduce.sh` writes figures, logs and JSON files to `results/` and compares every JSON number
against `reference/`, the files the article was written from. It runs in about a minute on a
laptop. Tested with Python 3.12 in a clean container.

## What produces what

| Article | Script | Output |
|---|---|---|
| Fig. 3, the feasibility test against the budget (Sec. III-C) | `make_fig_feasibility.py` | `fig_feasibility.pdf`, `feasibility.json` |
| Fig. 4, the observability field and the detectability contour (Secs. III-B, V) | `make_fig_field.py` | `fig_field.pdf`, `field.json` |
| Fig. 5, the evening of 4 August 2026 (Sec. V) | `make_fig_episode.py` | `fig_episode.pdf`, `episode_stats.json` |
| Required set and best split at the room centre; floor shares; directed links; position perturbation; field under the centre-aimed allocation; halved SNR; exchange airtime (Secs. III, IV) | `robustness.py` | `robustness.json` |
| Required set and allocation program over the room, two sensitivity metrics | `required_set_lp.py` | `required_set_lp.log` |
| Threshold and floor sweeps, detectability against localizability | `allocation_floor_sweep.py` | `allocation_floor_sweep.log` |

`model.py` holds the forward model and the Cramer-Rao bound shared by all scripts; its
docstring states the assumptions.

## Data

- `data/positions.json`: the ten node positions in metres, from the mesh's own position solver
  (not a survey), keyed by the last two bytes of each node's MAC address, as printed in Fig. 4.
- `data/episode.json`: the evening of 4 August 2026, 19:00 to 23:00 local time (UTC+2), one row
  per 10 s bin. `ts` is the bin start in Unix seconds, `p_present` the presence belief,
  `reported_occupied` the reported state, `z_roll` the movement statistic (null before the
  per-link references are warm), `s_absence` the accumulated absence score that triggers release.

Two inputs come from the deployment and are not regenerated here: the signal and noise levels in
`model.py` (0.072 and 0.039, measured over a 144-minute confirmed-empty run and an occupied run),
and the adaptive-focus airtime shares of Sec. III-B (0.25 and 0.41, from the 19 July 2026 logs).

## License

Code: MIT (`LICENSE`). Data: CC BY 4.0.
