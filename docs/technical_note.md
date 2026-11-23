# Technical note: models and numbers behind the article

This note accompanies K. Khamaisi and B. Rodrigues, "Foveation and Observability in
Next-Generation Wireless Sensing Networks", IEEE Communications Magazine (under revision).
The article explains each result in words, as the magazine requires. This note gives the
underlying models, every parameter, and the script that produces each figure and number.
`reproduce.sh` regenerates all of them and compares them with `reference/`, the outputs the
article was written from.

## 1. Deployment

| Item | Value | Source |
|---|---|---|
| Nodes | 10 ESP32 nodes (ESP32-C5 and ESP32-C3), firmware in `firmware/` | |
| Room | 5 m by 4 m | |
| Links | 45, one per unordered node pair | |
| Band | 2.4 GHz; the model uses a carrier of 2.437 GHz (wavelength 12.3 cm) | `scripts/model.py` |
| Node positions | estimated by the mesh itself, not surveyed | `data/positions.json` |
| Sounding | one transmitter per 20 ms slot, 50 triggers per second | `controller/node_controller.py` |
| Per trigger | a burst of ten short broadcast frames at the basic rate; every other node extracts CSI | `firmware/` |
| BFI | compressed beamforming reports of the access point's clients, overheard on the same 2.4 GHz channel | `firmware/` |

## 2. Link sensitivity model

The article uses a widely used model from radio tomographic imaging (RTI): a smooth, Gaussian
variant of the ellipse weight model of Wilson and Patwari (IEEE TMC 2010). A person at point $x$
perturbs link $(i,j)$, with endpoints $n_i$ and $n_j$, by a weight that decays as a Gaussian in
the excess path length

$$\rho_{ij}(x) = \lVert x-n_i\rVert + \lVert x-n_j\rVert - \lVert n_i-n_j\rVert ,$$

$$w_{ij}(x) = \exp\!\left(-\frac{\rho_{ij}(x)^2}{2\sigma_{ij}^2}\right), \qquad
\sigma_{ij} = 0.5\sqrt{\lambda\, d_{ij}} ,$$

where $d_{ij}$ is the link length and $\lambda$ the wavelength. $\sigma_{ij}$ equals the
first Fresnel radius at mid-link: for a 4 m link at 2.4 GHz it is 0.35 m, so the zone is about
0.7 m wide.

Two inputs were measured on the mesh, over a 144-minute confirmed-empty run and an occupied
run:

| Input | Value | Meaning |
|---|---|---|
| `SIGNAL_REL` | 0.072 | fractional CSI amplitude change caused by an occupant |
| `NOISE_REL` | 0.039 | fractional CSI amplitude fluctuation in the empty room |
| SNR | 0.072 / 0.039 = 1.85 | |

Assumptions: one person, noise independent across links, one sample per link unless stated
otherwise. Every assumption makes the results optimistic.

## 3. Observability field (Sec. III-B and III-C, Fig. 3)

**Localization.** Each link reports its CSI amplitude change $S\,w_{ij}(x)$ plus independent
Gaussian noise of standard deviation $N$, with $S$ = `SIGNAL_REL` and $N$ = `NOISE_REL`. The
Fisher information $J(x)$ measures how strongly these measurements change when the person moves
slightly away from $x$: large where many links are sensitive to small displacements, small where
none are. For this Gaussian model it sums the outer products of the sensitivity gradients of all
45 links, scaled by the squared SNR:

$$J(x) = \mathrm{SNR}^2 \sum_{(i,j)} \nabla w_{ij}(x)\, \nabla w_{ij}(x)^{\mathsf T},
\qquad
\nabla w_{ij}(x) = -\frac{\rho_{ij}}{\sigma_{ij}^2}\, w_{ij}(x)
\left(\frac{x-n_i}{\lVert x-n_i\rVert} + \frac{x-n_j}{\lVert x-n_j\rVert}\right).$$

The Cramér–Rao bound states that no unbiased estimator can have a lower RMS position error
than $\sqrt{\operatorname{tr} J(x)^{-1}}$. It is evaluated on a 10 cm grid. A point is **not localizable** when this bound exceeds 1 m. With $K$ samples
per link the information scales by $K$ and the bound by $1/\sqrt{K}$, which is why the geometry
fixes the shape of the field and the sounding budget its scale.

**Detection.** The detection signal-to-noise ratio of a presence test against a known empty
reference is

$$\delta(x) = \mathrm{SNR}\sqrt{\sum_{(i,j)} w_{ij}(x)^2} ,$$

the CSI change a moving person at $x$ causes, combined over all links, in units of the
empty-room noise standard deviation. It is the output SNR of a detector that combines all links
optimally (in detection theory, the deflection coefficient). A point is **not detectable** when
$\delta(x) < 3$.

The bound assumes that the person is known to be present, whereas $\delta$ asks whether a
moving person is noticed at all. The two thresholds are not ordered by construction. With one
sample per link the detection threshold is the stricter one, so the not-detectable region
contains the not-localizable region, and 12.9% of the room is localizable but not detectable
(near the walls). With one second of samples the two regions almost coincide.

| Result | One sample per link | One second of samples |
|---|---|---|
| Median localization bound | 31 cm | |
| Not localizable | 18.6% | 10.8% |
| Not detectable | 31.4% | 13.1% |
| Detectable but not localizable | 0.0% | 0.6% |
| Localizable but not detectable | 12.9% | 3.0% |
| Not localizable, SNR halved | 29.0% | |
| Not localizable, node positions perturbed (Gaussian, σ = 0.5 m, 40 draws), 10th to 90th percentile | 12.1% to 32.1% (median 22.7%) | |

Scripts: `scripts/model.py` (model and bound), `scripts/make_fig_field.py` (Fig. 3 and
`results/field.json`), `scripts/robustness.py` (SNR and position checks,
`results/robustness.json`).

## 4. Feasibility test (Sec. III-D, Fig. 4)

**Link rate under broadcast sounding.** Node $i$ transmits $b_i$ times per second, with
$\sum_i b_i = B$. Every other node measures each transmission, so link $(i,j)$ is sampled at

$$r_{ij} = b_i + b_j .$$

Under uniform sounding $b_i = B/T$ for $T$ nodes, so every link gets $2B/T$. With $B = 200$ and
$T = 10$: 20 transmissions per node per second, and a sampling rate of 40 Hz per link.

**Required rate.** A person moving at speed $v$ changes a reflected path length at up to $2v$,
so the maximum Doppler shift is $f_D = 2v/\lambda$. For walking, $v = 1.4$ m/s gives
$f_D \approx 23$ Hz, and sampling without aliasing needs a sampling rate of
$2f_D \approx 46$ Hz per link. Limbs move faster than the torso, so this is a lower bound. Detecting motion, or
following a position (a walker crosses a 0.7 m Fresnel zone in about 0.5 s), needs far less.

**Required links.** At location $x$, sort the links by $w_{ij}(x)$ and take the shortest prefix
that carries 80% of $\sum w_{ij}(x)$. At the room centre this gives 12 links, which together
touch all ten nodes.

**Best split.** The highest rate the slowest required link can reach is the linear program

$$\max_{b,\,r}\; r \quad \text{s.t.}\quad b_i + b_j \ge r \;\; \forall (i,j)\ \text{required},
\qquad \sum_i b_i = B, \qquad b_i \ge \beta B/T ,$$

where $\beta$ is the periphery floor: the share every node keeps, as a fraction of its uniform
share. Without a floor ($\beta = 0$) the optimum is $r^* = B/\tau^*$, with $\tau^*$ the
fractional vertex-cover number of the required links: the least total weight on the nodes such
that every required link has endpoint weights summing to at least one. Uniform sounding places
weight 1/2 on every node, a cover of weight $T/2$, so it is optimal only when $\tau^* = T/2$.
Touching every node does not make it so. A sufficient condition for foveation to beat uniform
sounding is a set of fewer than $T/2$ nodes that covers all required links, since its indicator
weights form a cover of weight below $T/2$. The converse does not hold, because $\tau^*$ can be
fractional (half-integral). In the test room, $\tau^*$ takes the values 1, 2, 3, 4, 4.5 and 5,
and uniform sounding is optimal ($\tau^* = 5$) at 120 of 1,911 grid points (6%). At the room
centre $\tau^* = 4$: four nodes cover all
12 required links, so $r^* = 200/4 = 50$ Hz against 40 Hz uniform. The other six nodes stop
transmitting, and the $\binom{6}{2} = 15$ links among them are no longer sampled.

A location **supports** walking when $r^* \ge 46$ Hz. Fig. 4 shows the supported share of the
room against $B$.

| Budget $B$ (exchanges/s) | Uniform ($\beta = 1$) | Foveation, no floor ($\beta = 0$) | Floor $\beta = 1/2$ | Airtime at 352.9 µs per exchange |
|---|---|---|---|---|
| 50 (deployed) | 0% | 22.5% | 0% | 1.8% |
| 92 | 0% | 56.0% | 22.5% | 3.3% |
| 184 | 0% | 93.6% | 86.3% | 6.5% |
| 200 | 0% | 93.6% | 86.3% | 7.1% |
| 230 | 100% | 100% | 100% | 8.1% |

With $\beta = 1/3$, the supported share at $B = 200$ stays at 93.6% and no link is left
unsampled (median unsampled links: 0, against 28 without a floor). Under the centre-aimed
allocation the not-localizable share of the field stays at 18% to 19% for every floor, so the
price of the fovea is the periphery's sampling rate, not its observability.

At 50 exchanges per second the 22.5% that pass are the locations whose required links all end
at one node ($\tau^* = 1$), which can then transmit at the full rate.

**Robustness.** Required-link share 0.6, 0.7, 0.8, 0.9 at $B = 200$: supported share between
97% and 90% (`results/allocation_floor_sweep.log`). Treating the two directions of a link as
separate series (only the transmitter refreshes each) halves the uniform rate to 20 Hz, while
the supported share under foveation is unchanged at 93.6%.

**Note on airtime.** The airtime column prices an 802.11bf exchange (Section 5). Our nodes do
not implement 802.11bf, and their legacy bursts cost several times more per trigger, so the
1.8% at 50 exchanges per second does not describe the deployed mesh.

Scripts: `scripts/make_fig_feasibility.py` (Fig. 4, `results/feasibility.json`),
`scripts/required_set_lp.py` (room centre, `results/required_set_lp.log`),
`scripts/allocation_floor_sweep.py` (floors and threshold robustness), `scripts/robustness.py`.

## 5. Airtime of one 802.11bf exchange (Sec. IV-A)

Computed from 802.11 frame durations, not measured:

| Part | µs |
|---|---|
| Channel access | 110.5 |
| NDP announcement (NDPA) | 76.0 |
| Null data packet (NDP) | 49.6 |
| Two short interframe spaces | 32.0 |
| Compressed report | 84.8 |
| **Total** | **352.9** |
| Without the report (CSI processed on the node) | 268.1 |

230 exchanges per second therefore take 230 × 352.9 µs = 81 ms of every second (8.1%), and 92
exchanges take 32 ms (3.3%). Source: `EXCHANGE_PARTS_US` in `scripts/robustness.py`.

## 6. Occupancy release rule (Sec. V)

`controller/release.py` is an extract of the deployed watcher. Once an entry is registered, the
room stays occupied until a release path fires. The deployed watcher has further release paths
(an exit signature at the doorway, a half-life decay and a BFI likelihood release), which are not
in the extract and did not fire in the recording. The path that fired is an absence score $S$, in
nats, which releases the room when it reaches 6.

Per 10 s bin:

| Condition | Effect on $S$ |
|---|---|
| Fewer than three evidence sources live (CSI motion statistic, mesh-wide CSI statistic, BFI with at least two reporting clients) | none |
| Any live source active ($\lvert z\rvert > 2.5$) | no credit. Penalty if the CSI motion statistic $z \ge 6.39$: 4.9 nats; else if $z \ge 2.5$: 1.85 nats; else if the BFI statistic is active and positive: 1.85 nats; otherwise none. $S$ never drops below 0 |
| All sources quiet, and device traffic dropped below its floor in the last 20 minutes | +0.05 nats |
| All sources quiet during an active probe | +0.15 nats |
| All sources quiet within 5 minutes after a negative probe | +0.05 nats |
| All sources quiet, no traffic drop, no probe | none |

The release level of 6 nats corresponds to nominal odds of $e^6 \approx 400$ to 1 in favour of
absence. With 0.05 nats per bin it takes at least 120 quiet bins (20 minutes), and with
continuous probing 40 bins (about 7 minutes). The per-bin credits are design parameters of the
deployed system, not quantities estimated from data. A moderate penalty cancels the credit of
37 quiet bins (about six minutes), a strong one that of 98 bins.

On 4 August 2026 the room was occupied from 19:34 and released at 21:50 (819 bins). The
logged reason per bin during the stay:

| Reason | Bins |
|---|---|
| All three quiet, credited | 539 |
| All three quiet, credited during a probe | 16 |
| CSI active | 102 |
| Mesh active, or CSI with mesh or BFI active | 10 |
| Too few live sources | 84 |
| Quiet but no traffic drop | 68 |

The departure time was not logged, so the release delay cannot be measured. The replay in
`controller/replay_episode.py` reproduces the logged score in every bin.

Data: `data/episode_inputs.jsonl` (all inputs per bin), `data/episode.json` (Fig. 5).

## 7. What is not regenerated

The signal and noise levels (0.072 and 0.039) come from the deployment logs and enter
`scripts/model.py` as constants. Every other number above is computed by the scripts listed.
