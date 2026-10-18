"""The focused tier's sounding cycle: aim seats and floor seats inside a node envelope.

Extracted from the deployed watcher (scripts/watcher/focus_directive.py) and its courier
(backend/src/core/focus/producers/watcher_belief.py). Links are undirected "A|B" chords between
node identifiers; one cycle is the set of nodes that transmit, one node per slot, every other
node measuring.

The split is derived from the envelope: an aim needs NODE_BUDGET_MIN nodes, and the floor takes
what is left in links of two endpoints. The aim is the leading chords of the target that fit the
node budget. The floor is a node-covering pool led by the door-zone chords, swept by a clock
round-robin fast enough to revisit every pool link inside the liveness window; spare envelope
seats are then filled from the pool, in clock order or ordered by node standing.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence


@dataclass(frozen=True)
class Envelope:
    max_nodes: int = 6              # transmitting nodes the executor honours (deployed: 6)
    node_budget_min: int = 2        # nodes an aim needs to be an aim at all
    floor_endpoints: int = 2        # a floor slot is a link, and a link has two endpoints
    floor_slots_min: int = 1
    dwell_s: float = 20.0           # rotation quantum, never slower than this
    liveness_s: float = 300.0       # a link unsounded longer than this counts as unobserved
    sweep_margin: float = 0.8       # share of the liveness window the floor sweep may use
    bin_s: float = 10.0             # the floor changes at most once per authoring bin

    @property
    def floor_slots(self) -> int:
        return max(self.floor_slots_min,
                   (self.max_nodes - self.node_budget_min) // self.floor_endpoints)

    @property
    def node_budget_max(self) -> int:
        return max(self.node_budget_min, self.max_nodes - self.floor_slots * self.floor_endpoints)


def nodes_of(links: Sequence[str]) -> List[str]:
    """Distinct endpoint nodes of chords, in link order."""
    out: List[str] = []
    for lk in links:
        for node in lk.split("|"):
            if node and node not in out:
                out.append(node)
    return out


def chords_within_budget(links: Sequence[str], node_budget: int) -> List[str]:
    """The longest prefix of `links` whose endpoints fit `node_budget` nodes."""
    out: List[str] = []
    seen: set = set()
    for lk in links:
        if not lk or "|" not in lk:
            continue
        ends = [e for e in lk.split("|", 1) if e]
        new = [e for e in ends if e not in seen]
        if node_budget and len(seen) + len(new) > node_budget:
            break
        out.append(lk)
        seen.update(new)
    return out


def floor_dwell_s(pool_size: int, env: Envelope, slots: Optional[int] = None) -> float:
    """Rotation quantum that sweeps `pool_size` links inside the liveness window.

    The sweep is P * dwell / slots, so dwell <= slots * margin * liveness / P; never slower than
    the directive dwell, never faster than the authoring bin (below it the rotation aliases).
    """
    s_ = max(1, int(slots or env.floor_slots))
    p = max(1, int(pool_size or 1))
    if env.liveness_s <= 0 or env.dwell_s <= 0:
        return env.dwell_s
    return max(env.bin_s, min(env.dwell_s, s_ * env.sweep_margin * env.liveness_s / p))


def floor_pool(door_zone_chords: Sequence[str], live_nodes: Sequence[str],
               env: Envelope) -> List[str]:
    """The floor's pool: door-zone chords first, extended so every live node is covered,
    trimmed of redundant door-zone chords until the sweep fits the liveness window."""
    live = {n for n in (live_nodes or ()) if n}
    slots = env.floor_slots
    boundary = [lk for lk in (door_zone_chords or ()) if lk and "|" in lk]
    if not live:
        return list(boundary)

    def endpoints(lk):
        return tuple(lk.split("|", 1))

    kept = [lk for lk in boundary if all(e in live for e in endpoints(lk))]
    covered = {e for lk in kept for e in endpoints(lk)}
    uncovered = sorted(live - covered)
    added = []
    for i in range(0, len(uncovered) - 1, 2):
        added.append("|".join(sorted((uncovered[i], uncovered[i + 1]))))
    if len(uncovered) % 2:
        odd = uncovered[-1]
        partner = sorted(covered)[0] if covered else (
            sorted(live - {odd})[0] if len(live) > 1 else None)
        if partner:
            added.append("|".join(sorted((odd, partner))))

    pool = kept + added
    fd = floor_dwell_s(len(pool), env, slots=slots)
    p_max = int(slots * env.liveness_s * env.sweep_margin / fd) if fd > 0 else len(pool)
    if len(pool) <= p_max:
        return pool
    need: Dict[str, int] = {}
    for lk in pool:
        for e in endpoints(lk):
            need[e] = need.get(e, 0) + 1
    trimmed = list(pool)
    for lk in list(kept)[::-1]:
        if len(trimmed) <= p_max:
            break
        if all(need[e] > 1 for e in endpoints(lk)):
            trimmed.remove(lk)
            for e in endpoints(lk):
                need[e] -= 1
    return trimmed


def rank_by_sigma(walk: Sequence[str], seated: set, node_sigma: Dict[str, float]) -> List[str]:
    """Order candidate links by the standing of the nodes each would add, descending;
    ties keep the incoming clock order."""
    scored = []
    for pos, lk in enumerate(walk):
        new = [n for n in lk.split("|") if n and n not in seated]
        s = max((node_sigma.get(n, 0.0) for n in new), default=0.0)
        scored.append((-s, pos, lk))
    scored.sort()
    return [lk for _, _, lk in scored]


def seat_fill(walk: Sequence[str], picked: List[str], seated: set, env: Envelope) -> List[str]:
    """Seat links from `walk` that bring new nodes, until the envelope is full. Mutates `seated`."""
    added: List[str] = []
    for lk in walk:
        if len(seated) >= env.max_nodes:
            break
        if lk in picked or lk in added:
            continue
        new = [n for n in lk.split("|") if n and n not in seated]
        if not new:
            continue
        if len(seated) + len(new) > env.max_nodes:
            continue
        added.append(lk)
        seated.update(new)
    return added


def rotate_floor(aim_links: Sequence[str], node_budget: int, ts: float, pool: Sequence[str],
                 env: Envelope, fill: bool = True,
                 node_sigma: Optional[Dict[str, float]] = None) -> Dict[str, List[str]]:
    """Floor links for the bin starting at `ts`: the clock picks, then the envelope fill.

    The fill walks on from the clock cursor (index order), or by node standing when
    `node_sigma` is given. Returns {"clock": [...], "fill": [...], "floor": clock + fill}.
    """
    slots = env.floor_slots
    pool = sorted(set(pool))
    if slots <= 0 or not pool:
        return {"clock": [], "fill": [], "floor": []}
    dwell = floor_dwell_s(len(pool), env, slots=slots)
    start = int(ts // dwell) * slots
    picked = [pool[(start + i) % len(pool)] for i in range(min(slots, len(pool)))]
    added: List[str] = []
    if fill:
        seated = set(nodes_of(chords_within_budget(aim_links, node_budget) if node_budget
                              else aim_links))
        seated.update(nodes_of(picked))
        walk = [pool[(start + i) % len(pool)] for i in range(min(slots, len(pool)), len(pool))]
        if node_sigma is not None:
            walk = rank_by_sigma(walk, set(seated), node_sigma)
        added = seat_fill(walk, picked, seated, env)
    return {"clock": picked, "fill": added, "floor": picked + added}


def seat_cycle(aim_links: Sequence[str], node_budget: int, floor_links: Sequence[str],
               env: Envelope) -> List[str]:
    """The courier and executor: seat aim chords within the node budget, then every floor link,
    dropping trailing aim chords (never the floor) to fit the envelope. Returns the transmitting
    nodes in cycle order, which the executor sorts by node identifier."""
    seated_links: List[str] = []
    nodes: List[str] = []
    for lk in aim_links:
        if lk in seated_links:
            continue
        ends = [n for n in lk.split("|") if n and n not in nodes]
        if node_budget and len(nodes) + len(ends) > node_budget:
            continue
        seated_links.append(lk)
        nodes.extend(ends)
    for lk in floor_links:
        if lk in seated_links:
            continue
        floor_nodes = [n for n in lk.split("|") if n and n not in nodes]
        while len(nodes) + len(floor_nodes) > env.max_nodes and len(seated_links) > 1:
            seated_links.pop()
            nodes = nodes_of(seated_links)
            floor_nodes = [n for n in lk.split("|") if n and n not in nodes]
        if len(nodes) + len(floor_nodes) <= env.max_nodes:
            seated_links.append(lk)
            nodes.extend(floor_nodes)
    return sorted(nodes)
