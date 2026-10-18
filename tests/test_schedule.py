"""The seat allocator against a directive the deployed system recorded on 8 September 2026."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "controller"))
import schedule as sc  # noqa: E402

FX = json.loads((ROOT / "tests" / "fixtures" / "directive_20260908.json").read_text())
ENV = sc.Envelope(max_nodes=FX["envelope"], dwell_s=FX["dwell_s"], liveness_s=FX["liveness_s"],
                  sweep_margin=FX["sweep_margin"], bin_s=FX["bin_s"])


def test_split_is_derived_from_the_envelope():
    assert ENV.floor_slots == FX["floor_slots"] == 2
    assert ENV.node_budget_max == FX["directive_in"]["node_budget"] == 2
    assert sc.Envelope(max_nodes=4).floor_slots == 1
    assert sc.Envelope(max_nodes=8).floor_slots == 3


def test_floor_pool_matches_the_deployed_pool():
    pool = sc.floor_pool(FX["door_zone_chords"], FX["roster"], ENV)
    assert pool == FX["pool"]
    assert len(pool) == FX["expected"]["capacity_links"]
    assert set(sc.nodes_of(pool)) == set(FX["roster"])


def test_sweep_fits_the_liveness_window():
    dwell = sc.floor_dwell_s(len(FX["pool"]), ENV)
    assert len(FX["pool"]) * dwell / ENV.floor_slots == FX["expected"]["sweep_s"]
    assert FX["expected"]["sweep_s"] <= ENV.sweep_margin * ENV.liveness_s


def test_rotation_reproduces_the_recorded_floor():
    aim = FX["directive_in"]["targets"][0]["links"]
    out = sc.rotate_floor(aim, FX["directive_in"]["node_budget"], FX["ts"], FX["pool"], ENV)
    assert out["clock"] == FX["expected"]["clock_picks"]
    assert out["fill"] == FX["expected"]["fill_index"]


def test_every_pool_link_is_revisited_within_one_sweep():
    dwell = sc.floor_dwell_s(len(FX["pool"]), ENV)
    seen = set()
    t = FX["ts"]
    while t < FX["ts"] + FX["expected"]["sweep_s"]:
        seen.update(sc.rotate_floor([], 0, t, FX["pool"], ENV, fill=False)["clock"])
        t += dwell
    assert seen == set(FX["pool"])


def test_cycle_seats_aim_then_floor_inside_the_envelope():
    aim = FX["directive_in"]["targets"][0]["links"]
    floor = sc.rotate_floor(aim, 2, FX["ts"], FX["pool"], ENV)["floor"]
    cycle = sc.seat_cycle(aim, 2, floor, ENV)
    assert len(cycle) <= ENV.max_nodes
    assert set(sc.nodes_of(aim[:1])) <= set(cycle)
    assert set(sc.nodes_of(floor)) <= set(cycle)
    assert cycle == sorted(cycle)


def test_floor_is_never_what_gets_dropped():
    env = sc.Envelope(max_nodes=4)
    cycle = sc.seat_cycle(["A|B", "C|D"], 4, ["E|F"], env)
    assert {"E", "F"} <= set(cycle) and len(cycle) == 4


def test_sigma_fill_prefers_the_highest_standing_new_node():
    walk = ["A|B", "A|C", "A|D"]
    assert sc.rank_by_sigma(walk, {"A"}, {"B": 0.1, "C": 0.9, "D": 0.5}) == ["A|C", "A|D", "A|B"]
    assert sc.rank_by_sigma(walk, {"A"}, {}) == walk
