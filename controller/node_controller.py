"""Host controller for the node firmware: discovers nodes, runs the sounding cycle and logs
what the nodes report.

Uniform mode triggers every registered node in turn. Focus mode runs the focused tier of the
article: each authoring bin it rotates the floor through the node-covering pool (schedule.py),
seats the aim chords and the floor inside the envelope, and triggers only the seated nodes, one
per slot. Every trigger is a UDP broadcast naming one transmitter; every other node answers with
a CSI batch tagged with the trigger's slot_id, which the controller maps back to the transmitter.

Writes one JSON line per CSI sample (tx, rx, rssi, mean amplitude) and per overheard
beamforming report, and prints a per-link summary at the end.

    python node_controller.py --duration 60 --out csi.jsonl
    python node_controller.py --mode focus --aim "AA:..:01|AA:..:02" --door "AA:..:01|AA:..:05"
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import socket
import time
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import protocol as proto
import schedule as sc

BROADCAST_ADDR = os.environ.get("BROADCAST_ADDR", "255.255.255.255")
HOST_PORT = int(os.environ.get("HOST_PORT", str(proto.HOST_PORT)))
TRIGGER_PORT = int(os.environ.get("TRIGGER_PORT", str(proto.TRIGGER_PORT)))
SLOT_MS = 20                      # one transmitter per slot; the deployed TDMA slot
RX_GUARD_MS = 5                   # receivers stop collecting this long before the next trigger
DISCOVER_S = 3.0                  # how long to collect HELLOs before the first cycle
DISCOVER_REPEATS = 3
AUTHORING_BIN_S = 10.0            # the floor is re-picked once per bin, as in the deployment
SLOT_ID_MODULO = 256              # slot_id is one byte on the wire
NODE_TIMEOUT_S = 10.0             # a node silent this long leaves the cycle


@dataclass
class Node:
    mac: str
    addr: str
    chip: str = ""
    version: str = ""
    last_seen: float = 0.0


@dataclass
class State:
    nodes: Dict[str, Node] = field(default_factory=dict)          # mac -> node
    by_addr: Dict[str, str] = field(default_factory=dict)         # source ip -> mac
    slot_tx: Dict[int, str] = field(default_factory=dict)         # slot_id -> transmitter mac
    links: Counter = field(default_factory=Counter)               # (tx, rx) -> samples
    bfi: Counter = field(default_factory=Counter)                 # capturing node -> reports
    triggers: int = 0


def amplitude(iq: List[int]) -> float:
    pairs = list(zip(iq[0::2], iq[1::2]))
    return sum(math.hypot(i, q) for i, q in pairs) / max(1, len(pairs))


class HostProtocol(asyncio.DatagramProtocol):
    def __init__(self, state: State, out):
        self.state = state
        self.out = out

    def datagram_received(self, data: bytes, addr: Tuple[str, int]) -> None:
        if len(data) < 2 or data[0] != proto.MAGIC:
            return
        now = time.time()
        st = self.state
        kind = data[1]
        if kind == proto.MSG_HELLO:
            h = proto.parse_hello(data)
            if h:
                st.nodes[h.mac] = Node(h.mac, addr[0], h.chip, h.version, now)
                st.by_addr[addr[0]] = h.mac
        elif kind == proto.MSG_HEARTBEAT:
            hb = proto.parse_heartbeat(data)
            if hb:
                node = st.nodes.setdefault(hb.mac, Node(hb.mac, addr[0]))
                node.addr, node.last_seen = addr[0], now
                st.by_addr[addr[0]] = hb.mac
        elif kind in (proto.MSG_CSI_V5, proto.MSG_CSI_V6):
            rx = st.by_addr.get(addr[0])
            for s in proto.parse_csi_batch(data):
                tx = st.slot_tx.get(s.slot_id)
                if not tx or not rx or tx == rx:
                    continue
                st.links[(tx, rx)] += 1
                if rx in st.nodes:
                    st.nodes[rx].last_seen = now
                self.out.write(json.dumps({"ts": now, "kind": "csi", "tx": tx, "rx": rx,
                                           "rssi": s.rssi, "amp": round(amplitude(s.iq), 3),
                                           "n_sc": len(s.iq) // 2,
                                           "trigger_seq": s.trigger_seq}) + "\n")
        elif kind == proto.MSG_BFI:
            b = proto.parse_bfi(data)
            if b:
                st.bfi[b.node] += 1
                self.out.write(json.dumps({"ts": now, "kind": "bfi", "node": b.node,
                                           "beamformee": b.beamformee, "beamformer": b.beamformer,
                                           "rssi": b.rssi, "channel": b.channel,
                                           "body_len": len(b.body)}) + "\n")


def live_nodes(st: State, now: float) -> List[str]:
    return sorted(m for m, n in st.nodes.items() if now - n.last_seen <= NODE_TIMEOUT_S)


def focus_cycle(roster: List[str], aim: List[str], door: List[str], ts: float,
                env: sc.Envelope) -> List[str]:
    aim = [lk for lk in aim if all(n in roster for n in lk.split("|"))]
    pool = sc.floor_pool(door, roster, env)
    budget = env.node_budget_max
    floor = sc.rotate_floor(aim, budget, ts, pool, env)["floor"]
    return sc.seat_cycle(aim, budget, floor, env)


async def run(args) -> State:
    st = State()
    out = open(args.out, "w") if args.out else open(os.devnull, "w")
    loop = asyncio.get_running_loop()
    transport, _ = await loop.create_datagram_endpoint(
        lambda: HostProtocol(st, out), local_addr=(args.bind, args.host_port))
    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    dest = (args.broadcast, args.trigger_port)
    env = sc.Envelope(max_nodes=args.envelope)
    aim = [lk for lk in (args.aim or "").split(",") if lk]
    door = [lk for lk in (args.door or "").split(",") if lk]
    try:
        for _ in range(DISCOVER_REPEATS):
            tx_sock.sendto(proto.build_discover(), dest)
            await asyncio.sleep(args.discover_s / DISCOVER_REPEATS)
        t_end = time.time() + args.duration
        seq, cycle, cycle_bin = 0, [], None
        while time.time() < t_end:
            now = time.time()
            roster = live_nodes(st, now)
            if len(roster) < 2:
                await asyncio.sleep(args.slot_ms / 1000.0)
                continue
            this_bin = int(now // AUTHORING_BIN_S)
            if args.mode == "uniform":
                cycle = roster
            elif this_bin != cycle_bin or not cycle:
                cycle = focus_cycle(roster, aim, door, this_bin * AUTHORING_BIN_S, env)
                cycle_bin = this_bin
            for slot, tx in enumerate(cycle):
                slot_id = slot % SLOT_ID_MODULO
                st.slot_tx[slot_id] = tx
                seq += 1
                tx_sock.sendto(proto.build_trigger(tx, seq, slot_id,
                                                   args.slot_ms - RX_GUARD_MS), dest)
                st.triggers += 1
                await asyncio.sleep(args.slot_ms / 1000.0)
    finally:
        transport.close()
        tx_sock.close()
        out.close()
    return st


def summary(st: State) -> dict:
    return {"nodes": {m: {"chip": n.chip, "version": n.version} for m, n in sorted(st.nodes.items())},
            "triggers": st.triggers,
            "links_with_csi": len(st.links),
            "csi_samples": sum(st.links.values()),
            "per_link": {f"{a}->{b}": c for (a, b), c in sorted(st.links.items())},
            "bfi_reports": dict(st.bfi)}


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--mode", choices=("uniform", "focus"), default="uniform")
    ap.add_argument("--aim", help="comma-separated aim chords, MAC|MAC")
    ap.add_argument("--door", help="comma-separated door-zone chords that lead the floor pool")
    ap.add_argument("--envelope", type=int, default=sc.Envelope().max_nodes)
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--slot-ms", type=int, default=SLOT_MS)
    ap.add_argument("--discover-s", type=float, default=DISCOVER_S)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--host-port", type=int, default=HOST_PORT)
    ap.add_argument("--trigger-port", type=int, default=TRIGGER_PORT)
    ap.add_argument("--broadcast", default=BROADCAST_ADDR)
    ap.add_argument("--out", type=Path)
    args = ap.parse_args(argv)
    st = asyncio.run(run(args))
    print(json.dumps(summary(st), indent=2))
    return 0 if st.links else 1


if __name__ == "__main__":
    raise SystemExit(main())
