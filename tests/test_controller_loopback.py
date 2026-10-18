"""End to end on the loopback interface: simulated nodes answer DISCOVER with HELLO and every
TRIGGER with a CSI batch, as the firmware does, and the controller must recover every link."""
import asyncio
import socket
import struct
import sys
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "controller"))
import node_controller as nc  # noqa: E402
import protocol as proto  # noqa: E402

MACS = [f"02:00:00:00:00:{i:02X}" for i in range(1, 5)]
ADDRS = [f"127.0.0.{i + 2}" for i in range(len(MACS))]


def free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def hello(mac: str, ip: str) -> bytes:
    body = bytes([0xC5, 0x01]) + proto.mac_bytes(mac) + bytes(int(x) for x in ip.split(".")) \
        + bytes([0x05, 5, 9, 0]) + b"SIM\x00\x00\x00\x00\x00" + struct.pack("<III", 1, 2, 3)
    return body + struct.pack("<I", proto.crc32(body))


def csi_batch(slot_id: int, seq: int) -> bytes:
    sample = struct.pack("<Hb", 0, -50) + bytes([3, 4] * 52) + struct.pack("<I", seq)
    body = bytes([0xC5, 0x07, slot_id, 1]) + struct.pack("<HH", seq & 0xFFFF, 0) + sample
    return body + struct.pack("<I", proto.crc32(body))


def simulated_mesh(trigger_port: int, host_port: int, stop: threading.Event):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", trigger_port))
    rx.settimeout(0.1)
    node_socks = []
    for ip in ADDRS:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind((ip, 0))
        node_socks.append(s)
    host = ("127.0.0.1", host_port)
    while not stop.is_set():
        try:
            data, _ = rx.recvfrom(64)
        except socket.timeout:
            continue
        if not proto.crc_ok(data):
            continue
        if data[1] == proto.MSG_DISCOVER:
            for mac, ip, s in zip(MACS, ADDRS, node_socks):
                s.sendto(hello(mac, ip), host)
        elif data[1] == proto.MSG_TRIGGER:
            tx = proto.mac_str(data[2:8])
            seq = struct.unpack("<I", data[8:12])[0]
            for mac, s in zip(MACS, node_socks):
                if mac != tx:
                    s.sendto(csi_batch(data[18], seq), host)
    for s in node_socks + [rx]:
        s.close()


def run_controller(extra):
    tp, hp = free_port(), free_port()
    stop = threading.Event()
    th = threading.Thread(target=simulated_mesh, args=(tp, hp, stop), daemon=True)
    th.start()
    args = nc.argparse.Namespace(mode="uniform", aim=None, door=None, envelope=6, duration=1.5,
                                 slot_ms=20, discover_s=0.3, bind="127.0.0.1", host_port=hp,
                                 trigger_port=tp, broadcast="127.0.0.1", out=None)
    for k, v in extra.items():
        setattr(args, k, v)
    try:
        return asyncio.run(nc.run(args))
    finally:
        stop.set()
        th.join(timeout=2)


def test_uniform_cycle_recovers_every_directed_link():
    st = run_controller({})
    assert sorted(st.nodes) == MACS
    expected = {(a, b) for a in MACS for b in MACS if a != b}
    assert set(st.links) == expected


def test_focus_cycle_triggers_only_seated_nodes():
    aim = f"{MACS[0]}|{MACS[1]}"
    st = run_controller({"mode": "focus", "aim": aim, "door": f"{MACS[0]}|{MACS[2]}",
                         "envelope": 3})
    transmitters = {tx for tx, _ in st.links}
    assert {MACS[0], MACS[1]} <= transmitters
    assert len(transmitters) <= 3
