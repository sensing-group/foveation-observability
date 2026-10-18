"""Binary protocol between the host and the node firmware (firmware/main/main.c).

Every message starts with the magic byte 0xC5 and a type byte, and ends with a little-endian
CRC-32 (zlib polynomial, as esp_crc32_le with seed 0) over all preceding bytes. Nodes send HELLO,
HEARTBEAT, CSI batches and BFI reports to the host on HOST_PORT; the host broadcasts TRIGGER and
DISCOVER to the nodes on TRIGGER_PORT. A TRIGGER names one transmitter: that node sends a burst,
every other node captures CSI from it and reports a batch tagged with the trigger's slot_id.
"""
from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass
from typing import List, Optional

MAGIC = 0xC5
HOST_PORT = 5000
TRIGGER_PORT = 5001

MSG_HELLO = 0x01
MSG_HEARTBEAT = 0x02
MSG_CSI_V5 = 0x07
MSG_CSI_V6 = 0x08
MSG_TRIGGER = 0x10
MSG_DISCOVER = 0x11
MSG_BFI = 0x40

TRIGGER_SIZE = 32
DISCOVER_SIZE = 8
HELLO_SIZE = 40
HEARTBEAT_MIN_SIZE = 24
BATCH_HEADER_V5_SIZE = 8
BATCH_HEADER_V6_SIZE = 12
SAMPLE_V5_SIZE = 107               # timestamp delta (2) + rssi (1) + 52 x I/Q int8 (104)
TRIGGER_SEQ_SIZE = 4               # appended to every sample since firmware v5.8.0
BFI_HEADER_SIZE = 36
BFI_HEADER_FMT = "<2B6s6s6sqbBBBBBH"
DIE_TEMP_UNAVAILABLE = -32768

BURSTS_PER_TRIGGER = 4             # deployed default
SAMPLES_PER_MEASUREMENT = 5
CHIP_TYPES = {0x03: "ESP32-C3", 0x05: "ESP32-C5", 0x06: "ESP32-C6"}


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def crc_ok(data: bytes) -> bool:
    return len(data) >= 4 and struct.unpack("<I", data[-4:])[0] == crc32(data[:-4])


def mac_bytes(mac: str) -> bytes:
    return bytes(int(b, 16) for b in mac.split(":"))


def mac_str(b: bytes) -> str:
    return ":".join(f"{x:02X}" for x in b)


def build_trigger(tx_mac: str, seq: int, slot_id: int, slot_ms: int,
                  bursts: int = BURSTS_PER_TRIGGER, samples: int = SAMPLES_PER_MEASUREMENT,
                  send_all: bool = False, burst_transport: int = 0, rate_index: int = 0,
                  burst_dest: int = 0) -> bytes:
    """TRIGGER: magic, type, tx_mac[6], seq u32, burst_count u16, slot_ms u16 (the receivers'
    collection window), samples u8, send_all u8, slot_id u8, reserved[9], crc32.

    reserved[3..5] carry the burst transport (0 = UDP broadcast), the ESP-NOW rate index and the
    burst destination (0 = broadcast), as the firmware's trigger handler reads them."""
    buf = bytearray(TRIGGER_SIZE)
    buf[0], buf[1] = MAGIC, MSG_TRIGGER
    buf[2:8] = mac_bytes(tx_mac)
    struct.pack_into("<IHH", buf, 8, seq & 0xFFFFFFFF, bursts & 0xFFFF, int(slot_ms) & 0xFFFF)
    buf[16], buf[17], buf[18] = samples & 0xFF, int(send_all), slot_id & 0xFF
    buf[21], buf[22], buf[23] = burst_transport & 0xFF, rate_index & 0xFF, burst_dest & 0xFF
    struct.pack_into("<I", buf, 28, crc32(bytes(buf[:-4])))
    return bytes(buf)


def build_discover() -> bytes:
    body = bytes([MAGIC, MSG_DISCOVER, 0, 0])
    return body + struct.pack("<I", crc32(body))


@dataclass
class Hello:
    mac: str
    ip: str
    chip: str
    version: str
    uptime_s: int
    free_heap: int


def parse_hello(data: bytes) -> Optional[Hello]:
    if len(data) < HELLO_SIZE or data[0] != MAGIC or data[1] != MSG_HELLO \
            or not crc_ok(data[:HELLO_SIZE]):
        return None
    tag = data[16:24].rstrip(b"\x00").decode("ascii", errors="ignore")
    uptime, heap, _ = struct.unpack("<III", data[24:36])
    return Hello(mac_str(data[2:8]), ".".join(str(b) for b in data[8:12]),
                 CHIP_TYPES.get(data[12], f"0x{data[12]:02X}"),
                 f"{data[13]}.{data[14]}.{data[15]}-{tag}", uptime, heap)


@dataclass
class Heartbeat:
    mac: str
    uptime_s: int
    free_heap: int
    die_temp_c: Optional[float]
    csi_frames: Optional[int]
    bfi_frames: Optional[int]


def parse_heartbeat(data: bytes) -> Optional[Heartbeat]:
    """HEARTBEAT is append-only across firmware versions; fields a shorter datagram does not
    carry come back as None."""
    if len(data) < HEARTBEAT_MIN_SIZE or data[0] != MAGIC or data[1] != MSG_HEARTBEAT \
            or not crc_ok(data):
        return None
    uptime, heap, _ = struct.unpack("<III", data[8:20])
    body = len(data) - 4
    temp = csi = bfi = None
    if body >= 22:
        t = struct.unpack("<h", data[20:22])[0]
        temp = None if t == DIE_TEMP_UNAVAILABLE else t / 10.0
    if body >= 30:
        csi, bfi = struct.unpack("<II", data[22:30])
    return Heartbeat(mac_str(data[2:8]), uptime, heap, temp, csi, bfi)


@dataclass
class CsiSample:
    slot_id: int
    rssi: int
    iq: List[int]                  # interleaved int8 values as the firmware sends them
    trigger_seq: Optional[int]


def parse_csi_batch(data: bytes) -> List[CsiSample]:
    """A v5 (52 subcarriers) or v6 (variable width, HE) CSI batch from one receiving node; the
    transmitter is the node named by the trigger with this slot_id."""
    if len(data) < 4 or data[0] != MAGIC or data[1] not in (MSG_CSI_V5, MSG_CSI_V6) \
            or not crc_ok(data):
        return []
    slot_id, count = data[2], data[3]
    if data[1] == MSG_CSI_V5:
        head, base = BATCH_HEADER_V5_SIZE, SAMPLE_V5_SIZE
    else:
        n_sc = struct.unpack("<H", data[8:10])[0]
        head, base = BATCH_HEADER_V6_SIZE, 3 + 2 * n_sc
    if count == 0:
        return []
    payload = len(data) - head - 4
    width = payload // count if payload % count == 0 else base
    has_seq = width == base + TRIGGER_SEQ_SIZE
    out, off = [], head
    for _ in range(count):
        if off + width > len(data) - 4:
            break
        rssi = struct.unpack("<b", data[off + 2:off + 3])[0]
        iq = list(struct.unpack(f"<{base - 3}b", data[off + 3:off + base]))
        seq = struct.unpack("<I", data[off + base:off + base + 4])[0] if has_seq else None
        out.append(CsiSample(slot_id, rssi, iq, seq))
        off += width
    return out


@dataclass
class BfiReport:
    node: str                      # the node that overheard the report
    beamformee: str
    beamformer: str
    rssi: int
    channel: int
    body: bytes                    # the raw action-frame body (compressed beamforming report)


def parse_bfi(data: bytes) -> Optional[BfiReport]:
    if len(data) < BFI_HEADER_SIZE + 4 or data[0] != MAGIC or data[1] != MSG_BFI \
            or not crc_ok(data):
        return None
    f = struct.unpack(BFI_HEADER_FMT, data[:BFI_HEADER_SIZE])
    return BfiReport(mac_str(f[2]), mac_str(f[3]), mac_str(f[4]), f[6], f[7],
                     data[BFI_HEADER_SIZE:BFI_HEADER_SIZE + f[-1]])
