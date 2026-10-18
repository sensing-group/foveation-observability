"""Wire formats against the firmware source: sizes are read from firmware/main/main.c, so a
change on either side fails here."""
import re
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "controller"))
import protocol as proto  # noqa: E402

MAIN_C = (ROOT / "firmware" / "main" / "main.c").read_text()


def fw_define(name: str) -> int:
    m = re.search(rf"#define\s+{name}\s+(0x[0-9A-Fa-f]+|\d+)", MAIN_C)
    assert m, name
    return int(m.group(1), 0)


def test_sizes_and_types_match_the_firmware():
    assert proto.TRIGGER_SIZE == fw_define("BINARY_TRIGGER_SIZE")
    assert proto.DISCOVER_SIZE == fw_define("BINARY_DISCOVER_SIZE")
    assert proto.HELLO_SIZE == fw_define("BINARY_HELLO_SIZE")
    assert proto.BATCH_HEADER_V5_SIZE == fw_define("BINARY_BATCH_HEADER_SIZE")
    assert proto.SAMPLE_V5_SIZE == fw_define("BINARY_SAMPLE_SIZE_V5")
    assert proto.SAMPLE_V5_SIZE + proto.TRIGGER_SEQ_SIZE == fw_define("BINARY_SAMPLE_SIZE_V5_1")
    assert proto.BATCH_HEADER_V6_SIZE == fw_define("BINARY_V6_HEADER_SIZE")
    assert proto.MAGIC == fw_define("BINARY_PROTO_MAGIC")
    for name, val in (("MSG_TYPE_HELLO", proto.MSG_HELLO), ("MSG_TYPE_HEARTBEAT", proto.MSG_HEARTBEAT),
                      ("MSG_TYPE_TRIGGER", proto.MSG_TRIGGER), ("MSG_TYPE_DISCOVER", proto.MSG_DISCOVER),
                      ("MSG_TYPE_BFI", proto.MSG_BFI)):
        assert val == fw_define(name)
    assert proto.MSG_CSI_V5 == fw_define("BINARY_CSI_MAGIC_V5_1")
    assert proto.MSG_CSI_V6 == fw_define("BINARY_CSI_MAGIC_V6")
    assert struct.calcsize(proto.BFI_HEADER_FMT) == proto.BFI_HEADER_SIZE
    assert "sizeof(bfi_header_t) == 36" in MAIN_C


def test_chip_codes_match_the_firmware():
    pairs = re.findall(r"case CHIP_(ESP32\w*):\s*return (0x[0-9A-Fa-f]+);", MAIN_C)
    assert pairs
    for chip, code in pairs:
        name = chip.replace("ESP32", "ESP32-", 1).rstrip("-")
        assert proto.CHIP_TYPES[int(code, 16)] == name, (chip, code)


def test_trigger_layout_and_crc():
    t = proto.build_trigger("02:00:00:00:00:40", seq=7, slot_id=3, slot_ms=15)
    assert len(t) == proto.TRIGGER_SIZE and t[:2] == bytes([0xC5, 0x10])
    assert t[2:8] == bytes.fromhex("020000000040")
    seq, bursts, slot_ms = struct.unpack("<IHH", t[8:16])
    assert (seq, bursts, slot_ms) == (7, proto.BURSTS_PER_TRIGGER, 15)
    assert t[18] == 3 and proto.crc_ok(t)
    assert proto.crc_ok(proto.build_discover()) and len(proto.build_discover()) == proto.DISCOVER_SIZE


def _hello(mac="02:00:00:00:00:40"):
    body = bytes([0xC5, 0x01]) + proto.mac_bytes(mac) + bytes([192, 0, 2, 7, 0x08, 5, 9, 0]) \
        + b"UNICAST\x00" + struct.pack("<III", 100, 200000, 150000)
    return body + struct.pack("<I", proto.crc32(body))


def test_hello_and_heartbeat_parse():
    h = proto.parse_hello(_hello())
    assert h.mac == "02:00:00:00:00:40" and h.chip == "ESP32-C5" and h.version == "5.9.0-UNICAST"
    body = bytes([0xC5, 0x02]) + proto.mac_bytes("02:00:00:00:00:40") + struct.pack(
        "<IIIhII", 5, 6, 7, 412, 1000, 20) + bytes(32 - 4)
    hb = proto.parse_heartbeat(body + struct.pack("<I", proto.crc32(body)))
    assert hb.die_temp_c == 41.2 and hb.csi_frames == 1000 and hb.bfi_frames == 20
    assert proto.parse_hello(_hello()[:-1] + b"\x00") is None          # bad CRC


def test_csi_batches_v5_and_v6():
    sample = struct.pack("<Hb", 10, -50) + bytes(range(104)) + struct.pack("<I", 99)
    body = bytes([0xC5, 0x07, 4, 2]) + struct.pack("<HH", 1, 2) + sample * 2
    s = proto.parse_csi_batch(body + struct.pack("<I", proto.crc32(body)))
    assert len(s) == 2 and s[0].slot_id == 4 and s[0].rssi == -50 and s[0].trigger_seq == 99
    assert len(s[0].iq) == 104
    n_sc = 245
    sample6 = struct.pack("<Hb", 0, -61) + bytes(2 * n_sc)
    body6 = bytes([0xC5, 0x08, 1, 1]) + struct.pack("<HHH", 1, 2, n_sc) + bytes([3, 0]) + sample6
    s6 = proto.parse_csi_batch(body6 + struct.pack("<I", proto.crc32(body6)))
    assert len(s6) == 1 and len(s6[0].iq) == 2 * n_sc and s6[0].trigger_seq is None


def test_bfi_parse():
    payload = b"\x15\x00" + bytes(40)
    hdr = struct.pack(proto.BFI_HEADER_FMT, 0xC5, 0x40, proto.mac_bytes("02:00:00:00:00:40"),
                      proto.mac_bytes("AA:BB:CC:00:00:01"), proto.mac_bytes("AA:BB:CC:00:00:02"),
                      123456, -55, 36, 3, 0xD0, 21, 0, len(payload))
    b = proto.parse_bfi(hdr + payload + struct.pack("<I", proto.crc32(hdr + payload)))
    assert b.node == "02:00:00:00:00:40" and b.channel == 36 and b.body == payload
    assert b.category == 21 and b.action == 0 and b.is_beamforming_report
    hdr2 = hdr[:32] + bytes([21, 2]) + hdr[34:]          # VHT Operating Mode Notification
    b2 = proto.parse_bfi(hdr2 + payload + struct.pack("<I", proto.crc32(hdr2 + payload)))
    assert not b2.is_beamforming_report


def test_short_v6_batch_is_dropped_not_raised():
    body = bytes([0xC5, 0x08, 1, 1])
    assert proto.parse_csi_batch(body + struct.pack("<I", proto.crc32(body))) == []
