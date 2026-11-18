"""Binary protocol helpers (see docs/ICD.md and docs/ARCHITECTURE.md). Little-endian, XOR checksums.

External (ESP-NOW / USB / UART passthrough):
  0x01 COMMAND (GS -> Rover, 13 B):  <BBBBffB       0xAA 0x01 seq cmd_id distance_m angle_deg xor
  0x02 NAV     (Rover -> GS, 36 B):  <BBBIfffffffB  0xAA 0x02 seq ts_ms yaw pitch roll ax ay az vel xor
  0x03 POWER   (Rover -> GS, 17 B):  <BBBIffBB      0xAA 0x03 seq ts_ms voltage_v current_a flags xor
  0x04 THERMAL (Rover -> GS, 21 B):  <BBBIfffBB     0xAA 0x04 seq ts_ms t_bat t_pcb t_lidar flags xor
  0x05 HEALTH  (Rover -> GS, 15 B):  <BBBIBBBBHBB   0xAA 0x05 seq ts_ms status obs cpu_t cpu_p ram_mb flags xor
  0x06 MAP     (Rover -> GS, <=205): <BBBBBB        0xAA 0x06 map_seq chunk total len + payload + xor

Internal UART (Raspberry Pi <-> ESP32-Onboard, 115200 baud):
  0x55 ENCODERS     (ESP32 -> Pi, 12 B): <BBiiBB    0xAA 0x55 left_ticks right_ticks status xor
  0x66 DRIVE_STATE  (Pi -> ESP32, 4 B):  <BBBB      0xAA 0x66 obstacle_state xor
  0x90 PI_TELEMETRY (Pi -> ESP32, 12 B): <BBfBBHBB  0xAA 0x90 velocity_mps cpu_temp_c cpu_pct ram_mb pi_flags xor
"""
import struct
from typing import List, Optional, Tuple

SYNC = 0xAA

# Logical & frame IDs per docs/ICD.md
CMD_ID = 0x01
NAV_ID = 0x02
POWER_ID = 0x03
THERMAL_ID = 0x04
HEALTH_ID = 0x05
MAP_ID = 0x06

# Internal UART IDs (and legacy aliases)
P1_ID = 0x55
P2_ID = 0x66
P3_ID = MAP_ID
P3_LEGACY_ID = 0x77
P4_ID = CMD_ID
P4_LEGACY_ID = 0x88
PI_TEL_ID = 0x90

# Command codes (cmd_id in 0x01 COMMAND)
CMD_STOP = 0
CMD_MOVE = 1
CMD_START_MISSION = 2
CMD_STOP_MISSION = 3

# Struct formats
P1_FMT = '<BBiiBB'
P1_LEN = struct.calcsize(P1_FMT)                # 12
P2_FMT = '<BBBB'
P2_LEN = struct.calcsize(P2_FMT)                # 4
P3_HDR_FMT = '<BBBBBB'
P3_HDR_LEN = struct.calcsize(P3_HDR_FMT)        # 6
P3_MAX_PAYLOAD = 198
P4_FMT = '<BBBBffB'
P4_LEN = struct.calcsize(P4_FMT)                # 13
P4_LEGACY_FMT = '<BBBfB'
P4_LEGACY_LEN = struct.calcsize(P4_LEGACY_FMT)  # 8
PI_TEL_FMT = '<BBfBBHBB'
PI_TEL_LEN = struct.calcsize(PI_TEL_FMT)        # 12


def xor_checksum(data: bytes) -> int:
    """XOR of all bytes."""
    c = 0
    for b in data:
        c ^= b
    return c


def pack_packet1(left_ticks: int, right_ticks: int, status: int) -> bytes:
    """Build an ESP32 -> Pi encoder packet (0x55, 12 B)."""
    body = struct.pack('<BBiiB', SYNC, P1_ID, int(left_ticks), int(right_ticks), int(status) & 0xFF)
    return body + bytes([xor_checksum(body)])


def pack_packet2(obstacle_state: int) -> bytes:
    """Build a Pi -> ESP32 drive/obstacle-state packet (0x66, 4 B)."""
    body = struct.pack('<BBB', SYNC, P2_ID, int(obstacle_state) & 0xFF)
    return body + bytes([xor_checksum(body)])


def pack_packet3(seq: int, chunk_id: int, total: int, payload: bytes,
                 msg_id: int = P3_ID) -> bytes:
    """Build a Pi -> ESP32 -> GS map chunk frame (0x06, <= 205 B)."""
    if len(payload) > P3_MAX_PAYLOAD:
        raise ValueError(f'payload {len(payload)} > {P3_MAX_PAYLOAD}')
    body = struct.pack(
        P3_HDR_FMT, SYNC, msg_id & 0xFF, seq & 0xFF, chunk_id & 0xFF, total & 0xFF, len(payload)
    ) + payload
    return body + bytes([xor_checksum(body)])


def unpack_packet3(frame: bytes) -> Optional[Tuple[int, int, int, bytes]]:
    """Validate a MAP (0x06 or legacy 0x77) frame. Returns (seq, chunk_id, total, payload) or None."""
    if len(frame) < P3_HDR_LEN + 1:
        return None
    sync1, sync2, seq, chunk_id, total, plen = struct.unpack(P3_HDR_FMT, frame[:P3_HDR_LEN])
    if sync1 != SYNC or sync2 not in (P3_ID, P3_LEGACY_ID) or plen > P3_MAX_PAYLOAD:
        return None
    if len(frame) != P3_HDR_LEN + plen + 1:
        return None
    if xor_checksum(frame[:-1]) != frame[-1]:
        return None
    return seq, chunk_id, total, bytes(frame[P3_HDR_LEN:P3_HDR_LEN + plen])


def pack_packet4(cmd_id: int, distance_m: float = 0.0, angle_deg: float = 0.0,
                 seq: int = 0) -> bytes:
    """Build a 0x01 COMMAND packet (13 B, <BBBBffB)."""
    body = struct.pack('<BBBBff', SYNC, P4_ID, seq & 0xFF, cmd_id & 0xFF,
                       float(distance_m), float(angle_deg))
    return body + bytes([xor_checksum(body)])


def unpack_packet4(frame: bytes) -> Optional[Tuple[int, float, float, int]]:
    """Validate a COMMAND frame (13 B 0x01 or legacy 8 B 0x88).

    Returns (cmd_id, distance_m, angle_deg, seq) or None.
    """
    if len(frame) == P4_LEN:
        sync1, sync2, seq, cmd_id, dist, angle, chk = struct.unpack(P4_FMT, frame)
        if sync1 != SYNC or sync2 != P4_ID or xor_checksum(frame[:-1]) != chk:
            return None
        return cmd_id, dist, angle, seq
    if len(frame) == P4_LEGACY_LEN:
        sync1, sync2, cmd_id, dist, chk = struct.unpack(P4_LEGACY_FMT, frame)
        if sync1 != SYNC or sync2 != P4_LEGACY_ID or xor_checksum(frame[:-1]) != chk:
            return None
        return cmd_id, dist, 0.0, 0
    return None


def pack_pi_telemetry(velocity_mps: float, cpu_temp_c: int, cpu_pct: int,
                      ram_mb: int, pi_flags: int) -> bytes:
    """Build a Pi -> ESP32 internal telemetry frame (0x90, 12 B, <BBfBBHBB)."""
    t_clamped = max(0, min(255, int(round(cpu_temp_c))))
    p_clamped = max(0, min(100, int(round(cpu_pct))))
    r_clamped = max(0, min(65535, int(round(ram_mb))))
    body = struct.pack('<BBfBBHB', SYNC, PI_TEL_ID, float(velocity_mps),
                       t_clamped, p_clamped, r_clamped, int(pi_flags) & 0xFF)
    return body + bytes([xor_checksum(body)])


def unpack_pi_telemetry(frame: bytes) -> Optional[Tuple[float, int, int, int, int]]:
    """Validate a 0x90 PI_TELEMETRY frame. Returns (velocity_mps, cpu_temp_c, cpu_pct, ram_mb, pi_flags)."""
    if len(frame) != PI_TEL_LEN:
        return None
    sync1, sync2, vel, cpu_t, cpu_p, ram_mb, flags, chk = struct.unpack(PI_TEL_FMT, frame)
    if sync1 != SYNC or sync2 != PI_TEL_ID or xor_checksum(frame[:-1]) != chk:
        return None
    return vel, cpu_t, cpu_p, ram_mb, flags


class Packet1Parser:
    """Non-blocking byte-wise state machine: parses 0x55 (ENCODERS) and 0x01/0x88 (COMMAND),
    deduplicates repeated 0x01 command sequence numbers, and skips unknown packet IDs."""

    _FRAME_LENS = {
        P1_ID: P1_LEN,
        P4_ID: P4_LEN,
        P4_LEGACY_ID: P4_LEGACY_LEN,
    }

    def __init__(self):
        self._buf = bytearray()
        self.bad_checksums = 0
        self.commands: List[Tuple[int, float, float, int]] = []
        self._last_cmd_seq: Optional[int] = None

    def pop_commands(self) -> List[Tuple[int, float, float]]:
        """Return and clear parsed (cmd_id, distance_m, angle_deg) tuples."""
        cmds = [(cmd_id, dist, angle) for cmd_id, dist, angle, _ in self.commands]
        self.commands.clear()
        return cmds

    def feed(self, data: bytes) -> List[Tuple[int, int, int]]:
        """Consume bytes; return list of (left_ticks, right_ticks, status) for valid Packet 1s."""
        out = []
        for byte in data:
            n = len(self._buf)
            if n == 0:
                if byte == SYNC:
                    self._buf.append(byte)
                continue
            if n == 1:
                if byte in self._FRAME_LENS:
                    self._buf.append(byte)
                elif byte == SYNC:
                    pass  # stay on a fresh 0xAA
                else:
                    # Unknown packet ID: drop header and resync
                    self._buf.clear()
                continue
            self._buf.append(byte)
            target_len = self._FRAME_LENS[self._buf[1]]
            if len(self._buf) == target_len:
                frame = bytes(self._buf)
                self._buf.clear()
                if xor_checksum(frame[:-1]) == frame[-1]:
                    pkt_id = frame[1]
                    if pkt_id == P1_ID:
                        _, _, left, right, status, _ = struct.unpack(P1_FMT, frame)
                        out.append((left, right, status))
                    elif pkt_id == P4_ID:
                        _, _, seq, cmd_id, dist, angle, _ = struct.unpack(P4_FMT, frame)
                        if self._last_cmd_seq != seq:
                            self._last_cmd_seq = seq
                            self.commands.append((cmd_id, dist, angle, seq))
                    else:
                        _, _, cmd_id, dist, _ = struct.unpack(P4_LEGACY_FMT, frame)
                        self.commands.append((cmd_id, dist, 0.0, 0))
                else:
                    self.bad_checksums += 1
                    # A real sync pair may hide inside the rejected bytes; resync from them.
                    out.extend(self.feed(frame[1:]))
        return out


def msg_data_to_bytes(data) -> bytes:
    """Normalise ByteMultiArray.data (bytes, list of ints or list of 1-byte bytes) to bytes."""
    if isinstance(data, (bytes, bytearray)):
        return bytes(data)
    return b''.join(bytes([x]) if isinstance(x, int) else bytes(x) for x in data)


def bytes_to_msg_data(frame: bytes) -> list:
    """Convert bytes to the ByteMultiArray.data representation (list of 1-byte bytes)."""
    return [bytes([b]) for b in frame]
