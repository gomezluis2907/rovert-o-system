"""Unit tests: ICD framing (0x01, 0x06, 0x55, 0x66, 0x90), tick clamp, map compress/decode, bridge."""
import os
import sys
import time

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import ByteMultiArray, Float32, Float32MultiArray, UInt8

from roverto_autonomy.esp32_bridge_node import Esp32BridgeNode
from roverto_autonomy.laser_odom_node import icp_2d
from roverto_autonomy.map_compressor_node import compress_grid, frame_chunks
from roverto_autonomy.protocol import (
    bytes_to_msg_data, P3_ID, pack_packet1, pack_packet2, pack_packet3, pack_packet4,
    pack_pi_telemetry, Packet1Parser, unpack_packet3, unpack_packet4,
    unpack_pi_telemetry, xor_checksum)

GS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..',
                                  'ground_station_gui'))
sys.path.insert(0, GS)
import gs_map_decoder  # noqa: E402
from gs_map_decoder import MapDecoder  # noqa: E402
gs_map_decoder.SYNC2 = P3_ID  # in-memory match to ICD 0x06 while GS folder stays pending


def test_packet1_roundtrip_and_resync():
    p = Packet1Parser()
    good = pack_packet1(-1234, 5678, 2)
    assert len(good) == 12
    bad = bytearray(pack_packet1(1, 2, 1))
    bad[-1] ^= 0xFF
    cmd = pack_packet4(1, 5.0, -45.0, seq=7)
    assert len(cmd) == 13
    assert unpack_packet4(cmd) == (1, pytest.approx(5.0), pytest.approx(-45.0), 7)
    # Example from docs/ICD.md section 1: AA 01 04 01 00 00 00 40 00 00 00 00 EE
    icd_example = pack_packet4(1, 2.0, 0.0, seq=4)
    assert icd_example == bytes.fromhex('AA 01 04 01 00 00 00 40 00 00 00 00 EE')
    # Unknown packet ID (0xAA 0x99) from Electronics is ignored without breaking sync;
    # duplicate seq=7 command is deduplicated.
    out = p.feed(b'\x00\x13\xAA\x99\x01\x02' + bytes(bad) + cmd + cmd + b'\xAA' + good[:5])
    out += p.feed(good[5:])
    assert out == [(-1234, 5678, 2)]
    assert p.pop_commands() == [(1, pytest.approx(5.0), pytest.approx(-45.0))]
    assert p.bad_checksums == 1


def test_packet2_3_and_pi_telemetry_framing():
    p2 = pack_packet2(3)
    assert len(p2) == 4 and p2[:2] == b'\xAA\x66' and p2[3] == xor_checksum(p2[:3])
    f = pack_packet3(7, 1, 4, b'hello')
    assert f[:6] == bytes([0xAA, 0x06, 7, 1, 4, 5]) and len(f) == 12
    assert unpack_packet3(f) == (7, 1, 4, b'hello')
    assert unpack_packet3(f[:-1] + bytes([f[-1] ^ 1])) is None
    with pytest.raises(ValueError):
        pack_packet3(0, 0, 1, bytes(199))
    pt = pack_pi_telemetry(0.04, 54, 32, 1450, 0x03)
    assert len(pt) == 12 and pt[:2] == b'\xAA\x90'
    vel, cpu_t, cpu_p, ram_mb, flags = unpack_pi_telemetry(pt)
    assert vel == pytest.approx(0.04, abs=1e-5)
    assert (cpu_t, cpu_p, ram_mb, flags) == (54, 32, 1450, 0x03)


def test_compress_decode_roundtrip():
    h, w = 40, 60
    data = -np.ones((h, w), dtype=np.int8)
    data[5:30, 10:50] = 0
    data[5, 10:50] = 100
    data[12, 20] = 49
    data[13, 21] = 50
    raw, comp = compress_grid(w, h, 0.05, -1.0, -2.0, data.ravel())
    frames = frame_chunks(comp, 9)
    assert all(len(fr) <= 205 for fr in frames)
    dec, res = MapDecoder(), None
    for fr in frames:
        res = dec.parse_packet(fr)
    grid, meta = res
    assert (meta['width'], meta['height'], meta['resolution_mm']) == (40, 25, 50)
    assert (meta['origin_x_cm'], meta['origin_y_cm']) == (-50, -175)
    exp = np.where(data[5:30, 10:50] == -1, 0, np.where(data[5:30, 10:50] >= 50, 2, 1))
    assert (grid == exp).all()


def test_decoder_incomplete_map_reset():
    import logging
    records = []

    class H(logging.Handler):
        def emit(self, record):
            records.append(record.getMessage())
    handler = H()
    logging.getLogger('gs_map_decoder').addHandler(handler)
    data = np.zeros((300, 300), dtype=np.int8)
    data[::7, ::5] = 100
    rng = np.random.default_rng(0)
    data[rng.random(data.shape) > 0.5] = -1
    data[0, 0] = 0
    _, comp = compress_grid(300, 300, 0.05, 0, 0, data.ravel())
    f_old = frame_chunks(comp, 1)
    f_new = frame_chunks(comp, 2)
    assert len(f_old) > 1
    dec = MapDecoder()
    assert dec.parse_packet(f_old[0]) is None
    for fr in f_new:
        res = dec.parse_packet(fr)
    logging.getLogger('gs_map_decoder').removeHandler(handler)
    assert res is not None and any('Incomplete map seq=1' in r for r in records)


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _spin(node, secs):
    end = time.time() + secs
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def test_icp_2d_scan_matching():
    # Generate an L-shaped wall corner in the rover's front field of view
    xs = np.linspace(0.5, 2.5, 40)
    wall_a = np.column_stack([xs, np.ones_like(xs) * 1.5])
    wall_b = np.column_stack([np.ones_like(xs) * 2.5, np.linspace(-1.0, 1.5, 40)])
    prev_pts = np.vstack([wall_a, wall_b])
    # Rover advances +0.04 m in X and rotates +0.03 rad (points shift oppositely in body frame)
    true_dx, true_dy, true_dyaw = 0.04, -0.01, 0.03
    c, s = np.cos(true_dyaw), np.sin(true_dyaw)
    rot_inv = np.array([[c, s], [-s, c]])
    curr_pts = (prev_pts - np.array([true_dx, true_dy])) @ rot_inv.T
    dx, dy, dyaw = icp_2d(curr_pts, prev_pts)
    assert dx == pytest.approx(true_dx, abs=2e-3)
    assert dy == pytest.approx(true_dy, abs=2e-3)
    assert dyaw == pytest.approx(true_dyaw, abs=2e-3)


def test_bridge_mock_loopback(ros):
    node = Esp32BridgeNode(parameter_overrides=[
        rclpy.parameter.Parameter('mock_serial', value=True)])
    odoms, gs_cmds, mission_cmds = [], [], []
    helper = rclpy.create_node('bridge_test_helper')
    helper.create_subscription(Odometry, '/odom', lambda m: odoms.append(m), 10)
    helper.create_subscription(Float32, '/gs_command', lambda m: gs_cmds.append(m.data), 10)
    helper.create_subscription(
        Float32MultiArray, '/mission_command', lambda m: mission_cmds.append(list(m.data)), 10)
    obs_pub = helper.create_publisher(UInt8, '/obstacle_state', 10)
    chunk_pub = helper.create_publisher(ByteMultiArray, '/compressed_map_chunk', 100)
    ser = node.ser
    tpm = node.tpm

    ser.inject(pack_packet1(0, 0, 0))
    ser.inject(pack_packet1(int(0.5 * tpm), int(0.5 * tpm), 1))   # 0.5 m straight
    ser.inject(pack_packet4(1, 3.5, 30.0, seq=1))                 # Advance 3.5 m, turn +30 deg
    _spin(node, 0.1)
    ser.inject(pack_packet1(int(0.5 * tpm) + 100000, int(0.5 * tpm), 1))  # jump -> clamp
    ser.inject(pack_packet1(int(1.0 * tpm), int(1.0 * tpm), 1))  # jump too (relative to jump)
    for _ in range(30):
        _spin(node, 0.01)
        rclpy.spin_once(helper, timeout_sec=0.0)
    assert node.odom_calc.x == pytest.approx(0.5, abs=1e-3)       # jumps ignored
    assert node.odom_calc.y == pytest.approx(0.0, abs=1e-6)
    assert len(odoms) == 0                                        # publish_odom defaults False
    assert gs_cmds == [pytest.approx(3.5)]                        # 0x01 forwarded to /gs_command
    assert mission_cmds == [[pytest.approx(1.0), pytest.approx(3.5), pytest.approx(30.0)]]

    obs_pub.publish(UInt8(data=3))
    for i in range(4):
        chunk_pub.publish(ByteMultiArray(data=bytes_to_msg_data(
            pack_packet3(0, i, 4, bytes([i]) * 50))))
    for _ in range(60):
        rclpy.spin_once(helper, timeout_sec=0.0)
        _spin(node, 0.01)
    p2 = [w for _, w in ser.writes if len(w) == 4]
    assert p2 and any(w == pack_packet2(3) for w in p2)
    p_tel = [w for _, w in ser.writes if len(w) == 12]
    assert p_tel and unpack_pi_telemetry(p_tel[0]) is not None
    p3 = [(t, w) for t, w in ser.writes if len(w) == 57]
    assert [unpack_packet3(w)[1] for _, w in p3] == [0, 1, 2, 3]
    gaps = [b[0] - a[0] for a, b in zip(p3, p3[1:])]
    assert all(g >= 0.020 for g in gaps), gaps                    # paced map chunks
    helper.destroy_node()
    node.destroy_node()


def test_mission_turn_and_commands():
    import math
    from roverto_autonomy.sim_esp32_emulator_node import (
        EXECUTING, GOAL_REACHED, SimEsp32Emulator, STANDBY)
    node = SimEsp32Emulator()
    node.pose = (0.0, 0.0, 0.0)
    # 1. Pure turn (+90 deg, 0.0 m)
    node._apply_command(1, 0.0, 90.0)
    assert node.status == EXECUTING and node.turning_to_heading
    node.on_timer()
    node.pose = (0.0, 0.0, math.radians(90.0))
    node.on_timer()
    assert node.status == GOAL_REACHED and not node.turning_to_heading
    # 2. INICIAR MISION (cmd_id=2) resets to STANDBY
    node._apply_command(2, 0.0, 0.0)
    assert node.status == STANDBY
    # 3. DETENER MISION (cmd_id=3) enters GOAL_REACHED to flush final map
    node._apply_command(3, 0.0, 0.0)
    assert node.status == GOAL_REACHED
    node.destroy_node()


def test_bridge_realignment_hysteresis(ros):
    node = Esp32BridgeNode(parameter_overrides=[
        rclpy.parameter.Parameter('mock_serial', value=True)])
    node.rover_status = 1     # EXECUTING
    node.obstacle_state = 0   # CLEAR
    node.cmd_linear_x = 0.04  # advancing after passing obstacle
    # Deflected left -> sim_esp32_emulator_node commands negative angular.z (-0.35 rad/s) to realign right
    node.cmd_angular_z = -0.35
    assert node._effective_drive_state() == 2  # AVOID_RIGHT turn sent to ESP32
    # Still turning until error drops to <= 0.06 rad/s
    node.cmd_angular_z = -0.10
    assert node._effective_drive_state() == 2
    node.cmd_angular_z = -0.04
    assert node._effective_drive_state() == 0  # back to straight forward (CLEAR)
    node.destroy_node()


