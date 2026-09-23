"""Tests for the Phase 2 UART transport and Esp32ThrusterProvider.

Mirrors the ESP32 firmware protocol exactly (the firmware is the source of
truth):

    AA 55 | VERSION=01 | TYPE | LENGTH | SEQ(uint16 LE) | PAYLOAD | CRC16
    CRC-16/CCITT-FALSE over the ENTIRE frame INCLUDING the 0xAA 0x55 header.

Message types: 0x01 MOTOR_COMMAND, 0x02 ESTOP, 0x81 ACK, 0x82 STATUS.
E-STOP is a message TYPE (not a FLAGS bit). ACK declares LENGTH=3 but uses a
5-byte payload region (status + 2-byte echoed sequence). STATUS declares
LENGTH=14 but only its first 12 payload bytes are meaningful
(failsafe, estop, M1..M5 x int16).

Run from the project root:

    python -m unittest tests.test_uart_transport -v
"""

import os
import struct
import sys
import unittest
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from modules.controller.controller import MotionState
from modules.thrusters.thruster_manager import (
    DEFAULT_GPIO,
    Esp32ThrusterProvider,
    ThrusterId,
    ThrusterManager,
    mix,
)
from modules.thrusters.uart_transport import (
    HEADER,
    PROTOCOL_VERSION,
    MSG_ACK,
    MSG_ESTOP,
    MSG_MOTOR_COMMAND,
    MSG_STATUS,
    MOTOR_SCALE,
    MAX_FRAME_SIZE,
    Packet,
    IncompleteFrame,
    UartTransport,
    build_estop,
    build_frame,
    build_motor_command,
    crc16_ccitt,
    decode_ack,
    decode_motor_command,
    decode_status,
    motor_values_to_int16,
    parse_frame,
)

M1 = ThrusterId.M1_FRONT_VERTICAL
M2 = ThrusterId.M2_MIDDLE_RIGHT_HORIZONTAL
M3 = ThrusterId.M3_MIDDLE_LEFT_HORIZONTAL
M4 = ThrusterId.M4_BACK_RIGHT_VERTICAL
M5 = ThrusterId.M5_BACK_LEFT_VERTICAL
ALL = (M1, M2, M3, M4, M5)

ESP32_CONFIG = {
    "serial_port": "/dev/ttyUSB0",
    "baudrate": 460800,
    "timeout_ms": 100,
    "command_rate_hz": 10000,
    "heartbeat_timeout_ms": 1500,
    "reconnect_interval_s": 1.0,
}
PROVIDER_CONFIG = {
    "provider": "esp32",
    "esp32": ESP32_CONFIG,
    "motors": {
        t.name: {"gpio": DEFAULT_GPIO[t], "direction": 1} for t in ALL
    },
}


def firmware_crc(frame):
    """CRC exactly as the ESP32 firmware computes it (covers the AA 55 header)."""
    return crc16_ccitt(frame[: len(frame) - 2])


def firmware_process(frame):
    """Mimic the ESP32 firmware packet processor for cross-checks."""
    if len(frame) < 9 or frame[:2] != HEADER:
        return ("bad-header",)
    if frame[2] != PROTOCOL_VERSION:
        return ("bad-version",)
    mtype, length = frame[3], frame[4]
    seq = struct.unpack_from("<H", frame, 5)[0]
    if len(frame) != 7 + length + 2:
        return ("bad-length",)
    if struct.unpack_from("<H", frame, len(frame) - 2)[0] != firmware_crc(frame):
        return ("crc-mismatch",)
    payload = frame[7 : 7 + length]
    if mtype == MSG_MOTOR_COMMAND:
        return ("motor", seq, struct.unpack("<5h", payload[:10]), payload[10])
    if mtype == MSG_ESTOP:
        return ("estop", seq)
    return ("unknown", mtype)


def build_ack(acked_seq, status=0x00):
    """Build a firmware-style ACK (LENGTH=3, 5-byte payload region)."""
    echo = bytes(struct.pack("<H", acked_seq))
    return build_frame(acked_seq, MSG_ACK, bytes([status]) + echo)


def build_status(esp_seq, failsafe=0, estop=0, motors=(0, 0, 0, 0, 0)):
    """Build the exact frame the ESP32 firmware sendStatus() emits.

    AA55 | VERSION | TYPE=0x82 | LENGTH=14 | SEQ(2) FAILSAFE(1) ESTOP(1)
    M1..M5 int16(10) | CRC16 -> 21 bytes on the wire (LENGTH counts the
    2-byte sequence as part of the payload).
    """
    payload = struct.pack("<H", esp_seq & 0xFFFF)
    payload += bytes([failsafe, estop])
    payload += motor_values_to_int16(motors)
    head = struct.pack("<BBB", PROTOCOL_VERSION, MSG_STATUS, len(payload))
    frame = HEADER + head + payload
    return frame + struct.pack("<H", crc16_ccitt(frame))


class FakeSerial:
    """In-memory stand-in for ``serial.Serial``."""

    def __init__(self, port, baudrate, timeout, write_timeout, **kwargs):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.is_open = True
        self.out = bytearray()
        self.inbuf = bytearray()
        self.fail_on_write = False

    def write(self, data):
        if self.fail_on_write:
            raise OSError("device gone")
        self.out.extend(data)
        return len(data)

    def read(self, size):
        take = self.inbuf[:size]
        del self.inbuf[:size]
        return bytes(take)

    @property
    def in_waiting(self):
        return len(self.inbuf)

    def flush(self):
        pass

    def close(self):
        self.is_open = False

    def open(self):
        self.is_open = True


class FailingSerial(FakeSerial):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.is_open = False
        raise OSError("no such device")


class PacketBuildingTest(unittest.TestCase):
    def test_motor_command_frame_layout(self):
        frame = build_motor_command(7, [1.0, -0.5, 0.25, 0.0, -1.0])
        self.assertEqual(frame[:2], HEADER)
        self.assertEqual(frame[2], PROTOCOL_VERSION)
        self.assertEqual(frame[3], MSG_MOTOR_COMMAND)
        self.assertEqual(frame[4], 11)  # 5 x int16 + FLAGS byte
        self.assertEqual(struct.unpack_from("<H", frame, 5)[0], 7)  # uint16 LE seq
        values = struct.unpack("<5h", frame[7:17])
        self.assertEqual(values, (1000, -500, 250, 0, -1000))
        self.assertEqual(frame[17], 0)  # FLAGS byte

    def test_crc_covers_header_and_matches_firmware(self):
        frame = build_motor_command(1, [0.1, 0.2, 0.3, 0.4, 0.5])
        # CRC covers bytes 0..total-3: INCLUDES the AA 55 header.
        self.assertEqual(firmware_crc(frame), struct.unpack("<H", frame[-2:])[0])
        self.assertEqual(
            crc16_ccitt(b"123456789"), 0x29B1
        )  # CCITT-FALSE check value

    def test_values_are_clamped_before_transmission(self):
        frame = build_motor_command(1, [5.0, -9.0, 0.0, 1.0, -1.0])
        values = struct.unpack("<5h", frame[7:17])
        self.assertEqual(values, (1000, -1000, 0, 1000, -1000))

    def test_firmware_accepts_built_frames(self):
        frame = build_motor_command(3, [1.0, -1.0, 0.5, -0.5, 0.0])
        self.assertEqual(
            firmware_process(frame),
            ("motor", 3, (1000, -1000, 500, -500, 0), 0),
        )
        ef = build_estop(9)
        self.assertEqual(firmware_process(ef), ("estop", 9))
        self.assertEqual(ef[4], 0)  # ESTOP has zero-length payload

    def test_reserved_flags_rejected(self):
        with self.assertRaises(ValueError):
            build_motor_command(1, [0] * 5, flags=0x04)

    def test_parse_roundtrip(self):
        frame = build_motor_command(42, [0.1, -0.9, 0.5, 0.0, 1.0])
        packet, consumed = parse_frame(frame)
        self.assertIsInstance(packet, Packet)
        self.assertEqual(consumed, len(frame))
        self.assertEqual(packet.mtype, MSG_MOTOR_COMMAND)
        self.assertEqual(packet.sequence, 42)
        motors, flags = decode_motor_command(packet.payload)
        self.assertEqual(motors, (100, -900, 500, 0, 1000))
        self.assertEqual(flags, 0)

    def test_parse_rejects_bad_crc(self):
        frame = bytearray(build_motor_command(1, [0, 0, 0, 0, 0]))
        frame[-1] ^= 0xFF
        with self.assertRaises(ValueError) as ctx:
            parse_frame(bytes(frame))
        self.assertEqual(str(ctx.exception), "crc")

    def test_parse_rejects_header_modified_crc(self):
        # A single flip inside the AA 55 header must also fail CRC.
        frame = bytearray(build_motor_command(1, [0, 0, 0, 0, 0]))
        frame[1] ^= 0xFF
        self.assertNotEqual(
            crc16_ccitt(bytes(frame[:-2])),
            struct.unpack("<H", bytes(frame[-2:]))[0],
        )

    def test_parse_short_frame_raises_incomplete(self):
        frame = build_motor_command(1, [0, 0, 0, 0, 0])
        with self.assertRaises(IncompleteFrame):
            parse_frame(frame[:8])

    def test_parse_rejects_oversized_length(self):
        # LENGTH would push past the ESP32's 64-byte MAX_PACKET_SIZE.
        bad = HEADER + bytes([PROTOCOL_VERSION, MSG_ESTOP, 100]) + b"\x00\x00\x00\x00"
        with self.assertRaises(ValueError):
            parse_frame(bad)
        self.assertEqual(7 + 100 + 2, 109)
        self.assertGreater(109, MAX_FRAME_SIZE)

    def test_motor_values_to_int16_scale(self):
        payload = motor_values_to_int16([1.0, -1.0, 0.5, -0.5, 0.0])
        self.assertEqual(struct.unpack("<5h", payload), (1000, -1000, 500, -500, 0))


class AckStatusParsingTest(unittest.TestCase):
    def test_ack_ok(self):
        ack = build_ack(5, status=0x00)
        packet, consumed = parse_frame(ack)
        self.assertEqual(consumed, 12)  # total = 7 + LENGTH(3) + 2
        self.assertEqual(packet.mtype, MSG_ACK)
        status, echo = decode_ack(packet.payload)
        self.assertEqual((status, echo), (0, 5))

    def test_ack_statuses(self):
        for status in range(8):
            ack = build_ack(1, status=status)
            packet, _ = parse_frame(ack)
            got, echo = decode_ack(packet.payload)
            self.assertEqual(got, status)
            self.assertEqual(echo, 1)

    def test_status_frame(self):
        st = build_status(
            9, failsafe=1, estop=0, motors=(0.1, -0.1, 0.05, 0.0, -0.05)
        )
        self.assertEqual(len(st), 21)  # 5 + LENGTH(14) + CRC(2), no extra seq slot
        packet, consumed = parse_frame(st)
        self.assertEqual(consumed, 21)
        self.assertEqual(packet.mtype, MSG_STATUS)
        self.assertEqual(packet.sequence, 9)
        info = decode_status(packet.payload)
        self.assertEqual(
            info,
            {
                "sequence": 9,
                "failsafe": True,
                "estop": False,
                "motors": (100, -100, 50, 0, -50),
            },
        )


# -- transport tests -------------------------------------------------------


class TransportTest(unittest.TestCase):
    def _transport(self, **kwargs):
        cfg = dict(ESP32_CONFIG)
        cfg.update(kwargs)
        return UartTransport(cfg, serial_factory=FakeSerial)

    def test_open_success(self):
        t = self._transport()
        self.assertTrue(t.open())
        self.assertTrue(t.is_open())

    def test_open_without_device_sets_error(self):
        t = UartTransport(dict(ESP32_CONFIG), serial_factory=FailingSerial)
        self.assertFalse(t.open())
        self.assertFalse(t.is_open())
        self.assertGreaterEqual(t.stats["open_failures"], 1)
        self.assertIsNotNone(t.stats["last_error"])

    def test_open_without_port_configured(self):
        cfg = {"baudrate": 115200}
        t = UartTransport(cfg, serial_factory=FakeSerial)
        self.assertFalse(t.open())
        self.assertIn("serial_port", str(t.stats["last_error"]))

    def test_open_is_rate_limited(self):
        t = self._transport(reconnect_interval_s=60.0)
        self.assertTrue(t.open())
        t.close()
        self.assertFalse(t.is_open())
        self.assertFalse(t.open())  # throttled by reconnect_interval_s
        self.assertFalse(t.is_open())

    def test_send_writes_frame_and_counters(self):
        t = self._transport()
        t.open()
        frame = build_motor_command(1, [1, 0, 0, 0, -1])
        self.assertTrue(t.send(frame))
        self.assertEqual(t.stats["tx_frames"], 1)
        self.assertEqual(bytes(t._ser.out), frame)

    def test_send_drops_when_not_open(self):
        t = UartTransport(dict(ESP32_CONFIG), serial_factory=FailingSerial)
        self.assertFalse(t.send(b"\x00"))
        self.assertEqual(t.stats["dropped_frames"], 1)

    def test_send_failure_reconnects_next_time(self):
        t = self._transport(reconnect_interval_s=0.0)
        t.open()
        frame = build_motor_command(1, [0, 0, 0, 0, 0])
        self.assertTrue(t.send(frame))
        self.assertEqual(t.stats["tx_frames"], 1)
        t._ser.close()  # device disappears
        self.assertFalse(t.is_open())
        self.assertTrue(t.send(frame))  # auto-reopens when allowed
        self.assertEqual(t.stats["tx_frames"], 2)
        self.assertTrue(t.is_open())

    def test_pump_reads_ack_frame(self):
        t = self._transport()
        t.open()
        ack = build_ack(3)
        t._ser.inbuf.extend(ack)
        t.pump()
        packets = t.read_packets()
        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0].mtype, MSG_ACK)
        self.assertEqual(packets[0].sequence, 3)
        self.assertEqual(t.stats["rx_frames"], 1)

    def test_pump_resyncs_through_garbage(self):
        t = self._transport()
        t.open()
        status = build_status(9)
        t._ser.inbuf.extend(b"\x00\x01\x02" + status)
        t.pump()
        packets = t.read_packets()
        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0].mtype, MSG_STATUS)
        self.assertGreaterEqual(t.stats["resyncs"], 1)

    def test_pump_handles_corrupted_then_valid(self):
        t = self._transport()
        t.open()
        bad = bytearray(build_ack(1))
        bad[-1] ^= 0xFF
        good = build_ack(2)
        t._ser.inbuf.extend(bytes(bad) + good)
        t.pump()
        packets = t.read_packets()
        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0].sequence, 2)
        self.assertGreaterEqual(t.stats["crc_errors"], 1)

    def test_pump_buffers_partial_frame_in_two_reads(self):
        t = self._transport()
        t.open()
        status = build_status(5)
        half = len(status) // 2
        t._ser.inbuf.extend(status[:half])
        t.pump()
        self.assertEqual(t.read_packets(), [])
        t._ser.inbuf.extend(status[half:])
        t.pump()
        packets = t.read_packets()
        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0].sequence, 5)

    def test_close_is_safe_and_idempotent(self):
        t = self._transport()
        t.open()
        t.close()
        t.close()
        self.assertFalse(t.is_open())


# -- provider tests --------------------------------------------------------


def ack_provider(provider, transport, seq, status=0x00):
    """Feed a firmware-style ACK for a transmitted command into the provider."""
    transport._ser.inbuf.extend(build_ack(seq, status=status))
    provider._pump()


def ack_last(provider, transport, status=0x00):
    """ACK the most recently transmitted sequence (realistic handshake)."""
    out = bytes(transport._ser.out)
    frame = out[out.rfind(b"\xaa\x55"):]
    seq = struct.unpack_from("<H", frame, 5)[0]
    ack_provider(provider, transport, seq, status=status)


class Esp32ProviderTest(unittest.TestCase):
    def _open_provider(self):
        transport = UartTransport(dict(ESP32_CONFIG), serial_factory=FakeSerial)
        provider = Esp32ThrusterProvider(
            PROVIDER_CONFIG, sensors=None, transport=transport
        )
        provider._command_interval = 0.0  # disable rate limiting for these tests
        transport.open()
        return provider, transport

    def _last_frame(self, transport):
        """Return bytes from the last frame's header to end of the buffer."""
        out = bytes(transport._ser.out)
        return out[out.rfind(b"\xaa\x55"):]

    def test_setpoints_identical_to_mix(self):
        provider, _ = self._open_provider()
        motion = MotionState(surge=0.5, yaw=-0.3, heave=0.2)
        self.assertEqual(provider.setpoints(motion), mix(motion))

    def test_starts_gated_to_zero_until_acked(self):
        provider, transport = self._open_provider()
        provider.apply(MotionState(surge=1.0, heave=1.0), 0.1, setpoints={t: 1.0 for t in ALL})
        frame = self._last_frame(transport)
        packet, _ = parse_frame(frame)
        motors, flags = decode_motor_command(packet.payload)
        self.assertEqual(motors, (0, 0, 0, 0, 0))  # safety gate -> zeros
        self.assertEqual(flags, 0)

    def test_ack_ok_engages_and_sends_mixed_values(self):
        provider, transport = self._open_provider()
        # Send the zero handshake, then ACK it -> gate lifts.
        provider.apply(MotionState(), 0.1, setpoints={t: 0.0 for t in ALL})
        ack_last(provider, transport)
        self.assertFalse(provider.status()["failsafe"])
        self.assertFalse(provider.status()["gate"])
        setpoints = {M1: 1.0, M2: -1.0, M3: 0.5, M4: -0.5, M5: 0.0}
        provider.apply(MotionState(), 0.1, setpoints=setpoints)
        frame = self._last_frame(transport)
        packet, _ = parse_frame(frame)
        motors, flags = decode_motor_command(packet.payload)
        self.assertEqual(
            motors,
            tuple(int(round(setpoints[t] * MOTOR_SCALE)) for t in ALL),
        )
        self.assertEqual(flags, 0)

    def test_apply_reclamps_out_of_range_setpoints(self):
        provider, transport = self._open_provider()
        provider.apply(MotionState(), 0.1, setpoints={t: 0.0 for t in ALL})
        ack_last(provider, transport)
        setpoints = {t: 9.0 if i % 2 == 0 else -9.0 for i, t in enumerate(ALL)}
        provider.apply(MotionState(), 0.1, setpoints=setpoints)
        frame = self._last_frame(transport)
        packet, _ = parse_frame(frame)
        motors, _ = decode_motor_command(packet.payload)
        self.assertTrue(all(v in (1000, -1000) for v in motors))

    def test_sequence_is_uint16_and_increments(self):
        provider, transport = self._open_provider()
        provider.apply(MotionState(), 0.1, setpoints={t: 0.5 for t in ALL})
        ack_last(provider, transport)  # first frame is the zero handshake (seq 1)
        provider.apply(MotionState(), 0.1, setpoints={t: -0.5 for t in ALL})
        seq2 = parse_frame(self._last_frame(transport))[0].sequence
        self.assertEqual(seq2, 2)

    def test_estop_sends_estop_message_not_flag(self):
        provider, transport = self._open_provider()
        provider.apply(MotionState(surge=1.0), 0.1, setpoints={t: 1.0 for t in ALL})
        ack_last(provider, transport)
        provider.apply(MotionState(surge=1.0), 0.1, setpoints={t: 1.0 for t in ALL})
        provider.notify_estop(True)
        frame = self._last_frame(transport)
        packet, _ = parse_frame(frame)
        self.assertEqual(packet.mtype, MSG_ESTOP)  # a message type, not a flag
        self.assertEqual(packet.length, 0)

    def test_apply_while_estop_never_sends_motion(self):
        provider, transport = self._open_provider()
        provider.notify_estop(True)
        provider.apply(MotionState(surge=1.0, heave=1.0), 0.1, setpoints={t: 1.0 for t in ALL})
        frame = self._last_frame(transport)
        packet, _ = parse_frame(frame)
        self.assertEqual(packet.mtype, MSG_ESTOP)

    def test_rearm_streams_zeros_until_acked(self):
        provider, transport = self._open_provider()
        provider.apply(MotionState(surge=1.0), 0.1, setpoints={t: 1.0 for t in ALL})
        ack_last(provider, transport)
        provider.apply(MotionState(surge=1.0), 0.1, setpoints={t: 1.0 for t in ALL})
        provider.notify_estop(True)
        provider.notify_estop(False)  # re-arm: zeros only until ACK
        provider.apply(MotionState(surge=1.0), 0.1, setpoints={t: 1.0 for t in ALL})
        frame = self._last_frame(transport)
        packet, _ = parse_frame(frame)
        self.assertEqual(packet.mtype, MSG_MOTOR_COMMAND)
        motors, _ = decode_motor_command(packet.payload)
        self.assertEqual(motors, (0, 0, 0, 0, 0))  # still zero: re-arming
        self.assertTrue(provider.status().get("rearming"))
        # ACK the re-arm zero -> gate lifts, motion resumes.
        seq = packet.sequence
        ack_provider(provider, transport, seq)
        self.assertFalse(provider.status().get("rearming"))
        provider.apply(MotionState(surge=1.0), 0.1, setpoints={t: 1.0 for t in ALL})
        packet2, _ = parse_frame(self._last_frame(transport))
        motors2, _ = decode_motor_command(packet2.payload)
        self.assertEqual(motors2, (1000, 1000, 1000, 1000, 1000))

    def test_estop_keepalive_keeps_esstop(self):
        provider, transport = self._open_provider()
        provider.notify_estop(True)
        provider.estop_keepalive(0.1, {t: 0.0 for t in ALL})
        frame = self._last_frame(transport)
        packet, _ = parse_frame(frame)
        self.assertEqual(packet.mtype, MSG_ESTOP)

    def test_ack_updates_status_and_latency(self):
        provider, transport = self._open_provider()
        provider.apply(MotionState(), 0.1, setpoints={t: 0.0 for t in ALL})
        ack_last(provider, transport)
        status = provider.status()
        self.assertEqual(status["last_ack_seq"], 1)
        self.assertIsNotNone(status["last_ack_latency_s"])
        self.assertEqual(status["last_ack_status"], 0)

    def test_bad_ack_status_counts_missing_and_logs(self):
        provider, transport = self._open_provider()
        provider.apply(MotionState(), 0.1, setpoints={t: 0.0 for t in ALL})
        ack_last(provider, transport, status=0x07)  # estop active
        status = provider.status()
        self.assertEqual(status["last_ack_status"], 0x07)
        self.assertGreaterEqual(status["missing_acks"], 1)

    def test_status_seeds_sequence_and_reports_esp32(self):
        provider, transport = self._open_provider()
        provider._seq_seeded = False
        transport._ser.inbuf.extend(
            build_status(120, failsafe=0, estop=0, motors=(0.9, 0, 0, 0, -0.9))
        )
        provider._pump()
        status = provider.status()
        self.assertEqual(status["esp32_seq"], 120)
        self.assertEqual(status["esp32_motors"], (900, 0, 0, 0, -900))
        self.assertFalse(status["esp32_failsafe"])
        self.assertFalse(status["esp32_estop"])
        # Firmware only accepts STRICTLY newer sequences, so seed one ahead.
        self.assertEqual(provider._seq, 121)

    def test_status_with_failsafe_reasserts_gate(self):
        provider, transport = self._open_provider()
        transport._ser.inbuf.extend(build_status(3, failsafe=1))
        provider._pump()
        self.assertTrue(provider._gate)
        self.assertTrue(provider._failsafe)

    def test_stale_link_reasserts_gate(self):
        provider, transport = self._open_provider()
        provider.apply(MotionState(), 0.1, setpoints={t: 0.0 for t in ALL})
        ack_last(provider, transport)
        self.assertFalse(provider._gate)
        provider._link_timeout = 0.01
        provider._last_rx_at = time.time() - 1.0
        provider._check_link()
        self.assertTrue(provider._failsafe)
        self.assertTrue(provider._gate)

    def test_disconnected_provider_does_not_crash(self):
        transport = UartTransport(dict(ESP32_CONFIG), serial_factory=FailingSerial)
        provider = Esp32ThrusterProvider(
            PROVIDER_CONFIG, sensors=None, transport=transport
        )
        provider.apply(MotionState(surge=1.0), 0.1, setpoints={t: 1.0 for t in ALL})
        status = provider.status()
        self.assertFalse(status["connected"])
        self.assertGreaterEqual(status["dropped_frames"], 1)
        self.assertTrue(status["failsafe"])

    def test_stop_zeroes_then_closes(self):
        provider, transport = self._open_provider()
        provider.apply(MotionState(surge=1.0), 0.1, setpoints={t: 1.0 for t in ALL})
        ack_last(provider, transport)
        provider.apply(MotionState(surge=1.0), 0.1, setpoints={t: 1.0 for t in ALL})
        ser = transport._ser  # keep a handle: close() drops transport._ser
        provider.stop()
        self.assertFalse(provider.is_connected())
        out = bytes(ser.out)
        last = parse_frame(out[out.rfind(b"\xaa\x55"):])[0]
        self.assertEqual(last.mtype, MSG_MOTOR_COMMAND)
        motors, _ = decode_motor_command(last.payload)
        self.assertEqual(motors, (0, 0, 0, 0, 0))  # final zeroed command

    def test_manager_accepts_esp32_provider_without_hardware(self):
        mgr = ThrusterManager(PROVIDER_CONFIG, sensors=None)
        self.assertTrue(mgr.initialize())
        mgr.start()
        mgr.set_motion(MotionState(surge=1.0))
        mgr.update()
        mgr.update()
        self.assertIsInstance(mgr.provider, Esp32ThrusterProvider)
        self.assertFalse(mgr.provider.is_connected())
        mgr.stop()

    def test_manager_estop_reaches_provider(self):
        mgr = ThrusterManager(PROVIDER_CONFIG, sensors=None)
        mgr.initialize()
        mgr.emergency_stop()
        self.assertTrue(mgr.provider._estop)
        mgr.clear_emergency_stop()
        self.assertFalse(mgr.provider._estop)
        mgr.stop()

    def test_manager_failsafe_keeps_setpoints_but_applies_zeros(self):
        """Failsafe must NOT flatten the internal ramped setpoints.

        ''last_setpoints'' is the truth the GUI and power readout report: with
        W held the ramp still climbs M2/M3 (0 -> 0.1 -> ... -> soft ceiling)
        even while the ESP32 link is down. Safety is preserved because the
        provider is handed ZEROS to apply, and the ESP32 provider additionally
        keeps its wire-level gate closed while not ACKed.
        """
        mgr = ThrusterManager(PROVIDER_CONFIG, sensors=None)
        mgr.initialize()
        mgr.start()
        mgr.set_throttle_mode("soft")
        mgr.set_motion(MotionState(surge=1.0))
        mgr.update()  # not connected -> esp32 failsafe active
        for _ in range(5):  # 0 -> .1 -> .2 -> .3 -> .4 -> .4 (soft ceiling)
            mgr.update()
        # Internal setpoints are the real, correct ramped values ...
        self.assertAlmostEqual(mgr.last_setpoints[M2], 0.40, places=6)
        self.assertAlmostEqual(mgr.last_setpoints[M3], 0.40, places=6)
        self.assertEqual(mgr.last_setpoints[M1], 0.0)  # surge-only: no heave
        # ... but what the provider applies/transmits stays zero (failsafe).
        self.assertTrue(mgr.provider.status()["failsafe"])
        self.assertEqual(mgr.provider._pending, [0.0] * 5)
        mgr.stop()

    def test_initialize_starts_thread_once(self):
        provider, transport = self._open_provider()
        self.assertTrue(provider.initialize())
        self.assertIsNotNone(provider._thread)
        provider.stop()
        self.assertIsNone(provider._thread)


class UartConfigTest(unittest.TestCase):
    """Lock the real Pi<->ESP32 wiring + baud in configs/thrusters.yaml.

    The ESP32 uses its UART0 pins (RX0 = GPIO3, TX0 = GPIO1); the Pi uses the
    on-board /dev/ttyAMA0 (UART0, GPIO14 TXD0 / GPIO15 RXD0) at 460800 baud.
    These values must not silently regress.
    """

    def _esp32_cfg(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        path = os.path.join(root, "configs", "thrusters.yaml")
        with open(path) as f:
            import yaml

            data = yaml.safe_load(f)
        return data["thrusters"]["esp32"]

    def test_pi_device_and_baud(self):
        cfg = self._esp32_cfg()
        self.assertEqual(cfg["serial_port"], "/dev/ttyAMA0")
        self.assertEqual(cfg["baudrate"], 460800)

    def test_esp32_uart0_pins(self):
        cfg = self._esp32_cfg()
        self.assertEqual(cfg["rx_pin"], 3)  # ESP32 GPIO3 (RX0)
        self.assertEqual(cfg["tx_pin"], 1)  # ESP32 GPIO1 (TX0)

    def test_motor_gpio_mapping_unchanged(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        path = os.path.join(root, "configs", "thrusters.yaml")
        import yaml

        with open(path) as f:
            motors = yaml.safe_load(f)["thrusters"]["motors"]
        expected = {
            "M1_FRONT_VERTICAL": 25,
            "M2_MIDDLE_RIGHT_HORIZONTAL": 33,
            "M3_MIDDLE_LEFT_HORIZONTAL": 32,
            "M4_BACK_RIGHT_VERTICAL": 27,
            "M5_BACK_LEFT_VERTICAL": 26,
        }
        for name, pin in expected.items():
            self.assertEqual(motors[name]["gpio"], pin)

    def test_transport_uses_config_baud(self):
        cfg = dict(self._esp32_cfg())
        cfg.update({"serial_port": "/dev/ttyAMA0", "baudrate": 460800})
        transport = UartTransport(cfg, serial_factory=FakeSerial)
        self.assertEqual(transport.port, "/dev/ttyAMA0")
        self.assertEqual(transport.baudrate, 460800)
        transport.open()
        self.assertEqual(transport._ser.baudrate, 460800)


if __name__ == "__main__":
    unittest.main(verbosity=2)