"""UART transport for the ESP32 ESC controller (Phase 2 — Raspberry Pi side).

Matches the ESP32 firmware byte-for-byte (the firmware is the source of
truth). Frame layout (little-endian):

    BYTE   FIELD        NOTES
    0-1    HEADER      0xAA 0x55
    2      VERSION      0x01
    3      TYPE         0x01 MOTOR_COMMAND (Pi->ESP32) | 0x02 ESTOP (Pi->ESP32)
                       0x81 ACK | 0x82 STATUS (ESP32->Pi)
    4      LENGTH       payload length in bytes
    5-6    SEQUENCE     uint16 little-endian, monotonically increasing
    7..    PAYLOAD      type-specific (see below)
    last-2 CRC16        CRC-16/CCITT-FALSE over the WHOLE frame from byte 0
                        (the 0xAA 0x55 header) through the last payload byte,
                        NOT including the CRC trailer. The ESP32 firmware
                        computes crc16(packet, packetLength - 2).

MOTOR_COMMAND payload (11 bytes): M1..M5 int16 * MOTOR_SCALE (1000),
normalized -1.0..+1.0 <-> -1000..+1000, then a FLAGS byte (must be <= 0x03;
the firmware otherwise ACKs with status 4).

ESTOP payload: empty. The ESP32 latches E-STOP; motor commands are ignored
(ACK status 7) until the ESP32 is reset, then a zeroed re-arm handshake is
required (see Esp32ThrusterProvider).

ACK payload (declared length 3): ACK_STATUS(1) followed by ECHO_SEQ(2).
ACK_STATUS: 0 success, 1 CRC error, 2 invalid payload length, 3 invalid motor
value, 4 invalid flags, 5 old sequence, 6 unknown packet type, 7 estop active.

STATUS payload (14 bytes): failsafe(1) estop(1) M1..M5 int16(10).
"""

import struct
import time

try:  # optional dependency — degrades gracefully when pyserial is absent
    import serial
except ImportError:  # pragma: no cover - environment-specific
    serial = None

HEADER = b"\xaa\x55"
PROTOCOL_VERSION = 0x01

# Message types (both directions)
MSG_MOTOR_COMMAND = 0x01  # Pi -> ESP32: 5 motor setpoints (M1..M5) + flags
MSG_ESTOP = 0x02          # Pi -> ESP32: emergency stop (latches on the ESP32)
MSG_ACK = 0x81            # ESP32 -> Pi: acknowledgement (echoes sequence + status)
MSG_STATUS = 0x82         # ESP32 -> Pi: periodic status (failsafe/estop/M1..M5)

# ACK status codes (ESP32 firmware)
ACK_OK = 0
ACK_CRC = 1
ACK_BAD_LENGTH = 2
ACK_BAD_MOTOR = 3
ACK_BAD_FLAGS = 4
ACK_OLD_SEQ = 5
ACK_UNKNOWN_TYPE = 6
ACK_ESTOP = 7
ACK_NAMES = {
    ACK_OK: "OK",
    ACK_CRC: "CRC error",
    ACK_BAD_LENGTH: "invalid payload length",
    ACK_BAD_MOTOR: "invalid motor value",
    ACK_BAD_FLAGS: "invalid flags",
    ACK_OLD_SEQ: "old sequence",
    ACK_UNKNOWN_TYPE: "unknown packet type",
    ACK_ESTOP: "estop active",
}

# Motor int16 scale: normalized [-1.0, +1.0] <-> [-1000, +1000].
MOTOR_SCALE = 1000

# Maximum frame size (ESP32 firmware MAX_PACKET_SIZE = 64).
MAX_FRAME_SIZE = 64

_MOTOR_COMMAND_PAYLOAD = 5 * 2 + 1  # 5 x int16 + FLAGS byte == 11
_MIN_HEAD = 7  # header(2) + version + type + length + sequence(2)
_CRC_SIZE = 2


class IncompleteFrame(Exception):
    """Not enough bytes are buffered to finish the current frame yet."""


class Packet:
    """Decoded frame."""

    __slots__ = ("version", "mtype", "length", "sequence", "payload", "crc")

    def __init__(self, version, mtype, length, sequence, payload, crc):
        self.version = version
        self.mtype = mtype
        self.length = length
        self.sequence = sequence
        self.payload = payload
        self.crc = crc

    def __repr__(self):
        return (
            "Packet(version=%d mtype=0x%02x seq=%d len=%d)"
            % (self.version, self.mtype, self.sequence, self.length)
        )


class UartTransport:
    """Low-level serial packet transport.

    Polled (no background threads here): call :meth:`pump` each tick to absorb
    incoming bytes and enqueue parsed packets, then :meth:`read_packets` to
    drain them. Frames are written through :meth:`send`. All serial I/O is
    non-blocking (short reads via ``in_waiting``, write timeouts bounded).
    """

    def __init__(self, config=None, logger=None, serial_factory=None):
        self.config = config or {}
        self.logger = logger
        self.port = self.config.get("serial_port")
        self.baudrate = int(self.config.get("baudrate", 460800))
        self.timeout_ms = float(self.config.get("timeout_ms", 100))
        self.read_timeout = self.timeout_ms / 1000.0
        self.write_timeout = self.timeout_ms / 1000.0
        self.reconnect_interval_s = float(
            self.config.get("reconnect_interval_s", 1.0)
        )
        self.version = int(self.config.get("version", PROTOCOL_VERSION))
        self._serial_factory = serial_factory or serial.Serial
        self._ser = None
        self._rx = bytearray()
        self._pending = []
        self._last_open_attempt = 0.0
        self._connected_logged = False
        self.stats = dict(
            tx_frames=0,
            rx_frames=0,
            crc_errors=0,
            invalid_packets=0,
            resyncs=0,
            reconnect_attempts=0,
            open_failures=0,
            dropped_frames=0,
            last_error=None,
            last_rx_at=None,
        )

    # -- public helpers ---------------------------------------------------

    def is_open(self):
        return self._ser is not None and self._ser.is_open

    def open(self):
        """Open the serial port once; never raises, records failures instead."""
        now = time.monotonic()
        if self.is_open():
            return True
        if now - self._last_open_attempt < self.reconnect_interval_s:
            return False
        self._last_open_attempt = now
        self._close_ser()
        if serial is None:
            self.stats["last_error"] = "pyserial is not installed"
            self.stats["open_failures"] += 1
            return False
        if not self.port:
            self.stats["last_error"] = (
                "no serial port configured (thrusters.esp32.serial_port)"
            )
            self.stats["open_failures"] += 1
            return False
        try:
            self._ser = self._serial_factory(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.read_timeout,
                write_timeout=self.write_timeout,
            )
            self.stats["last_error"] = None
            if self.logger is not None and not self._connected_logged:
                self._connected_logged = True
                self.logger.info(
                    "ESP32 connected: %s @ %d baud", self.port, self.baudrate
                )
            return True
        except (OSError, ValueError) as exc:  # missing device, bad params, ...
            self.stats["last_error"] = str(exc)
            self.stats["open_failures"] += 1
            self._ser = None
            if self.logger is not None:
                self.logger.warning("ESP32 UART open failed: %s", exc)
            return False

    def close(self):
        """Close the port safely (idempotent, never raises)."""
        self._close_ser()
        self._rx = bytearray()
        if self.logger is not None and self._connected_logged:
            self._connected_logged = False
            self.logger.info("ESP32 disconnected")

    def _close_ser(self):
        if self._ser is not None:
            try:
                if self.is_open():
                    self._ser.close()
            except (OSError, ValueError):  # pragma: no cover - rare
                pass
            self._ser = None

    def send(self, frame):
        """Write one raw frame. Returns True on success; reconnects on failure."""
        if not self.is_open() and not self.open():
            self.stats["dropped_frames"] += 1
            return False
        try:
            count = self._ser.write(frame)
            if count != len(frame):
                raise OSError("short write: %d/%d bytes" % (count, len(frame)))
            self._flush()
            self.stats["tx_frames"] += 1
            return True
        except Exception as exc:  # SerialException / OSError / Timeout ...
            self.stats["last_error"] = str(exc)
            self.stats["dropped_frames"] += 1
            self.stats["reconnect_attempts"] += 1
            self._close_ser()
            if self.logger is not None:
                self.logger.warning("ESP32 UART write failed: %s (reconnecting)", exc)
            return False

    def _flush(self):
        try:
            self._ser.flush()
        except (OSError, ValueError, AttributeError):  # pragma: no cover
            pass

    def pump(self):
        """Read whatever bytes are available and parse any complete frames."""
        if not self.is_open():
            return
        try:
            waiting = self._ser.in_waiting
        except (OSError, ValueError):  # pragma: no cover
            self._close_ser()
            return
        if waiting:
            try:
                chunk = self._ser.read(min(waiting, 1024))
            except (OSError, ValueError):  # pragma: no cover
                self._close_ser()
                return
            if chunk:
                self._rx.extend(chunk)
        packets = self._parse_available()
        if packets:
            self._pending.extend(packets)
            self.stats["last_rx_at"] = time.time()

    def read_packets(self):
        packets = list(self._pending)
        self._pending = []
        return packets

    def reconnect(self):
        """Force a reconnect attempt (close stale handle, reopen)."""
        self.stats["reconnect_attempts"] += 1
        self._close_ser()
        return self.open()

    # -- RX parsing -------------------------------------------------------

    def _parse_available(self):
        """Parse as many frames as possible from the RX buffer, resync as needed."""
        packets = []
        data = self._rx
        while data:
            if len(data) < _MIN_HEAD or data[: len(HEADER)] != HEADER:
                idx = bytes(data).find(HEADER)
                if idx < 0:
                    self._rx = bytearray()
                    break
                del data[:idx]
                self.stats["resyncs"] += 1
                continue
            try:
                packet, consumed = parse_frame(bytes(data), version=self.version)
            except IncompleteFrame:
                break
            except ValueError as exc:
                if str(exc) == "crc":
                    self.stats["crc_errors"] += 1
                else:
                    self.stats["invalid_packets"] += 1
                del data[0]
                self.stats["resyncs"] += 1
                continue
            del data[:consumed]
            packets.append(packet)
        self._rx = bytearray(data)
        if packets:
            self.stats["rx_frames"] += len(packets)
        return packets


# -- packet building / parsing ---------------------------------------------


def crc16_ccitt(data):
    """CRC-16/CCITT-FALSE over ``bytes`` (same algorithm as the ESP32)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8) & 0xFFFF
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def clamp(value, lo=-1.0, hi=1.0):
    return max(lo, min(hi, float(value)))


def build_frame(sequence, mtype, payload, version=PROTOCOL_VERSION):
    """Frame layout (little-endian):

        HEADER | VERSION | TYPE | LENGTH | SEQ(16) | PAYLOAD | CRC16

    CRC covers the whole frame **including the 0xAA 0x55 header** and up to
    the last payload byte (the ESP32 firmware computes crc over
    ``packet[0 : len-2]``). Sequence is a 16-bit counter.
    """
    length = len(payload)
    head = struct.pack("<BBBH", version, mtype, length, sequence & 0xFFFF)
    frame = HEADER + head + payload
    crc = crc16_ccitt(frame)
    return frame + struct.pack("<H", crc)


def motor_values_to_int16(motors):
    """Pack five normalized [-1,1] floats into a 10-byte int16 payload (M1..M5)."""
    values = []
    for value in motors:
        value = clamp(value, -1.0, 1.0)
        values.append(int(round(value * MOTOR_SCALE)))
    return struct.pack("<5h", *values)


def build_motor_command(sequence, motors, flags=0, version=PROTOCOL_VERSION):
    """Build the Pi -> ESP32 MOTOR_COMMAND frame.

    ``motors`` must be the five already-mixed, clamped setpoints in
    M1..M5 (ThrusterId) order. Values are re-clamped here as a hard guarantee
    before transmission. Payload = 5 x int16 + FLAGS byte (11 bytes).
    """
    if flags & 0xFC:
        raise ValueError("reserved FLAGS bits set (firmware ACKs status 4)")
    payload = motor_values_to_int16(motors) + struct.pack("<B", flags & 0xFF)
    return build_frame(sequence, MSG_MOTOR_COMMAND, payload, version)


def build_estop(sequence, version=PROTOCOL_VERSION):
    """Build the Pi -> ESP32 ESTOP frame (empty payload; latches E-STOP)."""
    return build_frame(sequence, MSG_ESTOP, b"", version)


def parse_frame(data, version=PROTOCOL_VERSION):
    """Parse exactly one frame starting at ``data[0]``.

    Returns ``(Packet, consumed)``. Raises ``IncompleteFrame`` when more bytes
    are needed, and ``ValueError`` when the frame is present but invalid
    (wrong version, bad length, or CRC mismatch).
    """
    if len(data) < _MIN_HEAD or data[: len(HEADER)] != HEADER:
        raise ValueError("missing header")
    length = data[4]
    mtype = data[3]
    # ESP32 firmware quirk: sendStatus() counts the 2-byte sequence INSIDE
    # LENGTH (LENGTH=14 for seq + failsafe + estop + M1..M5), so a STATUS
    # frame is 5 + LENGTH + 2 bytes on the wire while every other frame type
    # is 7 + LENGTH + 2 (sequence lives in the fixed 2-byte header slot).
    head = 5 if mtype == MSG_STATUS else _MIN_HEAD
    total = head + length + _CRC_SIZE
    if total > MAX_FRAME_SIZE:
        raise ValueError("invalid length")
    if len(data) < total:
        raise IncompleteFrame()
    ver = data[2]
    sequence = struct.unpack_from("<H", data, 5)[0]
    payload = data[head : head + length]
    crc = struct.unpack_from("<H", data, head + length)[0]
    if ver != version:
        raise ValueError("version mismatch")
    if crc16_ccitt(data[:total - _CRC_SIZE]) != crc:
        raise ValueError("crc")
    packet = Packet(
        version=ver,
        mtype=mtype,
        length=length,
        sequence=sequence,
        payload=bytes(payload),
        crc=crc,
    )
    return packet, total


# -- payload decoders -------------------------------------------------------


def decode_motor_command(payload):
    """Return ``(motors, flags)`` from a MOTOR_COMMAND payload (11 bytes)."""
    if len(payload) < _MOTOR_COMMAND_PAYLOAD:
        raise ValueError("bad motor payload")
    motors = struct.unpack("<5h", payload[:10])
    return motors, payload[10]


def decode_ack(payload):
    """Return ``(ack_status, echo_sequence)`` from an ACK payload.

    The ESP32 firmware declares LENGTH=3 but writes 5 payload bytes
    (status + 2-byte echo). The CRC covers bytes 0..9 either way, so framing
    is consistent; only the first 3 bytes are meaningful per the protocol.
    """
    if len(payload) < 3:
        raise ValueError("bad ack payload")
    status = payload[0]
    echo = payload[1] | (payload[2] << 8)
    return status, echo


def decode_status(payload):
    """Return ``dict(failsafe, estop, motors)`` from a STATUS payload (14 bytes).

    The firmware's sendStatus() declares LENGTH=14 and writes
    sequence(2) failsafe(1) estop(1) M1..M5 int16(10), with that sequence also
    occupying the header sequence slot (the whole frame is 21 bytes on the wire).
    """
    if len(payload) < 14:
        raise ValueError("bad status payload")
    return {
        "sequence": struct.unpack_from("<H", payload, 0)[0],
        "failsafe": bool(payload[2]),
        "estop": bool(payload[3]),
        "motors": tuple(struct.unpack_from("<5h", payload, 4)),
    }