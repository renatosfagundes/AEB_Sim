"""
CAN Bus Communication Model for AEB System
============================================
Models CAN 2.0B bus at 500 kbps with periodic message scheduling,
bus load computation, and signal packing per ISO 11898-1:2015.

Message IDs and signal definitions match aeb_system.dbc.

Ref: ISO 11898-1:2015, CSS Electronics CAN bus guides
"""
import struct
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from config import CANConfig


# ============================================================
# CAN Message dataclass
# ============================================================
@dataclass
class CANMessage:
    """Single CAN 2.0B frame."""
    msg_id: int = 0
    dlc: int = 0
    period_ms: int = 0
    name: str = ""
    data: bytes = b""
    timestamp: float = 0.0
    tx_count: int = 0


# ============================================================
# CAN Bus Model
# ============================================================
class CANBus:
    """
    CAN 2.0B bus model at 500 kbps.

    Handles periodic message registration, transmission scheduling,
    bus load calculation, and message logging.
    """

    # CAN 2.0B standard frame overhead bits (SOF + arbitration + control + CRC + ACK + EOF + IFS)
    FRAME_OVERHEAD_BITS = 47

    def __init__(self, config: CANConfig):
        self.config = config
        self.baud_rate = config.baud_rate
        self.stuff_bit_factor = config.stuff_bit_factor
        self.max_bus_load = config.max_bus_load

        # Registered messages: msg_id -> CANMessage (template)
        self._messages: Dict[int, CANMessage] = {}

        # Scheduling: msg_id -> next_tx_time
        self._schedule: Dict[int, float] = {}

        # Transmission queue: list of (msg_id, data, timestamp)
        self._tx_queue: deque = deque(maxlen=256)

        # Message log: recent transmitted messages
        self._msg_log: deque = deque(maxlen=1000)

        # Statistics
        self._tx_counts: Dict[int, int] = {}
        self._worst_latency: float = 0.0
        self._bus_load: float = 0.0

        # Register messages from config
        for msg_id, dlc, period_ms, name in config.messages:
            self.register_message(msg_id, dlc, period_ms, name)

    def register_message(self, msg_id: int, dlc: int, period_ms: int, name: str) -> None:
        """Register a periodic CAN message definition."""
        msg = CANMessage(
            msg_id=msg_id,
            dlc=dlc,
            period_ms=period_ms,
            name=name,
        )
        self._messages[msg_id] = msg
        self._tx_counts[msg_id] = 0
        # Initialize schedule: first transmission at t=0
        self._schedule[msg_id] = 0.0

    def transmit(self, msg_id: int, data: bytes, t: float) -> None:
        """Queue a message for transmission on the bus."""
        if msg_id not in self._messages:
            raise ValueError(f"Message 0x{msg_id:03X} not registered")
        template = self._messages[msg_id]
        if len(data) > template.dlc:
            raise ValueError(
                f"Data length {len(data)} exceeds DLC {template.dlc} "
                f"for {template.name}"
            )
        self._tx_queue.append((msg_id, data, t))

    def tick(self, t: float, dt: float) -> List[CANMessage]:
        """
        Process message scheduling for the current time step.

        Checks if periodic messages are due and processes the TX queue.
        Returns list of messages transmitted in this tick.
        """
        transmitted: List[CANMessage] = []

        # Check periodic schedule
        for msg_id, next_time in list(self._schedule.items()):
            if t >= next_time:
                template = self._messages[msg_id]
                period_s = template.period_ms / 1000.0
                # Update schedule for next period
                self._schedule[msg_id] = next_time + period_s

        # Process TX queue (priority: lower msg_id = higher priority)
        pending = []
        while self._tx_queue:
            pending.append(self._tx_queue.popleft())

        # Sort by msg_id (CAN arbitration: lower ID wins)
        pending.sort(key=lambda x: x[0])

        for msg_id, data, tx_time in pending:
            template = self._messages[msg_id]
            self._tx_counts[msg_id] = self._tx_counts.get(msg_id, 0) + 1

            msg = CANMessage(
                msg_id=msg_id,
                dlc=template.dlc,
                period_ms=template.period_ms,
                name=template.name,
                data=data,
                timestamp=t,
                tx_count=self._tx_counts[msg_id],
            )
            transmitted.append(msg)
            self._msg_log.append(msg)

            # Track worst-case latency (time from queue to transmit)
            latency = t - tx_time
            if latency > self._worst_latency:
                self._worst_latency = latency

        # Update bus load
        self._bus_load = self._compute_bus_load()

        return transmitted

    def get_bus_load(self) -> float:
        """
        Compute current bus load percentage.

        Formula:
            stuffed_frame_bits = (47 + 8 * DLC) * stuff_bit_factor
            load = sum(stuffed_frame_bits * frequency_hz) / baud_rate
        """
        return self._bus_load

    def _compute_bus_load(self) -> float:
        """Internal bus load computation."""
        total_bits_per_sec = 0.0
        for msg_id, template in self._messages.items():
            frame_bits = (self.FRAME_OVERHEAD_BITS + 8 * template.dlc)
            stuffed_bits = frame_bits * self.stuff_bit_factor
            freq_hz = 1000.0 / template.period_ms
            total_bits_per_sec += stuffed_bits * freq_hz
        return total_bits_per_sec / self.baud_rate

    def get_statistics(self) -> dict:
        """
        Return bus statistics.

        Returns:
            dict with keys:
                bus_load:       current bus load [0..1]
                msg_counts:     dict of msg_id -> tx_count
                worst_latency:  worst-case message latency [s]
        """
        return {
            "bus_load": self._bus_load,
            "msg_counts": dict(self._tx_counts),
            "worst_latency": self._worst_latency,
        }

    def get_message_log(self) -> list:
        """Return list of recently transmitted CANMessage objects."""
        return list(self._msg_log)


# ============================================================
# Signal Packing Functions
# ============================================================

def pack_ego_vehicle(v_ego: float, a_ego: float, yaw_rate: float,
                     steering_angle: float) -> bytes:
    """
    Pack ego vehicle data into 8 bytes (CAN ID 0x100).

    Signals:
        v_ego:          uint16, factor 0.01, offset 0      [m/s]
        a_ego:          int16,  factor 0.001, offset -32    [m/s^2]
        yaw_rate:       int16,  factor 0.01, offset -327.68 [deg/s]
        steering_angle: int16,  factor 0.1, offset -3276.8  [deg]
    """
    v_raw = int(round(v_ego / 0.01))
    v_raw = max(0, min(0xFFFF, v_raw))

    a_raw = int(round((a_ego - (-32.0)) / 0.001))
    a_raw = max(-32768, min(32767, a_raw))

    yr_raw = int(round((yaw_rate - (-327.68)) / 0.01))
    yr_raw = max(-32768, min(32767, yr_raw))

    sa_raw = int(round((steering_angle - (-3276.8)) / 0.1))
    sa_raw = max(-32768, min(32767, sa_raw))

    return struct.pack("<HhHh", v_raw, a_raw, yr_raw, sa_raw)


def pack_radar_target(distance: float, rel_speed: float, ttc: float,
                      confidence: int = 15) -> bytes:
    """
    Pack radar target data into 8 bytes (CAN ID 0x120).

    Signals:
        distance:   uint16, factor 0.01, offset 0       [m]
        rel_speed:  int16,  factor 0.01, offset -327.68  [m/s]
        ttc:        uint16, factor 0.001, offset 0       [s]
        confidence: uint8,  [0-15]
        reserved:   uint8
    """
    d_raw = int(round(distance / 0.01))
    d_raw = max(0, min(0xFFFF, d_raw))

    rs_raw = int(round((rel_speed - (-327.68)) / 0.01))
    rs_raw = max(-32768, min(32767, rs_raw))

    ttc_raw = int(round(ttc / 0.001))
    ttc_raw = max(0, min(0xFFFF, ttc_raw))

    conf = max(0, min(15, confidence))

    return struct.pack("<hhHBB", d_raw, rs_raw, ttc_raw, conf, 0x00)


def pack_brake_cmd(brake_request: bool, brake_pressure: float,
                   mode: int, alive: int, crc: int) -> bytes:
    """
    Pack brake command into 4 bytes (CAN ID 0x080).

    Bit layout:
        bit 0:      brake_request (bool)
        bits 1-15:  brake_pressure, factor 0.1 [bar]
        bits 16-18: mode (0-7)
        bits 24-27: alive (4-bit rolling counter)
        bits 28-31: crc (4-bit CRC)
    """
    # Byte 0-1: brake_request (bit 0) + brake_pressure (bits 1-15)
    bp_raw = int(round(brake_pressure / 0.1))
    bp_raw = max(0, min(0x7FFF, bp_raw))  # 15 bits max

    word0 = (int(brake_request) & 0x01) | ((bp_raw & 0x7FFF) << 1)

    # Byte 2: mode (bits 0-2)
    byte2 = mode & 0x07

    # Byte 3: alive (bits 0-3) + crc (bits 4-7)
    byte3 = (alive & 0x0F) | ((crc & 0x0F) << 4)

    return struct.pack("<HBB", word0, byte2, byte3)


def pack_fsm_state(state: int, alert_level: int, brake_active: bool,
                   ttc_threshold: float) -> bytes:
    """
    Pack FSM state data into 4 bytes (CAN ID 0x200).

    Signals:
        state:          uint8 (0-6: OFF..POST_BRAKE)
        alert_level:    uint8 (0-3: NONE, WARNING, BRAKE_LOW, BRAKE_HIGH)
        brake_active:   uint8 (bool, 0 or 1)
        ttc_threshold:  uint8, factor 0.1, offset 0 [s]
    """
    st = max(0, min(6, state))
    al = max(0, min(3, alert_level))
    ba = 1 if brake_active else 0
    ttc_raw = int(round(ttc_threshold / 0.1))
    ttc_raw = max(0, min(255, ttc_raw))

    return struct.pack("<BBBB", st, al, ba, ttc_raw)


def pack_alert(alert_type: int, alert_active: bool,
               buzzer_cmd: int) -> bytes:
    """
    Pack alert data into 2 bytes (CAN ID 0x300).

    Signals:
        alert_type:   uint8 (0-3: NONE, VISUAL, AUDIBLE, BOTH)
        alert_active: bit 0 of byte 1
        buzzer_cmd:   bits 1-3 of byte 1 (0-7: pattern)
    """
    at = max(0, min(3, alert_type))
    byte1 = (int(alert_active) & 0x01) | ((buzzer_cmd & 0x07) << 1)

    return struct.pack("<BB", at, byte1)
