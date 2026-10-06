"""Receive-only Piper CAN snapshots; SI units and per-frame freshness checks."""

import math
import struct
import threading
import time

import can

KEYS = tuple(f"joint_{i}.pos" for i in range(1, 7)) + ("gripper.pos",)
JOINT_FACTOR = 180_000 / math.pi
GRIPPER_FACTOR = 1_000_000
FEEDBACK_IDS = (0x2A5, 0x2A6, 0x2A7, 0x2A8)
CONTROL_IDS = (0x155, 0x156, 0x157, 0x159)


class TargetFramesUnavailable(ConnectionError):
    """Leader targets are missing or stale; distinct from a CAN transport failure."""


class PassiveReader:
    """Never transmits or changes arm roles, torque, calibration, or CAN settings."""

    def __init__(self, channel, source="feedback", timeout_s=0.5):
        if source not in ("feedback", "control"):
            raise ValueError("source must be feedback or control")
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be positive and finite")
        self.channel = channel
        self.ids = FEEDBACK_IDS if source == "feedback" else CONTROL_IDS
        self.timeout_s = timeout_s
        self.frames = {}
        self.receive_delays_s = {}
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.bus = None
        self.thread = None
        self.error = None

    @property
    def is_connected(self):
        return self.bus is not None and self.error is None and self.thread.is_alive()

    def connect(self):
        if self.bus is not None:
            raise ConnectionError("Reader is already connected")
        self.frames.clear()
        self.receive_delays_s.clear()
        self.error = None
        self.stop.clear()
        self.bus = can.Bus(interface="socketcan", channel=self.channel, receive_own_messages=False)
        self.thread = threading.Thread(target=self._receive, daemon=True)
        self.thread.start()

    def _receive(self):
        try:
            while not self.stop.is_set():
                msg = self.bus.recv(timeout=0.05)
                if msg is not None:
                    self.ingest(msg)
        except Exception as exc:  # noqa: BLE001 - propagate reader thread failure to the control thread.
            self.error = exc

    def ingest(self, msg):
        if msg.is_error_frame:
            self.error = ConnectionError(f"CAN error frame on {self.channel}")
            return
        if msg.is_extended_id or msg.is_remote_frame or len(msg.data) != 8:
            return
        if msg.arbitration_id in (*self.ids, 0x2A1, 0x151):
            # SocketCAN timestamps receipt in the kernel. Timestamping only here
            # would make an old queued packet look fresh after a reader stall.
            now = time.monotonic()
            delay = max(0.0, time.time() - msg.timestamp) if msg.timestamp > 0 else 0.0
            with self.lock:
                self.frames[msg.arbitration_id] = (bytes(msg.data), now - delay)
                self.receive_delays_s[msg.arbitration_id] = delay

    def timing(self):
        """Separate frame age from kernel-to-reader delivery delay."""
        with self.lock:
            now = time.monotonic()
            return {
                "max_frame_age_ms": round(1000 * max(
                    (now - self.frames[i][1] for i in self.ids if i in self.frames), default=0.0
                ), 2),
                "max_receive_delay_ms": round(1000 * max(
                    (self.receive_delays_s.get(i, 0.0) for i in self.ids), default=0.0
                ), 2),
            }

    def positions(self):
        if self.error is not None:
            raise ConnectionError(f"CAN reader failed on {self.channel}") from self.error
        now = time.monotonic()
        with self.lock:
            frames = dict(self.frames)
        missing = [hex(i) for i in self.ids if i not in frames]
        if missing:
            raise TargetFramesUnavailable(f"{self.channel}: missing frames {', '.join(missing)}")
        # Some teaching firmware stops all transmissions when stationary.
        # A recent 0x151 can retain targets, but silence is not a live connection.
        heartbeat_live = (
            self.ids == CONTROL_IDS and 0x151 in frames and now - frames[0x151][1] <= self.timeout_s
        )
        stale = [] if heartbeat_live else [hex(i) for i in self.ids if now - frames[i][1] > self.timeout_s]
        if stale:
            raise TargetFramesUnavailable(f"{self.channel}: stale frames {', '.join(stale)}")
        raw = []
        for i in self.ids[:3]:
            raw.extend(struct.unpack(">ii", frames[i][0]))
        values = [v / JOINT_FACTOR for v in raw]
        values.append(struct.unpack(">i", frames[self.ids[3]][0][:4])[0] / GRIPPER_FACTOR)
        return dict(zip(KEYS, values, strict=True))

    def wait_ready(self, timeout_s):
        deadline = time.monotonic() + timeout_s
        while True:
            try:
                return self.positions()
            except ConnectionError:
                if self.error is not None or time.monotonic() >= deadline:
                    raise
                time.sleep(0.02)

    def check_follower_status(self):
        with self.lock:
            status = self.frames.get(0x2A1)
        if status is None or time.monotonic() - status[1] > self.timeout_s:
            raise ConnectionError(f"{self.channel}: missing or stale arm status")
        data = status[0]
        if data[0] != 1 or data[1] != 0 or data[2] != 1 or int.from_bytes(data[6:8], "big"):
            raise ConnectionError(f"{self.channel}: follower is not in healthy CAN joint mode: {data.hex()}")

    def disconnect(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=1)
        if self.bus is not None:
            self.bus.shutdown()
        self.bus = None
