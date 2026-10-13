"""Receive-only per-motor driver status. Never enables or moves the arm."""

import argparse
import json
import time

import can

FLAGS = (
    "voltage_too_low", "motor_overheating", "driver_overcurrent",
    "driver_overheating", "collision_status", "driver_error_status",
    "driver_enable_status", "stall_status",
)


def decode(data):
    if len(data) != 8:
        raise ValueError("Expected an eight-byte low-speed motor feedback frame")
    return {
        "voltage_v": int.from_bytes(data[:2], "big") / 10,
        "driver_temperature_c": int.from_bytes(data[2:4], "big", signed=True),
        "motor_temperature_c": int.from_bytes(data[4:5], "big", signed=True),
        "current_a": int.from_bytes(data[6:8], "big") / 1000,
        **{name: bool(data[5] & (1 << bit)) for bit, name in enumerate(FLAGS)},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", default="piper_follower")
    args = parser.parse_args()
    frames = {}
    with can.Bus(interface="socketcan", channel=args.channel) as bus:
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            msg = bus.recv(timeout=0.1)
            if msg is None:
                continue
            if msg.is_error_frame:
                raise ConnectionError("CAN error frame received")
            if not msg.is_extended_id and not msg.is_remote_frame and 0x261 <= msg.arbitration_id <= 0x266:
                frames[msg.arbitration_id] = (bytes(msg.data), msg.timestamp)
    now = time.time()
    report = {}
    for index in range(6):
        frame = frames.get(0x261 + index)
        report[f"J{index + 1}"] = (
            {**decode(frame[0]), "age_s": round(max(0, now - frame[1]), 3)}
            if frame else {"error": "missing motor feedback"}
        )
    print(json.dumps({"motors": report, "motor_commands_sent": False}, indent=2))


if __name__ == "__main__":
    main()
