"""Explicit standby -> CAN transition for a supported follower at physical zero.

Run with --execute --prepared only after stopping teleoperation and turning off
the teaching light. No reset, zeroing, role changes, or motor-enable commands.
"""

import argparse
import json
import struct
import time
from pathlib import Path
from types import SimpleNamespace

import can
import yaml

from lerobot_robot_piper_x.protocol import JOINT_FACTOR, KEYS, PassiveReader
from lerobot_robot_piper_x.safety import validate_initial_pose


def status(reader):
    with reader.lock:
        frame = reader.frames.get(0x2A1)
    if frame is None or time.monotonic() - frame[1] > reader.timeout_s:
        raise ConnectionError("Missing or stale follower status")
    data, stamp = frame
    if data[1] or int.from_bytes(data[6:8], "big"):
        raise ConnectionError(f"Follower fault: {data.hex()}")
    if data[3] not in (0, 2):
        raise ConnectionError(f"Teaching/playback not stopped: {data.hex()}")
    if data[0] not in (0, 1, 2):
        raise ConnectionError(f"Unexpected control mode: {data.hex()}")
    return data, stamp


def switch(reader, config, timeout=5.0):
    def checked_positions():
        positions = reader.positions()
        validate_initial_pose(positions, config)
        return positions

    def send(can_id, data):
        reader.bus.send(can.Message(arbitration_id=can_id, is_extended_id=False, data=data), timeout=0.2)

    checked_positions()
    initial, _ = status(reader)
    print(f"Initial status: {initial.hex()}", flush=True)
    if initial[0] == 1 and initial[2] == 1:
        print("Already in healthy CAN joint mode; no commands sent.", flush=True)
        return
    for mode in (0, 1):
        checked_positions()
        status(reader)
        if mode == 1:
            # Replace any retained joint target with the measured current pose.
            positions = checked_positions()
            raw = [round(positions[k] * JOINT_FACTOR) for k in KEYS[:6]]
            for i, can_id in enumerate((0x155, 0x156, 0x157)):
                send(can_id, struct.pack(">ii", *raw[2 * i : 2 * i + 2]))
        started = time.monotonic()
        deadline = started + timeout
        print(f"Requesting ctrl_mode={mode}", flush=True)
        while True:
            checked_positions()
            data, stamp = status(reader)
            if stamp > started and data[0] == mode and (mode == 0 or data[2] == 1):
                print(f"Confirmed status: {data.hex()}", flush=True)
                break
            if time.monotonic() >= deadline:
                raise ConnectionError(
                    f"Mode {mode} not acknowledged; latest status={data.hex()}. "
                    "Stopped without reset or motor enable."
                )
            # 0x151: ctrl mode, MOVE J, 10% speed, position/velocity mode.
            send(0x151, bytes([mode, 1, 10, 0, 0, 0, 0, 0]))
            time.sleep(0.05)
    reader.check_follower_status()
    print("CAN joint mode confirmed. No motor-enable command sent.", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--prepared", action="store_true",
                        help="Confirm supported physical zero pose, teaching light off, teleop stopped")
    args = parser.parse_args()
    if args.execute and not args.prepared:
        parser.error("--execute requires --prepared after physically preparing the follower")
    settings = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "configs/teleoperate.yaml").read_text()
    )["robot"]
    config = SimpleNamespace(joint_limits_rad=settings["joint_limits_rad"],
                             gripper_max_m=settings.get("gripper_max_m", 0.07))
    reader = PassiveReader("piper_follower")
    try:
        reader.connect()
        positions = reader.wait_ready(5)
        data, _ = status(reader)
        print(json.dumps({"status_hex": data.hex(), "positions_rad_m": positions}), flush=True)
        if args.execute:
            switch(reader, config)
    finally:
        reader.disconnect()


if __name__ == "__main__":
    main()
