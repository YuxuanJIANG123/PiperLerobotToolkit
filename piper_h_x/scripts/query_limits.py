"""Query stored follower limits. Sends only 0x472 queries, never motion or settings."""

import json
import math
import struct
import time
from pathlib import Path

import can


def main():
    limits = {}
    with can.Bus(
        interface="socketcan",
        channel="piper_follower",
        can_filters=[{"can_id": 0x473, "can_mask": 0x7FF, "extended": False}],
    ) as bus:
        for joint in range(1, 7):
            bus.send(
                can.Message(
                    arbitration_id=0x472, is_extended_id=False, data=bytes([joint, 1, 0, 0, 0, 0, 0, 0])
                )
            )
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                msg = bus.recv(timeout=0.1)
                if msg is None or msg.arbitration_id != 0x473 or len(msg.data) != 8:
                    continue
                index, maximum, minimum, speed = struct.unpack(">BhhH", bytes(msg.data[:7]))
                if index == joint:
                    limits[f"joint_{joint}"] = {
                        "min_deg": minimum / 10,
                        "max_deg": maximum / 10,
                        "min_rad": math.radians(minimum / 10),
                        "max_rad": math.radians(maximum / 10),
                        "max_speed_rad_s": speed / 1000,
                        "raw_hex": bytes(msg.data).hex(),
                    }
                    break
    report = {
        "channel": "piper_follower",
        "stored_limits": limits,
        "missing_joints": [i for i in range(1, 7) if f"joint_{i}" not in limits],
    }
    output = Path("outputs/read_only_check/firmware_limits.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(output.read_text())
    if report["missing_joints"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
