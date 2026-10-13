"""Query firmware and capture CAN traffic; sends only firmware-read requests."""

import collections
import json
import select
import time
from contextlib import ExitStack
from pathlib import Path

import can


def main():
    report = {}
    with ExitStack() as stack:
        buses = {name: stack.enter_context(can.Bus(interface="socketcan", channel=name))
                 for name in ("piper_leader", "piper_follower")}
        counts = {name: collections.Counter() for name in buses}
        last = {name: {} for name in buses}
        firmware = {name: bytearray() for name in buses}
        for bus in buses.values():
            bus.send(can.Message(arbitration_id=0x4AF, is_extended_id=False,
                                 data=bytes([1, 0, 0, 0, 0, 0, 0, 0])), timeout=0.2)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            ready, _, _ = select.select(list(buses.values()), [], [], 0.1)
            for bus in ready:
                name = next(name for name, candidate in buses.items() if candidate is bus)
                msg = bus.recv(timeout=0)
                if msg is None:
                    continue
                counts[name][hex(msg.arbitration_id)] += 1
                last[name][hex(msg.arbitration_id)] = bytes(msg.data).hex()
                if msg.arbitration_id == 0x4AF:
                    firmware[name].extend(msg.data)
        for name in buses:
            report[name] = {"counts": counts[name], "latest_hex": last[name],
                            "firmware_ascii": firmware[name].decode("ascii", errors="replace")}
    output = Path("outputs/read_only_check/arm_audit.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(output.read_text())


if __name__ == "__main__":
    main()
