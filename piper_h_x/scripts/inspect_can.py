"""Read raw CAN frames without sending any messages (Linux standard library only)."""

import collections
import json
import select
import socket
import struct
import time

sockets = {}
try:
    for channel in ("piper_leader", "piper_follower"):
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((channel,))
        sockets[sock] = channel
    counts = {channel: collections.Counter() for channel in sockets.values()}
    latest = {channel: {} for channel in sockets.values()}
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        ready, _, _ = select.select(list(sockets), [], [], 0.1)
        for sock in ready:
            frame = sock.recv(16)
            can_id, length, data = struct.unpack("=IB3x8s", frame)
            channel = sockets[sock]
            counts[channel][hex(can_id)] += 1
            if can_id in (0x151, 0x155, 0x156, 0x157, 0x159, 0x2A1, 0x2A5, 0x2A6, 0x2A7, 0x2A8):
                latest[channel][hex(can_id)] = data[:length].hex()
    print(json.dumps({"counts": counts, "latest_hex": latest}, indent=2))
finally:
    for sock in sockets:
        sock.close()
