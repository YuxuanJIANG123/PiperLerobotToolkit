"""Passive CAN inspection; safe to run before motor enable."""

import argparse
import json
import time

from .protocol import PassiveReader


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", default="piper_follower")
    parser.add_argument("--source", choices=("feedback", "control"), default="feedback")
    parser.add_argument("--seconds", type=float, default=3)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    reader = PassiveReader(args.channel, args.source, timeout_s=3.0 if args.source == "control" else 0.5)
    reader.connect()
    try:
        time.sleep(args.seconds)
        with reader.lock:
            frames = dict(reader.frames)
        result = {
            "channel": args.channel,
            "source": args.source,
            "frame_ages_s": {hex(k): round(time.monotonic() - v[1], 3) for k, v in frames.items()},
        }
        result["timing"] = reader.timing()
        if 0x2A1 in frames:
            data = frames[0x2A1][0]
            result["arm_status"] = {
                "raw_hex": data.hex(),
                "ctrl_mode": data[0],
                "arm_status": data[1],
                "mode_feed": data[2],
                "teach_status": data[3],
                "motion_status": data[4],
                "trajectory_num": data[5],
                "err_code": int.from_bytes(data[6:8], "big"),
            }
        try:
            result["positions_rad_m"] = reader.positions()
        except ConnectionError as exc:
            result["error"] = str(exc)
        print(json.dumps(result, indent=2))
        if "error" in result:
            raise SystemExit(1)
    finally:
        reader.disconnect()


if __name__ == "__main__":
    main()
