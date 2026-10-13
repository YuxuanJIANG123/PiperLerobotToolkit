"""Explicit one-degree wrist probe; default is receive-only. Stop other controllers first."""

import argparse
import json
import math
import time
from pathlib import Path

import yaml

from lerobot_robot_piper_x import PiperX, PiperXConfig
from lerobot_robot_piper_x.protocol import KEYS
from lerobot_robot_piper_x.safety import validate_initial_pose


def target_for(initial, joint, direction, config):
    target = dict(initial)
    target[f"joint_{joint}.pos"] += math.radians(direction)
    validate_initial_pose(initial, config)
    validate_initial_pose(target, config)
    return target


def probe(robot, joint, direction):
    initial = robot.reader.positions()
    target = target_for(initial, joint, direction, robot.config)
    key = f"joint_{joint}.pos"
    started = time.monotonic()
    next_log = 0
    while time.monotonic() - started < 3:
        present = robot.reader.positions()
        for k in KEYS[:6]:
            bound = math.radians(1.5 if k == key else 0.5)
            if abs(present[k] - initial[k]) > bound:
                raise RuntimeError(f"Unexpected movement on {k}; probe stopped")
        status = robot.sdk.GetArmLowSpdInfoMsgs()
        if time.time() - status.time_stamp > 0.5:
            raise ConnectionError("Stale motor status; probe stopped")
        for i in range(1, 7):
            flags = getattr(status, f"motor_{i}").foc_status
            if not flags.driver_enable_status or any(getattr(flags, name) for name in (
                "voltage_too_low", "motor_overheating", "driver_overcurrent",
                "driver_overheating", "collision_status", "driver_error_status", "stall_status",
            )):
                raise ConnectionError(f"Motor J{i} disabled or faulted; probe stopped")
        sent = robot.send_action(target)
        elapsed = time.monotonic() - started
        if elapsed >= next_log:
            print(json.dumps({"joint": joint, "elapsed_s": round(elapsed, 2),
                              "measured_delta_deg": math.degrees(present[key] - initial[key]),
                              "command_delta_deg": math.degrees(sent[key] - initial[key])}), flush=True)
            next_log += 0.25
        time.sleep(1 / 30)
    print("Probe finished; retaining last bounded target. No automatic return.", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", type=int, choices=(4, 5), required=True)
    parser.add_argument("--direction", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--step-rad", type=float, choices=(0.003, 0.006, 0.012), default=0.006)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--prepared", action="store_true",
                        help="Other controllers stopped, wrist path clear, physical stop accessible")
    args = parser.parse_args()
    if args.execute and not args.prepared:
        parser.error("--execute requires --prepared")
    settings = yaml.safe_load((Path(__file__).resolve().parents[1] / "configs/teleoperate.yaml").read_text())["robot"]
    settings.pop("type", None)
    settings.update(cameras={}, enable_motors=args.execute, reset_to_zero_on_connect=False,
                    high_follow=False, motion_speed=10, max_joint_step_rad=args.step_rad, debug_motion=False)
    robot = PiperX(PiperXConfig(**settings))
    try:
        robot.connect()
        initial = robot.reader.positions()
        target = target_for(initial, args.joint, args.direction, robot.config)
        print(json.dumps({"initial": initial, "requested": target, "execute": args.execute}), flush=True)
        if args.execute:
            probe(robot, args.joint, args.direction)
    except KeyboardInterrupt:
        print("Probe interrupted; retaining last bounded target.", flush=True)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
