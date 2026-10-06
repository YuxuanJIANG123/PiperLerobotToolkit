"""Dry-run H->X mapping: reads CAN only, or uses synthetic poses. Never opens a command SDK."""

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import yaml

from lerobot_robot_piper_x.end_effector import EndEffectorConfig, RelativeEndEffector
from lerobot_robot_piper_x.protocol import KEYS, PassiveReader, TargetFramesUnavailable


def preview(config_path, seconds, synthetic=False):
    settings = yaml.safe_load(Path(config_path).read_text())
    robot, teleop = settings["robot"], settings["teleop"]
    mapper = RelativeEndEffector(
        EndEffectorConfig(**teleop.get("eef", {})),
        robot["joint_limits_rad"],
        robot.get("gripper_max_m", 0.07),
    )
    leader = PassiveReader(
        teleop["can_name"], teleop.get("source", "control"), teleop.get("feedback_timeout_s", 3.0)
    )
    follower = PassiveReader(robot["can_name"], "feedback", robot.get("feedback_timeout_s", 0.5))
    initial = dict(zip(KEYS, [0.1, 0.6, -1.0, 0.2, 0.4, 0.2, 0.02], strict=True))
    current = dict(initial)
    paused_since = None
    next_log = 0
    samples, tracking, paused = 0, 0, 0
    try:
        if not synthetic:
            leader.connect()
            follower.connect()
            print(
                "Receive-only preview: move the supported leader and gripper to provide initial frames.",
                flush=True,
            )
            follower.wait_ready(5)
            leader.wait_ready(teleop.get("connect_timeout_s", 10))
        began = time.monotonic()
        while (elapsed := time.monotonic() - began) < seconds:
            if synthetic:
                action = dict(initial)
                action[KEYS[1]] += 0.04 * math.sin(elapsed)
                action[KEYS[2]] += 0.02 * math.sin(elapsed)
            else:
                current = follower.positions()  # Follower faults never become a leader pause.
                try:
                    action = leader.positions()
                except TargetFramesUnavailable:
                    if paused_since is None:
                        paused_since = time.monotonic()
                        mapper.reset()
                        print(
                            json.dumps({"status": "leader_silent", "motor_commands_sent": False}), flush=True
                        )
                    time.sleep(1 / 30)
                    continue
                if paused_since is not None:
                    with leader.lock:
                        refreshed = all(leader.frames[i][1] > paused_since for i in leader.ids)
                    if not refreshed:
                        time.sleep(1 / 30)
                        continue
                    paused_since = None
            target = mapper.map(action, current)
            samples += 1
            if mapper.diagnostics["status"] == "tracking":
                tracking += 1
            elif mapper.diagnostics["status"] != "anchored":
                paused += 1
            if elapsed >= next_log:
                print(
                    json.dumps(
                        {**mapper.diagnostics, "target_joints_rad_m": target, "motor_commands_sent": False}
                    ),
                    flush=True,
                )
                next_log += 0.5
            if synthetic:
                # Idealized feedback for numerical demonstration, not a dynamics model.
                current = target
            time.sleep(1 / 30)
        report = {
            "synthetic": synthetic,
            "samples": samples,
            "tracking_samples": tracking,
            "paused_samples": paused,
            "motor_commands_sent": False,
        }
        print(json.dumps(report), flush=True)
        return report
    finally:
        leader.disconnect()
        follower.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/teleoperate_eef.yaml")
    parser.add_argument("--seconds", type=float, default=15)
    parser.add_argument("--synthetic", action="store_true")
    args = parser.parse_args()
    if not np.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be positive and finite")
    try:
        preview(args.config, args.seconds, args.synthetic)
    except KeyboardInterrupt:
        print("\nPreview stopped. No motor commands were sent.", flush=True)


if __name__ == "__main__":
    main()
