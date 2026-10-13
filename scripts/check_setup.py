"""Read follower state and one frame per configured camera; never enable motors."""

import json
from dataclasses import replace
from pathlib import Path

import cv2
import draccus
from lerobot.scripts.lerobot_record import RecordConfig
from lerobot.utils.import_utils import register_third_party_plugins

from lerobot_robot_piper_x import PiperX
from lerobot_robot_piper_x.protocol import KEYS
from lerobot_robot_piper_x.safety import validate_initial_pose

register_third_party_plugins()
config = draccus.parse(RecordConfig, config_path="configs/record.yaml", args=[])
output = Path("outputs/read_only_check")
output.mkdir(parents=True, exist_ok=True)
robot = PiperX(replace(config.robot, enable_motors=False, calibration_dir=output / "calibration"))
try:
    robot.connect()
    observation = robot.get_observation()
    report = {"positions_rad_m": {key: observation[key] for key in KEYS}, "cameras": {}}
    for name in robot.cameras:
        frame = observation[name]
        path = output / f"{name}.png"
        if not cv2.imwrite(str(path), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)):
            raise OSError(f"Could not save {path}")
        report["cameras"][name] = {"shape": list(frame.shape), "preview": str(path)}
    try:
        positions = report["positions_rad_m"]
        validate_initial_pose(positions, robot.config)
        report["initial_pose_within_configured_envelope"] = True
    except ValueError as exc:
        report["initial_pose_within_configured_envelope"] = False
        report["limit_error"] = str(exc)
    assert robot.sdk is None, "Read-only check must not instantiate the command SDK"
    report["motor_commands_sent"] = False
    text = json.dumps(report, indent=2)
    (output / "report.json").write_text(text + "\n")
    print(text)
finally:
    robot.disconnect()
