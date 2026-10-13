"""Apply the narrow Piper-X sent-target recording fix to the pinned checkout."""

from pathlib import Path

path = Path(__file__).resolve().parents[1] / "vendor/lerobot/src/lerobot/scripts/lerobot_record.py"
source = path.read_text()
anchor = "            _sent_action = robot.send_action(robot_action_to_send)\n"
addition = """            # Piper-X records the bounded, quantized target actually sent over CAN.
            # Other robots retain upstream action-representation semantics.
            if robot.name == "piper_x":
                action_values = _sent_action
"""
if anchor + addition in source:
    print("Piper-X recording patch already applied")
elif source.count(anchor) == 1 and 'if robot.name == "piper_x":' not in source:
    path.write_text(source.replace(anchor, anchor + addition))
    print("Applied Piper-X sent-target recording patch")
else:
    raise SystemExit("Unexpected LeRobot recorder source; review the patch before proceeding")
