"""Route Piper-X's unrecorded reset phase to its saved-pose controller."""

from pathlib import Path

path = Path(__file__).resolve().parents[1] / "vendor/lerobot/src/lerobot/scripts/lerobot_record.py"
source = path.read_text()
anchor = '    # Multi-teleop is LeKiwi-only: one arm leader plus a keyboard that drives the base.\n'
addition = '''    # Piper-X reset motion is never added to the dataset or driven by the leader.
    if dataset is None and robot.name == "piper_x" and robot.config.recording_reset_pose is not None:
        robot.recording_reset(events, control_time_s, fps)
        return

'''
if addition + anchor in source:
    print("Piper-X recording reset patch already applied")
elif source.count(anchor) == 1:
    path.write_text(source.replace(anchor, addition + anchor))
    print("Applied Piper-X recording reset patch")
else:
    raise SystemExit("Unexpected LeRobot recorder source; review reset patch")
