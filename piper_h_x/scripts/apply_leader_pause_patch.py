"""Add the Piper observation-aware pause hook to the pinned LeRobot loops."""

from pathlib import Path

root = Path(__file__).resolve().parents[1] / "vendor/lerobot/src/lerobot/scripts"
for filename, variable in (("lerobot_teleoperate.py", "raw_action"), ("lerobot_record.py", "act")):
    path = root / filename
    source = path.read_text()
    anchor = f"                {variable} = teleop.get_action()\n"
    replacement = (
        '                if robot.name == "piper_x" and teleop.name == "piper_x_leader":\n'
        f"                    {variable} = teleop.get_action_for_observation(obs, follower_config=robot.config)\n"
        "                else:\n"
        f"                    {variable} = teleop.get_action()\n"
    )
    previous = replacement.replace("obs, follower_config=robot.config", "obs")
    if replacement in source:
        print(f"Pause hook already applied: {filename}")
    elif source.count(previous) == 1:
        path.write_text(source.replace(previous, replacement))
        print(f"Updated observation hook with follower limits: {filename}")
    elif source.count(anchor) == 1:
        path.write_text(source.replace(anchor, replacement))
        print(f"Applied pause hook: {filename}")
    else:
        raise SystemExit(f"Unexpected source: {filename}; review before patching")
