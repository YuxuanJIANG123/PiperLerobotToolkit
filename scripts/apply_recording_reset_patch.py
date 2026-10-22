"""Route Piper-X's unrecorded reset phase to its saved-pose controller."""

from pathlib import Path

path = Path(__file__).resolve().parents[1] / "vendor/lerobot/src/lerobot/scripts/lerobot_record.py"
source = path.read_text()
anchor = '    # Multi-teleop is LeKiwi-only: one arm leader plus a keyboard that drives the base.\n'
addition = '''    # Piper-X reset motion is never added to the dataset or driven by the leader.
    if dataset is None and robot.name == "piper_x" and robot.config.recording_reset_pose is not None:
        if (teleop is not None and teleop.name == "piper_x_leader"
                and teleop.config.automatic_reset):
            teleop.recording_reset(robot, events, control_time_s, fps)
        else:
            robot.recording_reset(events, control_time_s, fps)
        return

'''
previous_reset = '''    # Piper-X reset motion is never added to the dataset or driven by the leader.
    if dataset is None and robot.name == "piper_x" and robot.config.recording_reset_pose is not None:
        robot.recording_reset(events, control_time_s, fps)
        return

'''
if addition + anchor in source:
    print("Piper-X recording reset patch already applied")
elif previous_reset + anchor in source:
    path.write_text(source.replace(previous_reset + anchor, addition + anchor))
    print("Updated Piper-X paired recording reset patch")
elif source.count(anchor) == 1:
    path.write_text(source.replace(anchor, addition + anchor))
    print("Applied Piper-X recording reset patch")
else:
    raise SystemExit("Unexpected LeRobot recorder source; review reset patch")

# Gate every episode, including retries, before any episode frames or timing.
source = path.read_text()
anchor = '                log_say(f"Recording episode {episode_index}", cfg.play_sounds)\n'
addition = '''                # Prepare the Piper leader before capture, outside episode timing.
                if (robot.name == "piper_x" and teleop is not None
                        and teleop.name == "piper_x_leader" and teleop.config.require_start_alignment):
                    if not teleop.prepare_recording(robot, events, cfg.dataset.fps):
                        break
'''
previous = addition.replace(
    "# Prepare the Piper leader before capture, outside episode timing.",
    "# Passive Piper leaders must be manually aligned before capture.",
).replace("teleop.prepare_recording(", "teleop.wait_for_start_alignment(")
if addition + anchor in source:
    print("Piper-X leader alignment patch already applied")
elif previous + anchor in source:
    path.write_text(source.replace(previous + anchor, addition + anchor))
    print("Updated Piper-X leader reset/alignment patch")
elif source.count(anchor) == 1:
    path.write_text(source.replace(anchor, addition + anchor))
    print("Applied Piper-X leader alignment patch")
else:
    raise SystemExit("Unexpected LeRobot recorder source; review leader alignment patch")

source = path.read_text()
anchor = "                dataset.save_episode()\n"
addition = '''                # A button press before the first frame must not save an empty episode.
                if robot.name == "piper_x" and not dataset.has_pending_frames():
                    timer.restart()
                    continue
'''
if addition + anchor in source:
    print("Piper-X empty episode guard already applied")
elif source.count(anchor) == 1:
    path.write_text(source.replace(anchor, addition + anchor))
    print("Applied Piper-X empty episode guard")
else:
    raise SystemExit("Unexpected LeRobot recorder source; review empty episode guard")

# Keep the large config dump and timing summaries behind the existing motion
# diagnostic switch for Piper; preserve upstream logging for other robots.
source = path.read_text()
anchor = "    logging.info(pformat(asdict(cfg)))\n"
replacement = '''    if cfg.robot.type != "piper_x" or cfg.robot.debug_motion:
        logging.info(pformat(asdict(cfg)))
'''
if replacement not in source:
    if source.count(anchor) != 1:
        raise SystemExit("Unexpected recorder config logging; review quiet output patch")
    source = source.replace(anchor, replacement)
for line in (
    '                    timer.log_episode_summary("discarded episode")\n',
    '                timer.log_episode_summary(f"episode {episode_index}")\n',
    '        timer.log_run_summary()\n',
):
    indent = line[:len(line) - len(line.lstrip())]
    replacement = indent + 'if robot.name != "piper_x" or robot.config.debug_motion:\n' + "    " + line
    if replacement not in source:
        if source.count(line) != 1:
            raise SystemExit("Unexpected recorder timing logging; review quiet output patch")
        source = source.replace(line, replacement)
path.write_text(source)
print("Piper-X diagnostic output follows robot.debug_motion")
