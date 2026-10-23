"""Bounded motorized leader reset, followed by an acknowledged teaching handoff."""

import logging
import math
import time

from .piper_x import PiperXConfig
from .protocol import GRIPPER_FACTOR, JOINT_FACTOR, KEYS
from .safety import limit_action, validate_initial_pose

logger = logging.getLogger(__name__)


class TeachingModeEnded(ConnectionError):
    """A healthy leader has left teaching; recording can finish its episode."""


def validate_leader_feedback(present, limits):
    """Use the shared endpoint feedback tolerances for leader resets."""
    validate_initial_pose(present, limits)


def reset_error_summary(present, target, joint_tolerance=0.005, gripper_tolerance=0.001):
    """Report axes still outside return tolerances in human-readable units."""
    errors = []
    for i, key in enumerate(KEYS):
        delta = target[key] - present[key]
        if abs(delta) > (joint_tolerance if i < 6 else gripper_tolerance):
            unit, scale = ("deg", 180 / math.pi) if i < 6 else ("mm", 1000)
            errors.append(f"{key}: remaining {delta * scale:+.2f} {unit}")
    return "; ".join(errors) or "all axes within tolerance"


def leader_status(reader):
    if reader.error is not None:
        raise ConnectionError("Leader CAN reader failed") from reader.error
    with reader.lock:
        frame = reader.frames.get(0x2A1)
    if frame is None or time.monotonic() - frame[1] > reader.timeout_s:
        raise ConnectionError("Missing or stale leader status")
    data, stamp = frame
    # 0x0B is teaching-record status, not a hardware fault.
    if data[1] not in (0, 0x0B) or int.from_bytes(data[6:8], "big"):
        raise ConnectionError(f"Leader fault: {data.hex()}")
    if data[0] not in (0, 1, 2) or data[3] not in (0, 1, 2):
        raise ConnectionError(f"Unsupported leader mode/playback state: {data.hex()}")
    return data, stamp


def require_teaching(reader, after=None):
    data, stamp = leader_status(reader)
    if after is None and data[1] == 0 and data[3] in (0, 2):
        raise TeachingModeEnded(f"Leader teaching mode turned off: {data.hex()}")
    if data[0] != 2 or data[3] != 1 or (after is not None and stamp <= after):
        raise ConnectionError(f"Leader teaching mode not acknowledged: {data.hex()}")


def reset_leader(leader, follower, events, fps):
    """Move only the leader; hold the follower and never disable either arm."""
    config = leader.config
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError("Leader reset fps must be positive and finite")
    if config.can_name == follower.config.can_name:
        raise ValueError("Leader and follower must use different CAN interfaces")
    target = follower.config.recording_reset_pose
    if target is None:
        raise ValueError("Automatic leader reset requires recording_reset_pose")
    limits = PiperXConfig(
        joint_limits_rad=config.reset_joint_limits_rad,
        gripper_max_m=config.reset_gripper_max_m,
        max_joint_step_rad=config.reset_max_joint_step_rad,
        max_gripper_step_m=config.reset_max_gripper_step_m,
    )
    validate_initial_pose(target, limits, feedback=False)
    validate_leader_feedback(leader.reader.positions(), limits)
    leader_status(leader.reader)
    from piper_sdk import C_PiperInterface_V2

    sdk = C_PiperInterface_V2(config.can_name)

    def checked():
        if events.get("stop_recording"):
            raise InterruptedError("Leader reset cancelled before recording")
        leader_status(leader.reader)
        present = leader.reader.positions()
        validate_leader_feedback(present, limits)
        follower.send_action(target)
        return present

    def seed():
        present = checked()
        # Clamp even seed commands to the absolute limits while bounding their
        # offset from the original feedback (which may be barely outside).
        sent = limit_action(present, present, limits)
        sdk.JointCtrl(*(round(sent[k] * JOINT_FACTOR) for k in KEYS[:6]))

    def transition(label, command, acknowledged, timeout=None):
        started = time.monotonic()
        deadline = started + (config.reset_mode_timeout_s if timeout is None else timeout)
        logger.info("Leader reset: %s", label)
        while True:
            checked()
            data, stamp = leader_status(leader.reader)
            if stamp > started and acknowledged(data):
                return started
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Leader {label} timed out; latest status={data.hex()}")
            command()
            time.sleep(1 / fps)

    try:
        sdk.ConnectPort(piper_init=False)
        if config.teaching_mode_switch == "manual":
            logger.warning("ACTION: turn OFF the leader teaching button, then keep clear for powered reset. "
                           "Waiting up to %.0f s; q/Esc cancels.", config.teaching_button_timeout_s)
            transition("wait for teaching OFF button", lambda: None,
                       lambda d: d[0] in (0, 1, 2) and d[3] in (0, 2) and d[1] == 0,
                       timeout=config.teaching_button_timeout_s)
        else:
            # Replace retained targets before requesting SDK teaching exit.
            seed()
            transition("exit teaching", lambda: sdk.MotionCtrl_1(0, 0, 2),
                       lambda d: d[3] in (0, 2) and d[1] == 0)
        seed()
        transition("standby", lambda: sdk.MotionCtrl_2(0, 1, config.reset_motion_speed, 0),
                   lambda d: d[0] == 0 and d[1] == 0 and d[3] in (0, 2))
        seed()
        transition("CAN joint control", lambda: sdk.MotionCtrl_2(1, 1, config.reset_motion_speed, 0),
                   lambda d: d[0] == 1 and d[2] == 1 and d[1] == 0 and d[3] in (0, 2))
        seed()
        deadline = time.monotonic() + config.reset_mode_timeout_s
        while True:
            checked()
            data, _ = leader_status(leader.reader)
            if data[0] != 1 or data[2] != 1 or data[1] != 0 or data[3] not in (0, 2):
                raise ConnectionError("Leader left CAN joint control before motor enable")
            if sdk.EnablePiper():
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("Leader motor enable timed out")
            time.sleep(1 / fps)
        move_started = time.monotonic()
        deadline = move_started + config.reset_timeout_s
        last_log = float("-inf")
        logger.warning("Leader returning under motor power; keep both arm paths clear")
        while True:
            present = checked()
            data, _ = leader_status(leader.reader)
            if data[0] != 1 or data[2] != 1 or data[1] != 0 or data[3] not in (0, 2):
                raise ConnectionError(f"Leader left CAN joint control during reset: {data.hex()}")
            if all(abs(present[k] - target[k]) <= (
                    config.reset_joint_tolerance_rad if i < 6 else config.reset_gripper_tolerance_m)
                   for i, k in enumerate(KEYS)):
                logger.info("Leader reached saved pose; entering teaching mode")
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("Leader return to saved pose timed out: "
                                   + reset_error_summary(present, target, config.reset_joint_tolerance_rad,
                                                         config.reset_gripper_tolerance_m))
            sent = limit_action(target, present, limits)
            now = time.monotonic()
            if config.debug_motion and now - last_log >= 1:
                logger.info(
                    "Leader reset progress %.1f/%.1f s: measured(deg/mm)=%s "
                    "command(deg/mm)=%s; %s",
                    now - move_started, config.reset_timeout_s,
                    [round(present[k] * (180 / math.pi if i < 6 else 1000), 2)
                     for i, k in enumerate(KEYS)],
                    [round(sent[k] * (180 / math.pi if i < 6 else 1000), 2)
                     for i, k in enumerate(KEYS)],
                    reset_error_summary(present, target, config.reset_joint_tolerance_rad,
                                        config.reset_gripper_tolerance_m),
                )
                last_log = now
            sdk.MotionCtrl_2(1, 1, config.reset_motion_speed, 0)
            sdk.JointCtrl(*(round(sent[k] * JOINT_FACTOR) for k in KEYS[:6]))
            sdk.GripperCtrl(round(sent[KEYS[-1]] * GRIPPER_FACTOR), 1000, 1, 0)
            time.sleep(1 / fps)
        if config.teaching_mode_switch == "manual":
            logger.warning("ACTION: reset complete — support the leader and turn ON its teaching button. "
                           "Keep it near the saved pose until capture starts. Waiting up to %.0f s.",
                           config.teaching_button_timeout_s)
            started = transition("wait for teaching ON button", lambda: None,
                                 lambda d: d[0] == 2 and d[3] == 1,
                                 timeout=config.teaching_button_timeout_s)
        else:
            started = transition("enter teaching", lambda: sdk.MotionCtrl_1(0, 0, 1),
                                 lambda d: d[0] == 2 and d[3] == 1)
        require_teaching(leader.reader, after=started)
        _, handoff_stamp = leader_status(leader.reader)
        # All position pairs must have arrived since the teaching handoff.
        deadline = time.monotonic() + config.reset_mode_timeout_s
        while True:
            checked()
            require_teaching(leader.reader)
            with leader.reader.lock:
                fresh = all(leader.reader.frames.get(i, (None, 0))[1] > handoff_stamp
                            for i in leader.reader.ids)
            if fresh:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("Leader feedback did not refresh after teaching handoff")
            time.sleep(1 / fps)
        leader._teaching_required = True
        logger.info("Leader teaching mode confirmed; checking start alignment")
    finally:
        # No automatic torque disable, emergency-stop recovery, zeroing or role changes.
        sdk.DisconnectPort()
