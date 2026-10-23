import logging
import math
import time
from dataclasses import dataclass, field

from lerobot.teleoperators import Teleoperator, TeleoperatorConfig

from .protocol import KEYS, PassiveReader, TargetFramesUnavailable

logger = logging.getLogger(__name__)


@TeleoperatorConfig.register_subclass("piper_x_leader")
@dataclass
class PiperXLeaderConfig(TeleoperatorConfig):
    can_name: str = "piper_leader"
    source: str = "feedback"
    connect_timeout_s: float = 10.0
    feedback_timeout_s: float = 0.5
    control_mode: str = "joint"
    require_start_alignment: bool = False
    automatic_reset: bool = False
    debug_motion: bool = False
    teaching_mode_switch: str = "sdk"
    teaching_button_timeout_s: float = 120.0
    reset_joint_tolerance_rad: float = 0.005
    reset_gripper_tolerance_m: float = 0.001
    reset_motion_speed: int = 10
    reset_max_joint_step_rad: float = 0.003
    reset_max_gripper_step_m: float = 0.001
    reset_gripper_max_m: float = 0.07
    reset_timeout_s: float = 60.0
    reset_mode_timeout_s: float = 5.0
    # Independent conservative leader envelope; verify for this installation.
    reset_joint_limits_rad: list[list[float]] = field(default_factory=lambda: [
        [math.radians(lo), math.radians(hi)]
        for lo, hi in [(-92, 92), (-1.3, 90), (-80, 2.4), (-90, 90), (-77, 19), (-90, 90)]
    ])
    alignment_joint_tolerance_rad: float = 0.05
    alignment_gripper_tolerance_m: float = 0.005
    alignment_hold_s: float = 0.5
    alignment_timeout_s: float = 120.0

    def __post_init__(self):
        if self.control_mode != "joint":
            raise ValueError("Only direct joint control is supported")
        if self.source not in ("control", "feedback"):
            raise ValueError("source must be control or feedback")
        for value in (self.connect_timeout_s, self.feedback_timeout_s,
                      self.alignment_joint_tolerance_rad, self.alignment_gripper_tolerance_m,
                      self.alignment_hold_s, self.alignment_timeout_s,
                      self.reset_max_joint_step_rad, self.reset_max_gripper_step_m,
                      self.reset_joint_tolerance_rad, self.reset_gripper_tolerance_m,
                      self.teaching_button_timeout_s,
                      self.reset_gripper_max_m, self.reset_timeout_s, self.reset_mode_timeout_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Timeouts must be positive and finite")
        if not 1 <= self.reset_motion_speed <= 100:
            raise ValueError("reset_motion_speed must be 1..100")
        if self.teaching_mode_switch not in ("sdk", "manual"):
            raise ValueError("teaching_mode_switch must be sdk or manual")
        if self.automatic_reset and (
            self.reset_joint_tolerance_rad > self.alignment_joint_tolerance_rad
            or self.reset_gripper_tolerance_m > self.alignment_gripper_tolerance_m
        ):
            raise ValueError("Reset tolerances must not exceed alignment tolerances")
        if len(self.reset_joint_limits_rad) != 6 or any(
            len(pair) != 2 or not all(math.isfinite(v) for v in pair) or pair[0] >= pair[1]
            for pair in self.reset_joint_limits_rad
        ):
            raise ValueError("reset_joint_limits_rad must contain six finite [min, max] pairs")
        if self.automatic_reset and (self.source != "feedback" or not self.require_start_alignment):
            raise ValueError("automatic_reset requires source=feedback and require_start_alignment=true")


class PiperXLeader(Teleoperator):
    config_class = PiperXLeaderConfig
    name = "piper_x_leader"

    def __init__(self, config):
        super().__init__(config)
        self.config = config
        self.reader = PassiveReader(config.can_name, config.source, config.feedback_timeout_s)
        self._hold_positions = None
        self._hold_since = None
        self._teaching_required = False
        self._prepared_for_capture = False

    @property
    def action_features(self):
        return dict.fromkeys(KEYS, float)

    @property
    def feedback_features(self):
        return {}

    @property
    def is_connected(self):
        return self.reader.is_connected

    @property
    def is_calibrated(self):
        return True

    def calibrate(self):
        pass

    def configure(self):
        pass

    def connect(self, calibrate=True):
        try:
            logger.info("Waiting for leader targets: gently move the supported leader and its gripper")
            self.reader.connect()
            self.reader.wait_ready(self.config.connect_timeout_s)
        except BaseException:
            self.reader.disconnect()
            raise

    def get_action(self):
        return self._read_action()

    def prepare_recording(self, robot, events, fps):
        """Reset the motorized leader if opted in, then gate capture on alignment."""
        if events.get("stop_recording"):
            return False
        if self.config.automatic_reset and self._prepared_for_capture:
            from .leader_reset import TeachingModeEnded

            # Preparation can become stale during the environment reset window.
            # Recheck the physical pose and teaching state before reusing it.
            try:
                pose = self._read_action()
                target = robot.config.recording_reset_pose
                self._prepared_for_capture = all(
                    math.isfinite(pose[k]) and abs(pose[k] - target[k]) <= (
                        self.config.alignment_joint_tolerance_rad if i < 6
                        else self.config.alignment_gripper_tolerance_m
                    ) for i, k in enumerate(KEYS)
                )
            except (TeachingModeEnded, TargetFramesUnavailable):
                self._prepared_for_capture = False
        if self.config.automatic_reset and not self._prepared_for_capture:
            from .leader_reset import reset_leader

            try:
                reset_leader(self, robot, events, fps)
            except InterruptedError:
                if events.get("stop_recording"):
                    return False
                raise
        self._prepared_for_capture = False
        return self.wait_for_start_alignment(robot, events, fps)

    def recording_reset(self, robot, events, hold_time_s, fps):
        """Return follower, prepare leader, then run the environment reset window."""
        self._prepared_for_capture = False
        robot.recording_reset(events, hold_time_s=0, fps=fps)
        if events.get("stop_recording"):
            return
        if not self.prepare_recording(robot, events, fps):
            return
        self._prepared_for_capture = True
        if hold_time_s > 0:
            logger.info("Both arms prepared; environment reset window started")
            robot.recording_reset(events, hold_time_s=hold_time_s, fps=fps)
        else:
            logger.info("Both arms prepared; saving episode with no extra environment hold")

    def get_recording_action(self, observation, events, follower_config=None):
        """A manual teaching-OFF event ends capture without pursuing new targets."""
        from .leader_reset import TeachingModeEnded

        try:
            return self.get_action_for_observation(observation, follower_config)
        except TeachingModeEnded:
            if not (self.config.automatic_reset and self.config.teaching_mode_switch == "manual"):
                raise
            logger.info("Leader teaching turned OFF: ending episode and starting reset")
            events["exit_early"] = False
            return None

    def wait_for_start_alignment(self, robot, events, fps):
        """Hold the follower while the user manually aligns the passive leader.

        Called before recording starts, so waiting consumes no episode time or
        dataset frames. In automatic mode, teaching OFF restarts the bounded
        leader reset before alignment resumes.
        """
        from .leader_reset import TeachingModeEnded

        target = robot.config.recording_reset_pose
        if target is None:
            raise ValueError("Leader alignment requires robot.recording_reset_pose")
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("Alignment fps must be positive and finite")
        started = time.monotonic()
        aligned_since = None
        last_log = float("-inf")
        logger.warning("Keep the supported leader and gripper at the saved starting pose; "
                       "recording waits for alignment. Press q/Esc to stop.")
        while True:
            if events.get("stop_recording"):
                return False
            now = time.monotonic()
            if now - started >= self.config.alignment_timeout_s:
                raise TimeoutError("Leader alignment timed out; turn teaching ON and match the saved pose. "
                                   "Recording was not started")
            # Keeps all follower freshness, status and command bounds in force.
            robot.send_action(target)
            present = robot.reader.positions()
            try:
                leader = self._read_action()
            except TargetFramesUnavailable:
                leader = None
            except TeachingModeEnded:
                if not (self.config.automatic_reset and self.config.teaching_mode_switch == "manual"):
                    raise
                from .leader_reset import reset_leader

                logger.info("Teaching turned OFF before capture; returning leader to saved pose")
                try:
                    reset_leader(self, robot, events, fps)
                except InterruptedError:
                    if events.get("stop_recording"):
                        return False
                    raise
                started = time.monotonic()
                aligned_since = None
                continue
            tolerances = {k: self.config.alignment_joint_tolerance_rad if i < 6
                          else self.config.alignment_gripper_tolerance_m for i, k in enumerate(KEYS)}
            aligned = leader is not None and all(
                math.isfinite(leader[k]) and abs(leader[k] - target[k]) <= tolerances[k]
                and math.isfinite(present[k]) and abs(present[k] - target[k]) <= tolerances[k]
                for k in KEYS
            )
            if aligned:
                if aligned_since is None:
                    aligned_since = now
                if now - aligned_since >= self.config.alignment_hold_s:
                    self._hold_positions = None
                    self._hold_since = None
                    events["exit_early"] = False
                    logger.info("Leader and follower aligned; preparation complete")
                    return True
            else:
                aligned_since = None
            if now - last_log >= (1 if self.config.debug_motion else 5):
                if leader is None:
                    logger.warning("Waiting for fresh leader feedback; gently move the supported leader")
                elif self.config.debug_motion:
                    logger.warning("Leader minus saved pose: joints(deg)=%s gripper(mm)=%.1f",
                                   [round(math.degrees(leader[k] - target[k]), 1) for k in KEYS[:6]],
                                   1000 * (leader[KEYS[-1]] - target[KEYS[-1]]))
                else:
                    logger.warning("Waiting for start alignment: keep the leader near the saved pose")
                last_log = now
            time.sleep(1 / fps)

    def _read_action(self):
        if not self.is_connected:
            raise ConnectionError("Piper-X leader is not connected")
        if self._teaching_required:
            from .leader_reset import require_teaching

            require_teaching(self.reader)
        return self.reader.positions()

    def get_action_for_observation(self, observation, follower_config=None):
        """Pause at the follower pose on target silence; never pursue a stale target.

        This hook is used by the pinned teleoperation/recording loops. Standalone
        get_action remains strict. CAN faults still raise. After pausing, every
        target frame must arrive again before following resumes.
        """
        try:
            action = self._read_action()
        except TargetFramesUnavailable:
            action = None
        if self._hold_since is not None and action is not None:
            with self.reader.lock:
                refreshed = all(
                    self.reader.frames.get(i, (None, 0))[1] > self._hold_since for i in self.reader.ids
                )
            if not refreshed:
                action = None
        if action is None:
            if self._hold_positions is None:
                self._hold_positions = {k: float(observation[k]) for k in KEYS}
                if not all(math.isfinite(v) for v in self._hold_positions.values()):
                    self._hold_positions = None
                    raise ValueError("Cannot pause at non-finite follower positions")
                self._hold_since = time.monotonic()
                logger.warning("Leader targets unavailable: pausing at follower pose until all targets refresh")
            return dict(self._hold_positions)
        if self._hold_positions is not None:
            logger.info("All leader targets refreshed: resuming bounded following")
        self._hold_positions = None
        self._hold_since = None
        return action

    def send_feedback(self, feedback):
        if feedback:
            raise ValueError("Piper-X leader does not support force feedback")

    def disconnect(self):
        self.reader.disconnect()
        self._hold_positions = None
        self._hold_since = None
        self._teaching_required = False
        self._prepared_for_capture = False
