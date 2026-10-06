import logging
import math
import time
from dataclasses import dataclass, field

from lerobot.teleoperators import Teleoperator, TeleoperatorConfig

from .end_effector import EndEffectorConfig, RelativeEndEffector
from .protocol import KEYS, PassiveReader, TargetFramesUnavailable

logger = logging.getLogger(__name__)


@TeleoperatorConfig.register_subclass("piper_x_leader")
@dataclass
class PiperXLeaderConfig(TeleoperatorConfig):
    can_name: str = "piper_leader"
    source: str = "control"
    connect_timeout_s: float = 10.0
    feedback_timeout_s: float = 3.0
    control_mode: str = "joint"
    eef: EndEffectorConfig = field(default_factory=EndEffectorConfig)

    def __post_init__(self):
        if self.control_mode not in ("joint", "end_effector"):
            raise ValueError("control_mode must be joint or end_effector")
        if self.source not in ("control", "feedback"):
            raise ValueError("source must be control or feedback")
        for value in (self.connect_timeout_s, self.feedback_timeout_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Timeouts must be positive and finite")


class PiperXLeader(Teleoperator):
    config_class = PiperXLeaderConfig
    name = "piper_x_leader"

    def __init__(self, config):
        super().__init__(config)
        if config.control_mode == "end_effector" and not config.eef.frames_verified:
            raise ValueError(
                "End-effector live mode requires verified base alignment, TCPs, and joint conventions. "
                "Run scripts/preview_end_effector.py first; set --teleop.eef.frames_verified=true "
                "only after checking those transforms on your installed arms."
            )
        self.config = config
        self.reader = PassiveReader(config.can_name, config.source, config.feedback_timeout_s)
        self._hold_positions = None
        self._hold_since = None
        self._eef_mapper = None
        self._last_eef_log = float("-inf")

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
        if self.config.control_mode == "end_effector":
            raise RuntimeError("End-effector mode requires the patched observation-aware LeRobot loop")
        return self._read_action()

    def _read_action(self):
        if not self.is_connected:
            raise ConnectionError("Piper-X leader is not connected")
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
                if self._eef_mapper is not None:
                    self._eef_mapper.reset()
                logger.warning("Leader targets unavailable: pausing at follower pose until all targets refresh")
            return dict(self._hold_positions)
        if self._hold_positions is not None:
            logger.info("All leader targets refreshed: resuming bounded following")
        self._hold_positions = None
        self._hold_since = None
        if self.config.control_mode == "end_effector":
            if follower_config is None:
                raise RuntimeError("End-effector mode requires follower configuration; reapply the loop patch")
            if self._eef_mapper is None:
                self._eef_mapper = RelativeEndEffector(
                    self.config.eef, follower_config.joint_limits_rad, follower_config.gripper_max_m
                )
            action = self._eef_mapper.map(action, observation)
            if time.monotonic() - self._last_eef_log >= 1.0:
                logger.info("End-effector mapping: %s", self._eef_mapper.diagnostics)
                self._last_eef_log = time.monotonic()
        return action

    def send_feedback(self, feedback):
        if feedback:
            raise ValueError("Piper-X leader does not support force feedback")

    def disconnect(self):
        self.reader.disconnect()
        self._hold_positions = None
        self._hold_since = None
        if self._eef_mapper is not None:
            self._eef_mapper.reset()
