import logging
import math
import time
from dataclasses import dataclass

from lerobot.teleoperators import Teleoperator, TeleoperatorConfig

from .protocol import KEYS, PassiveReader, TargetFramesUnavailable

logger = logging.getLogger(__name__)


@TeleoperatorConfig.register_subclass("piper_x_leader")
@dataclass
class PiperXLeaderConfig(TeleoperatorConfig):
    can_name: str = "piper_leader"
    source: str = "control"
    connect_timeout_s: float = 10.0
    feedback_timeout_s: float = 3.0

    def __post_init__(self):
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
        self.config = config
        self.reader = PassiveReader(config.can_name, config.source, config.feedback_timeout_s)
        self._hold_positions = None
        self._hold_since = None

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
        if not self.is_connected:
            raise ConnectionError("Piper-X leader is not connected")
        return self.reader.positions()

    def get_action_for_observation(self, observation):
        """Pause at the follower pose on target silence; never pursue a stale target.

        This hook is used by the pinned teleoperation/recording loops. Standalone
        get_action remains strict. CAN faults still raise. After pausing, every
        target frame must arrive again before following resumes.
        """
        try:
            action = self.get_action()
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
