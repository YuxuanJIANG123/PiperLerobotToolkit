"""LeRobot follower adapter. Joint angles in radians, gripper opening in metres."""

import logging
import math
import time
from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.robots import Robot, RobotConfig

from .protocol import GRIPPER_FACTOR, JOINT_FACTOR, KEYS, PassiveReader
from .safety import limit_action, validate_initial_pose

logger = logging.getLogger(__name__)


@RobotConfig.register_subclass("piper_x")
@dataclass
class PiperXConfig(RobotConfig):
    can_name: str = "piper_follower"
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    enable_motors: bool = False
    motion_speed: int = 10
    # Use the SDK's planned position/velocity joint mode by default. Instant
    # response (0xAD) is optional and must be validated for the installed firmware.
    high_follow: bool = False
    debug_motion: bool = False
    connect_timeout_s: float = 5.0
    feedback_timeout_s: float = 0.5
    max_joint_step_rad: float = 0.003
    max_gripper_step_m: float = 0.001
    gripper_max_m: float = 0.07
    gripper_effort: int = 1000
    # Conservative envelope inherited from this PC's earlier Piper deployment.
    # Verify against this arm's installation; these are not claimed factory limits.
    joint_limits_rad: list[list[float]] = field(
        default_factory=lambda: [
            [math.radians(lo), math.radians(hi)]
            for lo, hi in [(-92, 92), (-1.3, 90), (-80, 2.4), (-90, 90), (-77, 19), (-90, 90)]
        ]
    )

    def __post_init__(self):
        super().__post_init__()
        for name in (
            "connect_timeout_s",
            "feedback_timeout_s",
            "max_joint_step_rad",
            "max_gripper_step_m",
            "gripper_max_m",
        ):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if not 1 <= self.motion_speed <= 100 or not 0 <= self.gripper_effort <= 5000:
            raise ValueError("motion_speed must be 1..100 and gripper_effort 0..5000")
        if len(self.joint_limits_rad) != 6 or any(
            len(pair) != 2 or not all(math.isfinite(v) for v in pair) or pair[0] >= pair[1]
            for pair in self.joint_limits_rad
        ):
            raise ValueError("joint_limits_rad must contain six finite [min, max] pairs")
        if set(self.cameras) & set(KEYS):
            raise ValueError("Camera names must not overlap joint names")


class PiperX(Robot):
    config_class = PiperXConfig
    name = "piper_x"

    def __init__(self, config):
        super().__init__(config)
        self.config = config
        self.reader = PassiveReader(config.can_name, timeout_s=config.feedback_timeout_s)
        self.cameras = make_cameras_from_configs(config.cameras)
        self.sdk = None
        self._connected = False
        self._last_motion_log = float("-inf")

    @property
    def action_features(self):
        return dict.fromkeys(KEYS, float)

    @property
    def observation_features(self):
        return {
            **self.action_features,
            **{name: (cam.height, cam.width, 3) for name, cam in self.cameras.items()},
        }

    @property
    def is_connected(self):
        return (
            self._connected
            and self.reader.is_connected
            and all(c.is_connected for c in self.cameras.values())
        )

    @property
    def is_calibrated(self):
        # Uses the arm's existing factory zero offsets. Never re-zero on connect.
        return True

    def calibrate(self):
        pass

    def configure(self):
        pass

    def connect(self, calibrate=True):
        if self._connected:
            raise ConnectionError("Piper-X already connected")
        try:
            self.reader.connect()
            current = self.reader.wait_ready(self.config.connect_timeout_s)
            if self.config.enable_motors:
                validate_initial_pose(current, self.config)
            for camera in self.cameras.values():
                camera.connect()
            if self.config.enable_motors:
                # Validate the initial pose before any command or enable request.
                current = self.reader.positions()
                validate_initial_pose(current, self.config)
                from piper_sdk import C_PiperInterface_V2

                self.sdk = C_PiperInterface_V2(self.config.can_name)
                self.sdk.ConnectPort(piper_init=False)
                # Seed the current pose before entering CAN control: changing
                # modes must not reactivate a previous session's joint target.
                self.sdk.JointCtrl(*(round(current[k] * JOINT_FACTOR) for k in KEYS[:6]))
                deadline = time.monotonic() + self.config.connect_timeout_s
                while True:
                    self.reader.positions()
                    self.sdk.MotionCtrl_1(0, 0, 0x02)  # Exit drag teaching on follower only.
                    self.sdk.MotionCtrl_2(
                        0x01, 0x01, self.config.motion_speed, 0xAD if self.config.high_follow else 0
                    )
                    status = self.sdk.GetArmStatus()
                    if (
                        status.time_stamp > 0
                        and status.arm_status.ctrl_mode == 1
                        and status.arm_status.mode_feed == 1
                        and status.arm_status.err_code == 0
                        and status.arm_status.arm_status == 0
                    ):
                        break
                    if time.monotonic() >= deadline:
                        raise ConnectionError(
                            "Follower did not enter CAN joint control: "
                            f"status_timestamp={status.time_stamp}, "
                            f"ctrl_mode={status.arm_status.ctrl_mode}, "
                            f"mode_feed={status.arm_status.mode_feed}, "
                            f"arm_status={status.arm_status.arm_status}, "
                            f"err_code={status.arm_status.err_code}. "
                            "Expected fresh status with ctrl_mode=1, mode_feed=1, "
                            "arm_status=0 and err_code=0."
                        )
                    time.sleep(0.05)
                current = self.reader.positions()
                validate_initial_pose(current, self.config)
                self.sdk.JointCtrl(*(round(current[k] * JOINT_FACTOR) for k in KEYS[:6]))
                deadline = time.monotonic() + self.config.connect_timeout_s
                while not self.sdk.EnablePiper():
                    self.reader.positions()
                    if time.monotonic() >= deadline:
                        raise ConnectionError("Follower enable timed out")
                    time.sleep(0.05)
            self._connected = True
        except BaseException:
            self.disconnect()
            raise

    def get_observation(self):
        if not self.is_connected:
            raise ConnectionError("Piper-X is not connected")
        return {**self.reader.positions(), **{name: cam.async_read() for name, cam in self.cameras.items()}}

    def send_action(self, action):
        if not self.is_connected:
            raise ConnectionError("Piper-X is not connected")
        if self.sdk is None:
            raise RuntimeError("Read-only connection: set enable_motors=true to command the follower")
        self.reader.check_follower_status()
        present = self.reader.positions()
        sent = limit_action(action, present, self.config)
        self.sdk.MotionCtrl_2(0x01, 0x01, self.config.motion_speed, 0xAD if self.config.high_follow else 0)
        self.sdk.JointCtrl(*(round(sent[k] * JOINT_FACTOR) for k in KEYS[:6]))
        self.sdk.GripperCtrl(round(sent[KEYS[-1]] * GRIPPER_FACTOR), self.config.gripper_effort, 0x01, 0)
        if self.config.debug_motion and time.monotonic() - self._last_motion_log >= 1.0:
            self._last_motion_log = time.monotonic()
            logger.info(
                "Motion diagnostic (J1-J6 degrees, gripper mm): measured=%s leader=%s "
                "sdk_target=%s motor_enable=%s feedback_timing=%s",
                [round(present[k] * (180 / math.pi if i < 6 else 1000), 3)
                 for i, k in enumerate(KEYS)],
                [round(action[k] * (180 / math.pi if i < 6 else 1000), 3)
                 for i, k in enumerate(KEYS)],
                [round(sent[k] * (180 / math.pi if i < 6 else 1000), 3)
                 for i, k in enumerate(KEYS)],
                self.sdk.GetArmEnableStatus(),
                self.reader.timing(),
            )
        return sent

    def disconnect(self):
        # Close communication while retaining the last bounded position target.
        # Do not home or disable torque: an unsupported arm could fall.
        self._connected = False
        for camera in self.cameras.values():
            try:
                if camera.is_connected:
                    camera.disconnect()
            except Exception:
                logger.exception("Camera cleanup failed")
        try:
            if self.sdk is not None:
                self.sdk.DisconnectPort()
        except Exception:
            logger.exception("Piper SDK cleanup failed")
        self.sdk = None
        try:
            self.reader.disconnect()
        except Exception:
            logger.exception("CAN reader cleanup failed")
