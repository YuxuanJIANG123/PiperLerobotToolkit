"""LeRobot v2.1 integration for one keyboard-controlled AgileX Piper."""

import time
from dataclasses import replace

import torch

from lerobot.common.robot_devices.cameras.utils import make_cameras_from_configs
from lerobot.common.robot_devices.motors.utils import make_motors_buses_from_configs
from lerobot.common.robot_devices.robots.configs import PiperRobotConfig
from lerobot.common.robot_devices.teleop.piper_keyboard import PiperKeyboardController
from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError, RobotDeviceNotConnectedError


class PiperRobot:
    def __init__(self, config: PiperRobotConfig | None = None, **kwargs):
        self.config = replace(config or PiperRobotConfig(), **kwargs)
        self.robot_type = self.config.type
        self.cameras = make_cameras_from_configs(self.config.cameras)
        self.config.follower_arm["main"].can_name = self.config.can_name
        self.piper_motors = make_motors_buses_from_configs(self.config.follower_arm)
        self.arm = self.piper_motors["main"]
        self.teleop = (
            None
            if self.config.inference_time
            else PiperKeyboardController(self.config.joint_step_rad, self.config.gripper_step_m)
        )
        self.logs: dict[str, float] = {}
        self.is_connected = False

    @property
    def camera_features(self) -> dict:
        return {
            f"observation.images.{name}": {
                "shape": (camera.height, camera.width, camera.channels),
                "names": ["height", "width", "channels"],
                "info": None,
            }
            for name, camera in self.cameras.items()
        }

    @property
    def motor_features(self) -> dict:
        names = list(self.arm.motor_names)
        feature = {"dtype": "float32", "shape": (len(names),), "names": names}
        return {"action": feature.copy(), "observation.state": feature.copy()}

    def connect(self) -> None:
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError("Piper is already connected")
        if not self.arm.connect(enable=True):
            raise ConnectionError("Piper did not report enabled within the timeout")
        connected = []
        try:
            for camera in self.cameras.values():
                camera.connect()
                connected.append(camera)
            if self.teleop is not None:
                self.teleop.start()
            self.is_connected = True
        except Exception:
            for camera in reversed(connected):
                camera.disconnect()
            # The arm may be gravity-loaded. Keep it enabled at its measured
            # position instead of dropping it because a camera failed to start.
            self.arm.write(self.arm.read_positions(), speed=self.config.motion_speed)
            raise

    def disconnect(self) -> None:
        if not self.is_connected:
            return
        if self.teleop is not None:
            self.teleop.stop()
        try:
            self.arm.write(self.arm.read_positions(), speed=self.config.motion_speed)
        finally:
            if self.config.disable_on_disconnect:
                self.arm.connect(enable=False)
            for camera in self.cameras.values():
                camera.disconnect()
            self.is_connected = False

    def run_calibration(self) -> None:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError("Piper is not connected")

    def teleop_step(self, record_data=False):
        self._require_connected()
        if self.teleop is None:
            self.teleop = PiperKeyboardController(self.config.joint_step_rad, self.config.gripper_step_m)
            self.teleop.start()
        before = time.perf_counter()
        state = self.arm.read_positions()
        requested = self.teleop.action_from_feedback(state)
        action = self.arm.limit_target(requested, state, self.config.max_relative_target_rad)
        self.logs["read_pos_dt_s"] = time.perf_counter() - before
        self.arm.write(action, speed=self.config.motion_speed)
        if not record_data:
            return
        observation = {"observation.state": torch.tensor(state, dtype=torch.float32)}
        for name, camera in self.cameras.items():
            observation[f"observation.images.{name}"] = torch.from_numpy(camera.async_read())
        return observation, {"action": torch.tensor(action, dtype=torch.float32)}

    def capture_observation(self) -> dict:
        self._require_connected()
        observation = {"observation.state": torch.tensor(self.arm.read_positions(), dtype=torch.float32)}
        for name, camera in self.cameras.items():
            observation[f"observation.images.{name}"] = torch.from_numpy(camera.async_read())
        return observation

    def send_action(self, action: torch.Tensor) -> torch.Tensor:
        self._require_connected()
        state = self.arm.read_positions()
        target = self.arm.limit_target(action.tolist(), state, self.config.max_relative_target_rad)
        self.arm.write(target, speed=self.config.motion_speed)
        return torch.tensor(target, dtype=action.dtype, device=action.device)

    def teleop_safety_stop(self) -> None:
        self._require_connected()
        self.arm.write(self.arm.read_positions(), speed=self.config.motion_speed)

    def _require_connected(self) -> None:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError("Piper is not connected")
