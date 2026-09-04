"""Small, unit-explicit wrapper around the AgileX Piper SDK."""

import time
from collections.abc import Sequence

import numpy as np
from piper_sdk import C_PiperInterface_V2

from lerobot.common.robot_devices.motors.configs import PiperMotorsBusConfig

JOINT_FACTOR = 1000.0 * 180.0 / np.pi  # rad -> SDK 0.001 degree
GRIPPER_FACTOR = 1_000_000.0  # metre -> SDK micrometre
JOINT_LIMITS_RAD = (
    np.asarray(
        [(-92000, 92000), (-1300, 90000), (-80000, 2400), (-90000, 90000), (-77000, 19000), (-90000, 90000)],
        dtype=np.float64,
    )
    / JOINT_FACTOR
)
GRIPPER_LIMIT_M = (0.0, 0.08)


class PiperMotorsBus:
    def __init__(self, config: PiperMotorsBusConfig):
        self.piper = C_PiperInterface_V2(config.can_name)
        self.piper.ConnectPort()
        self.motors = config.motors

    @property
    def motor_names(self) -> list[str]:
        return list(self.motors)

    @property
    def motor_models(self) -> list[str]:
        return [model for _, model in self.motors.values()]

    @property
    def motor_indices(self) -> list[int]:
        return [index for index, _ in self.motors.values()]

    def connect(self, enable: bool, timeout_s: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if enable:
                enable_fn = getattr(self.piper, "EnablePiper", None)
                if enable_fn is not None:
                    enable_fn()
                else:
                    self.piper.EnableArm(7)
            else:
                self.piper.DisableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x02, 0)
            if self._enabled() == enable:
                return True
            time.sleep(0.1)
        return False

    def _enabled(self) -> bool:
        info = self.piper.GetArmLowSpdInfoMsgs()
        motors = (info.motor_1, info.motor_2, info.motor_3, info.motor_4, info.motor_5, info.motor_6)
        return all(motor.foc_status.driver_enable_status for motor in motors)

    def set_calibration(self):
        return

    def revert_calibration(self):
        return

    def apply_calibration(self):
        return

    def read(self) -> dict[str, int]:
        joints = self.piper.GetArmJointMsgs().joint_state
        gripper = self.piper.GetArmGripperMsgs().gripper_state
        return {
            "joint_1": joints.joint_1,
            "joint_2": joints.joint_2,
            "joint_3": joints.joint_3,
            "joint_4": joints.joint_4,
            "joint_5": joints.joint_5,
            "joint_6": joints.joint_6,
            "gripper": gripper.grippers_angle,
        }

    def read_positions(self) -> list[float]:
        raw = self.read()
        return [raw[f"joint_{index}"] / JOINT_FACTOR for index in range(1, 7)] + [
            raw["gripper"] / GRIPPER_FACTOR
        ]

    def limit_target(
        self, target: Sequence[float], feedback: Sequence[float], max_joint_delta_rad: float
    ) -> list[float]:
        if len(target) != 7 or len(feedback) != 7:
            raise ValueError("Piper target and feedback must contain 6 joints plus gripper")
        target_array = np.asarray(target, dtype=np.float64)
        feedback_array = np.asarray(feedback, dtype=np.float64)
        if not np.all(np.isfinite(target_array)) or not np.all(np.isfinite(feedback_array)):
            raise ValueError("Refusing to command non-finite Piper positions")
        target_array[:6] = np.clip(
            target_array[:6],
            feedback_array[:6] - max_joint_delta_rad,
            feedback_array[:6] + max_joint_delta_rad,
        )
        target_array[:6] = np.clip(target_array[:6], JOINT_LIMITS_RAD[:, 0], JOINT_LIMITS_RAD[:, 1])
        target_array[6] = np.clip(target_array[6], *GRIPPER_LIMIT_M)
        return target_array.tolist()

    def write(self, target_joint: Sequence[float], speed: int = 20) -> None:
        if len(target_joint) != 7:
            raise ValueError("Piper command must contain 6 joints plus gripper")
        joints = [round(float(value) * JOINT_FACTOR) for value in target_joint[:6]]
        gripper = round(float(target_joint[6]) * GRIPPER_FACTOR)
        self.piper.MotionCtrl_2(0x01, 0x01, int(speed), 0x00)
        self.piper.JointCtrl(*joints)
        self.piper.GripperCtrl(gripper, 1000, 0x01, 0)
