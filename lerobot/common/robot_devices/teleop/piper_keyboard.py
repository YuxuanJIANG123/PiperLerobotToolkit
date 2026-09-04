"""Thread-safe keyboard teleoperator for one AgileX Piper arm."""

import threading

from lerobot.common.robot_devices.teleop.terminal_keyboard import TerminalKeyboardListener


class PiperKeyboardController:
    """Convert held keys into feedback-referenced joint position increments."""

    JOINT_KEYS = (("q", "a"), ("w", "s"), ("e", "d"), ("r", "f"), ("t", "g"), ("y", "h"))

    def __init__(self, joint_step_rad: float = 0.003, gripper_step_m: float = 0.0002):
        self.joint_step_rad = joint_step_rad
        self.gripper_step_m = gripper_step_m
        self._pending: set[str] = set()
        self._lock = threading.Lock()
        self._listener = TerminalKeyboardListener(self._on_key)

    def _on_key(self, key: str) -> None:
        if key in {item for pair in self.JOINT_KEYS for item in pair} | {"o", "l"}:
            with self._lock:
                self._pending.add(key)

    def start(self) -> None:
        self._listener.start()

    def stop(self) -> None:
        self._listener.stop()
        with self._lock:
            self._pending.clear()

    def action_from_feedback(self, feedback: list[float]) -> list[float]:
        if len(feedback) != 7:
            raise ValueError(f"Expected 6 joints and gripper, got {len(feedback)} values")
        with self._lock:
            pressed = self._pending.copy()
            self._pending.clear()
        action = [float(value) for value in feedback]
        for index, (positive, negative) in enumerate(self.JOINT_KEYS):
            action[index] += ((positive in pressed) - (negative in pressed)) * self.joint_step_rad
        action[6] += (("o" in pressed) - ("l" in pressed)) * self.gripper_step_m
        return action

    def set_pressed_for_test(self, *keys: str) -> None:
        with self._lock:
            self._pending = {key.lower() for key in keys}
