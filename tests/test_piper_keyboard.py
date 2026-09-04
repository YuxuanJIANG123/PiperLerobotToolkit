import pytest

from lerobot.common.robot_devices.teleop.piper_keyboard import PiperKeyboardController


def test_keyboard_action_is_feedback_referenced():
    controller = PiperKeyboardController(joint_step_rad=0.01, gripper_step_m=0.002)
    feedback = [0.4, -0.2, -0.5, 0.1, -0.3, 0.7, 0.04]
    controller.set_pressed_for_test("q", "s", "o")
    assert controller.action_from_feedback(feedback) == pytest.approx(
        [0.41, -0.21, -0.5, 0.1, -0.3, 0.7, 0.042]
    )
    # Input events are pulses and are consumed once.
    assert controller.action_from_feedback(feedback) == feedback


def test_opposite_keys_cancel_and_no_keys_hold():
    controller = PiperKeyboardController()
    feedback = [0.1] * 7
    assert controller.action_from_feedback(feedback) == feedback
    controller.set_pressed_for_test("q", "a", "o", "l")
    assert controller.action_from_feedback(feedback) == feedback


def test_feedback_dimension_is_checked():
    with pytest.raises(ValueError, match="6 joints and gripper"):
        PiperKeyboardController().action_from_feedback([0.0] * 6)
