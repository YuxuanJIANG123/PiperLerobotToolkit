import pytest
from test_piper import FakeReader, FakeSDK

from lerobot_robot_piper_x import PiperX, PiperXConfig
from lerobot_robot_piper_x.protocol import JOINT_FACTOR, KEYS


def test_zero_requires_explicit_enable_and_valid_limits():
    with pytest.raises(ValueError, match="enable_motors"):
        PiperXConfig(reset_to_zero_on_connect=True)
    limits = [[0.1, 1.0]] * 6
    with pytest.raises(ValueError, match="Zero must"):
        PiperXConfig(enable_motors=True, reset_to_zero_on_connect=True, joint_limits_rad=limits)


def make_robot(tmp_path):
    robot = PiperX(PiperXConfig(calibration_dir=tmp_path, enable_motors=True))
    robot.reader, robot.sdk, robot._connected = FakeReader(), FakeSDK(), True
    return robot


def test_zero_is_gradual_and_preserves_gripper(tmp_path, monkeypatch):
    robot = make_robot(tmp_path)
    pose = dict.fromkeys(KEYS, 0.02)
    robot.reader.positions = lambda: dict(pose)
    monkeypatch.setattr("lerobot_robot_piper_x.piper_x.time.sleep", lambda _: None)
    steps = []

    def move(*raw):
        updated = [v / JOINT_FACTOR for v in raw]
        assert all(abs(v - pose[k]) <= robot.config.max_joint_step_rad for k, v in zip(KEYS, updated))
        pose.update(zip(KEYS[:6], updated))
        steps.append(updated)

    robot.sdk.JointCtrl = move
    robot.reset_to_zero()
    assert len(steps) > 1
    assert all(abs(pose[k]) <= 0.005 for k in KEYS[:6])
    assert all(args[0] == 20000 for name, args in robot.sdk.calls if name == "gripper")


def test_zero_timeout_and_feedback_fault_stop_commands(tmp_path, monkeypatch):
    robot = make_robot(tmp_path)
    robot.reader.positions = lambda: dict.fromkeys(KEYS, 0.02)
    times = iter([0.0, 61.0])
    monkeypatch.setattr("lerobot_robot_piper_x.piper_x.time.monotonic", lambda: next(times))
    with pytest.raises(TimeoutError):
        robot.reset_to_zero()
    assert robot.sdk.calls == []
    monkeypatch.setattr("lerobot_robot_piper_x.piper_x.time.monotonic", lambda: 0)

    def fault():
        raise ConnectionError("stale feedback")

    robot.reader.check_follower_status = fault
    with pytest.raises(ConnectionError, match="stale feedback"):
        robot.reset_to_zero()
    assert robot.sdk.calls == []
