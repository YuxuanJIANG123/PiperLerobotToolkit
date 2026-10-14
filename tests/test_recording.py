"""Exercise the actual LeRobot recorder and v3 dataset writer with fake arms."""

import numpy as np
import pyarrow.parquet as pq
import pytest
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.processor import make_default_processors
from lerobot.scripts.lerobot_record import record_loop
from test_piper import FakeReader, FakeSDK

from lerobot_robot_piper_x import PiperX, PiperXConfig, PiperXLeader, PiperXLeaderConfig
from lerobot_robot_piper_x.protocol import KEYS, TargetFramesUnavailable


@pytest.mark.parametrize("silent", [False, True])
def test_recorded_action_is_transmitted_target(tmp_path, silent):
    robot = PiperX(PiperXConfig(calibration_dir=tmp_path / "calibration"))
    robot.reader, robot.sdk, robot._connected = FakeReader(), FakeSDK(), True
    leader = PiperXLeader(PiperXLeaderConfig(calibration_dir=tmp_path / "calibration"))
    leader.reader = FakeReader()
    leader.reader.positions = lambda: dict.fromkeys(KEYS, 1.0)
    if silent:
        def unavailable():
            raise TargetFramesUnavailable("stale leader targets")
        leader.reader.positions = unavailable
    features = {
        "observation.state": {"dtype": "float32", "shape": (7,), "names": list(KEYS)},
        "action": {"dtype": "float32", "shape": (7,), "names": list(KEYS)},
    }
    root = tmp_path / "dataset"
    dataset = LeRobotDataset.create(
        "local/test_piper_x", fps=30, features=features, root=root, robot_type="piper_x", use_videos=False
    )
    teleop_proc, action_proc, obs_proc = make_default_processors()
    try:
        record_loop(
            robot=robot,
            events={"exit_early": False},
            fps=30,
            teleop_action_processor=teleop_proc,
            robot_action_processor=action_proc,
            robot_observation_processor=obs_proc,
            dataset=dataset,
            teleop=leader,
            control_time_s=0.08,
            single_task="test",
        )
        dataset.save_episode()
    finally:
        dataset.finalize()
        robot.disconnect()
        leader.disconnect()
    table = pq.read_table(next((root / "data").rglob("*.parquet")))
    actions = np.asarray(table["action"].to_pylist())
    observations = np.asarray(table["observation.state"].to_pylist())
    assert len(actions) >= 1
    if silent:
        assert np.all(actions == 0)
    else:
        assert np.all(actions[:, :6] > 0)
        assert np.allclose(actions[:, 6], 0.001)
    assert np.all(actions[:, :6] <= 0.003)
    assert np.all(observations == 0)
    assert dataset.meta.info.codebase_version == "v3.0"


def test_unrecorded_reset_uses_saved_pose_not_leader(tmp_path, monkeypatch):
    import lerobot_robot_piper_x.piper_x as module

    target = dict.fromkeys(KEYS, 0.02)
    robot = PiperX(PiperXConfig(calibration_dir=tmp_path, recording_reset_pose=target))
    robot.reader, robot.sdk, robot._connected = FakeReader(), FakeSDK(), True
    present = dict.fromkeys(KEYS, 0.0)
    robot.reader.positions = lambda: present.copy()
    commands = []
    original_send = robot.send_action

    def send(action):
        sent = original_send(action)
        commands.append(sent)
        present.update(sent)
        return sent

    robot.send_action = send
    clock = [0.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(module.time, "sleep", lambda dt: clock.__setitem__(0, clock[0] + dt))
    teleop_proc, action_proc, obs_proc = make_default_processors()
    record_loop(
        robot=robot, events={"exit_early": False}, fps=30,
        teleop_action_processor=teleop_proc, robot_action_processor=action_proc,
        robot_observation_processor=obs_proc, control_time_s=0.1,
    )
    assert len(commands) > 1
    assert abs(present[KEYS[0]] - target[KEYS[0]]) <= 0.005
    assert abs(present[KEYS[-1]] - target[KEYS[-1]]) <= 0.001
    assert commands[0][KEYS[0]] <= robot.config.max_joint_step_rad
    assert commands[0][KEYS[-1]] <= robot.config.max_gripper_step_m
    robot.disconnect()


def test_recording_reset_timeout_and_stop(tmp_path, monkeypatch):
    import lerobot_robot_piper_x.piper_x as module

    robot = PiperX(PiperXConfig(
        calibration_dir=tmp_path, recording_reset_pose=dict.fromkeys(KEYS, 0.02),
        recording_reset_timeout_s=0.05,
    ))
    robot.reader, robot.sdk, robot._connected = FakeReader(), FakeSDK(), True
    clock = [0.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(module.time, "sleep", lambda dt: clock.__setitem__(0, clock[0] + dt))
    robot.recording_reset({"stop_recording": True}, 10, 30)
    assert robot.sdk.calls == []
    with pytest.raises(TimeoutError, match="Recording reset timed out"):
        robot.recording_reset({"exit_early": True}, 10, 30)
    robot.disconnect()


def test_recording_reset_rejects_invalid_pose():
    with pytest.raises(ValueError, match="recording_reset_pose"):
        PiperXConfig(recording_reset_pose={})
    with pytest.raises(ValueError, match="outside the configured limits"):
        PiperXConfig(recording_reset_pose=dict.fromkeys(KEYS, float("nan")))
