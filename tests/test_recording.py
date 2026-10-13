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
