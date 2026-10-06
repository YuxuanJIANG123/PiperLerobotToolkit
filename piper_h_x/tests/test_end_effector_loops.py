"""Exercise Cartesian mapping through the installed LeRobot loops without motors."""

import math

import pytest
from lerobot.processor import make_default_processors
from lerobot.scripts.lerobot_record import record_loop
from lerobot.scripts.lerobot_teleoperate import teleop_loop
from test_end_effector import Q, action
from test_piper import FakeReader, FakeSDK

from lerobot_robot_piper_x import PiperX, PiperXConfig, PiperXLeader, PiperXLeaderConfig
from lerobot_robot_piper_x.end_effector import EndEffectorConfig
from lerobot_robot_piper_x.kinematics import ArmKinematics
from lerobot_robot_piper_x.protocol import JOINT_FACTOR


@pytest.mark.parametrize("recording", [False, True])
def test_eef_targets_pass_through_real_loop_and_joint_limiter(tmp_path, recording):
    robot = PiperX(
        PiperXConfig(calibration_dir=tmp_path, joint_limits_rad=ArmKinematics("piper_x").limits.tolist())
    )
    robot.reader, robot.sdk, robot._connected = FakeReader(), FakeSDK(), True
    robot.reader.positions = lambda: action()
    leader = PiperXLeader(
        PiperXLeaderConfig(
            calibration_dir=tmp_path,
            control_mode="end_effector",
            eef=EndEffectorConfig(frames_verified=True, max_solve_time_s=1),
        )
    )
    leader.reader = FakeReader()
    calls = 0

    def positions():
        nonlocal calls
        calls += 1
        return action(Q if calls == 1 else Q + 0.02)

    leader.reader.positions = positions
    teleop_proc, action_proc, obs_proc = make_default_processors()
    try:
        if recording:
            record_loop(
                robot=robot,
                events={"exit_early": False},
                fps=30,
                teleop_action_processor=teleop_proc,
                robot_action_processor=action_proc,
                robot_observation_processor=obs_proc,
                teleop=leader,
                control_time_s=0.12,
            )
        else:
            teleop_loop(leader, robot, 30, teleop_proc, action_proc, obs_proc, duration=0.12)
        joint_targets = [args for name, args in robot.sdk.calls if name == "joints"]
        assert len(joint_targets) >= 2
        for target in joint_targets:
            for raw, initial in zip(target, Q, strict=True):
                assert abs(raw / JOINT_FACTOR - initial) <= robot.config.max_joint_step_rad + 1e-12
                assert math.isfinite(raw)
        assert joint_targets[-1] != joint_targets[0]
        assert leader._eef_mapper.diagnostics["status"] == "tracking"
    finally:
        robot.disconnect()
        leader.disconnect()
