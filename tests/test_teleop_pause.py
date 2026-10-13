from lerobot.processor import make_default_processors
from lerobot.scripts.lerobot_teleoperate import teleop_loop
from test_piper import FakeReader, FakeSDK

from lerobot_robot_piper_x import PiperX, PiperXConfig, PiperXLeader, PiperXLeaderConfig
from lerobot_robot_piper_x.protocol import TargetFramesUnavailable


def test_actual_teleop_loop_holds_on_silent_leader(tmp_path):
    robot = PiperX(PiperXConfig(calibration_dir=tmp_path))
    robot.reader, robot.sdk, robot._connected = FakeReader(), FakeSDK(), True
    leader = PiperXLeader(PiperXLeaderConfig(calibration_dir=tmp_path))
    leader.reader = FakeReader()

    def unavailable():
        raise TargetFramesUnavailable("stale targets")

    leader.reader.positions = unavailable
    teleop_proc, action_proc, obs_proc = make_default_processors()
    try:
        teleop_loop(leader, robot, 30, teleop_proc, action_proc, obs_proc, duration=0.08)
        joints = [args for name, args in robot.sdk.calls if name == "joints"]
        assert joints
        assert all(args == (0, 0, 0, 0, 0, 0) for args in joints)
    finally:
        robot.disconnect()
        leader.disconnect()


def test_feedback_leader_pauses_and_requires_all_frames_to_resume(tmp_path, monkeypatch):
    import time

    from test_piper import feed

    from lerobot_robot_piper_x.protocol import FEEDBACK_IDS, KEYS, PassiveReader

    leader = PiperXLeader(PiperXLeaderConfig(calibration_dir=tmp_path, source="feedback"))
    monkeypatch.setattr(PassiveReader, "is_connected", property(lambda self: True))
    assert leader.reader.ids == FEEDBACK_IDS
    feed(leader.reader)
    observation = dict.fromkeys(KEYS, 0.0)
    assert leader.get_action_for_observation(observation) != observation
    frame_id = FEEDBACK_IDS[0]
    payload, _ = leader.reader.frames[frame_id]
    leader.reader.frames[frame_id] = (payload, time.monotonic() - 10)
    assert leader.get_action_for_observation(observation) == observation
    # A partial refresh must retain the original hold even if the follower drifts.
    leader.reader.frames[frame_id] = (payload, time.monotonic())
    drifted = dict.fromkeys(KEYS, 0.001)
    assert leader.get_action_for_observation(drifted) == observation
    feed(leader.reader)
    assert leader.get_action_for_observation(drifted) == leader.reader.positions()
