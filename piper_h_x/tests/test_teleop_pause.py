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
