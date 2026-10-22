from types import SimpleNamespace

import pytest

from lerobot_robot_piper_x import PiperXLeader, PiperXLeaderConfig
from lerobot_robot_piper_x.protocol import KEYS, TargetFramesUnavailable


def setup_alignment(tmp_path, monkeypatch):
    leader = PiperXLeader(PiperXLeaderConfig(
        calibration_dir=tmp_path, alignment_hold_s=0.1, alignment_timeout_s=1,
    ))
    target = dict.fromkeys(KEYS, 0.0)
    sent = []
    robot = SimpleNamespace(
        config=SimpleNamespace(recording_reset_pose=target),
        reader=SimpleNamespace(positions=lambda: target.copy()),
        send_action=lambda action: sent.append(dict(action)),
    )
    clock = [0.0]
    monkeypatch.setattr("lerobot_robot_piper_x.leader.time.monotonic", lambda: clock[0])
    monkeypatch.setattr("lerobot_robot_piper_x.leader.time.sleep",
                        lambda dt: clock.__setitem__(0, clock[0] + dt))
    return leader, robot, sent, clock


def test_waits_for_manual_alignment_and_dwell(tmp_path, monkeypatch):
    leader, robot, sent, clock = setup_alignment(tmp_path, monkeypatch)

    def position():
        pose = robot.config.recording_reset_pose.copy()
        if clock[0] < 0.2:
            pose[KEYS[0]] = 0.2
        return pose

    monkeypatch.setattr(leader, "_read_action", position)
    events = {"exit_early": True}
    assert leader.wait_for_start_alignment(robot, events, 30)
    assert clock[0] >= 0.3
    assert not events["exit_early"]
    assert sent and all(p == robot.config.recording_reset_pose for p in sent)


@pytest.mark.parametrize("axis", [KEYS[0], KEYS[-1]])
def test_misaligned_joint_or_gripper_cannot_start(tmp_path, monkeypatch, axis):
    leader, robot, sent, _ = setup_alignment(tmp_path, monkeypatch)
    monkeypatch.setattr(leader, "_read_action", lambda: {**robot.config.recording_reset_pose, axis: 0.1})
    with pytest.raises(TimeoutError, match="alignment timed out"):
        leader.wait_for_start_alignment(robot, {}, 30)
    assert sent


def test_stale_leader_waits_but_transport_fault_aborts(tmp_path, monkeypatch):
    leader, robot, _, _ = setup_alignment(tmp_path, monkeypatch)

    def stale():
        raise TargetFramesUnavailable("stale")

    monkeypatch.setattr(leader, "_read_action", stale)
    with pytest.raises(TimeoutError):
        leader.wait_for_start_alignment(robot, {}, 30)

    def fault():
        raise ConnectionError("bus fault")

    monkeypatch.setattr(leader, "_read_action", fault)
    with pytest.raises(ConnectionError, match="bus fault"):
        leader.wait_for_start_alignment(robot, {}, 30)


def test_cancel_does_not_send_or_start(tmp_path, monkeypatch):
    leader, robot, sent, _ = setup_alignment(tmp_path, monkeypatch)
    assert not leader.wait_for_start_alignment(robot, {"stop_recording": True}, 30)
    assert sent == []


def test_follower_must_also_reach_saved_pose(tmp_path, monkeypatch):
    leader, robot, _, _ = setup_alignment(tmp_path, monkeypatch)
    monkeypatch.setattr(leader, "_read_action", lambda: robot.config.recording_reset_pose.copy())
    robot.reader.positions = lambda: dict.fromkeys(KEYS, 0.1)
    with pytest.raises(TimeoutError):
        leader.wait_for_start_alignment(robot, {}, 30)


def test_wider_gripper_tolerance_accepts_95mm_leader(tmp_path, monkeypatch):
    leader, robot, _, _ = setup_alignment(tmp_path, monkeypatch)
    leader.config.alignment_gripper_tolerance_m = 0.030
    robot.config.recording_reset_pose[KEYS[-1]] = 0.0694
    monkeypatch.setattr(leader, "_read_action",
                        lambda: {**robot.config.recording_reset_pose, KEYS[-1]: 0.0956})
    assert leader.wait_for_start_alignment(robot, {}, 30)


def test_paired_reset_prepares_leader_before_hold_and_does_not_reset_twice(tmp_path, monkeypatch):
    leader, robot, _, _ = setup_alignment(tmp_path, monkeypatch)
    leader.config.automatic_reset = True
    calls = []
    robot.recording_reset = lambda events, hold_time_s, fps: calls.append(("follower", hold_time_s))
    monkeypatch.setattr("lerobot_robot_piper_x.leader_reset.reset_leader",
                        lambda *args: calls.append(("leader", 0)))
    monkeypatch.setattr(leader, "wait_for_start_alignment", lambda *args: True)
    leader.recording_reset(robot, {}, 10, 30)
    assert calls == [("follower", 0), ("leader", 0), ("follower", 10)]
    assert leader.prepare_recording(robot, {}, 30)
    assert calls == [("follower", 0), ("leader", 0), ("follower", 10)]
    # The next reset must return the leader again.
    leader.recording_reset(robot, {}, 10, 30)
    assert calls.count(("leader", 0)) == 2


def test_second_episode_waits_for_teaching_on_without_resetting_again(tmp_path, monkeypatch, caplog):
    from lerobot_robot_piper_x.leader_reset import TeachingModeEnded

    leader, robot, sent, clock = setup_alignment(tmp_path, monkeypatch)
    leader.config.automatic_reset = True
    leader.config.teaching_mode_switch = "manual"
    leader._prepared_for_capture = True

    def action():
        if clock[0] < 0.2:
            raise TeachingModeEnded("0200010201000000")
        return robot.config.recording_reset_pose.copy()

    monkeypatch.setattr(leader, "_read_action", action)

    def unwanted_reset(*args):
        pytest.fail("Already prepared leader must not reset again")

    monkeypatch.setattr("lerobot_robot_piper_x.leader_reset.reset_leader", unwanted_reset)
    assert leader.prepare_recording(robot, {}, 30)
    assert clock[0] >= 0.3  # Fresh dwell after teaching ON.
    assert "teaching is OFF before capture" in caplog.text
    assert sent and all(p == robot.config.recording_reset_pose for p in sent)


def test_teaching_off_wait_can_cancel(tmp_path, monkeypatch):
    from lerobot_robot_piper_x.leader_reset import TeachingModeEnded

    leader, robot, _, clock = setup_alignment(tmp_path, monkeypatch)
    leader.config.automatic_reset = True
    leader.config.teaching_mode_switch = "manual"
    events = {}

    def action():
        if clock[0] >= 0.1:
            events["stop_recording"] = True
        raise TeachingModeEnded("off")

    monkeypatch.setattr(leader, "_read_action", action)
    assert not leader.wait_for_start_alignment(robot, events, 30)


def test_teaching_off_wait_still_times_out(tmp_path, monkeypatch):
    from lerobot_robot_piper_x.leader_reset import TeachingModeEnded

    leader, robot, _, _ = setup_alignment(tmp_path, monkeypatch)
    leader.config.automatic_reset = True
    leader.config.teaching_mode_switch = "manual"

    def action():
        raise TeachingModeEnded("off")

    monkeypatch.setattr(leader, "_read_action", action)
    with pytest.raises(TimeoutError, match="turn teaching ON"):
        leader.wait_for_start_alignment(robot, {}, 30)


def test_zero_environment_hold_still_resets_both_arms(tmp_path, monkeypatch):
    leader, robot, _, _ = setup_alignment(tmp_path, monkeypatch)
    leader.config.automatic_reset = True
    calls = []
    robot.recording_reset = lambda events, hold_time_s, fps: calls.append(("follower", hold_time_s))
    monkeypatch.setattr("lerobot_robot_piper_x.leader_reset.reset_leader",
                        lambda *args: calls.append(("leader", 0)))
    monkeypatch.setattr(leader, "wait_for_start_alignment", lambda *args: True)
    leader.recording_reset(robot, {}, 0, 30)
    assert calls == [("follower", 0), ("leader", 0)]
    assert leader._prepared_for_capture
