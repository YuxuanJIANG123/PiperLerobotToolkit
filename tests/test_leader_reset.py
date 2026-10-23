import sys
import threading
from types import SimpleNamespace

import pytest

from lerobot_robot_piper_x import PiperXLeader, PiperXLeaderConfig
from lerobot_robot_piper_x.leader_reset import require_teaching, reset_leader
from lerobot_robot_piper_x.protocol import FEEDBACK_IDS, GRIPPER_FACTOR, JOINT_FACTOR, KEYS
from lerobot_robot_piper_x.safety import GRIPPER_FEEDBACK_TOLERANCE_M


def simulation(tmp_path, monkeypatch, reject=None):
    clock = [1.0]
    pose = dict.fromkeys(KEYS, 0.02)
    target = dict.fromkeys(KEYS, 0.0)
    target[KEYS[-1]] = 0.01
    status = bytearray([2, 0x0B, 1, 1, 0, 0, 0, 0])
    calls, follower_calls = [], []
    leader = PiperXLeader(PiperXLeaderConfig(
        calibration_dir=tmp_path, automatic_reset=True, require_start_alignment=True,
        debug_motion=True,
        reset_mode_timeout_s=0.2, reset_timeout_s=2,
    ))
    reader = SimpleNamespace(
        error=None, lock=threading.Lock(), frames={}, timeout_s=0.5,
        ids=FEEDBACK_IDS, positions=lambda: pose.copy(), is_connected=True,
        simulated_status=status,
    )
    leader.reader = reader

    def tick(dt):
        clock[0] += dt
        reader.frames[0x2A1] = (bytes(status), clock[0])
        for frame_id in FEEDBACK_IDS:
            reader.frames[frame_id] = (bytes(8), clock[0])

    tick(0)
    monkeypatch.setattr("lerobot_robot_piper_x.leader_reset.time.monotonic", lambda: clock[0])
    monkeypatch.setattr("lerobot_robot_piper_x.leader_reset.time.sleep", tick)

    class SDK:
        def ConnectPort(self, **kwargs):
            assert kwargs == {"piper_init": False}
            calls.append("connect")

        def DisconnectPort(self):
            calls.append("disconnect")

        def JointCtrl(self, *raw):
            values = dict(zip(KEYS[:6], (v / JOINT_FACTOR for v in raw)))
            assert all(lo <= values[k] <= hi for k, (lo, hi)
                       in zip(KEYS[:6], leader.config.reset_joint_limits_rad))
            assert all(abs(values[k] - pose[k]) <= leader.config.reset_max_joint_step_rad + 1e-12
                       for k in KEYS[:6])
            pose.update(values)
            calls.append("joints")

        def GripperCtrl(self, raw, effort, enabled, zero):
            assert enabled == 1 and zero == 0
            value = raw / GRIPPER_FACTOR
            assert 0 <= value <= leader.config.reset_gripper_max_m
            assert abs(value - pose[KEYS[-1]]) <= max(leader.config.reset_max_gripper_step_m,
                        max(0.0, -pose[KEYS[-1]], pose[KEYS[-1]] - leader.config.reset_gripper_max_m)) + 1e-12
            pose[KEYS[-1]] = value
            calls.append("gripper")

        def MotionCtrl_1(self, emergency, track, teaching):
            assert emergency == track == 0
            calls.append(f"teach:{teaching}")
            if teaching == 2 and reject != "exit":
                status[1], status[3] = 0, 2
            if teaching == 1 and reject != "teach":
                status[0], status[1], status[3] = 2, 0x0B, 1

        def MotionCtrl_2(self, mode, joint_mode, speed, high_follow):
            assert joint_mode == 1 and speed == 10 and high_follow == 0
            calls.append(f"mode:{mode}")
            if reject != "can" or mode != 1:
                status[0], status[1], status[2] = mode, 0, 1

        def EnablePiper(self):
            calls.append("enable")
            return reject != "enable"

    monkeypatch.setitem(sys.modules, "piper_sdk", SimpleNamespace(C_PiperInterface_V2=lambda _: SDK()))
    follower = SimpleNamespace(
        config=SimpleNamespace(can_name="piper_follower", recording_reset_pose=target),
        send_action=lambda action: follower_calls.append(dict(action)),
    )
    return leader, follower, calls, follower_calls, pose, clock, tick


def test_reset_handoff_and_bounded_motion(tmp_path, monkeypatch, caplog):
    leader, follower, calls, follower_calls, pose, _, _ = simulation(tmp_path, monkeypatch)
    with caplog.at_level("INFO", logger="lerobot_robot_piper_x.leader_reset"):
        reset_leader(leader, follower, {}, 30)
    assert "Leader reset progress" in caplog.text
    assert "remaining" in caplog.text
    assert "Leader reached saved pose" in caplog.text
    assert calls.index("joints") < calls.index("teach:2") < calls.index("mode:0")
    assert calls.index("mode:0") < calls.index("mode:1") < calls.index("enable") < calls.index("teach:1")
    assert calls[-1] == "disconnect"
    assert calls.count("gripper") > 1
    assert leader._teaching_required
    require_teaching(leader.reader)
    assert all(abs(pose[k] - follower.config.recording_reset_pose[k]) <= (0.005 if i < 6 else 0.001)
               for i, k in enumerate(KEYS))
    assert follower_calls and all(p == follower.config.recording_reset_pose for p in follower_calls)


@pytest.mark.parametrize("reject", ["exit", "can", "enable", "teach"])
def test_failed_transition_never_prepares_capture(tmp_path, monkeypatch, reject):
    leader, follower, calls, _, _, _, _ = simulation(tmp_path, monkeypatch, reject)
    with pytest.raises(TimeoutError):
        reset_leader(leader, follower, {}, 30)
    assert not leader._teaching_required
    assert calls[-1] == "disconnect"
    if reject in ("exit", "can"):
        assert "enable" not in calls
        assert "gripper" not in calls


def test_cancel_and_missing_feedback_do_not_move(tmp_path, monkeypatch):
    leader, follower, calls, _, _, _, _ = simulation(tmp_path, monkeypatch)
    assert not leader.prepare_recording(follower, {"stop_recording": True}, 30)
    assert "joints" not in calls
    calls.clear()
    leader.reader.frames.clear()
    with pytest.raises(ConnectionError, match="status"):
        reset_leader(leader, follower, {}, 30)
    assert calls == []


def test_independent_leader_limits_refuse_out_of_range_pose(tmp_path, monkeypatch):
    leader, follower, calls, _, pose, _, _ = simulation(tmp_path, monkeypatch)
    pose[KEYS[0]] = 2.0
    with pytest.raises(ValueError, match="outside"):
        reset_leader(leader, follower, {}, 30)
    assert calls == []


def test_teaching_loss_or_stale_status_stops_capture_actions(tmp_path, monkeypatch):
    leader, follower, _, _, _, clock, _ = simulation(tmp_path, monkeypatch)
    reset_leader(leader, follower, {}, 30)
    assert leader.get_action()
    leader.reader.frames[0x2A1] = (bytes([1, 0, 1, 2, 0, 0, 0, 0]), clock[0])
    with pytest.raises(ConnectionError, match="teaching"):
        leader.get_action()
    clock[0] += 1
    with pytest.raises(ConnectionError, match="stale"):
        leader.get_action()


def test_automatic_reset_requires_feedback_and_alignment():
    with pytest.raises(ValueError, match="automatic_reset"):
        PiperXLeaderConfig(automatic_reset=True)
    with pytest.raises(ValueError, match="automatic_reset"):
        PiperXLeaderConfig(automatic_reset=True, require_start_alignment=True, source="control")


def test_stalled_motion_never_enters_teaching(tmp_path, monkeypatch):
    leader, follower, calls, _, _, _, _ = simulation(tmp_path, monkeypatch)
    sdk = sys.modules["piper_sdk"].C_PiperInterface_V2(leader.config.can_name)
    sdk.JointCtrl = lambda *raw: calls.append("joints")
    sys.modules["piper_sdk"].C_PiperInterface_V2 = lambda _: sdk
    with pytest.raises(TimeoutError, match=r"saved pose timed out: joint_1.pos: remaining"):
        reset_leader(leader, follower, {}, 30)
    assert "teach:1" not in calls
    assert calls[-1] == "disconnect"


def test_cancel_during_return_does_not_attempt_teaching(tmp_path, monkeypatch):
    leader, follower, calls, _, _, clock, tick = simulation(tmp_path, monkeypatch)
    events = {}

    def cancel(dt):
        tick(dt)
        if clock[0] > 1.11:
            events["stop_recording"] = True

    monkeypatch.setattr("lerobot_robot_piper_x.leader_reset.time.sleep", cancel)
    assert not leader.prepare_recording(follower, events, 30)
    assert "teach:1" not in calls
    assert calls[-1] == "disconnect"


@pytest.mark.parametrize("opening", [-0.002, 0.072])
def test_small_gripper_endpoint_offset_can_reset_with_bounded_targets(tmp_path, monkeypatch, opening):
    leader, follower, calls, _, pose, _, _ = simulation(tmp_path, monkeypatch)
    pose[KEYS[-1]] = opening
    # Opening from 72 mm to 10 mm takes more than the short simulated timeout.
    leader.config.reset_timeout_s = 5
    reset_leader(leader, follower, {}, 30)
    assert "gripper" in calls
    assert leader._teaching_required


@pytest.mark.parametrize("opening", [-GRIPPER_FEEDBACK_TOLERANCE_M - 0.0001,
                                     0.07 + GRIPPER_FEEDBACK_TOLERANCE_M + 0.0001,
                                     float("nan"), float("inf")])
def test_large_or_nonfinite_gripper_offset_still_refused(tmp_path, monkeypatch, opening):
    leader, follower, calls, _, pose, _, _ = simulation(tmp_path, monkeypatch)
    pose[KEYS[-1]] = opening
    with pytest.raises(ValueError):
        reset_leader(leader, follower, {}, 30)
    assert calls == []


def test_negative_saved_target_is_not_allowed(tmp_path, monkeypatch):
    leader, follower, calls, _, _, _, _ = simulation(tmp_path, monkeypatch)
    follower.config.recording_reset_pose[KEYS[-1]] = -0.0007
    with pytest.raises(ValueError):
        reset_leader(leader, follower, {}, 30)
    assert calls == []


def test_manual_buttons_wait_without_sending_teaching_commands(tmp_path, monkeypatch, caplog):
    leader, follower, calls, _, _, _, tick = simulation(tmp_path, monkeypatch)
    leader.config.teaching_mode_switch = "manual"
    status = leader.reader.simulated_status
    leader.config.teaching_button_timeout_s = 1

    def button_tick(dt):
        if "reset complete" in caplog.text:
            status[0], status[1], status[3] = 2, 0, 1
        elif not any(c.startswith("mode:") for c in calls):
            # No joint targets or motor enable may be sent while waiting for OFF.
            assert "joints" not in calls and "enable" not in calls
            status[0], status[1], status[3] = 2, 0, 2
        tick(dt)

    monkeypatch.setattr("lerobot_robot_piper_x.leader_reset.time.sleep", button_tick)
    with caplog.at_level("WARNING"):
        reset_leader(leader, follower, {}, 30)
    assert "turn OFF" in caplog.text and "turn ON" in caplog.text
    assert not any(c.startswith("teach:") for c in calls)
    assert leader._teaching_required


def test_configured_tolerance_accepts_observed_wrist_errors(tmp_path, monkeypatch):
    leader, follower, calls, _, pose, _, _ = simulation(tmp_path, monkeypatch)
    leader.config.reset_joint_tolerance_rad = 0.061086523819801536
    target = follower.config.recording_reset_pose
    pose.update(target)
    target[KEYS[4]], target[KEYS[5]] = -0.04091051766674708, 0.05173155902911193
    reset_leader(leader, follower, {}, 30)
    assert leader._teaching_required
    assert "gripper" not in calls
    assert pose[KEYS[4]] == pose[KEYS[5]] == 0


def test_reset_tolerance_must_fit_alignment():
    with pytest.raises(ValueError, match="alignment tolerances"):
        PiperXLeaderConfig(automatic_reset=True, require_start_alignment=True,
                           reset_joint_tolerance_rad=0.061, alignment_joint_tolerance_rad=0.05)


@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf")])
def test_invalid_reset_tolerance(value):
    with pytest.raises(ValueError):
        PiperXLeaderConfig(reset_joint_tolerance_rad=value)


def test_manual_off_timeout_sends_no_leader_motion(tmp_path, monkeypatch):
    leader, follower, calls, _, _, _, _ = simulation(tmp_path, monkeypatch)
    leader.config.teaching_mode_switch = "manual"
    leader.config.teaching_button_timeout_s = 0.2
    with pytest.raises(TimeoutError, match="teaching OFF"):
        reset_leader(leader, follower, {}, 30)
    assert calls == ["connect", "disconnect"]


def test_error_beyond_configured_tolerance_still_times_out(tmp_path, monkeypatch):
    leader, follower, calls, _, pose, _, _ = simulation(tmp_path, monkeypatch)
    leader.config.reset_joint_tolerance_rad = 0.061086523819801536
    pose[KEYS[5]] = 0.08
    sdk = sys.modules["piper_sdk"].C_PiperInterface_V2(leader.config.can_name)
    sdk.JointCtrl = lambda *raw: calls.append("joints")
    sys.modules["piper_sdk"].C_PiperInterface_V2 = lambda _: sdk
    with pytest.raises(TimeoutError, match="joint_6.pos"):
        reset_leader(leader, follower, {}, 30)
    assert "teach:1" not in calls


def test_wide_leader_gripper_range_and_start_tolerance(tmp_path, monkeypatch):
    leader, follower, calls, _, pose, _, _ = simulation(tmp_path, monkeypatch)
    leader.config.reset_gripper_max_m = 0.10
    leader.config.reset_gripper_tolerance_m = 0.030
    leader.config.alignment_gripper_tolerance_m = 0.030
    follower.config.recording_reset_pose[KEYS[-1]] = 0.0694
    pose.update(follower.config.recording_reset_pose)
    pose[KEYS[-1]] = 0.0956
    reset_leader(leader, follower, {}, 30)
    assert leader._teaching_required
    assert "gripper" not in calls


def test_quiet_reset_keeps_button_prompts_without_motion_dump(tmp_path, monkeypatch, caplog):
    leader, follower, _, _, _, _, _ = simulation(tmp_path, monkeypatch)
    leader.config.debug_motion = False
    with caplog.at_level("INFO", logger="lerobot_robot_piper_x.leader_reset"):
        reset_leader(leader, follower, {}, 30)
    assert "Leader reset progress" not in caplog.text
    assert "measured(deg/mm)" not in caplog.text
    assert "Leader reached saved pose" in caplog.text


@pytest.mark.parametrize("upper", [False, True])
def test_tiny_joint_endpoint_overrun_resets_with_in_range_seed(tmp_path, monkeypatch, upper):
    import math

    leader, follower, calls, _, pose, _, _ = simulation(tmp_path, monkeypatch)
    lo, hi = leader.config.reset_joint_limits_rad[1]
    pose[KEYS[1]] = hi + math.radians(0.002) if upper else lo - math.radians(0.002)
    follower.config.recording_reset_pose[KEYS[1]] = hi - 0.02 if upper else lo + 0.02
    reset_leader(leader, follower, {}, 30)
    assert leader._teaching_required and "joints" in calls


def test_joint_endpoint_excursion_beyond_feedback_tolerance_is_rejected(tmp_path, monkeypatch):
    import math

    leader, follower, calls, _, pose, _, _ = simulation(tmp_path, monkeypatch)
    pose[KEYS[1]] = leader.config.reset_joint_limits_rad[1][1] + math.radians(0.02)
    with pytest.raises(ValueError, match="outside"):
        reset_leader(leader, follower, {}, 30)
    assert calls == []
