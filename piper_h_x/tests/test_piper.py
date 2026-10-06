import math
import struct
import time
from types import SimpleNamespace

import can
import pytest

from lerobot_robot_piper_x import PiperX, PiperXConfig
from lerobot_robot_piper_x.protocol import CONTROL_IDS, FEEDBACK_IDS, JOINT_FACTOR, KEYS, PassiveReader
from lerobot_robot_piper_x.safety import limit_action, validate_initial_pose
from lerobot_teleoperator_piper_x import PiperXLeader, PiperXLeaderConfig


def feed(reader, ids=FEEDBACK_IDS, raw=(1000, -2000, 3000, -4000, 5000, -6000), gripper=25000):
    for index, can_id in enumerate(ids[:3]):
        reader.ingest(
            can.Message(
                arbitration_id=can_id,
                is_extended_id=False,
                data=struct.pack(">ii", *raw[index * 2 : index * 2 + 2]),
            )
        )
    reader.ingest(
        can.Message(arbitration_id=ids[-1], is_extended_id=False, data=struct.pack(">i", gripper) + b"\0" * 4)
    )


def test_decode_si_units():
    reader = PassiveReader("test")
    feed(reader)
    positions = reader.positions()
    assert list(positions) == list(KEYS)
    assert positions[KEYS[0]] == pytest.approx(math.radians(1))
    assert positions[KEYS[1]] == pytest.approx(math.radians(-2))
    assert positions[KEYS[-1]] == 0.025


def test_each_joint_pair_must_arrive_and_remain_fresh():
    reader = PassiveReader("test")
    feed(reader)
    del reader.frames[0x2A6]
    with pytest.raises(ConnectionError, match="missing frames 0x2a6"):
        reader.positions()
    feed(reader)
    data, _ = reader.frames[0x2A6]
    reader.frames[0x2A6] = (data, time.monotonic() - 5)
    with pytest.raises(ConnectionError, match="stale frames 0x2a6"):
        reader.positions()


def test_stationary_leader_requires_live_heartbeat():
    reader = PassiveReader("test", source="control", timeout_s=3)
    feed(reader, CONTROL_IDS)
    reader.frames = {key: (data, time.monotonic() - 10) for key, (data, _) in reader.frames.items()}
    with pytest.raises(ConnectionError):
        reader.positions()
    reader.ingest(can.Message(arbitration_id=0x151, is_extended_id=False, data=bytes(8)))
    assert len(reader.positions()) == 7
    del reader.frames[0x159]
    with pytest.raises(ConnectionError, match="missing frames 0x159"):
        reader.positions()


def test_error_frame_invalidates_reader():
    reader = PassiveReader("test")
    feed(reader)
    reader.ingest(can.Message(is_error_frame=True))
    with pytest.raises(ConnectionError, match="CAN reader failed"):
        reader.positions()


def test_queued_feedback_cannot_masquerade_as_fresh():
    reader = PassiveReader("test")
    feed(reader)
    reader.ingest(can.Message(arbitration_id=0x2A5, is_extended_id=False,
                             timestamp=time.time() - 2, data=bytes(8)))
    with pytest.raises(ConnectionError, match="stale frames 0x2a5"):
        reader.positions()
    assert reader.timing()["max_receive_delay_ms"] >= 1990


def test_new_kernel_timestamp_reports_low_receive_delay():
    reader = PassiveReader("test")
    feed(reader)
    reader.ingest(can.Message(arbitration_id=0x2A5, is_extended_id=False,
                             timestamp=time.time(), data=bytes(8)))
    assert len(reader.positions()) == 7
    assert reader.timing()["max_receive_delay_ms"] < 100


def test_limits_and_quantization():
    config = PiperXConfig()
    present = dict.fromkeys(KEYS, 0.0)
    action = dict.fromkeys(KEYS, 100.0)
    sent = limit_action(action, present, config)
    for key in KEYS[:6]:
        assert 0 < sent[key] <= config.max_joint_step_rad
        assert sent[key] * JOINT_FACTOR == pytest.approx(round(sent[key] * JOINT_FACTOR))
    assert sent[KEYS[-1]] == config.max_gripper_step_m
    assert action[KEYS[0]] == 100  # Caller input is not mutated.


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_targets_rejected(bad):
    present = dict.fromkeys(KEYS, 0.0)
    action = {**present, KEYS[0]: bad}
    with pytest.raises(ValueError, match="Non-finite"):
        limit_action(action, present, PiperXConfig())


def test_invalid_schema_and_out_of_envelope_feedback():
    present = dict.fromkeys(KEYS, 0.0)
    with pytest.raises(ValueError, match="exactly"):
        limit_action({}, present, PiperXConfig())
    present[KEYS[0]] = 3.0
    with pytest.raises(ValueError, match="outside"):
        limit_action(present, present, PiperXConfig())


def test_initial_pose_reports_all_outside_axes():
    present = dict.fromkeys(KEYS, 0.0)
    present["joint_2.pos"] = math.radians(-2.417)
    present["joint_3.pos"] = math.radians(2.599)
    with pytest.raises(ValueError) as error:
        validate_initial_pose(present, PiperXConfig())
    message = str(error.value)
    assert "joint_2.pos: measured -2.417 deg" in message
    assert "joint_3.pos: measured 2.599 deg" in message


def test_invalid_startup_pose_does_not_open_cameras_or_sdk(tmp_path):
    robot = PiperX(PiperXConfig(calibration_dir=tmp_path, enable_motors=True))
    current = dict.fromkeys(KEYS, 0.0)
    current["joint_2.pos"] = -1
    robot.reader = SimpleNamespace(
        connect=lambda: None, wait_ready=lambda _: current, disconnect=lambda: None
    )
    calls = []
    robot.cameras = {"test": SimpleNamespace(is_connected=False, connect=lambda: calls.append("camera"))}
    with pytest.raises(ValueError, match="Cannot enable"):
        robot.connect()
    assert calls == []
    assert robot.sdk is None


@pytest.mark.parametrize(
    "kwargs",
    [
        {"max_joint_step_rad": 0},
        {"feedback_timeout_s": float("nan")},
        {"motion_speed": 101},
        {"joint_limits_rad": [[0, 1]]},
    ],
)
def test_invalid_config(kwargs):
    with pytest.raises(ValueError):
        PiperXConfig(**kwargs)


def test_discovery_and_disconnected_features(tmp_path):
    from lerobot.robots.utils import make_robot_from_config
    from lerobot.teleoperators.utils import make_teleoperator_from_config
    from lerobot.utils.import_utils import register_third_party_plugins

    register_third_party_plugins()
    robot = make_robot_from_config(PiperXConfig(calibration_dir=tmp_path))
    leader = make_teleoperator_from_config(PiperXLeaderConfig(calibration_dir=tmp_path))
    assert isinstance(robot, PiperX)
    assert isinstance(leader, PiperXLeader)
    assert robot.action_features == robot.observation_features == leader.action_features
    assert robot.sdk is None
    assert not robot.is_connected
    with pytest.raises(ConnectionError):
        robot.send_action(dict.fromkeys(KEYS, 0.0))


class FakeReader:
    is_connected = True

    def positions(self):
        return dict.fromkeys(KEYS, 0.0)

    def check_follower_status(self):
        pass

    def disconnect(self):
        self.is_connected = False


class FakeSDK:
    def __init__(self):
        self.calls = []

    def MotionCtrl_2(self, *args):
        self.calls.append(("mode", args))

    def JointCtrl(self, *args):
        self.calls.append(("joints", args))

    def GripperCtrl(self, *args):
        self.calls.append(("gripper", args))

    def DisconnectPort(self):
        self.calls.append(("disconnect", ()))


def test_send_reports_exact_transmitted_target_and_disconnect_does_not_move(tmp_path):
    robot = PiperX(PiperXConfig(calibration_dir=tmp_path))
    sdk = FakeSDK()
    robot.reader, robot.sdk, robot._connected = FakeReader(), sdk, True
    sent = robot.send_action(dict.fromkeys(KEYS, 1.0))
    assert sdk.calls[0] == ("mode", (1, 1, 10, 0))
    assert sdk.calls[1][1] == tuple(round(sent[k] * JOINT_FACTOR) for k in KEYS[:6])
    assert sdk.calls[2][1][0] == round(sent[KEYS[-1]] * 1_000_000)
    robot.disconnect()
    assert sdk.calls[-1][0] == "disconnect"
    assert len(sdk.calls) == 4


def test_stale_feedback_prevents_all_writes(tmp_path):
    robot = PiperX(PiperXConfig(calibration_dir=tmp_path))
    sdk = FakeSDK()

    def stale():
        raise ConnectionError("stale")

    robot.reader = SimpleNamespace(
        is_connected=True, positions=stale, disconnect=lambda: None, check_follower_status=lambda: None
    )
    robot.sdk, robot._connected = sdk, True
    with pytest.raises(ConnectionError, match="stale"):
        robot.send_action(dict.fromkeys(KEYS, 0.0))
    assert sdk.calls == []
    robot.disconnect()


def test_faulted_or_stale_follower_status_is_rejected():
    reader = PassiveReader("test")
    with pytest.raises(ConnectionError, match="missing or stale"):
        reader.check_follower_status()
    healthy = bytes([1, 0, 1, 0, 0, 0, 0, 0])
    reader.ingest(can.Message(arbitration_id=0x2A1, is_extended_id=False, data=healthy))
    reader.check_follower_status()
    reader.ingest(
        can.Message(arbitration_id=0x2A1, is_extended_id=False, data=bytes([1, 0, 1, 0, 0, 0, 0, 1]))
    )
    with pytest.raises(ConnectionError, match="healthy"):
        reader.check_follower_status()


def test_silent_leader_pauses_at_follower_and_requires_all_new_targets():
    leader = PiperXLeader(PiperXLeaderConfig())
    reader = leader.reader
    reader.bus = object()
    reader.thread = SimpleNamespace(is_alive=lambda: True)
    feed(reader, CONTROL_IDS)
    reader.frames = {k: (data, time.monotonic() - 10) for k, (data, _) in reader.frames.items()}
    follower = dict.fromkeys(KEYS, 0.01)
    assert leader.get_action_for_observation(follower) == follower
    # A heartbeat alone must not resume a pre-pause target.
    reader.ingest(can.Message(arbitration_id=0x151, is_extended_id=False, data=bytes(8)))
    assert leader.get_action_for_observation(dict.fromkeys(KEYS, 0.02)) == follower
    # Nor may a partial target refresh resume a mixture of old and new targets.
    reader.ingest(can.Message(arbitration_id=0x155, is_extended_id=False, data=bytes(8)))
    assert leader.get_action_for_observation(follower) == follower
    feed(reader, CONTROL_IDS)
    assert leader.get_action_for_observation(follower) == reader.positions()
    assert leader._hold_positions is None


def test_leader_pause_does_not_hide_transport_failure():
    leader = PiperXLeader(PiperXLeaderConfig())
    leader.reader.bus = object()
    leader.reader.thread = SimpleNamespace(is_alive=lambda: True)
    leader.reader.error = ConnectionError("CAN error")
    with pytest.raises(ConnectionError):
        leader.get_action_for_observation(dict.fromkeys(KEYS, 0.0))


def test_leader_pause_does_not_accept_nonfinite_observation():
    leader = PiperXLeader(PiperXLeaderConfig())
    leader.reader.bus = object()
    leader.reader.thread = SimpleNamespace(is_alive=lambda: True)
    with pytest.raises(ValueError, match="non-finite"):
        leader.get_action_for_observation(dict.fromkeys(KEYS, float("nan")))
