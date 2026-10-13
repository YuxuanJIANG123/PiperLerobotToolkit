import importlib.util
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from lerobot_robot_piper_x.protocol import KEYS

spec = importlib.util.spec_from_file_location(
    "mode_switch", Path(__file__).resolve().parents[1] / "scripts/switch_follower_mode.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Reader:
    timeout_s = 0.5

    def __init__(self, acknowledge=True):
        self.lock = threading.Lock()
        self.frames = {0x2A1: (bytes([2, 0, 1, 2, 0, 0, 0, 0]), time.monotonic())}
        self.bus = self
        self.sent = []
        self.acknowledge = acknowledge

    def positions(self):
        return dict.fromkeys(KEYS, 0.0)

    def send(self, msg, timeout):
        self.sent.append(msg)
        if self.acknowledge and msg.arbitration_id == 0x151:
            self.frames[0x2A1] = (bytes([msg.data[0], 0, 1, 2, 0, 0, 0, 0]), time.monotonic())

    def check_follower_status(self):
        assert self.frames[0x2A1][0][0] == 1


CONFIG = SimpleNamespace(joint_limits_rad=[[-1, 1]] * 6, gripper_max_m=0.07)


def test_standby_ack_precedes_target_seed_and_can_request():
    reader = Reader()
    module.switch(reader, CONFIG)
    assert [m.arbitration_id for m in reader.sent] == [0x151, 0x155, 0x156, 0x157, 0x151]
    assert reader.sent[0].data[0] == 0
    assert reader.sent[-1].data[0] == 1
    assert all(m.data == bytes(8) for m in reader.sent[1:4])


def test_failed_standby_never_sends_target_or_can_mode():
    reader = Reader(acknowledge=False)
    with pytest.raises(ConnectionError, match="not acknowledged"):
        module.switch(reader, CONFIG, timeout=0.01)
    assert len(reader.sent) == 1
    assert reader.sent[0].arbitration_id == 0x151
    assert reader.sent[0].data[0] == 0


@pytest.mark.parametrize("raw,age", [
    (bytes([2, 0, 1, 1, 0, 0, 0, 0]), 0),
    (bytes([2, 1, 1, 2, 0, 0, 0, 0]), 0),
    (bytes([2, 0, 1, 2, 0, 0, 0, 0]), 1),
])
def test_unsafe_status_sends_nothing(raw, age):
    reader = Reader()
    reader.frames[0x2A1] = (raw, time.monotonic() - age)
    with pytest.raises(ConnectionError):
        module.switch(reader, CONFIG)
    assert not reader.sent


def test_out_of_bounds_sends_nothing():
    reader = Reader()
    reader.positions = lambda: dict.fromkeys(KEYS, 2.0)
    with pytest.raises(ValueError):
        module.switch(reader, CONFIG)
    assert not reader.sent
