import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from lerobot_robot_piper_x import PiperXConfig
from lerobot_robot_piper_x.protocol import KEYS

spec = importlib.util.spec_from_file_location(
    "probe", Path(__file__).resolve().parents[1] / "scripts/probe_wrist.py"
)
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def test_probe_changes_only_selected_joint_and_respects_limits():
    initial = dict.fromkeys(KEYS, 0.0)
    initial[KEYS[-1]] = 0.02
    cfg = PiperXConfig()
    target = probe.target_for(initial, 4, -1, cfg)
    assert target[KEYS[3]] == -math.radians(1)
    assert all(target[k] == initial[k] for k in KEYS if k != KEYS[3])
    initial[KEYS[3]] = cfg.joint_limits_rad[3][1]
    with pytest.raises(ValueError):
        probe.target_for(initial, 4, 1, cfg)


def test_stale_motor_status_sends_no_motion(monkeypatch):
    initial = dict.fromkeys(KEYS, 0.0)
    sent = []
    robot = SimpleNamespace(
        config=PiperXConfig(), reader=SimpleNamespace(positions=lambda: initial),
        sdk=SimpleNamespace(GetArmLowSpdInfoMsgs=lambda: SimpleNamespace(time_stamp=0)),
        send_action=sent.append,
    )
    monkeypatch.setattr(probe.time, "time", lambda: 100)
    with pytest.raises(ConnectionError, match="Stale"):
        probe.probe(robot, 4, 1)
    assert sent == []
