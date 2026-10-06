import math
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from lerobot_robot_piper_x import PiperXConfig, PiperXLeader, PiperXLeaderConfig
from lerobot_robot_piper_x.end_effector import EndEffectorConfig, RelativeEndEffector
from lerobot_robot_piper_x.kinematics import ArmKinematics, rotation, rotation_vector, transform
from lerobot_robot_piper_x.protocol import CONTROL_IDS, KEYS, TargetFramesUnavailable

Q = np.array([0.1, 0.6, -1.0, 0.2, 0.4, 0.2])


def action(q=Q, gripper=0.02):
    return dict(zip(KEYS, [*q, gripper], strict=True))


def mapper(**kwargs):
    return RelativeEndEffector(
        EndEffectorConfig(max_solve_time_s=1, **kwargs), ArmKinematics("piper_x").limits, 0.07
    )


@pytest.mark.parametrize("model", ["piper_h", "piper_x"])
def test_geometric_jacobian_matches_finite_differences_with_tcp(model):
    arm = ArmKinematics(model, tcp_xyz=[0.01, 0.02, 0.10], tcp_rpy=[0.1, 0.2, -0.1])
    pose, jacobian = arm.forward(Q, True)
    for j in range(6):
        moved = Q.copy()
        moved[j] += 1e-6
        next_pose = arm.forward(moved)
        derivative = np.r_[
            (next_pose[:3, 3] - pose[:3, 3]) / 1e-6,
            rotation_vector(next_pose[:3, :3] @ pose[:3, :3].T) / 1e-6,
        ]
        np.testing.assert_allclose(derivative, jacobian[:, j], atol=1e-6)


def test_pinned_models_have_distinct_geometry_and_expected_fk():
    np.testing.assert_allclose(
        ArmKinematics("piper_h").forward(Q)[:3, 3], [0.09874015, 0.01739476, 0.46252451], atol=1e-8
    )
    np.testing.assert_allclose(
        ArmKinematics("piper_x").forward(Q)[:3, 3], [0.12269283, -0.00138772, 0.49171390], atol=1e-8
    )


@pytest.mark.parametrize("angle", [0, 1e-9, 0.4, math.pi - 1e-7, math.pi])
def test_rotation_log_roundtrip(angle):
    matrix = rotation([1, 2, 3], angle)
    vector = rotation_vector(matrix)
    reconstructed = np.eye(3) if np.linalg.norm(vector) == 0 else rotation(vector, np.linalg.norm(vector))
    np.testing.assert_allclose(reconstructed, matrix, atol=2e-7)


def test_ik_reaches_known_target_with_joint_limits():
    arm = ArmKinematics("piper_x")
    desired = arm.forward(Q + np.array([0.04, -0.03, 0.02, 0.02, -0.02, 0.03]))
    result = arm.inverse(desired, Q, arm.limits, max_time_s=1)
    assert result.reason == "ok"
    pose = arm.forward(result.joints)
    assert np.linalg.norm(pose[:3, 3] - desired[:3, 3]) < 0.0005
    assert np.linalg.norm(rotation_vector(pose[:3, :3] @ desired[:3, :3].T)) < 0.005
    assert np.all(result.joints >= arm.limits[:, 0])
    assert np.all(result.joints <= arm.limits[:, 1])


def test_unreachable_ik_never_returns_partial_solution():
    arm = ArmKinematics("piper_x")
    desired = arm.forward(Q)
    desired[0, 3] += 10
    result = arm.inverse(desired, Q, arm.limits, max_time_s=1)
    assert result.joints is None
    assert result.reason in ("unreachable_or_no_convergence", "singular")


def test_singularity_and_timeout_are_explicit_failures(monkeypatch):
    arm = ArmKinematics("piper_x")
    goal = arm.forward(Q + 0.02)
    assert arm.inverse(goal, Q, arm.limits, min_singular_value=10).reason == "singular"
    clock = iter([0, 1])
    monkeypatch.setattr("lerobot_robot_piper_x.kinematics.time.perf_counter", lambda: next(clock))
    assert arm.inverse(goal, Q, arm.limits).reason == "solve_timeout"


def test_anchor_does_not_jump_even_with_different_initial_poses():
    m = mapper()
    follower = action(Q + 0.1, 0.04)
    assert m.map(action(), follower) == follower
    assert m.diagnostics["status"] == "anchored"


def test_relative_translation_and_rotation_follow_alignment():
    m = mapper(track_orientation=True, leader_to_follower_rpy=[0, 0, math.pi / 2])
    m.map(action(), action(Q + 0.1))
    lead = m.leader_anchor.copy()
    lead[:3, 3] += [0.02, 0, 0]
    lead[:3, :3] = rotation([1, 0, 0], 0.1) @ lead[:3, :3]
    desired = m.desired_pose(lead)
    np.testing.assert_allclose(desired[:3, 3] - m.follower_anchor[:3, 3], [0, 0.01, 0], atol=1e-10)
    np.testing.assert_allclose(
        desired[:3, :3], rotation([0, 1, 0], 0.1) @ m.follower_anchor[:3, :3], atol=1e-10
    )


@pytest.mark.parametrize("track_orientation", [False, True])
def test_h_to_x_mapping_matches_cartesian_goal_not_joint_copy(track_orientation):
    m = mapper(track_orientation=track_orientation)
    m.map(action(), action())
    lead = action(Q + np.array([0.01, 0.015, -0.01, 0.01, -0.01, 0.01]), 0.03)
    target = m.map(lead, action())
    assert m.diagnostics["status"] == "tracking"
    actual = m.follower.forward([target[k] for k in KEYS[:6]])
    desired = m.desired_pose(m.leader.forward([lead[k] for k in KEYS[:6]]))
    assert np.linalg.norm(actual[:3, 3] - desired[:3, 3]) < m.config.position_tolerance_m
    assert np.linalg.norm(rotation_vector(actual[:3, :3] @ desired[:3, :3].T)) < 0.005
    assert not np.allclose([target[k] for k in KEYS[:6]], [lead[k] for k in KEYS[:6]])
    assert target[KEYS[-1]] == pytest.approx(0.03)


def test_outside_relative_workspace_holds_then_recovers():
    m = mapper(max_translation_m=0.003)
    m.map(action(), action())
    held = m.map(action(Q + 0.2), action())
    assert m.diagnostics["status"] == "relative_workspace_limit"
    assert held == action()
    assert m.map(action(Q + 0.2), action(Q + 0.001)) == held
    m.map(action(), action())
    assert m.diagnostics["status"] == "tracking"


def test_large_joint_branch_jump_is_rejected(monkeypatch):
    m = mapper(max_solution_delta_rad=0.001)
    m.map(action(), action())
    monkeypatch.setattr(m.follower, "inverse", lambda *a, **k: SimpleNamespace(
        joints=Q + 0.02, reason="ok", position_error_m=0, orientation_error_rad=0, iterations=1
    ))
    assert m.map(action(Q + 0.02), action()) == action()
    assert m.diagnostics["status"] == "joint_solution_too_far"


def test_distant_goal_advances_through_nearby_cartesian_waypoints():
    m = mapper(max_solution_delta_rad=0.03)
    measured = action()
    m.map(action(), measured)
    goal = action(Q + 0.08)
    first_error = None
    fractions = []
    for _ in range(30):
        target = m.map(goal, measured)
        assert m.diagnostics["status"] == "tracking"
        assert max(abs(target[k] - measured[k]) for k in KEYS[:6]) <= 0.03 + 1e-9
        fractions.append(m.diagnostics["waypoint_fraction"])
        error = m.diagnostics["tracking_position_error_m"]
        if first_error is None:
            first_error = error
        measured = target
    assert min(fractions) < 1
    assert error < first_error / 10


def test_offsets_model_limits_and_nonfinite_inputs():
    m = mapper(follower_joint_offsets_rad=[0.01] * 6)
    assert m.map(action(), action()) == action()
    with pytest.raises(ValueError, match="finite"):
        m.map(action(Q * float("nan")), action())
    outside = Q.copy()
    outside[2] = 0.1
    with pytest.raises(ValueError, match="outside"):
        m.map(action(), action(outside))


def test_raw_joint_fallback_and_unverified_live_mode_are_blocked():
    with pytest.raises(ValueError, match="verified"):
        PiperXLeader(PiperXLeaderConfig(control_mode="end_effector"))
    leader = PiperXLeader(
        PiperXLeaderConfig(control_mode="end_effector", eef=EndEffectorConfig(frames_verified=True))
    )
    with pytest.raises(RuntimeError, match="observation-aware"):
        leader.get_action()


def test_observation_hook_passes_real_follower_limits_and_reanchors_after_pause():
    leader = PiperXLeader(
        PiperXLeaderConfig(
            control_mode="end_effector", eef=EndEffectorConfig(frames_verified=True, max_solve_time_s=1)
        )
    )
    leader.reader = SimpleNamespace(is_connected=True, positions=lambda: action())
    robot_config = PiperXConfig(joint_limits_rad=ArmKinematics("piper_x").limits.tolist())
    assert leader.get_action_for_observation(action(), robot_config) == action()

    def silent():
        raise TargetFramesUnavailable("silent")

    leader.reader.positions = silent
    assert leader.get_action_for_observation(action(), robot_config) == action()
    assert leader._eef_mapper.leader_anchor is None
    # On recovery all frames must be fresh, and the new anchor must hold the
    # actual follower pose even if the leader moved far during the pause.
    leader.reader.lock = threading.Lock()
    leader.reader.ids = CONTROL_IDS
    leader.reader.frames = {i: (bytes(8), time.monotonic()) for i in CONTROL_IDS}
    leader.reader.positions = lambda: action(Q + 0.2)
    follower_after_pause = action(Q + 0.01)
    assert leader.get_action_for_observation(follower_after_pause, robot_config) == follower_after_pause
    assert leader._eef_mapper.diagnostics["status"] == "anchored"


def test_invalid_configuration():
    for kwargs in (
        {"translation_scale": 0},
        {"max_solve_time_s": float("nan")},
        {"leader_tcp_xyz": [1, 2]},
        {"max_iterations": 0},
    ):
        with pytest.raises(ValueError):
            EndEffectorConfig(**kwargs)
    with pytest.raises(ValueError):
        transform(rpy=[1, 2])
