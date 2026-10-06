"""Relative Piper-H TCP motion -> Piper-X bounded joint targets."""

import math
import time
from dataclasses import dataclass, field

import numpy as np

from .kinematics import ArmKinematics, rotation, rotation_vector, transform
from .protocol import KEYS


def _vector(value, length, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {length} finite numbers")
    return result


@dataclass
class EndEffectorConfig:
    # These transforms must be checked on the installed arms before live use.
    frames_verified: bool = False
    translation_scale: float = 0.5
    track_orientation: bool = False
    # Rotation taking leader-base displacement vectors into follower-base coordinates.
    leader_to_follower_rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # Tool center relative to URDF link6. Zero means link6, not gripper fingertips.
    leader_tcp_xyz: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    follower_tcp_xyz: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    leader_tcp_rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    follower_tcp_rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # URDF angle = reported angle + verified offset; no inferred sign changes.
    leader_joint_offsets_rad: list[float] = field(default_factory=lambda: [0.0] * 6)
    follower_joint_offsets_rad: list[float] = field(default_factory=lambda: [0.0] * 6)
    max_translation_m: float = 0.10
    max_rotation_rad: float = 0.7
    max_solution_delta_rad: float = 0.5
    position_tolerance_m: float = 0.0005
    orientation_tolerance_rad: float = 0.005
    min_singular_value: float = 1e-5
    max_iterations: int = 60
    max_solve_time_s: float = 0.02
    gripper_scale: float = 1.0

    def __post_init__(self):
        for name in (
            "translation_scale",
            "max_translation_m",
            "max_rotation_rad",
            "max_solution_delta_rad",
            "position_tolerance_m",
            "orientation_tolerance_rad",
            "min_singular_value",
            "max_solve_time_s",
            "gripper_scale",
        ):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if not isinstance(self.max_iterations, int) or not 1 <= self.max_iterations <= 200:
            raise ValueError("max_iterations must be 1..200")
        if self.max_rotation_rad >= math.pi:
            raise ValueError("max_rotation_rad must be less than pi")
        for name in (
            "leader_to_follower_rpy",
            "leader_tcp_xyz",
            "follower_tcp_xyz",
            "leader_tcp_rpy",
            "follower_tcp_rpy",
        ):
            _vector(getattr(self, name), 3, name)
        for name in ("leader_joint_offsets_rad", "follower_joint_offsets_rad"):
            _vector(getattr(self, name), 6, name)


class RelativeEndEffector:
    def __init__(self, config, follower_limits, gripper_max_m):
        self.config = config
        self.leader = ArmKinematics("piper_h", config.leader_tcp_xyz, config.leader_tcp_rpy)
        self.follower = ArmKinematics("piper_x", config.follower_tcp_xyz, config.follower_tcp_rpy)
        self.alignment = transform(rpy=config.leader_to_follower_rpy)[:3, :3]
        self.leader_offsets = np.asarray(config.leader_joint_offsets_rad)
        self.follower_offsets = np.asarray(config.follower_joint_offsets_rad)
        bounds = np.asarray(follower_limits, dtype=float)
        if bounds.shape != (6, 2) or not np.all(np.isfinite(bounds)):
            raise ValueError("Expected six finite follower limit pairs")
        # Physical firmware/config limits AND the robot model limits must hold.
        self.limits = np.column_stack(
            (
                np.maximum(bounds[:, 0], self.follower.limits[:, 0] - self.follower_offsets),
                np.minimum(bounds[:, 1], self.follower.limits[:, 1] - self.follower_offsets),
            )
        )
        if np.any(self.limits[:, 0] >= self.limits[:, 1]):
            raise ValueError("Follower and model joint limits have no valid intersection")
        if not math.isfinite(gripper_max_m) or gripper_max_m <= 0:
            raise ValueError("Invalid gripper range")
        self.gripper_max_m = gripper_max_m
        self.reset()

    def reset(self):
        self.leader_anchor = None
        self.follower_anchor = None
        self.gripper_anchors = None
        self.previous_solution = None
        self.hold = None
        self.diagnostics = {"status": "awaiting_reference"}

    @staticmethod
    def values(action):
        return _vector([action[k] for k in KEYS], 7, "joint/gripper positions")

    def desired_pose(self, leader_pose):
        desired = self.follower_anchor.copy()
        desired[:3, 3] += self.config.translation_scale * (
            self.alignment @ (leader_pose[:3, 3] - self.leader_anchor[:3, 3])
        )
        if self.config.track_orientation:
            delta = leader_pose[:3, :3] @ self.leader_anchor[:3, :3].T
            desired[:3, :3] = self.alignment @ delta @ self.alignment.T @ self.follower_anchor[:3, :3]
        return desired

    def _pause(self, present, reason):
        if self.hold is None:
            self.hold = dict(zip(KEYS, present.tolist(), strict=True))
        self.diagnostics["status"] = reason
        return dict(self.hold)

    def map(self, leader_action, observation):
        started = time.perf_counter()
        lead, current = self.values(leader_action), self.values(observation)
        if (
            np.any(current[:6] < self.limits[:, 0] - 1e-7)
            or np.any(current[:6] > self.limits[:, 1] + 1e-7)
            or not 0 <= current[6] <= self.gripper_max_m
        ):
            raise ValueError("Follower pose outside the intersection of configured and URDF limits")
        leader_pose = self.leader.forward(lead[:6] + self.leader_offsets)
        follower_pose = self.follower.forward(current[:6] + self.follower_offsets)
        if self.leader_anchor is None:
            self.leader_anchor, self.follower_anchor = leader_pose.copy(), follower_pose.copy()
            self.gripper_anchors = (lead[6], current[6])
            self.diagnostics = {
                "status": "anchored",
                "leader_xyz_m": leader_pose[:3, 3].tolist(),
                "target_xyz_m": follower_pose[:3, 3].tolist(),
                "target_rotation_vector_rad": rotation_vector(follower_pose[:3, :3]).tolist(),
            }
            return dict(zip(KEYS, current.tolist(), strict=True))
        desired = self.desired_pose(leader_pose)
        translation = np.linalg.norm(desired[:3, 3] - self.follower_anchor[:3, 3])
        angle = np.linalg.norm(rotation_vector(desired[:3, :3] @ self.follower_anchor[:3, :3].T))
        self.diagnostics = {
            "status": "solving",
            "leader_xyz_m": leader_pose[:3, 3].tolist(),
            "target_xyz_m": desired[:3, 3].tolist(),
            "target_rotation_vector_rad": rotation_vector(desired[:3, :3]).tolist(),
            "measured_xyz_m": follower_pose[:3, 3].tolist(),
            "tracking_position_error_m": float(np.linalg.norm(desired[:3, 3] - follower_pose[:3, 3])),
            "relative_translation_m": float(translation),
            "max_translation_m": self.config.max_translation_m,
        }
        if translation > self.config.max_translation_m or angle > self.config.max_rotation_rad:
            return self._pause(current, "relative_workspace_limit")
        seed = current[:6]
        if (
            self.previous_solution is not None
            and np.max(np.abs(self.previous_solution - seed)) <= self.config.max_solution_delta_rad
        ):
            seed = self.previous_solution
        result = self.follower.inverse(
            desired,
            seed + self.follower_offsets,
            self.limits + self.follower_offsets[:, None],
            position_tolerance=self.config.position_tolerance_m,
            orientation_tolerance=self.config.orientation_tolerance_rad,
            max_iterations=self.config.max_iterations,
            min_singular_value=self.config.min_singular_value,
            max_time_s=self.config.max_solve_time_s,
        )
        self.diagnostics.update(
            position_error_m=result.position_error_m,
            orientation_error_rad=result.orientation_error_rad,
            iterations=result.iterations,
            solve_time_ms=round((time.perf_counter() - started) * 1000, 2),
        )
        if result.joints is None:
            return self._pause(current, result.reason)
        solution = result.joints - self.follower_offsets
        fraction = 1.0
        if np.max(np.abs(solution - current[:6])) > self.config.max_solution_delta_rad:
            # A valid distant solution must not permanently latch a hold while
            # the measured arm can never catch up. Solve nearer Cartesian goals
            # from measured joints; never accept a distant branch or partial IK.
            delta_rotation = rotation_vector(desired[:3, :3] @ follower_pose[:3, :3].T)
            angle_delta = np.linalg.norm(delta_rotation)
            for fraction in (0.5, 0.25, 0.125, 0.0625):
                remaining = self.config.max_solve_time_s - (time.perf_counter() - started)
                if remaining <= 0:
                    break
                waypoint = follower_pose.copy()
                waypoint[:3, 3] += fraction * (desired[:3, 3] - follower_pose[:3, 3])
                if angle_delta > 1e-12:
                    waypoint[:3, :3] = rotation(delta_rotation, fraction * angle_delta) @ follower_pose[:3, :3]
                candidate = self.follower.inverse(
                    waypoint, current[:6] + self.follower_offsets,
                    self.limits + self.follower_offsets[:, None],
                    position_tolerance=self.config.position_tolerance_m,
                    orientation_tolerance=self.config.orientation_tolerance_rad,
                    max_iterations=self.config.max_iterations,
                    min_singular_value=self.config.min_singular_value,
                    max_time_s=remaining,
                )
                if candidate.joints is not None:
                    nearby = candidate.joints - self.follower_offsets
                    if np.max(np.abs(nearby - current[:6])) <= self.config.max_solution_delta_rad:
                        solution = nearby
                        break
            else:
                return self._pause(current, "joint_solution_too_far")
            if np.max(np.abs(solution - current[:6])) > self.config.max_solution_delta_rad:
                return self._pause(current, "joint_solution_too_far")
        self.diagnostics.update(
            waypoint_fraction=fraction,
            commanded_joint_delta_rad=float(np.max(np.abs(solution - current[:6]))),
            solve_time_ms=round((time.perf_counter() - started) * 1000, 2),
        )
        self.previous_solution = solution.copy()
        self.hold = None
        self.diagnostics["status"] = "tracking"
        gripper = self.gripper_anchors[1] + self.config.gripper_scale * (lead[6] - self.gripper_anchors[0])
        return dict(
            zip(KEYS, [*solution.tolist(), float(np.clip(gripper, 0, self.gripper_max_m))], strict=True)
        )
