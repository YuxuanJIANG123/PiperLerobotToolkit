"""URDF forward kinematics and bounded local IK; no hardware access."""

import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def rotation_vector(matrix):
    """Principal SO(3) logarithm, including rotations close to pi."""
    theta = np.arccos(np.clip((np.trace(matrix) - 1) / 2, -1.0, 1.0))
    skew = np.array([matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]])
    if theta < 1e-7:
        return skew / 2
    if np.pi - theta < 1e-5:
        _, vectors = np.linalg.eigh((matrix + matrix.T) / 2)
        axis = vectors[:, -1]
        if axis @ skew < 0:
            axis = -axis
        return axis * theta
    return skew * theta / (2 * np.sin(theta))


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    roll, pitch, yaw = rpy
    result = np.eye(4)
    result[:3, :3] = rotation((0, 0, 1), yaw) @ rotation((0, 1, 0), pitch) @ rotation((1, 0, 0), roll)
    result[:3, 3] = xyz
    return result


@dataclass
class IKResult:
    joints: np.ndarray | None
    reason: str
    position_error_m: float
    orientation_error_rad: float
    iterations: int


class ArmKinematics:
    """Six revolute joints from base_link to link6, followed by a configured TCP."""

    def __init__(self, model, tcp_xyz=(0, 0, 0), tcp_rpy=(0, 0, 0)):
        if model not in ("piper_h", "piper_x"):
            raise ValueError("Supported models: piper_h, piper_x")
        path = Path(__file__).with_name("models") / f"{model}.urdf"
        root = ET.parse(path).getroot()
        self.origins, self.axes, limits = [], [], []
        parent = "base_link"
        for index in range(1, 7):
            joint = root.find(f"./joint[@name='joint{index}']")
            if joint is None or joint.attrib["type"] != "revolute":
                raise ValueError("Unexpected robot model chain")
            if joint.find("parent").attrib["link"] != parent:
                raise ValueError("Non-serial robot model chain")
            parent = joint.find("child").attrib["link"]
            origin = joint.find("origin")
            xyz = np.fromstring(origin.attrib["xyz"], sep=" ")
            rpy = np.fromstring(origin.attrib["rpy"], sep=" ")
            self.origins.append(transform(xyz, rpy))
            self.axes.append(np.fromstring(joint.find("axis").attrib["xyz"], sep=" "))
            limit = joint.find("limit")
            limits.append([float(limit.attrib["lower"]), float(limit.attrib["upper"])])
        self.limits = np.asarray(limits)
        self.tcp = transform(tcp_xyz, tcp_rpy)

    def forward(self, joints, with_jacobian=False):
        q = np.asarray(joints, dtype=float)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError("Expected six finite joint angles in radians")
        pose = np.eye(4)
        points, axes = [], []
        for origin, axis, angle in zip(self.origins, self.axes, q, strict=True):
            pose = pose @ origin
            points.append(pose[:3, 3].copy())
            axes.append(pose[:3, :3] @ axis)
            turn = np.eye(4)
            turn[:3, :3] = rotation(axis, angle)
            pose = pose @ turn
        pose = pose @ self.tcp
        if not with_jacobian:
            return pose
        jacobian = np.empty((6, 6))
        for i, (point, axis) in enumerate(zip(points, axes, strict=True)):
            jacobian[:3, i] = np.cross(axis, pose[:3, 3] - point)
            jacobian[3:, i] = axis
        return pose, jacobian

    def inverse(
        self,
        desired,
        seed,
        limits,
        *,
        position_tolerance=0.0005,
        orientation_tolerance=0.005,
        max_iterations=60,
        min_singular_value=1e-5,
        max_time_s=0.02,
    ):
        """Damped local solve with joint projection, step cap and backtracking.

        Returns no joint target on nonconvergence/singularity. This is not a
        collision planner. Orientation residuals/Jacobian are scaled by 0.1 m/rad.
        """
        started = time.perf_counter()
        q = np.asarray(seed, dtype=float).copy()
        bounds = np.asarray(limits, dtype=float)
        goal = np.asarray(desired, dtype=float)
        if bounds.shape != (6, 2) or not np.all(np.isfinite(bounds)) or np.any(bounds[:, 0] >= bounds[:, 1]):
            raise ValueError("Invalid IK joint limits")
        if goal.shape != (4, 4) or not np.all(np.isfinite(goal)):
            raise ValueError("Invalid target transform")
        if (
            not np.allclose(goal[3], [0, 0, 0, 1])
            or not np.allclose(goal[:3, :3].T @ goal[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(goal[:3, :3]), 1)
        ):
            raise ValueError("Target must be a rigid transform")
        self.forward(q)  # Validate shape and finiteness before comparisons.
        if np.any(q < bounds[:, 0] - 1e-7) or np.any(q > bounds[:, 1] + 1e-7):
            return IKResult(None, "seed_outside_limits", float("inf"), float("inf"), 0)
        q = np.clip(q, bounds[:, 0], bounds[:, 1])

        def residual(pose):
            return np.r_[goal[:3, 3] - pose[:3, 3], rotation_vector(goal[:3, :3] @ pose[:3, :3].T)]

        for iteration in range(max_iterations + 1):
            pose, jacobian = self.forward(q, with_jacobian=True)
            error = residual(pose)
            pe, oe = np.linalg.norm(error[:3]), np.linalg.norm(error[3:])
            if time.perf_counter() - started > max_time_s:
                return IKResult(None, "solve_timeout", float(pe), float(oe), iteration)
            if pe <= position_tolerance and oe <= orientation_tolerance:
                return IKResult(q, "ok", float(pe), float(oe), iteration)
            if iteration == max_iterations:
                break
            error[3:] *= 0.1
            jacobian[3:] *= 0.1
            singular_values = np.linalg.svd(jacobian, compute_uv=False)
            if singular_values[-1] < min_singular_value:
                return IKResult(None, "singular", float(pe), float(oe), iteration)
            step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 0.002**2 * np.eye(6), error)
            step *= min(1.0, 0.1 / max(np.max(np.abs(step)), 1e-12))
            improved = False
            for scale in (1.0, 0.5, 0.25, 0.125, 0.0625):
                candidate = np.clip(q + scale * step, bounds[:, 0], bounds[:, 1])
                next_error = residual(self.forward(candidate))
                next_error[3:] *= 0.1
                if np.linalg.norm(next_error) < np.linalg.norm(error) - 1e-12:
                    q = candidate
                    improved = True
                    break
            if not improved:
                break
        return IKResult(None, "unreachable_or_no_convergence", float(pe), float(oe), iteration)
