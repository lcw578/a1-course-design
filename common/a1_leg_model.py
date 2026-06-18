#!/usr/bin/env python3
"""A1 single-leg kinematics used by the course design tasks.

The model mirrors the geometry already used in rl_sar's A1 state-estimation
nodes: hip abduction/adduction rotates around body X, thigh and calf rotate
around the locally transformed Y axes, and the foot position is expressed in
the base/body frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Tuple

import numpy as np


LEG_NAMES = ("FR", "FL", "RR", "RL")


def rot_x(theta: float) -> np.ndarray:
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=float,
    )


def rot_y(theta: float) -> np.ndarray:
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ],
        dtype=float,
    )


def homogeneous(rotation: np.ndarray, translation: Iterable[float]) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation, dtype=float)
    return transform


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=float).reshape(3)
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=float,
    )


def unskew(matrix: np.ndarray) -> np.ndarray:
    return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]], dtype=float)


def adjoint(transform: np.ndarray) -> np.ndarray:
    """Adjoint matrix for twists ordered as [omega; v]."""

    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    result = np.zeros((6, 6), dtype=float)
    result[:3, :3] = rotation
    result[3:, :3] = skew(translation) @ rotation
    result[3:, 3:] = rotation
    return result


def se3_to_twist(matrix: np.ndarray) -> np.ndarray:
    """Convert an se(3) matrix to a 6D twist [omega; v]."""

    return np.concatenate([unskew(matrix[:3, :3]), matrix[:3, 3]])


@dataclass(frozen=True)
class LegGeometry:
    hip_origin: np.ndarray
    thigh_origin: np.ndarray


@dataclass(frozen=True)
class A1LegParameters:
    front_hip_origin_x: float = 0.1805
    rear_hip_origin_x: float = -0.1805
    left_hip_origin_y: float = 0.047
    right_hip_origin_y: float = -0.047
    left_thigh_origin_y: float = 0.0838
    right_thigh_origin_y: float = -0.0838
    thigh_length: float = 0.2
    calf_length: float = 0.2


class A1LegKinematics:
    """Forward/inverse kinematics and geometric Jacobian for one A1 leg."""

    joint_limits = {
        "hip": (-0.802851455917, 0.802851455917),
        "thigh": (-1.0471975512, 4.18879020479),
        "calf": (-2.69653369433, -0.916297857297),
    }

    default_q = np.array([0.0, 0.8, -1.5], dtype=float)

    def __init__(self, params: A1LegParameters | None = None) -> None:
        self.params = params or A1LegParameters()
        p = self.params
        self.leg_geometry: Dict[str, LegGeometry] = {
            "FR": LegGeometry(
                np.array([p.front_hip_origin_x, p.right_hip_origin_y, 0.0], dtype=float),
                np.array([0.0, p.right_thigh_origin_y, 0.0], dtype=float),
            ),
            "FL": LegGeometry(
                np.array([p.front_hip_origin_x, p.left_hip_origin_y, 0.0], dtype=float),
                np.array([0.0, p.left_thigh_origin_y, 0.0], dtype=float),
            ),
            "RR": LegGeometry(
                np.array([p.rear_hip_origin_x, p.right_hip_origin_y, 0.0], dtype=float),
                np.array([0.0, p.right_thigh_origin_y, 0.0], dtype=float),
            ),
            "RL": LegGeometry(
                np.array([p.rear_hip_origin_x, p.left_hip_origin_y, 0.0], dtype=float),
                np.array([0.0, p.left_thigh_origin_y, 0.0], dtype=float),
            ),
        }

    @property
    def lower_limits(self) -> np.ndarray:
        return np.array([self.joint_limits[k][0] for k in ("hip", "thigh", "calf")], dtype=float)

    @property
    def upper_limits(self) -> np.ndarray:
        return np.array([self.joint_limits[k][1] for k in ("hip", "thigh", "calf")], dtype=float)

    def clamp_joints(self, q: np.ndarray) -> np.ndarray:
        return np.clip(np.asarray(q, dtype=float), self.lower_limits, self.upper_limits)

    def forward_kinematics(self, q: np.ndarray, leg_name: str = "FR") -> Tuple[np.ndarray, dict]:
        if leg_name not in self.leg_geometry:
            raise ValueError(f"Unknown leg '{leg_name}', expected one of {LEG_NAMES}")

        q = np.asarray(q, dtype=float).reshape(3)
        geometry = self.leg_geometry[leg_name]
        q_hip, q_thigh, q_calf = [float(v) for v in q]

        r_hip = rot_x(q_hip)
        r_thigh = rot_y(q_thigh)
        r_calf = rot_y(q_calf)
        r_hip_thigh = r_hip @ r_thigh
        r_total = r_hip_thigh @ r_calf

        origin_hip = geometry.hip_origin
        origin_thigh = origin_hip + r_hip @ geometry.thigh_origin
        origin_calf = origin_thigh + r_hip_thigh @ np.array(
            [0.0, 0.0, -self.params.thigh_length], dtype=float
        )
        foot_pos = origin_calf + r_total @ np.array(
            [0.0, 0.0, -self.params.calf_length], dtype=float
        )

        data = {
            "origin_hip": origin_hip,
            "origin_thigh": origin_thigh,
            "origin_calf": origin_calf,
            "axis_hip": np.array([1.0, 0.0, 0.0], dtype=float),
            "axis_thigh": r_hip @ np.array([0.0, 1.0, 0.0], dtype=float),
            "axis_calf": r_hip_thigh @ np.array([0.0, 1.0, 0.0], dtype=float),
            "r_hip": r_hip,
            "r_hip_thigh": r_hip_thigh,
            "r_total": r_total,
        }
        return foot_pos, data

    def transform_chain(self, q: np.ndarray, leg_name: str = "FR") -> Tuple[np.ndarray, ...]:
        """Return the equivalent homogeneous transform chain base->foot.

        This is the implementation counterpart to the DH table in the report.
        The order follows the URDF joint semantics: translate to the joint
        origin, rotate about the joint axis, then translate to the next joint.
        """

        geometry = self.leg_geometry[leg_name]
        q = np.asarray(q, dtype=float).reshape(3)
        return (
            homogeneous(np.eye(3), geometry.hip_origin),
            homogeneous(rot_x(q[0]), [0.0, 0.0, 0.0]),
            homogeneous(np.eye(3), geometry.thigh_origin),
            homogeneous(rot_y(q[1]), [0.0, 0.0, 0.0]),
            homogeneous(np.eye(3), [0.0, 0.0, -self.params.thigh_length]),
            homogeneous(rot_y(q[2]), [0.0, 0.0, 0.0]),
            homogeneous(np.eye(3), [0.0, 0.0, -self.params.calf_length]),
        )

    def foot_transform(self, q: np.ndarray, leg_name: str = "FR") -> np.ndarray:
        transform = np.eye(4, dtype=float)
        for step in self.transform_chain(q, leg_name):
            transform = transform @ step
        return transform

    def jacobian(self, q: np.ndarray, leg_name: str = "FR") -> np.ndarray:
        foot_pos, data = self.forward_kinematics(q, leg_name)
        jac = np.column_stack(
            [
                np.cross(data["axis_hip"], foot_pos - data["origin_hip"]),
                np.cross(data["axis_thigh"], foot_pos - data["origin_thigh"]),
                np.cross(data["axis_calf"], foot_pos - data["origin_calf"]),
            ]
        )
        return jac

    def spatial_jacobian(self, q: np.ndarray, leg_name: str = "FR") -> np.ndarray:
        """Return the Modern Robotics space Jacobian Js.

        Columns are twists expressed in the trunk/base frame and ordered as
        [omega; v]. For a revolute joint with axis omega through point p,
        v = -omega x p. This differs from the foot-origin linear velocity
        Jacobian returned by :meth:`jacobian`.
        """

        _, data = self.forward_kinematics(q, leg_name)
        axes = (data["axis_hip"], data["axis_thigh"], data["axis_calf"])
        origins = (data["origin_hip"], data["origin_thigh"], data["origin_calf"])
        columns = []
        for axis, origin in zip(axes, origins):
            omega = np.asarray(axis, dtype=float).reshape(3)
            v = -np.cross(omega, np.asarray(origin, dtype=float).reshape(3))
            columns.append(np.concatenate([omega, v]))
        return np.column_stack(columns)

    def body_jacobian(self, q: np.ndarray, leg_name: str = "FR") -> np.ndarray:
        """Return the Modern Robotics body Jacobian Jb.

        Jb = Ad(T_bs^-1) Js, where T_bs is the base-to-foot homogeneous
        transform.
        """

        transform = self.foot_transform(q, leg_name)
        return adjoint(np.linalg.inv(transform)) @ self.spatial_jacobian(q, leg_name)

    def point_velocity_jacobian_from_spatial(
        self, q: np.ndarray, leg_name: str = "FR"
    ) -> np.ndarray:
        """Recover foot-origin velocity Jacobian from Js.

        For a space twist V_s = [omega_s; v_s], the end-effector origin velocity
        is p_dot = v_s + omega_s x p, with p expressed in the base frame.
        """

        transform = self.foot_transform(q, leg_name)
        foot_pos = transform[:3, 3]
        jac_spatial = self.spatial_jacobian(q, leg_name)
        jac_point = np.zeros((3, 3), dtype=float)
        for i in range(3):
            omega = jac_spatial[:3, i]
            v = jac_spatial[3:, i]
            jac_point[:, i] = v + np.cross(omega, foot_pos)
        return jac_point

    def inverse_kinematics(
        self,
        target: np.ndarray,
        seed: np.ndarray | None = None,
        leg_name: str = "FR",
        max_iters: int = 100,
        tolerance: float = 1e-9,
        damping: float = 1e-4,
    ) -> Tuple[np.ndarray, float, int, bool]:
        """Damped least-squares IK for foot position."""

        target = np.asarray(target, dtype=float).reshape(3)
        candidate_seeds = []
        candidate_seeds.extend(self.analytic_ik_candidates(target, leg_name))
        if seed is not None:
            candidate_seeds.append(np.asarray(seed, dtype=float))
        candidate_seeds.append(self.default_q)

        best_q = self.clamp_joints(candidate_seeds[0])
        best_error = float("inf")
        best_iteration = 0

        for raw_seed in candidate_seeds:
            q = self.clamp_joints(raw_seed)
            for iteration in range(1, max_iters + 1):
                pos, _ = self.forward_kinematics(q, leg_name)
                error = target - pos
                err_norm = float(np.linalg.norm(error))
                if err_norm < best_error:
                    best_q = q.copy()
                    best_error = err_norm
                    best_iteration = iteration
                if err_norm <= tolerance:
                    return q, err_norm, iteration, True

                jac = self.jacobian(q, leg_name)
                lhs = jac @ jac.T + damping * np.eye(3)
                step = jac.T @ np.linalg.solve(lhs, error)
                q = self.clamp_joints(q + step)

        return best_q, best_error, best_iteration, best_error <= tolerance

    def analytic_ik_candidates(self, target: np.ndarray, leg_name: str = "FR") -> list[np.ndarray]:
        """Closed-form position IK candidates for the A1 leg geometry."""

        if leg_name not in self.leg_geometry:
            raise ValueError(f"Unknown leg '{leg_name}', expected one of {LEG_NAMES}")

        target = np.asarray(target, dtype=float).reshape(3)
        geometry = self.leg_geometry[leg_name]
        p = target - geometry.hip_origin
        y_offset = float(geometry.thigh_origin[1])
        radius_yz = float(np.hypot(p[1], p[2]))
        if radius_yz < abs(y_offset):
            return [self.default_q.copy()]

        phi = float(np.arctan2(p[2], p[1]))
        hip_options = [phi + np.arccos(y_offset / radius_yz), phi - np.arccos(y_offset / radius_yz)]
        candidates: list[np.ndarray] = []
        l1 = self.params.thigh_length
        l2 = self.params.calf_length

        for q_hip in hip_options:
            v = rot_x(-q_hip) @ p - geometry.thigh_origin
            x = float(v[0])
            z = float(v[2])
            distance_sq = x * x + z * z
            cos_calf = (distance_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
            if cos_calf < -1.0 - 1e-9 or cos_calf > 1.0 + 1e-9:
                continue
            cos_calf = float(np.clip(cos_calf, -1.0, 1.0))
            for q_calf in (-np.arccos(cos_calf), np.arccos(cos_calf)):
                q_thigh = np.arctan2(-x, -z) - np.arctan2(
                    l2 * np.sin(q_calf),
                    l1 + l2 * np.cos(q_calf),
                )
                q = np.array([q_hip, q_thigh, q_calf], dtype=float)
                q = self.clamp_joints(q)
                candidates.append(q)

        return candidates or [self.default_q.copy()]

    def sample_workspace(
        self, n_samples: int = 5000, leg_name: str = "FR", seed: int = 7
    ) -> Tuple[np.ndarray, np.ndarray]:
        rng = np.random.default_rng(seed)
        q = rng.uniform(self.lower_limits, self.upper_limits, size=(n_samples, 3))
        points = np.array([self.forward_kinematics(row, leg_name)[0] for row in q], dtype=float)
        return q, points

    def finite_difference_jacobian(
        self, q: np.ndarray, leg_name: str = "FR", eps: float = 1e-6
    ) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(3)
        jac = np.zeros((3, 3), dtype=float)
        for i in range(3):
            dq = np.zeros(3, dtype=float)
            dq[i] = eps
            p_plus, _ = self.forward_kinematics(q + dq, leg_name)
            p_minus, _ = self.forward_kinematics(q - dq, leg_name)
            jac[:, i] = (p_plus - p_minus) / (2.0 * eps)
        return jac

    def finite_difference_spatial_jacobian(
        self, q: np.ndarray, leg_name: str = "FR", eps: float = 1e-6
    ) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(3)
        transform = self.foot_transform(q, leg_name)
        transform_inv = np.linalg.inv(transform)
        jac = np.zeros((6, 3), dtype=float)
        for i in range(3):
            dq = np.zeros(3, dtype=float)
            dq[i] = eps
            t_plus = self.foot_transform(q + dq, leg_name)
            t_minus = self.foot_transform(q - dq, leg_name)
            t_dot = (t_plus - t_minus) / (2.0 * eps)
            jac[:, i] = se3_to_twist(t_dot @ transform_inv)
        return jac

    def finite_difference_body_jacobian(
        self, q: np.ndarray, leg_name: str = "FR", eps: float = 1e-6
    ) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(3)
        transform_inv = np.linalg.inv(self.foot_transform(q, leg_name))
        jac = np.zeros((6, 3), dtype=float)
        for i in range(3):
            dq = np.zeros(3, dtype=float)
            dq[i] = eps
            t_plus = self.foot_transform(q + dq, leg_name)
            t_minus = self.foot_transform(q - dq, leg_name)
            t_dot = (t_plus - t_minus) / (2.0 * eps)
            jac[:, i] = se3_to_twist(transform_inv @ t_dot)
        return jac

    def equivalent_dh_rows(self, leg_name: str = "FR") -> list[dict]:
        """Return report-friendly equivalent joint/link rows.

        A1's hip joint is orthogonal to the thigh/calf pitch joints. The report
        can present these rows as an equivalent MDH-style chain with explicit
        rotations and offsets, while the code validates the transforms directly.
        """

        geometry = self.leg_geometry[leg_name]
        return [
            {
                "index": "base",
                "joint": f"trunk_to_{leg_name}_hip",
                "axis": "-",
                "theta": "-",
                "translation_after_rotation_m": geometry.hip_origin.tolist(),
                "description": "base/trunk frame to hip joint origin",
            },
            {
                "index": 1,
                "joint": f"{leg_name}_hip_joint",
                "axis": "x",
                "theta": "q1",
                "translation_after_rotation_m": geometry.thigh_origin.tolist(),
                "description": "hip abduction/adduction; URDF axis x",
            },
            {
                "index": 2,
                "joint": f"{leg_name}_thigh_joint",
                "axis": "y",
                "theta": "q2",
                "translation_after_rotation_m": [0.0, 0.0, -self.params.thigh_length],
                "description": "thigh pitch; URDF axis y",
            },
            {
                "index": 3,
                "joint": f"{leg_name}_calf_joint",
                "axis": "y",
                "theta": "q3",
                "translation_after_rotation_m": [0.0, 0.0, -self.params.calf_length],
                "description": "calf pitch; URDF axis y",
            },
        ]
