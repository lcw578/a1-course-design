#!/usr/bin/env python3
"""Simplified sagittal-plane dynamics for one A1 leg.

This is intentionally a 2R model for the thigh/calf pitch joints. The A1 hip
abduction joint is handled in the kinematics tasks, while dynamics/control
experiments use the 2R sagittal subsystem that can be clearly derived in a
course report and compared against PID.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PlanarLegParams:
    l1: float = 0.2
    l2: float = 0.2
    m1: float = 1.013
    m2: float = 0.166
    lc1: float = 0.1
    lc2: float = 0.1
    i1: float = 0.005139339
    i2: float = 0.003014022
    gravity: float = 9.81


class PlanarLegDynamics:
    """Two-link manipulator dynamics with angles measured from horizontal."""

    def __init__(self, params: PlanarLegParams | None = None) -> None:
        self.params = params or PlanarLegParams()

    def mass_matrix(self, q: np.ndarray) -> np.ndarray:
        p = self.params
        q = np.asarray(q, dtype=float).reshape(2)
        c2 = np.cos(q[1])
        m11 = (
            p.i1
            + p.i2
            + p.m1 * p.lc1**2
            + p.m2 * (p.l1**2 + p.lc2**2 + 2.0 * p.l1 * p.lc2 * c2)
        )
        m12 = p.i2 + p.m2 * (p.lc2**2 + p.l1 * p.lc2 * c2)
        m22 = p.i2 + p.m2 * p.lc2**2
        return np.array([[m11, m12], [m12, m22]], dtype=float)

    def coriolis_vector(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        p = self.params
        q = np.asarray(q, dtype=float).reshape(2)
        dq = np.asarray(dq, dtype=float).reshape(2)
        s2 = np.sin(q[1])
        h = p.m2 * p.l1 * p.lc2 * s2
        return np.array(
            [
                -h * (2.0 * dq[0] * dq[1] + dq[1] ** 2),
                h * dq[0] ** 2,
            ],
            dtype=float,
        )

    def gravity_vector(self, q: np.ndarray) -> np.ndarray:
        p = self.params
        q = np.asarray(q, dtype=float).reshape(2)
        g1 = (p.m1 * p.lc1 + p.m2 * p.l1) * p.gravity * np.cos(q[0])
        g1 += p.m2 * p.lc2 * p.gravity * np.cos(q[0] + q[1])
        g2 = p.m2 * p.lc2 * p.gravity * np.cos(q[0] + q[1])
        return np.array([g1, g2], dtype=float)

    def inertia_vector(self, q: np.ndarray, ddq: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(2)
        ddq = np.asarray(ddq, dtype=float).reshape(2)
        return self.mass_matrix(q) @ ddq

    def decompose_dynamics(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        ddq: np.ndarray,
    ) -> dict[str, np.ndarray]:
        q = np.asarray(q, dtype=float).reshape(2)
        dq = np.asarray(dq, dtype=float).reshape(2)
        ddq = np.asarray(ddq, dtype=float).reshape(2)
        inertia = self.inertia_vector(q, ddq)
        coriolis = self.coriolis_vector(q, dq)
        gravity = self.gravity_vector(q)
        return {
            "inertia": inertia,
            "coriolis": coriolis,
            "gravity": gravity,
            "tau_model": inertia + coriolis + gravity,
        }

    def forward_dynamics(self, q: np.ndarray, dq: np.ndarray, tau: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(2)
        dq = np.asarray(dq, dtype=float).reshape(2)
        tau = np.asarray(tau, dtype=float).reshape(2)
        rhs = tau - self.coriolis_vector(q, dq) - self.gravity_vector(q)
        return np.linalg.solve(self.mass_matrix(q), rhs)

    def inverse_dynamics(self, q: np.ndarray, dq: np.ndarray, ddq: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(2)
        dq = np.asarray(dq, dtype=float).reshape(2)
        ddq = np.asarray(ddq, dtype=float).reshape(2)
        return self.mass_matrix(q) @ ddq + self.coriolis_vector(q, dq) + self.gravity_vector(q)

    def step(self, q: np.ndarray, dq: np.ndarray, tau: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        ddq = self.forward_dynamics(q, dq, tau)
        dq_next = np.asarray(dq, dtype=float) + ddq * dt
        q_next = np.asarray(q, dtype=float) + dq_next * dt
        return q_next, dq_next, ddq
