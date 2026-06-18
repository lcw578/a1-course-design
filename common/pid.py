#!/usr/bin/env python3
"""Small vector PID implementation for the course-design controllers."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class VectorPID:
    kp: np.ndarray
    ki: np.ndarray
    kd: np.ndarray
    integral_limit: float = 1.0

    def __post_init__(self) -> None:
        self.kp = np.asarray(self.kp, dtype=float)
        self.ki = np.asarray(self.ki, dtype=float)
        self.kd = np.asarray(self.kd, dtype=float)
        self.integral = np.zeros_like(self.kp, dtype=float)

    def reset(self) -> None:
        self.integral[:] = 0.0

    def update(
        self,
        position_error: np.ndarray,
        velocity_error: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        position_error = np.asarray(position_error, dtype=float)
        velocity_error = np.asarray(velocity_error, dtype=float)
        self.integral = np.clip(
            self.integral + position_error * dt,
            -self.integral_limit,
            self.integral_limit,
        )
        return self.kp * position_error + self.kd * velocity_error + self.ki * self.integral

