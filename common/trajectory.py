#!/usr/bin/env python3
"""Polynomial trajectory helpers."""

from __future__ import annotations

import numpy as np


def quintic_coefficients(q0: np.ndarray, qf: np.ndarray, duration: float) -> np.ndarray:
    q0 = np.asarray(q0, dtype=float)
    qf = np.asarray(qf, dtype=float)
    if duration <= 0.0:
        raise ValueError("duration must be positive")

    coeffs = np.zeros((q0.size, 6), dtype=float)
    coeffs[:, 0] = q0
    delta = qf - q0
    t = duration
    coeffs[:, 3] = 10.0 * delta / t**3
    coeffs[:, 4] = -15.0 * delta / t**4
    coeffs[:, 5] = 6.0 * delta / t**5
    return coeffs


def evaluate_quintic(coeffs: np.ndarray, time_s: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    t = float(time_s)
    powers = np.array([1.0, t, t**2, t**3, t**4, t**5], dtype=float)
    dpowers = np.array([0.0, 1.0, 2.0 * t, 3.0 * t**2, 4.0 * t**3, 5.0 * t**4], dtype=float)
    ddpowers = np.array([0.0, 0.0, 2.0, 6.0 * t, 12.0 * t**2, 20.0 * t**3], dtype=float)
    return coeffs @ powers, coeffs @ dpowers, coeffs @ ddpowers


def make_time_grid(duration: float, dt: float) -> np.ndarray:
    if duration <= 0.0 or dt <= 0.0:
        raise ValueError("duration and dt must be positive")
    steps = int(np.floor(duration / dt)) + 1
    return np.linspace(0.0, duration, steps, dtype=float)

