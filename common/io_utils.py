#!/usr/bin/env python3
"""I/O helpers shared by task scripts."""

from __future__ import annotations

from pathlib import Path


COURSE_ROOT = Path(__file__).resolve().parents[1]
RESULTS_DIR = COURSE_ROOT / "results"


def ensure_results_dir(task_name: str) -> Path:
    path = RESULTS_DIR / task_name
    path.mkdir(parents=True, exist_ok=True)
    return path


def configure_matplotlib() -> None:
    import matplotlib

    matplotlib.use("Agg")

