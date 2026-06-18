#!/usr/bin/env python3
"""Task 1: A1 single-leg kinematics, IK validation, and workspace plots."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.a1_leg_model import A1LegKinematics
from common.io_utils import configure_matplotlib, ensure_results_dir


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--leg", default="FR", choices=["FR", "FL", "RR", "RL"])
    parser.add_argument("--samples", type=int, default=5000)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    configure_matplotlib()
    import matplotlib.pyplot as plt

    model = A1LegKinematics()
    out_dir = ensure_results_dir("task1_kinematics")

    q_samples, points = model.sample_workspace(args.samples, args.leg, args.seed)
    workspace_path = out_dir / f"{args.leg.lower()}_workspace_3d.png"
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1.5, alpha=0.45)
    ax.set_xlabel("x in base frame [m]")
    ax.set_ylabel("y in base frame [m]")
    ax.set_zlabel("z in base frame [m]")
    ax.set_title(f"A1 {args.leg} foot reachable workspace")
    fig.tight_layout()
    fig.savefig(workspace_path, dpi=180)
    plt.close(fig)

    xy_path = out_dir / f"{args.leg.lower()}_workspace_xz.png"
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.scatter(points[:, 0], points[:, 2], s=2.0, alpha=0.45)
    ax.set_xlabel("x in base frame [m]")
    ax.set_ylabel("z in base frame [m]")
    ax.set_title(f"A1 {args.leg} foot workspace projection")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(xy_path, dpi=180)
    plt.close(fig)

    rng = np.random.default_rng(args.seed + 1)
    validation_rows = []
    max_position_error = 0.0
    for index in range(100):
        q_true = rng.uniform(model.lower_limits, model.upper_limits)
        target, _ = model.forward_kinematics(q_true, args.leg)
        q_solution, error, iterations, converged = model.inverse_kinematics(
            target,
            seed=model.default_q,
            leg_name=args.leg,
            max_iters=120,
            tolerance=1e-8,
        )
        max_position_error = max(max_position_error, error)
        validation_rows.append(
            {
                "index": index,
                "target_x": target[0],
                "target_y": target[1],
                "target_z": target[2],
                "q_true_hip": q_true[0],
                "q_true_thigh": q_true[1],
                "q_true_calf": q_true[2],
                "q_ik_hip": q_solution[0],
                "q_ik_thigh": q_solution[1],
                "q_ik_calf": q_solution[2],
                "position_error_m": error,
                "iterations": iterations,
                "converged": converged,
            }
        )

    csv_path = out_dir / f"{args.leg.lower()}_ik_validation.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(validation_rows[0].keys()))
        writer.writeheader()
        writer.writerows(validation_rows)

    dh_rows = model.equivalent_dh_rows(args.leg)
    dh_path = out_dir / f"{args.leg.lower()}_urdf_equivalent_mdh_table.csv"
    with dh_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(dh_rows[0].keys()))
        writer.writeheader()
        writer.writerows(dh_rows)

    summary = {
        "leg": args.leg,
        "workspace_samples": args.samples,
        "joint_limits_rad": {
            "lower": model.lower_limits.tolist(),
            "upper": model.upper_limits.tolist(),
        },
        "equivalent_transform_rows": model.equivalent_dh_rows(args.leg),
        "max_ik_position_error_m": max_position_error,
        "outputs": {
            "workspace_3d": str(workspace_path),
            "workspace_xz": str(xy_path),
            "ik_validation_csv": str(csv_path),
            "urdf_equivalent_mdh_table_csv": str(dh_path),
        },
    }
    summary_path = out_dir / f"{args.leg.lower()}_task1_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
