# Source Code Extract for Course Submission

This directory contains the A1-related source-code subset used by the course-design experiments. It is intentionally not a full copy of the upstream `rl_sar` repository.

## Included Components

| Path | Purpose |
| --- | --- |
| `a1_description/` | A1 URDF, xacro, meshes, ROS 2 control config, Gazebo/RViz launch files. |
| `a1_policy/` | A1 policy configuration and `legged_gym/model.pt` used by the RL locomotion controller. |
| `rl_sar_a1_runtime/` | A1-specific runtime/control sources used by the Gazebo/Nav2 experiments. |
| `robot_joint_controller/` | ROS 2 joint controller source that receives `RobotCommand` and writes joint effort commands. |
| `robot_msgs/` | ROS message definitions used by the joint controller and RL runtime. |

## Course-Relevant Control Chain

The full simulation-control pipeline used in the report is:

```text
Nav2 /cmd_vel
  -> rl_sar_a1_runtime/src/rl_sim.cpp
  -> A1 legged_gym policy in a1_policy/
  -> robot_msgs/RobotCommand
  -> robot_joint_controller
  -> Gazebo A1 URDF model from a1_description/
```

The Nav2 goal-evaluation pipeline also uses:

```text
rl_sar_a1_runtime/scripts/gazebo_ground_truth_odom.py
rl_sar_a1_runtime/scripts/evaluate_nav2_goal_sequence.py
rl_sar_a1_runtime/config/a1_nav2_ground_truth_refined.yaml
rl_sar_a1_runtime/maps/a1_nav_world_empty.yaml
```

## Scope

Only A1/course-design related files are included here. Other robots, hardware SDKs, large third-party robot libraries, build folders, logs, and cache files are intentionally excluded.

This subset is provided for grading transparency: it shows the URDF model, control node, message/controller interface, policy configuration, and evaluation scripts used by the course-design report.
