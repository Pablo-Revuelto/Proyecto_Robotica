"""Gazebo piece manipulation via the `gz service` CLI.

Three operations used by MoveExecutor:
  * `attach_piece`    — no-op; physical carry is not modelled in sim
  * `detach_piece`    — no-op; piece is teleported by the caller before detach
  * `teleport_model`  — set_pose service to move a piece to world coordinates
  * `delete_model`    — remove a captured piece from the world

The DetachableJoint plugin requires static SDF-level configuration and cannot
be used for ad-hoc dynamic joints. For simulation correctness we teleport pieces
using the UserCommands /world/<world>/set_pose service instead.
"""

from __future__ import annotations

import subprocess


def _gz_call(service: str, reqtype: str, reptype: str, req: str,
             timeout_ms: int = 3000) -> bool:
    cmd = [
        "gz", "service", "-s", service,
        "--reqtype", reqtype, "--reptype", reptype,
        "--timeout", str(timeout_ms),
        "--req", req,
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    return res.returncode == 0 and "data: true" in res.stdout


def attach_piece(world: str, robot_model: str, robot_link: str,
                 piece_model: str) -> bool:
    """Simulate grasp: always succeeds (piece will be teleported on place)."""
    return True


def detach_piece(world: str, robot_model: str, robot_link: str,
                 piece_model: str) -> bool:
    """Simulate release: always succeeds."""
    return True


def teleport_model(world: str, model_name: str,
                   x: float, y: float, z: float) -> bool:
    """Move a Gazebo model to (x, y, z) via the UserCommands set_pose service."""
    req = (
        f'name: "{model_name}", '
        f'position: {{x: {x:.6f}, y: {y:.6f}, z: {z:.6f}}}, '
        f'orientation: {{x: 0, y: 0, z: 0, w: 1}}'
    )
    return _gz_call(
        f"/world/{world}/set_pose",
        "gz.msgs.Pose", "gz.msgs.Boolean", req,
    )


def delete_model(world: str, model_name: str) -> bool:
    """Remove a captured piece from the Gazebo world."""
    req = f'name: "{model_name}", type: 2'
    return _gz_call(
        f"/world/{world}/remove",
        "gz.msgs.Entity", "gz.msgs.Boolean", req,
    )
