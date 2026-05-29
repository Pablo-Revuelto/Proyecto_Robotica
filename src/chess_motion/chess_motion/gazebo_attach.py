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
import threading

from rclpy.time import Time


def _gz_call(service: str, reqtype: str, reptype: str, req: str,
             timeout_ms: int = 3000) -> bool:
    cmd = [
        "gz", "service", "-s", service,
        "--reqtype", reqtype, "--reptype", reptype,
        "--timeout", str(timeout_ms),
        "--req", req,
    ]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True,
                             timeout=timeout_ms / 1000.0 + 1.0)
    except subprocess.TimeoutExpired:
        return False
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
                   x: float, y: float, z: float,
                   timeout_ms: int = 3000) -> bool:
    """Move a Gazebo model to (x, y, z) via the UserCommands set_pose service."""
    req = (
        f'name: "{model_name}", '
        f'position: {{x: {x:.6f}, y: {y:.6f}, z: {z:.6f}}}, '
        f'orientation: {{x: 0, y: 0, z: 0, w: 1}}'
    )
    return _gz_call(
        f"/world/{world}/set_pose",
        "gz.msgs.Pose", "gz.msgs.Boolean", req, timeout_ms=timeout_ms,
    )


def delete_model(world: str, model_name: str) -> bool:
    """Remove a captured piece from the Gazebo world."""
    req = f'name: "{model_name}", type: 2'
    return _gz_call(
        f"/world/{world}/remove",
        "gz.msgs.Entity", "gz.msgs.Boolean", req,
    )


class PieceFollower:
    """Make a Gazebo piece visually follow a robot TCP frame during transport.

    Runs in its OWN daemon thread, so it never blocks the ROS executor. Each
    tick it reads the latest `world -> tip_frame` transform from the given tf2
    buffer and `set_pose`s the piece there, plus a configurable `offset`
    (metres, world axes) so the piece's top magnet sits at the gripper magnet.
    Orientation is kept upright (identity). Start/stop are explicit and the
    thread is joined on stop, so nothing is left running if a move fails.

    Tuning: if the carried piece looks too HIGH, make offset z more negative;
    too LOW (sinking into the gripper), make it less negative. See
    `magnet_offset_z` in move_executor_node.py.
    """

    def __init__(self, tf_buffer, logger, world: str, model_name: str,
                 tip_frame: str, offset=(0.0, 0.0, 0.0),
                 rate_hz: float = 10.0, gz_timeout_ms: int = 800) -> None:
        self._tf_buffer = tf_buffer
        self._logger = logger
        self._world = world
        self._model = model_name
        self._tip = tip_frame
        self._ox, self._oy, self._oz = offset
        self._period = 1.0 / max(rate_hz, 1.0)
        self._gz_timeout_ms = gz_timeout_ms
        self._stop = threading.Event()
        self._thread = None

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        if self._logger is not None:
            self._logger.info(
                f"PieceFollower: following {self._model} -> {self._tip} "
                f"@ {1.0 / self._period:.0f} Hz, offset="
                f"({self._ox:.3f}, {self._oy:.3f}, {self._oz:.3f}), "
                f"orientation=fixed upright")

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                tf = self._tf_buffer.lookup_transform("world", self._tip, Time())
                if self._stop.is_set():
                    break
                t = tf.transform.translation
                teleport_model(self._world, self._model,
                               t.x + self._ox, t.y + self._oy, t.z + self._oz,
                               timeout_ms=self._gz_timeout_ms)
            except Exception:  # noqa: BLE001 — TF not ready / transient; retry next tick
                pass
            self._stop.wait(self._period)

    def stop(self) -> None:
        self._stop.set()
        t = self._thread
        self._thread = None
        if t is not None and t.is_alive():
            t.join(timeout=3.0)
            if t.is_alive() and self._logger is not None:
                self._logger.warn(
                    f"PieceFollower: thread for {self._model} did not stop in 3s")
        if self._logger is not None:
            self._logger.info(f"PieceFollower: stopped following {self._model}")
