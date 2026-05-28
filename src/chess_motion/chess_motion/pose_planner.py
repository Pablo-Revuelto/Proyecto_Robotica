"""Build sequences of TCP poses for picking and placing chess pieces.

Pose convention: poses are expressed in the world frame for the `gripper_tip`
link. The magnetic gripper points along the gripper_tip's +Z axis (already
oriented downwards thanks to the IRB120 `tool0` 90° pitch in the URDF macro).
We therefore request a top-down grasp where the gripper_tip frame has its
Z axis pointing -Z in world (i.e. orientation: roll=pi, pitch=0, yaw=...).

Phases per move:
    1. approach_from   — Above the source square, at `approach_clearance` m.
    2. grasp           — Just above the piece magnet, at `grasp_z`.
    3. lift            — Back up to approach_clearance after attach.
    4. approach_to     — Above the destination square.
    5. place           — At the destination piece top (board_z + piece_height).
    6. release         — After detach, back up to approach_clearance.
    7. retreat         — Move to the robot's `ready` group state.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

from chess_brain.board_geometry import BoardGeometry


@dataclass(frozen=True)
class GraspPose:
    """A 6-DoF pose for the gripper_tip in the world frame."""
    x: float
    y: float
    z: float
    roll: float = math.pi    # tip pointing -Z (top-down)
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class MotionPlan:
    approach_from: GraspPose
    grasp:         GraspPose
    lift:          GraspPose
    approach_to:   GraspPose
    place:         GraspPose
    release:       GraspPose


def build_plan(from_square: str, to_square: str,
               piece_height: float,
               geometry: BoardGeometry,
               approach_clearance: float = 0.10,
               grasp_clearance: float = 0.005) -> MotionPlan:
    fx, fy, _ = geometry.algebraic_to_world(from_square)
    tx, ty, _ = geometry.algebraic_to_world(to_square)

    grasp_z   = geometry.board_z + piece_height + grasp_clearance
    place_z   = geometry.board_z + piece_height + grasp_clearance
    above_z_f = grasp_z + approach_clearance
    above_z_t = place_z + approach_clearance

    return MotionPlan(
        approach_from=GraspPose(fx, fy, above_z_f),
        grasp        =GraspPose(fx, fy, grasp_z),
        lift         =GraspPose(fx, fy, above_z_f),
        approach_to  =GraspPose(tx, ty, above_z_t),
        place        =GraspPose(tx, ty, place_z),
        release      =GraspPose(tx, ty, above_z_t),
    )


# Heights per piece, mirrors chess_gazebo/config/board_layout.yaml.
DEFAULT_PIECE_HEIGHTS: Dict[str, float] = {
    "pawn": 0.045, "rook": 0.055, "knight": 0.060,
    "bishop": 0.065, "queen": 0.080, "king": 0.085,
}


def captured_square_offboard(captured_model: str, geometry: BoardGeometry,
                             side: str) -> Tuple[str, GraspPose]:
    """Pick a free off-board cell to dump a captured piece.

    A simple deterministic policy: white captures go to a slot on the +Y side
    of the board (operator's left), black captures to -Y. Within each side we
    pile pieces along +X / -X in 5 cm steps. Returns (slot_name, place pose).
    """
    sign = 1 if side == "white" else -1
    # Naive global counter via hash of model name to scatter pieces visually.
    slot_index = abs(hash(captured_model)) % 8
    x = (slot_index - 3.5) * geometry.square_size
    y = sign * (geometry.square_size * 6.0)
    pose = GraspPose(x, y, geometry.board_z + 0.05)
    return f"captured_{slot_index}", pose
