"""Pick-and-place executor for the dual IRB120 chess scene.

Exposes the `/chess/execute_move` action (chess_msgs/ExecuteChessMove).

Internally it owns a `MoveItPy` instance with two planning components
(`white_arm`, `black_arm`), the piece registry, and the Gazebo attach/detach
helpers. For each goal:
  1. Pick which arm executes the move (white pieces → white arm, etc.).
  2. Build the `MotionPlan` of TCP poses (see `pose_planner`).
  3. For each phase, plan + execute a Cartesian-ish goal through MoveIt.
  4. Between `grasp` and `lift`, attach the piece in Gazebo;
     between `place` and `release`, detach.
  5. For captures, the captured piece is removed before the place phase.

Feedback `phase` lets clients display progress.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from chess_brain.board_geometry import BoardGeometry
from chess_msgs.action import ExecuteChessMove
from chess_msgs.msg import ChessMove

from .gazebo_attach import (attach_piece, delete_model, detach_piece,
                            teleport_model)
from .piece_registry import PieceRegistry
from .pose_planner import (DEFAULT_PIECE_HEIGHTS, GraspPose, MotionPlan,
                           build_plan, captured_square_offboard)


_PIECE_FROM_CODE = {
    ChessMove.PIECE_PAWN:   "pawn",
    ChessMove.PIECE_KNIGHT: "knight",
    ChessMove.PIECE_BISHOP: "bishop",
    ChessMove.PIECE_ROOK:   "rook",
    ChessMove.PIECE_QUEEN:  "queen",
    ChessMove.PIECE_KING:   "king",
}


def _euler_to_quat(roll: float, pitch: float, yaw: float):
    # Avoid pulling tf_transformations into the top-level imports; use a
    # local quaternion-from-euler so the node loads even if that package
    # isn't installed.
    cy = math.cos(yaw * 0.5);   sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class MoveExecutor(Node):

    def __init__(self) -> None:
        super().__init__("chess_move_executor")
        self.declare_parameter("world",          "chess_world")
        self.declare_parameter("square_size",    0.05)
        self.declare_parameter("board_z",        0.02)
        self.declare_parameter("approach_clearance", 0.10)
        self.declare_parameter("grasp_clearance",    0.005)
        self.declare_parameter("initial_fen",
                               "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1")

        self._world = self.get_parameter("world").get_parameter_value().string_value
        self._geometry = BoardGeometry(
            square_size=float(self.get_parameter("square_size").value),
            board_z=float(self.get_parameter("board_z").value),
            centre=(0.0, 0.0),
        )
        self._approach = float(self.get_parameter("approach_clearance").value)
        self._grasp_c  = float(self.get_parameter("grasp_clearance").value)

        self._registry = PieceRegistry()
        self._registry.populate_from_initial_fen(
            self.get_parameter("initial_fen").get_parameter_value().string_value
        )

        # Lazy MoveItPy initialisation: defer the heavy MoveIt import so the
        # node can announce the action server quickly, and so missing MoveIt
        # at import-time produces a clearer error.
        self._moveit = None
        self._arms = {}

        self._action = ActionServer(
            self, ExecuteChessMove, "/chess/execute_move", self._execute_cb)

        self.get_logger().info("Move executor ready, waiting for goals.")

    # ---- MoveIt setup --------------------------------------------------

    def _ensure_moveit(self) -> None:
        if self._moveit is not None:
            return
        from moveit.planning import MoveItPy
        self._moveit = MoveItPy(node_name="chess_moveit_py")
        self._arms = {
            "white": self._moveit.get_planning_component("white_arm"),
            "black": self._moveit.get_planning_component("black_arm"),
        }

    # ---- Action handling ----------------------------------------------

    def _execute_cb(self, goal_handle):
        self._ensure_moveit()

        move: ChessMove = goal_handle.request.move
        color = "white" if move.color == ChessMove.COLOR_WHITE else "black"
        piece = _PIECE_FROM_CODE.get(move.piece, "pawn")
        from_sq = move.from_square.algebraic
        to_sq   = move.to_square.algebraic

        self.get_logger().info(
            f"Executing {color} {piece} {from_sq}→{to_sq} (UCI={move.uci})")

        feedback = ExecuteChessMove.Feedback()
        def emit(phase: str, progress: float) -> None:
            feedback.phase = phase
            feedback.progress = progress
            goal_handle.publish_feedback(feedback)

        result = ExecuteChessMove.Result()
        result.success = False

        piece_model = self._registry.model_at(from_sq)
        if piece_model is None:
            result.error = f"No piece registered at {from_sq}"
            goal_handle.abort()
            return result

        plan = build_plan(
            from_square=from_sq,
            to_square=to_sq,
            piece_height=DEFAULT_PIECE_HEIGHTS.get(piece, 0.05),
            geometry=self._geometry,
            approach_clearance=self._approach,
            grasp_clearance=self._grasp_c,
        )

        try:
            emit("approach_from", 0.10); self._goto(color, plan.approach_from)
            emit("grasp",         0.25); self._goto(color, plan.grasp)

            ok = attach_piece(self._world, "dual_irb120_chess",
                              f"{color}_gripper_tip", piece_model)
            if not ok:
                raise RuntimeError(f"Failed to attach {piece_model}")

            emit("lift",          0.40); self._goto(color, plan.lift)

            if move.is_capture:
                captured = self._registry.remove(to_sq)
                if captured and not delete_model(self._world, captured):
                    self.get_logger().warn(
                        f"Failed to delete captured model {captured}")

            emit("transport",     0.55); self._goto(color, plan.approach_to)
            emit("place",         0.70); self._goto(color, plan.place)

            # Teleport the piece to the destination square so it appears placed.
            teleport_model(self._world, piece_model,
                           plan.place.x, plan.place.y, self._geometry.board_z)

            ok = detach_piece(self._world, "dual_irb120_chess",
                              f"{color}_gripper_tip", piece_model)
            if not ok:
                raise RuntimeError(f"Failed to detach {piece_model}")

            emit("release",       0.85); self._goto(color, plan.release)
            self._registry.apply_move(from_sq, to_sq)

            emit("retreat",       0.95); self._go_named(color, "ready")

            emit("done", 1.0)
            result.success = True
            result.error = ""
            goal_handle.succeed()
            return result

        except Exception as exc:        # noqa: BLE001
            self.get_logger().error(f"Motion failed: {exc}")
            result.error = str(exc)
            goal_handle.abort()
            return result

    # ---- MoveIt helpers -----------------------------------------------

    def _goto(self, color: str, pose: GraspPose) -> None:
        arm = self._arms[color]
        target = PoseStamped()
        target.header.frame_id = "world"
        target.pose = self._pose_from_grasp(pose)
        tip_link = f"{color}_gripper_tip"

        arm.set_start_state_to_current_state()
        arm.set_goal_state(pose_stamped_msg=target, pose_link=tip_link)
        plan_result = arm.plan()
        if not plan_result:
            raise RuntimeError(
                f"Planning failed for {color} arm at "
                f"({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f})")
        self._moveit.execute(plan_result.trajectory, controllers=[])

    def _go_named(self, color: str, state_name: str) -> None:
        arm = self._arms[color]
        arm.set_start_state_to_current_state()
        arm.set_goal_state(configuration_name=state_name)
        plan_result = arm.plan()
        if plan_result:
            self._moveit.execute(plan_result.trajectory, controllers=[])

    @staticmethod
    def _pose_from_grasp(pose: GraspPose) -> Pose:
        p = Pose()
        p.position.x = pose.x; p.position.y = pose.y; p.position.z = pose.z
        qx, qy, qz, qw = _euler_to_quat(pose.roll, pose.pitch, pose.yaw)
        p.orientation.x = qx; p.orientation.y = qy
        p.orientation.z = qz; p.orientation.w = qw
        return p


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = MoveExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
