"""Pick-and-place executor for the dual IRB120 chess scene.

Exposes the `/chess/execute_move` action (chess_msgs/ExecuteChessMove).

Instead of owning its own MoveIt instance, this node is a *client* of the
already-running `move_group` (the `moveit_msgs/action/MoveGroup` action on
`/move_action`). For each goal it sends pose / joint goals with
`plan_only=False`, so move_group plans AND executes through the arm
controllers. This avoids running a second, redundant MoveIt core (MoveItPy)
alongside the one launched for RViz.

For each goal:
  1. Pick which arm executes the move (white pieces → white arm, etc.).
  2. Build the `MotionPlan` of TCP poses (see `pose_planner`).
  3. For each phase, send a pose goal to move_group (plan + execute).
  4. Between `grasp` and `lift`, attach the piece in Gazebo;
     between `place` and `release`, detach.
  5. For captures, the captured piece is removed before the place phase.

Feedback `phase` lets clients display progress.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from threading import Event

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Vector3
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint,
                             MoveItErrorCodes, OrientationConstraint,
                             PositionConstraint)

from chess_brain.board_geometry import BoardGeometry
from chess_msgs.action import ExecuteChessMove
from chess_msgs.msg import ChessMove

from .gazebo_attach import (PieceFollower, attach_piece, delete_model,
                            detach_piece, teleport_model)
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
        # Visual piece-follow offset (metres, world axes) added to the gripper
        # TCP while carrying a piece. The Z base is auto-set to -piece_height so
        # the piece's top magnet sits at the gripper magnet; these params nudge
        # it: piece too HIGH → make magnet_offset_z more negative; too LOW →
        # less negative.
        self.declare_parameter("magnet_offset_x", 0.0)
        self.declare_parameter("magnet_offset_y", 0.0)
        self.declare_parameter("magnet_offset_z", 0.0)
        # 25 Hz: smooth enough; combined with gravity-off pieces this avoids the
        # set_pose-vs-physics bouncing.
        self.declare_parameter("follow_rate_hz",  25.0)
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
        self._magnet_off = (
            float(self.get_parameter("magnet_offset_x").value),
            float(self.get_parameter("magnet_offset_y").value),
            float(self.get_parameter("magnet_offset_z").value),
        )
        self._follow_rate = float(self.get_parameter("follow_rate_hz").value)

        self._registry = PieceRegistry()
        self._registry.populate_from_initial_fen(
            self.get_parameter("initial_fen").get_parameter_value().string_value
        )

        self._group = {"white": "white_arm", "black": "black_arm"}
        self._tip   = {"white": "white_gripper_tip", "black": "black_gripper_tip"}
        # Named rest poses, read from the SRDF (e.g. "ready") for retreat moves.
        self._group_states = self._load_group_states()

        # Reuse the existing move_group instead of a second MoveIt core.
        self._move_group = ActionClient(self, MoveGroup, "/move_action")

        # TF, used to make a carried piece follow the gripper TCP (Option 3).
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._action = ActionServer(
            self, ExecuteChessMove, "/chess/execute_move", self._execute_cb)

        self.get_logger().info("Move executor ready, waiting for goals.")

    # ---- Setup ---------------------------------------------------------

    def _load_group_states(self) -> dict:
        """Parse <group_state> joint targets from the MoveIt SRDF."""
        states: dict = {}
        try:
            srdf = (Path(get_package_share_directory("chess_moveit_config"))
                    / "config" / "chess.srdf")
            root = ET.fromstring(srdf.read_text())
            for gs in root.iter("group_state"):
                key = (gs.get("group"), gs.get("name"))
                states[key] = {j.get("name"): float(j.get("value"))
                               for j in gs.findall("joint")}
        except Exception as exc:        # noqa: BLE001
            self.get_logger().warn(f"Could not load SRDF group states: {exc}")
        return states

    @staticmethod
    def _wait_for_future(future, timeout_sec: float) -> bool:
        """Block until `future` is done without re-spinning the node."""
        done = Event()
        future.add_done_callback(lambda _f: done.set())
        return done.wait(timeout_sec)

    # ---- Action handling ----------------------------------------------

    def _execute_cb(self, goal_handle):
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

        piece_height = DEFAULT_PIECE_HEIGHTS.get(piece, 0.05)
        plan = build_plan(
            from_square=from_sq,
            to_square=to_sq,
            piece_height=piece_height,
            geometry=self._geometry,
            approach_clearance=self._approach,
            grasp_clearance=self._grasp_c,
        )

        follower = None
        try:
            emit("approach_from", 0.10); self._goto(color, plan.approach_from)
            emit("grasp",         0.25); self._goto(color, plan.grasp)

            ok = attach_piece(self._world, "dual_irb120_chess",
                              f"{color}_gripper_tip", piece_model)
            if not ok:
                raise RuntimeError(f"Failed to attach {piece_model}")

            # Option 3: the piece visually follows the gripper TCP during the
            # carry. Base Z offset = -piece_height (top magnet at the gripper),
            # plus the configurable magnet_offset_* nudge.
            ox, oy, oz = self._magnet_off
            follow_offset = (ox, oy, oz - piece_height)
            self.get_logger().info(
                f"Follow START: {piece_model} -> {color}_gripper_tip "
                f"@ {self._follow_rate:.0f} Hz, offset={follow_offset} "
                f"(z base=-piece_height={-piece_height:.3f}+{oz:.3f}), "
                f"orientation=fixed upright")
            follower = PieceFollower(
                self._tf_buffer, self.get_logger(), self._world, piece_model,
                f"{color}_gripper_tip",
                offset=follow_offset, rate_hz=self._follow_rate)
            follower.start()

            emit("lift",          0.40); self._goto(color, plan.lift)

            if move.is_capture:
                captured = self._registry.remove(to_sq)
                if captured and not delete_model(self._world, captured):
                    self.get_logger().warn(
                        f"Failed to delete captured model {captured}")

            emit("transport",     0.55); self._goto(color, plan.approach_to)
            emit("place",         0.70); self._goto(color, plan.place)

            # Stop following, then snap the piece to the exact destination square.
            follower.stop(); follower = None
            self.get_logger().info(
                f"Follow STOP: snapping {piece_model} to {to_sq} "
                f"({plan.place.x:.3f}, {plan.place.y:.3f}, {self._geometry.board_z:.3f})")
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
        finally:
            # Never leave the follower thread running if the move ended early.
            if follower is not None:
                follower.stop()

    # ---- move_group client --------------------------------------------

    def _goto(self, color: str, pose: GraspPose) -> None:
        """Plan + execute a TCP pose goal for the given arm via move_group."""
        target = self._pose_from_grasp(pose)
        constraints = self._pose_goal(self._tip[color], target)
        self._send_goal(color, constraints,
                        f"{color} pose ({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f})")

    def _go_named(self, color: str, state_name: str) -> None:
        """Best-effort move to a named SRDF state (e.g. 'ready')."""
        joints = self._group_states.get((self._group[color], state_name))
        if not joints:
            self.get_logger().warn(
                f"No SRDF state '{state_name}' for {color}_arm; skipping.")
            return
        c = Constraints()
        for jname, val in joints.items():
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        try:
            self._send_goal(color, c, f"{color} {state_name}")
        except RuntimeError as exc:
            self.get_logger().warn(f"Retreat to '{state_name}' failed: {exc}")

    def _send_goal(self, color: str, constraints: Constraints, label: str) -> None:
        if not self._move_group.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("move_group action server /move_action unavailable")

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self._group[color]
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.goal_constraints.append(constraints)
        req.workspace_parameters.header.frame_id = "world"
        req.workspace_parameters.min_corner.x = -2.0
        req.workspace_parameters.min_corner.y = -2.0
        req.workspace_parameters.min_corner.z = -2.0
        req.workspace_parameters.max_corner.x = 2.0
        req.workspace_parameters.max_corner.y = 2.0
        req.workspace_parameters.max_corner.z = 2.0

        # plan AND execute through move_group's controllers; start from current.
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        send_future = self._move_group.send_goal_async(goal)
        if not self._wait_for_future(send_future, 15.0):
            raise RuntimeError(f"{label}: timed out sending goal")
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{label}: goal rejected by move_group")

        result_future = handle.get_result_async()
        if not self._wait_for_future(result_future, 120.0):
            raise RuntimeError(f"{label}: timed out waiting for execution")
        code = result_future.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"{label}: move_group failed (error code {code})")

    @staticmethod
    def _pose_goal(tip_link: str, pose: Pose) -> Constraints:
        """A position + orientation constraint set for a TCP pose goal."""
        c = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = "world"
        pc.link_name = tip_link
        pc.target_point_offset = Vector3()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]          # 1 cm tolerance
        region = BoundingVolume()
        region.primitives.append(sphere)
        region_pose = Pose()
        region_pose.position = pose.position
        region_pose.orientation.w = 1.0
        region.primitive_poses.append(region_pose)
        pc.constraint_region = region
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = "world"
        oc.link_name = tip_link
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        return c

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
