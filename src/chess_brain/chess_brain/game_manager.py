"""Top-level game orchestrator.

Responsibilities (SRP):
  - Maintain authoritative game state via `ChessEngine`.
  - On a transcribed utterance (topic `/chess/voice/utterance`), call the
    voice parser service (`/chess/voice/parse`) to obtain a candidate move.
  - Validate it against the engine.
  - Dispatch a validated `ChessMove` to the `chess_motion` action server
    (`/chess/execute_move`).
  - Publish the current board state (`/chess/board_state`) for UIs and
    perception sanity-checking.

Perception cross-check: if a `BoardState` arrives from `/chess/perceived_state`
and disagrees with the engine, log a warning. Perception is observation, not
authority -- the engine remains the source of truth.
"""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from chess_msgs.action import ExecuteChessMove
from chess_msgs.msg import BoardState, ChessMove
from chess_msgs.srv import ParseVoiceCommand

from .board_geometry import BoardGeometry
from .chess_engine import PythonChessEngine
from .msg_conversions import parsed_to_msg


class GameManager(Node):

    def __init__(self) -> None:
        super().__init__("chess_game_manager")
        self.declare_parameter("square_size", 0.05)
        self.declare_parameter("board_z", 0.02)
        self.declare_parameter("board_centre_x", 0.0)
        self.declare_parameter("board_centre_y", 0.0)
        self.declare_parameter("initial_fen", "")

        ss = self.get_parameter("square_size").value
        bz = self.get_parameter("board_z").value
        cx = self.get_parameter("board_centre_x").value
        cy = self.get_parameter("board_centre_y").value
        fen = self.get_parameter("initial_fen").get_parameter_value().string_value

        self._geometry = BoardGeometry(square_size=ss, board_z=bz, centre=(cx, cy))
        self._engine = PythonChessEngine(fen or None)
        self._lock = threading.Lock()
        self._busy = False

        self._board_pub = self.create_publisher(BoardState, "/chess/board_state", 1)
        self._utt_sub = self.create_subscription(
            String, "/chess/voice/utterance", self._on_utterance, 5)
        self._perceived_sub = self.create_subscription(
            BoardState, "/chess/perceived_state", self._on_perceived, 5)

        self._parse_client = self.create_client(
            ParseVoiceCommand, "/chess/voice/parse")
        self._motion_client = ActionClient(
            self, ExecuteChessMove, "/chess/execute_move")

        self._publish_state()
        self.get_logger().info("Game manager up. Waiting for voice utterances.")

    # ---- Inputs --------------------------------------------------------

    def _on_utterance(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        with self._lock:
            if self._busy:
                self.get_logger().warn(f"Busy; dropping utterance: {text!r}")
                return
            self._busy = True
        # Run the blocking pipeline in a daemon thread so the ROS executor
        # remains free to dispatch service/action responses (avoids deadlock
        # when spin_until_future_complete is called from within a callback).
        threading.Thread(
            target=self._handle_utterance_safe,
            args=(text,),
            daemon=True,
        ).start()

    def _handle_utterance_safe(self, text: str) -> None:
        try:
            self._handle_utterance(text)
        finally:
            with self._lock:
                self._busy = False

    def _on_perceived(self, msg: BoardState) -> None:
        if msg.fen and msg.fen.split(" ")[0] != self._engine.fen().split(" ")[0]:
            self.get_logger().warn(
                f"Perception disagrees with engine: perception={msg.fen}, "
                f"engine={self._engine.fen()}")

    # ---- Pipeline ------------------------------------------------------

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        """Block until `future` is done WITHOUT spinning this node.

        The pipeline runs in a worker thread while main() already spins the
        node; calling rclpy.spin_until_future_complete(self, ...) here re-spins
        the same node from a second thread, which is racy and intermittently
        never completes the future ("Voice parser timed out"). Waiting on a
        done-callback lets the main executor deliver the result instead.
        """
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        return done.wait(timeout_sec)

    def _handle_utterance(self, text: str) -> None:
        if not self._parse_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Voice parser service unavailable.")
            return

        req = ParseVoiceCommand.Request()
        req.utterance = text
        req.board_state = self._current_state_msg()

        future = self._parse_client.call_async(req)
        if not self._wait_for_future(future, 10.0) or future.result() is None:
            self.get_logger().error("Voice parser timed out.")
            return
        result: ParseVoiceCommand.Response = future.result()
        if not result.success:
            self.get_logger().warn(f"Parse failed: {result.error}")
            return

        parsed = self._engine.validate(result.move.uci)
        if parsed is None:
            self.get_logger().warn(
                f"Illegal move from voice ({result.move.uci}); ignoring.")
            return

        chess_move = parsed_to_msg(parsed, self._geometry)
        self.get_logger().info(f"Dispatching move: {parsed.san} ({parsed.uci})")
        self._dispatch(chess_move, parsed_to_apply=parsed)

    def _dispatch(self, msg: ChessMove, parsed_to_apply) -> None:
        if not self._motion_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Motion action server unavailable.")
            return
        goal = ExecuteChessMove.Goal()
        goal.move = msg
        future = self._motion_client.send_goal_async(goal)
        if not self._wait_for_future(future, 5.0):
            self.get_logger().error("Motion goal timed out.")
            return
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("Motion goal rejected.")
            return
        result_future = handle.get_result_async()
        if not self._wait_for_future(result_future, 120.0):
            self.get_logger().error("Motion result timed out.")
            return
        res = result_future.result()
        if res is None or not res.result.success:
            err = res.result.error if res else "no result"
            self.get_logger().error(f"Motion execution failed: {err}")
            return

        self._engine.apply(parsed_to_apply)
        self._publish_state()
        self.get_logger().info(f"Move applied. Turn: {self._engine.turn()}")

    # ---- State publishing ---------------------------------------------

    def _current_state_msg(self) -> BoardState:
        msg = BoardState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.fen = self._engine.fen()
        msg.turn = (ChessMove.COLOR_WHITE if self._engine.turn() == "white"
                    else ChessMove.COLOR_BLACK)
        msg.confidence = 1.0
        # NOTE: msg.pieces (uint8[64]) intentionally left zeroed for now; the
        # FEN string is the authoritative encoding consumed downstream.
        return msg

    def _publish_state(self) -> None:
        self._board_pub.publish(self._current_state_msg())


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = GameManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
