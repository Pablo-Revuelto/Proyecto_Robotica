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

Perception cross-check (advisory, silent by default): when a `BoardState`
arrives from `/chess/perceived_state`, validate ONLY the squares perception
actually reports a piece on (`pieces[idx] != 0`). A non-detected square is
never treated as empty, and the full FEN is never compared. A rate-limited
warning is emitted only when a high-confidence detection contradicts an
occupied engine square, and only when enough reliable detections are present.
Perception is observation, not authority -- the engine remains the source of
truth and is never modified, and execution is never blocked.
"""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rcl_interfaces.msg import (FloatingPointRange, IntegerRange,
                                ParameterDescriptor)
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


# FEN piece letter -> ChessMove.PIECE_* code (color carried separately by case).
_FEN_PIECE_CODE = {
    "p": ChessMove.PIECE_PAWN,   "n": ChessMove.PIECE_KNIGHT,
    "b": ChessMove.PIECE_BISHOP, "r": ChessMove.PIECE_ROOK,
    "q": ChessMove.PIECE_QUEEN,  "k": ChessMove.PIECE_KING,
}
# Inverse of the BoardState piece encoding, for human-readable log messages.
_CODE_PIECE_NAME = {
    ChessMove.PIECE_PAWN: "pawn",     ChessMove.PIECE_KNIGHT: "knight",
    ChessMove.PIECE_BISHOP: "bishop", ChessMove.PIECE_ROOK: "rook",
    ChessMove.PIECE_QUEEN: "queen",   ChessMove.PIECE_KING: "king",
}
_BLACK_BIT = 0x80


class GameManager(Node):

    def __init__(self) -> None:
        super().__init__("chess_game_manager")
        self.declare_parameter("square_size", 0.05)
        self.declare_parameter("board_z", 0.02)
        self.declare_parameter("board_centre_x", 0.0)
        self.declare_parameter("board_centre_y", 0.0)
        self.declare_parameter("initial_fen", "")

        # Advisory perception validator (silent by default). See module docstring.
        self.declare_parameter(
            "perception_validation_enabled", True,
            ParameterDescriptor(
                description="Master switch for advisory perception validation."))
        self.declare_parameter(
            "perception_validation_confidence", 0.80,
            ParameterDescriptor(
                description="Min board confidence before a perception warning.",
                floating_point_range=[FloatingPointRange(
                    from_value=0.0, to_value=1.0, step=0.0)]))
        self.declare_parameter(
            "perception_min_detections", 2,
            ParameterDescriptor(
                description="Min reliable detections before a perception warning.",
                integer_range=[IntegerRange(
                    from_value=0, to_value=64, step=1)]))
        self.declare_parameter(
            "perception_warn_cooldown_sec", 10.0,
            ParameterDescriptor(
                description="Per-square cooldown (s) between perception warnings.",
                floating_point_range=[FloatingPointRange(
                    from_value=0.0, to_value=3600.0, step=0.0)]))
        self.declare_parameter(
            "perception_warn_on_empty_engine", False,
            ParameterDescriptor(
                description="Warn when engine is empty but perception detects a "
                            "piece (default off: treated as likely false positive)."))
        self.declare_parameter(
            "perception_log_matches", False,
            ParameterDescriptor(
                description="Emit a debug log when a detection agrees with the engine."))

        self._perc_enabled = bool(
            self.get_parameter("perception_validation_enabled").value)
        self._perc_conf = float(
            self.get_parameter("perception_validation_confidence").value)
        self._perc_min_det = int(
            self.get_parameter("perception_min_detections").value)
        self._perc_cooldown = float(
            self.get_parameter("perception_warn_cooldown_sec").value)
        self._perc_warn_on_empty = bool(
            self.get_parameter("perception_warn_on_empty_engine").value)
        self._perc_log_matches = bool(
            self.get_parameter("perception_log_matches").value)
        self._perc_last_warn: dict = {}   # square index -> last warn time (s)

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
        """Advisory perception cross-check. Never blocks, never mutates state.

        Only squares perception actually reports a piece on are considered
        (``pieces[idx] != 0``); a non-detected square is never assumed empty,
        and the full FEN is never compared. A contradiction warning is emitted
        only when (a) enough reliable detections are present, (b) board
        confidence clears the threshold, and (c) a detected piece disagrees
        with an *occupied* engine square -- rate-limited per square.
        """
        if not self._perc_enabled:
            return

        # Squares perception is confident a piece sits on. 0 == not detected,
        # which we never treat as "empty".
        detections = [(idx, b) for idx, b in enumerate(msg.pieces) if b != 0]
        if not detections:
            return

        engine_codes = self._engine_occupancy()
        contradictions = []
        for idx, perceived in detections:
            engine = engine_codes.get(idx, 0)
            if engine == perceived:
                if self._perc_log_matches:
                    self.get_logger().debug(
                        f"Perception agrees on {self._square_name(idx)}: "
                        f"{self._describe_code(perceived)}")
                continue
            if engine == 0:
                # Engine empty but perception sees something -> likely a false
                # positive. Off by default; debug only.
                if self._perc_warn_on_empty:
                    contradictions.append((idx, engine, perceived))
                else:
                    self.get_logger().debug(
                        f"Perception sees {self._describe_code(perceived)} on "
                        f"{self._square_name(idx)} where engine is empty "
                        f"(ignored).")
                continue
            # Same square, occupied by both, but different piece/colour.
            contradictions.append((idx, engine, perceived))

        if not contradictions:
            return

        # Gate warnings on detection count and board confidence (debug only
        # when suppressed -- never an error, never blocking).
        if len(detections) < self._perc_min_det:
            self.get_logger().debug(
                f"Perception contradiction suppressed: only {len(detections)} "
                f"detection(s) (< {self._perc_min_det}).")
            return
        if msg.confidence < self._perc_conf:
            self.get_logger().debug(
                f"Perception contradiction suppressed: confidence "
                f"{msg.confidence:.2f} < {self._perc_conf:.2f}.")
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        for idx, engine, perceived in contradictions:
            last = self._perc_last_warn.get(idx)
            if last is not None and (now - last) < self._perc_cooldown:
                continue
            self._perc_last_warn[idx] = now
            self.get_logger().warn(
                f"Perception disagrees on {self._square_name(idx)}: "
                f"engine={self._describe_code(engine)}, "
                f"perception={self._describe_code(perceived)} "
                f"(conf={msg.confidence:.2f}). Engine remains authority.")

    # ---- Perception helpers -------------------------------------------

    def _engine_occupancy(self) -> dict:
        """Engine FEN placement -> {square index: BoardState piece byte}.

        Square index matches the perception encoding: ``rank * 8 + file`` with
        rank 0 = rank "1" and file 0 = file "a". Only occupied squares appear.
        """
        placement = self._engine.fen().split(" ", 1)[0]
        codes: dict = {}
        rank = 7  # FEN lists rank 8 first; index 7 is rank "8".
        for row in placement.split("/"):
            file = 0
            for ch in row:
                if ch.isdigit():
                    file += int(ch)
                    continue
                code = _FEN_PIECE_CODE.get(ch.lower())
                if code is None:
                    continue
                if ch.islower():
                    code |= _BLACK_BIT
                codes[rank * 8 + file] = code
                file += 1
            rank -= 1
        return codes

    @staticmethod
    def _square_name(idx: int) -> str:
        return f"{chr(ord('a') + (idx % 8))}{(idx // 8) + 1}"

    @staticmethod
    def _describe_code(byte: int) -> str:
        if byte == 0:
            return "empty"
        colour = "black" if (byte & _BLACK_BIT) else "white"
        name = _CODE_PIECE_NAME.get(byte & ~_BLACK_BIT, "?")
        return f"{colour} {name}"

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
