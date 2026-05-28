"""Estimate the board state from the overhead camera and publish it.

Pipeline:
  /overhead_camera/image  →  (cv_bridge)  →  PieceDetector
                                           ↓
                              [ list of PieceObservation ]
                                           ↓
                              IntrinsicHomography (px → world XY)
                                           ↓
                              BoardGeometry.world_to_square (XY → file/rank)
                                           ↓
                              Reduce conflicts (one piece per square: keep
                              highest-confidence detection)
                                           ↓
                              Compose BoardState (FEN + pieces[64])
                                           ↓
                  /chess/perceived_state  (chess_msgs/BoardState)

Backends:
  - YoloPieceDetector: production path. Provide weights via parameters.
  - NullDetector: emits an empty board state (so launch files don't break
    while the model is being trained).
"""

from __future__ import annotations

import math
from typing import Dict, List, Tuple

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from chess_brain.board_geometry import BoardGeometry
from chess_msgs.msg import BoardState, ChessMove

from .detectors import (NullDetector, PieceDetector, PieceObservation,
                        YoloPieceDetector)
from .homography import IntrinsicHomography


_PIECE_CODE = {
    "pawn":   ChessMove.PIECE_PAWN,
    "knight": ChessMove.PIECE_KNIGHT,
    "bishop": ChessMove.PIECE_BISHOP,
    "rook":   ChessMove.PIECE_ROOK,
    "queen":  ChessMove.PIECE_QUEEN,
    "king":   ChessMove.PIECE_KING,
}
_FEN_LETTER = {
    "pawn": "p", "knight": "n", "bishop": "b",
    "rook": "r", "queen": "q", "king": "k",
}


class BoardStateEstimator(Node):

    def __init__(self) -> None:
        super().__init__("chess_board_state_estimator")
        self.declare_parameter("yolo_weights",   "")
        self.declare_parameter("conf_threshold", 0.4)

        self.declare_parameter("camera_height", 1.2)
        self.declare_parameter("hfov",          1.2)
        self.declare_parameter("image_width",   1280)
        self.declare_parameter("image_height",  720)

        self.declare_parameter("square_size",     0.05)
        self.declare_parameter("board_z",         0.02)
        self.declare_parameter("board_centre_x",  0.0)
        self.declare_parameter("board_centre_y",  0.0)

        self.declare_parameter("publish_rate", 5.0)

        weights = self.get_parameter("yolo_weights").get_parameter_value().string_value
        conf = float(self.get_parameter("conf_threshold").value)
        self._detector: PieceDetector = (
            YoloPieceDetector(weights=weights, conf_threshold=conf)
            if weights else NullDetector()
        )
        if isinstance(self._detector, NullDetector):
            self.get_logger().warn(
                "No YOLO weights configured: publishing empty board states. "
                "Set `yolo_weights:=/path/to/best.pt` once trained.")

        self._homography = IntrinsicHomography(
            camera_height=float(self.get_parameter("camera_height").value),
            hfov_rad=float(self.get_parameter("hfov").value),
            image_width=int(self.get_parameter("image_width").value),
            image_height=int(self.get_parameter("image_height").value),
        )
        self._geometry = BoardGeometry(
            square_size=float(self.get_parameter("square_size").value),
            board_z=float(self.get_parameter("board_z").value),
            centre=(float(self.get_parameter("board_centre_x").value),
                    float(self.get_parameter("board_centre_y").value)),
        )

        self._bridge = CvBridge()
        self._last_obs: List[PieceObservation] = []

        self._sub = self.create_subscription(
            Image, "/overhead_camera/image", self._on_image, 1)
        self._pub = self.create_publisher(BoardState, "/chess/perceived_state", 1)

        period = 1.0 / float(self.get_parameter("publish_rate").value)
        self._timer = self.create_timer(period, self._publish_state)

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:        # noqa: BLE001
            self.get_logger().error(f"cv_bridge error: {exc}")
            return
        try:
            self._last_obs = self._detector.detect(frame)
        except Exception as exc:        # noqa: BLE001
            self.get_logger().error(f"Detector error: {exc}")
            self._last_obs = []

    def _publish_state(self) -> None:
        per_square: Dict[Tuple[int, int], PieceObservation] = {}
        for obs in self._last_obs:
            x, y = self._homography.pixel_to_world(obs.cx, obs.cy)
            file, rank = self._geometry.world_to_square(x, y)
            if not (0 <= file < 8 and 0 <= rank < 8):
                continue
            existing = per_square.get((file, rank))
            if existing is None or obs.confidence > existing.confidence:
                per_square[(file, rank)] = obs

        msg = BoardState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pieces = [0] * 64
        for (file, rank), obs in per_square.items():
            idx = rank * 8 + file
            code = _PIECE_CODE[obs.piece]
            if obs.color == "black":
                code |= 0x80
            msg.pieces[idx] = code
        msg.fen = self._build_fen(per_square)
        msg.confidence = self._average_confidence(per_square)
        self._pub.publish(msg)

    def _build_fen(self, per_square: Dict[Tuple[int, int], PieceObservation]) -> str:
        ranks = []
        for rank in range(7, -1, -1):
            row = ""
            empty = 0
            for file in range(8):
                obs = per_square.get((file, rank))
                if obs is None:
                    empty += 1
                    continue
                if empty:
                    row += str(empty)
                    empty = 0
                letter = _FEN_LETTER[obs.piece]
                row += letter.upper() if obs.color == "white" else letter
            if empty:
                row += str(empty)
            ranks.append(row)
        # Side-to-move / castling / etc. are unknown to perception alone -- the
        # game manager is the authority, so we publish a placeholder.
        return "/".join(ranks) + " w - - 0 1"

    @staticmethod
    def _average_confidence(per_square: Dict[Tuple[int, int], PieceObservation]
                            ) -> float:
        if not per_square:
            return 0.0
        return float(sum(o.confidence for o in per_square.values())) / len(per_square)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = BoardStateEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
