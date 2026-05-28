"""ROS service `/chess/voice/parse`.

Wraps a `MoveParser` (LangChain LLM by default, regex fallback otherwise) and
exposes it as the `chess_msgs/srv/ParseVoiceCommand` service consumed by the
game manager. The parser must return a UCI move that is in the legal move
list for the provided FEN; otherwise the service responds with `success=False`.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from chess_msgs.srv import ParseVoiceCommand
from chess_brain.chess_engine import PythonChessEngine
from chess_brain.board_geometry import BoardGeometry
from chess_brain.msg_conversions import parsed_to_msg

from .parsers import LangChainLLMParser, MoveParser, RegexFallbackParser


class VoiceParserNode(Node):

    def __init__(self) -> None:
        super().__init__("chess_voice_parser")
        self.declare_parameter("use_llm",         True)
        self.declare_parameter("llm_model_id",    "meta-llama/Meta-Llama-3-8B-Instruct")
        self.declare_parameter("llm_temperature", 0.0)
        self.declare_parameter("huggingface_api_token", "")
        self.declare_parameter("square_size",     0.05)
        self.declare_parameter("board_z",         0.02)
        self.declare_parameter("board_centre_x",  0.0)
        self.declare_parameter("board_centre_y",  0.0)

        hf_token = self.get_parameter("huggingface_api_token").get_parameter_value().string_value
        if hf_token:
            import os
            os.environ["HUGGINGFACEHUB_API_TOKEN"] = hf_token

        ss = self.get_parameter("square_size").value
        bz = self.get_parameter("board_z").value
        cx = self.get_parameter("board_centre_x").value
        cy = self.get_parameter("board_centre_y").value
        self._geometry = BoardGeometry(square_size=ss, board_z=bz, centre=(cx, cy))

        self._parser: MoveParser = self._select_parser()
        self._srv = self.create_service(
            ParseVoiceCommand, "/chess/voice/parse", self._handle_parse)

    def _select_parser(self) -> MoveParser:
        if self.get_parameter("use_llm").value:
            model_id = self.get_parameter("llm_model_id").get_parameter_value().string_value
            temperature = float(self.get_parameter("llm_temperature").value)
            self.get_logger().info(f"Using LangChain LLM parser: {model_id} (temperature={temperature})")
            return LangChainLLMParser(model_id=model_id, temperature=temperature)
        self.get_logger().info("Using regex fallback parser.")
        return RegexFallbackParser()

    def _handle_parse(self, req: ParseVoiceCommand.Request,
                      res: ParseVoiceCommand.Response) -> ParseVoiceCommand.Response:
        fen = req.board_state.fen
        if not fen:
            res.success = False
            res.error = "Empty FEN in board_state."
            return res

        engine = PythonChessEngine(fen)
        legal = engine.legal_moves_uci()
        attempt = self._parser.parse(req.utterance, fen, legal)
        if attempt.uci is None:
            res.success = False
            res.error = f"Parser could not produce a legal UCI ({attempt.explanation})."
            return res

        parsed = engine.validate(attempt.uci)
        if parsed is None:
            res.success = False
            res.error = f"Parser returned illegal UCI: {attempt.uci}."
            return res

        res.success = True
        res.error = ""
        res.move = parsed_to_msg(parsed, self._geometry)
        return res


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = VoiceParserNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
