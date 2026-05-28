"""Conversions between python-chess / `ParsedMove` and the ROS `ChessMove` msg."""

from __future__ import annotations

from typing import Optional

from chess_msgs.msg import ChessMove, Square
from geometry_msgs.msg import Point

from .board_geometry import BoardGeometry
from .chess_engine import ParsedMove


_PIECE_CODE = {
    "pawn":   ChessMove.PIECE_PAWN,
    "knight": ChessMove.PIECE_KNIGHT,
    "bishop": ChessMove.PIECE_BISHOP,
    "rook":   ChessMove.PIECE_ROOK,
    "queen":  ChessMove.PIECE_QUEEN,
    "king":   ChessMove.PIECE_KING,
}
_COLOR_CODE = {"white": ChessMove.COLOR_WHITE, "black": ChessMove.COLOR_BLACK}


def _square_msg(square: str, geometry: BoardGeometry) -> Square:
    f, r = BoardGeometry.algebraic_to_file_rank(square)
    x, y, z = geometry.square_to_world(f, r)
    msg = Square()
    msg.algebraic = square
    msg.file = f
    msg.rank = r
    msg.world_position = Point(x=x, y=y, z=z)
    return msg


def parsed_to_msg(parsed: ParsedMove, geometry: BoardGeometry) -> ChessMove:
    msg = ChessMove()
    msg.color = _COLOR_CODE[parsed.color]
    msg.piece = _PIECE_CODE[parsed.piece]
    msg.from_square = _square_msg(parsed.from_square, geometry)
    msg.to_square = _square_msg(parsed.to_square, geometry)
    msg.is_capture = parsed.is_capture
    msg.captured_piece = (_PIECE_CODE[parsed.captured_piece]
                          if parsed.captured_piece else ChessMove.PIECE_NONE)
    msg.is_promotion = parsed.is_promotion
    msg.promotion_piece = (_PIECE_CODE[parsed.promotion_piece]
                           if parsed.promotion_piece else ChessMove.PIECE_NONE)
    msg.is_castling = parsed.is_castling
    msg.is_en_passant = parsed.is_en_passant
    msg.uci = parsed.uci
    msg.san = parsed.san
    return msg
