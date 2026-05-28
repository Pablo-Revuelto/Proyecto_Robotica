"""Chess rule engine wrapper.

The rest of the project does not depend on python-chess directly. It depends
on the `ChessEngine` Protocol defined here, so we can swap python-chess for
a different implementation without touching any callers.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Protocol

import chess


@dataclass(frozen=True)
class ParsedMove:
    """Validated chess move with rule-level metadata."""
    uci: str
    san: str
    color: str            # "white" | "black"
    piece: str            # "pawn", "knight", "bishop", "rook", "queen", "king"
    from_square: str
    to_square: str
    is_capture: bool
    captured_piece: Optional[str]
    is_promotion: bool
    promotion_piece: Optional[str]
    is_castling: bool
    is_en_passant: bool


class ChessEngine(Protocol):
    def fen(self) -> str: ...
    def turn(self) -> str: ...
    def legal_moves_uci(self) -> List[str]: ...
    def validate(self, uci_or_san: str) -> Optional[ParsedMove]: ...
    def apply(self, move: ParsedMove) -> None: ...
    def reset(self, fen: Optional[str] = None) -> None: ...


_PIECE_NAMES = {
    chess.PAWN: "pawn", chess.KNIGHT: "knight", chess.BISHOP: "bishop",
    chess.ROOK: "rook", chess.QUEEN: "queen", chess.KING: "king",
}


class PythonChessEngine:
    """Concrete `ChessEngine` backed by python-chess."""

    def __init__(self, fen: Optional[str] = None) -> None:
        self._board = chess.Board(fen) if fen else chess.Board()

    def fen(self) -> str:
        return self._board.fen()

    def turn(self) -> str:
        return "white" if self._board.turn == chess.WHITE else "black"

    def legal_moves_uci(self) -> List[str]:
        return [m.uci() for m in self._board.legal_moves]

    def validate(self, uci_or_san: str) -> Optional[ParsedMove]:
        move = self._coerce(uci_or_san)
        if move is None or move not in self._board.legal_moves:
            return None
        return self._describe(move)

    def apply(self, move: ParsedMove) -> None:
        m = chess.Move.from_uci(move.uci)
        if m not in self._board.legal_moves:
            raise ValueError(f"Illegal move: {move.uci}")
        self._board.push(m)

    def reset(self, fen: Optional[str] = None) -> None:
        self._board = chess.Board(fen) if fen else chess.Board()

    # ---- helpers ------------------------------------------------------

    def _coerce(self, s: str) -> Optional[chess.Move]:
        try:
            return chess.Move.from_uci(s)
        except ValueError:
            pass
        try:
            return self._board.parse_san(s)
        except ValueError:
            return None

    def _describe(self, move: chess.Move) -> ParsedMove:
        piece_obj = self._board.piece_at(move.from_square)
        captured = self._board.piece_at(move.to_square)
        is_ep = self._board.is_en_passant(move)
        if captured is None and is_ep:
            ep_sq = chess.square(chess.square_file(move.to_square),
                                 chess.square_rank(move.from_square))
            captured = self._board.piece_at(ep_sq)
        return ParsedMove(
            uci=move.uci(),
            san=self._board.san(move),
            color="white" if piece_obj.color == chess.WHITE else "black",
            piece=_PIECE_NAMES[piece_obj.piece_type],
            from_square=chess.square_name(move.from_square),
            to_square=chess.square_name(move.to_square),
            is_capture=captured is not None,
            captured_piece=_PIECE_NAMES[captured.piece_type] if captured else None,
            is_promotion=move.promotion is not None,
            promotion_piece=_PIECE_NAMES[move.promotion] if move.promotion else None,
            is_castling=self._board.is_castling(move),
            is_en_passant=is_ep,
        )
