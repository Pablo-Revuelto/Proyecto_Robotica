"""Authoritative mapping {square: gazebo_model_name} for the chess scene.

`chess_gazebo.spawn_chess_pieces` names every spawned model as
`<color>_<piece>_<square>` (e.g. `white_pawn_e2`). After each move we update
the registry so the new origin square holds the moved model (the square name
stays in the model name even after movement; that's fine -- the registry is
the truth, the name is just a label).

This is the same simple bookkeeping that any chess game manager would do, and
keeps `chess_motion` independent from the live Gazebo entity list.
"""

from __future__ import annotations

from typing import Dict, Iterable, Optional


class PieceRegistry:

    def __init__(self) -> None:
        self._square_to_model: Dict[str, str] = {}

    def populate_from_initial_fen(self, fen: str) -> None:
        from chess_gazebo.spawn_chess_pieces import parse_fen_placement
        for placement in parse_fen_placement(fen):
            name = f"{placement.color}_{placement.piece}_{placement.square}"
            self._square_to_model[placement.square] = name

    def model_at(self, square: str) -> Optional[str]:
        return self._square_to_model.get(square)

    def apply_move(self, from_sq: str, to_sq: str,
                   captured_model: Optional[str] = None) -> None:
        model = self._square_to_model.pop(from_sq, None)
        if model is None:
            raise KeyError(f"No piece registered at {from_sq}")
        # If the destination already held a piece (capture), it must have
        # been removed by chess_motion *before* this call.
        if to_sq in self._square_to_model and captured_model is None:
            raise RuntimeError(f"{to_sq} occupied before move; captured_model "
                               "must be provided")
        self._square_to_model[to_sq] = model

    def remove(self, square: str) -> Optional[str]:
        return self._square_to_model.pop(square, None)

    def items(self) -> Iterable[tuple]:
        return self._square_to_model.items()
