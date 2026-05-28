"""Pure-Python conversions between algebraic squares and world coordinates.

This module is intentionally free of ROS / numpy dependencies so it can be
used in any context (perception, motion, voice, tests).

Convention (matches chess_gazebo/config/board_layout.yaml):
  - Board centre at (cx, cy) in world XY.
  - +X axis points from rank 1 (white side) to rank 8 (black side).
  - +Y axis points from file 'h' to file 'a'.
  - Each square is `square_size` meters on a side.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

_FILES = "abcdefgh"


@dataclass(frozen=True)
class BoardGeometry:
    """Immutable description of the board's physical placement."""
    square_size: float
    board_z: float
    centre: Tuple[float, float] = (0.0, 0.0)

    # ---- algebraic <-> (file, rank) ------------------------------------

    @staticmethod
    def algebraic_to_file_rank(square: str) -> Tuple[int, int]:
        if len(square) != 2:
            raise ValueError(f"Invalid square: {square!r}")
        f = _FILES.index(square[0].lower())
        r = int(square[1]) - 1
        if not (0 <= r < 8):
            raise ValueError(f"Invalid rank in square: {square!r}")
        return f, r

    @staticmethod
    def file_rank_to_algebraic(file: int, rank: int) -> str:
        if not (0 <= file < 8 and 0 <= rank < 8):
            raise ValueError(f"file/rank out of range: {file}/{rank}")
        return f"{_FILES[file]}{rank + 1}"

    # ---- (file, rank) <-> world ---------------------------------------

    def square_to_world(self, file: int, rank: int) -> Tuple[float, float, float]:
        cx, cy = self.centre
        x = cx + (rank - 3.5) * self.square_size
        y = cy + (3.5 - file) * self.square_size
        return x, y, self.board_z

    def world_to_square(self, x: float, y: float) -> Tuple[int, int]:
        cx, cy = self.centre
        file = round(3.5 - (y - cy) / self.square_size)
        rank = round((x - cx) / self.square_size + 3.5)
        return int(file), int(rank)

    def algebraic_to_world(self, square: str) -> Tuple[float, float, float]:
        f, r = self.algebraic_to_file_rank(square)
        return self.square_to_world(f, r)
