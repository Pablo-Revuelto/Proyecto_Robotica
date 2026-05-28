"""Spawn the 32 chess pieces in Gazebo Harmonic from a FEN string.

Strategy
--------
* Read board layout + per-piece geometry from `config/board_layout.yaml`.
* Convert the FEN to a {square: (color, piece)} dict.
* For each occupied square:
    1. xacro-render `chess_description/urdf/chess_piece.urdf.xacro` to URDF/SDF
       with the correct colour, dimensions and (optional) mesh URI.
    2. Call the Gazebo `/world/<world>/create` service via the `gz service` CLI
       to spawn the model at the square centre.
* The spawned model names follow `<color>_<piece>_<square>` (e.g. `white_pawn_e2`),
  which is the name `chess_motion` uses for attach/detach.

This node is launched once at startup. It exits when all pieces have been
created (or after a configurable timeout).
"""

from __future__ import annotations

import os
import subprocess
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


# ---- FEN parsing ---------------------------------------------------------

FEN_TO_PIECE: Dict[str, str] = {
    "p": "pawn", "r": "rook", "n": "knight",
    "b": "bishop", "q": "queen", "k": "king",
}


@dataclass(frozen=True)
class PiecePlacement:
    color: str           # "white" | "black"
    piece: str           # "pawn", "rook", ...
    square: str          # "e2"
    file: int            # 0..7 (a..h)
    rank: int            # 0..7 (1..8)


def parse_fen_placement(fen: str) -> Iterable[PiecePlacement]:
    board_field = fen.split(" ", 1)[0]
    ranks = board_field.split("/")
    if len(ranks) != 8:
        raise ValueError(f"Malformed FEN ranks: {fen}")
    # Rank 8 is index 0 in FEN; we iterate from rank 8 down to rank 1.
    for rank_idx_from_top, rank_str in enumerate(ranks):
        rank = 7 - rank_idx_from_top   # 0..7  ← rank 1..8
        file = 0
        for ch in rank_str:
            if ch.isdigit():
                file += int(ch)
                continue
            color = "white" if ch.isupper() else "black"
            piece = FEN_TO_PIECE[ch.lower()]
            square = f"{chr(ord('a') + file)}{rank + 1}"
            yield PiecePlacement(color, piece, square, file, rank)
            file += 1


# ---- Board → world ------------------------------------------------------

def square_world_xy(file: int, rank: int, square_size: float,
                    centre: Tuple[float, float]) -> Tuple[float, float]:
    """Map a (file, rank) to world (x, y) at the centre of the square.

    Convention (see config/board_layout.yaml): +X points from white (rank 1)
    to black (rank 8); +Y points along files from h to a (so file 'a' is +Y).
    """
    cx, cy = centre
    x = cx + (rank - 3.5) * square_size
    y = cy + (3.5 - file) * square_size
    return x, y


# ---- xacro / gz spawn ---------------------------------------------------

def _resolve_mesh_uri(uri: str) -> str:
    """Convert package:// URI to file:// absolute path so Gazebo can find the mesh.

    Gazebo Harmonic's URDF importer rewrites package:// to model://, which then
    requires GZ_SIM_RESOURCE_PATH to be set. Using file:// avoids this entirely.
    """
    if not uri.startswith("package://"):
        return uri
    rest = uri[len("package://"):]
    pkg, _, rel = rest.partition("/")
    try:
        share = get_package_share_directory(pkg)
        return f"file://{share}/{rel}"
    except Exception:
        return uri


def render_piece_sdf(piece_xacro: Path, name: str, color: str,
                     geometry: dict) -> str:
    """Render the chess_piece xacro into a URDF string (Gazebo accepts URDF)."""
    cmd = [
        "xacro", str(piece_xacro),
        f"name:={name}",
        f"colour:={color}",
        f"mesh_uri:={geometry.get('mesh_uri', '')}",
        f"height:={geometry['height']}",
        f"radius:={geometry['radius']}",
        f"mass:={geometry['mass']}",
    ]
    out = subprocess.run(cmd, check=True, capture_output=True, text=True)
    return out.stdout


def gz_spawn(world: str, model_name: str, urdf_str: str,
             x: float, y: float, z: float, timeout_s: float = 5.0) -> bool:
    """Call the Gazebo `/world/<world>/create` service via a temp URDF file.

    The temp file is NOT deleted immediately: Gazebo Harmonic queues the
    EntityFactory request and reads sdf_filename asynchronously after returning
    the service response, so deleting before it reads causes Error Code 1.
    Files accumulate in /tmp but are tiny (~5 KB each) and cleared on reboot.
    """
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as fh:
        fh.write(urdf_str)
        urdf_path = fh.name
    req = (
        f'sdf_filename: "{urdf_path}", '
        f'name: "{model_name}", '
        f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}, '
        f'allow_renaming: false'
    )
    cmd = [
        "gz", "service",
        "-s", f"/world/{world}/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", str(int(timeout_s * 1000)),
        "--req", req,
    ]
    r = subprocess.run(cmd, capture_output=True, text=True)
    return r.returncode == 0 and "data: true" in r.stdout


# ---- ROS node -----------------------------------------------------------

class ChessPieceSpawner(Node):

    def __init__(self) -> None:
        super().__init__("chess_piece_spawner")
        self.declare_parameter("world",          "chess_world")
        self.declare_parameter("layout_file",    "")
        self.declare_parameter("initial_delay",  4.0)
        self.declare_parameter("piece_xacro",    "")

        world = self.get_parameter("world").get_parameter_value().string_value
        layout = self.get_parameter("layout_file").get_parameter_value().string_value
        delay = self.get_parameter("initial_delay").get_parameter_value().double_value
        piece_xacro = self.get_parameter("piece_xacro").get_parameter_value().string_value

        if not layout:
            layout = str(Path(get_package_share_directory("chess_gazebo"))
                         / "config" / "board_layout.yaml")
        if not piece_xacro:
            piece_xacro = str(Path(get_package_share_directory("chess_description"))
                              / "urdf" / "chess_piece.urdf.xacro")

        self.get_logger().info(f"Waiting {delay:.1f}s for Gazebo to be ready...")
        time.sleep(delay)
        self._spawn_all(world, Path(layout), Path(piece_xacro))

    def _spawn_all(self, world: str, layout_yaml: Path, piece_xacro: Path) -> None:
        cfg = yaml.safe_load(layout_yaml.read_text())
        square_size = float(cfg["square_size"])
        centre = tuple(cfg["board_centre"])
        board_z = float(cfg["board_z"])
        pieces_cfg = cfg["pieces"]
        fen = cfg["initial_fen"]

        ok = 0
        for placement in parse_fen_placement(fen):
            raw_geom = pieces_cfg[placement.piece]
            # Resolve package:// to file:// so Gazebo finds the mesh directly.
            geometry = {**raw_geom,
                        "mesh_uri": _resolve_mesh_uri(raw_geom.get("mesh_uri", ""))}
            x, y = square_world_xy(placement.file, placement.rank, square_size, centre)
            name = f"{placement.color}_{placement.piece}_{placement.square}"
            urdf = render_piece_sdf(piece_xacro, name, placement.color, geometry)
            if gz_spawn(world, name, urdf, x, y, board_z):
                ok += 1
                self.get_logger().info(f"Spawned {name} at ({x:.3f}, {y:.3f})")
            else:
                self.get_logger().warn(f"Failed to spawn {name}")
        self.get_logger().info(f"Spawned {ok} pieces.")


def main(argv: Optional[list] = None) -> None:
    rclpy.init(args=argv)
    node = ChessPieceSpawner()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
