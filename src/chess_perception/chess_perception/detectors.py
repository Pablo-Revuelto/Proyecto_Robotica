"""Piece-detection backends.

A `PieceDetector` consumes an OpenCV BGR frame and returns a list of detected
chess pieces in image pixel coordinates. Mapping from pixels → board squares
is the responsibility of `homography.py`, so the detector stays pluggable.

Two concrete implementations:

* `YoloPieceDetector` — Ultralytics YOLOv8 with 12 classes
  (white/black × pawn, rook, knight, bishop, queen, king). The model weights
  are NOT shipped; provide the path via the `weights` parameter. To train one
  see chess_perception/models/README_TRAINING.md.

* `NullDetector` — returns nothing. Used as the default until weights are
  available; combined with the ground-truth backend the system still works.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Protocol


@dataclass(frozen=True)
class PieceObservation:
    color: str             # "white" | "black"
    piece: str             # "pawn", "rook", ...
    cx: float              # image-space centre, pixels
    cy: float
    confidence: float


class PieceDetector(Protocol):
    def detect(self, bgr_image) -> List[PieceObservation]: ...


# Standard 12-class label set used both by training scripts and inference.
YOLO_LABELS: List[str] = [
    "white_pawn", "white_knight", "white_bishop",
    "white_rook", "white_queen",  "white_king",
    "black_pawn", "black_knight", "black_bishop",
    "black_rook", "black_queen",  "black_king",
]


class NullDetector:
    """Returns no observations -- safe default when no model is loaded."""
    def detect(self, _bgr_image) -> List[PieceObservation]:
        return []


class YoloPieceDetector:
    """Ultralytics YOLOv8 backend."""

    def __init__(self, weights: str, conf_threshold: float = 0.4) -> None:
        self._weights_path = weights
        self._conf = conf_threshold
        self._model = None

    def _ensure_loaded(self) -> None:
        if self._model is not None:
            return
        from ultralytics import YOLO
        self._model = YOLO(self._weights_path)

    def detect(self, bgr_image) -> List[PieceObservation]:
        self._ensure_loaded()
        results = self._model.predict(bgr_image, conf=self._conf, verbose=False)
        observations: List[PieceObservation] = []
        for result in results:
            names = result.names                       # {idx: "white_pawn", ...}
            boxes = result.boxes
            if boxes is None:
                continue
            xyxy = boxes.xyxy.cpu().numpy()
            cls  = boxes.cls.cpu().numpy().astype(int)
            confs = boxes.conf.cpu().numpy()
            for (x1, y1, x2, y2), c, p in zip(xyxy, cls, confs):
                label = names.get(int(c), "")
                parsed = self._parse_label(label)
                if parsed is None:
                    continue
                color, piece = parsed
                cx = float((x1 + x2) / 2.0)
                cy = float((y1 + y2) / 2.0)
                observations.append(PieceObservation(color, piece, cx, cy, float(p)))
        return observations

    # Short-code label scheme (e.g. weights_v4.pt): "wP", "bK", ...
    _SHORT_COLOR = {"w": "white", "b": "black"}
    _SHORT_PIECE = {
        "p": "pawn", "n": "knight", "b": "bishop",
        "r": "rook", "q": "queen", "k": "king",
    }
    _PIECES = ("pawn", "knight", "bishop", "rook", "queen", "king")

    @classmethod
    def _parse_label(cls, label: str) -> Optional[tuple]:
        """Map a YOLO class name to (color, piece).

        Accepts the three label schemes seen across our trained models:
          * "white_pawn" / "black_king"   (underscore, full words)
          * "white-pawn" / "black-king"   (hyphen, full words)
          * "wP" / "bK"                    (short codes, K=king/N=knight)
        Returns None for anything that doesn't map to a valid piece.
        """
        # Full-word schemes, separated by "_" or "-".
        for sep in ("_", "-"):
            if sep in label:
                color, piece = label.split(sep, 1)
                color, piece = color.lower(), piece.lower()
                if color in ("white", "black") and piece in cls._PIECES:
                    return color, piece
                return None
        # Short-code scheme: 2 chars, e.g. "wP".
        if len(label) == 2:
            color = cls._SHORT_COLOR.get(label[0].lower())
            piece = cls._SHORT_PIECE.get(label[1].lower())
            if color and piece:
                return color, piece
        return None
