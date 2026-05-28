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

    @staticmethod
    def _parse_label(label: str) -> Optional[tuple]:
        if "_" not in label:
            return None
        color, piece = label.split("_", 1)
        if color not in ("white", "black"):
            return None
        if piece not in ("pawn", "knight", "bishop", "rook", "queen", "king"):
            return None
        return color, piece
