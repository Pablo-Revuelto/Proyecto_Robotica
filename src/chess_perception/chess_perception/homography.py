"""Image-to-board mapping for the overhead RGB camera.

Two options:

1. `IntrinsicHomography`: when the camera is perfectly nadir (as in the
   simulated `chess_world.sdf`), we can compute the image→world mapping
   analytically from the camera intrinsics + height. This is what we use by
   default in simulation. No checkerboard calibration needed.

2. `MarkerHomography`: at runtime, detect 4 ArUco markers placed at the board
   corners (or any 4 known fiducials), then use `cv2.findHomography`. Plug in
   when moving from simulation to real hardware.

Returned coordinates are in metres in the world frame, with board centre at
(0,0) as defined by chess_brain.board_geometry.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class IntrinsicHomography:
    """Nadir-camera homography from intrinsics.

    The camera looks straight down from height `camera_height` (m), centred
    above the board centre. The principal point is the image centre. With a
    horizontal FoV `hfov_rad` and image width `image_width`, the per-pixel
    metres at the board plane is:
        m_per_px = (2 * camera_height * tan(hfov_rad/2)) / image_width
    """
    camera_height: float
    hfov_rad: float
    image_width: int
    image_height: int

    @property
    def m_per_px(self) -> float:
        import math
        return (2.0 * self.camera_height * math.tan(self.hfov_rad / 2.0)) \
            / float(self.image_width)

    def pixel_to_world(self, px: float, py: float) -> Tuple[float, float]:
        """Image pixel → world (x, y) on the board plane (z = board top)."""
        cx_px = self.image_width / 2.0
        cy_px = self.image_height / 2.0
        m = self.m_per_px
        # In the world frame, the world +X axis points from rank 1 (top of
        # the image when the overhead camera looks down with default
        # orientation) to rank 8 (bottom of the image), so image y → world x.
        x =  (py - cy_px) * m
        y = -(px - cx_px) * m
        return x, y
