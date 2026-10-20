"""Pure, fail-closed depth observation and temporal stability logic.

This module deliberately contains no ROS or robot-control imports.  It turns a
depth ROI supplied by the semantic camera into a robust camera-frame point and
requires three consecutive, mutually consistent observations before declaring
the target usable.  A missing ROI, invalid depth, or a jump in the target
returns ``ready=False``; callers must never infer a grasp target from absence
of data.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import isfinite
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class Roi:
    """Pixel ROI in half-open ``(x1, y1, x2, y2)`` form."""

    x1: int
    y1: int
    x2: int
    y2: int

    def clipped(self, width: int, height: int) -> "Roi":
        return Roi(
            max(0, min(self.x1, width)),
            max(0, min(self.y1, height)),
            max(0, min(self.x2, width)),
            max(0, min(self.y2, height)),
        )

    @property
    def valid(self) -> bool:
        return self.x2 > self.x1 and self.y2 > self.y1

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2 - 1) / 2.0, (self.y1 + self.y2 - 1) / 2.0)


@dataclass(frozen=True)
class DepthObservation:
    ready: bool
    reason: str
    roi: Roi | None = None
    pixel_x: float | None = None
    pixel_y: float | None = None
    x_m: float | None = None
    y_m: float | None = None
    z_m: float | None = None
    valid_ratio: float = 0.0


def _finite_values(values: np.ndarray, minimum_mm: float, maximum_mm: float) -> np.ndarray:
    values = np.asarray(values)
    valid = values[(values >= minimum_mm) & (values <= maximum_mm)]
    return valid[np.isfinite(valid)]


def robust_depth_mm(
    depth_mm: np.ndarray,
    roi: Roi,
    *,
    minimum_mm: float = 150.0,
    maximum_mm: float = 4000.0,
    minimum_valid_ratio: float = 0.35,
) -> tuple[float | None, float, str]:
    """Return a MAD-trimmed median depth for an ROI.

    The ratio is measured over all pixels in the clipped ROI.  A flat invalid
    or out-of-range patch is rejected instead of being converted to a zero
    distance.
    """

    if depth_mm.ndim != 2:
        return None, 0.0, "depth is not mono16"
    h, w = depth_mm.shape
    roi = roi.clipped(w, h)
    if not roi.valid:
        return None, 0.0, "ROI is outside the depth image"
    patch = np.asarray(depth_mm[roi.y1 : roi.y2, roi.x1 : roi.x2], dtype=np.float64)
    total = patch.size
    valid = _finite_values(patch, minimum_mm, maximum_mm)
    ratio = float(valid.size) / float(total) if total else 0.0
    if ratio < minimum_valid_ratio:
        return None, ratio, f"insufficient valid depth ({ratio:.2f})"
    median = float(np.median(valid))
    mad = float(np.median(np.abs(valid - median)))
    # A zero MAD is common on a planar face.  Keep all values in that case.
    if mad > 0.0:
        trimmed = valid[np.abs(valid - median) <= max(3.0 * mad, 12.0)]
        if trimmed.size >= max(3, int(valid.size * 0.25)):
            valid = trimmed
            median = float(np.median(valid))
    return median, ratio, "ok"


def camera_point_from_pixel(
    pixel_x: float,
    pixel_y: float,
    depth_mm: float,
    *,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> tuple[float, float, float]:
    """Project a depth pixel into the Aurora camera frame in metres."""

    z = depth_mm / 1000.0
    return ((pixel_x - cx) * z / fx, (pixel_y - cy) * z / fy, z)


class StabilityGate:
    """Require three consecutive observations with bounded motion."""

    def __init__(
        self,
        *,
        required_frames: int = 3,
        max_pixel_delta: float = 3.0,
        max_xyz_delta_m: float = 0.015,
    ) -> None:
        self.required_frames = max(1, int(required_frames))
        self.max_pixel_delta = float(max_pixel_delta)
        self.max_xyz_delta_m = float(max_xyz_delta_m)
        self._history: deque[DepthObservation] = deque(maxlen=self.required_frames)

    def reset(self) -> None:
        self._history.clear()

    @property
    def count(self) -> int:
        return len(self._history)

    def push(self, observation: DepthObservation) -> bool:
        if not observation.ready:
            self.reset()
            return False

        def restart_from_current() -> None:
            # A jump invalidates the previous temporal chain, but the newest
            # valid observation is still the correct first frame of the next
            # chain.  Keeping it makes the reset observable as count == 1 and
            # prevents a moving target from being compared with stale data.
            self._history.clear()
            self._history.append(observation)

        previous = self._history[-1] if self._history else None
        if previous is not None:
            if previous.pixel_x is None or observation.pixel_x is None:
                restart_from_current()
                return False
            if previous.pixel_y is None or observation.pixel_y is None:
                restart_from_current()
                return False
            if abs(observation.pixel_x - previous.pixel_x) > self.max_pixel_delta:
                restart_from_current()
                return False
            if abs(observation.pixel_y - previous.pixel_y) > self.max_pixel_delta:
                restart_from_current()
                return False
            for before, after in zip(
                (previous.x_m, previous.y_m, previous.z_m),
                (observation.x_m, observation.y_m, observation.z_m),
            ):
                if before is None or after is None or abs(after - before) > self.max_xyz_delta_m:
                    restart_from_current()
                    return False
        self._history.append(observation)
        return len(self._history) >= self.required_frames


def observe_roi(
    depth_mm: np.ndarray,
    roi: Roi,
    *,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    minimum_mm: float = 150.0,
    maximum_mm: float = 4000.0,
) -> DepthObservation:
    """Create one camera-frame observation from a semantic ROI."""

    clipped = roi.clipped(depth_mm.shape[1], depth_mm.shape[0])
    depth, ratio, reason = robust_depth_mm(
        depth_mm,
        clipped,
        minimum_mm=minimum_mm,
        maximum_mm=maximum_mm,
    )
    if depth is None:
        return DepthObservation(False, reason, roi=clipped, valid_ratio=ratio)
    px, py = clipped.center
    x, y, z = camera_point_from_pixel(px, py, depth, fx=fx, fy=fy, cx=cx, cy=cy)
    if not all(isfinite(value) for value in (x, y, z)):
        return DepthObservation(False, "non-finite camera point", roi=clipped, valid_ratio=ratio)
    return DepthObservation(
        True,
        "ok",
        roi=clipped,
        pixel_x=px,
        pixel_y=py,
        x_m=x,
        y_m=y,
        z_m=z,
        valid_ratio=ratio,
    )
