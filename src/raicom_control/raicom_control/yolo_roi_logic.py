"""Selection and normalization of YOLO detections for Aurora."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Detection:
    label: str
    score: float
    bbox: tuple[float, float, float, float]


def detections_in_normalized_roi(
    detections: list[Detection],
    roi: tuple[float, float, float, float],
    *,
    image_width: int,
    image_height: int,
) -> list[Detection]:
    """Keep boxes whose centres are inside a fixed-scene normalized ROI."""

    if image_width <= 0 or image_height <= 0:
        raise ValueError('image dimensions must be positive')
    x, y, w, h = roi
    if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0 and 0.0 < w <= 1.0 and 0.0 < h <= 1.0):
        raise ValueError('candidate ROI must be normalized and positive')
    x2, y2 = x + w, y + h
    if x2 > 1.0 or y2 > 1.0:
        raise ValueError('candidate ROI exceeds normalized image bounds')
    selected = []
    for detection in detections:
        x1, y1, bx2, by2 = detection.bbox
        cx = (x1 + bx2) / 2.0
        cy = (y1 + by2) / 2.0
        cx /= float(image_width)
        cy /= float(image_height)
        if x <= cx <= x2 and y <= cy <= y2:
            selected.append(detection)
    return selected


def choose_detection(
    detections: list[Detection],
    *,
    desired_label: str = '',
    minimum_score: float = 0.4,
) -> Detection | None:
    """Choose one semantic target, or fail closed on ambiguity.

    A requested label is preferred only when exactly one qualifying detection
    exists.  With no requested label, exactly one qualifying detection is also
    required; multiple boxes are never silently assigned to the gripper.
    """

    qualified = [d for d in detections if d.score >= minimum_score and d.bbox[2] > d.bbox[0] and d.bbox[3] > d.bbox[1]]
    if desired_label:
        qualified = [d for d in qualified if d.label == desired_label]
    if len(qualified) != 1:
        return None
    return qualified[0]


def normalized_roi(detection: Detection, image_width: int, image_height: int) -> dict[str, float | str]:
    if image_width <= 0 or image_height <= 0:
        raise ValueError('image dimensions must be positive')
    x1, y1, x2, y2 = detection.bbox
    return {
        'x': max(0.0, min(1.0, x1 / image_width)),
        'y': max(0.0, min(1.0, y1 / image_height)),
        'w': max(0.0, min(1.0, (x2 - x1) / image_width)),
        'h': max(0.0, min(1.0, (y2 - y1) / image_height)),
        'space': 'normalized',
        'label': detection.label,
        'score': float(detection.score),
    }
