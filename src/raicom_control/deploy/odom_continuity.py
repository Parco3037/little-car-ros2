"""One-shot SE(2) frame continuity; no resetting/rebasing after activation."""
import math
import numpy as np


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


class ContinuityAnchor:
    def __init__(self, previous, current):
        p, c = np.asarray(previous, dtype=float), np.asarray(current, dtype=float)
        if p.shape != (3,) or c.shape != (3,) or not np.isfinite([p, c]).all():
            raise ValueError('Invalid anchor poses')
        self.yaw = float(p[2]-c[2])
        self.xy = p[:2]-rotation(self.yaw)@c[:2]

    def transform(self, pose):
        p = np.asarray(pose, dtype=float)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise ValueError('Invalid input pose')
        return np.array([*(self.xy+rotation(self.yaw)@p[:2]), p[2]+self.yaw])

    def covariance(self, flat):
        c = np.asarray(flat, dtype=float).reshape(6, 6)
        if not np.isfinite(c).all() or not np.allclose(c, c.T, atol=1e-6):
            raise ValueError('Nonfinite or asymmetric covariance')
        j = np.eye(6)
        j[:2, :2] = rotation(self.yaw)
        j[3:5, 3:5] = rotation(self.yaw)
        return (j@c@j.T).reshape(36).tolist()
