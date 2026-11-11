"""Scan-to-map localisation on a likelihood field (lite stand-in for AMCL / slam_toolbox localisation).

predict(): dead-reckoning from odometry increments.
correct(): local search around the prediction that maximises the mean likelihood of the scan end-points.
The match score doubles as a health signal: persistently low scores raise LOCALIZATION_LOST.
"""
from __future__ import annotations

import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from ..kinematics import wrap


class LikelihoodField:
    def __init__(self, segments: np.ndarray, size_xy: tuple[float, float], res: float = 0.05, sigma: float = 0.08):
        import cv2
        self.res = res
        nx, ny = int(round(size_xy[0] / res)) + 1, int(round(size_xy[1] / res)) + 1
        occ = np.zeros((ny, nx), np.uint8)
        for x0, y0, x1, y1 in segments:
            cv2.line(occ, (int(round(x0 / res)), int(round(y0 / res))), (int(round(x1 / res)), int(round(y1 / res))), 1, 1)
        d = distance_transform_edt(occ == 0) * res
        self.field = np.exp(-0.5 * (d / sigma) ** 2).astype(np.float32)
        self.occ = occ

    def score(self, pts: np.ndarray) -> float:
        ij = np.round(pts / self.res).astype(int)
        ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < self.field.shape[1]) & (ij[:, 1] < self.field.shape[0])
        if not ok.any():
            return 0.0
        return float(self.field[ij[ok, 1], ij[ok, 0]].sum() / len(pts))


class Localizer:
    SUSPECT = 0.55        # below this the pose is not trusted: the rover pauses
    LOST_COUNT = 8        # consecutive suspect matches (1.6 s at 5 Hz) -> LOCALIZATION_LOST

    @property
    def suspect(self) -> bool:
        return self.low_count > 0

    def __init__(self, field: LikelihoodField, pose: tuple[float, float, float], lidar_x: float = 0.15):
        self.f, self.lidar_x = field, lidar_x
        self.x, self.y, self.yaw = pose
        self.score = 1.0
        self.low_count = 0
        self.lost = False

    @property
    def pose(self):
        return (self.x, self.y, self.yaw)

    def predict(self, ds: float, dyaw: float):
        self.x += ds * math.cos(self.yaw + dyaw / 2)
        self.y += ds * math.sin(self.yaw + dyaw / 2)
        self.yaw = wrap(self.yaw + dyaw)

    def _endpoints(self, x, y, yaw, ranges, angles):
        ok = np.isfinite(ranges)
        a = angles[ok] + yaw
        ox, oy = x + self.lidar_x * math.cos(yaw), y + self.lidar_x * math.sin(yaw)
        return np.column_stack((ox + ranges[ok] * np.cos(a), oy + ranges[ok] * np.sin(a)))

    def correct(self, ranges: np.ndarray, angles: np.ndarray, gain: float = 0.6) -> float:
        ranges, angles = ranges[::4], angles[::4]
        if np.isfinite(ranges).sum() < 20:
            self.score = 0.0
        else:
            best = (self.f.score(self._endpoints(self.x, self.y, self.yaw, ranges, angles)), 0.0, 0.0, 0.0)
            for step_xy, step_a in ((0.06, 0.03), (0.02, 0.01)):
                bx, by, ba = best[1:]
                for dx in (-step_xy, 0, step_xy):
                    for dy in (-step_xy, 0, step_xy):
                        for da in (-step_a, 0, step_a):
                            sc = self.f.score(self._endpoints(self.x + bx + dx, self.y + by + dy, self.yaw + ba + da, ranges, angles))
                            if sc > best[0] + 1e-4:
                                best = (sc, bx + dx, by + dy, ba + da)
            self.score = best[0]
            if self.score > 0.35:
                self.x += gain * best[1]
                self.y += gain * best[2]
                self.yaw = wrap(self.yaw + gain * best[3])
        self.low_count = self.low_count + 1 if self.score < self.SUSPECT else 0
        self.lost = self.low_count >= self.LOST_COUNT
        return self.score
