"""Minimal occupancy-grid mapper (log-odds) from lidar scans at known poses.

Used to produce a facility map in simulation and as a cross-check of the survey drawing. On the robot,
slam_toolbox performs SLAM; this module only assumes poses are already estimated.
"""
from __future__ import annotations

import math

import cv2
import numpy as np


class GridMapper:
    def __init__(self, size_xy: tuple[float, float], res: float = 0.05):
        self.res = res
        self.shape = (int(round(size_xy[1] / res)) + 1, int(round(size_xy[0] / res)) + 1)
        self.logodds = np.zeros(self.shape, np.float32)

    def integrate(self, pose, ranges: np.ndarray, angles: np.ndarray, lidar_x: float = 0.15, max_range: float = 8.0):
        x, y, yaw = pose
        ox, oy = x + lidar_x * math.cos(yaw), y + lidar_x * math.sin(yaw)
        o = (int(round(ox / self.res)), int(round(oy / self.res)))
        free = np.zeros(self.shape, np.uint8)
        hits = []
        for r, a in zip(ranges[::2], angles[::2]):
            rr = min(r, max_range) if np.isfinite(r) else max_range
            e = (int(round((ox + rr * math.cos(a + yaw)) / self.res)), int(round((oy + rr * math.sin(a + yaw)) / self.res)))
            cv2.line(free, o, e, 1, 1)
            if np.isfinite(r) and r < max_range and 0 <= e[1] < self.shape[0] and 0 <= e[0] < self.shape[1]:
                hits.append(e)
        self.logodds -= 0.2 * free
        for e in hits:
            self.logodds[e[1], e[0]] += 1.1
        np.clip(self.logodds, -4, 6, out=self.logodds)

    def occupancy(self) -> np.ndarray:
        """uint8 image: 0 = occupied, 254 = free, 205 = unknown (ROS map_server convention)."""
        img = np.full(self.shape, 205, np.uint8)
        img[self.logodds < -0.5] = 254
        img[self.logodds > 1.0] = 0
        return img

    def save(self, path_png: str, origin=(0.0, 0.0)):
        cv2.imwrite(path_png, np.flipud(self.occupancy()))
        with open(path_png.rsplit(".", 1)[0] + ".yaml", "w") as f:
            f.write(f"image: {path_png.split('/')[-1]}\nresolution: {self.res}\norigin: [{origin[0]}, {origin[1]}, 0.0]\n"
                    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
