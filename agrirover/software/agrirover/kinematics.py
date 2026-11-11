"""Differential-drive kinematics and odometry."""
from __future__ import annotations

import math
from dataclasses import dataclass


def twist_to_wheels(v: float, w: float, r: float, track: float) -> tuple[float, float]:
    """Body twist (m/s, rad/s) -> (left, right) wheel angular velocity [rad/s]."""
    return (v - w * track / 2) / r, (v + w * track / 2) / r


def wheels_to_twist(wl: float, wr: float, r: float, track: float) -> tuple[float, float]:
    return r * (wl + wr) / 2, r * (wr - wl) / track


def limit_twist(v: float, w: float, v_max: float, w_max: float, wheel_w_max: float, r: float, track: float) -> tuple[float, float]:
    """Clamp a twist so that neither body nor wheel limits are exceeded, preserving curvature."""
    v = max(-v_max, min(v_max, v))
    w = max(-w_max, min(w_max, w))
    wl, wr = twist_to_wheels(v, w, r, track)
    m = max(abs(wl), abs(wr))
    if m > wheel_w_max:
        s = wheel_w_max / m
        v, w = v * s, w * s
    return v, w


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def as_tuple(self):
        return (self.x, self.y, self.yaw)


class Odometry:
    """Wheel-encoder odometry with optional gyro heading (the gyro replaces the wheel-derived yaw increment)."""

    def __init__(self, r: float, track: float, cpr: int, pose: Pose2D | None = None):
        self.r, self.track, self.cpr = r, track, cpr
        self.pose = pose or Pose2D()
        self._prev: tuple[int, int] | None = None
        self.distance = 0.0

    def update(self, enc_left: int, enc_right: int, gyro_z: float | None = None, dt: float | None = None) -> Pose2D:
        if self._prev is None:
            self._prev = (enc_left, enc_right)
            return self.pose
        k = 2 * math.pi * self.r / self.cpr
        dl, dr = (enc_left - self._prev[0]) * k, (enc_right - self._prev[1]) * k
        self._prev = (enc_left, enc_right)
        ds = (dl + dr) / 2
        dth = (dr - dl) / self.track
        if gyro_z is not None and dt is not None and math.isfinite(gyro_z):
            dth = gyro_z * dt
        p = self.pose
        p.x += ds * math.cos(p.yaw + dth / 2)
        p.y += ds * math.sin(p.yaw + dth / 2)
        p.yaw = wrap(p.yaw + dth)
        self.distance += abs(ds)
        return p
