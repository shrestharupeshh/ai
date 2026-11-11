"""Local controller: rotate-to-heading + regulated pure pursuit, with a lidar protective field.

Behaviour mirrors the Nav2 configuration shipped in ros2_ws (RotationShim + RegulatedPurePursuit +
collision monitor) so that simulation results are qualitatively transferable, not numerically identical.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..kinematics import wrap


@dataclass
class ControllerParams:
    v_max: float = 0.30            # aisle transit speed (R-MOB-02)
    w_max: float = 0.9
    lookahead: float = 0.35
    rotate_thresh: float = 0.5     # rad: rotate in place first if the path heading error exceeds this
    xy_tol: float = 0.04
    yaw_tol: float = 0.04
    a_lin: float = 0.5
    a_dec: float = 1.0
    a_ang: float = 2.0
    stop_dist: float = 0.10        # protective field depth in front of the nose at standstill (+ speed-dependent part)
    slow_dist: float = 0.60
    half_width: float = 0.20
    nose: float = 0.245            # axle to bumper face
    swept_radius: float = 0.280    # turn-in-place envelope (bumper corners)
    small_turn: float = 0.35       # rad: corrections below this are allowed even where a full turn is not
    rotate_margin: float = 0.03


class PathFollower:
    def __init__(self, params: ControllerParams | None = None):
        self.p = params or ControllerParams()
        self.path: list[tuple[float, float]] = []
        self.goal_yaw: float | None = None
        self.idx = 0
        self.v = self.w = 0.0
        self.done = False
        self.blocked = False
        self.blocked_reason = ""
        self.phase = "idle"
        self._backup_left = 0.0
        self._turn_err = 0.0

    def set_path(self, path, goal_yaw: float | None = None):
        self.path, self.goal_yaw, self.idx, self.done, self.blocked = list(path), goal_yaw, 0, False, False
        self.phase = "track"

    def cancel(self):
        self.path, self.done, self.phase = [], True, "idle"
        self._backup_left = 0.0

    def start_backup(self, distance: float = 0.20):
        """Recovery: reverse straight for `distance` (used when an obstacle sits inside the turning envelope)."""
        self._backup_left = distance

    def rotation_blocked(self, scan_xy_robot: np.ndarray | None) -> bool:
        """True if any return lies inside the turn-in-place envelope (would be swept by a corner while rotating)."""
        if scan_xy_robot is None or len(scan_xy_robot) == 0:
            return False
        r = np.hypot(scan_xy_robot[:, 0], scan_xy_robot[:, 1])
        return bool((r < self.p.swept_radius + self.p.rotate_margin).any())

    def rear_clear(self, scan_xy_robot: np.ndarray | None, dist: float = 0.30) -> bool:
        if scan_xy_robot is None:
            return False
        x, y = scan_xy_robot[:, 0], scan_xy_robot[:, 1]
        behind = (np.abs(y) < self.p.half_width) & (x < 0)
        return not behind.any() or float(np.max(x[behind])) < -(self.p.nose + dist)

    def protective_field(self, scan_xy_robot: np.ndarray, v: float) -> tuple[bool, float]:
        """scan points in the robot frame (N, 2). Returns (stop, speed_scale)."""
        if scan_xy_robot is None or len(scan_xy_robot) == 0:
            return False, 1.0
        p = self.p
        x, y = scan_xy_robot[:, 0], scan_xy_robot[:, 1]
        ahead = (np.abs(y) < p.half_width) & (x > 0.05)
        if not ahead.any():
            return False, 1.0
        d = float(np.min(x[ahead])) - p.nose
        stop_d = p.stop_dist + v * v / (2 * p.a_dec) + 0.12 * v      # braking distance + latency at the current speed
        if d < stop_d:
            return True, 0.0
        if d < p.slow_dist:
            return False, max(0.25, (d - stop_d) / (p.slow_dist - stop_d))
        return False, 1.0

    def step(self, pose: tuple[float, float, float], dt: float, scan_xy_robot: np.ndarray | None = None,
             rotate_ok: bool = True) -> tuple[float, float]:
        p = self.p
        x, y, yaw = pose
        v_cmd = w_cmd = 0.0
        self.blocked = False
        self.blocked_reason = ""
        if self._backup_left > 0:
            if self.rear_clear(scan_xy_robot):
                self.v += max(-p.a_lin * dt, min(p.a_lin * dt, -0.10 - self.v))
                self.w = 0.0
                self._backup_left -= abs(self.v) * dt
                return self.v, self.w
            self._backup_left = 0.0
        if self.path and not self.done:
            gx, gy = self.path[-1]
            dist_goal = math.hypot(gx - x, gy - y)
            # advance to the closest path point inside a forward window (monotonic, tolerant of corner cutting)
            hi = min(len(self.path), self.idx + 40)
            d2 = [(self.path[k][0] - x) ** 2 + (self.path[k][1] - y) ** 2 for k in range(self.idx, hi)]
            self.idx += min(range(len(d2)), key=d2.__getitem__)
            if dist_goal < p.xy_tol or self.phase == "final_yaw":
                self.phase = "final_yaw"
                if self.goal_yaw is None:
                    self.done = True
                else:
                    e = wrap(self.goal_yaw - yaw)
                    self._turn_err = e
                    if abs(e) < p.yaw_tol and abs(self.w) < 0.1:
                        self.done = True
                    else:
                        w_cmd = max(-p.w_max, min(p.w_max, 2.5 * e))
                        w_cmd = math.copysign(max(abs(w_cmd), 0.12), e)
            else:
                # lookahead point
                j = self.idx
                while j < len(self.path) - 1 and math.hypot(self.path[j][0] - x, self.path[j][1] - y) < p.lookahead:
                    j += 1
                lx, ly = self.path[j]
                err = wrap(math.atan2(ly - y, lx - x) - yaw)
                self._turn_err = err
                if abs(err) > p.rotate_thresh or (self.phase == "rotate" and abs(err) > 0.08):
                    self.phase = "rotate"
                    w_cmd = math.copysign(min(p.w_max, max(0.25, 2.0 * abs(err))), err)
                else:
                    self.phase = "track"
                    ld = max(0.08, math.hypot(lx - x, ly - y))
                    curv = 2 * math.sin(err) / ld
                    v_cmd = p.v_max * max(0.3, 1 - abs(err) / p.rotate_thresh)        # slow down in turns
                    v_cmd = min(v_cmd, math.sqrt(2 * 0.4 * max(dist_goal - p.xy_tol * 0.5, 0.0)) + 0.03)   # approach ramp
                    w_cmd = max(-p.w_max, min(p.w_max, v_cmd * curv))
        if v_cmd > 0 and scan_xy_robot is not None:
            stop, scale = self.protective_field(scan_xy_robot, self.v)
            if stop:
                e = self._turn_err
                can_turn = not self.rotation_blocked(scan_xy_robot) and (rotate_ok or abs(e) <= p.small_turn)
                if self.phase == "track" and abs(e) > 0.08 and can_turn:
                    # something ahead but the path bends away from it (e.g. approaching a way-point next to a wall
                    # at an angle): square up to the path first instead of waiting
                    self.phase = "rotate"
                    v_cmd, w_cmd = 0.0, math.copysign(min(p.w_max, max(0.25, 2.0 * abs(e))), e)
                else:
                    self.blocked, self.blocked_reason = True, "ahead"
                    v_cmd = w_cmd = 0.0
            else:
                v_cmd *= scale
        if abs(w_cmd) > 0.05 and v_cmd < 0.05:
            big = abs(self._turn_err) > p.small_turn
            if self.rotation_blocked(scan_xy_robot) or (big and not rotate_ok):
                self.blocked, self.blocked_reason = True, "rotation"   # a corner would sweep through an obstacle / bench
                v_cmd = w_cmd = 0.0
        # acceleration limits
        dv_up, dv_dn = p.a_lin * dt, p.a_dec * dt
        self.v += max(-dv_dn, min(dv_up, v_cmd - self.v)) if v_cmd >= self.v else max(-dv_dn, v_cmd - self.v)
        self.w += max(-p.a_ang * dt, min(p.a_ang * dt, w_cmd - self.w))
        if self.done:
            self.v = self.w = 0.0
        return self.v, self.w
