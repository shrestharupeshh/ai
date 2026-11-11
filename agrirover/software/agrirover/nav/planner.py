"""Grid costmap + A* global planner (the reference/"lite" stack used headless and in CI).

On the robot the same job is done by Nav2 (see ros2_ws); this implementation exists so that the
complete mission logic can be simulated and regression-tested without ROS, and for route-energy research.
"""
from __future__ import annotations

import heapq
import math

import numpy as np
from scipy.ndimage import distance_transform_edt


class Costmap:
    def __init__(self, occupied: np.ndarray, res: float, robot_radius: float, inflation: float = 0.25, margin: float = 0.02):
        self.res, self.robot_radius = res, robot_radius
        self.occ = occupied.copy()
        self.inflation, self.margin = inflation, margin
        self._rebuild()

    def _rebuild(self):
        self.dist = distance_transform_edt(~self.occ) * self.res          # metres to the nearest obstacle cell
        self.lethal = self.dist <= (self.robot_radius + self.margin)
        d = np.clip(self.dist - self.robot_radius - self.margin, 0, None)
        self.cost = np.where(self.lethal, np.inf, 1.0 + 6.0 * np.exp(-d / max(self.inflation, 1e-3) * 3.0))

    def add_obstacles(self, pts_xy: np.ndarray):
        """Mark world points (N, 2) as occupied and rebuild the inflation layers."""
        if len(pts_xy) == 0:
            return
        ij = np.floor(pts_xy / self.res).astype(int)
        ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < self.occ.shape[1]) & (ij[:, 1] < self.occ.shape[0])
        self.occ[ij[ok, 1], ij[ok, 0]] = True
        self._rebuild()

    def cell(self, x: float, y: float) -> tuple[int, int]:
        return int(y / self.res), int(x / self.res)

    def is_free(self, x: float, y: float) -> bool:
        i, j = self.cell(x, y)
        return 0 <= i < self.occ.shape[0] and 0 <= j < self.occ.shape[1] and not self.lethal[i, j]

    def clearance(self, x: float, y: float) -> float:
        i, j = self.cell(x, y)
        return float(self.dist[i, j]) - self.robot_radius


_MOVES = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]


def astar(cm: Costmap, start: tuple[float, float], goal: tuple[float, float]) -> list[tuple[float, float]] | None:
    """Minimum-cost path on the inflated grid. Returns world waypoints (cell centres) or None."""
    s, g = cm.cell(*start), cm.cell(*goal)
    ny, nx = cm.occ.shape
    if not (0 <= g[0] < ny and 0 <= g[1] < nx) or cm.lethal[g]:
        return None
    if cm.lethal[s]:     # start inside inflation (e.g. after a bump): escape to the nearest free cell first
        free = np.argwhere(~cm.lethal)
        k = np.argmin(((free - np.array(s)) ** 2).sum(1))
        s = tuple(free[k])
    best = {s: 0.0}
    prev = {}
    pq = [(0.0, s)]
    while pq:
        f, cur = heapq.heappop(pq)
        if cur == g:
            break
        gc = best[cur]
        if f > gc + math.hypot(cur[0] - g[0], cur[1] - g[1]) + 1e-6:
            continue
        for di, dj, c in _MOVES:
            n = (cur[0] + di, cur[1] + dj)
            if not (0 <= n[0] < ny and 0 <= n[1] < nx) or cm.lethal[n]:
                continue
            if di and dj and (cm.lethal[cur[0] + di, cur[1]] or cm.lethal[cur[0], cur[1] + dj]):
                continue
            ng = gc + c * cm.cost[n]
            if ng < best.get(n, math.inf):
                best[n] = ng
                prev[n] = cur
                heapq.heappush(pq, (ng + math.hypot(n[0] - g[0], n[1] - g[1]), n))
    if g not in prev and g != s:
        return None
    cells = [g]
    while cells[-1] in prev:
        cells.append(prev[cells[-1]])
    cells.reverse()
    path = [((j + 0.5) * cm.res, (i + 0.5) * cm.res) for i, j in cells]
    path[-1] = goal
    return smooth(cm, path)


def _line_free(cm: Costmap, a, b) -> bool:
    n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (cm.res * 0.5)))
    for k in range(n + 1):
        t = k / n
        if not cm.is_free(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t):
            return False
    return True


def smooth(cm: Costmap, path: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Greedy line-of-sight shortcutting, then re-densify to 5 cm spacing."""
    if len(path) < 3:
        return path
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _line_free(cm, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    dense = [out[0]]
    for a, b in zip(out[:-1], out[1:]):
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.05))
        dense += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]
    return dense


def path_length(path) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path[:-1], path[1:]))
