"""Greenhouse world model shared by the simulator, the planner and the mission planner.

Two different obstacle views are produced on purpose:
  * lidar_segments(): what a 2D lidar at 0.215 m actually sees - walls, bench LEGS and clutter.
  * nav_grid():       what the robot must not drive through - walls, full bench footprints
                      (keep-out from the facility drawing; the mast would hit the bench top) and
                      any obstacles that have been mapped.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .. import config


@dataclass
class Station:
    sid: str
    x: float
    y: float
    yaw: float
    aisle: int
    views: list = field(default_factory=list)   # [{"camera": "left"|"right", "bench": int, "plants": [plant ids]}]


@dataclass
class Box:
    name: str
    x0: float
    y0: float
    x1: float
    y1: float


class World:
    def __init__(self, cfg: dict | None = None):
        self.cfg = cfg or config.greenhouse()
        c = self.cfg
        self.w, self.h = c["room"]["x"], c["room"]["y"]
        b = c["bench"]
        self.benches = [Box(f"bench{i+1}", b["x0"], y0, b["x0"] + b["length"], y0 + b["width"]) for i, y0 in enumerate(c["benches_y0"])]
        self.obstacles = [Box(o["name"], o["x"] - o["size"][0] / 2, o["y"] - o["size"][1] / 2, o["x"] + o["size"][0] / 2, o["y"] + o["size"][1] / 2)
                          for o in c.get("obstacles", [])]
        self.dynamic: list[Box] = []           # obstacles added at run time (people, carts)
        self.dock = (c["dock"]["x"], c["dock"]["y"], c["dock"]["yaw"])
        self.aisles_y = self._aisle_centres()
        self.stations, self.plants = self._stations_and_plants()

    # ------------------------------------------------------------------ layout
    def _aisle_centres(self) -> list[float]:
        edges = [0.0]
        for bx in self.benches:
            edges += [bx.y0, bx.y1]
        edges.append(self.h)
        return [(edges[i] + edges[i + 1]) / 2 for i in range(0, len(edges), 2)]

    def aisle_widths(self) -> list[float]:
        edges = [0.0]
        for bx in self.benches:
            edges += [bx.y0, bx.y1]
        edges.append(self.h)
        return [edges[i + 1] - edges[i] for i in range(0, len(edges), 2)]

    def _stations_and_plants(self):
        c = self.cfg
        st, pl = [], {}
        sp, rows, per = c["plants"]["spacing"], c["plants"]["rows_per_side"], c["plants"]["per_station"]
        for a, ya in enumerate(self.aisles_y):
            for k in range(c["stations"]["count"]):
                x = c["stations"]["x_first"] + k * c["stations"]["pitch"]
                s = Station(f"A{a}-S{k+1:02d}", x, ya, 0.0, a)
                # camera "left" looks towards +y (north), "right" towards -y, when the rover faces +x
                for cam, bi in (("left", a), ("right", a - 1)):
                    if 0 <= bi < len(self.benches):
                        bx = self.benches[bi]
                        side = "S" if cam == "left" else "N"     # which side of that bench we look at
                        ids = []
                        for r in range(rows):
                            for j in range(per):
                                pid = f"B{bi+1}{side}-R{r+1}-P{k*per+j+1:03d}"
                                px = x + (j - (per - 1) / 2) * sp
                                py = (bx.y0 + 0.15 + r * sp) if side == "S" else (bx.y1 - 0.15 - r * sp)
                                pl[pid] = dict(id=pid, x=round(px, 3), y=round(py, 3), bench=bi + 1, station=s.sid, camera=cam)
                                ids.append(pid)
                        s.views.append(dict(camera=cam, bench=bi + 1, plants=ids))
                st.append(s)
        return st, pl

    # ------------------------------------------------------------------ lidar view
    def legs(self) -> list[Box]:
        b = self.cfg["bench"]
        s, out = b["leg_size"], []
        for bx in self.benches:
            n = int(round(b["length"] / b["leg_pitch"])) + 1
            for i in range(n):
                x = bx.x0 + i * b["leg_pitch"]
                x = min(max(x, bx.x0 + s / 2), bx.x1 - s / 2)
                for y in (bx.y0 + s / 2, bx.y1 - s / 2):
                    out.append(Box("leg", x - s / 2, y - s / 2, x + s / 2, y + s / 2))
        return out

    def lidar_segments(self) -> np.ndarray:
        """(N, 4) array of wall segments x0, y0, x1, y1 visible in the scan plane."""
        segs = [(0, 0, self.w, 0), (self.w, 0, self.w, self.h), (self.w, self.h, 0, self.h), (0, self.h, 0, 0)]
        for bx in self.legs() + self.obstacles + self.dynamic:
            segs += [(bx.x0, bx.y0, bx.x1, bx.y0), (bx.x1, bx.y0, bx.x1, bx.y1), (bx.x1, bx.y1, bx.x0, bx.y1), (bx.x0, bx.y1, bx.x0, bx.y0)]
        return np.array(segs, float)

    # ------------------------------------------------------------------ navigation view
    def nav_grid(self, res: float = 0.05, include_obstacles: bool = True, extra: list[Box] | None = None) -> "np.ndarray":
        nx, ny = int(round(self.w / res)), int(round(self.h / res))
        g = np.zeros((ny, nx), bool)
        g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = True
        boxes = list(self.benches) + (self.obstacles if include_obstacles else []) + (extra or [])
        for bx in boxes:
            g[max(0, int(math.floor(bx.y0 / res + 1e-6))):int(math.ceil(bx.y1 / res - 1e-6)),
              max(0, int(math.floor(bx.x0 / res + 1e-6))):int(math.ceil(bx.x1 / res - 1e-6))] = True
        return g

    def collides(self, x: float, y: float, radius: float) -> str | None:
        """Ground-truth collision test of a disc against everything physical (benches count: mast/top deck)."""
        if x < radius or y < radius or x > self.w - radius or y > self.h - radius:
            return "wall"
        for bx in self.benches + self.obstacles + self.dynamic:
            dx = max(bx.x0 - x, 0, x - bx.x1)
            dy = max(bx.y0 - y, 0, y - bx.y1)
            if dx * dx + dy * dy < radius * radius:
                return bx.name
        return None
