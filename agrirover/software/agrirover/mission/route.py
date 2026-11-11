"""Inspection route planning: station ordering and an energy/time model of a route."""
from __future__ import annotations

import itertools
import math

from ..sim.world import Station, World


def boustrophedon(world: World, aisles: list[int] | None = None) -> list[Station]:
    """Serpentine order: up aisle 0, back down aisle 1, ... (the default mission)."""
    out = []
    for n, a in enumerate(aisles if aisles is not None else range(len(world.aisles_y))):
        st = sorted([s for s in world.stations if s.aisle == a], key=lambda s: s.x)
        out += st if n % 2 == 0 else st[::-1]
    return out


def naive_order(world: World) -> list[Station]:
    """Every aisle entered from the head corridor (always west to east): simple, but longer."""
    return sorted(world.stations, key=lambda s: (s.aisle, s.x))


def leg_cost(a, b, world: World, v: float = 0.30, w: float = 0.8) -> tuple[float, float, float]:
    """(distance m, rotation rad, time s) for travelling between two (x, y) points along the aisle graph."""
    (ax, ay), (bx, by) = a, b
    if abs(ay - by) < 1e-6:
        d, rot = abs(ax - bx), 0.0
    else:
        x_head, x_tail = world.benches[0].x0 - 0.6, world.benches[0].x1 + 0.6
        via_head = abs(ax - x_head) + abs(ay - by) + abs(bx - x_head)
        via_tail = abs(ax - x_tail) + abs(ay - by) + abs(bx - x_tail)
        d, rot = min(via_head, via_tail), math.pi          # two 90 degree turns
    return d, rot, d / v + rot / w


def route_metrics(order: list[Station], world: World, dwell_s: float = 6.0, p_drive: float = 19.0, p_stop: float = 14.5,
                  p_turn: float = 18.5, v: float = 0.30, w: float = 0.8) -> dict:
    """Predicted distance, duration and energy for dock -> stations -> dock (powers from electrical_calcs.md)."""
    pts = [world.dock[:2]] + [(s.x, s.y) for s in order] + [world.dock[:2]]
    dist = rot = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        d, r, _ = leg_cost(a, b, world, v, w)
        dist += d
        rot += r
    t_drive, t_turn, t_stop = dist / v, rot / w, dwell_s * len(order)
    e = (p_drive * t_drive + p_turn * t_turn + p_stop * t_stop) / 3600.0
    return dict(distance_m=round(dist, 1), rotation_rad=round(rot, 2), duration_s=round(t_drive + t_turn + t_stop, 0), energy_wh=round(e, 2))


def best_aisle_order(world: World) -> tuple[list[Station], dict]:
    """Exhaustive search over aisle permutations and per-aisle directions (4 aisles -> 384 routes)."""
    n = len(world.aisles_y)
    best = None
    for perm in itertools.permutations(range(n)):
        for dirs in itertools.product((0, 1), repeat=n):
            order = []
            for a, dflag in zip(perm, dirs):
                st = sorted([s for s in world.stations if s.aisle == a], key=lambda s: s.x)
                order += st if dflag == 0 else st[::-1]
            m = route_metrics(order, world)
            if best is None or m["energy_wh"] < best[1]["energy_wh"]:
                best = (order, m)
    return best
