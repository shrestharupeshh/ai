"""Load the shared design / site configuration files."""
from __future__ import annotations

import os
from functools import lru_cache
from pathlib import Path

import yaml


def project_root() -> Path:
    env = os.environ.get("AGRIROVER_ROOT")
    if env:
        return Path(env)
    here = Path(__file__).resolve()
    for p in here.parents:
        if (p / "config" / "rover.yaml").exists():
            return p
    raise FileNotFoundError("config/rover.yaml not found; set AGRIROVER_ROOT")


@lru_cache(maxsize=None)
def rover() -> dict:
    return yaml.safe_load((project_root() / "config" / "rover.yaml").read_text())


def greenhouse(path: str | os.PathLike | None = None) -> dict:
    p = Path(path) if path else project_root() / "config" / "greenhouse.yaml"
    return yaml.safe_load(p.read_text())


def wheel_radius() -> float:
    return rover()["chassis"]["wheel_diameter_mm"] / 2000.0


def track_width() -> float:
    return rover()["chassis"]["track_width_mm"] / 1000.0


def footprint_outline() -> list[tuple[float, float]]:
    """Convex outline of the rover in the base frame (m): bumper, chamfered deck, wheels. Matches mechanical/cad."""
    c = rover()["chassis"]
    L, W, ch = c["plate_length_mm"] / 1000, c["plate_width_mm"] / 1000, c["plate_corner_chamfer_mm"] / 1000
    hw = (c["track_width_mm"] + c["wheel_width_mm"]) / 2000
    wr = c["wheel_diameter_mm"] / 2000
    bx, by = L / 2 + 0.020, (W - 2 * ch + 0.030) / 2
    return [(bx, by), (L / 2 - ch, W / 2), (wr, hw), (-wr, hw), (-L / 2 + ch, W / 2), (-L / 2, W / 2 - ch),
            (-L / 2, -W / 2 + ch), (-L / 2 + ch, -W / 2), (-wr, -hw), (wr, -hw), (L / 2 - ch, -W / 2), (bx, -by)]


def footprint_radius() -> float:
    """Circumscribed radius = turn-in-place envelope (m); see mechanical calcs section 8."""
    import math
    return max(math.hypot(x, y) for x, y in footprint_outline())


def inscribed_radius() -> float:
    """Half the overall width (m): the narrowest gap the rover can drive straight through."""
    return half_width()


def half_width() -> float:
    c = rover()["chassis"]
    return (c["track_width_mm"] + c["wheel_width_mm"]) / 2000.0
