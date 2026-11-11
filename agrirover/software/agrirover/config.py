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


def footprint_radius() -> float:
    """Swept radius when turning in place (m): plate corner, see mechanical calcs section 8."""
    c = rover()["chassis"]
    import math
    L, W, ch = c["plate_length_mm"], c["plate_width_mm"], c["plate_corner_chamfer_mm"]
    return max(math.hypot(L / 2, W / 2 - ch), math.hypot(L / 2 - ch, W / 2), L / 2 + 12) / 1000.0


def half_width() -> float:
    c = rover()["chassis"]
    return (c["track_width_mm"] + c["wheel_width_mm"]) / 2000.0
