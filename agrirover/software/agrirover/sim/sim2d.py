"""Headless 2D rover simulator: differential-drive dynamics, encoders, gyro, 2D lidar, battery, bumper.

Fidelity: kinematic body with first-order wheel-speed response and noise models. It is a software-in-the-loop
test bench for navigation and mission logic, NOT a substitute for Gazebo physics or for hardware tests.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .. import config
from ..kinematics import twist_to_wheels, wheels_to_twist, wrap
from .world import World


@dataclass
class Faults:
    lidar_dead: bool = False
    link_lost: bool = False          # SBC <-> MCU link
    estop: bool = False
    slip_left: float = 0.0           # fraction of commanded wheel motion lost to slip
    slip_right: float = 0.0
    encoder_dead_left: bool = False


@dataclass
class SimState:
    t: float = 0.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    wl: float = 0.0
    wr: float = 0.0
    enc_l: float = 0.0
    enc_r: float = 0.0
    soc_wh: float = 0.0
    bumper: bool = False
    collided: str | None = None
    distance: float = 0.0
    energy_wh: float = 0.0
    min_clearance: float = math.inf
    collisions: int = 0
    faults: Faults = field(default_factory=Faults)


class RoverSim:
    LIDAR_RAYS = 500          # RPLIDAR C1: 5 kHz sample rate at 10 Hz
    LIDAR_RANGE = 12.0
    LIDAR_MIN = 0.05

    def __init__(self, world: World, pose: tuple[float, float, float] | None = None, seed: int = 0, soc: float = 1.0):
        self.world = world
        self.rng = np.random.default_rng(seed)
        cfg = config.rover()
        self.r, self.track = config.wheel_radius(), config.track_width()
        self.cpr = cfg["motor"]["encoder_cpr_motor"] * cfg["motor"]["gear_ratio"]
        self.capacity_wh = cfg["battery"]["nominal_v"] * cfg["battery"]["capacity_ah"]
        self.loads = cfg["power_loads_w"]
        c = cfg["chassis"]
        self.lidar_x = c["lidar_x_mm"] / 1000
        x, y, yaw = pose or world.dock
        self.s = SimState(x=x, y=y, yaw=yaw, soc_wh=self.capacity_wh * soc)
        self.cmd = (0.0, 0.0)             # wheel set-points rad/s
        self.t_cmd = 0.0
        self.enabled = True
        self.gyro_bias = float(self.rng.normal(0, 0.002))
        self.cv_active = False
        self._gyro, self._p_now = 0.0, 0.0
        outline = config.footprint_outline()
        fp = []
        for (x0, y0), (x1, y1) in zip(outline, outline[1:] + outline[:1]):
            fp += [(x0 + (x1 - x0) * k / 6, y0 + (y1 - y0) * k / 6) for k in range(6)]
        self._fp = np.array(fp)

    # ------------------------------------------------------------------ interface used by the "brain"
    def set_wheels(self, wl: float, wr: float):
        if not self.s.faults.link_lost:
            self.cmd = (wl, wr)
            self.t_cmd = self.s.t

    def read_base(self) -> dict | None:
        """Telemetry as the MCU would report it; None when the link is down."""
        if self.s.faults.link_lost:
            return None
        s = self.s
        return dict(t=s.t, enc_left=int(round(s.enc_l)), enc_right=int(round(s.enc_r)),
                    gyro_z=self._gyro, vbat=self.battery_voltage(), soc=s.soc_wh / self.capacity_wh,
                    bumper=s.bumper, estop=s.faults.estop, i_bat=self._p_now / max(self.battery_voltage(), 1.0))

    def battery_voltage(self) -> float:
        soc = max(0.0, min(1.0, self.s.soc_wh / self.capacity_wh))
        # flat LiFePO4 curve: knee below 10 %, small rise above 95 %
        v = 12.8 + 0.5 * (soc - 0.5)
        if soc < 0.10:
            v -= (0.10 - soc) * 20.0
        if soc > 0.95:
            v += (soc - 0.95) * 12.0
        return v

    # ------------------------------------------------------------------ dynamics
    def _footprint_hit(self, x, y, yaw) -> str | None:
        c, s = math.cos(yaw), math.sin(yaw)
        pts = np.column_stack((x + c * self._fp[:, 0] - s * self._fp[:, 1], y + s * self._fp[:, 0] + c * self._fp[:, 1]))
        w = self.world
        if (pts[:, 0] < 0).any() or (pts[:, 1] < 0).any() or (pts[:, 0] > w.w).any() or (pts[:, 1] > w.h).any():
            return "wall"
        for bx in w.benches + w.obstacles + w.dynamic:
            if ((pts[:, 0] > bx.x0) & (pts[:, 0] < bx.x1) & (pts[:, 1] > bx.y0) & (pts[:, 1] < bx.y1)).any():
                return bx.name
        return None

    def clearance(self) -> float:
        """Smallest distance from the footprint outline to anything physical (ground truth)."""
        s = self.s
        c, sn = math.cos(s.yaw), math.sin(s.yaw)
        pts = np.column_stack((s.x + c * self._fp[:, 0] - sn * self._fp[:, 1], s.y + sn * self._fp[:, 0] + c * self._fp[:, 1]))
        w = self.world
        d = min(pts[:, 0].min(), pts[:, 1].min(), (w.w - pts[:, 0]).min(), (w.h - pts[:, 1]).min())
        for bx in w.benches + w.obstacles + w.dynamic:
            dx = np.maximum.reduce([bx.x0 - pts[:, 0], np.zeros(len(pts)), pts[:, 0] - bx.x1])
            dy = np.maximum.reduce([bx.y0 - pts[:, 1], np.zeros(len(pts)), pts[:, 1] - bx.y1])
            d = min(d, float(np.sqrt(dx * dx + dy * dy).min()))
        return d

    def step(self, dt: float = 0.02):
        s, f = self.s, self.s.faults
        wl_set, wr_set = self.cmd
        if s.t - self.t_cmd > 0.3 or f.estop or s.bumper or not self.enabled:     # firmware command timeout / stops
            wl_set = wr_set = 0.0
        tau = 0.08
        a_max = 30.0 * dt
        s.wl += max(-a_max, min(a_max, (wl_set - s.wl) * dt / tau))
        s.wr += max(-a_max, min(a_max, (wr_set - s.wr) * dt / tau))
        # encoders see wheel rotation; the body sees rotation minus slip
        s.enc_l += 0.0 if f.encoder_dead_left else s.wl * dt * self.cpr / (2 * math.pi)
        s.enc_r += s.wr * dt * self.cpr / (2 * math.pi)
        wl_eff = s.wl * (1 - f.slip_left) * (1 + self.rng.normal(0, 0.003))
        wr_eff = s.wr * (1 - f.slip_right) * (1 + self.rng.normal(0, 0.003))
        v, w = wheels_to_twist(wl_eff, wr_eff, self.r, self.track)
        nx, ny, nyaw = s.x + v * math.cos(s.yaw + w * dt / 2) * dt, s.y + v * math.sin(s.yaw + w * dt / 2) * dt, wrap(s.yaw + w * dt)
        hit = self._footprint_hit(nx, ny, nyaw)
        if hit:
            if not s.collided:
                s.collisions += 1
            s.collided, s.bumper = hit, True
            s.wl = s.wr = 0.0
            w = v = 0.0
        else:
            s.distance += abs(v) * dt
            s.x, s.y, s.yaw = nx, ny, nyaw
            s.collided = None
        self._gyro = w + self.gyro_bias + float(self.rng.normal(0, 0.003))
        # energy
        L = self.loads
        p_drive = 0.0
        if abs(s.wl) + abs(s.wr) > 0.05:
            p_drive = 1.5 * (1.2 + 9.0 * abs(v) + 2.0 * abs(w))      # ~3.9 W at 0.3 m/s (mechanical calcs section 10) + 50 %
        sbc = L["sbc_cv_peak"] if self.cv_active else (L["sbc_nav"] if p_drive > 0 else 0.7 * L["sbc_nav"])
        self._p_now = (sbc + L["lidar"] + L["cameras"] + L["mcu_sensors"] + L["wifi_extra"] + L["leds_misc"]) / L["dcdc_efficiency"] + p_drive
        e = self._p_now * dt / 3600.0
        s.soc_wh = max(0.0, s.soc_wh - e)
        s.energy_wh += e
        s.t += dt
        s.min_clearance = min(s.min_clearance, self.clearance())

    def clear_bumper(self):
        self.s.bumper = False

    # ------------------------------------------------------------------ sensors
    def lidar_scan(self) -> np.ndarray | None:
        """Ranges (LIDAR_RAYS,) in the lidar frame, inf where nothing is hit; None if the sensor is dead."""
        if self.s.faults.lidar_dead:
            return None
        s = self.s
        ox, oy = s.x + self.lidar_x * math.cos(s.yaw), s.y + self.lidar_x * math.sin(s.yaw)
        ang = s.yaw + np.linspace(-math.pi, math.pi, self.LIDAR_RAYS, endpoint=False)
        dx, dy = np.cos(ang)[:, None], np.sin(ang)[:, None]
        seg = self.world.lidar_segments()
        x1, y1, x2, y2 = seg[:, 0][None], seg[:, 1][None], seg[:, 2][None], seg[:, 3][None]
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
            u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
        t = np.where((np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1), t, np.inf)
        r = t.min(axis=1) + self.rng.normal(0, 0.01, self.LIDAR_RAYS)
        r[(r > self.LIDAR_RANGE) | (r < self.LIDAR_MIN)] = np.inf
        return r

    def scan_angles(self) -> np.ndarray:
        return np.linspace(-math.pi, math.pi, self.LIDAR_RAYS, endpoint=False)
