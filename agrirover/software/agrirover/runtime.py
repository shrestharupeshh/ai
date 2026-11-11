"""Rover runtime: wires base interface, odometry, localisation, planning, control, mission logic, vision and storage.

The same class drives the headless simulator (sim/sim2d.py) in tests, in the dashboard's demo mode and in the
research experiments. On the physical robot the equivalent wiring is done by the ROS 2 nodes in ros2_ws, which
reuse the pure-Python modules (kinematics, vision, storage, mission/route) but replace planner/controller by Nav2.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from . import config
from .hw.env_sensors import SimulatedEnvSensors
from .kinematics import Odometry, Pose2D, limit_twist, twist_to_wheels, wrap
from .mission import route
from .nav.controller import ControllerParams, PathFollower
from .nav.localization import LikelihoodField, Localizer
from .nav.mapping import GridMapper
from .nav.planner import Costmap, astar, path_length
from .sim import camera_sim
from .sim.sim2d import RoverSim
from .sim.world import Box, Station, World
from .storage.db import Store
from .vision import pipeline


@dataclass
class MissionParams:
    settle_time_s: float = 1.5          # mast ring-down before capture (mechanical calcs section 9)
    capture_time_s: float = 1.2         # per view: exposure + inference on the SBC
    max_retakes: int = 2
    soc_return: float = 0.25            # abort remaining stations and return to dock
    soc_critical: float = 0.10          # stop where you are
    blocked_replan_s: float = 2.0
    blocked_skip_s: float = 25.0
    goal_timeout_s: float = 180.0


class SimCamera:
    """Camera stand-in: renders the synthetic station view, shifted by the true stopping error of the robot."""

    def __init__(self, world: World, sim: RoverSim, truth: dict[str, camera_sim.PlantTruth], blur_px: float = 0.0, gain: float = 1.0):
        self.world, self.sim, self.truth, self.blur_px, self.gain = world, sim, truth, blur_px, gain
        self.n = 0

    def capture(self, station: Station, view: dict):
        plants = [self.truth[pid] for pid in view["plants"]]
        rows, cols = self.world.cfg["plants"]["rows_per_side"], self.world.cfg["plants"]["per_station"]
        self.n += 1
        s = self.sim.s
        sign = 1.0 if view["camera"] == "left" else -1.0
        ex = (s.x - station.x) * math.cos(station.yaw) + (s.y - station.y) * math.sin(station.yaw)      # along-aisle stop error
        ey = -(s.x - station.x) * math.sin(station.yaw) + (s.y - station.y) * math.cos(station.yaw)     # lateral error
        img, truth = camera_sim.render_view(plants, rows, cols, gain=self.gain, blur_px=self.blur_px, seed=self.n,
                                            shift_px=(-sign * ex * camera_sim.PX_PER_M, sign * ey * camera_sim.PX_PER_M))
        slots = [(p.pid, *camera_sim.slot_px(*divmod(k, cols), rows, cols)) for k, p in enumerate(plants)]
        return img, slots, camera_sim.PX_PER_M, camera_sim.GRAY_PATCH, truth


class Rover:
    DT = 0.02

    def __init__(self, world: World, sim: RoverSim, store: Store, truth: dict | None = None, env=None,
                 params: MissionParams | None = None, epoch: float | None = None, camera=None, save_images: bool = True,
                 build_map: bool = False):
        self.world, self.sim, self.store = world, sim, store
        self.p = params or MissionParams()
        self.epoch = epoch if epoch is not None else time.time()
        self.r, self.track = config.wheel_radius(), config.track_width()
        self.odom = Odometry(self.r, self.track, sim.cpr, Pose2D(*world.dock))
        static = World(world.cfg)                                       # facility drawing: no clutter, no people
        static.obstacles = []
        self.field = LikelihoodField(static.lidar_segments(), (world.w, world.h))
        self.loc = Localizer(self.field, world.dock, sim.lidar_x)
        self._static_grid = world.nav_grid(0.05, include_obstacles=False)
        self.costmap = Costmap(self._static_grid, 0.05, config.footprint_radius())
        self.follower = PathFollower(ControllerParams(half_width=config.half_width() + 0.03))
        self.env = env or SimulatedEnvSensors((world.w, world.h))
        self.truth = truth or {pid: camera_sim.PlantTruth(pid, seed=i) for i, pid in enumerate(world.plants)}
        self.camera = camera or SimCamera(world, sim, self.truth)
        self.save_images = save_images
        self.mapper = GridMapper((world.w, world.h)) if build_map else None
        self.state = "IDLE"
        self.detail = ""
        self.mission_id: int | None = None
        self.queue: list[Station] = []
        self.current: Station | None = None
        self.done_stations: list[str] = []
        self.skipped: list[str] = []
        self.events: list[tuple[float, str, str]] = []
        self._t_state = 0.0
        self._t_blocked: float | None = None
        self._t_goal = 0.0
        self._last_scan = None
        self._scan_xy_robot = None
        self._tick = 0
        self._prev_odom = (self.odom.pose.x, self.odom.pose.y, self.odom.pose.yaw)
        self._view_i = 0
        self._retakes = 0
        self._env_id = None
        self._stop_requested = False
        self._paused_state: str | None = None
        self._dist0 = self._e0 = 0.0
        self.stop_errors: list[tuple[str, float, float]] = []          # (station, position error m, yaw error rad) ground truth
        self.capture_log: list[dict] = []

    # ------------------------------------------------------------------ helpers
    @property
    def now(self) -> float:
        return self.epoch + self.sim.s.t

    @property
    def pose(self):
        return self.loc.pose

    def _event(self, level: str, code: str, msg: str):
        self.events.append((self.sim.s.t, code, msg))
        self.store.add_event(level, code, msg, self.mission_id, self.now)

    def _set(self, state: str, detail: str = ""):
        if state != self.state:
            self.state, self._t_state = state, self.sim.s.t
        self.detail = detail

    def _plan_to(self, x: float, y: float, yaw: float | None) -> bool:
        path = astar(self.costmap, self.pose[:2], (x, y))
        if path is None:
            return False
        self.follower.set_path(path, yaw)
        self._t_goal = self.sim.s.t
        self._t_blocked = None
        return True

    # ------------------------------------------------------------------ operator API (dashboard / tests)
    def start_mission(self, name: str = "inspection", order: list[Station] | None = None) -> int:
        if self.state not in ("IDLE", "DONE", "ABORTED"):
            raise RuntimeError(f"cannot start a mission in state {self.state}")
        self.queue = list(order if order is not None else route.boustrophedon(self.world))
        self.done_stations, self.skipped, self._stop_requested = [], [], False
        self.mission_id = self.store.start_mission(name, len(self.queue), self.now)
        self._dist0, self._e0 = self.sim.s.distance, self.sim.s.energy_wh
        self._event("info", "MISSION_START", f"{name}: {len(self.queue)} stations")
        self._next_station()
        return self.mission_id

    def stop_mission(self):
        """Operator stop: finish nothing, return to the dock."""
        if self.state in ("NAVIGATING", "SETTLING", "CAPTURING"):
            self._stop_requested = True
            self.queue = []
            self._event("warn", "MISSION_STOP", "operator requested stop; returning to dock")
            self._return("operator stop")

    def resume(self):
        """Operator acknowledgement after an E-stop / bumper / fault has been cleared."""
        if self.state == "ESTOP" and not self.sim.s.faults.estop:
            self.sim.clear_bumper()
            self._event("info", "RESUME", "operator resume after stop")
            if self.current is not None and self._plan_to(self.current.x, self.current.y, self.current.yaw):
                self._set("NAVIGATING", self.current.sid)
            else:
                self._return("resume")

    # ------------------------------------------------------------------ mission state machine
    def _next_station(self):
        while self.queue:
            self.current = self.queue.pop(0)
            if self._plan_to(self.current.x, self.current.y, self.current.yaw):
                self._set("NAVIGATING", self.current.sid)
                return
            self.skipped.append(self.current.sid)
            self._event("warn", "STATION_UNREACHABLE", f"no path to {self.current.sid}; skipped")
        self.current = None
        self._return("stations complete")

    def _return(self, why: str):
        self.current = None
        if self._plan_to(self.world.dock[0], self.world.dock[1], self.world.dock[2]):
            self._set("RETURNING", why)
        else:
            self._finish("ABORTED", f"no path to dock ({why})")

    def _finish(self, status: str, why: str = ""):
        self.follower.cancel()
        self._set(status, why)
        if self.mission_id is not None:
            flagged = self.store.query("SELECT COUNT(*) AS n FROM observations WHERE mission_id=? AND status='attention'", (self.mission_id,))[0]["n"]
            self.store.update_mission(self.mission_id, finished=self.now, status=status.lower(), stations_done=len(self.done_stations),
                                      distance_m=round(self.sim.s.distance - self._dist0, 2), energy_wh=round(self.sim.s.energy_wh - self._e0, 3),
                                      summary=dict(reason=why, skipped=self.skipped, plants_flagged=flagged,
                                                   duration_s=round(self.sim.s.t - (self.events[0][0] if self.events else 0), 1)))
        self._event("info" if status == "DONE" else "error", f"MISSION_{status}", why)

    def _capture_view(self) -> bool:
        """Capture and analyse the current view. Returns True when the station is finished."""
        st = self.current
        if self._view_i >= len(st.views):
            return True
        view = st.views[self._view_i]
        self.sim.cv_active = True
        img, slots, ppm, patch, truth = self.camera.capture(st, view)
        res = pipeline.analyse(img, slots, ppm, patch)
        if not res["quality"]["ok"] and self._retakes < self.p.max_retakes:
            self._retakes += 1
            self._event("warn", "IMAGE_RETAKE", f"{st.sid}/{view['camera']}: {','.join(res['quality']['reasons'])}")
            return False
        rel = ""
        if self.save_images:
            rel = f"m{self.mission_id:04d}_{st.sid}_{view['camera']}.jpg"
            cv2.imwrite(str(self.store.image_dir / rel), img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        for o in res["plants"]:
            self.store.add_observation(self.mission_id, self.now, self.world.plants[o["plant_id"]], st.sid, view["camera"], self.pose,
                                       rel, o, res["quality"], self._env_id, res["level"])
        self.capture_log.append(dict(station=st.sid, camera=view["camera"], quality=res["quality"], plants=res["plants"], truth=truth["plants"]))
        self._view_i += 1
        self._retakes = 0
        return self._view_i >= len(st.views)

    def _mission_tick(self, base: dict | None):
        t, p = self.sim.s.t, self.p
        active = self.state in ("NAVIGATING", "SETTLING", "CAPTURING", "RETURNING")
        # ---- safety / health supervision (highest priority first)
        if base is None:
            if self.state != "FAULT":
                self.follower.cancel()
                self._set("FAULT", "base link lost")
                self._event("error", "LINK_LOST", "no telemetry from the base controller; motion stops via firmware time-out")
            return
        if self.state == "FAULT" and self.detail == "base link lost":
            self._event("info", "LINK_RESTORED", "telemetry restored; mission aborted, operator action required")
            self._finish("ABORTED", "base link lost")
            return
        if (base["estop"] or base["bumper"]) and self.state != "ESTOP" and (active or base["estop"]):
            self.follower.cancel()
            self._set("ESTOP", "emergency stop" if base["estop"] else "bumper contact")
            self._event("error", "ESTOP" if base["estop"] else "BUMPER", self.detail)
            return
        if not active:
            return
        if self._last_scan is None:
            self.follower.cancel()
            self._event("error", "LIDAR_FAULT", "no lidar data: protective field unavailable, stopping")
            self._finish("ABORTED", "lidar fault")
            return
        if self.loc.lost:
            self.follower.cancel()
            self._event("error", "LOCALIZATION_LOST", f"scan match score {self.loc.score:.2f}; stopping")
            self._finish("ABORTED", "localization lost")
            return
        if base["soc"] < p.soc_critical:
            self.follower.cancel()
            self._event("error", "BATTERY_CRITICAL", f"SoC {base['soc']*100:.0f} %: stopping in place")
            self._finish("ABORTED", "battery critical")
            return
        if base["soc"] < p.soc_return and self.state != "RETURNING":
            self._event("warn", "BATTERY_LOW", f"SoC {base['soc']*100:.0f} %: aborting remaining {len(self.queue) + 1} stations, returning to dock")
            self.queue = []
            self._return("low battery")
            return
        # ---- nominal sequence
        if self.state in ("NAVIGATING", "RETURNING"):
            if self.follower.blocked:
                self._t_blocked = self._t_blocked if self._t_blocked is not None else t
                waited = t - self._t_blocked
                if waited > p.blocked_replan_s and int(waited / p.blocked_replan_s) != int((waited - self.DT) / p.blocked_replan_s):
                    self._mark_obstacles()
                    goal = (self.current.x, self.current.y, self.current.yaw) if self.current else self.world.dock
                    if self._plan_to(*goal):
                        if waited < 2 * p.blocked_replan_s:
                            self._event("info", "REPLAN", "path blocked: obstacle added to costmap, replanned")
                        self._t_blocked = t - waited            # same blockage episode until motion resumes
                    else:
                        self._t_blocked = t - waited            # keep the clock running
                        self.costmap = Costmap(self._static_grid, 0.05, config.footprint_radius())   # obstacle may be a person: forget and retry
                        self._plan_to(*goal)
                        self._t_blocked = t - waited
                if waited > p.blocked_skip_s and self.state == "NAVIGATING":
                    self.skipped.append(self.current.sid)
                    self._event("warn", "STATION_SKIPPED", f"{self.current.sid}: blocked for {waited:.0f} s")
                    self.costmap = Costmap(self._static_grid, 0.05, config.footprint_radius())
                    self._next_station()
                return
            self._t_blocked = None
            if self.follower.done:
                if self.state == "RETURNING":
                    self._finish("DONE" if not self.skipped and not self._stop_requested and self.detail == "stations complete" else "ABORTED"
                                 if self.detail in ("low battery",) else "DONE", self.detail)
                    return
                s = self.sim.s
                self.stop_errors.append((self.current.sid, math.hypot(s.x - self.current.x, s.y - self.current.y), abs(wrap(s.yaw - self.current.yaw))))
                self._view_i = self._retakes = 0
                self._set("SETTLING", self.current.sid)
            elif t - self._t_goal > p.goal_timeout_s:
                self._event("warn", "GOAL_TIMEOUT", f"{self.current.sid if self.current else 'dock'} not reached in {p.goal_timeout_s:.0f} s")
                if self.state == "NAVIGATING":
                    self.skipped.append(self.current.sid)
                    self._next_station()
                else:
                    self._finish("ABORTED", "dock not reachable")
        elif self.state == "SETTLING":
            if t - self._t_state >= p.settle_time_s:
                try:
                    sample = self.env.read(self.pose[0], self.pose[1], self.now)
                    self._env_id = self.store.add_env(self.mission_id, self.now, self.pose[0], self.pose[1], self.current.sid, sample)
                except Exception as exc:                        # sensor failure must not stop the inspection
                    self._env_id = None
                    self._event("warn", "ENV_SENSOR_FAULT", f"{self.current.sid}: {exc}")
                self._set("CAPTURING", self.current.sid)
        elif self.state == "CAPTURING":
            if t - self._t_state >= p.capture_time_s:
                self._t_state = t
                if self._capture_view():
                    self.sim.cv_active = False
                    self.done_stations.append(self.current.sid)
                    self.store.update_mission(self.mission_id, stations_done=len(self.done_stations))
                    self._next_station()

    def _mark_obstacles(self):
        """Add lidar returns that are not explained by the facility drawing to the costmap (within 2.5 m)."""
        if self._last_scan is None:
            return
        r, a = self._last_scan
        ok = np.isfinite(r) & (r < 2.5)
        x, y, yaw = self.pose
        ox, oy = x + self.sim.lidar_x * math.cos(yaw), y + self.sim.lidar_x * math.sin(yaw)
        pts = np.column_stack((ox + r[ok] * np.cos(a[ok] + yaw), oy + r[ok] * np.sin(a[ok] + yaw)))
        ij = np.round(pts / self.field.res).astype(int)
        inb = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < self.field.field.shape[1]) & (ij[:, 1] < self.field.field.shape[0])
        pts, ij = pts[inb], ij[inb]
        unknown = self.field.field[ij[:, 1], ij[:, 0]] < 0.05          # > ~0.2 m from every mapped feature
        self.costmap.add_obstacles(pts[unknown])

    # ------------------------------------------------------------------ main loop
    def step(self):
        dt, sim = self.DT, self.sim
        base = sim.read_base()
        if base is not None:
            self.odom.update(base["enc_left"], base["enc_right"], base["gyro_z"], dt)
            ox, oy, oyaw = self.odom.pose.as_tuple()
            px, py, pyaw = self._prev_odom
            ds = math.hypot(ox - px, oy - py) * (1 if (ox - px) * math.cos(pyaw) + (oy - py) * math.sin(pyaw) >= 0 else -1)
            self.loc.predict(ds, wrap(oyaw - pyaw))
            self._prev_odom = (ox, oy, oyaw)
        if self._tick % 5 == 0:                                         # 10 Hz lidar
            r = sim.lidar_scan()
            if r is None:
                self._last_scan = self._scan_xy_robot = None
            else:
                a = sim.scan_angles()
                self._last_scan = (r, a)
                ok = np.isfinite(r)
                self._scan_xy_robot = np.column_stack((sim.lidar_x + r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])))
                if self._tick % 25 == 0:                                # 2 Hz scan matching
                    self.loc.correct(r, a)
                    if self.mapper is not None:
                        self.mapper.integrate(self.pose, r, a, sim.lidar_x)
        self._mission_tick(base)
        v, w = self.follower.step(self.pose, dt, self._scan_xy_robot) if self.state in ("NAVIGATING", "RETURNING") else (0.0, 0.0)
        if self.state not in ("NAVIGATING", "RETURNING"):
            self.follower.v = self.follower.w = 0.0
        v, w = limit_twist(v, w, 0.5, 1.2, 12.0, self.r, self.track)
        sim.set_wheels(*twist_to_wheels(v, w, self.r, self.track))
        sim.step(dt)
        if self._tick % 50 == 0 and base is not None:
            self.store.add_status(self.now, self.pose, v, base["vbat"], base["soc"], self.state, self.mission_id,
                                  dict(detail=self.detail, station=self.current.sid if self.current else None, loc_score=round(self.loc.score, 2),
                                       stations_done=len(self.done_stations), stations_left=len(self.queue), i_bat=round(base["i_bat"], 2)))
        self._tick += 1

    def run(self, max_time_s: float = 3600.0, until=("DONE", "ABORTED")) -> str:
        t_end = self.sim.s.t + max_time_s
        while self.sim.s.t < t_end and self.state not in until:
            self.step()
        return self.state


def make_demo(db_path: str | Path = "data/runtime/agrirover.db", seed: int = 1, abnormal_fraction: float = 0.08, build_map: bool = False,
              cfg: dict | None = None, soc: float = 1.0, save_images: bool = True, age_days: float = 21.0) -> Rover:
    """Standard simulated rover in the reference greenhouse with a seeded mix of plant conditions."""
    world = World(cfg)
    sim = RoverSim(world, seed=seed, soc=soc)
    rng = np.random.default_rng(seed)
    truth = {}
    for i, pid in enumerate(world.plants):
        health, sev = "healthy", 0.0
        if rng.random() < abnormal_fraction:
            health = str(rng.choice(["chlorosis", "tipburn", "purpling", "stunted", "missing"]))
            sev = float(rng.uniform(0.5, 1.0))
        truth[pid] = camera_sim.PlantTruth(pid, age_days + float(rng.normal(0, 0.5)), health, sev, seed=i)
    return Rover(world, sim, Store(db_path), truth, SimulatedEnvSensors((world.w, world.h), seed), build_map=build_map, save_images=save_images)
