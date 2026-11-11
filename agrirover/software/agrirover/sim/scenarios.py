"""Simulation test scenarios with measurable acceptance criteria.

    python -m agrirover.sim.scenarios [--out report.json]

Every scenario runs the complete runtime (odometry, scan matching, planning, control, mission logic, vision,
storage) against the headless simulator. PASS here means "the software behaves as specified in this model";
it is not evidence of physical performance. See docs/10_validation_test_report.md.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .. import config
from ..runtime import MissionParams, Rover, SimCamera
from ..hw.env_sensors import SimulatedEnvSensors
from ..storage.db import Store
from . import camera_sim
from .sim2d import RoverSim
from .world import Box, World


def small_cfg(aisle: float = 0.8, benches: int = 2, stations: int = 3, length: float = 4.0, outer_aisle: float | None = None) -> dict:
    """A reduced greenhouse (fast to simulate) with the same construction rules as the reference house.
    `aisle` is the width between benches; `outer_aisle` the two wall-side aisles (defaults to `aisle`)."""
    cfg = copy.deepcopy(config.greenhouse())
    outer = aisle if outer_aisle is None else outer_aisle
    bw = cfg["bench"]["width"]
    cfg["bench"]["length"] = length
    cfg["benches_y0"] = [round(outer + i * (bw + aisle), 3) for i in range(benches)]
    cfg["room"] = {"x": cfg["bench"]["x0"] + length + 2.0, "y": round(2 * outer + benches * bw + (benches - 1) * aisle, 3)}
    cfg["aisle_width"] = aisle
    cfg["dock"] = {"x": 1.0, "y": round(max(outer / 2, 0.45), 3), "yaw": 0.0}       # dock stands in the open head corridor
    cfg["stations"]["count"] = stations
    cfg["obstacles"] = []
    return cfg


def build(cfg: dict | None = None, seed: int = 3, soc: float = 1.0, abnormal: float = 0.15, params: MissionParams | None = None,
          env=None, blur_px: float = 0.0, tmp: str | None = None) -> Rover:
    world = World(cfg if cfg is not None else small_cfg())
    sim = RoverSim(world, seed=seed, soc=soc)
    rng = np.random.default_rng(seed)
    truth = {}
    for i, pid in enumerate(world.plants):
        health, sev = "healthy", 0.0
        if rng.random() < abnormal:
            health, sev = str(rng.choice(["chlorosis", "tipburn", "purpling", "stunted", "missing"])), float(rng.uniform(0.7, 1.0))
        truth[pid] = camera_sim.PlantTruth(pid, 21.0, health, sev, seed=i)
    store = Store(Path(tmp or tempfile.mkdtemp(prefix="agrirover_")) / "sim.db")
    rover = Rover(world, sim, store, truth, env or SimulatedEnvSensors((world.w, world.h), seed), params=params, save_images=False,
                  epoch=1_780_000_000.0)
    rover.camera = SimCamera(world, sim, truth, blur_px=blur_px)
    return rover


@dataclass
class Result:
    name: str
    purpose: str
    passed: bool
    criteria: list = field(default_factory=list)      # [(text, ok, measured)]
    metrics: dict = field(default_factory=dict)

    def as_dict(self):
        return dict(name=self.name, purpose=self.purpose, passed=self.passed,
                    criteria=[dict(criterion=c, ok=bool(o), measured=m) for c, o, m in self.criteria], metrics=self.metrics)


def _metrics(r: Rover) -> dict:
    s = r.sim.s
    err = np.array([[a, b] for _, a, b in r.stop_errors]) if r.stop_errors else np.zeros((0, 2))
    return dict(final_state=r.state, detail=r.detail, sim_time_s=round(s.t, 1), distance_m=round(s.distance, 2), energy_wh=round(s.energy_wh, 3),
                collisions=s.collisions, min_clearance_m=round(s.min_clearance, 3), stations_done=len(r.done_stations), skipped=r.skipped,
                stop_err_max_m=round(float(err[:, 0].max()), 3) if len(err) else None,
                stop_yaw_err_max_deg=round(float(np.degrees(err[:, 1].max())), 2) if len(err) else None,
                dock_error_m=round(math.hypot(s.x - r.world.dock[0], s.y - r.world.dock[1]), 3),
                events=sorted({e[1] for e in r.events}))


def _result(name, purpose, r: Rover, checks) -> Result:
    m = _metrics(r)
    crit = [(text, bool(fn(m, r)), str(meas(m, r))) for text, fn, meas in checks]
    return Result(name, purpose, all(c[1] for c in crit), crit, m)


NO_COLLISION = ("no collision", lambda m, r: m["collisions"] == 0, lambda m, r: m["collisions"])


def s_nominal() -> Result:
    r = build()
    r.start_mission()
    r.run(900)
    n = len(r.world.stations)
    return _result("nominal_mission", "Dock -> all stations -> capture -> dock, 0.8 m aisles", r, [
        ("mission state DONE", lambda m, r: m["final_state"] == "DONE", lambda m, r: m["final_state"]),
        (f"all {n} stations inspected", lambda m, r: m["stations_done"] == n, lambda m, r: m["stations_done"]),
        NO_COLLISION,
        ("min clearance >= 0.05 m", lambda m, r: m["min_clearance_m"] >= 0.05, lambda m, r: m["min_clearance_m"]),
        ("stop position error <= 0.05 m", lambda m, r: m["stop_err_max_m"] <= 0.05, lambda m, r: m["stop_err_max_m"]),
        ("stop heading error <= 3 deg", lambda m, r: m["stop_yaw_err_max_deg"] <= 3.0, lambda m, r: m["stop_yaw_err_max_deg"]),
        ("returns within 0.10 m of the dock", lambda m, r: m["dock_error_m"] <= 0.10, lambda m, r: m["dock_error_m"]),
        ("every plant in view has a stored observation", lambda m, r: r.store.query("SELECT COUNT(DISTINCT plant_id) n FROM observations")[0]["n"] == len(r.world.plants),
         lambda m, r: r.store.query("SELECT COUNT(DISTINCT plant_id) n FROM observations")[0]["n"]),
        ("one environment sample per station", lambda m, r: r.store.query("SELECT COUNT(*) n FROM env_samples")[0]["n"] == n,
         lambda m, r: r.store.query("SELECT COUNT(*) n FROM env_samples")[0]["n"]),
        ("no false obstacle detections in an empty house", lambda m, r: "OBSTACLE_DETECTED" not in m["events"], lambda m, r: m["events"]),
    ])


def s_narrow_aisle() -> Result:
    r = build(small_cfg(aisle=0.60, outer_aisle=0.80, benches=3))
    r.start_mission()
    r.run(900)
    return _result("narrow_aisle_0.60m", "Transit and inspection in 0.60 m aisles between benches (R-ENV-03); wall-side aisles 0.80 m", r, [
        ("mission state DONE", lambda m, r: m["final_state"] == "DONE", lambda m, r: m["final_state"]),
        NO_COLLISION,
        ("min clearance >= 0.03 m", lambda m, r: m["min_clearance_m"] >= 0.03, lambda m, r: m["min_clearance_m"]),
        ("no station skipped", lambda m, r: not m["skipped"], lambda m, r: m["skipped"]),
        ("stop position error <= 0.05 m", lambda m, r: m["stop_err_max_m"] <= 0.05, lambda m, r: m["stop_err_max_m"]),
    ])


def s_static_obstacle() -> Result:
    r = build()
    w = r.world
    w.dynamic.append(Box("crate", w.benches[0].x1 + 0.35, w.aisles_y[0] + 0.25, w.benches[0].x1 + 0.75, w.aisles_y[0] + 0.95))   # in the tail corridor
    r.start_mission()
    r.run(1200)
    return _result("unmapped_obstacle", "Crate not in the facility drawing blocks the planned line through the tail corridor", r, [
        ("mission state DONE", lambda m, r: m["final_state"] == "DONE", lambda m, r: m["final_state"]),
        NO_COLLISION,
        ("obstacle detected and added to the costmap", lambda m, r: "OBSTACLE_DETECTED" in m["events"], lambda m, r: m["events"]),
        ("min clearance >= 0.03 m", lambda m, r: m["min_clearance_m"] >= 0.03, lambda m, r: m["min_clearance_m"]),
    ])


def s_blocked_aisle() -> Result:
    r = build(params=MissionParams(blocked_skip_s=12.0))
    w = r.world
    y = w.aisles_y[1]
    w.dynamic.append(Box("cart", 4.9, y - 0.4, 5.3, y + 0.4))            # cart completely blocking aisle 1
    r.start_mission()
    r.run(1800)
    n = len(w.stations)
    return _result("blocked_aisle", "A parked cart closes an aisle completely: reroute or skip, never push through", r, [
        ("mission terminates at the dock (DONE)", lambda m, r: m["final_state"] == "DONE" and m["dock_error_m"] < 0.1, lambda m, r: (m["final_state"], m["dock_error_m"])),
        NO_COLLISION,
        ("every station is either inspected or reported skipped", lambda m, r: m["stations_done"] + len(m["skipped"]) == n, lambda m, r: (m["stations_done"], m["skipped"])),
        ("blockage reported (OBSTACLE_DETECTED / REPLAN / STATION_SKIPPED)", lambda m, r: bool({"OBSTACLE_DETECTED", "REPLAN", "STATION_SKIPPED"} & set(m["events"])), lambda m, r: m["events"]),
    ])


def s_person() -> Result:
    r = build()
    r.start_mission()
    t_in = t_out = None
    stopped_d = None
    while r.sim.s.t < 900 and r.state not in ("DONE", "ABORTED"):
        s = r.sim.s
        if t_in is None and r.state == "NAVIGATING" and r.follower.v > 0.25 and abs(math.sin(s.yaw)) < 0.1:
            d = 0.62 * (1 if math.cos(s.yaw) > 0 else -1)                # person steps out 0.22 m ahead of the bumper (inside the protective field)
            r.world.dynamic.append(Box("person", s.x + d - 0.15, s.y - 0.2, s.x + d + 0.15, s.y + 0.2))
            t_in, x_in = s.t, s.x
        if t_in is not None and t_out is None:
            if abs(r.sim.s.wl) < 0.05 and stopped_d is None and s.t - t_in > 0.2:
                stopped_d = abs(s.x - x_in)
            if s.t - t_in > 8.0:
                r.world.dynamic.clear()
                t_out = s.t
        r.step()
    res = _result("person_steps_into_path", "A person steps 0.22 m in front of the bumper at 0.3 m/s, stays 8 s, then leaves", r, [
        ("mission state DONE", lambda m, r: m["final_state"] == "DONE", lambda m, r: m["final_state"]),
        NO_COLLISION,
        ("stopping distance <= 0.15 m (R-SAF-02)", lambda m, r: stopped_d is not None and stopped_d <= 0.15, lambda m, r: None if stopped_d is None else round(stopped_d, 3)),
        ("never closer than 0.05 m to the person", lambda m, r: m["min_clearance_m"] >= 0.05, lambda m, r: m["min_clearance_m"]),
    ])
    return res


def s_low_battery() -> Result:
    r = build(soc=0.2515)
    r.start_mission()
    r.run(900)
    return _result("low_battery", "State of charge crosses the 25 % return threshold during the mission", r, [
        ("BATTERY_LOW raised", lambda m, r: "BATTERY_LOW" in m["events"], lambda m, r: m["events"]),
        ("returns to dock (<= 0.10 m)", lambda m, r: m["dock_error_m"] <= 0.10, lambda m, r: m["dock_error_m"]),
        ("mission recorded as aborted, not done", lambda m, r: r.store.missions()[0]["status"] == "aborted", lambda m, r: r.store.missions()[0]["status"]),
        NO_COLLISION,
    ])


def _run_until_moving(r: Rover, t_min: float = 12.0):
    while not (r.sim.s.t > t_min and r.state == "NAVIGATING" and r.follower.v > 0.28):
        r.step()


def s_lidar_failure() -> Result:
    r = build()
    r.start_mission()
    _run_until_moving(r)
    d0 = r.sim.s.distance
    r.sim.s.faults.lidar_dead = True
    r.run(30)
    for _ in range(100):
        r.step()
    d = r.sim.s.distance - d0
    return _result("lidar_failure", "Lidar stops delivering scans while driving", r, [
        ("mission ABORTED with LIDAR_FAULT", lambda m, r: m["final_state"] == "ABORTED" and "LIDAR_FAULT" in m["events"], lambda m, r: (m["final_state"], m["detail"])),
        ("stops within 0.15 m of the fault", lambda m, r: d <= 0.15, lambda m, r: round(d, 3)),
        NO_COLLISION,
    ])


def s_estop() -> Result:
    r = build()
    r.start_mission()
    _run_until_moving(r)
    d0 = r.sim.s.distance
    r.sim.s.faults.estop = True
    for _ in range(150):
        r.step()
    d = r.sim.s.distance - d0
    state_during = r.state
    r.resume()                                   # must be refused while the button is still pressed
    refused = r.state == "ESTOP"
    r.sim.s.faults.estop = False
    r.resume()
    r.run(900)
    return _result("emergency_stop", "E-stop pressed while driving; released; operator resumes", r, [
        ("stops within 0.15 m (R-SAF-02)", lambda m, r: d <= 0.15, lambda m, r: round(d, 3)),
        ("state ESTOP while pressed", lambda m, r: state_during == "ESTOP", lambda m, r: state_during),
        ("resume refused while still pressed", lambda m, r: refused, lambda m, r: refused),
        ("mission completes after release + operator resume", lambda m, r: m["final_state"] == "DONE", lambda m, r: m["final_state"]),
        NO_COLLISION,
    ])


def s_link_loss() -> Result:
    r = build()
    r.start_mission()
    _run_until_moving(r)
    d0 = r.sim.s.distance
    r.sim.s.faults.link_lost = True
    for _ in range(250):
        r.step()
    d = r.sim.s.distance - d0
    st = r.state
    r.sim.s.faults.link_lost = False
    for _ in range(50):
        r.step()
    return _result("lost_base_link", "SBC <-> base-controller link drops for 5 s while driving", r, [
        ("rover stops by firmware time-out within 0.20 m", lambda m, r: d <= 0.20, lambda m, r: round(d, 3)),
        ("FAULT state during the outage", lambda m, r: st == "FAULT", lambda m, r: st),
        ("mission ABORTED after the link returns (no automatic restart)", lambda m, r: m["final_state"] == "ABORTED", lambda m, r: m["final_state"]),
        NO_COLLISION,
    ])


def s_env_sensor_failure() -> Result:
    r = build(env=SimulatedEnvSensors(failed=True))
    r.start_mission()
    r.run(900)
    return _result("env_sensor_failure", "Environmental sensor bus fails (I2C error on every read)", r, [
        ("mission still completes", lambda m, r: m["final_state"] == "DONE", lambda m, r: m["final_state"]),
        ("ENV_SENSOR_FAULT reported", lambda m, r: "ENV_SENSOR_FAULT" in m["events"], lambda m, r: m["events"]),
        ("plant observations stored without an environment link", lambda m, r: r.store.query("SELECT COUNT(*) n FROM observations WHERE env_id IS NULL")[0]["n"] > 0,
         lambda m, r: r.store.query("SELECT COUNT(*) n FROM observations WHERE env_id IS NULL")[0]["n"]),
    ])


def s_localization_failure() -> Result:
    r = build()
    r.start_mission()
    _run_until_moving(r)
    s = r.sim.s
    s.x, s.y, s.yaw = s.x + 0.9, s.y + 0.05, s.yaw + 0.5            # "kidnap": the estimate is suddenly wrong
    t0 = s.t
    r.run(60)
    dt = s.t - t0
    return _result("localization_failure", "Pose estimate suddenly wrong by 0.9 m / 29 deg (robot pushed or map mismatch)", r, [
        ("LOCALIZATION_LOST raised and mission ABORTED", lambda m, r: "LOCALIZATION_LOST" in m["events"] and m["final_state"] == "ABORTED", lambda m, r: (m["final_state"], m["detail"])),
        ("detected within 10 s", lambda m, r: dt <= 10.0, lambda m, r: round(dt, 1)),
    ])


def s_wheel_slip() -> Result:
    r = build()
    r.sim.s.faults.slip_left = 0.10
    r.start_mission()
    r.run(1200)
    return _result("wheel_slip_10pct", "Left wheel loses 10 % of its travel (wet floor) for the whole mission", r, [
        ("mission state DONE", lambda m, r: m["final_state"] == "DONE", lambda m, r: m["final_state"]),
        NO_COLLISION,
        ("stop position error <= 0.08 m despite slip", lambda m, r: m["stop_err_max_m"] <= 0.08, lambda m, r: m["stop_err_max_m"]),
    ])


def s_blurred_images() -> Result:
    r = build(blur_px=13.0)
    r.start_mission()
    r.run(1200)
    rows = r.store.query("SELECT status, COUNT(*) n FROM observations GROUP BY status")
    counts = {x["status"]: x["n"] for x in rows}
    return _result("blurred_images", "Camera delivers motion-blurred frames (13 px) at every stop", r, [
        ("retakes attempted", lambda m, r: "IMAGE_RETAKE" in m["events"], lambda m, r: m["events"]),
        ("all observations marked unreliable (no plant flagged from a bad frame)", lambda m, r: set(counts) <= {"unreliable"}, lambda m, r: counts),
        ("mission still completes", lambda m, r: m["final_state"] == "DONE", lambda m, r: m["final_state"]),
    ])


def s_detection_accuracy() -> Result:
    """Software verification of the vision pipeline against synthetic ground truth - not an agronomic claim."""
    r = build(abnormal=0.25, seed=11)
    r.start_mission()
    r.run(900)
    tp = fp = fn = tn = 0
    area_err = []
    for c in r.capture_log:
        for o, t in zip(c["plants"], c["truth"]):
            bad, flagged = t["health"] != "healthy", o["status"] in ("attention", "missing")
            tp += bad and flagged
            fn += bad and not flagged
            fp += (not bad) and flagged
            tn += (not bad) and not flagged
            if t["area_px"] > 0 and o["present"]:
                area_err.append(abs(o["area_cm2"] - t["area_px"] / camera_sim.PX_PER_M ** 2 * 1e4) / (t["area_px"] / camera_sim.PX_PER_M ** 2 * 1e4))
    rec, prec = tp / max(tp + fn, 1), tp / max(tp + fp, 1)
    res = _result("vision_on_synthetic_truth", "Flag recall / precision and canopy-area error on synthetic plants (severity 0.7-1.0)", r, [
        ("recall >= 0.90 on synthetic abnormal plants", lambda m, r: rec >= 0.90, lambda m, r: f"{rec:.3f} ({tp}/{tp+fn})"),
        ("precision >= 0.95", lambda m, r: prec >= 0.95, lambda m, r: f"{prec:.3f} ({tp}/{tp+fp})"),
        ("median canopy-area error <= 5 %", lambda m, r: float(np.median(area_err)) <= 0.05, lambda m, r: f"{float(np.median(area_err))*100:.1f} %"),
    ])
    res.metrics.update(tp=tp, fp=fp, fn=fn, tn=tn)
    return res


SCENARIOS = [s_nominal, s_narrow_aisle, s_static_obstacle, s_blocked_aisle, s_person, s_low_battery, s_lidar_failure, s_estop,
             s_link_loss, s_env_sensor_failure, s_localization_failure, s_wheel_slip, s_blurred_images, s_detection_accuracy]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None)
    ap.add_argument("--only", default=None)
    a = ap.parse_args()
    results = []
    for fn in SCENARIOS:
        if a.only and a.only not in fn.__name__:
            continue
        res = fn()
        results.append(res)
        print(f"{'PASS' if res.passed else 'FAIL'}  {res.name:28s} {res.metrics.get('final_state', ''):8s} t={res.metrics.get('sim_time_s')} s")
        for c, ok, meas in res.criteria:
            print(f"      [{'x' if ok else ' '}] {c}: {meas}")
    if a.out:
        Path(a.out).write_text(json.dumps([r.as_dict() for r in results], indent=2, default=str) + "\n")
    return 0 if all(r.passed for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
