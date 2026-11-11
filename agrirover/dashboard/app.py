"""AgriRover dashboard backend (FastAPI).

    AGRIROVER_MODE=sim uvicorn dashboard.app:app --port 8080        # from the agrirover/ directory

Modes
  sim   (default) the headless simulator runs in a background thread - a complete demo with no hardware
  db    read-only view of an existing database (e.g. copied from the robot); mission control is disabled
On the robot, the ROS 2 node `agrirover_apps/dashboard_bridge` implements the same RobotLink interface
(start / stop / resume / status) against Nav2 and the mission node; see docs/08_dashboard_api.md.
"""
from __future__ import annotations

import json
import os
import sys
import threading
import time
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "software"))

from agrirover import config  # noqa: E402
from agrirover.mission import route  # noqa: E402
from agrirover.sim.world import World  # noqa: E402
from agrirover.storage.db import Store  # noqa: E402

MODE = os.environ.get("AGRIROVER_MODE", "sim")
DATA = Path(os.environ.get("AGRIROVER_DATA", ROOT / "data" / "runtime"))
SPEED = float(os.environ.get("AGRIROVER_SIM_SPEED", "8"))


class SimLink:
    """RobotLink implementation backed by the headless simulator."""

    def __init__(self, data_dir: Path, speed: float):
        from agrirover.runtime import make_demo
        self.rover = make_demo(data_dir / "agrirover.db", seed=int(time.time()) % 1000, abnormal_fraction=0.08)
        self.rover.epoch = time.time() - self.rover.sim.s.t
        self.store, self.world, self.speed = self.rover.store, self.rover.world, speed
        self._cmds: list = []
        self._lock = threading.Lock()
        self._run = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        r = self.rover
        while self._run:
            t0 = time.time()
            with self._lock:
                cmds, self._cmds = self._cmds, []
            for fn in cmds:
                try:
                    fn()
                except Exception as exc:        # surfaced to the operator through the event log
                    r.store.add_event("error", "COMMAND_REJECTED", str(exc), r.mission_id)
            for _ in range(max(1, int(self.speed * 0.1 / r.DT))):
                r.step()
            time.sleep(max(0.0, 0.1 - (time.time() - t0)))

    def _post(self, fn):
        with self._lock:
            self._cmds.append(fn)

    def start(self, name: str, order: str):
        w = self.world
        stations = {"serpentine": route.boustrophedon, "naive": route.naive_order, "optimal": lambda x: route.best_aisle_order(x)[0]}[order](w)
        self._post(lambda: self.rover.start_mission(name, stations))

    def stop(self):
        self._post(self.rover.stop_mission)

    def resume(self):
        self._post(self.rover.resume)

    def set_fault(self, name: str, value: bool):
        f = self.rover.sim.s.faults
        if not hasattr(f, name):
            raise KeyError(name)
        self._post(lambda: setattr(f, name, value))

    def live(self) -> dict:
        r, s = self.rover, self.rover.sim.s
        return dict(x=round(r.pose[0], 3), y=round(r.pose[1], 3), yaw=round(r.pose[2], 3), state=r.state, detail=r.detail,
                    soc=round(s.soc_wh / r.sim.capacity_wh, 4), battery_v=round(r.sim.battery_voltage(), 2), mission_id=r.mission_id,
                    station=r.current.sid if r.current else None, stations_done=len(r.done_stations), stations_left=len(r.queue),
                    loc_score=round(r.loc.score, 2), estop=s.faults.estop, sim_time_s=round(s.t, 1), speed_factor=self.speed)


class DbLink:
    """Read-only link: shows whatever is in the database."""

    def __init__(self, data_dir: Path):
        self.store, self.world = Store(data_dir / "agrirover.db"), World()

    def live(self) -> dict:
        st = self.store.latest_status() or {}
        d = json.loads(st.get("detail") or "{}")
        return dict(x=st.get("x"), y=st.get("y"), yaw=st.get("yaw"), state=st.get("state", "UNKNOWN"), detail=d.get("detail", ""),
                    soc=st.get("soc"), battery_v=st.get("battery_v"), mission_id=st.get("mission_id"), station=d.get("station"),
                    stations_done=d.get("stations_done"), stations_left=d.get("stations_left"), loc_score=d.get("loc_score"), age_s=time.time() - st.get("ts", 0))

    def _ro(self, *a, **k):
        raise HTTPException(409, "mission control is not available in read-only (db) mode")

    start = stop = resume = set_fault = _ro


link = SimLink(DATA, SPEED) if MODE == "sim" else DbLink(DATA)
app = FastAPI(title="AgriRover GH-1 dashboard API", version="0.1.0")


class StartReq(BaseModel):
    name: str = "inspection"
    order: str = "serpentine"


class FaultReq(BaseModel):
    name: str
    value: bool


def _row(r: dict) -> dict:
    for k in ("flags", "metrics", "quality", "summary", "detail"):
        if isinstance(r.get(k), str):
            try:
                r[k] = json.loads(r[k])
            except ValueError:
                pass
    return r


@app.get("/api/status")
def status():
    # robot_time is the robot's clock: in simulation it runs faster than wall time (speed factor)
    return dict(mode=MODE, robot=link.live(), server_time=time.time(), robot_time=getattr(getattr(link, "rover", None), "now", time.time()))


@app.get("/api/map")
def facility_map():
    w = link.world
    return dict(room=dict(x=w.w, y=w.h), benches=[b.__dict__ for b in w.benches], dock=w.dock,
                stations=[dict(id=s.sid, x=s.x, y=s.y, aisle=s.aisle) for s in w.stations], plants=list(w.plants.values()),
                footprint=config.footprint_outline())


@app.post("/api/mission/start")
def mission_start(req: StartReq):
    if req.order not in ("serpentine", "naive", "optimal"):
        raise HTTPException(422, "order must be serpentine, naive or optimal")
    link.start(req.name, req.order)
    return dict(accepted=True)


@app.post("/api/mission/stop")
def mission_stop():
    link.stop()
    return dict(accepted=True)


@app.post("/api/robot/resume")
def robot_resume():
    link.resume()
    return dict(accepted=True)


@app.post("/api/sim/fault")
def sim_fault(req: FaultReq):
    if MODE != "sim":
        raise HTTPException(409, "fault injection exists only in simulation mode")
    try:
        link.set_fault(req.name, req.value)
    except KeyError:
        raise HTTPException(422, f"unknown fault {req.name}")
    return dict(accepted=True)


@app.get("/api/missions")
def missions(limit: int = 50):
    return [_row(m) for m in link.store.missions(limit)]


@app.get("/api/missions/{mid}")
def mission(mid: int):
    m = link.store.query("SELECT * FROM missions WHERE id=?", (mid,))
    if not m:
        raise HTTPException(404, "mission not found")
    obs = link.store.query("SELECT plant_id, station_id, camera, status, flags, confidence, area_cm2, image_path, ts FROM observations WHERE mission_id=? ORDER BY id", (mid,))
    counts: dict = {}
    for o in obs:
        counts[o["status"]] = counts.get(o["status"], 0) + 1
    return dict(mission=_row(m[0]), counts=counts, flagged=[_row(o) for o in obs if o["status"] in ("attention", "missing")],
                env=link.store.query("SELECT * FROM env_samples WHERE mission_id=? ORDER BY ts", (mid,)))


@app.get("/api/plants")
def plants():
    return [_row(p) for p in link.store.plant_latest()]


@app.get("/api/plants/{plant_id}/history")
def plant_history(plant_id: str):
    return [_row(p) for p in link.store.plant_history(plant_id)]


@app.get("/api/env")
def env(since: float = 0.0, limit: int = 2000):
    return link.store.env_history(since, limit)


@app.get("/api/events")
def events(limit: int = 100):
    return link.store.events(limit)


@app.get("/api/growth")
def growth():
    return link.store.growth_rates()


@app.get("/images/{name}")
def image(name: str):
    p = (link.store.image_dir / name).resolve()
    if link.store.image_dir.resolve() not in p.parents or not p.is_file():
        raise HTTPException(404, "image not found")
    return FileResponse(p)


@app.get("/api/health")
def health():
    return JSONResponse(dict(ok=True, mode=MODE))


app.mount("/", StaticFiles(directory=Path(__file__).parent / "static", html=True), name="static")
