"""SQLite data store for missions, plant observations, environment samples, robot status and events.

One file (default data/runtime/agrirover.db) + an image directory. The dashboard reads the same file;
SQLite WAL mode allows one writer (the robot) and concurrent readers.
"""
from __future__ import annotations

import json
import sqlite3
import threading
import time
from pathlib import Path

SCHEMA = """
CREATE TABLE IF NOT EXISTS missions (
  id INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT, started REAL, finished REAL, status TEXT,
  stations_planned INTEGER, stations_done INTEGER DEFAULT 0, distance_m REAL DEFAULT 0, energy_wh REAL DEFAULT 0, summary TEXT);
CREATE TABLE IF NOT EXISTS observations (
  id INTEGER PRIMARY KEY AUTOINCREMENT, mission_id INTEGER, ts REAL, plant_id TEXT, station_id TEXT, camera TEXT,
  x REAL, y REAL, robot_x REAL, robot_y REAL, robot_yaw REAL, image_path TEXT, status TEXT, flags TEXT, confidence REAL,
  area_cm2 REAL, diameter_cm REAL, metrics TEXT, quality TEXT, env_id INTEGER, pipeline_level TEXT);
CREATE INDEX IF NOT EXISTS idx_obs_plant ON observations (plant_id, ts);
CREATE INDEX IF NOT EXISTS idx_obs_mission ON observations (mission_id);
CREATE TABLE IF NOT EXISTS env_samples (
  id INTEGER PRIMARY KEY AUTOINCREMENT, mission_id INTEGER, ts REAL, x REAL, y REAL, station_id TEXT,
  temperature_c REAL, humidity_pct REAL, co2_ppm REAL, light REAL, light_unit TEXT, vpd_kpa REAL, source TEXT);
CREATE TABLE IF NOT EXISTS robot_status (
  id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL, x REAL, y REAL, yaw REAL, v REAL, battery_v REAL, soc REAL, state TEXT,
  mission_id INTEGER, detail TEXT);
CREATE TABLE IF NOT EXISTS events (
  id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL, level TEXT, code TEXT, message TEXT, mission_id INTEGER);
"""


class Store:
    def __init__(self, path: str | Path = "data/runtime/agrirover.db", image_dir: str | Path | None = None):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.image_dir = Path(image_dir) if image_dir else self.path.parent / "images"
        self.image_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self.db = sqlite3.connect(str(self.path), check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.executescript(SCHEMA)
        self.db.commit()

    def _exec(self, sql: str, args=()) -> int:
        with self._lock:
            cur = self.db.execute(sql, args)
            self.db.commit()
            return cur.lastrowid

    def query(self, sql: str, args=()) -> list[dict]:
        with self._lock:
            return [dict(r) for r in self.db.execute(sql, args).fetchall()]

    # ------------------------------------------------------------------ writers
    def start_mission(self, name: str, stations: int, ts: float | None = None) -> int:
        return self._exec("INSERT INTO missions (name, started, status, stations_planned) VALUES (?,?,?,?)",
                          (name, ts or time.time(), "running", stations))

    def update_mission(self, mid: int, **kw):
        if "summary" in kw and not isinstance(kw["summary"], str):
            kw["summary"] = json.dumps(kw["summary"])
        keys = ", ".join(f"{k}=?" for k in kw)
        self._exec(f"UPDATE missions SET {keys} WHERE id=?", (*kw.values(), mid))

    def add_env(self, mid, ts, x, y, station_id, sample: dict) -> int:
        return self._exec("INSERT INTO env_samples (mission_id, ts, x, y, station_id, temperature_c, humidity_pct, co2_ppm, light, light_unit, vpd_kpa, source)"
                          " VALUES (?,?,?,?,?,?,?,?,?,?,?,?)",
                          (mid, ts, x, y, station_id, sample.get("temperature_c"), sample.get("humidity_pct"), sample.get("co2_ppm"),
                           sample.get("light"), sample.get("light_unit"), sample.get("vpd_kpa"), sample.get("source", "")))

    def add_observation(self, mid, ts, plant: dict, station_id, camera, robot_pose, image_path, obs: dict, quality: dict, env_id, level) -> int:
        flags = obs.get("flags", {})
        metrics = {k: obs[k] for k in ("green_frac", "yellow_frac", "brown_frac", "purple_frac", "mean_hue", "exg", "cx", "cy") if k in obs}
        return self._exec("INSERT INTO observations (mission_id, ts, plant_id, station_id, camera, x, y, robot_x, robot_y, robot_yaw, image_path, status,"
                          " flags, confidence, area_cm2, diameter_cm, metrics, quality, env_id, pipeline_level) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
                          (mid, ts, plant["id"], station_id, camera, plant["x"], plant["y"], *robot_pose, image_path, obs.get("status"),
                           json.dumps(flags), max(flags.values()) if flags else None, obs.get("area_cm2"), obs.get("diameter_cm"),
                           json.dumps(metrics), json.dumps(quality), env_id, level))

    def add_status(self, ts, pose, v, battery_v, soc, state, mission_id=None, detail: dict | None = None):
        self._exec("INSERT INTO robot_status (ts, x, y, yaw, v, battery_v, soc, state, mission_id, detail) VALUES (?,?,?,?,?,?,?,?,?,?)",
                   (ts, *pose, v, battery_v, soc, state, mission_id, json.dumps(detail or {})))

    def add_event(self, level: str, code: str, message: str, mission_id=None, ts: float | None = None):
        self._exec("INSERT INTO events (ts, level, code, message, mission_id) VALUES (?,?,?,?,?)", (ts or time.time(), level, code, message, mission_id))

    # ------------------------------------------------------------------ readers used by the dashboard
    def latest_status(self) -> dict | None:
        r = self.query("SELECT * FROM robot_status ORDER BY id DESC LIMIT 1")
        return r[0] if r else None

    def missions(self, limit: int = 50) -> list[dict]:
        return self.query("SELECT * FROM missions ORDER BY id DESC LIMIT ?", (limit,))

    def plant_latest(self) -> list[dict]:
        """Most recent observation of every plant."""
        return self.query("SELECT o.* FROM observations o JOIN (SELECT plant_id, MAX(id) AS mid FROM observations GROUP BY plant_id) m ON o.id = m.mid ORDER BY o.plant_id")

    def plant_history(self, plant_id: str) -> list[dict]:
        return self.query("SELECT * FROM observations WHERE plant_id=? ORDER BY ts", (plant_id,))

    def env_history(self, since: float = 0.0, limit: int = 5000) -> list[dict]:
        return self.query("SELECT * FROM env_samples WHERE ts>=? ORDER BY ts DESC LIMIT ?", (since, limit))[::-1]

    def events(self, limit: int = 100) -> list[dict]:
        return self.query("SELECT * FROM events ORDER BY id DESC LIMIT ?", (limit,))

    def growth_rates(self) -> list[dict]:
        """Relative growth rate of projected canopy area per plant between its first and last valid observation (1/day)."""
        import math
        rows = self.query("SELECT plant_id, ts, area_cm2 FROM observations WHERE status IN ('ok','attention') AND area_cm2 > 0 ORDER BY plant_id, ts")
        out, cur = [], {}
        for r in rows:
            cur.setdefault(r["plant_id"], []).append(r)
        for pid, rs in cur.items():
            if len(rs) >= 2 and rs[-1]["ts"] - rs[0]["ts"] > 3600:
                days = (rs[-1]["ts"] - rs[0]["ts"]) / 86400
                out.append(dict(plant_id=pid, n=len(rs), days=round(days, 2),
                                rgr_per_day=round(math.log(rs[-1]["area_cm2"] / rs[0]["area_cm2"]) / days, 4)))
        return out
