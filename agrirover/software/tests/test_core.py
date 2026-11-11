"""Unit tests: kinematics, protocol, planner, controller geometry, storage, design consistency."""
import json
import math
import subprocess
from pathlib import Path

import numpy as np
import pytest

from agrirover import config
from agrirover.hw import protocol as P
from agrirover.kinematics import Odometry, limit_twist, twist_to_wheels, wheels_to_twist
from agrirover.mission import route
from agrirover.nav.planner import Costmap, astar, path_length
from agrirover.sim.world import World
from agrirover.storage.db import Store

ROOT = config.project_root()


def test_kinematics_round_trip():
    r, track = config.wheel_radius(), config.track_width()
    for v, w in [(0.3, 0.0), (0.0, 1.0), (0.25, -0.6), (-0.1, 0.3)]:
        assert wheels_to_twist(*twist_to_wheels(v, w, r, track), r, track) == pytest.approx((v, w))
    wl, wr = twist_to_wheels(0.5, 0.0, r, track)
    assert wl == wr == pytest.approx(10.0)          # 0.5 m/s on 100 mm wheels
    v, w = limit_twist(2.0, 0.0, 0.5, 1.2, 12.0, r, track)
    assert v == pytest.approx(0.5)


def test_odometry_square():
    r, track, cpr = 0.05, 0.33, 4480
    odo = Odometry(r, track, cpr)
    enc = [0.0, 0.0]
    odo.update(0, 0)
    def drive(dl, dr, n=200):
        for _ in range(n):
            enc[0] += dl / n
            enc[1] += dr / n
            odo.update(int(round(enc[0])), int(round(enc[1])))
    side = 1.0 / (2 * math.pi * r) * cpr
    turn = (math.pi / 2 * track / 2) / (2 * math.pi * r) * cpr
    for _ in range(4):
        drive(side, side)
        drive(-turn, turn)
    assert abs(odo.pose.x) < 0.01 and abs(odo.pose.y) < 0.01
    assert odo.distance == pytest.approx(4.0, abs=0.02)


def test_protocol_round_trip_and_errors():
    assert P.crc16(b"123456789") == 0x29B1
    for payload in (b"", b"\x00", b"\x00\x00\x01", bytes(range(48)), b"\x11" * 40):
        assert P.cobs_decode(P.cobs_encode(payload)) == payload
        assert 0 not in P.cobs_encode(payload)
    ps = P.FrameParser()
    frame = P.cmd_wheels(5, 3.5, -1.0)
    out = list(ps.feed(b"\x07garbage\x00" + frame + frame[:5]))
    assert len(out) == 1 and out[0][0] == P.MSG_CMD_WHEELS and ps.rx_err == 1
    bad = bytearray(frame)
    bad[3] ^= 0x40
    assert list(ps.feed(bytes(bad))) == []


def test_protocol_matches_firmware(tmp_path):
    """The C implementation must produce byte-identical frames to the Python mirror."""
    fw = ROOT / "firmware"
    exe = tmp_path / "t"
    subprocess.run(["gcc", "-std=c11", "-O1", "-DAR_HOST_BUILD", "-D_GNU_SOURCE", str(fw / "core/protocol.c"), str(fw / "core/control.c"),
                    str(fw / "app/main.c"), str(fw / "tests/test_core.c"), "-lm", "-o", str(exe)], check=True)
    lines = subprocess.run([str(exe), "--emit"], check=True, capture_output=True, text=True).stdout.split()
    ps = P.FrameParser()
    (mtype, seq, payload), = list(ps.feed(bytes.fromhex(lines[0])))
    st = P.decode_state(payload)
    assert (mtype, seq) == (P.MSG_STATE, 42)
    assert (st.t_ms, st.enc_left, st.enc_right, st.vbat_mv, st.ibat_ma, st.i_right_ma) == (1000, 4480, -4480, 13100, 1500, -340)
    assert st.state_name == "ACTIVE" and st.fault_names == ["BUMPER"] and st.gyro_z == pytest.approx(0.5)
    assert bytes.fromhex(lines[1]) == P.cmd_wheels(1, 3.5, -1.0)


def test_world_layout():
    w = World()
    assert len(w.benches) == 3 and len(w.aisles_y) == 4 and len(w.stations) == 32
    assert all(abs(a - 0.8) < 1e-9 for a in w.aisle_widths())
    assert len(w.plants) == 384
    assert all(any(b.x0 <= p["x"] <= b.x1 and b.y0 <= p["y"] <= b.y1 for b in w.benches) for p in w.plants.values())
    assert len({s.sid for s in w.stations}) == 32


def test_planner_finds_clear_path_and_rejects_blocked():
    w = World()
    cm = Costmap(w.nav_grid(0.05, include_obstacles=False), 0.05, config.inscribed_radius(), circumscribed=config.footprint_radius())
    a, b = w.dock[:2], (w.stations[-1].x, w.stations[-1].y)
    path = astar(cm, a, b)
    assert path is not None and path_length(path) > 5
    assert all(cm.is_free(x, y) for x, y in path)
    assert astar(cm, a, (5.0, 1.4)) is None                       # goal inside a bench
    blocked = w.nav_grid(0.05, include_obstacles=False)
    blocked[:, 30:34] = True                                      # wall across the head corridor
    assert astar(Costmap(blocked, 0.05, config.inscribed_radius()), a, b) is None


def test_route_optimiser_not_worse_than_serpentine():
    w = World()
    serp = route.route_metrics(route.boustrophedon(w), w)
    naive = route.route_metrics(route.naive_order(w), w)
    best = route.best_aisle_order(w)[1]
    assert best["energy_wh"] <= serp["energy_wh"] + 1e-9 <= naive["energy_wh"] + 1e-9
    assert naive["distance_m"] > serp["distance_m"]


def test_store_round_trip(tmp_path):
    st = Store(tmp_path / "a.db")
    mid = st.start_mission("t", 2, 1000.0)
    eid = st.add_env(mid, 1001.0, 1.0, 2.0, "A0-S01", dict(temperature_c=21.5, humidity_pct=70, co2_ppm=600, light=250, light_unit="x", vpd_kpa=0.8))
    plant = dict(id="B1S-R1-P001", x=2.1, y=0.95)
    for ts, area in ((1002.0, 100.0), (1002.0 + 2 * 86400, 150.0)):
        st.add_observation(mid, ts, plant, "A0-S01", "left", (2.5, 0.4, 0.0), "img.jpg",
                           dict(status="attention", flags={"chlorosis": 0.9}, area_cm2=area, diameter_cm=11.3, yellow_frac=0.5), dict(ok=True), eid, "DEMONSTRATION")
    st.update_mission(mid, status="done", summary=dict(ok=True))
    latest = st.plant_latest()
    assert len(latest) == 1 and latest[0]["area_cm2"] == 150.0 and json.loads(latest[0]["flags"]) == {"chlorosis": 0.9}
    assert st.growth_rates()[0]["rgr_per_day"] == pytest.approx(math.log(1.5) / 2, abs=1e-3)
    assert st.missions()[0]["status"] == "done" and len(st.env_history()) == 1


def test_design_is_consistent_with_requirements():
    """Guards against silent drift between rover.yaml, the calculations and the software footprint."""
    cfg = config.rover()
    rq = cfg["requirements"]
    mech = json.loads((ROOT / "mechanical/calcs/mechanical_results.json").read_text())
    elec = json.loads((ROOT / "electrical/calcs/electrical_results.json").read_text())
    assert mech["overall_width_mm"] / 1000 <= rq["max_width_m"]
    assert mech["mass_design_kg"] <= rq["max_mass_kg"]
    assert mech["ground_clearance_mm"] / 1000 >= rq["ground_clearance_m"]
    assert mech["torque_margin_cont"] >= 2.0 and mech["v_at_peak_11V"] >= rq["v_max"]
    assert 2 * mech["swept_radius_mm"] / 1000 < rq["aisle_min_uturn_m"]
    assert mech["stop_dist_lidar_m"] <= rq["stop_distance_m"]
    assert elec["runtime_h"] >= rq["runtime_h"]
    assert config.footprint_radius() * 1000 == pytest.approx(mech["swept_radius_mm"], abs=0.5)
    cad = json.loads((ROOT / "mechanical/export/cad_summary.json").read_text())
    assert cad["interferences"] == []
    assert cad["envelope_mm"][1] / 1000 <= rq["max_width_m"] and cad["envelope_mm"][2] / 1000 <= rq["max_height_m"]


def test_camera_geometry_covers_station():
    """Each camera must see the near half of the bench over one station pitch (config/greenhouse.yaml assumptions)."""
    gh, c = config.greenhouse(), config.rover()["chassis"]
    cam_z, tilt = c["camera_z_mm"] / 1000, math.radians(c["camera_tilt_deg"])
    dz = cam_z - (gh["bench"]["top_z"] + 0.05)
    near, far = gh["aisle_width"] / 2, gh["aisle_width"] / 2 + gh["bench"]["width"] / 2
    ang_near, ang_far = math.atan2(dz, near), math.atan2(dz, far)
    vfov = math.radians(gh["camera"]["vfov_deg"])
    assert tilt - vfov / 2 <= ang_far and ang_near <= tilt + vfov / 2
    slant = math.hypot(dz, (near + far) / 2)
    assert 2 * slant * math.tan(math.radians(gh["camera"]["hfov_deg"]) / 2) >= gh["stations"]["pitch"]
