"""Dashboard API against the simulator (documented interface: docs/08_dashboard_api.md)."""
import importlib
import sys
import time

import pytest

pytest.importorskip("fastapi")
from fastapi.testclient import TestClient  # noqa: E402

from agrirover import config  # noqa: E402


@pytest.fixture(scope="module")
def client(tmp_path_factory):
    import os
    os.environ.update(AGRIROVER_MODE="sim", AGRIROVER_DATA=str(tmp_path_factory.mktemp("dash")), AGRIROVER_SIM_SPEED="300")
    sys.path.insert(0, str(config.project_root()))
    app_mod = importlib.import_module("dashboard.app")
    with TestClient(app_mod.app) as c:
        yield c
    app_mod.link._run = False


def wait_state(client, states, timeout=90):
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = client.get("/api/status").json()["robot"]["state"]
        if st in states:
            return st
        time.sleep(0.2)
    raise AssertionError(f"state {states} not reached, last {st}")


def test_map_and_idle_status(client):
    m = client.get("/api/map").json()
    assert len(m["stations"]) == 32 and len(m["plants"]) == 384 and len(m["benches"]) == 3
    s = client.get("/api/status").json()
    assert s["mode"] == "sim" and s["robot"]["state"] == "IDLE" and 0.9 < s["robot"]["soc"] <= 1.0
    assert client.get("/").status_code == 200 and "AgriRover" in client.get("/").text


def test_estop_then_full_mission(client):
    assert client.post("/api/mission/start", json={"order": "bogus"}).status_code == 422
    assert client.post("/api/mission/start", json={"name": "api test", "order": "optimal"}).json()["accepted"]
    wait_state(client, {"NAVIGATING"})
    client.post("/api/sim/fault", json={"name": "estop", "value": True})
    wait_state(client, {"ESTOP"})
    client.post("/api/robot/resume")                       # refused while pressed
    time.sleep(0.5)
    assert client.get("/api/status").json()["robot"]["state"] == "ESTOP"
    client.post("/api/sim/fault", json={"name": "estop", "value": False})
    client.post("/api/robot/resume")
    assert wait_state(client, {"DONE", "ABORTED"}) == "DONE"
    ms = client.get("/api/missions").json()
    assert ms[0]["status"] == "done" and ms[0]["stations_done"] == 32 and ms[0]["distance_m"] > 30
    detail = client.get(f"/api/missions/{ms[0]['id']}").json()
    assert len(detail["env"]) == 32 and sum(detail["counts"].values()) == 384
    plants = client.get("/api/plants").json()
    assert len(plants) == 384
    flagged = [p for p in plants if p["status"] == "attention"]
    assert flagged and all(isinstance(p["flags"], dict) and p["pipeline_level"] == "DEMONSTRATION" for p in flagged)
    hist = client.get(f"/api/plants/{flagged[0]['plant_id']}/history").json()
    assert hist and hist[0]["image_path"]
    assert client.get(f"/images/{hist[0]['image_path']}").headers["content-type"] == "image/jpeg"
    assert client.get("/images/..%2Fagrirover.db").status_code == 404
    codes = [e["code"] for e in client.get("/api/events").json()]
    assert "ESTOP" in codes and "MISSION_DONE" in codes and "RESUME" in codes
    assert client.get("/api/missions/9999").status_code == 404 and client.post("/api/sim/fault", json={"name": "nope", "value": True}).status_code == 422
