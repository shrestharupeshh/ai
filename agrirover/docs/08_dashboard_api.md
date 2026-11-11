# 08 — Dashboard and robot interface

Start (simulation demo, no hardware):

```bash
cd agrirover
pip install -e "software[dashboard]"
AGRIROVER_MODE=sim uvicorn dashboard.app:app --host 0.0.0.0 --port 8080
```

`AGRIROVER_MODE=db` serves an existing database read-only (`AGRIROVER_DATA` = directory holding `agrirover.db` and
`images/`). `AGRIROVER_SIM_SPEED` sets the simulation speed factor (default 8).

The page shows the facility map with the robot footprint, stations and one dot per plant coloured by its latest
status; battery, pose, mission progress and localisation score; start / stop-and-return / resume controls; mean
climate values and the temperature trace of the latest mission; a table of flagged plants with confidences; the
selected plant's history and image; mission list; event log. There is no authentication: run it on a trusted network.

## HTTP API (JSON)

| Method and path | Purpose | Notes |
|---|---|---|
| `GET /api/status` | live robot state | `robot.state` ∈ IDLE, NAVIGATING, SETTLING, CAPTURING, RETURNING, DONE, ABORTED, ESTOP, FAULT; `soc` 0–1; pose in the map frame; `robot_time` |
| `GET /api/map` | room, benches, dock, stations, plant positions, footprint outline | static per site |
| `POST /api/mission/start` | body `{"name": str, "order": "serpentine" \| "optimal" \| "naive"}` | 422 on an unknown order; ignored (event `COMMAND_REJECTED`) if a mission is running |
| `POST /api/mission/stop` | abandon remaining stations and return to the dock | |
| `POST /api/robot/resume` | operator acknowledgement after an E-stop or bumper stop | refused while the E-stop is still pressed |
| `GET /api/missions?limit=` | mission list with distance, energy, status, summary | newest first |
| `GET /api/missions/{id}` | one mission: status counts, flagged observations, climate samples | 404 if unknown |
| `GET /api/plants` | latest observation of every plant | `flags` = `{name: confidence}`; `pipeline_level` |
| `GET /api/plants/{plant_id}/history` | all observations of a plant | includes image file name, metrics, quality record |
| `GET /api/env?since=&limit=` | climate samples | each with map position and station |
| `GET /api/growth` | relative growth rate of canopy area per plant | needs ≥ 2 missions more than 1 h apart |
| `GET /api/events?limit=` | event log | `level` ∈ info, warn, error |
| `GET /images/{file}` | stored JPEG | path traversal rejected |
| `POST /api/sim/fault` | `{"name": "estop" \| "lidar_dead" \| "link_lost" \| …, "value": bool}` | simulation mode only (409 otherwise) |
| `GET /api/health` | liveness | |

## Robot side of the interface

The backend talks to the robot through a small `RobotLink` object with `start(name, order)`, `stop()`, `resume()` and
`live()`. Two implementations exist: `SimLink` (headless simulator, tested by `software/tests/test_dashboard.py`) and
`DbLink` (read-only). On the physical robot the same four calls map to the mission node's `~/start` and `~/stop`
services, the base node's `~/clear_fault`, and the `robot_status` table that the mission node writes; that ROS-backed
link is the remaining integration step and is **not implemented yet** — today the dashboard on a real robot runs in
`db` mode and missions are started with `ros2 service call /agrirover_mission/start std_srvs/srv/Trigger`.

## Data model

SQLite file with five tables — `missions`, `observations`, `env_samples`, `robot_status`, `events`
(`software/agrirover/storage/db.py`). Images are JPEG files named `m<mission>_<station>_<camera>.jpg`.
