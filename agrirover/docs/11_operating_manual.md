# 11 — Operating manual

For the operator of a built and commissioned rover. Sections marked *(sim)* can be practised today with the simulator
(`docs/08_dashboard_api.md`).

## 1. Before every session

1. Walk the route: aisles clear of hoses, crates and carts; floor free of standing water deeper than a film.
2. Robot: wheels and casters turn freely; mast and camera head tight; lens and grey card clean and dry; bumper moves
   and springs back; E-stop released (twist) — then press it once and confirm the motors are dead (section 4).
3. Battery above 30 % on the dashboard (≥ 13.0 V resting).
4. Tell people working in the house that the robot will run. Maximum speed in aisles is 0.3 m/s.

## 2. Power-up

1. Place the robot on the dock mark, nose along the aisle (the localisation starts from this pose).
2. Connect the battery Powerpole. Green LED on ARC-1; the MCU LED blinks; the Pi boots in about 40 s.
3. Open the dashboard (`http://<robot>:8080`). State should be `IDLE`, localisation score above 0.8.

## 3. Running an inspection *(sim)*

1. Choose the route order (serpentine is the default; "energy-optimal" searches aisle orders and directions).
2. **Start inspection.** The robot drives to each station, waits 1.5 s for the mast to settle, takes one image per
   camera, records the climate sample, and moves on. Progress shows as stations done / total.
3. It returns to the dock by itself when finished, when the battery reaches 25 %, or when **Stop & return** is pressed.
4. Review *Plants needing attention*. Click a plant for its image and metrics. Flags are prompts to go and look at the
   plant; they are not diagnoses and their thresholds have not been validated on real crops.

What the robot does on its own: slows and stops for anything in its path, waits, replans around obstacles it can pass,
skips a station it cannot reach within 25 s and reports it, retakes blurred or badly exposed images, continues if the
climate sensors fail (logged), aborts and stops if the lidar fails or localisation becomes unreliable.

## 4. Stopping

| Situation | Action | Result |
|---|---|---|
| Anything unsafe | Press the red E-stop on the rear of the deck | controlled stop in < 0.15 m, motor power removed after ≈ 0.4 s, state `ESTOP` |
| Planned stop | **Stop & return** on the dashboard | returns to the dock |
| Robot touched something | (automatic) bumper stop, state `ESTOP` "bumper contact" | waits for a person |

After an E-stop or bumper stop: remove the cause, twist the E-stop to release, press **Resume**. The robot will not
move until both have happened. *(sim: the "Sim E-stop" button toggles the simulated input.)*

Never stop the robot by standing in front of it as a routine, never ride on it, never reach under the deck while the
battery is connected.

## 5. Charging

Rev A has no automatic charging. Switch the robot off by disconnecting the Powerpole, connect the Bioenno 14.6 V
charger to the pack's charge lead, charge in a dry place off the greenhouse floor, attended, away from flammable
material. Typical charge from 25 %: about 5 h at 2 A. Do not use a lead-acid charger. Do not charge below 0 °C.

## 6. Shutdown and storage

Dashboard idle → `sudo poweroff` on the Pi (or the shutdown button if fitted) → wait for the Pi LED → disconnect the
battery. Store at 40–60 % charge if unused for more than a month. Wipe the robot dry; do not hose it.

## 7. Getting the data

Everything is on the robot in `~/agrirover_data/agrirover.db` and `~/agrirover_data/images/`. Copy both and browse
them offline with `AGRIROVER_MODE=db AGRIROVER_DATA=<folder> uvicorn dashboard.app:app`, or query the SQLite file
directly (schema in `software/agrirover/storage/db.py`).

## 8. Routine maintenance

| Interval | Task |
|---|---|
| Weekly | clean lenses and grey card; check wheel set screws; test E-stop and bumper; inspect cables at the mast base |
| Monthly | check motor shaft play (bushing wear, see docs/05 §5); tighten deck and mast screws; inspect connectors for corrosion; back up the database |
| Each crop cycle | re-check camera focus/exposure settings and the pixel-scale calibration; compare the map with the house if benches moved |
| Yearly / 500 h | replace casters if worn; capacity test of the battery (full discharge at ~2 A should give ≥ 9.6 Ah) |

## 9. Setting up a new greenhouse

1. Measure the house and edit `config/greenhouse.yaml` (room, bench positions and size, aisle width, dock, station
   pitch, plant spacing). Run `python3 tools/gen_ros_assets.py` to regenerate the map, keep-out mask, station list and
   Gazebo world, and `python -m agrirover.sim.run` to check the mission in simulation.
2. On site, drive the robot manually with `slam.launch.py`, save the map and compare it with the generated one
   (bench legs and walls should coincide within 5 cm); correct the YAML if not.
3. Calibrate the cameras (`docs/07_computer_vision.md` §5 step 1).
4. Run the acceptance tests in `docs/10_validation_test_report.md` §4 before unattended use.
