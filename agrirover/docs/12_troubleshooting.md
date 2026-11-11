# 12 — Troubleshooting

Start with the dashboard event log and, on the robot, `ros2 topic echo /base/state --once`.

| Symptom | Likely cause | Check / fix |
|---|---|---|
| ARC-1 green LED off | battery disconnected, F1 or the in-line fuse open, pack BMS tripped, reversed polarity (protected: nothing happens) | voltage at J1; fuses; BMS resets when the charger is connected briefly |
| Green LED on, MCU LED dark | module not seated, wrong header orientation, firmware not flashed | USB end of the Black Pill must point to the "USB end" silkscreen; reflash by DFU (hold BOOT0, tap NRST, `make flash`) |
| State `ESTOP` and cannot be cleared | E-stop still pressed, J14 unplugged or a broken wire in either pole | continuity J14 pins 1–2 and 3–4 with the button released |
| Fault `RAIL` | motor rail commanded on but VMOT absent: E-stop pole 1 open while pole 2 reads closed, Q2/Q3 fault | red VMOT LED; voltage at J5 |
| Fault `BUMPER` with nothing touching | bumper switch stuck or loop open | continuity J15; spring return |
| Fault `OVERCURRENT` | wheel blocked, robot pushing an obstacle, gearbox damage | free the wheel; spin by hand; clear the fault |
| Fault `DRIVER` | TB9051FTG over-temperature / under-voltage, DIAG wiring | driver carrier supply; let it cool |
| Fault `LINK_LOST`, dashboard `base link lost` | UART cable, wrong port, Pi serial console still enabled on the UART, base node crashed | `tools/base_console.py`; disable the Linux serial console on `/dev/ttyAMA0` |
| Fault `LOW_BATTERY` | pack below 11.2 V for 5 s | charge; if it happens early, capacity-test the pack |
| One wheel runs backwards / robot spins on a straight command | motor leads or encoder A/B swapped | bring-up step 8 in docs/06 |
| Wheel speed oscillates or hunts | PI gains too high for the real motor, encoder noise | lower `kp`, then `ki` (`CMD_GAINS`); check encoder cable routing away from motor leads |
| Robot drifts off the aisle centre, low localisation score | map does not match the house (benches moved), lidar window dirty, strong sunlight, many people | clean the lidar; re-survey; check `/scan` in RViz |
| `LOCALIZATION_LOST`, mission aborted | robot was pushed or lifted, started away from the dock pose | put it back on the dock mark and restart |
| Stops in the aisle "blocked" with nothing visible | reflective film, hanging leaves or irrigation lines at lidar height (0.215 m); bench leg map offset | look at the scan; trim/raise the obstruction; widen nothing in software before understanding the cause |
| Station skipped | aisle blocked for more than 25 s | clear the aisle; rerun |
| `IMAGE_RETAKE` on most stations, plants `unreliable` | focus drifted, lens dirty, mast loose (ringing), exposure wrong for the light | check `edge_width_px` and `mean` in the stored quality record; refocus; increase `settle_time_s` |
| Many false colour flags after a lighting change | grey card dirty, shaded or out of its ROI | clean / re-aim the card; re-record the ROI |
| Flags on the wrong plant | stop error too large or pixel scale wrong | check stop repeatability (T-NAV-02); recalibrate the homography |
| Climate values missing (`ENV_SENSOR_FAULT`) | Qwiic cable, I²C address clash, sensor condensation | `i2cdetect -y 1` should show 0x62 (SCD41), 0x44 (SHT45), 0x39 (AS7341) |
| CO₂ reads high and flat | SCD41 needs ~5 s between samples and a few minutes after power-up; self-heating in a blocked shield | wait; check airflow through the louvres |
| Pi reboots when motors start | 5 V sag: USB-C pigtail too thin, regulator overloaded | measure 5 V at the Pi GPIO; shorten/thicken the lead |
| Dashboard `OFFLINE` | Wi-Fi gap (the mission continues), backend not running | reconnect; `systemctl status agrirover-dashboard` |
| `colcon build` fails on `sllidar_ros2` | driver not cloned into the workspace | see `ros2_ws/README.md` |
| Simulation: `python -m agrirover.sim.scenarios` reports a FAIL | a code or parameter change broke a behaviour | the failing criterion is printed with its measured value |
