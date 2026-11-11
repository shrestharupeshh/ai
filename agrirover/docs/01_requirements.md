# 01 — System requirements

AgriRover GH-1, revision A. Every numeric target below is also stored in `config/rover.yaml`; the automated test
`software/tests/test_core.py::test_design_is_consistent_with_requirements` fails if the calculations, CAD summary or
software footprint drift away from them. Requirements are changed only by editing this file and `rover.yaml` together
and recording the change in section 7.

Status column: **A** = met by analysis/calculation, **S** = exercised in simulation, **I** = met by inspection of the
design files, **—** = not yet verified. Nothing is marked as physically tested, because no hardware has been built.

## 1. Operating environment (assumptions)

| ID | Assumption | Value | Basis |
|---|---|---|---|
| R-ENV-01 | Facility | Indoor greenhouse or vertical farm, benches/racks in parallel rows | project brief |
| R-ENV-02 | Floor | Flat sealed concrete, may be wet; thresholds / hoses ≤ 15 mm | assumed; confirm in site survey |
| R-ENV-03 | Minimum aisle for straight transit | 0.60 m between benches | assumed lower bound for walk aisles |
| R-ENV-04 | Minimum aisle for turning in place, and any aisle bounded by a wall | 0.70 m | turning envelope Ø 0.56 m + margin |
| R-ENV-05 | Floor slope | ≤ 5 % (drain falls, door ramps) | assumed |
| R-ENV-06 | Air | 10–35 °C, 30–95 % RH non-condensing on electronics; occasional splashes from above | assumed; IPX4 design intent for the electronics bay, **not tested** |
| R-ENV-07 | Light | Sunlight through glazing plus LED grow lights (including strongly coloured spectra) | drives the grey-card colour reference and a DToF lidar |
| R-ENV-08 | Connectivity | Facility Wi-Fi (2.4/5 GHz) with gaps; robot must finish a mission without it | project brief |
| R-ENV-09 | People | Workers share the aisles; no physical separation | project brief |
| R-ENV-10 | Bench visibility | Bench tops are above the lidar plane: only legs are visible | geometry; see R-SAF-07 |

## 2. Mechanical and mobility

| ID | Requirement | Target | Design value | Status |
|---|---|---|---|---|
| R-MEC-01 | Length incl. bumper | ≤ 0.50 m | 0.482 m | A, I |
| R-MEC-02 | Width over wheels | ≤ 0.40 m | 0.354 m | A, I |
| R-MEC-03 | Height incl. mast | ≤ 1.45 m (mast adjustable 0.8–1.4 m) | 1.253 m | I |
| R-MEC-04 | Mass incl. payload | ≤ 15 kg | 11.4 kg (budget) | A |
| R-MEC-05 | Payload on the upper deck | 2 kg | included in all calculations | A |
| R-MEC-06 | Ground clearance | ≥ 30 mm | 31.5 mm (motor can) | A |
| R-MOB-01 | Maximum speed | 0.5 m/s | motor gives 0.59 m/s at peak load, 11 V | A |
| R-MOB-02 | Aisle transit speed | 0.3 m/s | controller set-point | S |
| R-MOB-03 | Acceleration | 0.5 m/s² | torque margin 2.3× on the gearbox limit | A |
| R-MOB-04 | Minimum path turning radius | 0 (turn in place) | differential drive, axle centred | A, S |
| R-MOB-05 | Turning envelope | Ø ≤ 0.60 m | Ø 0.559 m (bumper corners) | A |
| R-MOB-06 | Static stability | tip angle ≥ 20° any direction, at maximum mast height and payload | 26.6° lateral, 37° rear | A |

## 3. Power

| ID | Requirement | Target | Design value | Status |
|---|---|---|---|---|
| R-PWR-01 | Operating time per charge | ≥ 4 h mission profile | 7.8 h estimated | A |
| R-PWR-02 | Energy reserve never used in normal operation | 20 % | return at 25 % SoC, stop at 10 % | A, S |
| R-PWR-03 | Battery protection | cell-level BMS, fuse within 150 mm of the pack, reverse-polarity protection | Bioenno pack BMS, F1 10 A, Q1 | I |
| R-PWR-04 | Charging | manufacturer-approved CC/CV charger, robot powered down, attended (rev A) | Bioenno 14.6 V 2 A | I |
| R-PWR-05 | Monitoring | bus voltage ±1 %, current ±3 %, logged at ≥ 1 Hz | INA219 + 5 mΩ 1 % shunt | I |

## 4. Sensing, navigation and data

| ID | Requirement | Target | Status |
|---|---|---|---|
| R-NAV-01 | Localise in a mapped house | position error ≤ 0.05 m RMS in aisles | S (0.007 m RMS in the simulator; optimistic) |
| R-NAV-02 | Stop repeatability at inspection stations | ≤ 0.05 m, ≤ 3° | S (≤ 0.048 m, ≤ 2.3°) |
| R-NAV-03 | Complete an inspection mission without manual control | all reachable stations, return to dock | S |
| R-NAV-04 | Unmapped obstacles | detect, stop or replan; never push | S |
| R-SEN-01 | Air temperature | ±0.3 °C (sensor ±0.1 °C typ.) | I (SHT45 datasheet figure, not verified here) |
| R-SEN-02 | Relative humidity | ±3 %RH | I |
| R-SEN-03 | CO₂ | ±(50 ppm + 5 %) | I (SCD41) |
| R-SEN-04 | Light | relative spectral irradiance; PPFD only after calibration against a quantum sensor | — |
| R-SEN-05 | Plant images | ≥ 1 image per plant per mission, ≥ 0.8 px/mm on the canopy, sharpness and exposure gated | S (synthetic images) |
| R-DAT-01 | Every observation stored with plant ID, map position, robot pose, timestamp, image, metrics, flags with confidence, environment sample | SQLite schema | S |
| R-DAT-02 | Operate through Wi-Fi outages; data stays on the robot | local database + images | I |
| R-COM-01 | Operator interface | web dashboard on the facility LAN; start/stop, status, results | S (against the simulator) |

## 5. Safety

| ID | Requirement | Target | Status |
|---|---|---|---|
| R-SAF-01 | Emergency stop | red/yellow latching button, 2 NC poles; removes motor power in hardware within 0.5 s independent of software | I (circuit), A (delay 0.39 s) |
| R-SAF-02 | Protective stop distance from 0.5 m/s | ≤ 0.15 m (controlled stop at 1.5 m/s²) | A (0.14 m incl. lidar latency); S at 0.3 m/s |
| R-SAF-03 | Contact bumper | full-width front bumper, stops the robot, latched until acknowledged | I, S |
| R-SAF-04 | Loss of command / link | wheels stop within 0.3 s without a valid command; rail dropped after 2 s | unit-tested in firmware, S |
| R-SAF-05 | Speed with people present | ≤ 0.3 m/s in aisles; contact force not assessed | — (needs measurement; see docs/13) |
| R-SAF-06 | Sensor health | stop if lidar data is stale > 0.5 s or localisation confidence is low | S |
| R-SAF-07 | Benches not visible to the lidar | never enter bench footprints: keep-out map + bumper + map-checked reversing/turning | S; residual risk documented |
| R-SAF-08 | Stored energy | LiFePO₄ chemistry, fused, no charging while unattended in rev A | I |

## 6. Operations

| ID | Requirement | Target |
|---|---|---|
| R-OPS-01 | Duty | up to 2 inspection missions per day, 5 days per week; 500 operating hours per year |
| R-OPS-02 | Service access | battery swap and fuse replacement without tools other than a screwdriver, < 5 min |
| R-OPS-03 | Wash-down | wipe-clean only; no pressure washing |

## 7. Change log

| Date | Change | Reason |
|---|---|---|
| 2026-11-10 | Initial targets | project start |
| 2026-11-10 | Track width 340 → 330 mm, mast 1050 → 1070 mm | motor/bracket layout in CAD |
| 2026-11-11 | Turning envelope corrected from Ø 0.51 m to Ø 0.559 m (R-MOB-05 added) | simulation scenario exposed that bumper corners, not deck corners, define the envelope |
| 2026-11-11 | R-ENV-03 scoped to aisles between benches; wall-side aisles moved to R-ENV-04 | turning beside a wall leaves 20 mm margin at 0.60 m — not a defensible requirement |
