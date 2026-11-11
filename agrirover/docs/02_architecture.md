# 02 — System architecture

## 1. Block diagram

```
                         Wi-Fi (facility LAN)                    operator browser
                               │                                       │
┌──────────────────────────────┴───────────────────────────────────────┴──────────┐
│ HIGH-LEVEL COMPUTER  Raspberry Pi 5 (Ubuntu 24.04, ROS 2 Jazzy)                 │
│  Nav2 + AMCL + keep-out filter + collision monitor     mission node             │
│  robot_localization EKF                                vision pipeline (OpenCV) │
│  sllidar_ros2 ─ USB ─ RPLIDAR C1                       SQLite + images          │
│  v4l2_camera ×2 ─ USB ─ plant cameras L/R              dashboard (FastAPI)      │
│  env_sensor_node ─ I²C (Qwiic) ─ SCD41 · SHT45 · AS7341 in the mast shield      │
│  base_node ─ UART 115200, COBS + CRC16 framed ──────────────┐                   │
└─────────────────────────────────────────────────────────────┼───────────────────┘
        5 V 5 A buck (Pololu D36V50F5) ◄── VSBC (F2 5 A)      │
┌─────────────────────────────────────────────────────────────┼───────────────────┐
│ ARC-1 CARRIER BOARD (custom, 2-layer)                       │                   │
│  battery in → F1 10 A → Q1 reverse-polarity → 5 mΩ shunt/INA219 → VBAT          │
│  VBAT → Q2 high-side switch ─► VMOT   (gate: MCU enable AND E-stop pole 1)      │
│  VBAT → TSR 1-2450 → 5 V → AMS1117 → 3.3 V                                      │
│  LOW-LEVEL CONTROLLER  STM32F411 "Black Pill" (socketed), 100 Hz loop           │
│   TIM2/TIM3 quadrature ◄─ encoders       TIM4 PWM ─► 2 × TB9051FTG ─► motors    │
│   ADC ◄─ motor current monitors, VMOT    I²C3 ◄─ INA219, LSM6DSOX gyro          │
│   GPIO ◄─ E-stop pole 2, bumper loop, driver fault      IWDG watchdog           │
└─────────────────────────────────────────────────────────────────────────────────┘
   12.8 V 12 Ah LiFePO₄ pack with BMS (Bioenno BLF-1212A)
```

## 2. Division of responsibility

| Layer | Runs | Responsible for | Must keep working when the other layer fails |
|---|---|---|---|
| Low-level controller (C, bare metal) | 100 Hz, hard deadlines | wheel-speed PI + feed-forward, acceleration limits, encoder and current acquisition, battery monitor, safety state machine (E-stop, bumper, stall, link loss, low battery), telemetry | yes: stops the robot by itself |
| High-level computer (Python/C++ under ROS 2) | 1–20 Hz, best effort | localisation, planning, path following, protective field, mission sequencing, vision, storage, dashboard | no: if it hangs, the firmware's command time-out stops the wheels |
| Hardware safety chain | continuous | E-stop pole 1 removes the motor-rail gate drive; RC network gives a ~0.4 s controlled-stop window, then the rail is off regardless of software | independent of both computers |

## 3. Software architecture

```
agrirover/                       repository root
├── config/                      rover.yaml (design parameters) · greenhouse.yaml (site model)
├── software/agrirover/          ROS-independent Python core — fully unit- and scenario-tested here
│   ├── kinematics.py            differential-drive kinematics, odometry
│   ├── hw/                      protocol.py (mirror of the firmware protocol) · base_driver.py · env_sensors.py
│   ├── nav/                     planner.py (costmap + A*) · controller.py · localization.py · mapping.py
│   ├── vision/                  quality.py · pipeline.py (classical, DEMONSTRATION) · onnx_detector.py (EXPERIMENTAL)
│   ├── mission/route.py         station ordering, route energy/time model
│   ├── storage/db.py            SQLite store (missions, observations, env samples, status, events)
│   ├── sim/                     world.py · sim2d.py · camera_sim.py · scenarios.py · run.py
│   └── runtime.py               wires everything for headless runs (reference "lite" navigator)
├── firmware/                    core/ (portable C) · hal/stm32f411 · tests/ (host)
├── ros2_ws/src/                 agrirover_description · agrirover_bringup · agrirover_base · agrirover_apps
├── dashboard/                   FastAPI backend + static UI
├── mechanical/ · electrical/    calculations, CAD, PCB generator and outputs
└── manufacturing/ · research/ · docs/
```

Two navigation paths exist on purpose:

* **Reference navigator** (`software/agrirover/nav`, `runtime.py`): costmap, A*, rotate-then-pursue controller,
  scan-match localisation. Pure Python, runs headless in seconds, is what the 14 scenarios and the research experiments
  execute. It is deliberately simple and is *not* intended to replace Nav2 on the robot.
* **Production navigator** (`ros2_ws`): Nav2 (Smac 2D planner, Regulated Pure Pursuit behind a rotation shim, AMCL,
  keep-out filter, collision monitor) configured with the same footprint, speeds and protective-field geometry. It has
  been statically checked but **not executed** (no ROS 2 runtime was available where this package was produced).

Everything above the motion layer is shared: vision, storage, route ordering, the serial protocol and the dashboard
are the same modules in both paths.

## 4. Interfaces

| Interface | Medium | Definition |
|---|---|---|
| SBC ↔ MCU | 3.3 V UART, 115200 8N1 | `firmware/core/protocol.h`; frames `COBS(type, seq, payload, CRC16)` + `0x00`; `CMD_WHEELS` (rad/s) at 20 Hz down, `STATE` (37 bytes) at 50 Hz up. Python mirror cross-checked byte-for-byte in CI |
| MCU ↔ motor drivers | 12-pin header J4 | 4 × PWM (20 kHz), EN, DIAG (open drain), 2 × current monitor, 3.3 V/5 V/GND |
| Environment sensors | I²C 100 kHz (Qwiic) on the SBC | vendor CircuitPython drivers wrapped by `hw/env_sensors.py` |
| ROS topics | DDS | `/cmd_vel`, `/wheel/odometry`, `/imu/data_raw`, `/scan`, `/battery_state`, `/base/state`, `/camera_{left,right}/image_raw`, `/env/*`, TF `map→odom→base_footprint→…` |
| Data store | SQLite (WAL) + JPEG files | `software/agrirover/storage/db.py` |
| Dashboard | HTTP/JSON | `docs/08_dashboard_api.md` |

## 5. Plant identity and location

Plants are not recognised individually by appearance. Identity comes from geometry: the facility model
(`config/greenhouse.yaml`) defines bench positions, plant spacing and the inspection stations; each station view
contains a known, ordered set of plant slots with map coordinates. A plant ID such as `B2S-R1-P014` means bench 2,
south side, row 1, position 14. The vision pipeline assigns canopy pixels to the nearest slot, so an observation is
tied to a physical location as long as the robot stops within tolerance (R-NAV-02) — the sensitivity of this
assumption to stop error is quantified in `research/results/exp3_positioning_sensitivity.csv`.

## 6. Operating system and ROS 2 distribution

| Item | Choice | Reason / compatibility note |
|---|---|---|
| SBC OS | Ubuntu Server 24.04 LTS (arm64) | first Ubuntu LTS with Raspberry Pi 5 support; Tier-1 platform for Jazzy |
| ROS 2 | Jazzy Jalisco (LTS, supported to May 2029) | binary packages for 24.04 arm64; Nav2, slam_toolbox, robot_localization available |
| Simulator | Gazebo Harmonic (gz-sim 8) | the Gazebo release paired with Jazzy |
| Lidar driver | `sllidar_ros2` (Slamtec) | supports the C1; built from source in the workspace |
| Cameras | `v4l2_camera` with UVC webcams | avoids the libcamera/Ubuntu integration risk of CSI cameras on the Pi 5 |
| Python | 3.12 (system) | core library requires ≥ 3.10; tested here on 3.13 |

Compatibility items that could not be checked without the hardware are listed in `docs/10_validation_test_report.md` §5.
