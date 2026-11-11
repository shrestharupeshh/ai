# 03 — Design decisions and trade-offs

Each decision records the options considered, the choice and what would change it. Scores are 1 (poor) – 5 (good).

## D1. Drive architecture

| Criterion (weight) | Differential drive, 2 driven + casters | 4-wheel skid-steer | 4-wheel independent drive/steer |
|---|---|---|---|
| Manoeuvrability in 0.6–0.8 m aisles (5) | 5 — turns about its centre | 3 — turns in place but scrubs | 5 |
| Odometry quality (4) | 4 — rolling contact, little slip | 2 — slip is intrinsic to turning | 4 |
| Cost and part count (4) | 5 — 2 motors, 2 drivers | 3 — 4 motors | 1 — 8 actuators |
| Mechanical and control simplicity (4) | 5 | 4 | 1 |
| Floor damage / energy when turning (2) | 5 | 2 | 5 |
| Traction on ramps and wet patches (2) | 3 | 5 | 5 |
| Weighted total (max 105) | **97** | 65 | 69 |

**Choice:** differential drive with the axle at the geometric centre, one rear and one front caster on the centreline
(diamond). The front caster is shimmed 2 mm clear so three points carry the robot on an imperfect floor; 89 % of the
weight sits on the driven wheels. **Would change if** the site has slopes above ~8 % or loose/soft ground.

Centre-axle rather than rear-axle drive was chosen because a rear-axle layout of the same deck has a 0.72 m turning
envelope and could not turn in a 0.70 m aisle.

## D2. High-level computer

Compute need, estimated (to be measured, T-PWR-02): Nav2 + AMCL ≈ 1–1.5 cores of a Cortex-A76; classical vision on a
960 px image ≈ 60 ms; an optional YOLOv8n-class ONNX model ≈ 150–300 ms per frame on CPU — and inference runs only
while the robot is stopped at a station, so there is no real-time vision requirement.

| | Raspberry Pi 5 (8 GB) | Jetson Orin Nano Super | x86 mini PC (N100) |
|---|---|---|---|
| Meets the compute need | yes (CPU only) | yes, with large GPU margin | yes |
| Power | 4–10 W | 7–25 W | 8–20 W |
| Price (Nov 2026) | ~130–200 USD, volatile | ~250 USD | ~180 USD |
| ROS 2 Jazzy support | Ubuntu 24.04 arm64, Tier 1 | JetPack lags Ubuntu releases; usually via containers | Tier 1 |
| I/O for this robot | UART, I²C, 4 × USB | same + CSI | needs USB adapters for UART/I²C |

**Choice:** Raspberry Pi 5. **Would change if** a real-time or multi-camera neural pipeline becomes a requirement
(then Jetson Orin Nano; nothing in the software is Pi-specific apart from device paths).

## D3. Low-level controller

| | STM32F411 (Black Pill / Nucleo) | ESP32-S3 | RP2040 (Pico) |
|---|---|---|---|
| Quadrature decoding | hardware timer encoder mode, 2 × free of CPU load | PCNT unit (works; glitch filter coarse) | PIO program (works) |
| Motor PWM + ADC timing | advanced timers, 12-bit ADC | LEDC + ADC with known non-linearity | PWM slices, 12-bit ADC |
| Determinism | bare metal, no radio stack | Wi-Fi/BT stack shares the SoC | good |
| 5 V-tolerant inputs | most pins | none | none |
| Debug / recovery | SWD, ROM DFU bootloader | USB-JTAG | SWD, UF2 |
| Radio (not wanted next to the safety loop) | none | Wi-Fi/BT | none |

**Choice:** STM32F411CEU6 on a socketed Black Pill module, developed on a NUCLEO-F411RE. No CAN: the F411 has no CAN
controller and nothing on the robot speaks CAN; the SPI expansion header is reserved for an MCP2515 if that changes.
**Trade-off accepted:** the Black Pill is a hobby-grade module without supply guarantees (rev B can place the MCU
directly once the pin map is proven).

## D4. Custom firmware protocol instead of micro-ROS / ros2_control

A 37-byte framed telemetry message and two commands are enough for a two-wheel base. A small protocol can be unit-tested
on the host, cross-checked between C and Python, and has no agent process whose failure modes need analysing. The cost
is that the base is not a `ros2_control` hardware interface; `agrirover_base` publishes standard topics instead.
**Would change if** the base gains more axes or is shared with other ROS projects.

## D5. Is a custom PCB necessary?

Not strictly: a bench prototype can be wired from modules. A carrier board is nevertheless justified because the
functions that modules do not provide are exactly the safety- and reliability-relevant ones — fused distribution,
reverse-polarity protection, the E-stop gate chain with its delayed cut, battery monitoring, filtered encoder inputs
and locking connectors in a humid, vibrating enclosure. The board therefore **carries** modules instead of replacing
them: the MCU, motor drivers and the 5 V/5 A SBC supply stay commercial modules. This keeps fine-pitch parts and
switching-converter layout off the first custom board.

## D6. Battery

| | LiFePO₄ 4S pack with BMS | Li-ion 3S/4S (NMC) | Sealed lead-acid 12 V |
|---|---|---|---|
| Energy for ~1.5 kg | 154 Wh | ~220 Wh | ~45 Wh |
| Thermal-runaway behaviour | benign | needs more care | benign |
| Cycle life | > 2000 | 300–800 | 200–400 |
| Voltage window vs 12 V motors | 10–14.6 V | 9–12.6 / 12–16.8 V | 10.5–14.4 V |
| Off-the-shelf pack with BMS and matched charger from a US vendor | yes | yes | yes |

**Choice:** Bioenno BLF-1212A (12.8 V, 12 Ah) and its 14.6 V charger. The 12 Ah pack gives ~7.8 h against a 4 h
requirement; the 6 Ah version would also meet the requirement (~3.9 h with reserve is marginal) and saves 0.7 kg.

## D7. Sensors: essential, optional, rejected

| Sensor | Verdict | Reason |
|---|---|---|
| 2D lidar (RPLIDAR C1, DToF) | essential | localisation and the protective field; DToF chosen because triangulation lidars degrade in daylight |
| Wheel encoders | essential | speed control and odometry |
| Gyro (LSM6DSOX) | essential | wheel odometry alone is poor in turn-in-place manoeuvres |
| RGB cameras ×2 (left/right) | essential | plant images of both benches in one pass |
| Grey reference card in view | essential | colour metrics are meaningless under changing / coloured light without it; also the blur target |
| T/RH (SHT45), CO₂ (SCD41) | essential | project objective; SHT45 because the SCD41's own T/RH is ±0.8 °C / ±6 % |
| Spectral light sensor (AS7341) | essential, with caveat | lux is meaningless under LED spectra; PPFD still needs calibration |
| Contact bumper | essential | benches are invisible to the lidar |
| Depth camera (e.g. OAK-D Lite) | optional, **recommended before working around people** | would see bench edges and low obstacles the 2D lidar cannot; adds cost, compute and a calibration |
| Soil-moisture probe | rejected | hydroponic lettuce has no soil to probe and the rover has no manipulator |
| GNSS | rejected | indoors |

## D8. Vision approach

Start with a classical, explainable pipeline (segmentation by saturation / excess green, slot-based association, colour
fractions, relative size) whose failure modes are understood, and gate every claim behind an image-quality check and a
confidence value. A trained detector is provided for as an interface only; no weights are shipped because no labelled
data from the target greenhouse exists. See `docs/07_computer_vision.md`.

## D9. What was deliberately left out of revision A

Spraying, harvesting, any manipulator, automatic docking/charging, multi-robot coordination, cloud services.
