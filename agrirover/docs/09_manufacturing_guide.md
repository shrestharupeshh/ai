# 09 — Manufacturing and sourcing guide

Files: `manufacturing/bom_master.csv` (33 lines, every purchased and fabricated item), `manufacturing/cost_summary.md`,
`electrical/pcb/fab/` (Gerbers, drill, pick-and-place, PCB BOM), `mechanical/export/` (DXF, STEP, STL, 3MF, drawings).

## 1. How to read the BOM

* `manufacturer_hq` is where the brand is based; `country_of_origin` is where the part is made **only when the
  manufacturer states it** — otherwise it reads "not verified". Pololu, Adafruit, Bioenno, 80/20 and SendCutSend are
  US companies; most of the electronic components they sell or use are manufactured in Asia. Adafruit states that it
  manufactures its own breakout boards in New York, and 80/20 extrudes in Indiana; treat both as "confirm on the
  packing slip". The Raspberry Pi 5 is built in Wales. The lidar and the MCU module are Chinese products with no
  equivalent US-made part at a comparable price.
* `price_spec_verified` distinguishes prices checked on the web in November 2026 from recalled or estimated ones. About
  40 % of the parts total (502 of 1268 USD) rests on checked prices.
* No stock level is recorded. Check availability when ordering; the only item with a visible supply problem during
  these checks was the Raspberry Pi 5, whose list price rose several times in 2026.

## 2. Supplier plan (US-based distributors)

| Supplier | Items | Typical lead time |
|---|---|---|
| Pololu (Las Vegas, NV) | gearmotors, brackets, wheels, hub adaptors, motor-driver carriers, 5 V regulator | ships next business day when in stock |
| DigiKey / Mouser | ARC-1 components, connectors, fuses, Adafruit boards, STM32 module (via Adafruit), switches | 1–3 days |
| Adafruit (New York, NY) | sensor breakouts, Qwiic cables, Black Pill | 1–5 days |
| Bioenno Power (California) | battery and charger | 2–5 days, ground shipping only (lithium) |
| PiShop.us / Micro Center / DigiKey | Raspberry Pi 5, cooler, storage | variable |
| DFRobot US warehouse / RobotShop / Amazon | RPLIDAR C1 | 2–7 days |
| SendCutSend (Reno, NV) / OSH Cut (Utah) | laser-cut aluminium and polycarbonate from the DXFs | 3–7 days |
| 80/20 / Misumi USA / McMaster-Carr | T-slot extrusion cut to length, gussets, fasteners, casters, grommets, glands | 1–5 days |
| OSH Park (US-made boards) or JLCPCB | ARC-1 bare boards | 5–12 days |

## 3. Fabrication notes

**Laser-cut plates.** Upload the DXFs as supplied (units mm, 1:1). 5052-H32, 4.0 mm (lower deck), 3.0 mm (upper deck,
tray), 2.0 mm (caster shim). Ask for deburring. Holes are drawn at clearance size: Ø3.4 (M3), Ø4.5 (M4), Ø5.5 (M5).
Check the first article against drawings PRT-101…103 (hole tables are generated from the model).

**Extrusion.** 20 × 20 mm T-slot (6 mm slot), 4 × 100.0 ± 0.2 mm and 1 × 1070 ± 1 mm, square ends. Tap M5 in the
centre bore (drill 4.2 mm if the bore is undersize): both ends of the posts, one end of the mast.

**Printed parts.** ASA, 0.2 mm layers, 4 perimeters, 40 % gyroid. Orientations: battery cradle open side up; camera
head with the mast bore vertical; sensor shield base down (the louvres are self-supporting cones); bumper flat.
Expect ±0.3 mm; ream the 20.4 mm mast bores if tight.

**ARC-1 PCB.** 2 layers, FR-4 1.6 mm, 1 oz, HASL or ENIG, any colour; minimum track/space 0.25/0.2 mm, minimum drill
0.4 mm, board 100 × 82 mm. Upload `arc1_gerbers.zip`; `arc1_pos.csv` and `arc1_bom.csv` serve an assembly house, but
the board is designed for hand assembly (0805, SOIC-8, DPAK, through-hole). **Do not order before completing the
review steps in `docs/04_electrical.md` §4.**

## 4. Incoming inspection

| Item | Check |
|---|---|
| Battery | label values (voltage, capacity, maximum continuous discharge); open-circuit voltage 13.0–13.6 V; Powerpole polarity (red = +) with a multimeter |
| Gearmotors | free-running current at 12 V ≈ 0.2 A; no axial play; encoder gives two clean square waves |
| Black Pill | silkscreen order of both header rows against `design.py`; MCU marking STM32F411CEU6 |
| Plates | overall size, hole positions against the hole tables, flatness < 1 mm over the length |
| ARC-1 | visual, then bring-up table in `docs/06_assembly_guide.md` §E |

## 5. Cost

Prototype material cost ≈ **1 420 USD** (parts 1 268 USD + 12 % shipping/tax allowance), plus roughly 16 h of assembly
and bring-up. Estimated unit cost in a batch of 25 ≈ 1 200 USD including 4 h of assembly labour. The breakdown and the
assumptions behind the batch figure are in `manufacturing/cost_summary.md`; both are estimates, not quotations.
