# 05 — Mechanical design

Model: `mechanical/cad/rover_cad.py` (CadQuery, parametric from `config/rover.yaml`). Calculations:
`mechanical/calcs/mechanical_calcs.md`. Exports in `mechanical/export/`: STEP assembly + part STEPs, STL and 3MF for
printed parts, DXF flat patterns, drawings (PDF/PNG), `cad_parts_list.csv`, `cad_summary.json`.

## 1. Layout

A two-deck chassis. The lower deck (4 mm 5052-H32) carries the two gearmotors underslung in stamped brackets, the two
casters, the battery cradle and the electronics tray. Four 100 mm lengths of 2020 extrusion carry the upper deck
(3 mm 5052-H32), which is the roof of the electronics bay and carries the lidar, the 1.07 m mast, the E-stop and a
50 mm M5 payload grid. Polycarbonate skirts and printed corner pieces close the bay; the upper deck overhangs them so
drips run off. The mast top carries a printed twin camera head (35° below horizontal, one camera to each side) and,
130 mm below it, a louvred shield for the climate sensors.

Key numbers (from the calculations and the CAD summary): envelope 482 × 354 × 1253 mm; design mass 11.4 kg with 2 kg
payload (9.4 kg empty budget, 8.0 kg modelled in CAD without wiring and small hardware); CG 20 mm behind the axle,
222 mm above the floor; ground clearance 31.5 mm; turning envelope Ø 559 mm.

## 2. Parts we make

| Part | Process | Material | File |
|---|---|---|---|
| Lower deck, upper deck, electronics tray, caster shim | laser cut | 5052-H32, 4 / 3 / 3 / 2 mm | `export/dxf/*.dxf`, drawings PRT-101…103 |
| Side and end skirts | laser cut or routed | polycarbonate 2 mm | `export/dxf/skirt_*.dxf` |
| Deck posts ×4, mast | cut to length, tap M5 in the centre bore | 6063-T5 2020 T-slot | lengths in `cad_parts_list.csv` |
| Battery cradle, camera head, sensor shield, lidar adaptor, skirt corners ×4, bumper | FDM print | ASA (UV- and humidity-stable); PETG acceptable away from glazing | `export/stl`, `export/3mf` |

Printed parts are not in the primary load path: the cradle locates the battery (the strap and the deck carry it), the
camera head carries 0.3 kg. Print with 4 perimeters, 40 % infill, layers parallel to the clamp screws' axis for the
camera head.

## 3. Purchased mechanical parts

Motors, brackets, wheels, hub adaptors, casters, gussets and fasteners — see `manufacturing/bom_master.csv`. Their CAD
geometry is an **envelope built from published overall dimensions**, marked `COTS` in the model, and must not be used
to derive hole patterns: the motor-bracket and caster hole positions in the lower deck drawing are to be confirmed
against the delivered parts before the plate is cut (measure, edit the constants at the top of `rover_cad.py`,
re-export).

## 4. Results of the calculations

| Check | Result | Requirement / limit |
|---|---|---|
| Wheel torque, accelerating up a 5 % ramp | 0.43 N·m per wheel | gearbox continuous limit 0.98 N·m → margin 2.3 |
| Speed at that load, 11 V bus | 0.59 m/s | ≥ 0.5 m/s |
| Traction margin, wet floor (μ = 0.35 assumed) | 2.4× | > 1 |
| Stop distance from 0.5 m/s at 1.5 m/s² | 93 mm (bumper/E-stop), 143 mm (lidar field) | ≤ 150 mm |
| Static tip angle | 26.6° lateral, 37° rearward | ≥ 20° |
| Lower-deck bending stress / deflection | 4.9 MPa / 0.23 mm | yield 193 MPa |
| Motor shaft bending (2.5× dynamic factor) | 162 MPa | shaft steel > 400 MPa; **bushing radial rating unpublished** |
| Mast first bending mode | ≈ 6.8 Hz | sets the 1.5 s settle time before capture |

## 5. Mechanical risks carried into the build

1. **Gearmotor output bushing.** Each wheel puts ~50 N static radial load on a 6 mm shaft 27 mm from the gearbox face.
   Pololu publishes no radial rating. Test T-MEC-04 inspects for play after 20 h; the deck has room for an outboard
   bearing block if needed.
2. **Hole patterns of purchased parts** are unverified (section 3).
3. **Caster height.** The design needs 71 mm overall caster height; the rear caster sits on a 2 mm shim. Choose casters
   of 65–71 mm and adjust the shim stack so the deck is level and the front caster clears the floor by 2 mm.
4. **Mast vibration** on rough thresholds may need a diagonal brace (T-slot gussets already allow one).
5. **Ingress.** The skirt/deck arrangement is drip-shedding by design only; no IP test has been done. Do not hose down.
6. **The bumper** is modelled as a bar; its guide pins, springs and micro-switch brackets are described in the assembly
   guide but are not detailed in CAD.
