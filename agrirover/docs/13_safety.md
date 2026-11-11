# 13 — Safety assessment (revision A)

A 11 kg, 0.3–0.5 m/s machine is low-energy, but it works unattended among people, water and plants, carries a 154 Wh
battery and has a 1.25 m mast. This is an engineering hazard review to guide the design and the test plan. It is **not**
a conformity assessment: no standard (ISO 12100, ISO 13482, ISO 3691-4, UL 3100, IEC 62133 …) has been applied
formally, and no safety function has a verified performance level.

| # | Hazard | Cause | Design measures | Residual risk / open action |
|---|---|---|---|---|
| 1 | Collision with a person's legs or feet | person steps into the path, lidar misses dark/thin objects, software fault | 0.3 m/s in aisles; lidar protective field (stop ≤ 0.15 m); full-width bumper; firmware time-outs; E-stop | contact force and pressure at the bumper not measured → **T-SAF-05** before operating near people |
| 2 | Collision with bench structure | benches are above the lidar plane | keep-out mask from the facility drawing; aisle-entry way-points; map-checked turning and reversing; bumper; localisation-confidence stop | a wrong map or a localisation error can still bring the mast or deck against a bench edge at low speed → site survey check; depth camera recommended (docs/03 D7) |
| 3 | Running over hoses, cables, feet | low obstacles below 0.2 m are invisible to the lidar | 31 mm ground clearance; route inspection before missions; bumper lower edge 70 mm above the floor | bumper does not reach the floor → consider a skirt strip or a downward ToF sensor |
| 4 | Tip-over | mast height, ramps, collision with a kerb | CG 222 mm high; static tip angle 26.6°; 5 % slope limit | payload raised above the deck or mast extended beyond 1.4 m invalidates the calculation |
| 5 | Unintended motion at start-up or after a fault | MCU reset, software hang, lost link | motor rail off by default (gate pull-up), enable needs MCU **and** closed E-stop loop; watchdog; latched faults need an explicit clear; resume refused while the E-stop is pressed | single-channel design: one FET, one MCU input |
| 6 | E-stop ineffective | welded FET, wiring error | hardware gate chain independent of firmware; `VMOT_SENSE` cross-check; weekly functional test | no redundancy; uncontrolled coast distance unmeasured → **T-SAF-03** |
| 7 | Battery fire / venting | short circuit, over-charge, mechanical damage, water | LiFePO₄; pack BMS; fuse within 150 mm; reverse-polarity FET; branch fuses; pack strapped inside the bay; attended charging off the robot floor area | charger compatibility and BMS limits taken from the vendor description, not tested |
| 8 | Electric shock | — | system voltage ≤ 14.6 V DC (extra-low voltage) | none beyond burns from short circuits → insulated tools, remove jewellery |
| 9 | Water ingress → short circuit, corrosion | irrigation drips, condensation | roof-deck overhang, skirts, glands, conformal-coat option for ARC-1, sealed switches | no IP rating; **T-ENV-01** drip test |
| 10 | Pinch / entanglement | exposed wheels, bumper travel | wheels partly shrouded by the deck; low torque (stall force ≈ 50 N per wheel, current-limited) | long hair/cords near wheels → operator instruction |
| 11 | Mast strike / eye-level camera head | mast at 1.25 m in a walkway | rounded printed head, bright colour | — |
| 12 | Laser | lidar | Class 1 device per the vendor | — |
| 13 | Wrong agronomic decision from robot output | false or missed flags taken as diagnosis | outputs labelled as screening with confidence and pipeline level; image always shown | organisational: a person inspects before any treatment |
| 14 | Biosecurity: spreading pathogens between benches or houses | wheels and casters carry water and debris | smooth wipe-clean wheels and deck | no disinfection procedure defined → site-specific protocol needed |
| 15 | Loss of data / privacy | images may include workers | cameras point down at benches; data stays on the robot; no cloud | add a site notice |

## Conditions for first operation

1. Bring-up table in `docs/06` passed, including the E-stop test.
2. T-SAF-01…04 passed in an empty area.
3. First ten missions attended by a person within reach of the E-stop.
4. No operation with the public or untrained staff present until T-SAF-05 has been done and reviewed.
