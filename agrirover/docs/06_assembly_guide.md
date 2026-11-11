# 06 — Assembly guide

Tools: 2/2.5/3/4 mm hex keys, M5 tap and wrench, drill press or hand drill, soldering station, crimp tools for JST-XH
and Powerpole, multimeter, bench supply with current limit (0–15 V, 3 A), thread-locker (medium), calipers.
Read `docs/13_safety.md` first. Estimated time: 12–16 h including electrical bring-up.

## A. Before cutting metal

1. Order the purchased parts first. When they arrive, measure: motor-bracket base holes, caster plate hole pattern and
   overall caster height, lidar base hole pattern, E-stop panel cut-out.
2. Correct the constants at the top of `mechanical/cad/rover_cad.py` if they differ; run
   `python3 mechanical/cad/rover_cad.py && python3 mechanical/cad/make_drawings.py`; check that
   `export/cad_summary.json` reports no interferences.
3. Send the DXFs for cutting; print the STL/3MF set in ASA.

## B. Chassis

1. Tap M5 × 12 deep in both ends of the four 100 mm posts and in one end of the mast.
2. Deburr the plates. Fit grommets to the 16 mm cable holes.
3. Bolt each gearmotor to its bracket (6 × M3 × 6, thread-locker; **screw engagement into the gearbox face must not
   exceed 3 mm**). Bolt the brackets under the lower deck, shafts outward (M3 × 8, nyloc nuts).
4. Slide the hub adaptors on the D-shafts, set screws on the flat, thread-locker. Bolt on the wheels.
5. Fit the casters: rear caster on the 2 mm shim, front caster directly on the deck. Stand the deck on a flat surface:
   it should rest on both wheels and the rear caster with ~2 mm under the front caster. Adjust shims.
6. Bolt the four posts to the lower deck from below (M5 × 12 button head).

## C. Electronics bay

1. Bolt the battery cradle to the lower deck (4 × M4). Fit the strap through the slots.
2. Mount ARC-1, the Raspberry Pi, the two motor-driver carriers and the 5 V regulator on the electronics tray with
   M3 / M2.5 nylon standoffs. Stand the tray on four M3 × 15 standoffs on the lower deck.
3. Wire per `docs/04_electrical.md` §5. Leave the battery disconnected. Route motor power away from the encoder and
   UART leads; cable-tie to the tray edge.
4. Fit the bumper: two M5 shoulder screws through the nose of the lower deck as guide pins, light compression springs
   behind the bar, one sealed micro-switch each side wired in series (NC), 3–5 mm travel before the bar touches the deck.

## D. Upper deck, mast, sensors

1. Fit the E-stop in the 22.3 mm hole (rear left), the lidar on its adaptor plate with its cable exit facing the mast,
   cable glands in the 16 mm holes.
2. Bolt the mast to the upper deck through the centre hole (M5 into the tapped end) and add the four gussets.
3. Clamp the camera head at 1230 mm above the floor. Screw the cameras to the tilted faces with ¼-20 screws. Fix the
   grey reference card on a short arm so that it appears in the top-left corner of each image.
4. Push the sensor shield onto the mast 130 mm below the camera head; mount SCD41, SHT45 inside, AS7341 in the cap
   pocket looking up. Daisy-chain Qwiic cables down the mast slot to the Pi.
5. Lower the upper deck onto the posts, feeding cables through the grommets; 4 × M5.
6. Fit skirts and corner pieces last (M3 into the deck edge holes) — leave them off for bring-up.

## E. Electrical bring-up (in this order)

| Step | Action | Expected | If not |
|---|---|---|---|
| 1 | ARC-1 alone, no module, no fuses: check J1+ to GND resistance | > 10 kΩ | find the short before applying power |
| 2 | Bench supply 12.8 V limited to 0.2 A on J1, F1 fitted | green LED on; +5 V = 4.9–5.1 V; +3.3 V = 3.2–3.4 V; current < 30 mA | power off, inspect U3/U4 orientation |
| 3 | Reverse the supply leads briefly | no current, nothing warm | Q1/D1 orientation |
| 4 | With E-stop released, jumper `MOTOR_EN` (A2 pin 14) to 3.3 V | red VMOT LED on, VMOT ≈ supply | Q2/Q3, J14 loop |
| 5 | Press the E-stop | VMOT LED goes out after ≈ 0.4 s | C3/R3 values |
| 6 | **Verify the Black Pill pinout against `design.py`**, then plug the module in; flash `agrirover_fw.bin` by DFU | on-board LED blinks at 1 Hz | docs/12 |
| 7 | Connect the Pi UART; run `python3 tools/base_console.py /dev/ttyAMA0` | state `IDLE`, battery voltage within 1 % of a multimeter | swap TX/RX; check baud |
| 8 | Wheels off the ground, drivers and motors connected, supply limit 3 A: command 2 rad/s | both wheels turn forward, encoder counts increase on both | swap motor leads or encoder A/B for a wheel running backwards |
| 9 | Press the E-stop while running | wheels brake to a stop in < 0.5 s, state `ESTOP`, cannot be re-enabled until released and cleared | do not proceed until this works |
| 10 | Battery in place of the bench supply; repeat 7–9 | same | |

Only after step 10 put the robot on the floor, and only in an empty area for the first drives (`docs/11`).
