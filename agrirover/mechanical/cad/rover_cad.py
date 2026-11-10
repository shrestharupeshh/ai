#!/usr/bin/env python3
"""Parametric CAD model of AgriRover GH-1 (CadQuery / OpenCascade).

    python3 mechanical/cad/rover_cad.py            # build + export everything

Geometry is driven by config/rover.yaml. World frame: x forward, y left,
z up, origin on the floor below the drive-axle centre. Units: mm.

Part classes
  FAB   parts we manufacture (laser-cut, machined, cut-to-length, printed).
        Geometry here is the manufacturing definition.
  COTS  purchased parts. Geometry is a simplified ENVELOPE built from the
        published overall dimensions for fit checks only - never manufacture
        from it and verify hole patterns against the physical part.
"""
from __future__ import annotations

import csv
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import cadquery as cq
import yaml

ROOT = Path(__file__).resolve().parents[2]
EXPORT = ROOT / "mechanical" / "export"
CFG = yaml.safe_load((ROOT / "config/rover.yaml").read_text())
C = CFG["chassis"]

# ----------------------------------------------------------------- parameters
L, W, CH = C["plate_length_mm"], C["plate_width_mm"], C["plate_corner_chamfer_mm"]
T_LO, T_UP = C["lower_plate_thk_mm"], C["upper_plate_thk_mm"]
Z_LO = C["lower_plate_bottom_z_mm"]            # underside of lower plate
Z_LO_TOP = Z_LO + T_LO
POST = C["deck_post_len_mm"]
Z_UP = Z_LO_TOP + POST                          # underside of upper plate
Z_UP_TOP = Z_UP + T_UP
TRACK, WD, WW = C["track_width_mm"], C["wheel_diameter_mm"], C["wheel_width_mm"]
AXLE_Z = WD / 2
CASTER_X = C["caster_x_mm"]
MAST_H, MAST_X = C["mast_height_mm"], C["mast_x_mm"]
CAM_Z, CAM_TILT = C["camera_z_mm"], C["camera_tilt_deg"]
LIDAR_X = C["lidar_x_mm"]
BAT = CFG["battery"]["size_mm"]                 # 215 x 56 x 79
POST_XY = [(sx * (L / 2 - 32), sy * (W / 2 - 30)) for sx in (1, -1) for sy in (1, -1)]  # clear of the corner chamfer
BRACKET_Y = W / 2 - 10                          # outer face of motor bracket
MOTOR_BRACKET_HOLES_X = (-21, 21)               # bracket base slots (2 x M3 per bracket row)
TRAY_STANDOFFS = [(40, -75), (40, 75), (200, -75), (200, 75)]
RHO = {"al": 2.70e-6, "pc": 1.20e-6, "asa": 1.07e-6 * 0.55, "steel": 7.85e-6}  # kg/mm^3 (ASA at ~55 % effective infill)


@dataclass
class Part:
    name: str
    shape: cq.Workplane
    kind: str                     # FAB or COTS
    process: str
    material: str
    color: tuple
    qty_locs: list = field(default_factory=list)   # list of cq.Location
    density: float | None = None  # kg/mm^3 for FAB parts
    mass_kg: float | None = None  # catalogue mass for COTS parts
    pn: str = ""
    note: str = ""


def loc(x=0, y=0, z=0, rx=0, ry=0, rz=0) -> cq.Location:
    return cq.Location((x, y, z), (rx, ry, rz))


# ----------------------------------------------------------------- FAB parts
def deck_outline(thk: float) -> cq.Workplane:
    return cq.Workplane("XY").rect(L, W).extrude(thk).edges("|Z").chamfer(CH)


def lower_plate() -> cq.Workplane:
    p = deck_outline(T_LO)
    top = p.faces(">Z").workplane()
    p = top.pushPoints(POST_XY).hole(5.5)                                   # M5 into post ends
    # motor brackets (underslung): 2 x M3 each side
    pts = [(x, s * (BRACKET_Y - 12)) for s in (1, -1) for x in MOTOR_BRACKET_HOLES_X]
    pts += [(x, s * (BRACKET_Y - 28)) for s in (1, -1) for x in MOTOR_BRACKET_HOLES_X]
    p = p.faces(">Z").workplane().pushPoints(pts).hole(3.4)
    # casters: 4 x M4 on a 30 x 30 pattern, front and rear
    cpts = [(sx * CASTER_X + dx, dy) for sx in (1, -1) for dx in (-15, 15) for dy in (-15, 15)]
    p = p.faces(">Z").workplane().pushPoints(cpts).hole(4.5)
    # battery cradle (4 x M4) and electronics-tray standoffs (4 x M3)
    bpts = [(-45 + dx, dy) for dx in (-45, 45) for dy in (-95, 95)]
    p = p.faces(">Z").workplane().pushPoints(bpts).hole(4.5)
    p = p.faces(">Z").workplane().pushPoints(TRAY_STANDOFFS).hole(3.4)
    # motor cable pass-throughs (grommeted) and a drain hole at the low point
    p = p.faces(">Z").workplane().pushPoints([(0, 60), (0, -60)]).hole(16)
    p = p.faces(">Z").workplane().pushPoints([(-120, 0)]).hole(6)
    # skirt mounting tabs: M3 along the edges
    spts = [(x, s * (W / 2 - 6)) for s in (1, -1) for x in (-150, -75, 75, 150)]
    p = p.faces(">Z").workplane().pushPoints(spts).hole(3.4)
    return p


def upper_plate() -> cq.Workplane:
    p = deck_outline(T_UP)
    p = p.faces(">Z").workplane().pushPoints(POST_XY).hole(5.5)
    # mast foot: centre M5 into extrusion end + 4 x M5 gusset bolts
    mpts = [(MAST_X, 0)] + [(MAST_X + dx, dy) for dx, dy in ((30, 0), (-30, 0), (0, 30), (0, -30))]
    p = p.faces(">Z").workplane().pushPoints(mpts).hole(5.5)
    # lidar adaptor: 4 x M3 on 45 x 45, cable gland hole behind it
    lpts = [(LIDAR_X + dx, dy) for dx in (-22.5, 22.5) for dy in (-22.5, 22.5)]
    p = p.faces(">Z").workplane().pushPoints(lpts).hole(3.4)
    p = p.faces(">Z").workplane().pushPoints([(LIDAR_X - 55, 0), (MAST_X + 45, 40)]).hole(16)
    # E-stop (22.3 mm panel hole) rear-left, status beacon (12 mm) rear-right
    p = p.faces(">Z").workplane().pushPoints([(-195, 60)]).hole(22.3)
    p = p.faces(">Z").workplane().pushPoints([(-195, -60)]).hole(12.2)
    # payload bay: M5 grid 50 mm pitch, mid-deck
    gpts = [(x, y) for x in (-50, 0, 50) for y in (-100, -50, 50, 100)]
    p = p.faces(">Z").workplane().pushPoints(gpts).hole(5.5)
    spts = [(x, s * (W / 2 - 6)) for s in (1, -1) for x in (-150, -75, 75, 150)]
    p = p.faces(">Z").workplane().pushPoints(spts).hole(3.4)
    return p


def extrusion_2020(length: float) -> cq.Workplane:
    """Simplified 20x20 T-slot (6 mm slot) profile with a 4.2 mm centre bore (tap M5)."""
    p = cq.Workplane("XY").rect(20, 20).extrude(length).edges("|Z").fillet(1.0)
    slot = cq.Workplane("XY").rect(6.2, 6.0).extrude(length)
    for ang in (0, 90, 180, 270):
        p = p.cut(slot.translate((0, 7.0, 0)).rotate((0, 0, 0), (0, 0, 1), ang))
    return p.faces(">Z").workplane().hole(4.2)


def battery_cradle() -> cq.Workplane:
    """Printed (ASA) cradle; battery lies transverse. Two strap slots for a 25 mm hook-and-loop strap."""
    lx, ly, wall, h = BAT[1] + 2.0, BAT[0] + 2.0, 4.0, 30.0
    p = cq.Workplane("XY").rect(lx + 2 * wall, ly + 2 * wall).extrude(h + 3)
    p = p.cut(cq.Workplane("XY").workplane(offset=3).rect(lx, ly).extrude(h))
    ears = cq.Workplane("XY").rect(110, 20).extrude(3)
    for sy in (-95, 95):
        p = p.union(ears.translate((0, sy * 1.0 + (12 if sy > 0 else -12), 0)))
    p = p.faces("<Z").workplane().pushPoints([(dx, dy) for dx in (-45, 45) for dy in (-107, 107)]).hole(4.5)
    slot = cq.Workplane("XY").rect(lx + 2 * wall + 2, 27).extrude(3.5).translate((0, 0, 3))
    return p.cut(slot.translate((0, 60, 0))).cut(slot.translate((0, -60, 0)))


def electronics_tray() -> cq.Workplane:
    """3 mm aluminium tray on four M3 x 15 standoffs: carrier PCB, Pi 5, two motor-driver carriers, DC/DC."""
    p = cq.Workplane("XY").rect(190, 180).extrude(3).edges("|Z").fillet(4)
    p = p.faces(">Z").workplane().pushPoints([(dx, dy) for dx in (-80, 80) for dy in (-75, 75)]).hole(3.4)
    pcb = [(-35 + dx, 40 + dy) for dx in (-46, 46) for dy in (-36, 36)]            # 100 x 80 PCB, 92 x 72 holes
    pi = [(-52.5 + 3.5 + dx, -52 + dy) for dx in (0, 58) for dy in (-24.5, 24.5)]  # Pi 5: 58 x 49 pattern, M2.5
    p = p.faces(">Z").workplane().pushPoints(pcb).hole(3.4)
    p = p.faces(">Z").workplane().pushPoints(pi).hole(2.7)
    p = p.faces(">Z").workplane().pushPoints([(55, -30), (55, -70), (75, 20), (75, 60)]).hole(2.7)
    return p


def mast_gusset() -> cq.Workplane:
    """Machined/cast-style 2020 corner gusset (COTS-equivalent geometry, modelled for fit)."""
    p = (cq.Workplane("XZ").polyline([(0, 0), (28, 0), (28, 4), (4, 28), (0, 28)]).close().extrude(18)
         .translate((0, 9, 0)))
    p = p.faces("<Z").workplane().pushPoints([(18, 0)]).hole(5.5)
    return p


def camera_mount() -> cq.Workplane:
    """Printed (ASA) twin camera head clamping on the 2020 mast.
    Two faces tilted CAM_TILT below horizontal, one looking left (+y), one right (-y)."""
    body = cq.Workplane("XY").rect(44, 60).extrude(46).translate((0, 0, -23))
    body = body.cut(cq.Workplane("XY").rect(20.4, 20.4).extrude(60).translate((0, 0, -30)))
    body = body.cut(cq.Workplane("XY").rect(2.0, 30).extrude(60).translate((16, 0, -30)))   # clamp split
    body = body.faces(">X").workplane(centerOption="CenterOfBoundBox").pushPoints([(0, 12), (0, -12)]).hole(4.5, 8)
    for s in (1, -1):
        plate = (cq.Workplane("XY").rect(44, 5).extrude(40).translate((0, 0, -20))
                 .faces(">Y" if s > 0 else "<Y").workplane(centerOption="CenterOfBoundBox")
                 .pushPoints([(0, 0)]).hole(6.6)                       # 1/4-20 tripod screw clearance
                 .rotate((0, 0, 0), (1, 0, 0), -s * CAM_TILT)
                 .translate((0, s * 40, -4)))
        web = (cq.Workplane("XY").rect(44, 30).extrude(13).translate((0, s * 36, 9.5)))   # bridge above the camera
        body = body.union(plate).union(web)
    return body


def sensor_pod() -> cq.Workplane:
    """Printed (white ASA) louvred radiation shield for SCD41 / SHT45 / AS7341 breakouts.
    Three stacked rings on four pillars over a base with a Qwiic cable exit; light sensor sits on the cap."""
    p = cq.Workplane("XY").circle(34).extrude(3)
    p = p.faces(">Z").workplane().pushPoints([(0, 0)]).hole(10)
    for i in range(3):
        ring = cq.Workplane("XY").workplane(offset=10 + i * 12).circle(34).circle(22).extrude(2.5)
        cone = (cq.Workplane("XY").workplane(offset=10 + i * 12 + 2.5).circle(34).workplane(offset=6).circle(26).loft()
                .cut(cq.Workplane("XY").workplane(offset=10 + i * 12).circle(22).extrude(12)))
        p = p.union(ring).union(cone)
    for ang in (45, 135, 225, 315):
        x, y = 27 * math.cos(math.radians(ang)), 27 * math.sin(math.radians(ang))
        p = p.union(cq.Workplane("XY").center(x, y).circle(3).extrude(46))
    cap = cq.Workplane("XY").workplane(offset=46).circle(34).extrude(3)
    cap = cap.faces(">Z").workplane().rect(20, 14).cutBlind(-1.2)        # AS7341 window pocket
    arm = cq.Workplane("XY").rect(40, 22).extrude(6).translate((-48, 0, 0))
    arm = arm.cut(cq.Workplane("XY").rect(20.4, 20.4).extrude(6).translate((-56, 0, 0)).translate((0, 0, 0)))
    return p.union(cap).union(arm)


def lidar_adaptor() -> cq.Workplane:
    """Printed adaptor plate: upper-deck 45 x 45 M3 pattern to the lidar base pattern (verify against the unit)."""
    p = cq.Workplane("XY").rect(62, 62).extrude(6).edges("|Z").fillet(5)
    p = p.faces(">Z").workplane().pushPoints([(dx, dy) for dx in (-22.5, 22.5) for dy in (-22.5, 22.5)]).cboreHole(3.4, 6.5, 3.2)
    p = p.faces(">Z").workplane().pushPoints([(dx, dy) for dx in (-14, 14) for dy in (-21.5, 21.5)]).hole(2.7)
    return p


def skirt_side() -> cq.Workplane:
    """2 mm polycarbonate side skirt (laser-cut / routed), overlaps both decks, closes the electronics bay."""
    h = POST + T_LO + T_UP + 4
    p = cq.Workplane("XZ").rect(L - 2 * CH - 4, h).extrude(2)
    p = p.faces(">Y").workplane().pushPoints([(x, z) for x in (-150, -75, 75, 150) for z in (-h / 2 + 8, h / 2 - 8)]).hole(3.4)
    return p


def skirt_end() -> cq.Workplane:
    h = POST + T_LO + T_UP + 4
    p = cq.Workplane("YZ").rect(W - 2 * CH - 4, h).extrude(2)
    return p


def skirt_corner() -> cq.Workplane:
    """Printed (ASA) chamfer-corner closure, 4 off (mirrored pairs share one STL)."""
    h = POST + T_LO + T_UP + 4
    n = CH * math.sqrt(2) - 1.0
    return cq.Workplane("XZ").rect(n, h).extrude(2.4).translate((0, 2.4, 0))   # local +y = outward


def bumper() -> cq.Workplane:
    """Printed (TPU/ASA) front bumper bar on two guide pins; trips two series NC micro-switches."""
    p = cq.Workplane("XY").rect(12, W - 2 * CH + 30).extrude(34).edges("|Z").fillet(4)
    p = p.faces("<X").workplane(centerOption="CenterOfBoundBox").pushPoints([(-80, 0), (80, 0)]).hole(5.2, 8)
    return p


def caster_shim() -> cq.Workplane:
    p = cq.Workplane("XY").rect(42, 42).extrude(C["front_caster_gap_mm"])
    return p.faces(">Z").workplane().pushPoints([(dx, dy) for dx in (-15, 15) for dy in (-15, 15)]).hole(4.5)


# ----------------------------------------------------------------- COTS envelopes
def motor_env() -> cq.Workplane:
    """Pololu 37D x 70L gearmotor + encoder; axis along +Y, gearbox face at y=0, shaft toward +Y."""
    body = cq.Workplane("XZ").circle(18.5).extrude(22)                      # gearbox, toward -Y
    can = cq.Workplane("XZ").workplane(offset=22).circle(17.25).extrude(48)
    enc = cq.Workplane("XZ").workplane(offset=70).circle(16.5).extrude(15)
    boss = cq.Workplane("XZ").workplane(offset=-6).center(0, 7).circle(6).extrude(6)
    shaft = cq.Workplane("XZ").workplane(offset=-22).center(0, 7).circle(3).extrude(22)
    return body.union(can).union(enc).union(boss).union(shaft)


def motor_bracket_env() -> cq.Workplane:
    """Pololu 1084-style stamped aluminium L-bracket (representative). Vertical face at y in [0, 2.5]."""
    face = cq.Workplane("XZ").rect(48, 46).extrude(-2.5).translate((0, 0, 0))
    face = face.faces(">Y").workplane(centerOption="CenterOfBoundBox").hole(13)
    base = cq.Workplane("XY").rect(48, 34).extrude(2.5).translate((0, -17 + 2.5, 23 - 2.5))
    return face.union(base)


def wheel_env() -> cq.Workplane:
    tyre = cq.Workplane("XZ").circle(WD / 2).circle(30).extrude(WW).translate((0, WW / 2, 0))
    hub = cq.Workplane("XZ").circle(30).circle(4).extrude(WW - 4).translate((0, WW / 2 - 2, 0))
    return tyre.union(hub)


def caster_env(height: float) -> cq.Workplane:
    plate = cq.Workplane("XY").rect(42, 42).extrude(-3)
    fork = cq.Workplane("XY").workplane(offset=-3).circle(12).extrude(-(height - 3 - 30))
    wheel = cq.Workplane("XZ").circle(20).extrude(18).translate((-12, 9, -(height - 20)))
    return plate.union(fork).union(wheel)


def box_env(x, y, z, fillet=0.0) -> cq.Workplane:
    p = cq.Workplane("XY").rect(x, y).extrude(z)
    return p.edges("|Z").fillet(fillet) if fillet else p


def lidar_env() -> cq.Workplane:
    base = cq.Workplane("XY").rect(55.6, 55.6).extrude(20).edges("|Z").fillet(8)
    head = cq.Workplane("XY").workplane(offset=20).circle(26).extrude(21.3)
    return base.union(head)


def camera_env() -> cq.Workplane:
    body = cq.Workplane("XY").rect(38, 30).extrude(30).translate((0, 0, -15))
    lens = cq.Workplane("XZ").circle(9).extrude(-8).translate((0, 15, 0))
    return body.union(lens)


def estop_env() -> cq.Workplane:
    return (cq.Workplane("XY").circle(11).extrude(-40).union(cq.Workplane("XY").circle(15).extrude(12))
            .union(cq.Workplane("XY").workplane(offset=12).circle(20).extrude(16)))


def screw(d: float, length: float, head_d: float, head_h: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(head_d / 2).extrude(head_h).union(cq.Workplane("XY").circle(d / 2).extrude(-length))


# ----------------------------------------------------------------- assembly
def build_parts() -> list[Part]:
    al, steel, black, blue, white, pcbg, clear = (0.78, 0.8, 0.82), (0.5, 0.5, 0.52), (0.1, 0.1, 0.1), (0.1, 0.3, 0.7), (0.93, 0.93, 0.9), (0.05, 0.4, 0.15), (0.6, 0.75, 0.85)
    parts: list[Part] = []
    add = parts.append
    add(Part("lower_plate", lower_plate(), "FAB", "laser cut, 5052-H32 4.0 mm", "Al 5052-H32", al, [loc(z=Z_LO)], RHO["al"]))
    add(Part("upper_plate", upper_plate(), "FAB", "laser cut, 5052-H32 3.0 mm", "Al 5052-H32", al, [loc(z=Z_UP)], RHO["al"]))
    add(Part("deck_post", extrusion_2020(POST), "FAB", "cut to length 100 mm, tap M5 both ends", "Al 6063-T5 2020 T-slot", al,
             [loc(x, y, Z_LO_TOP) for x, y in POST_XY], RHO["al"], pn="e.g. 80/20 20-2020 / Misumi HFS5-2020"))
    add(Part("mast", extrusion_2020(MAST_H), "FAB", f"cut to length {MAST_H} mm, tap M5 one end", "Al 6063-T5 2020 T-slot", al,
             [loc(MAST_X, 0, Z_UP_TOP)], RHO["al"]))
    add(Part("mast_gusset", mast_gusset(), "COTS", "2020 corner bracket", "die-cast Al", steel,
             [loc(MAST_X + 10, 0, Z_UP_TOP), loc(MAST_X - 10, 0, Z_UP_TOP, rz=180),
              loc(MAST_X, 10, Z_UP_TOP, rz=90), loc(MAST_X, -10, Z_UP_TOP, rz=270)], mass_kg=0.012, pn="2020 corner bracket"))
    add(Part("battery_cradle", battery_cradle(), "FAB", "FDM print, ASA", "ASA", black, [loc(-45, 0, Z_LO_TOP)], RHO["asa"]))
    add(Part("electronics_tray", electronics_tray(), "FAB", "laser cut, 5052 3.0 mm", "Al 5052-H32", al, [loc(120, 0, Z_LO_TOP + 15)], RHO["al"]))
    add(Part("camera_mount", camera_mount(), "FAB", "FDM print, ASA", "ASA", black, [loc(MAST_X, 0, CAM_Z)], RHO["asa"]))
    add(Part("sensor_pod", sensor_pod(), "FAB", "FDM print, white ASA", "ASA (white)", white, [loc(MAST_X + 56, 0, CAM_Z - 130)], RHO["asa"]))
    add(Part("lidar_adaptor", lidar_adaptor(), "FAB", "FDM print, ASA", "ASA", black, [loc(LIDAR_X, 0, Z_UP_TOP)], RHO["asa"]))
    zc = (Z_LO + Z_UP_TOP) / 2
    add(Part("skirt_side", skirt_side(), "FAB", "laser cut / routed, PC 2.0 mm", "polycarbonate", clear,
             [loc(0, W / 2 + 2, zc), loc(0, -W / 2, zc)], RHO["pc"]))
    add(Part("skirt_end", skirt_end(), "FAB", "laser cut / routed, PC 2.0 mm", "polycarbonate", clear,
             [loc(L / 2, 0, zc), loc(-L / 2 - 2, 0, zc)], RHO["pc"]))
    cx, cy = L / 2 - CH / 2, W / 2 - CH / 2
    add(Part("skirt_corner", skirt_corner(), "FAB", "FDM print, ASA", "ASA", black,
             [loc(cx, cy, zc, rz=-45), loc(-cx, cy, zc, rz=45),
              loc(cx, -cy, zc, rz=-135), loc(-cx, -cy, zc, rz=135)], RHO["asa"]))
    add(Part("bumper", bumper(), "FAB", "FDM print, ASA shell (TPU face optional)", "ASA", (0.9, 0.7, 0.1), [loc(L / 2 + 14, 0, Z_LO - 2)], RHO["asa"]))
    add(Part("caster_shim", caster_shim(), "FAB", "laser cut, Al 2.0 mm", "Al 5052", al, [loc(-CASTER_X, 0, Z_LO - C["front_caster_gap_mm"])], RHO["al"]))
    # --- COTS
    for s, name in ((1, "L"), (-1, "R")):
        rz = 0 if s > 0 else 180
        add(Part(f"motor_{name}", motor_env(), "COTS", "gearmotor with encoder", "-", steel,
                 [loc(0, s * (BRACKET_Y - 2.5), AXLE_Z - 7, rz=rz)], mass_kg=0.235, pn="Pololu 4754"))
        add(Part(f"motor_bracket_{name}", motor_bracket_env(), "COTS", "stamped Al L-bracket", "Al", al,
                 [loc(0, s * (BRACKET_Y - 2.5), AXLE_Z, rz=rz)], mass_kg=0.025, pn="Pololu 1084 (pair)"))
        add(Part(f"wheel_{name}", wheel_env(), "COTS", "PU scooter wheel + 6 mm hub adaptor", "PU / Al", black,
                 [loc(0, s * TRACK / 2, AXLE_Z)], mass_kg=0.25, pn="Pololu 3278 + 2674"))
    add(Part("caster_front", caster_env(Z_LO - C["front_caster_gap_mm"]), "COTS", "40 mm swivel caster", "-", steel,
             [loc(CASTER_X, 0, Z_LO)], mass_kg=0.15, pn="see BOM (height to be shimmed)"))
    add(Part("caster_rear", caster_env(Z_LO - C["front_caster_gap_mm"]), "COTS", "40 mm swivel caster", "-", steel,
             [loc(-CASTER_X, 0, Z_LO - C["front_caster_gap_mm"])], mass_kg=0.15, pn="see BOM"))
    add(Part("battery", box_env(BAT[1], BAT[0], BAT[2], 4), "COTS", "LiFePO4 12.8 V 12 Ah", "-", blue,
             [loc(-45, 0, Z_LO_TOP + 3)], mass_kg=1.5, pn="Bioenno BLF-1212A"))
    add(Part("control_pcb", box_env(100, 80, 1.6), "FAB", "PCB, see electrical/pcb", "FR-4", pcbg,
             [loc(120 - 35, 40, Z_LO_TOP + 18 + 8)], mass_kg=0.09))
    add(Part("sbc_pi5", box_env(85, 56, 18), "COTS", "Raspberry Pi 5 8 GB + active cooler", "-", pcbg,
             [loc(120 - 52.5 + 3.5 + 29, -52, Z_LO_TOP + 18 + 6)], mass_kg=0.09, pn="SC1112"))
    add(Part("motor_driver", box_env(25.4, 20.3, 6), "COTS", "TB9051FTG carrier", "-", (0.5, 0.1, 0.5),
             [loc(120 + 65, 20, Z_LO_TOP + 18 + 6, rz=90), loc(120 + 65, 60, Z_LO_TOP + 18 + 6, rz=90)], mass_kg=0.004, pn="Pololu 2997"))
    add(Part("dcdc_5v", box_env(43, 22, 12), "COTS", "5 V 5 A buck", "-", (0.1, 0.5, 0.2),
             [loc(120 + 55, -50, Z_LO_TOP + 18 + 6, rz=90)], mass_kg=0.02, pn="Pololu D36V50F5"))
    add(Part("lidar", lidar_env(), "COTS", "2D DToF lidar", "-", black, [loc(LIDAR_X, 0, Z_UP_TOP + 6)], mass_kg=0.11, pn="Slamtec RPLIDAR C1"))
    add(Part("camera", camera_env(), "COTS", "UVC camera", "-", (0.2, 0.2, 0.2),
             [loc(MAST_X, 0, CAM_Z, rz=rz) * loc(0, 40, -4) * loc(rx=-CAM_TILT) * loc(0, 17.6, 0) for rz in (0, 180)],
             mass_kg=0.16, pn="see BOM"))
    add(Part("estop", estop_env(), "COTS", "22 mm E-stop, 1NC", "-", (0.8, 0.05, 0.05), [loc(-195, 60, Z_UP_TOP)], mass_kg=0.07, pn="see BOM"))
    add(Part("screw_m5x12_bhcs", screw(5, 12, 9.5, 2.75), "COTS", "ISO 7380 M5x12 A2", "A2 stainless", steel,
             [loc(x, y, Z_UP_TOP) for x, y in POST_XY] + [loc(x, y, Z_LO, rx=180) for x, y in POST_XY] + [loc(MAST_X, 0, Z_UP, rx=180)],
             mass_kg=0.003, pn="McMaster 92095A210 (verify)"))
    return parts


def assemble(parts: list[Part]) -> cq.Assembly:
    asm = cq.Assembly(name="agrirover_gh1")
    for pt in parts:
        for i, lc in enumerate(pt.qty_locs):
            asm.add(pt.shape, name=f"{pt.name}_{i+1}" if len(pt.qty_locs) > 1 else pt.name, loc=lc, color=cq.Color(*pt.color, 1.0))
    return asm


def part_mass(pt: Part) -> float:
    if pt.mass_kg is not None:
        return pt.mass_kg
    return pt.shape.val().Volume() * pt.density


def interference_report(parts: list[Part]) -> list[tuple[str, str, float]]:
    """Pairwise boolean-intersection volume between FAB parts and everything else (mm^3 > 5 reported)."""
    placed = []
    for pt in parts:
        for i, lc in enumerate(pt.qty_locs):
            placed.append((f"{pt.name}_{i+1}", pt.shape.val().moved(lc), pt.name))
    hits = []
    for i in range(len(placed)):
        for j in range(i + 1, len(placed)):
            a, b = placed[i], placed[j]
            if "screw" in a[2] or "screw" in b[2]:      # threads engage tapped holes by design
                continue
            ba, bb = a[1].BoundingBox(), b[1].BoundingBox()
            if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin
                    or ba.zmax < bb.zmin or bb.zmax < ba.zmin):
                continue
            try:
                v = a[1].intersect(b[1]).Volume()
            except Exception:
                continue
            if v > 5.0:
                hits.append((a[0], b[0], v))
    return hits


def export_all() -> dict:
    parts = build_parts()
    for d in ("step", "stl", "3mf", "dxf"):
        (EXPORT / d).mkdir(parents=True, exist_ok=True)
    asm = assemble(parts)
    asm.save(str(EXPORT / "step" / "agrirover_gh1_assembly.step"))
    rows, total, cgx, cgz = [], 0.0, 0.0, 0.0
    for pt in parts:
        if pt.kind == "FAB" and pt.name != "control_pcb":
            cq.exporters.export(pt.shape, str(EXPORT / "step" / f"{pt.name}.step"))
            if "print" in pt.process:
                cq.exporters.export(pt.shape, str(EXPORT / "stl" / f"{pt.name}.stl"), tolerance=0.05, angularTolerance=0.2)
                cq.exporters.export(pt.shape, str(EXPORT / "3mf" / f"{pt.name}.3mf"), tolerance=0.05, angularTolerance=0.2)
            if "laser" in pt.process:
                sec = pt.shape.faces("<Z" if pt.name not in ("skirt_side", "skirt_end") else (">Y" if pt.name == "skirt_side" else ">X"))
                cq.exporters.export(cq.Workplane().add(sec.val()), str(EXPORT / "dxf" / f"{pt.name}.dxf"))
        m = part_mass(pt)
        for lc in pt.qty_locs:
            com = pt.shape.val().moved(lc).Center()
            total += m
            cgx += m * com.x
            cgz += m * com.z
        bb = pt.shape.val().BoundingBox()
        rows.append(dict(part=pt.name, qty=len(pt.qty_locs), kind=pt.kind, process=pt.process, material=pt.material,
                         unit_mass_kg=round(m, 4), bbox_mm=f"{bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f}", pn=pt.pn))
    with open(EXPORT / "cad_parts_list.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    hits = interference_report(parts)
    bb = asm.toCompound().BoundingBox()
    summary = dict(
        modelled_mass_kg=round(total, 3), cg_x_mm=round(cgx / total, 1), cg_z_mm=round(cgz / total, 1),
        envelope_mm=[round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)],
        z_min_mm=round(bb.zmin, 2), n_parts=sum(len(p.qty_locs) for p in parts),
        interferences=[dict(a=a, b=b, vol_mm3=round(v, 1)) for a, b, v in hits],
        note="modelled mass excludes wiring, most fasteners, env. sensor boards and payload; see mass budget in config/rover.yaml",
    )
    (EXPORT / "cad_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


if __name__ == "__main__":
    s = export_all()
    print(json.dumps(s, indent=2))
    sys.exit(0)
