#!/usr/bin/python3.12
"""Step 1 of the board build (runs under KiCad's Python, needs `pcbnew`).

Creates the outline, loads every footprint from the stock KiCad libraries,
places it per PLACEMENT, assigns nets from design.py, and writes
  build/arc1_placed.kicad_pcb
  build/pads.json            (pad geometry for the router)
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pcbnew

sys.path.insert(0, str(Path(__file__).parent))
import design  # noqa: E402

PCB = Path(__file__).resolve().parents[1]
BUILD = PCB / "build"
FPDIR = "/usr/share/kicad/footprints"
OX, OY = 100.0, 60.0          # board origin on the KiCad sheet (mm)
BW, BH = 100.0, 82.0          # board size (mm)

# ref: (x, y, rotation_deg)  -- board coordinates, origin top-left, y down
PLACEMENT = {
    "H1": (4, 4, 0), "H2": (96, 4, 0), "H3": (4, 78, 0), "H4": (96, 78, 0),
    # battery in -> fuse -> reverse FET -> shunt -> VBAT (left to right along the top edge)
    "J1": (11, 12, 270), "F1": (19, 9, 0), "Q1": (39, 10, 180), "R1": (36, 16.5, 0), "D1": (42.5, 16.5, 0),
    "R2": (52, 10, 0), "U2": (52, 18.5, 0), "C1": (58.5, 19, 90),
    "D2": (60, 10, 90), "C2": (64.5, 10, 90),
    # fused outputs (top-right, connectors on the right edge)
    "F2": (70, 3.5, 0), "J6": (89, 17, 90), "F3": (70, 11.5, 0), "J7": (89, 30.5, 90),
    # motor rail switch
    "Q2": (72, 24, 0), "R3": (62, 19.5, 0), "C3": (62, 22.5, 0), "D3": (62, 25.5, 0), "R4": (62, 28.5, 0),
    "J5": (89, 44, 90), "C5": (77, 37, 0), "C6": (70, 31, 0),
    "R8": (70, 34.5, 0), "R9": (70, 38, 0), "C7": (70, 41.5, 0), "D4": (79, 45.5, 0), "R10": (72, 45.5, 0),
    # logic supplies
    "U3": (10.5, 28.5, 0), "C8": (21.5, 27.5, 90), "U4": (28, 28.5, 90), "C9": (27.5, 37, 0), "C10": (21, 34.5, 0),
    "D5": (10.5, 35.5, 0), "R11": (15.5, 37.5, 0),
    # MCU sockets (USB end up)
    "A1": (38, 27.5, 0), "A2": (53.24, 27.5, 0),
    # encoders (left edge)
    "J2": (6, 42, 270), "R12": (14, 43, 0), "R13": (14, 46.5, 0), "C11": (20, 43, 0), "C12": (20, 46.5, 0),
    "J3": (6, 56, 270), "R14": (14, 57, 0), "R15": (14, 60.5, 0), "C13": (20, 57, 0), "C14": (20, 60.5, 0),
    # E-stop + bumper (bottom edge, left)
    "J14": (12, 77.5, 0), "Q3": (16, 68, 0), "R5": (22, 67, 0), "R6": (22, 70.5, 0), "R7": (10.5, 71.5, 0), "C4": (16, 72.3, 0),
    "J15": (26, 77.5, 0), "R24": (30.5, 66, 90), "C17": (30.5, 71.5, 90),
    # motor driver interface + filters (right of MCU)
    "J4": (61, 53, 90), "R16": (64, 58, 0), "R17": (70, 58, 0), "R18": (64, 61.5, 0), "C15": (70, 61.5, 0),
    "R19": (64, 65, 0), "C16": (70, 65, 0),
    # I2C
    "J8": (93, 55, 90), "J9": (93, 64, 90), "J10": (84, 52, 0), "R20": (79, 51, 0), "R21": (79, 54.5, 0),
    # UART / SPI / debug (bottom edge, right)
    "J11": (80, 77.5, 0), "R22": (80, 68, 0), "R23": (80, 71.5, 0), "J12": (72, 70, 0), "J13": (62, 71.5, 0),
}


def mm(v):
    return pcbnew.FromMM(float(v))


def main():
    BUILD.mkdir(exist_ok=True)
    board = pcbnew.BOARD()
    ds = board.GetDesignSettings()
    ds.SetCopperLayerCount(2)
    ds.m_TrackMinWidth = mm(0.2)
    ds.m_MinClearance = mm(0.2)
    ds.m_ViasMinSize = mm(0.6)
    ds.m_MinThroughDrill = mm(0.3)
    ds.m_CopperEdgeClearance = mm(0.4)
    # outline with 2 mm corner radius approximated by chamfers -> simple rectangle keeps fab drawings trivial
    pts = [(0, 0), (BW, 0), (BW, BH), (0, BH)]
    for i in range(4):
        seg = pcbnew.PCB_SHAPE(board)
        seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
        seg.SetStart(pcbnew.VECTOR2I(mm(OX + pts[i][0]), mm(OY + pts[i][1])))
        seg.SetEnd(pcbnew.VECTOR2I(mm(OX + pts[(i + 1) % 4][0]), mm(OY + pts[(i + 1) % 4][1])))
        seg.SetLayer(pcbnew.Edge_Cuts)
        seg.SetWidth(mm(0.1))
        board.Add(seg)
    nets = {}
    for name in design.nets():
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        nets[name] = ni
    pads_out, boxes = [], []
    for ref, lib_id, val, fp_id, pins, *_ in design.COMPONENTS:
        if not fp_id:
            continue
        lib, name = fp_id.split(":")
        fp = pcbnew.FootprintLoad(f"{FPDIR}/{lib}.pretty", name)
        if fp is None:
            raise SystemExit(f"footprint not found: {fp_id}")
        fp.SetReference(ref)
        fp.SetValue(val)
        fp.SetFPID(pcbnew.LIB_ID(lib, name))
        x, y, rot = PLACEMENT[ref]
        board.Add(fp)
        fp.SetPosition(pcbnew.VECTOR2I(mm(OX + x), mm(OY + y)))
        fp.SetOrientationDegrees(rot)
        fp.Reference().SetTextSize(pcbnew.VECTOR2I(mm(0.8), mm(0.8)))
        fp.Reference().SetTextThickness(mm(0.12))
        fp.Value().SetVisible(False)
        if ref.startswith("H"):
            fp.Reference().SetVisible(False)
        padnums = {p.GetNumber() for p in fp.Pads() if p.GetNumber()}
        for pin in pins:
            if pin not in padnums:
                raise SystemExit(f"{ref}: design pin {pin} has no pad in {fp_id} (pads {sorted(padnums)})")
        for pad in fp.Pads():
            num = pad.GetNumber()
            net = pins.get(num, "NC") if num else "NC"
            if net != "NC":
                pad.SetNet(nets[net])
            bb = pad.GetBoundingBox()
            ls = pad.GetLayerSet()
            pads_out.append(dict(
                ref=ref, num=num, net=net if net != "NC" else "",
                x=pcbnew.ToMM(pad.GetPosition().x) - OX, y=pcbnew.ToMM(pad.GetPosition().y) - OY,
                w=pcbnew.ToMM(bb.GetWidth()), h=pcbnew.ToMM(bb.GetHeight()),
                front=bool(ls.Contains(pcbnew.F_Cu)), back=bool(ls.Contains(pcbnew.B_Cu)),
                drill=pcbnew.ToMM(pad.GetDrillSize().x), npth=pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH))
        bb = fp.GetBoundingBox(False, False)
        boxes.append((ref, pcbnew.ToMM(bb.GetLeft()) - OX, pcbnew.ToMM(bb.GetTop()) - OY,
                      pcbnew.ToMM(bb.GetRight()) - OX, pcbnew.ToMM(bb.GetBottom()) - OY))
    # overlap / outline report (body boxes)
    problems = []
    for i, a in enumerate(boxes):
        if a[1] < 0 or a[2] < 0 or a[3] > BW or a[4] > BH:
            problems.append(f"{a[0]} extends past the board edge: {[round(v, 1) for v in a[1:]]}")
        for b in boxes[i + 1:]:
            if {a[0], b[0]} == {"A1", "A2"}:
                continue
            if a[1] < b[3] and b[1] < a[3] and a[2] < b[4] and b[2] < a[4]:
                problems.append(f"{a[0]} overlaps {b[0]}")
    (BUILD / "pads.json").write_text(json.dumps(dict(board=[BW, BH], origin=[OX, OY], pads=pads_out, boxes=boxes), indent=1))
    pcbnew.SaveBoard(str(BUILD / "arc1_placed.kicad_pcb"), board)
    print(f"placed {len(boxes)} footprints, {len(pads_out)} pads")
    for p in problems:
        print("PLACEMENT:", p)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
