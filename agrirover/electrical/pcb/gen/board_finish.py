#!/usr/bin/python3.12
"""Step 3 of the board build (KiCad Python): add routed tracks and vias,
pour GND on both layers, run KiCad's DRC, save arc1.kicad_pcb.
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import pcbnew

PCB = Path(__file__).resolve().parents[1]
BUILD = PCB / "build"


def mm(v):
    return pcbnew.FromMM(float(v))


def main() -> int:
    pads = json.loads((BUILD / "pads.json").read_text())
    routes = json.loads((BUILD / "routes.json").read_text())
    OX, OY = pads["origin"]
    BW, BH = pads["board"]
    board = pcbnew.LoadBoard(str(BUILD / "arc1_placed.kicad_pcb"))
    P = lambda x, y: pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))  # noqa: E731
    for t in routes["tracks"]:
        if abs(t["x0"] - t["x1"]) < 1e-6 and abs(t["y0"] - t["y1"]) < 1e-6:
            continue
        tr = pcbnew.PCB_TRACK(board)
        tr.SetStart(P(t["x0"], t["y0"]))
        tr.SetEnd(P(t["x1"], t["y1"]))
        tr.SetWidth(mm(t["w"]))
        tr.SetLayer(pcbnew.F_Cu if t["layer"] == 0 else pcbnew.B_Cu)
        tr.SetNet(board.FindNet(t["net"]))
        board.Add(tr)
    for v in routes["vias"]:
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(P(v["x"], v["y"]))
        via.SetViaType(pcbnew.VIATYPE_THROUGH)
        via.SetWidth(mm(routes["via_d"]))
        via.SetDrill(mm(routes["via_drill"]))
        via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        via.SetNet(board.FindNet(v["net"]))
        board.Add(via)
    gnd = board.FindNet("GND")
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        z = pcbnew.ZONE(board)
        z.SetLayer(layer)
        z.SetNet(gnd)
        z.SetZoneName("GND_F" if layer == pcbnew.F_Cu else "GND_B")
        ol = z.Outline()
        ol.NewOutline()
        for x, y in ((0.5, 0.5), (BW - 0.5, 0.5), (BW - 0.5, BH - 0.5), (0.5, BH - 0.5)):
            ol.Append(mm(OX + x), mm(OY + y))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)       # battery return current flows through these joints
        z.SetLocalClearance(mm(0.3))
        z.SetMinThickness(mm(0.25))
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        board.Add(z)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())

    def silk(text, x, y, size=1.0, layer=pcbnew.F_SilkS):
        t = pcbnew.PCB_TEXT(board)
        t.SetText(text)
        t.SetPosition(P(x, y))
        t.SetLayer(layer)
        t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
        t.SetTextThickness(mm(size * 0.15))
        board.Add(t)

    silk("AgriRover ARC-1 rev A", 50, 80.6, 1.0)
    silk("BAT+", 4.2, 24.2, 0.8)
    silk("USB end", 45.6, 25.3, 0.8)
    silk("VERIFY BLACKPILL PINOUT BEFORE FITTING", 50, 3.0, 1.0, pcbnew.B_SilkS)
    tb = board.GetTitleBlock()
    tb.SetTitle("AgriRover GH-1 control & power carrier (ARC-1)")
    tb.SetRevision("A")
    tb.SetDate("2026-11-10")
    out = PCB / "arc1.kicad_pcb"
    pcbnew.SaveBoard(str(out), board)
    board = pcbnew.LoadBoard(str(out))
    rpt = PCB / "reports" / "drc_report.txt"
    pcbnew.WriteDRCReport(board, str(rpt), pcbnew.EDA_UNITS_MILLIMETRES, True)
    txt = rpt.read_text()
    counts = {k: int(v) for v, k in re.findall(r"\*\* Found (\d+) ([A-Za-z ]+?) \*\*", txt)}
    kinds = {}
    for m in re.finditer(r"^\[(\w+)\]: ", txt, re.M):
        kinds[m.group(1)] = kinds.get(m.group(1), 0) + 1
    summary = dict(drc=counts, by_type=kinds, tracks=len(routes["tracks"]), vias=len(routes["vias"]),
                   track_len_mm=routes["track_len_mm"], unrouted=routes["failed"], router_notes=routes["log"])
    (PCB / "reports" / "drc_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
