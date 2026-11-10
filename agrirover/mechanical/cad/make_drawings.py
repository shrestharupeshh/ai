#!/usr/bin/env python3
"""Generate 2D drawings (PDF + PNG) from the CadQuery model using OpenCascade
hidden-line removal. Dimensions are computed from the model, not typed in.

Outputs to mechanical/export/drawings/.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepLib import BRepLib
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

sys.path.insert(0, str(Path(__file__).parent))
import rover_cad as rc  # noqa: E402

OUT = rc.EXPORT / "drawings"
VIEWS = {  # name: (view direction (towards viewer), x-axis of the sheet)
    "top": ((0, 0, 1), (1, 0, 0)),
    "side": ((0, -1, 0), (1, 0, 0)),     # looking from the right-hand side (+x to the right)
    "front": ((1, 0, 0), (0, 1, 0)),     # looking at the nose
    "iso": ((1, -1.2, 0.8), (1.2, 1, 0)),
}


def project(shape, view: str, hidden=False):
    d, x = VIEWS[view]
    hlr = HLRBRep_Algo()
    hlr.Add(shape)
    hlr.Projector(HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(*d), gp_Dir(*x))))
    hlr.Update()
    hlr.Hide()
    hs = HLRBRep_HLRToShape(hlr)
    groups = [hs.VCompound(), hs.OutLineVCompound(), hs.Rg1LineVCompound()]
    if hidden:
        groups += [hs.HCompound()]
    lines = []
    for gi, comp in enumerate(groups):
        if comp.IsNull():
            continue
        BRepLib.BuildCurves3d_s(comp)
        ex = TopExp_Explorer(comp, TopAbs_EDGE)
        while ex.More():
            c = BRepAdaptor_Curve(TopoDS.Edge_s(ex.Current()))
            disc = GCPnts_QuasiUniformDeflection(c, 0.2)
            if disc.IsDone() and disc.NbPoints() > 1:
                pts = np.array([[disc.Value(i).X(), disc.Value(i).Y()] for i in range(1, disc.NbPoints() + 1)])
                lines.append((pts, gi == 3))
            ex.Next()
    return lines


def draw(ax, lines, ox=0.0, oy=0.0, lw=0.5):
    for pts, hid in lines:
        ax.plot(pts[:, 0] + ox, pts[:, 1] + oy, color="0.55" if hid else "k", lw=lw * (0.5 if hid else 1), ls="--" if hid else "-")


def bbox(lines):
    allp = np.vstack([p for p, _ in lines])
    return allp[:, 0].min(), allp[:, 0].max(), allp[:, 1].min(), allp[:, 1].max()


def dim_h(ax, x0, x1, y, text=None, off=0.0, fs=6):
    ax.annotate("", (x0, y + off), (x1, y + off), arrowprops=dict(arrowstyle="<->", lw=0.4, color="b"))
    for x in (x0, x1):
        ax.plot([x, x], [y, y + off * 1.08], color="b", lw=0.25)
    ax.text((x0 + x1) / 2, y + off, text or f"{abs(x1-x0):.0f}", ha="center", va="bottom", fontsize=fs, color="b")


def dim_v(ax, y0, y1, x, text=None, off=0.0, fs=6):
    ax.annotate("", (x + off, y0), (x + off, y1), arrowprops=dict(arrowstyle="<->", lw=0.4, color="b"))
    for y in (y0, y1):
        ax.plot([x, x + off * 1.08], [y, y], color="b", lw=0.25)
    ax.text(x + off, (y0 + y1) / 2, text or f"{abs(y1-y0):.0f}", ha="right" if off < 0 else "left", va="center", fontsize=fs, color="b", rotation=90)


def sheet(title: str, number: str, size=(16.54, 11.69)):
    fig = plt.figure(figsize=size)
    ax = fig.add_axes([0.02, 0.02, 0.96, 0.96])
    ax.set_aspect("equal")
    ax.axis("off")
    fig.text(0.98, 0.012, f"AgriRover GH-1  |  {title}  |  DWG {number}  |  rev {rc.CFG['meta']['revision']}  |  "
             f"{rc.CFG['meta']['date']}  |  units mm  |  third-angle  |  general tol. ±0.2 (cut), ±0.5 (printed)  |  "
             "generated from mechanical/cad/rover_cad.py", ha="right", fontsize=6.5,
             bbox=dict(boxstyle="square", fc="w", ec="k", lw=0.6))
    return fig, ax


def save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / f"{name}.pdf")
    fig.savefig(OUT / f"{name}.png", dpi=110)
    plt.close(fig)


def general_arrangement(parts):
    asm = rc.assemble(parts).toCompound().wrapped
    fig, ax = sheet("General arrangement", "GA-001")
    side, front, top, iso = (project(asm, v) for v in ("side", "front", "top", "iso"))
    draw(ax, side, 0, 0)
    sx0, sx1, sy0, sy1 = bbox(side)
    dim_h(ax, sx0, sx1, sy0, off=-45)
    dim_v(ax, sy0, sy1, sx0, off=-40)
    dim_v(ax, 0, rc.Z_LO, sx1, text=f"{rc.Z_LO:.0f} (plate underside)", off=30)
    dim_v(ax, 0, rc.Z_UP_TOP, sx1, text=f"{rc.Z_UP_TOP:.0f} (deck top)", off=75)
    dim_h(ax, -rc.CASTER_X, rc.CASTER_X, sy0, text=f"{2*rc.CASTER_X:.0f} caster base", off=-95)
    ax.text((sx0 + sx1) / 2, sy1 + 25, "SIDE VIEW (right)", ha="center", fontsize=8)
    ox = 650
    draw(ax, front, ox, 0)
    fx0, fx1, fy0, fy1 = bbox(front)
    dim_h(ax, fx0 + ox, fx1 + ox, fy0, off=-45)
    dim_h(ax, -rc.TRACK / 2 + ox, rc.TRACK / 2 + ox, fy0, text=f"{rc.TRACK:.0f} track", off=-95)
    dim_v(ax, 0, rc.CAM_Z, fx1 + ox, text=f"{rc.CAM_Z:.0f} camera axis", off=40)
    dim_v(ax, 0, rc.AXLE_Z - 18.5, fx0 + ox, text=f"{rc.AXLE_Z-18.5:.1f} ground clearance", off=-40)
    ax.text(ox, fy1 + 25, "FRONT VIEW", ha="center", fontsize=8)
    ox2, oy2 = 1250, 250
    draw(ax, top, ox2, oy2)
    tx0, tx1, ty0, ty1 = bbox(top)
    dim_h(ax, tx0 + ox2, tx1 + ox2, ty0 + oy2, off=-45)
    dim_v(ax, ty0 + oy2, ty1 + oy2, tx0 + ox2, off=-40)
    r = rc.L / 2 + 0  # swept circle from calcs
    import json
    sw = json.loads((rc.ROOT / "mechanical/calcs/mechanical_results.json").read_text())["swept_radius_mm"]
    th = np.linspace(0, 2 * np.pi, 200)
    ax.plot(ox2 + sw * np.cos(th), oy2 + sw * np.sin(th), color="r", lw=0.4, ls="-.")
    ax.text(ox2, oy2 + sw + 12, f"swept circle R{sw:.0f} (turn in place)", ha="center", fontsize=6, color="r")
    ax.text(ox2, ty1 + oy2 + 60, "TOP VIEW", ha="center", fontsize=8)
    draw(ax, iso, 1250, 850, lw=0.35)
    ax.text(1250, 720, "ISOMETRIC (not to scale reference)", ha="center", fontsize=8)
    save(fig, "GA-001_general_arrangement")


def plate_drawing(name, number, shape, thk, notes):
    fig, ax = sheet(f"{name} — {notes}", number)
    top = project(shape.val().wrapped, "top")
    draw(ax, top)
    x0, x1, y0, y1 = bbox(top)
    dim_h(ax, x0, x1, y0, off=-22)
    dim_v(ax, y0, y1, x0, off=-22)
    ax.plot([x0 - 10, x1 + 10], [0, 0], color="g", lw=0.3, ls="-.")
    ax.plot([0, 0], [y0 - 10, y1 + 10], color="g", lw=0.3, ls="-.")
    ax.text(x1 + 12, 0, "X (fwd)", fontsize=6, color="g", va="center")
    ax.text(0, y1 + 12, "Y (left)", fontsize=6, color="g", ha="center")
    # hole table from circular edges on the top face
    holes = {}
    for e in shape.faces(">Z").edges("%CIRCLE").vals():
        c = e.Center()
        holes[(round(c.x, 2), round(c.y, 2))] = round(e.radius() * 2, 2)
    by_d = {}
    for (x, y), d in holes.items():
        by_d.setdefault(d, []).append((x, y))
    tx, ty = x1 + 45, y1
    ax.text(tx, ty, "HOLE TABLE (datum: plate centre = axle centreline)\nTag  Ø [mm]  X      Y", fontsize=6, family="monospace", va="top")
    row, tag_i = 0, 0
    for d in sorted(by_d):
        tag = chr(ord("A") + tag_i)
        tag_i += 1
        for i, (x, y) in enumerate(sorted(by_d[d])):
            ax.text(x + d / 2 + 1, y + d / 2 + 1, f"{tag}{i+1}", fontsize=4.2, color="b")
            ax.text(tx, ty - 22 - row * 8.2, f"{tag}{i+1:<3} {d:>5.1f}  {x:>7.1f} {y:>7.1f}", fontsize=5, family="monospace", va="top")
            row += 1
    ax.text(x0, y0 - 60, f"Material: {notes}. Thickness {thk} mm. Corner chamfer {rc.CH} x 45°. Deburr all edges; "
            "break sharp corners R0.5. Flat pattern supplied as DXF (mechanical/export/dxf).", fontsize=6.5)
    ax.set_xlim(x0 - 60, x1 + 260)
    save(fig, f"{number}_{name}")


def chassis_detail(parts):
    keep = [p for p in parts if p.name not in ("mast", "camera_mount", "camera", "sensor_pod", "skirt_side", "skirt_end", "skirt_corner")]
    asm = rc.assemble(keep).toCompound().wrapped
    fig, ax = sheet("Chassis assembly (skirts and mast removed)", "ASM-002")
    side = project(asm, "side", hidden=True)
    top = project(asm, "top", hidden=True)
    draw(ax, side, 0, 0, lw=0.6)
    draw(ax, top, 0, 420, lw=0.6)
    dim_v(ax, 0, rc.AXLE_Z, -rc.L / 2 - 14, text=f"{rc.AXLE_Z:.0f} axle", off=-25)
    dim_v(ax, rc.Z_LO_TOP, rc.Z_UP, rc.L / 2, text=f"{rc.POST:.0f} deck spacing", off=30)
    dim_h(ax, -rc.L / 2, rc.L / 2, 420 - rc.W / 2, off=-40)
    balloons = {"1 lower plate": (-120, rc.Z_LO + 2), "2 upper plate": (-60, rc.Z_UP + 1.5), "3 deck post (4x)": (rc.L / 2 - 32, 130),
                "4 gearmotor (2x)": (0, rc.AXLE_Z - 7), "5 wheel (2x)": (30, 20), "6 caster (2x)": (rc.CASTER_X, 30),
                "7 battery": (-45, 120), "8 electronics tray": (120, rc.Z_LO_TOP + 16), "9 lidar": (rc.LIDAR_X, rc.Z_UP_TOP + 30),
                "10 bumper": (rc.L / 2 + 14, rc.Z_LO + 15), "11 E-stop": (-195, rc.Z_UP_TOP + 20)}
    for i, (t, (x, y)) in enumerate(balloons.items()):
        ax.annotate(t, (x, y), (x * 1.15 + (60 if x >= 0 else -60), 250 + (i % 4) * 14), fontsize=6,
                    arrowprops=dict(arrowstyle="-", lw=0.3), ha="center")
    save(fig, "ASM-002_chassis_assembly")


def main():
    parts = rc.build_parts()
    general_arrangement(parts)
    chassis_detail(parts)
    by = {p.name: p for p in parts}
    plate_drawing("lower_plate", "PRT-101", by["lower_plate"].shape, rc.T_LO, "Al 5052-H32 sheet, laser cut")
    plate_drawing("upper_plate", "PRT-102", by["upper_plate"].shape, rc.T_UP, "Al 5052-H32 sheet, laser cut")
    plate_drawing("electronics_tray", "PRT-103", by["electronics_tray"].shape, 3.0, "Al 5052-H32 sheet, laser cut")
    print("drawings written to", OUT)


if __name__ == "__main__":
    main()
