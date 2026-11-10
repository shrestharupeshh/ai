#!/usr/bin/env python3
"""Render reports/arc1_board_preview.png from the generated Gerbers-independent data (pads + routes)."""
import json
from pathlib import Path
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
P = Path(__file__).resolve().parents[1]
d = json.loads((P / "build/pads.json").read_text()); r = json.loads((P / "build/routes.json").read_text())
fig, axs = plt.subplots(1, 2, figsize=(20, 8.6))
for layer, ax, name, col in ((0, axs[0], "F.Cu (top)", "#b03020"), (1, axs[1], "B.Cu (bottom, viewed from top)", "#2040a0")):
    ax.add_patch(Rectangle((0, 0), *d["board"], fc="#0f3d1e", ec="k", lw=1.5))
    for t in r["tracks"]:
        if t["layer"] == layer:
            ax.plot([t["x0"], t["x1"]], [t["y0"], t["y1"]], color=col, lw=t["w"] * 5.3, solid_capstyle="round")
    for p in d["pads"]:
        if (p["front"] if layer == 0 else p["back"]) or p["drill"] > 0:
            ax.add_patch(Rectangle((p["x"] - p["w"] / 2, p["y"] - p["h"] / 2), p["w"], p["h"], fc="#d9b44a", lw=0))
            if p["drill"] > 0:
                ax.add_patch(Circle((p["x"], p["y"]), p["drill"] / 2, fc="k", lw=0))
    for v in r["vias"]:
        ax.add_patch(Circle((v["x"], v["y"]), 0.4, fc="#d9b44a", lw=0)); ax.add_patch(Circle((v["x"], v["y"]), 0.2, fc="k", lw=0))
    if layer == 0:
        for ref, x0, y0, x1, y1 in d["boxes"]:
            ax.text((x0 + x1) / 2, (y0 + y1) / 2, ref, fontsize=5.5, ha="center", va="center", color="w")
    ax.set_xlim(-2, d["board"][0] + 2); ax.set_ylim(d["board"][1] + 2, -2); ax.set_aspect("equal"); ax.set_title(name + " — GND pour not drawn"); ax.axis("off")
fig.savefig(P / "reports/arc1_board_preview.png", dpi=70, bbox_inches="tight")
