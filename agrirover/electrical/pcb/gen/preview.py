#!/usr/bin/env python3
"""Quick matplotlib preview of placement / routing (build/pads.json [+ routes.json])."""
import json, sys
from pathlib import Path
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
B = Path(__file__).resolve().parents[1] / "build"
d = json.loads((B / "pads.json").read_text())
fig, ax = plt.subplots(figsize=(15, 12.5))
ax.add_patch(Rectangle((0, 0), *d["board"], fill=False, lw=1.5))
for ref, x0, y0, x1, y1 in d["boxes"]:
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="0.6", lw=0.5))
    ax.text((x0 + x1) / 2, y0 - 0.2, ref, fontsize=6, ha="center", color="0.3")
for p in d["pads"]:
    c = "r" if p["front"] and not p["back"] else "goldenrod"
    ax.add_patch(Rectangle((p["x"] - p["w"] / 2, p["y"] - p["h"] / 2), p["w"], p["h"], fc=c, alpha=0.6, lw=0))
    if p["net"] and len(sys.argv) > 1 and sys.argv[1] == "nets":
        ax.text(p["x"], p["y"], p["net"], fontsize=2.5, ha="center", va="center")
rp = B / "routes.json"
if rp.exists():
    r = json.loads(rp.read_text())
    for t in r["tracks"]:
        ax.plot([t["x0"], t["x1"]], [t["y0"], t["y1"]], color="r" if t["layer"] == 0 else "b", lw=max(0.4, t["w"] * 4.2), alpha=0.55, solid_capstyle="round")
    for v in r["vias"]:
        ax.plot(v["x"], v["y"], "o", color="k", ms=2.5)
ax.set_xlim(-4, d["board"][0] + 4); ax.set_ylim(d["board"][1] + 4, -4); ax.set_aspect("equal"); ax.grid(lw=0.2)
ax.set_xticks(range(0, 101, 5)); ax.set_yticks(range(0, 86, 5))
fig.savefig(B / "preview.png", dpi=75, bbox_inches="tight")
