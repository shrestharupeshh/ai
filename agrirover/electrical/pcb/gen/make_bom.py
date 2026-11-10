#!/usr/bin/env python3
"""Write fab/arc1_bom.csv (grouped) from design.py."""
import csv, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import design
out = Path(__file__).resolve().parents[1] / "fab" / "arc1_bom.csv"
groups = {}
for ref, lib, val, fp, pins, mpn, mfr, desc in design.COMPONENTS:
    if ref.startswith("#") or ref.startswith("H") or ref == "A2":
        continue
    groups.setdefault((val, fp, mpn, mfr, desc), []).append(ref)
with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["Item", "Qty", "References", "Value", "Footprint", "Manufacturer", "MPN", "Description"])
    for i, ((val, fp, mpn, mfr, desc), refs) in enumerate(sorted(groups.items(), key=lambda kv: kv[1][0]), 1):
        w.writerow([i, len(refs), " ".join(refs), val, fp.split(":")[1], mfr, mpn, desc])
print("wrote", out, len(groups), "lines")
