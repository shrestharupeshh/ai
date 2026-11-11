"""Run the reference inspection mission in the headless simulator.

    python -m agrirover.sim.run --out ../sim/results/reference [--order optimal|serpentine|naive]

Writes: agrirover.db (+ images/), summary.json, trajectory.png, map.png/.yaml (occupancy grid built from the lidar).
"""
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np

from ..mission import route
from ..runtime import make_demo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data/runtime/sim")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--order", default="serpentine", choices=["serpentine", "naive", "optimal"])
    ap.add_argument("--no-images", action="store_true")
    a = ap.parse_args()
    out = Path(a.out)
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    r = make_demo(out / "agrirover.db", seed=a.seed, build_map=True, save_images=not a.no_images)
    order = {"serpentine": route.boustrophedon, "naive": route.naive_order, "optimal": lambda w: route.best_aisle_order(w)[0]}[a.order](r.world)
    predicted = route.route_metrics(order, r.world)
    r.start_mission(f"reference ({a.order})", order)
    traj = []
    while r.sim.s.t < 3600 and r.state not in ("DONE", "ABORTED"):
        r.step()
        if r._tick % 10 == 0:
            traj.append((r.sim.s.x, r.sim.s.y, *r.pose[:2]))
    s = r.sim.s
    err = np.array([[p, y] for _, p, y in r.stop_errors])
    tp = fp = fn = 0
    for c in r.capture_log:
        for o, t in zip(c["plants"], c["truth"]):
            bad, flagged = t["health"] != "healthy", o["status"] in ("attention", "missing")
            tp, fp, fn = tp + (bad and flagged), fp + ((not bad) and flagged), fn + (bad and not flagged)
    traj = np.array(traj)
    summary = dict(
        note="SIMULATION RESULT (headless 2D simulator, synthetic images). Not a physical test.",
        state=r.state, order=a.order, stations=len(r.done_stations), skipped=r.skipped, duration_s=round(s.t, 1), distance_m=round(s.distance, 2),
        energy_wh=round(s.energy_wh, 3), predicted=predicted, collisions=s.collisions, min_clearance_m=round(s.min_clearance, 3),
        stop_error_mean_m=round(float(err[:, 0].mean()), 4), stop_error_max_m=round(float(err[:, 0].max()), 4),
        stop_yaw_error_max_deg=round(float(np.degrees(err[:, 1].max())), 2),
        localisation_error_rms_m=round(float(np.sqrt(((traj[:, :2] - traj[:, 2:]) ** 2).sum(1).mean())), 4),
        plants_observed=len(r.world.plants), synthetic_truth=dict(true_positive=int(tp), false_positive=int(fp), false_negative=int(fn)),
        events=[e[1] for e in r.events if e[1] != "IMAGE_RETAKE"])
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    r.mapper.save(str(out / "map.png"))
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
        w = r.world
        fig, ax = plt.subplots(figsize=(12, 7))
        ax.add_patch(Rectangle((0, 0), w.w, w.h, fill=False, lw=2))
        for b in w.benches:
            ax.add_patch(Rectangle((b.x0, b.y0), b.x1 - b.x0, b.y1 - b.y0, fc="#cfe8cf", ec="g"))
        for b in w.obstacles:
            ax.add_patch(Rectangle((b.x0, b.y0), b.x1 - b.x0, b.y1 - b.y0, fc="0.5"))
        flagged = {x["plant_id"] for x in r.store.query("SELECT plant_id FROM observations WHERE status IN ('attention','missing')")}
        for p in w.plants.values():
            ax.plot(p["x"], p["y"], ".", color="r" if p["id"] in flagged else "g", ms=5 if p["id"] in flagged else 2)
        ax.plot(traj[:, 0], traj[:, 1], "b-", lw=0.8, label="simulated path (ground truth)")
        ax.plot([st.x for st in w.stations], [st.y for st in w.stations], "k+", ms=7, label="inspection stations")
        ax.plot(w.dock[0], w.dock[1], "ms", label="dock")
        ax.set_aspect("equal")
        ax.legend(loc="upper right", fontsize=8)
        ax.set_title(f"SIMULATION - reference mission ({a.order}): {summary['stations']} stations, {summary['distance_m']} m, "
                     f"{summary['duration_s']:.0f} s, {summary['energy_wh']:.2f} Wh; red = plants flagged")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        fig.savefig(out / "trajectory.png", dpi=110, bbox_inches="tight")
    except ImportError:
        pass
    print(json.dumps(summary, indent=2))
    return 0 if r.state == "DONE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
