#!/usr/bin/env python3
"""Research-extension experiments (simulation studies).

    python research/experiments.py [exp1 exp2 exp3]

Everything here is a MODEL STUDY on the simulator and synthetic images. The outputs are hypotheses and
experiment designs to be tested on a physical rover with real plants - not findings about real crops.
Results: research/results/*.csv, *.png, summary.json
"""
from __future__ import annotations

import csv
import json
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "software"))
OUT = ROOT / "research" / "results"

from agrirover.mission import route  # noqa: E402
from agrirover.runtime import make_demo  # noqa: E402
from agrirover.sim import camera_sim as cs  # noqa: E402
from agrirover.sim.world import World  # noqa: E402
from agrirover.vision import pipeline  # noqa: E402


def _plot(fn):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fn(plt)
    except ImportError:
        pass


def detection_curve(kind: str, severities, n: int = 24, shift_px: float = 0.0, seed: int = 0) -> list[float]:
    """Empirical P(flag | severity) of the screening pipeline on synthetic views (one affected plant per view)."""
    rng = np.random.default_rng(seed)
    out = []
    for sev in severities:
        hits = 0
        for k in range(n):
            plants = [cs.PlantTruth(f"P{i}", 22 + rng.normal(0, 1), seed=int(rng.integers(1e6))) for i in range(8)]
            j = int(rng.integers(8))
            plants[j].health, plants[j].severity = kind, float(sev)
            sx, sy = rng.normal(0, shift_px, 2) if shift_px > 0 else (0.0, 0.0)
            img, _ = cs.render_view(plants, seed=int(rng.integers(1e6)), gain=float(rng.uniform(0.85, 1.1)), shift_px=(float(sx), float(sy)))
            slots = [(p.pid, *cs.slot_px(*divmod(i, 4))) for i, p in enumerate(plants)]
            res = pipeline.analyse(img, slots, cs.PX_PER_M, cs.GRAY_PATCH)
            hits += res["plants"][j]["status"] in ("attention", "missing")
        out.append(hits / n)
    return out


def exp1():
    """RQ1: how does inspection interval affect the delay between stress onset and first detection?"""
    sev = np.linspace(0, 1, 11)
    curves = {k: detection_curve(k, sev) for k in ("chlorosis", "tipburn", "purpling", "stunted")}
    rng = np.random.default_rng(1)
    tau_days = {"chlorosis": 4.0, "tipburn": 3.0, "purpling": 6.0, "stunted": 8.0}      # assumed symptom development time constants
    intervals = [0.5, 1, 2, 3, 5, 7]
    rows = []
    world = World()
    e_mission = route.route_metrics(route.boustrophedon(world), world)["energy_wh"]
    for kind, curve in curves.items():
        for T in intervals:
            lat = []
            for _ in range(4000):
                onset = rng.uniform(0, T)                      # onset uniformly within an inspection interval
                t, found = T - onset, None
                while t < 60:
                    s = 1 - math.exp(-t / tau_days[kind])      # severity grows towards 1
                    if rng.random() < np.interp(s, sev, curve):
                        found = t
                        break
                    t += T
                lat.append(found if found is not None else 60.0)
            lat = np.array(lat)
            rows.append(dict(symptom=kind, interval_days=T, latency_median_d=round(float(np.median(lat)), 2),
                             latency_p90_d=round(float(np.percentile(lat, 90)), 2),
                             severity_at_detection_median=round(float(np.median(1 - np.exp(-lat / tau_days[kind]))), 2),
                             robot_energy_wh_per_day=round(e_mission / T, 2)))
    with open(OUT / "exp1_inspection_frequency.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with open(OUT / "exp1_detection_curves.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["severity"] + list(curves))
        for i, s in enumerate(sev):
            w.writerow([round(float(s), 2)] + [curves[k][i] for k in curves])

    def draw(plt):
        fig, ax = plt.subplots(1, 2, figsize=(11, 4))
        for k, c in curves.items():
            ax[0].plot(sev, c, marker="o", label=k)
            r = [x for x in rows if x["symptom"] == k]
            ax[1].plot([x["interval_days"] for x in r], [x["latency_median_d"] for x in r], marker="o", label=k)
        ax[0].set(xlabel="synthetic symptom severity", ylabel="P(flag)", title="Screening pipeline on SYNTHETIC images")
        ax[1].set(xlabel="inspection interval [days]", ylabel="median onset-to-detection delay [days]", title="Model study (assumed symptom dynamics)")
        ax[0].legend()
        ax[0].grid(alpha=.3)
        ax[1].grid(alpha=.3)
        fig.savefig(OUT / "exp1_inspection_frequency.png", dpi=110, bbox_inches="tight")
    _plot(draw)
    return dict(rows=len(rows), example=[r for r in rows if r["symptom"] == "chlorosis"])


def exp2():
    """RQ5/RQ6: how do route order and energy relate, and does the analytic route model predict the simulator?"""
    world = World()
    orders = {"naive": route.naive_order(world), "serpentine": route.boustrophedon(world), "optimal": route.best_aisle_order(world)[0]}
    rows = []
    for name, order in orders.items():
        pred = route.route_metrics(order, world)
        r = make_demo(Path(tempfile.mkdtemp()) / "a.db", seed=5, save_images=False)
        r.start_mission(name, [next(s for s in r.world.stations if s.sid == o.sid) for o in order])
        r.run(3600)
        s = r.sim.s
        rows.append(dict(route=name, state=r.state, pred_distance_m=pred["distance_m"], sim_distance_m=round(s.distance, 1),
                         pred_duration_s=pred["duration_s"], sim_duration_s=round(s.t, 0), pred_energy_wh=pred["energy_wh"],
                         sim_energy_wh=round(s.energy_wh, 2), sim_drive_share=round(1 - (len(order) * 4.5) / s.t, 2)))
    with open(OUT / "exp2_route_energy.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    return rows


def exp3():
    """RQ3: sensitivity of plant association and flags to camera positioning (robot stop) error."""
    sev = [0.8]
    rows = []
    for sigma_mm in (0, 10, 20, 40, 60, 90):
        px = sigma_mm / 1000 * cs.PX_PER_M
        p = np.mean([detection_curve(k, sev, n=40, shift_px=px, seed=3)[0] for k in ("chlorosis", "tipburn", "purpling")])
        # false flags on all-healthy views under the same positioning error
        rng = np.random.default_rng(7)
        fp = tot = 0
        for _ in range(40):
            plants = [cs.PlantTruth(f"P{i}", 22, seed=int(rng.integers(1e6))) for i in range(8)]
            sx, sy = rng.normal(0, px, 2) if px > 0 else (0.0, 0.0)
            img, _ = cs.render_view(plants, seed=int(rng.integers(1e6)), shift_px=(float(sx), float(sy)))
            res = pipeline.analyse(img, [(q.pid, *cs.slot_px(*divmod(i, 4))) for i, q in enumerate(plants)], cs.PX_PER_M, cs.GRAY_PATCH)
            fp += sum(o["status"] != "ok" for o in res["plants"])
            tot += 8
        rows.append(dict(stop_error_sigma_mm=sigma_mm, flag_recall_sev0p8=round(float(p), 3), false_flag_rate_healthy=round(fp / tot, 3)))
    with open(OUT / "exp3_positioning_sensitivity.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    return rows


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    which = sys.argv[1:] or ["exp1", "exp2", "exp3"]
    summary = {"note": "Simulation / synthetic-image model studies. Not experimental evidence about real plants or a physical robot."}
    for name in which:
        summary[name] = globals()[name]()
        print(name, json.dumps(summary[name], indent=1)[:1500])
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
