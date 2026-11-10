#!/usr/bin/env python3
"""Step 2 of the board build: a small two-layer grid maze router.

Reads build/pads.json (from board_place.py), routes every net except GND
(GND is carried by copper pours on both layers; SMD GND pads get a
stitching via), and writes build/routes.json.

Method: A* on a 0.127 mm grid, 8-connected, two layers. Clearance is exact
with respect to the rasterised copper: for each connection a Euclidean
distance transform of all foreign copper gives the set of cells where a
track centre-line of the requested width (or a via) may lie.
The result is verified afterwards by KiCad's own DRC (board_finish.py).
"""
from __future__ import annotations

import heapq
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.ndimage import distance_transform_edt as edt

sys.path.insert(0, str(Path(__file__).parent))
import design  # noqa: E402

BUILD = Path(__file__).resolve().parents[1] / "build"
GRID = 0.127
CLR = 0.20           # copper-to-copper clearance (mm)
MARGIN = 0.10        # rasterisation safety margin (mm)
EDGE = 0.60          # copper to board edge (mm)
VIA_D, VIA_DRILL = 0.8, 0.4
B_COST, VIA_COST, TURN = 1.8, 60.0, 3.0   # back-layer multiplier, via cost and turn penalty in grid steps
KEEP = 65535
FALLBACK_WIDTHS = [2.5, 1.6, 1.0, 0.6, 0.4, 0.25]


def route_all(priority: list[str]):
    d = json.loads((BUILD / "pads.json").read_text())
    BW, BH = d["board"]
    nx, ny = int(round(BW / GRID)) + 1, int(round(BH / GRID)) + 1
    netnames = sorted(design.nets())
    nid = {n: i + 1 for i, n in enumerate(netnames)}
    owner = np.zeros((2, ny, nx), np.uint16)

    def c(v):
        return int(round(v / GRID))

    # board edge keep-out
    e = c(EDGE)
    for l in (0, 1):
        owner[l, :e, :] = owner[l, -e:, :] = owner[l, :, :e] = owner[l, :, -e:] = KEEP
    pads = d["pads"]
    for p in pads:
        val = nid.get(p["net"], KEEP) if p["net"] else KEEP
        if p["ref"].startswith("H"):
            for l in (0, 1):
                cv2.circle(owner[l], (c(p["x"]), c(p["y"])), c(3.3), KEEP, -1)
            continue
        x0, x1 = c(p["x"] - p["w"] / 2), c(p["x"] + p["w"] / 2)
        y0, y1 = c(p["y"] - p["h"] / 2), c(p["y"] + p["h"] / 2)
        for l, on in ((0, p["front"]), (1, p["back"])):
            if on or p["drill"] > 0:
                owner[l, max(y0, 0):y1 + 1, max(x0, 0):x1 + 1] = val

    tracks, vias, failed, log = [], [], [], []
    SQ2 = math.sqrt(2)
    MOVES = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, SQ2), (1, -1, SQ2), (-1, 1, SQ2), (-1, -1, SQ2)]

    def allowed_maps(net_id, width):
        need = width / 2 + CLR + MARGIN
        dist = [edt((owner[l] == 0) | (owner[l] == net_id)) * GRID for l in (0, 1)]
        ok = [dist[l] >= need for l in (0, 1)]
        via_ok = np.minimum(dist[0], dist[1]) >= (VIA_D / 2 + CLR + MARGIN)
        return ok, via_ok

    def astar(src_cells, tmask, ok, via_ok, hop_cells, single_layer=None):
        """src_cells: list of (l, y, x). tmask: bool[2, ny, nx]. hop_cells: set of (y, x) where a free layer change exists (own THT pad)."""
        tany = tmask[0] | tmask[1]
        if not tany.any():
            return None
        h = edt(~tany).astype(np.float32)
        g = np.full((2, ny, nx), np.inf, np.float32)
        din = np.full((2, ny, nx), -1, np.int8)      # index of the move that reached the cell (approximate turn penalty)
        prev = {}
        pq = []
        for s in src_cells:
            g[s] = 0.0
            heapq.heappush(pq, (float(h[s[1], s[2]]), 0.0, s))
        while pq:
            f, gc, node = heapq.heappop(pq)
            l, y, x = node
            if gc > g[l, y, x]:
                continue
            if tmask[l, y, x]:
                path = [node]
                while node in prev:
                    node = prev[node]
                    path.append(node)
                return path[::-1]
            lc = 1.0 if l == 0 else B_COST
            okl = ok[l]
            dn = din[l, y, x]
            for k, (dx, dy, cst) in enumerate(MOVES):
                xx, yy = x + dx, y + dy
                if 0 <= xx < nx and 0 <= yy < ny and (okl[yy, xx] or tmask[l, yy, xx]):
                    ng = gc + cst * lc + (TURN if (dn >= 0 and dn != k) else 0.0)
                    if ng < g[l, yy, xx]:
                        g[l, yy, xx] = ng
                        din[l, yy, xx] = k
                        prev[(l, yy, xx)] = node
                        heapq.heappush(pq, (ng + float(h[yy, xx]), ng, (l, yy, xx)))
            if single_layer is None:
                ol = 1 - l
                hop = (y, x) in hop_cells
                if hop or (via_ok[y, x] and okl[y, x] and ok[ol][y, x]):
                    ng = gc + (0.5 if hop else VIA_COST)
                    if ng < g[ol, y, x]:
                        g[ol, y, x] = ng
                        prev[(ol, y, x)] = node
                        heapq.heappush(pq, (ng + float(h[y, x]), ng, (ol, y, x)))
        return None

    def commit(path, net, width, hop_cells, tree):
        net_id = nid[net]
        th = max(1, int(round(width / GRID)))
        i = 0
        while i < len(path) - 1:
            l, y, x = path[i]
            l2, y2, x2 = path[i + 1]
            if l2 != l:
                if (y, x) not in hop_cells:
                    vias.append(dict(x=x * GRID, y=y * GRID, net=net))
                    for ll in (0, 1):
                        cv2.circle(owner[ll], (x, y), int(math.ceil(VIA_D / 2 / GRID)), net_id, -1)
                tree[l2, y, x] = True
                i += 1
                continue
            dx, dy = x2 - x, y2 - y
            j = i + 1
            while j < len(path) - 1 and path[j + 1][0] == l and (path[j + 1][2] - path[j][2], path[j + 1][1] - path[j][1]) == (dx, dy):
                j += 1
            _, ye, xe = path[j]
            tracks.append(dict(x0=x * GRID, y0=y * GRID, x1=xe * GRID, y1=ye * GRID, w=width, layer=l, net=net))
            m = np.zeros((ny, nx), np.uint8)
            cv2.line(m, (x, y), (xe, ye), 1, th)
            sel = (m > 0) & ((owner[l] == 0) | (owner[l] == net_id))
            owner[l][sel] = net_id
            cl = np.zeros((ny, nx), np.uint8)
            cv2.line(cl, (x, y), (xe, ye), 1, 1)
            tree[l] |= cl > 0
            i = j

    def pad_cells(p):
        cells = []
        if p["front"]:
            cells.append((0, c(p["y"]), c(p["x"])))
        if p["back"]:
            cells.append((1, c(p["y"]), c(p["x"])))
        return cells

    def pad_width(net, p):
        cls = design.net_class(net)
        w = design.WIDTH_OVERRIDE.get(net, design.WIDTH[cls])
        if cls == "hi" and p["ref"] in design.THIN_REFS:
            w = 0.4
        if cls == "pwr" and not (p["front"] and p["back"]):
            w = min(w, max(0.3, min(p["w"], p["h"]) * 0.9))
        return w

    def uniq_pads(net):
        seen, out = set(), []
        for p in pads:
            if p["net"] == net:
                key = (p["ref"], p["num"], round(p["x"], 2), round(p["y"], 2))
                if key not in seen:
                    seen.add(key)
                    out.append(p)
        return out

    def span(net):
        ps = uniq_pads(net)
        return (max(p["x"] for p in ps) - min(p["x"] for p in ps)) + (max(p["y"] for p in ps) - min(p["y"] for p in ps))

    # GND stitching vias for front-only (SMD) ground pads
    gid = nid["GND"]
    gnd_tree = np.zeros((2, ny, nx), bool)
    for p in uniq_pads("GND"):
        if p["back"]:
            continue
        ok, via_ok = allowed_maps(gid, 0.4)
        tm = np.zeros((2, ny, nx), bool)
        tm[0] = via_ok & ok[0] & ok[1] & (owner[1] == 0)
        # keep the via out from under the pad itself (solder wicking) by requiring >= 0.9 mm from the pad centre
        yy, xx = np.ogrid[:ny, :nx]
        tm[0] &= ((xx - c(p["x"])) ** 2 + (yy - c(p["y"])) ** 2) >= c(max(p["w"], p["h"]) / 2 + 0.6) ** 2
        path = astar([(0, c(p["y"]), c(p["x"]))], tm, ok, via_ok, set(), single_layer=0)
        if not path:
            # no room for a via: tie the pad to the nearest through-hole / already-stitched GND copper instead
            tm2 = gnd_tree.copy()
            for q in uniq_pads("GND"):
                if q["back"]:
                    tm2[0, c(q["y"]), c(q["x"])] = True
            path = astar([(0, c(p["y"]), c(p["x"]))], tm2, ok, via_ok, set(), single_layer=0)
            if path:
                commit(path, "GND", 0.4, set(), gnd_tree)
            else:
                failed.append(f"GND stitch: {p['ref']}.{p['num']}")
            continue
        commit(path, "GND", 0.4, set(), gnd_tree)
        _, y, x = path[-1]
        vias.append(dict(x=x * GRID, y=y * GRID, net="GND"))
        for ll in (0, 1):
            cv2.circle(owner[ll], (x, y), int(math.ceil(VIA_D / 2 / GRID)), gid, -1)
    order = sorted([n for n in netnames if n != "GND"],
                   key=lambda n: ({"hi": 0, "pwr": 1, "sig": 2}[design.net_class(n)],
                                  priority.index(n) if n in priority else len(priority), span(n)))
    for net in order:
        ps = uniq_pads(net)
        net_id = nid[net]
        # same-footprint pads that share a pad number and touch/overlap (thermal tab sub-pads) are already one piece of copper
        ps.sort(key=lambda p: (-pad_width(net, p), p["x"], p["y"]))
        tree = np.zeros((2, ny, nx), bool)
        hop_cells = {(c(p["y"]), c(p["x"])) for p in ps if p["front"] and p["back"]}
        connected = [ps[0]]
        for cell in pad_cells(ps[0]):
            tree[cell] = True
        todo = ps[1:]
        while todo:
            todo.sort(key=lambda p: min(math.hypot(p["x"] - q["x"], p["y"] - q["y"]) for q in connected))
            p = todo.pop(0)
            # pads of one footprint with the same number that overlap the tree copper already are connected by construction
            if any(q["ref"] == p["ref"] and q["num"] == p["num"] and abs(q["x"] - p["x"]) < (q["w"] + p["w"]) / 2 + 0.01
                   and abs(q["y"] - p["y"]) < (q["h"] + p["h"]) / 2 + 0.01 for q in connected):
                connected.append(p)
                for cell in pad_cells(p):
                    tree[cell] = True
                continue
            want = pad_width(net, p)
            path = None
            for w in [want] + [fw for fw in FALLBACK_WIDTHS if fw < want]:
                ok, via_ok = allowed_maps(net_id, w)
                path = astar(pad_cells(p), tree, ok, via_ok, hop_cells)
                if path:
                    if w != want:
                        log.append(f"{net}: {p['ref']}.{p['num']} attached with {w} mm instead of {want} mm")
                    commit(path, net, w, hop_cells, tree)
                    break
            if not path:
                failed.append(f"{net}: {p['ref']}.{p['num']}")
            connected.append(p)
            for cell in pad_cells(p):
                tree[cell] = True

    # ground stitching: tie the front and back GND pours together on a ~6 mm lattice wherever both layers are free
    ok, via_ok = allowed_maps(gid, 0.4)
    free = via_ok & (owner[0] == 0) & (owner[1] == 0)
    n_st = 0
    step = c(6.0)
    for yy in range(c(3.0), ny - c(3.0), step):
        for xx in range(c(3.0), nx - c(3.0), step):
            win = free[yy - 4:yy + 5, xx - 4:xx + 5]
            if win.any():
                oy, ox = np.argwhere(win)[0]
                vy, vx = yy - 4 + oy, xx - 4 + ox
                vias.append(dict(x=vx * GRID, y=vy * GRID, net="GND"))
                n_st += 1
    log.append(f"{n_st} GND stitching vias added")
    total = sum(math.hypot(t["x1"] - t["x0"], t["y1"] - t["y0"]) for t in tracks)
    return dict(tracks=tracks, vias=vias, via_d=VIA_D, via_drill=VIA_DRILL, failed=failed, log=log, track_len_mm=round(total, 1))


def main() -> int:
    priority: list[str] = []
    best = None
    for attempt in range(6):
        res = route_all(priority)
        print(f"attempt {attempt + 1}: tracks {len(res['tracks'])}, vias {len(res['vias'])}, "
              f"length {res['track_len_mm']:.0f} mm, failed {res['failed']}", flush=True)
        if best is None or len(res["failed"]) < len(best["failed"]):
            best = res
        if not res["failed"]:
            break
        # rip-up-and-retry at net granularity: nets that failed are routed first on the next attempt
        for f in res["failed"]:
            net = f.split(":")[0]
            if net in priority:
                priority.remove(net)
            if not net.startswith("GND"):
                priority.insert(0, net)
    best["priority_order"] = priority
    (BUILD / "routes.json").write_text(json.dumps(best, indent=1))
    for m in best["log"]:
        print("NOTE", m)
    return 1 if best["failed"] else 0


if __name__ == "__main__":
    sys.exit(main())
