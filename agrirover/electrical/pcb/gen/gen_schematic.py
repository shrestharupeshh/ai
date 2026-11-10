#!/usr/bin/env python3
"""Generate arc1.kicad_sch (KiCad 7 format) from design.py.

Every symbol is taken verbatim from the installed KiCad symbol libraries.
Connectivity is expressed with a short wire stub + net label on every pin
(a "label-style" schematic): unambiguous for netlisting, easy to regenerate,
and still editable in the KiCad GUI.
"""
from __future__ import annotations

import math
import sys
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import design  # noqa: E402

SYMDIR = Path("/usr/share/kicad/symbols")
OUT = Path(__file__).resolve().parents[1] / "arc1"
NS = uuid.UUID("6f1d3c0a-0000-4000-8000-a611c0ffee00")
ROOT_UUID = str(uuid.uuid5(NS, "root"))


def uid(*k) -> str:
    return str(uuid.uuid5(NS, "/".join(map(str, k))))


# ----------------------------------------------------------------- s-expressions
def parse(text: str):
    i, n = 0, len(text)
    stack, cur = [], []
    while i < n:
        ch = text[i]
        if ch == "(":
            stack.append(cur)
            cur = []
            i += 1
        elif ch == ")":
            done = cur
            cur = stack.pop()
            cur.append(done)
            i += 1
        elif ch == '"':
            j = i + 1
            buf = []
            while text[j] != '"':
                if text[j] == "\\":
                    buf.append(text[j:j + 2])
                    j += 2
                else:
                    buf.append(text[j])
                    j += 1
            cur.append(("s", "".join(buf)))
            i = j + 1
        elif ch.isspace():
            i += 1
        else:
            j = i
            while j < n and not text[j].isspace() and text[j] not in "()":
                j += 1
            cur.append(text[i:j])
            i = j
    return cur[0]


def dump(x, ind=0) -> str:
    if isinstance(x, tuple):
        return '"' + x[1] + '"'
    if isinstance(x, str):
        return x
    inner = [dump(e, ind + 1) for e in x]
    one = "(" + " ".join(inner) + ")"
    if len(one) < 100 or all(not isinstance(e, list) for e in x):
        return one
    pad = "\n" + "  " * (ind + 1)
    head = [e for e in inner[:1]]
    return "(" + inner[0] + "".join((" " + s if not isinstance(e, list) else pad + s) for e, s in zip(x[1:], inner[1:])) + "\n" + "  " * ind + ")"


def S(s):
    return ("s", str(s))


_lib_cache: dict = {}


def load_symbol(lib_id: str):
    """Return a flattened symbol s-expr named lib_id (resolves `extends`)."""
    lib, name = lib_id.split(":")
    if lib not in _lib_cache:
        tree = parse((SYMDIR / f"{lib}.kicad_sym").read_text())
        _lib_cache[lib] = {e[1][1]: e for e in tree if isinstance(e, list) and e[0] == "symbol"}
    sym = _lib_cache[lib][name]
    ext = [e for e in sym if isinstance(e, list) and e[0] == "extends"]
    if ext:
        parent_name = ext[0][1][1]
        parent = _lib_cache[lib][parent_name]
        props = {e[1][1]: e for e in sym if isinstance(e, list) and e[0] == "property"}
        flat = []
        for e in parent:
            if isinstance(e, list) and e[0] == "property" and e[1][1] in props:
                flat.append(props[e[1][1]])
            elif isinstance(e, list) and e[0] == "symbol":
                sub = list(e)
                sub[1] = S(sub[1][1].replace(parent_name, name, 1))
                flat.append(sub)
            else:
                flat.append(e)
        sym = flat
    sym = list(sym)
    sym[1] = S(lib_id)
    return sym


def pins_of(sym) -> list[dict]:
    out = []
    for sub in sym:
        if isinstance(sub, list) and sub[0] == "symbol":
            for e in sub:
                if isinstance(e, list) and e[0] == "pin":
                    d = {"type": e[1]}
                    for f in e:
                        if isinstance(f, list):
                            if f[0] == "at":
                                d["x"], d["y"], d["a"] = float(f[1]), float(f[2]), float(f[3]) if len(f) > 3 else 0.0
                            elif f[0] == "number":
                                d["num"] = f[1][1]
                            elif f[0] == "name":
                                d["name"] = f[1][1]
                    out.append(d)
    return out


def r2(v):
    return f"{v:.2f}".rstrip("0").rstrip(".") if "." in f"{v:.2f}" else f"{v:.2f}"


FONT = ["effects", ["font", ["size", "1.27", "1.27"]]]


def build():
    comps = {c[0]: c for c in design.COMPONENTS}
    lib_syms, items = {}, []
    STUB = 2.54
    gx0, gy0 = 25.4, 30.48
    x_cursor, y_cursor, row_h = gx0, gy0, 0.0
    SHEET_W = 800.0
    for title, refs in design.GROUPS:
        # measure group
        cells = []
        for ref in refs:
            _, lib_id, val, fp, pins, *_ = comps[ref]
            if lib_id not in lib_syms:
                lib_syms[lib_id] = load_symbol(lib_id)
            pl = pins_of(lib_syms[lib_id])
            xs = [p["x"] for p in pl] or [0]
            ys = [p["y"] for p in pl] or [0]
            maxlab = max([len(n) for n in pins.values()] or [2]) * 1.1 + STUB + 3
            w = (max(xs) - min(xs)) + 2 * maxlab + 10
            h = (max(ys) - min(ys)) + 2 * maxlab * (1 if any(p["a"] in (90, 270) for p in pl) else 0.25) + 22
            cells.append((ref, w, h, -min(xs) + maxlab + 4, max(ys) + (maxlab if any(p["a"] == 270 for p in pl) else 6) + 10))
        gw = sum(c[1] for c in cells)
        if x_cursor + min(gw, SHEET_W - gx0) > SHEET_W:
            x_cursor, y_cursor, row_h = gx0, y_cursor + row_h + 12.7, 0.0
        items.append(["text", S(title), ["at", r2(x_cursor), r2(y_cursor - 5.08), "0"],
                      ["effects", ["font", ["size", "2", "2"], "bold"], ["justify", "left", "bottom"]], ["uuid", uid("t", title)]])
        cx, line_h, gx_start = x_cursor, 0.0, x_cursor
        for ref, w, h, ox, oy in cells:
            if cx + w > SHEET_W:
                cx = gx_start
                y_cursor += line_h + 5.08
                line_h = 0.0
            _, lib_id, val, fp, pins, *_ = comps[ref]
            sx = round((cx + ox) / 2.54) * 2.54
            sy = round((y_cursor + oy) / 2.54) * 2.54
            pl = pins_of(lib_syms[lib_id])
            ys = [p["y"] for p in pl] or [0]
            sym = ["symbol", ["lib_id", S(lib_id)], ["at", r2(sx), r2(sy), "0"], ["unit", "1"],
                   ["in_bom", "yes" if not ref.startswith("#") else "no"], ["on_board", "yes"], ["dnp", "no"], ["uuid", uid("sym", ref)],
                   ["property", S("Reference"), S(ref), ["at", r2(sx), r2(sy - max(ys) - 6.35), "0"], FONT + ([] if not ref.startswith("#") else ["hide"])],
                   ["property", S("Value"), S(val), ["at", r2(sx), r2(sy - max(ys) - 4.2), "0"], FONT],
                   ["property", S("Footprint"), S(fp), ["at", r2(sx), r2(sy), "0"], FONT + ["hide"]],
                   ["property", S("Datasheet"), S("~"), ["at", r2(sx), r2(sy), "0"], FONT + ["hide"]],
                   ["property", S("MPN"), S(comps[ref][5]), ["at", r2(sx), r2(sy), "0"], FONT + ["hide"]]]
            seen = set()
            for p in pl:
                px, py = sx + p["x"], sy - p["y"]
                a = math.radians(p["a"])
                dx, dy = -math.cos(a), math.sin(a)          # outward direction in sheet coordinates
                ex, ey = px + dx * STUB, py + dy * STUB
                net = pins.get(p["num"], pins.get(p.get("name", ""), None))
                if net is None:
                    raise SystemExit(f"{ref}: pin {p['num']} ({p.get('name')}) has no net assignment")
                if p["num"] not in seen:
                    sym.append(["pin", S(p["num"]), ["uuid", uid("pin", ref, p["num"])]])
                    seen.add(p["num"])
                if net == "NC":
                    items.append(["no_connect", ["at", r2(px), r2(py)], ["uuid", uid("nc", ref, p["num"], px, py)]])
                    continue
                if lib_id == "power:PWR_FLAG":
                    ex, ey = px, py + STUB
                items.append(["wire", ["pts", ["xy", r2(px), r2(py)], ["xy", r2(ex), r2(ey)]],
                              ["stroke", ["width", "0"], ["type", "default"]], ["uuid", uid("w", ref, p["num"], px, py)]])
                if abs(dx) > 0.5:
                    ang, just = ("180", "right") if dx < 0 else ("0", "left")
                else:
                    ang, just = ("90", "left") if ey < py else ("270", "right")
                items.append(["label", S(net), ["at", r2(ex), r2(ey), ang],
                              ["effects", ["font", ["size", "1.27", "1.27"]], ["justify", just, "bottom"]],
                              ["uuid", uid("l", ref, p["num"], px, py)]])
            sym.append(["instances", ["project", S("arc1"), ["path", S("/" + ROOT_UUID), ["reference", S(ref)], ["unit", "1"]]]])
            items.append(sym)
            cx += w
            line_h = max(line_h, h)
        x_cursor = cx + 15
        row_h = max(row_h, line_h)
        if x_cursor > SHEET_W - 80:
            x_cursor, y_cursor, row_h = gx0, y_cursor + row_h + 12.7, 0.0
    notes = ("NOTES\\n1. Generated from electrical/pcb/gen/design.py - edit there and run build.sh.\\n"
             "2. BlackPill header order (A1/A2) must be verified against the physical module before fabrication.\\n"
             "3. E-stop: opening J14 pins 1-2 removes the Q2 gate drive; C3/R3 hold Q2 on ~0.3 s while firmware brakes (stop cat. 1 style).\\n"
             "4. KiCad GUI ERC has not been run on this sheet (KiCad 7 CLI has no ERC); see electrical/pcb/reports.")
    items.append(["text", S(notes), ["at", r2(gx0), r2(y_cursor + row_h + 25), "0"],
                  ["effects", ["font", ["size", "1.8", "1.8"]], ["justify", "left", "top"]], ["uuid", uid("notes")]])
    sch = ["kicad_sch", ["version", "20230121"], ["generator", "agrirover_gen"], ["uuid", S(ROOT_UUID)],
           ["paper", S("User"), "841", str(int(max(420, y_cursor + row_h + 80)))],
           ["title_block", ["title", S("AgriRover GH-1 control & power carrier (ARC-1)")], ["date", S("2026-11-10")], ["rev", S("A")],
            ["company", S("AgriRover GH-1 open hardware")]],
           ["lib_symbols"] + list(lib_syms.values())] + items + [["sheet_instances", ["path", S("/"), ["page", S("1")]]]]
    OUT.with_suffix(".kicad_sch").write_text(dump(sch) + "\n")
    pro = OUT.with_suffix(".kicad_pro")
    if not pro.exists():
        pro.write_text('{\n  "meta": {"filename": "arc1.kicad_pro", "version": 1},\n  "sheets": [["%s", ""]]\n}\n' % ROOT_UUID)
    print("wrote", OUT.with_suffix(".kicad_sch"), f"({len(items)} items)")


if __name__ == "__main__":
    build()
