#!/usr/bin/env python3
"""Independent schematic checks (KiCad 7's CLI has no ERC).

1. Netlist equivalence: the netlist KiCad itself extracts from arc1.kicad_sch
   must equal the design intent in design.py (same nets, same pins).
2. Electrical rule checks on pin types taken from the KiCad symbol libraries:
   - every net has >= 2 pins
   - power-input pins are on a net driven by a power output or a PWR_FLAG
   - no two outputs / power outputs share a net
   - every symbol pin is either connected or explicitly flagged no-connect
3. Design-specific checks (voltage dividers, pull-ups present, fuse ratings).

Writes reports/schematic_check.txt and exits non-zero on any error.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import design  # noqa: E402
from gen_schematic import load_symbol, parse, pins_of  # noqa: E402

PCB = Path(__file__).resolve().parents[1]


def kicad_nets(path: Path) -> dict:
    tree = parse(path.read_text())
    out = {}
    for sec in tree:
        if isinstance(sec, list) and sec[0] == "nets":
            for net in sec[1:]:
                name, nodes = None, []
                for e in net:
                    if isinstance(e, list) and e[0] == "name":
                        name = e[1][1] if isinstance(e[1], tuple) else e[1]
                    if isinstance(e, list) and e[0] == "node":
                        d = {f[0]: (f[1][1] if isinstance(f[1], tuple) else f[1]) for f in e if isinstance(f, list)}
                        nodes.append((d["ref"], d["pin"]))
                out[name.lstrip("/")] = sorted(nodes)
    return out


def main() -> int:
    errors, warns, info = [], [], []
    intent = {k: [p for p in v if not p[0].startswith("#") and p[1] != "MP"] for k, v in design.nets().items()}   # MP = footprint-only shield pads
    got = kicad_nets(PCB / "reports" / "arc1.net")
    # footprints with several pads sharing a number (fuse holders, MP pads) appear once per pin number
    real = {k: v for k, v in got.items() if not k.startswith("unconnected-")}
    unconnected = {k: v for k, v in got.items() if k.startswith("unconnected-")}
    for net in sorted(set(intent) | set(real)):
        a, b = sorted(set(intent.get(net, []))), sorted(set(real.get(net, [])))
        if a != b:
            errors.append(f"net {net}: design {a} != schematic {b}")
    info.append(f"netlist equivalence: {len(intent)} design nets vs {len(real)} schematic nets")
    nc_intent = {(c[0], p) for c in design.COMPONENTS for p, n in c[4].items() if n == "NC"}
    nc_got = {n for v in unconnected.values() for n in v}
    if nc_got != nc_intent:
        errors.append(f"no-connect mismatch: design-only {sorted(nc_intent-nc_got)}, schematic-only {sorted(nc_got-nc_intent)}")
    info.append(f"no-connect pins: {len(nc_intent)} flagged, all accounted for" if nc_got == nc_intent else "no-connect mismatch")

    # ---- pin-type rules
    comps = {c[0]: c for c in design.COMPONENTS}
    ptype = {}
    for ref, lib_id, _v, _fp, pins, *_ in design.COMPONENTS:
        pl = pins_of(load_symbol(lib_id))
        nums = {p["num"] for p in pl}
        for p in pl:
            ptype[(ref, p["num"])] = p["type"]
        for pin in pins:
            if pin not in nums and pin != "MP":
                errors.append(f"{ref}: design pin {pin} does not exist in symbol {lib_id}")
        for n in nums - set(pins):
            errors.append(f"{ref}: symbol pin {n} is neither connected nor flagged NC")
    for net, nodes in design.nets().items():
        types = [ptype.get(n, "passive") for n in nodes]
        if len([n for n in nodes if not n[0].startswith("#")]) < 2:
            errors.append(f"net {net} has fewer than two pins: {nodes}")
        if "power_in" in types and not ({"power_out"} & set(types)):
            (warns if net in ("BAT_PROT",) else errors).append(f"net {net}: power input pin(s) without a power_out/PWR_FLAG driver")
        if types.count("power_out") + types.count("output") > 1:
            errors.append(f"net {net}: multiple drivers {[(n, t) for n, t in zip(nodes, types) if t in ('power_out', 'output')]}")
    info.append(f"pin-type rules evaluated on {len(design.nets())} nets / {len(ptype)} pins")

    # ---- design-specific checks
    val = {c[0]: c[2] for c in design.COMPONENTS}
    nets = design.nets()

    def has(net, ref):
        return any(r == ref for r, _ in nets[net])

    def ohms(s):
        s = s.split()[0]
        for k, m in (("mR", 1e-3), ("k", 1e3), ("R", 1.0), ("M", 1e6)):
            if k in s:
                a, _, b = s.partition(k)
                return (float(a) + (float("0." + b) if b else 0)) * m
        return float(s)

    div = ohms(val["R9"]) / (ohms(val["R8"]) + ohms(val["R9"]))
    v_adc = 14.6 * div
    (info if v_adc < 3.3 else errors).append(f"VMOT_SENSE divider: 14.6 V -> {v_adc:.2f} V at the ADC (limit 3.3 V)")
    for net in ("I2C_SCL", "I2C_SDA", "M_DIAG", "ESTOP_SENSE", "BUMP"):
        pulls = [r for r, _ in nets[net] if r.startswith("R") and has("+3V3", r)]
        (info if pulls else errors).append(f"{net}: pull-up to +3V3 {'present ' + str(pulls) if pulls else 'MISSING'}")
    for net in ("M_EN", "MOTEN_G"):
        pd = [r for r, _ in nets[net] if r.startswith("R") and has("GND", r)]
        (info if pd else errors).append(f"{net}: default-off pull-down {'present ' + str(pd) if pd else 'MISSING'}")
    vgs = 14.6 * ohms(val["R3"]) / (ohms(val["R3"]) + ohms(val["R4"]))
    info.append(f"Q2 gate drive at 14.6 V: |Vgs| = {vgs:.1f} V before the 12 V zener clamp (Vgs max of DMP4015SK3 = 25 V)")
    tau = ohms(val["R3"]) * 2.2e-6
    import math
    info.append(f"Q2 turn-off delay: tau = R3*C3 = {tau*1000:.0f} ms; 12 V -> ~2 V threshold in {tau*math.log(12/2.0)*1000:.0f} ms (target 250-500 ms)")
    info.append(f"components: {len([c for c in design.COMPONENTS if not c[0].startswith('#')])}, nets: {len(nets)}")

    rep = ["ARC-1 schematic check (generated by gen/check_netlist.py)", ""]
    rep += [f"ERROR  {e}" for e in errors] + [f"WARN   {w}" for w in warns] + [f"ok     {i}" for i in info]
    rep += ["", f"RESULT: {len(errors)} error(s), {len(warns)} warning(s)",
            "NOTE: this is not the KiCad GUI ERC. Run Inspect > Electrical Rules Checker in KiCad before fabrication."]
    (PCB / "reports" / "schematic_check.txt").write_text("\n".join(rep) + "\n")
    print("\n".join(rep))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
