#!/usr/bin/env bash
# Rebuild the ARC-1 schematic, board and fabrication outputs from gen/design.py.
# Requires: KiCad 7 (kicad-cli + pcbnew for /usr/bin/python3.12), python3 with numpy, scipy, opencv, matplotlib.
set -euo pipefail
cd "$(dirname "$0")"
PY=${PY:-python3}; KPY=${KPY:-/usr/bin/python3.12}
mkdir -p build reports fab/gerbers
$PY gen/gen_schematic.py
kicad-cli sch export netlist -o reports/arc1.net arc1.kicad_sch
kicad-cli sch export pdf -o reports/arc1_schematic.pdf arc1.kicad_sch
$PY gen/check_netlist.py
$KPY gen/board_place.py
$PY gen/route.py
$KPY gen/board_finish.py
rm -f fab/gerbers/*
kicad-cli pcb export gerbers -o fab/gerbers/ -l F.Cu,B.Cu,F.Paste,B.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts arc1.kicad_pcb
kicad-cli pcb export drill -o fab/gerbers/ --format excellon --excellon-separate-th --generate-map --map-format pdf arc1.kicad_pcb
kicad-cli pcb export pos -o fab/arc1_pos.csv --format csv --units mm --side both arc1.kicad_pcb
kicad-cli pcb export pdf -o reports/arc1_assembly_top.pdf -l F.Fab,F.SilkS,Edge.Cuts --black-and-white arc1.kicad_pcb
kicad-cli pcb export svg -o reports/arc1_board_top.svg -l F.Cu,F.SilkS,Edge.Cuts --page-size-mode 2 arc1.kicad_pcb
kicad-cli pcb export svg -o reports/arc1_board_bottom.svg -l B.Cu,Edge.Cuts --page-size-mode 2 arc1.kicad_pcb
$PY gen/make_bom.py
$PY gen/render_board.py
(cd fab && rm -f arc1_gerbers.zip && zip -qj arc1_gerbers.zip gerbers/*)
echo "ARC-1 build complete"
