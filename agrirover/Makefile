# AgriRover GH-1 — regenerate artefacts. See README.md.
PY ?= python3
.PHONY: all calcs cad pcb firmware ros-assets bom sim test report
all: calcs cad pcb firmware ros-assets bom sim test report
calcs:
	$(PY) mechanical/calcs/mechanical_calcs.py > /dev/null
	$(PY) electrical/calcs/electrical_calcs.py > /dev/null
cad: calcs
	$(PY) mechanical/cad/rover_cad.py > /dev/null
	$(PY) mechanical/cad/make_drawings.py
pcb:
	./electrical/pcb/build.sh
firmware:
	$(MAKE) -C firmware test stm32
ros-assets:
	$(PY) tools/gen_ros_assets.py
bom:
	$(PY) tools/make_bom.py
sim:
	cd software && $(PY) -m agrirover.sim.scenarios --out ../sim/results/scenario_report.json
	cd software && $(PY) -m agrirover.sim.run --out ../sim/results/reference --no-images
	rm -rf sim/results/reference/images sim/results/reference/agrirover.db*
test:
	cd software && $(PY) -m pytest -q
report:
	$(PY) tools/make_test_report.py
