# Research extension — autonomous inspection as an experimental instrument

The rover is a repeatable measuring device: the same viewpoints, the same sensors, timestamps and positions, as often
as the battery allows. That makes inspection *frequency*, *viewpoint* and *route* controllable experimental variables,
which a human scout cannot offer. This folder contains a small framework for turning that into a research programme at
the engineering / controlled-environment-agriculture interface.

`experiments.py` runs three model studies on the simulator and writes `results/`. **They are simulations with assumed
plant dynamics and synthetic images.** Their purpose is to fix the experimental designs, metrics and analysis code
before any plant is grown, and to produce quantitative hypotheses — not to answer the questions.

## Research questions, hypotheses and how each would be tested

| # | Question | Hypothesis | Independent variable | Outcome metrics | Status of the tooling |
|---|---|---|---|---|---|
| RQ1 | How does inspection frequency affect how early plant stress is detected? | Detection delay grows less than linearly with the inspection interval because symptom severity, not sampling, dominates for slowly developing stresses | inspection interval (0.5–7 days) | onset-to-first-flag delay, severity at detection, robot energy per day | `exp1`: Monte-Carlo with the pipeline's measured detection-vs-severity curve |
| RQ2 | Can autonomous inspection reduce manual scouting effort? | Robot screening lets a scout visit only flagged plants with no loss of sensitivity to stresses the camera can see | scouting protocol (full manual vs robot-directed) | scout minutes per bench, sensitivity against a full manual census | data model ready (per-plant history); needs a greenhouse trial |
| RQ3 | How does camera positioning affect classification? | Flag accuracy is flat up to a threshold of positioning error, then falls quickly as plants are assigned to the wrong slot | stop error (0–90 mm), later camera height and tilt | recall, false-flag rate, association error | `exp3` |
| RQ4 | Do environmental measurements improve abnormality detection? | Local VPD, light and CO₂ history carry predictive information for tip burn and growth lag beyond image features | feature set (image only vs image + climate) | AUROC / lead time on held-out benches | each observation is already linked to a climate sample; needs real data |
| RQ5 | How does energy consumption vary with the inspection route? | Route order changes mission energy by tens of percent because dwell power is comparable to drive power | aisle order and direction | Wh per mission, duration | `exp2` (analytic route model vs simulator) |
| RQ6 | Can missions be optimised for energy or information? | An adaptive policy (revisit flagged or fast-changing plants more often) detects stress earlier than fixed serpentine at equal energy | policy | detection delay at fixed Wh/day | route cost model and optimiser in `mission/route.py`; adaptive policy not implemented |

## What the model studies currently say (hypothesis-generating only)

* **exp1.** With the assumed symptom time constants (3–8 days), halving the inspection interval from daily to twice
  daily shortens the median detection delay by well under a day for the slow stresses, while doubling robot energy per
  day (1.7 → 3.5 Wh for the reference house). Fast, high-contrast symptoms are caught at the first visit at any
  interval up to a few days. If real symptom dynamics resemble the assumptions, daily inspection is near the knee of
  the curve — a specific, falsifiable prediction.
* **exp2.** Always entering aisles from the head corridor costs 1.98 Wh against 1.40 Wh for the serpentine route
  (simulated), a 41 % difference; exhaustive search over aisle orders finds nothing better than the serpentine in a
  rectangular house. The analytic model over-predicts energy by about 20 % and duration by about 20 %, mainly through
  its fixed dwell and turn allowances — good enough for ranking routes, not for absolute budgets.
* **exp3.** On synthetic images, flags are unaffected by stop errors up to 40 mm (σ) and degrade beyond 60 mm (recall
  0.97, 3 % false flags) — which supports the 50 mm stop-repeatability requirement and suggests that slot association,
  not colour analysis, will be the first thing to fail on a real robot.

## Proposed physical study (outline for a PhD-level proposal)

1. **Platform validation (months 0–3).** Build one rover; complete the physical test plan in
   `docs/10_validation_test_report.md`; publish stop repeatability, energy per mission and image-quality statistics.
2. **Dataset (months 3–9).** Two crop cycles of lettuce on three benches, twice-daily missions. Induce graded,
   documented stresses on randomised plots (nitrogen and calcium supply, EC, root-zone temperature, light), with
   untreated controls on every bench. Ground truth: weekly destructive sampling of sentinel plants (fresh mass, leaf
   area, tissue analysis), agronomist scoring twice a week, a quantum sensor and reference T/RH logger for the climate
   channel.
3. **RQ1/RQ3/RQ5 on real data (months 6–12).** Sub-sample the twice-daily record to emulate longer intervals; degrade
   positioning in software; log energy per route. Compare with the simulated predictions above.
4. **RQ4/RQ6 (months 9–18).** Image + climate models with bench-wise cross-validation; adaptive revisit policy
   evaluated first in replay on the dataset, then live.

**Design points that matter:** randomise treatments across benches to separate stress from position effects (the
climate gradient across a house is itself a confounder); split data by bench and date; pre-register the detection
threshold; report lead time relative to the agronomist, not just accuracy; treat the robot's own heat, light and
airflow as possible disturbances.

**Threats to validity.** One site, one cultivar and one lighting regime limit generality; RGB cannot separate causes
with similar symptoms; induced stresses are cleaner than commercial ones; the repeatable viewpoint that makes the
method work also means occluded plants are never seen.

## Reproducing

```bash
pip install -e "software[dev]"
python research/experiments.py            # ~3 min; writes research/results/
```
