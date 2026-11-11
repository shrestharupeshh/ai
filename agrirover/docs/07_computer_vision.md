# 07 — Computer vision and plant monitoring

## 1. What the system claims, and what it does not

| Level | Component | Evidence available | What may be said |
|---|---|---|---|
| **DEMONSTRATION** | Classical RGB pipeline (`software/agrirover/vision/pipeline.py`) | Verified against exact ground truth on **synthetic** images only | "The software segments, associates and measures as designed." Nothing about real lettuce |
| **EXPERIMENTAL** | ONNX detector hook (`vision/onnx_detector.py`) | Decoding/NMS code path unit-tested; **no trained weights exist** | Nothing about accuracy |
| **TRAINED MODEL** | — | none | — |
| **VALIDATED AGRICULTURAL PREDICTION** | — | none | — |

The rover produces **screening flags** ("this plant looks different: yellow fraction 0.42, confidence 0.9") to direct a
person to look. It does not diagnose. Chlorosis, tip burn, anthocyanin purpling and stunting each have many causes
(nutrition, root-zone oxygen, pathogens, light, temperature, cultivar), several of which look alike in RGB, and some
diseases show no visible symptom until late. Every stored observation carries the pipeline level and a confidence, and
the dashboard repeats the caveat.

## 2. Pipeline

1. **Quality gate** (`quality.py`). Exposure from the grey-level mean and clipped fractions. Blur from the 10–90 % edge
   spread across the grey reference card, which is rigidly fixed in the corner of the image — a content-independent
   measurement (a variance-of-Laplacian metric was tried first and rejected: it tracked sensor noise, not sharpness).
   A frame that fails is retaken up to twice; if it still fails, its plants are stored as `unreliable` with no flags.
2. **Colour normalisation.** Channel gains that map the grey card to its nominal value remove the cast of LED grow
   lights. Without a usable card, confidence is multiplied by 0.7.
3. **Vegetation mask.** Saturation- and excess-green-based, so yellow, brown and purple tissue is kept (a green-only
   index would delete exactly the plants of interest). Assumes a low-saturation background (white channels, grey
   benches, black plastic).
4. **Plant localisation.** Pixels are assigned to the nearest expected plant slot within one slot radius; slots come
   from the station geometry. Output per plant: centroid, projected canopy area (cm²), equivalent diameter.
5. **Leaf colour analysis.** Fractions of canopy pixels in green / yellow / brown / purple hue bands, mean hue, ExG.
6. **Screening flags** with logistic confidence: `chlorosis` (yellow fraction), `necrosis_or_tipburn` (brown
   fraction), `purpling`, `undersized` (area < ~62 % of the median of the same view), `missing`.
7. **Growth monitoring.** `Store.growth_rates()` computes the relative growth rate of canopy area per plant between
   missions.

Not implemented: individual leaf segmentation (rosette leaves overlap heavily; instance segmentation needs a trained
model and annotated data), height or biomass (needs depth), any pathogen-specific classifier.

## 3. Results on synthetic data (software verification)

From `sim/results/scenario_report.json` and `research/results/`: mask IoU 0.98; recall 35/35 and precision 35/35 on
plants with synthetic severity 0.7–1.0; median canopy-area error 2.6 %; correct flags under three colour casts; no
flag raised from a blurred frame; flag recall stays ≥ 0.94 and false flags ≤ 10 % up to a 90 mm stop error, with no
degradation up to 40 mm. These numbers describe a renderer of flat coloured ellipses on a grey bench. Real images
have shadows, specular highlights, overlapping neighbours, algae, labels, water droplets and perspective; expect
materially worse performance until thresholds are re-derived on real data.

## 4. Public datasets (for a trained model)

| Dataset | Content | Licence | Use here | Limitations |
|---|---|---|---|---|
| 3rd Autonomous Greenhouse Challenge — Online Challenge Lettuce Images (WUR, 4TU.ResearchData, doi:10.4121/15023088) | 388 top-view RGB + depth images of single lettuce plants with fresh/dry weight, height, diameter, leaf area | CC BY 4.0 (verified on the 4TU record) | growth-trait regression (diameter, leaf area) to check the area/diameter estimates | healthy plants only, one plant per image, fixed overhead camera, 4 cultivars — no stress or disease labels |
| Same challenge, 2022 time-series (doi:10.4121/21960932) | climate time series with annotated lettuce images | CC BY 4.0 (verified) | relating environment to growth | same imaging limits |
| "Lettuce NPK" deficiency images (Kaggle) | few hundred images, four nutrient classes | **not verified** — check the dataset page before any use | exploratory only | small, lab conditions, class leakage risk |
| PlantVillage | leaf disease images, many crops | CC0 / varies by mirror | **not suitable**: no lettuce, single detached leaves on uniform backgrounds; models trained on it are known to generalise poorly to field images |

None of these contains stressed lettuce photographed obliquely from an aisle under grow lights. **A model for this
rover has to be trained on data collected by this rover.** No training was run for this package and no training
result is reported.

## 5. Data-collection and labelling workflow

1. **Calibrate** each camera once per site: fix focus, exposure and white balance (V4L2 controls); photograph a
   checkerboard lying on the bench to obtain the homography to the bench plane and the pixel scale; record the grey
   card ROI. Store as the `rectification_file` used by the mission node. Until this exists the mission node stores
   images but marks every plant `UNCALIBRATED`.
2. **Collect** with the normal inspection mission, twice daily for at least two crop cycles, so every plant has a
   time series under varying daylight. The database already links image, plant, pose, time and climate.
3. **Label** a stratified sample with an agronomist: per plant slot, {healthy, chlorosis, tip burn, purpling, wilting,
   mechanical damage, missing, other} plus severity 0–3 and, where known, the confirmed cause. Use two annotators on a
   10 % overlap and report Cohen's κ. Tooling: Label Studio or CVAT export in COCO format.
4. **Split by bench and by date**, never by image, to avoid leakage between neighbouring views of the same plants.
5. **Baseline first**: evaluate the classical pipeline on the labelled set (per-class precision/recall with confidence
   intervals) before training anything.
6. **Train** a small detector/classifier (e.g. YOLOv8n via Ultralytics, exported to ONNX and run through
   `OnnxDetector`); report on the held-out benches; calibrate confidences (reliability diagram).
7. **Promote** a model to "trained" in the table above only with its dataset card, metrics and failure cases written
   down in this document.

## 6. Camera choice and geometry

Two UVC cameras look sideways, 35° below horizontal, from 1.23 m. With a bench top at 0.80 m and a 0.8 m aisle the
near half of a 1.2 m bench spans depression angles of about 44° to 21°; a 43° vertical field covers 13.5–56.5°. A
70° horizontal field covers 1.15 m of bench at mid-range, more than the 1.0 m station pitch. These inequalities are
enforced by `test_camera_geometry_covers_station`; the field-of-view values are assumptions for a C920-class webcam
and must be measured on the real lens.
