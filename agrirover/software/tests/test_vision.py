"""Vision pipeline against synthetic ground truth (software verification only)."""
import numpy as np
import pytest

from agrirover.sim import camera_sim as cs
from agrirover.vision import pipeline, quality


def view(states, age=22, **kw):
    plants = [cs.PlantTruth(f"P{i}", age, h, s, seed=i) for i, (h, s) in enumerate(states)]
    img, truth = cs.render_view(plants, **kw)
    slots = [(p.pid, *cs.slot_px(*divmod(k, 4))) for k, p in enumerate(plants)]
    return img, truth, slots


MIX = [("healthy", 0), ("chlorosis", 0.8), ("tipburn", 0.8), ("purpling", 0.8), ("stunted", 0.8), ("missing", 1), ("healthy", 0), ("healthy", 0)]
EXPECT = [None, "chlorosis", "necrosis_or_tipburn", "purpling", "undersized", "missing", None, None]


def test_segmentation_and_flags():
    img, truth, slots = view(MIX, seed=1)
    res = pipeline.analyse(img, slots, cs.PX_PER_M, cs.GRAY_PATCH)
    m, t = res["mask"] > 0, truth["mask"] > 0
    assert (m & t).sum() / (m | t).sum() > 0.95
    for obs, exp, tr in zip(res["plants"], EXPECT, truth["plants"]):
        if exp is None:
            assert obs["status"] == "ok" and not obs["flags"], obs
        else:
            assert exp in obs["flags"] and 0.5 <= obs["flags"][exp] <= 1.0, obs
        if tr["area_px"]:
            assert obs["area_cm2"] == pytest.approx(tr["area_px"] / cs.PX_PER_M ** 2 * 1e4, rel=0.06)


@pytest.mark.parametrize("cast", [(1.0, 1.0, 1.0), (1.12, 0.92, 1.1), (0.9, 1.0, 1.12)])
def test_grey_card_removes_colour_cast(cast):
    """Pink / blue grow-light casts must not create or hide colour flags once the reference card is used."""
    img, _, slots = view(MIX, seed=2, cast=cast, gain=0.95)
    res = pipeline.analyse(img, slots, cs.PX_PER_M, cs.GRAY_PATCH)
    assert res["white_balance"]["applied"]
    got = [next(iter(o["flags"]), None) if o["flags"] else None for o in res["plants"]]
    assert got == EXPECT


def test_without_reference_card_confidence_is_reduced():
    img, _, slots = view(MIX, seed=3)
    with_card = pipeline.analyse(img, slots, cs.PX_PER_M, cs.GRAY_PATCH)["plants"][1]["flags"]["chlorosis"]
    without = pipeline.analyse(img, slots, cs.PX_PER_M, None)["plants"][1]["flags"]["chlorosis"]
    assert without < with_card


def test_quality_gate():
    sharp, _, _ = view(MIX, seed=4)
    q = quality.assess(sharp, cs.GRAY_PATCH)
    assert q["ok"] and q["edge_width_px"] < 2
    blurred, _, slots = view(MIX, seed=4, blur_px=13)
    qb = quality.assess(blurred, cs.GRAY_PATCH)
    assert not qb["ok"] and "blurred" in qb["reasons"] and qb["edge_width_px"] > 8
    assert all(o["status"] in ("unreliable", "missing") for o in pipeline.analyse(blurred, slots, cs.PX_PER_M, cs.GRAY_PATCH)["plants"])
    assert "underexposed" in quality.assess(view(MIX, seed=4, gain=0.22)[0], cs.GRAY_PATCH)["reasons"]
    assert "overexposed" in quality.assess(view(MIX, seed=4, gain=1.6)[0], cs.GRAY_PATCH)["reasons"]


def test_low_severity_is_not_over_claimed():
    """Mild symptoms should produce low or no confidence rather than a confident flag."""
    img, _, slots = view([("chlorosis", 0.15)] + [("healthy", 0)] * 7, seed=5)
    flags = pipeline.analyse(img, slots, cs.PX_PER_M, cs.GRAY_PATCH)["plants"][0]["flags"]
    assert flags.get("chlorosis", 0.0) < 0.6


def test_growth_curve_monotonic():
    d = [cs.PlantTruth("p", a).diameter_m() for a in range(0, 40, 4)]
    assert all(b > a for a, b in zip(d, d[1:])) and 0.02 < d[0] < 0.05 and 0.2 < d[-1] < 0.26


def test_onnx_postprocess_shapes():
    """EXPERIMENTAL detector: only the decoding / NMS code path is tested (no model is shipped)."""
    from agrirover.vision.onnx_detector import OnnxDetector
    det = OnnxDetector.__new__(OnnxDetector)
    det.names, det.conf, det.iou, det.size = ["plant", "lesion"], 0.35, 0.5, 640
    pred = np.zeros((1, 6, 4), np.float32)
    pred[0, :, 0] = [320, 320, 100, 100, 0.9, 0.05]
    pred[0, :, 1] = [322, 321, 100, 100, 0.8, 0.05]      # duplicate -> suppressed
    pred[0, :, 2] = [100, 500, 40, 40, 0.1, 0.7]
    pred[0, :, 3] = [600, 50, 20, 20, 0.1, 0.1]          # below threshold
    out = det.postprocess(pred, 1.0, 0, 0)
    assert sorted(o["cls"] for o in out) == ["lesion", "plant"] and all(o["level"] == "EXPERIMENTAL" for o in out)
