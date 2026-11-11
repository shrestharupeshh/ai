"""Classical RGB plant-monitoring pipeline (DEMONSTRATION level).

Steps: quality gate -> grey-card colour normalisation -> vegetation segmentation -> per-plant instances by
watershed around the expected plant slots -> colour / size metrics -> screening flags with confidence.

Assumptions (hold for the synthetic renderer, must be re-established for a real camera):
  * the view is rectified to a known scale (px_per_m) and the plant slots of the station are known
  * a neutral grey reference card is visible at a fixed image location
  * the bench / channel background is low-saturation (white, grey, black plastic)
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field

import cv2
import numpy as np

from . import quality

# OpenCV hue is 0..179
HUE = dict(brown=(5, 22), yellow=(22, 36), green=(36, 90), purple=(125, 170))
FLAG_RULES = {                     # metric, threshold at which confidence = 0.5, logistic width
    "chlorosis": ("yellow_frac", 0.30, 0.08),
    "necrosis_or_tipburn": ("brown_frac", 0.045, 0.015),
    "purpling": ("purple_frac", 0.12, 0.04),
}


@dataclass
class PlantObservation:
    plant_id: str
    present: bool
    cx: float = 0.0
    cy: float = 0.0
    area_cm2: float = 0.0
    diameter_cm: float = 0.0
    green_frac: float = 0.0
    yellow_frac: float = 0.0
    brown_frac: float = 0.0
    purple_frac: float = 0.0
    mean_hue: float = 0.0
    exg: float = 0.0
    flags: dict = field(default_factory=dict)       # name -> confidence 0..1
    status: str = "ok"                              # ok | attention | missing | unreliable

    def to_dict(self):
        return asdict(self)


def white_balance(img: np.ndarray, patch_roi: tuple[int, int, int, int] | None, target: int = 118) -> tuple[np.ndarray, dict]:
    """Scale each channel so the grey reference card reads `target`. Returns (image, info)."""
    if patch_roi is None:
        return img, dict(applied=False, reason="no reference patch configured")
    x0, y0, x1, y1 = patch_roi
    patch = img[y0 + 5:y1 - 5, x0 + 5:x1 - 5].reshape(-1, 3).astype(np.float32)
    med = np.median(patch, axis=0)
    spread = float(patch.std(axis=0).max())
    if med.min() < 20 or med.max() > 245 or spread > 25:
        return img, dict(applied=False, reason="reference patch unusable", median=med.round(1).tolist())
    gains = target / med
    out = np.clip(img.astype(np.float32) * gains[None, None, :], 0, 255).astype(np.uint8)
    return out, dict(applied=True, gains=gains.round(3).tolist())


def vegetation_mask(img: np.ndarray, patch_roi=None) -> np.ndarray:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    b, g, r = [c.astype(np.float32) for c in cv2.split(img)]
    exg = 2 * g - r - b
    sat_fg = (hsv[..., 1] > 70) & (hsv[..., 2] > 45)           # coloured tissue of any hue (green, yellow, brown, purple)
    mask = (sat_fg | (exg > 40)).astype(np.uint8) * 255
    if patch_roi is not None:
        x0, y0, x1, y1 = patch_roi
        mask[max(0, y0 - 4):y1 + 4, max(0, x0 - 4):x1 + 4] = 0
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)


def _logistic(x: float, x50: float, width: float) -> float:
    return 1.0 / (1.0 + math.exp(-(x - x50) / width))


def analyse(img_bgr: np.ndarray, slots: list[tuple[str, int, int]], px_per_m: float, patch_roi=None,
            slot_radius_m: float = 0.125) -> dict:
    """slots: [(plant_id, cx_px, cy_px)] nominal plant positions in this view."""
    q = quality.assess(img_bgr, patch_roi)
    img, wb = white_balance(img_bgr, patch_roi)
    mask = vegetation_mask(img, patch_roi)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # instance labelling: each foreground pixel goes to the nearest slot centre within slot_radius
    yy, xx = np.mgrid[0:img.shape[0], 0:img.shape[1]]
    labels = np.zeros(mask.shape, np.int32)
    best = np.full(mask.shape, np.inf, np.float32)
    rmax = slot_radius_m * px_per_m * 1.15
    for k, (_pid, cx, cy) in enumerate(slots):
        d = np.hypot(xx - cx, yy - cy).astype(np.float32)
        sel = (d < best) & (d < rmax)
        labels[sel] = k + 1
        best[sel] = d[sel]
    labels[mask == 0] = 0
    quality_factor = 0.4 + 0.6 * q["score"] if q["ok"] else 0.3
    wb_factor = 1.0 if wb.get("applied") else 0.7
    obs: list[PlantObservation] = []
    areas = []
    for k, (pid, cx, cy) in enumerate(slots):
        m = labels == k + 1
        n = int(m.sum())
        area_cm2 = n / (px_per_m ** 2) * 1e4
        if area_cm2 < 3.0:                                    # less than ~2 cm across: treat as no plant
            obs.append(PlantObservation(pid, False, cx, cy, flags={"missing": round(0.9 * quality_factor, 2)}, status="missing"))
            continue
        ys, xs = np.nonzero(m)
        h = hsv[..., 0][m]
        s = hsv[..., 1][m]
        v = hsv[..., 2][m]
        col = s > 60
        nc = max(int(col.sum()), 1)
        frac = {name: float(((h >= lo) & (h < hi) & col).sum()) / nc for name, (lo, hi) in HUE.items()}
        frac["brown"] = float(((h >= HUE["brown"][0]) & (h < HUE["brown"][1]) & col & (v < 170)).sum()) / nc
        bgr = img[m].astype(np.float32)
        tot = bgr.sum(axis=1) + 1e-6
        exg = float(np.mean((2 * bgr[:, 1] - bgr[:, 2] - bgr[:, 0]) / tot))
        o = PlantObservation(pid, True, float(xs.mean()), float(ys.mean()), round(area_cm2, 1),
                             round(2 * math.sqrt(area_cm2 / math.pi), 1), round(frac["green"], 3), round(frac["yellow"], 3),
                             round(frac["brown"], 3), round(frac["purple"], 3), round(float(np.mean(h[col])) if col.any() else 0.0, 1),
                             round(exg, 3))
        for flag, (metric, x50, width) in FLAG_RULES.items():
            conf = _logistic(getattr(o, metric), x50, width) * quality_factor * wb_factor
            if conf >= 0.5:
                o.flags[flag] = round(conf, 2)
        areas.append(area_cm2)
        obs.append(o)
    # relative size check inside the view (plants at one station are the same age)
    if len(areas) >= 4:
        med = float(np.median(areas))
        for o in obs:
            if o.present and med > 0:
                ratio = o.area_cm2 / med
                conf = _logistic(0.62 - ratio, 0.0, 0.07) * quality_factor
                if conf >= 0.5:
                    o.flags["undersized"] = round(conf, 2)
    for o in obs:
        if not q["ok"]:
            o.status = "unreliable"
        elif o.status != "missing" and o.flags:
            o.status = "attention"
    return dict(quality=q, white_balance=wb, canopy_cover=round(float((mask > 0).mean()), 4),
                plants=[o.to_dict() for o in obs], mask=mask, level="DEMONSTRATION")
