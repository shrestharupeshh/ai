"""Image-quality gate: reject frames that would give misleading plant metrics."""
from __future__ import annotations

import cv2
import numpy as np

MIN_SHARPNESS = 0.40       # scale-ratio sharpness (content dependent: only catches gross blur); re-tune on the real camera
MAX_EDGE_WIDTH_PX = 5.0    # 10-90 % rise distance across the reference-card edges (content independent, preferred)
MAX_CLIP_FRAC = 0.05
MEAN_RANGE = (50, 205)


def sharpness(gray: np.ndarray) -> float:
    """Scale-ratio edge sharpness, ~1 for crisp edges and ~2/w for edges smeared over w pixels (at 480 px width).

    Ratio of the strongest gradients at 480 px width to those at 120 px width, evaluated separately in x and y and
    taking the weaker direction. Being a ratio it is independent of exposure and contrast; the down-sampling
    suppresses sensor noise (which dominates variance-of-Laplacian); the directional minimum catches motion blur.
    """
    h = max(1, int(gray.shape[0] * 480 / gray.shape[1]))
    fine = cv2.GaussianBlur(cv2.resize(gray, (480, h), interpolation=cv2.INTER_AREA), (3, 3), 0).astype(np.float32)
    coarse = cv2.resize(fine, (120, max(1, h // 4)), interpolation=cv2.INTER_AREA)
    out = []
    for dx, dy in ((1, 0), (0, 1)):
        f = np.percentile(np.abs(cv2.Sobel(fine, cv2.CV_32F, dx, dy)), 99.5)
        c = np.percentile(np.abs(cv2.Sobel(coarse, cv2.CV_32F, dx, dy)), 99.5)
        out.append(f / c if c > 8.0 else 0.0)            # no usable edges at all -> 0
    return float(min(out))


def edge_width_px(gray: np.ndarray, patch_roi: tuple[int, int, int, int]) -> float | None:
    """Worst 10-90 % edge-spread width (px) across the right and bottom edges of the grey reference card.

    The card is rigidly attached to the camera boom, so its edges are a fixed, known step target: their spread
    measures defocus and motion blur independently of what the plants look like.
    """
    x0, y0, x1, y1 = patch_roi
    half = 14
    if x1 + half >= gray.shape[1] or y1 + half >= gray.shape[0]:
        return None
    profiles = [gray[y0 + 8:y1 - 8, x1 - half:x1 + half].mean(axis=0), gray[y1 - half:y1 + half, x0 + 8:x1 - 8].mean(axis=1)]
    worst = 0.0
    for pr in profiles:
        pr = pr.astype(np.float32)
        lo, hi = float(pr[:4].mean()), float(pr[-4:].mean())
        if abs(hi - lo) < 12:                       # card not distinguishable from the background
            return None
        t = (pr - lo) / (hi - lo)
        xs = np.arange(len(t))
        i10 = float(np.interp(0.1, np.maximum.accumulate(t), xs))
        i90 = float(np.interp(0.9, np.maximum.accumulate(t), xs))
        worst = max(worst, i90 - i10)
    return worst


def assess(img_bgr: np.ndarray, patch_roi: tuple[int, int, int, int] | None = None) -> dict:
    g = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    sharp = sharpness(g)
    mean = float(g.mean())
    hi, lo = float((g >= 250).mean()), float((g <= 5).mean())
    reasons = []
    ew = edge_width_px(g, patch_roi) if patch_roi is not None else None
    if (ew is not None and ew > MAX_EDGE_WIDTH_PX) or (ew is None and sharp < MIN_SHARPNESS):
        reasons.append("blurred")
    if hi > MAX_CLIP_FRAC:
        reasons.append("overexposed")
    if lo > MAX_CLIP_FRAC or mean < MEAN_RANGE[0]:
        reasons.append("underexposed")
    if mean > MEAN_RANGE[1]:
        reasons.append("overexposed")
    sharp_score = min(sharp / (1.5 * MIN_SHARPNESS), 1.0) if ew is None else float(np.clip(1.25 - ew / (2 * MAX_EDGE_WIDTH_PX), 0, 1))
    score = float(np.clip(sharp_score * (1 - min(1.0, (hi + lo) / MAX_CLIP_FRAC * 0.5)), 0, 1))
    return dict(ok=not reasons, score=round(score, 3), sharpness=round(sharp, 3), edge_width_px=None if ew is None else round(ew, 2), mean=round(mean, 1),
                clipped_high=round(hi, 4), clipped_low=round(lo, 4), reasons=sorted(set(reasons)))
