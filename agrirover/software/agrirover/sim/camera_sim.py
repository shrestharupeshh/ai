"""Synthetic plant-view renderer with ground truth.

SYNTHETIC DATA. These images exist to exercise and regression-test the vision software (segmentation,
plant association, colour metrics, quality gate) against exact ground truth. They are stylised top views of
lettuce rosettes and say nothing about performance on real greenhouse imagery.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

HEALTH_STATES = ("healthy", "chlorosis", "tipburn", "purpling", "stunted", "missing")
IMG_W, IMG_H = 960, 540
PX_PER_M = 800.0                 # rectified view: 1.2 m x 0.675 m
GRAY_PATCH = (20, 20, 70, 70)    # x0, y0, x1, y1 of the 18 % grey reference card (px)
GRAY_VALUE = 118                 # sRGB value of an 18 % reflectance card


@dataclass
class PlantTruth:
    pid: str
    age_days: float = 21.0
    health: str = "healthy"
    severity: float = 0.0        # 0..1
    seed: int = 0

    def diameter_m(self) -> float:
        """Logistic growth of rosette diameter (illustrative parameters, not a calibrated crop model)."""
        d = 0.03 + 0.21 / (1 + math.exp(-(self.age_days - 18) / 5.0))
        if self.health == "stunted":
            d *= 1 - 0.55 * self.severity
        return d


def slot_px(row: int, col: int, rows: int = 2, cols: int = 4, spacing_m: float = 0.25) -> tuple[int, int]:
    """Nominal pixel position of plant slot (row 0 = nearest the aisle = bottom of the image)."""
    cx = IMG_W / 2 + (col - (cols - 1) / 2) * spacing_m * PX_PER_M
    cy = IMG_H - (0.15 + row * spacing_m) * PX_PER_M
    return int(round(cx)), int(round(cy))


def _leaf_colour(p: PlantTruth, rng, tip: bool = False):
    h, s, v = 55 + rng.normal(0, 3), 170 + rng.normal(0, 12), 150 + rng.normal(0, 14)     # healthy green (OpenCV HSV)
    if p.health == "chlorosis":
        h = h - 28 * p.severity
        s = s - 30 * p.severity
        v = v + 45 * p.severity
    elif p.health == "purpling":
        if rng.random() < 0.35 + 0.6 * p.severity:
            h, s, v = 150 + rng.normal(0, 5), 140, 105
    elif p.health == "tipburn" and tip:
        h, s, v = 14 + rng.normal(0, 2), 150, 95
    hsv = np.uint8([[[np.clip(h, 0, 179), np.clip(s, 0, 255), np.clip(v, 0, 255)]]])
    return tuple(int(c) for c in cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0])


def render_view(plants: list[PlantTruth], rows: int = 2, cols: int = 4, *, gain: float = 1.0,
                cast: tuple[float, float, float] = (1.0, 1.0, 1.0), blur_px: float = 0.0, noise: float = 2.0,
                seed: int = 0, shift_px: tuple[float, float] = (0.0, 0.0)) -> tuple[np.ndarray, dict]:
    """Render one station view. Returns (BGR image, ground truth dict)."""
    rng = np.random.default_rng(seed)
    img = np.full((IMG_H, IMG_W, 3), 172, np.uint8)
    img += rng.normal(0, 3, img.shape).astype(np.int8).view(np.uint8) * 0   # keep bench flat; texture added below
    for r in range(rows):                                   # white NFT channels with dark holes
        _, cy = slot_px(r, 0, rows, cols)
        cv2.rectangle(img, (0, cy - 44), (IMG_W, cy + 44), (226, 226, 226), -1)
        cv2.line(img, (0, cy - 44), (IMG_W, cy - 44), (150, 150, 150), 2)
        cv2.line(img, (0, cy + 44), (IMG_W, cy + 44), (150, 150, 150), 2)
    truth_mask = np.zeros((IMG_H, IMG_W), np.uint8)
    truth = {"plants": []}
    for k, p in enumerate(plants):
        r, c = divmod(k, cols)
        cx, cy = slot_px(r, c, rows, cols)
        prng = np.random.default_rng(p.seed * 7919 + 13)
        cx += int(prng.normal(0, 6))
        cy += int(prng.normal(0, 6))
        if p.health == "missing":
            cv2.circle(img, (cx, cy), 14, (45, 45, 45), -1)
            truth["plants"].append(dict(pid=p.pid, health=p.health, severity=p.severity, cx=cx, cy=cy, area_px=0, diameter_m=0.0))
            continue
        rad = p.diameter_m() / 2 * PX_PER_M
        one = np.zeros((IMG_H, IMG_W), np.uint8)
        n_leaves = int(8 + min(10, p.age_days / 3))
        for layer, scale in ((0, 1.0), (1, 0.68), (2, 0.38)):
            for i in range(n_leaves - 2 * layer):
                a = 2 * math.pi * i / (n_leaves - 2 * layer) + layer * 0.4 + prng.normal(0, 0.08)
                ll = rad * scale * (0.55 + prng.random() * 0.12)
                ctr = (int(cx + math.cos(a) * ll * 0.72), int(cy + math.sin(a) * ll * 0.72))
                axes = (max(2, int(ll * 0.62)), max(2, int(ll * 0.40)))
                col = _leaf_colour(p, prng)
                cv2.ellipse(img, ctr, axes, math.degrees(a), 0, 360, col, -1, cv2.LINE_AA)
                cv2.ellipse(one, ctr, axes, math.degrees(a), 0, 360, 255, -1)
                if p.health == "tipburn" and layer == 0 and prng.random() < 0.3 + 0.7 * p.severity:
                    tip = (int(cx + math.cos(a) * ll * 1.22), int(cy + math.sin(a) * ll * 1.22))
                    cv2.circle(img, tip, max(2, int(ll * (0.16 + 0.22 * p.severity))), _leaf_colour(p, prng, tip=True), -1, cv2.LINE_AA)
                    cv2.circle(one, tip, max(2, int(ll * (0.16 + 0.22 * p.severity))), 255, -1)
        truth_mask = np.maximum(truth_mask, one)
        truth["plants"].append(dict(pid=p.pid, health=p.health, severity=p.severity, cx=cx, cy=cy,
                                    area_px=int((one > 0).sum()), diameter_m=p.diameter_m()))
    if shift_px != (0.0, 0.0):          # robot stop error moves the scene; the reference card moves with the camera
        M = np.float32([[1, 0, shift_px[0]], [0, 1, shift_px[1]]])
        img = cv2.warpAffine(img, M, (IMG_W, IMG_H), borderMode=cv2.BORDER_REPLICATE)
        truth_mask = cv2.warpAffine(truth_mask, M, (IMG_W, IMG_H))
        for t in truth["plants"]:
            t["cx"] += shift_px[0]
            t["cy"] += shift_px[1]
    x0, y0, x1, y1 = GRAY_PATCH
    cv2.rectangle(img, (x0, y0), (x1, y1), (GRAY_VALUE,) * 3, -1)
    f = img.astype(np.float32) * gain * np.array(cast, np.float32)[None, None, :]
    f += rng.normal(0, noise, f.shape)
    img = np.clip(f, 0, 255).astype(np.uint8)
    if blur_px > 0.5:
        k = int(blur_px) | 1
        kern = np.zeros((k, k), np.float32)
        kern[k // 2, :] = 1.0 / k                              # horizontal motion blur
        img = cv2.filter2D(img, -1, kern)
    truth["mask"] = truth_mask
    truth["gain"], truth["cast"], truth["blur_px"] = gain, cast, blur_px
    return img, truth
