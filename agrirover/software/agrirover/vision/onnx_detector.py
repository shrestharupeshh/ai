"""EXPERIMENTAL: plant / lesion detector hook for a YOLO-style ONNX model.

No weights are shipped and no accuracy is claimed. The class loads any Ultralytics-exported detection model
(`yolo export format=onnx`), runs it with ONNX Runtime on the CPU and returns boxes with confidences. See
docs/07_computer_vision.md for the data-collection and training workflow that must precede its use.
"""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np


class OnnxDetector:
    def __init__(self, model_path: str | Path, class_names: list[str], conf: float = 0.35, iou: float = 0.5, size: int = 640):
        import onnxruntime as ort
        self.sess = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0].name
        self.names, self.conf, self.iou, self.size = class_names, conf, iou, size

    @staticmethod
    def letterbox(img: np.ndarray, size: int) -> tuple[np.ndarray, float, tuple[int, int]]:
        h, w = img.shape[:2]
        s = size / max(h, w)
        nh, nw = int(round(h * s)), int(round(w * s))
        out = np.full((size, size, 3), 114, np.uint8)
        top, left = (size - nh) // 2, (size - nw) // 2
        out[top:top + nh, left:left + nw] = cv2.resize(img, (nw, nh))
        return out, s, (left, top)

    def detect(self, img_bgr: np.ndarray) -> list[dict]:
        lb, s, (left, top) = self.letterbox(img_bgr, self.size)
        x = lb[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        pred = self.sess.run(None, {self.inp: x})[0]            # (1, 4 + nc, N) for YOLOv8-style heads
        return self.postprocess(pred, s, left, top)

    def postprocess(self, pred: np.ndarray, s: float, left: int, top: int) -> list[dict]:
        p = pred[0].T                                           # (N, 4 + nc)
        scores = p[:, 4:]
        cls = scores.argmax(1)
        conf = scores.max(1)
        keep = conf >= self.conf
        p, cls, conf = p[keep], cls[keep], conf[keep]
        boxes = np.column_stack(((p[:, 0] - p[:, 2] / 2 - left) / s, (p[:, 1] - p[:, 3] / 2 - top) / s, p[:, 2] / s, p[:, 3] / s))
        idx = cv2.dnn.NMSBoxes(boxes.tolist(), conf.tolist(), self.conf, self.iou)
        out = []
        for i in np.array(idx).reshape(-1):
            out.append(dict(box_xywh=[round(float(v), 1) for v in boxes[i]], confidence=round(float(conf[i]), 3),
                            cls=self.names[int(cls[i])] if int(cls[i]) < len(self.names) else str(int(cls[i])),
                            level="EXPERIMENTAL"))
        return out
