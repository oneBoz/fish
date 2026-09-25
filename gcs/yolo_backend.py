"""
YOLO box producer that runs on the laptop and on a Raspberry Pi.

    YoloBackend("yolo11n.pt")    -> ultralytics (laptop; pip install ultralytics)
    YoloBackend("yolo11n.onnx")  -> onnxruntime  (Pi;     pip install onnxruntime)

Both return the same list of boxes [x1, y1, x2, y2, conf, cls] in frame pixels, so the
swarm logic in vision.py does not care which one ran. Export for the Pi with
    yolo export model=yolo11n.pt format=onnx imgsz=320 opset=12
and copy the .onnx (and optionally a sidecar .names file, one class per line) to the Pi.
The same .onnx is the input for the AI HAT / Hailo compiler when you move to that board.
No module here imports the ground station, so this file can be copied to the Pi on its own.
"""
import ast
import os

import cv2
import numpy as np


class YoloBackend:
    def __init__(self, weights, imgsz=320, conf=0.25, iou=0.5, classes=""):
        self.weights = weights
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self.iou = float(iou)
        self.kind = None
        self.names = {}
        self.class_ids = None          # None = every class
        self.error = None
        self.last_ms = 0.0
        ext = os.path.splitext(weights)[1].lower()
        if ext == ".onnx":
            self._init_onnx(weights)
        else:
            self._init_ultralytics(weights)
        self.set_classes(classes)

    # ------------------------------------------------------------------ init
    def _init_ultralytics(self, weights):
        from ultralytics import YOLO  # type: ignore
        self.model = YOLO(weights)
        self.names = dict(self.model.names) if getattr(self.model, "names", None) else {}
        self.kind = "ultralytics"

    def _init_onnx(self, weights):
        import onnxruntime as ort  # type: ignore
        self.sess = ort.InferenceSession(weights, providers=["CPUExecutionProvider"])
        self.input_name = self.sess.get_inputs()[0].name
        shape = self.sess.get_inputs()[0].shape
        if isinstance(shape[-1], int) and shape[-1] > 0:
            self.imgsz = int(shape[-1])
        self.names = {}
        try:
            meta = self.sess.get_modelmeta().custom_metadata_map
            if "names" in meta:
                self.names = {int(k): v for k, v in ast.literal_eval(meta["names"]).items()}
        except Exception:  # noqa: BLE001
            pass
        side = os.path.splitext(weights)[0] + ".names"
        if not self.names and os.path.exists(side):
            with open(side) as f:
                self.names = {i: line.strip() for i, line in enumerate(f) if line.strip()}
        self.kind = "onnxruntime"

    def set_classes(self, spec):
        """'drone,kite' or '0,33' or '' (all). Unknown names are ignored with a note in .error."""
        spec = (spec or "").strip()
        if not spec:
            self.class_ids = None
            return
        ids, unknown = set(), []
        by_name = {str(v).lower(): int(k) for k, v in self.names.items()}
        for tok in spec.split(","):
            tok = tok.strip()
            if not tok:
                continue
            if tok.lstrip("-").isdigit():
                ids.add(int(tok))
            elif tok.lower() in by_name:
                ids.add(by_name[tok.lower()])
            else:
                unknown.append(tok)
        self.class_ids = ids or None
        self.error = ("Unknown class name(s): " + ", ".join(unknown)) if unknown else None

    def label(self, cls):
        return str(self.names.get(int(cls), int(cls)))

    # ------------------------------------------------------------------ detect
    def detect(self, frame_bgr):
        """-> list of [x1, y1, x2, y2, conf, cls] in frame pixels, best first."""
        t0 = cv2.getTickCount()
        try:
            boxes = self._detect_ultralytics(frame_bgr) if self.kind == "ultralytics" else self._detect_onnx(frame_bgr)
        except Exception as e:  # noqa: BLE001
            self.error = "YOLO inference failed: %s" % e
            boxes = []
        self.last_ms = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000.0
        boxes.sort(key=lambda b: -b[4])
        return boxes

    def _detect_ultralytics(self, frame):
        r = self.model.predict(frame, verbose=False, conf=self.conf, iou=self.iou, imgsz=self.imgsz,
                               classes=sorted(self.class_ids) if self.class_ids else None)[0]
        out = []
        if r.boxes is not None and len(r.boxes):
            xyxy = r.boxes.xyxy.cpu().numpy(); cf = r.boxes.conf.cpu().numpy(); cl = r.boxes.cls.cpu().numpy()
            for (x1, y1, x2, y2), c, k in zip(xyxy, cf, cl):
                out.append([float(x1), float(y1), float(x2), float(y2), float(c), int(k)])
        return out

    def _detect_onnx(self, frame):
        h, w = frame.shape[:2]
        s = self.imgsz
        scale = min(s / w, s / h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(frame, (nw, nh))
        canvas = np.full((s, s, 3), 114, np.uint8)
        px, py = (s - nw) // 2, (s - nh) // 2
        canvas[py:py + nh, px:px + nw] = resized
        blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        out = self.sess.run(None, {self.input_name: blob})[0]
        pred = out[0]
        if pred.shape[0] < pred.shape[1]:          # (4+nc, N) -> (N, 4+nc)
            pred = pred.T
        xywh, scores = pred[:, :4], pred[:, 4:]
        cls = scores.argmax(axis=1)
        conf = scores[np.arange(len(scores)), cls]
        keep = conf >= self.conf
        if self.class_ids is not None:
            keep &= np.isin(cls, list(self.class_ids))
        xywh, conf, cls = xywh[keep], conf[keep], cls[keep]
        if not len(conf):
            return []
        x1 = (xywh[:, 0] - xywh[:, 2] / 2 - px) / scale
        y1 = (xywh[:, 1] - xywh[:, 3] / 2 - py) / scale
        bw, bh = xywh[:, 2] / scale, xywh[:, 3] / scale
        rects = [[float(a), float(b), float(c), float(d)] for a, b, c, d in zip(x1, y1, bw, bh)]
        idx = cv2.dnn.NMSBoxes(rects, conf.astype(float).tolist(), self.conf, self.iou)
        idx = [int(i) for i in (np.array(idx).flatten() if len(idx) else [])]
        res = []
        for i in idx:
            a, b, c, d = rects[i]
            res.append([max(0.0, a), max(0.0, b), min(float(w), a + c), min(float(h), b + d), float(conf[i]), int(cls[i])])
        return res


def load_backend(weights, cfg):
    """Return (backend or None, error string or None). Never raises."""
    if not weights:
        return None, None
    try:
        be = YoloBackend(weights, imgsz=cfg.get("yolo_imgsz", 320), conf=cfg.get("yolo_conf", 0.25), classes=cfg.get("yolo_classes", ""))
        return be, be.error
    except ImportError as e:
        need = "onnxruntime" if str(weights).lower().endswith(".onnx") else "ultralytics"
        return None, "YOLO not loaded: pip install %s (%s); IR blob only" % (need, e)
    except Exception as e:  # noqa: BLE001
        return None, "YOLO not loaded (%s); IR blob only" % e
