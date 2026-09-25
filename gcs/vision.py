"""
Camera input + beacon detection for the last ~20 %.

Sources
  MjpegSource : the ESP32-CAM stream (http://<ip>:81/stream), same reader logic as light_tracker.
  Any object with .latest() -> BGR frame | None, .fps, .connected also works (FakeIRCam in link.py).

Detection (same idea as light_tracker/laptop/tracker.py find_bright_spot, relative mode):
  gray -> blur -> background = median, contrast = peak - background,
  cutoff = background + rel_threshold * contrast; contours >= min_area become
  candidates; brightest one is the beacon. Confidence is derived from contrast
  and blob size so the hand-over rule has a number to work with.

YOLO nano (optional): pass --yolo weights.pt. If `ultralytics` is installed the
model runs on every frame and, when it returns a box, that box centre replaces
the blob centroid and its score becomes the confidence. The IR blob is the
fallback whenever YOLO finds nothing.
"""
import threading
import time
from collections import deque

import cv2
import numpy as np
import requests


class MjpegSource(threading.Thread):
    def __init__(self, url):
        super().__init__(daemon=True)
        self.url = url
        self.frame = None
        self.seq = 0
        self.lock = threading.Lock()
        self.fps = 0.0
        self.connected = False
        self.dropped = 0
        self.error = None
        self._times = deque(maxlen=30)
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            try:
                resp = requests.get(self.url, stream=True, timeout=5)
                self.connected = True
                self.error = None
                buf = b""
                for chunk in resp.iter_content(chunk_size=4096):
                    if self._stop.is_set():
                        break
                    buf += chunk
                    newest, drained = None, 0
                    while True:
                        s = buf.find(b"\xff\xd8"); e = buf.find(b"\xff\xd9", s)
                        if s == -1 or e == -1:
                            break
                        newest = buf[s:e + 2]; buf = buf[e + 2:]; drained += 1
                    if drained > 1:
                        self.dropped += drained - 1
                    if newest is not None:
                        img = cv2.imdecode(np.frombuffer(newest, np.uint8), cv2.IMREAD_COLOR)
                        if img is not None:
                            with self.lock:
                                self.frame = img
                                self.seq += 1
                            self._times.append(time.time())
                            if len(self._times) > 1:
                                self.fps = (len(self._times) - 1) / (self._times[-1] - self._times[0])
                    if len(buf) > 2_000_000:
                        buf = b""
            except Exception as e:  # noqa: BLE001
                self.connected = False
                self.error = str(e)
                time.sleep(2)
        self.connected = False

    def latest(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()


def detect_beacon(frame, cfg):
    """Return dict(found, x, y, radius_px, area, peak, background, contrast, cutoff, conf)."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    k = int(cfg["blur"]) | 1
    blurred = cv2.GaussianBlur(gray, (k, k), 0) if k > 1 else gray
    background = float(np.median(blurred))
    peak = float(blurred.max())
    contrast = peak - background
    out = {"found": False, "x": None, "y": None, "radius_px": 0.0, "area": 0.0, "peak": peak,
           "background": background, "contrast": contrast, "cutoff": None, "conf": 0.0, "source": "ir"}
    if cfg["thresh_mode"] == "absolute":
        cutoff = float(cfg["threshold"])
    else:
        if contrast < cfg["min_contrast"]:
            return out
        cutoff = background + cfg["rel_threshold"] * contrast
    out["cutoff"] = cutoff
    _, mask = cv2.threshold(blurred, cutoff, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in contours:
        area = cv2.contourArea(c)
        if area < cfg["min_area"]:
            continue
        m = cv2.moments(c)
        if m["m00"] <= 0:
            continue
        cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        cmask = np.zeros_like(mask); cv2.drawContours(cmask, [c], -1, 255, -1)
        pk = float(cv2.minMaxLoc(blurred, mask=cmask)[1])
        cand = (pk, area, cx, cy)
        if best is None or cand > best:
            best = cand
    if best is None:
        return out
    pk, area, cx, cy = best
    out.update(found=True, x=float(cx), y=float(cy), area=float(area), radius_px=float(np.sqrt(area / np.pi)),
               conf=float(min(0.97, 0.35 + 0.5 * min(1.0, contrast / 180.0) + 0.15 * min(1.0, area / 400.0))))
    return out


class Vision(threading.Thread):
    def __init__(self, source, cfg, yolo_weights=None):
        super().__init__(daemon=True)
        self.source = source
        self.cfg = cfg
        self.lock = threading.Lock()
        self.result = self._empty()
        self.jpeg = None
        self.frames_seen = 0
        self.last_seen_t = None
        self.proc_ms = 0.0
        self.fps = 0.0
        self._times = deque(maxlen=30)
        self._stop = threading.Event()
        self.yolo = None
        self.yolo_error = None
        if yolo_weights:
            try:
                from ultralytics import YOLO  # type: ignore
                self.yolo = YOLO(yolo_weights)
            except Exception as e:  # noqa: BLE001
                self.yolo_error = "YOLO not loaded (%s); IR blob only" % e

    @staticmethod
    def _empty(w=640, h=480):
        return {"found": False, "state": "searching", "conf": 0.0, "frames": 0, "ex_px": 0.0, "ey_px": 0.0,
                "w": w, "h": h, "radius_px": 0.0, "boxes": [], "lost_s": 0.0, "source": "ir", "x": None, "y": None}

    def stop(self):
        self._stop.set()

    def run(self):
        last_seq = None
        while not self._stop.is_set():
            seq = getattr(self.source, 'seq', None)
            if seq is not None and seq == last_seq:
                time.sleep(0.005)
                continue
            frame = self.source.latest()
            if frame is None:
                with self.lock:
                    self.result = self._empty()
                    self.result["state"] = "none"
                time.sleep(0.05)
                continue
            last_seq = seq
            t0 = time.time()
            h, w = frame.shape[:2]
            det = detect_beacon(frame, self.cfg)
            boxes = []
            if self.yolo is not None:
                try:
                    r = self.yolo.predict(frame, verbose=False, conf=0.25, imgsz=320)[0]
                    if r.boxes is not None and len(r.boxes):
                        b = r.boxes
                        i = int(b.conf.argmax())
                        x1, y1, x2, y2 = [float(v) for v in b.xyxy[i]]
                        boxes = [[x1, y1, x2, y2, float(b.conf[i])]]
                        det.update(found=True, x=(x1 + x2) / 2, y=(y1 + y2) / 2, conf=float(b.conf[i]),
                                   radius_px=max(x2 - x1, y2 - y1) / 2, source="yolo")
                except Exception as e:  # noqa: BLE001
                    self.yolo_error = str(e)
            now = time.time()
            res = self._empty(w, h)
            res.update(found=det["found"], conf=round(det["conf"], 3), radius_px=round(det["radius_px"], 1), boxes=boxes,
                       source=det["source"], x=det["x"], y=det["y"], background=round(det["background"]), contrast=round(det["contrast"]),
                       cutoff=None if det["cutoff"] is None else round(det["cutoff"]))
            if det["found"]:
                res["ex_px"] = round(det["x"] - w / 2, 1)
                res["ey_px"] = round(det["y"] - h / 2, 1)
                self.frames_seen = self.frames_seen + 1 if det["conf"] >= self.cfg["conf_needed"] else 0
                self.last_seen_t = now
                res["state"] = "seen"
            else:
                self.frames_seen = 0
                res["state"] = "searching"
                res["lost_s"] = 0.0 if self.last_seen_t is None else round(now - self.last_seen_t, 2)
            res["frames"] = self.frames_seen
            self.proc_ms = (time.time() - t0) * 1000
            self._times.append(now)
            if len(self._times) > 1:
                self.fps = (len(self._times) - 1) / (self._times[-1] - self._times[0])
            jpeg = self.annotate(frame, res)
            with self.lock:
                self.result = res
                self.jpeg = jpeg

    def annotate(self, frame, res):
        img = frame.copy()
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.line(img, (cx - 16, cy), (cx + 16, cy), (200, 215, 230), 1)
        cv2.line(img, (cx, cy - 16), (cx, cy + 16), (200, 215, 230), 1)
        cv2.circle(img, (cx, cy), int(self.cfg["deadband_px"]), (200, 215, 230), 1)
        if res["found"]:
            x, y = int(res["x"]), int(res["y"]); r = int(max(6, res["radius_px"]))
            cv2.rectangle(img, (x - r - 6, y - r - 6), (x + r + 6, y + r + 6), (92, 154, 255), 2)
            label = "%s %.2f" % ("beacon" if res["source"] == "ir" else "yolo", res["conf"])
            cv2.rectangle(img, (x - r - 6, y - r - 24), (x - r - 6 + 8 * len(label) + 8, y - r - 6), (92, 154, 255), -1)
            cv2.putText(img, label, (x - r - 2, y - r - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.line(img, (cx, cy), (x, y), (255, 211, 127), 1)
            cv2.circle(img, (x, y), 3, (255, 211, 127), -1)
        cv2.putText(img, "IR %dx%d %s" % (w, h, res["state"].upper()), (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 215, 230), 1, cv2.LINE_AA)
        ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return enc.tobytes() if ok else None

    def latest(self):
        with self.lock:
            return dict(self.result), self.jpeg
