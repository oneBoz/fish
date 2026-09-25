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

Swarm mode (design section 15): every source (IR blob or YOLO box) is kept,
sources closer than `cluster_radius` px are one swarm, swarms keep persistent
IDs (SwarmTracker) and guidance follows the mass-weighted centroid of the
selected swarm, or the biggest one when none is selected.

YOLO (optional): pass --yolo weights.pt (ultralytics, laptop) or weights.onnx
(onnxruntime, Raspberry Pi). See yolo_backend.py. Boxes become sources next to
the IR blobs; in beacon mode the best box wins over the brightest blob.

Standalone check (also on a Pi):  python -m gcs.vision --image frame.jpg --yolo w.onnx --mode swarm
"""
import threading
import time
from collections import deque

import cv2
import numpy as np
import requests

from .yolo_backend import load_backend


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


def _source_conf(peak, background, area):
    return float(min(0.97, 0.35 + 0.5 * min(1.0, (peak - background) / 180.0) + 0.15 * min(1.0, area / 400.0)))


def detect_sources(frame, cfg):
    """Every light source in the frame (IR blob detector, relative or absolute cutoff).
    -> dict(sources=[{x, y, area, peak, mass, conf, src}], background, contrast, cutoff)"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    k = int(cfg["blur"]) | 1
    blurred = cv2.GaussianBlur(gray, (k, k), 0) if k > 1 else gray
    background = float(np.median(blurred))
    peak = float(blurred.max())
    contrast = peak - background
    out = {"sources": [], "background": background, "contrast": contrast, "cutoff": None}
    if cfg["thresh_mode"] == "absolute":
        cutoff = float(cfg["threshold"])
    else:
        if contrast < cfg["min_contrast"]:
            return out
        cutoff = background + cfg["rel_threshold"] * contrast
    out["cutoff"] = cutoff
    _, mask = cv2.threshold(blurred, cutoff, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cands = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < cfg["min_area"]:
            continue
        m = cv2.moments(c)
        if m["m00"] <= 0:
            continue
        x, y, w, h = cv2.boundingRect(c)
        roi = blurred[y:y + h, x:x + w]
        pk = float(roi.max()) if roi.size else peak
        cands.append({"x": m["m10"] / m["m00"], "y": m["m01"] / m["m00"], "area": float(area), "peak": pk,
                      "mass": float(roi.astype(np.float32).sum()), "conf": _source_conf(pk, background, area), "src": "ir"})
    cands.sort(key=lambda d: (-d["peak"], -d["mass"]))
    out["sources"] = cands[: int(cfg.get("max_sources", 32))]
    return out


def detect_beacon(frame, cfg):
    """Single brightest source, kept for callers of the old API."""
    d = detect_sources(frame, cfg)
    out = {"found": False, "x": None, "y": None, "radius_px": 0.0, "area": 0.0, "peak": d["background"] + d["contrast"],
           "background": d["background"], "contrast": d["contrast"], "cutoff": d["cutoff"], "conf": 0.0, "source": "ir"}
    if d["sources"]:
        b = d["sources"][0]
        out.update(found=True, x=b["x"], y=b["y"], area=b["area"], radius_px=float(np.sqrt(b["area"] / np.pi)), conf=b["conf"])
    return out


def cluster_sources(sources, radius):
    """Single-linkage clustering: two sources are in one swarm if within `radius` px of each other,
    directly or through a chain (radius 0 = everything is one swarm). Returns swarms sorted by mass."""
    n = len(sources)
    if n == 0:
        return []
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    if radius <= 0:
        for i in range(1, n):
            parent[find(i)] = find(0)
    else:
        r2 = radius * radius
        for i in range(n):
            for j in range(i + 1, n):
                dx = sources[i]["x"] - sources[j]["x"]; dy = sources[i]["y"] - sources[j]["y"]
                if dx * dx + dy * dy <= r2:
                    parent[find(i)] = find(j)
    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)
    swarms = []
    for members in groups.values():
        w = np.array([max(1e-6, sources[i]["mass"]) for i in members])
        xs = np.array([sources[i]["x"] for i in members]); ys = np.array([sources[i]["y"] for i in members])
        cx, cy = float((w * xs).sum() / w.sum()), float((w * ys).sum() / w.sum())
        spread = float(np.sqrt((w * ((xs - cx) ** 2 + (ys - cy) ** 2)).sum() / w.sum()))
        pts = np.array([[x, y] for x, y in zip(xs, ys)], np.float32)
        hull = cv2.convexHull(pts).reshape(-1, 2).tolist() if len(pts) >= 3 else pts.tolist()
        swarms.append({"x": cx, "y": cy, "n": len(members), "mass": float(w.sum()), "spread": spread, "hull": hull,
                       "conf": float(np.mean([sources[i]["conf"] for i in members])), "members": members})
    swarms.sort(key=lambda s: -s["mass"])
    return swarms


class SwarmTracker:
    """Persistent swarm IDs across frames plus the operator's selection (design section 15)."""
    HOLD_FRAMES = 10

    def __init__(self):
        self.tracks = {}          # id -> {x, y, missed}
        self.next_id = 1
        self.selected = None      # id chosen by the operator, None = biggest
        self.last_followed = None # {id, x, y, n, spread, ...} of the swarm last followed
        self.hold = 0

    def reset(self):
        self.__init__()

    def update(self, swarms, match_px):
        """Assign IDs in place (swarm['id']) and return the swarm to follow with a state:
        ('ok', swarm) | ('hold', last) | ('none', None)."""
        unmatched = dict(self.tracks)
        for sw in sorted(swarms, key=lambda s: -s["mass"]):
            best, bd = None, match_px * match_px
            for tid, tr in unmatched.items():
                d2 = (tr["x"] - sw["x"]) ** 2 + (tr["y"] - sw["y"]) ** 2
                if d2 < bd:
                    best, bd = tid, d2
            if best is None:
                best = self.next_id; self.next_id += 1
            else:
                unmatched.pop(best)
            sw["id"] = best
            self.tracks[best] = {"x": sw["x"], "y": sw["y"], "missed": 0}
        for tid in unmatched:
            self.tracks[tid]["missed"] += 1
            if self.tracks[tid]["missed"] > self.HOLD_FRAMES:
                self.tracks.pop(tid)
                if self.selected == tid:
                    self.selected = None
        chosen = None
        if self.selected is not None:
            chosen = next((s for s in swarms if s["id"] == self.selected), None)
            if chosen is None and self.selected in self.tracks and self.last_followed and self.hold < self.HOLD_FRAMES:
                self.hold += 1
                return "hold", self.last_followed
        if chosen is None and swarms:
            chosen = swarms[0]
        if chosen is None:
            self.hold = 0
            return "none", None
        self.hold = 0
        self.last_followed = {k: v for k, v in chosen.items() if k != "members"}
        return "ok", chosen

    def visible_ids(self):
        return sorted(tid for tid, tr in self.tracks.items() if tr["missed"] == 0)

    def select(self, tid):
        self.selected = None if tid is None else int(tid)

    def step(self, direction, swarms):
        ids = [s["id"] for s in sorted(swarms, key=lambda s: s["id"])]
        if not ids:
            return None
        cur = self.selected if self.selected in ids else (self.last_followed or {}).get("id")
        i = ids.index(cur) if cur in ids else -1
        self.selected = ids[(i + direction) % len(ids)]
        return self.selected

    def pick_at(self, x, y, swarms):
        if not swarms:
            return None
        best = min(swarms, key=lambda s: (s["x"] - x) ** 2 + (s["y"] - y) ** 2)
        self.selected = best["id"]
        return best["id"]


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
        self.tracker = SwarmTracker()
        self._swarms = []
        self._classes_applied = cfg.get("yolo_classes", "")
        self.yolo, self.yolo_error = load_backend(yolo_weights, cfg)

    @staticmethod
    def _empty(w=640, h=480):
        return {"found": False, "state": "searching", "conf": 0.0, "frames": 0, "ex_px": 0.0, "ey_px": 0.0,
                "w": w, "h": h, "radius_px": 0.0, "boxes": [], "lost_s": 0.0, "source": "ir", "x": None, "y": None,
                "mode": "beacon", "n_sources": 0, "sources": [], "swarms": [], "swarm_id": None, "swarm_n": 0}

    def stop(self):
        self._stop.set()

    # ---- operator selection (called from the station) ----
    def select(self, d):
        with self.lock:
            swarms = list(self._swarms)
            if "id" in d and d["id"] is not None:
                self.tracker.select(d["id"]); chosen = self.tracker.selected
            elif d.get("action") == "next":
                chosen = self.tracker.step(+1, swarms)
            elif d.get("action") == "prev":
                chosen = self.tracker.step(-1, swarms)
            elif d.get("action") == "auto":
                self.tracker.select(None); chosen = None
            elif "x" in d and "y" in d:
                chosen = self.tracker.pick_at(float(d["x"]), float(d["y"]), swarms)
            else:
                return {"ok": False, "error": "Give id, action (next/prev/auto) or x,y."}
            return {"ok": True, "selected": chosen, "visible": [s["id"] for s in swarms]}

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
                    self.result["mode"] = self.cfg.get("vision_mode", "beacon")
                time.sleep(0.05)
                continue
            last_seq = seq
            t0 = time.time()
            h, w = frame.shape[:2]
            mode = self.cfg.get("vision_mode", "beacon")
            det = detect_sources(frame, self.cfg)
            sources = list(det["sources"])
            boxes = []
            if self.yolo is not None:
                if self.cfg.get("yolo_classes", "") != self._classes_applied:
                    self._classes_applied = self.cfg.get("yolo_classes", ""); self.yolo.set_classes(self._classes_applied)
                self.yolo.conf = float(self.cfg.get("yolo_conf", 0.25))
                boxes = self.yolo.detect(frame)
                if self.yolo.error:
                    self.yolo_error = self.yolo.error
                for x1, y1, x2, y2, c, k in boxes:
                    area = max(1.0, (x2 - x1) * (y2 - y1))
                    sources.append({"x": (x1 + x2) / 2, "y": (y1 + y2) / 2, "area": area, "peak": 255.0 * c, "mass": area * c,
                                    "conf": float(c), "src": "yolo", "cls": self.yolo.label(k), "box": [x1, y1, x2, y2]})
            now = time.time()
            res = self._empty(w, h)
            res.update(mode=mode, boxes=[[round(v, 1) for v in b[:4]] + [round(b[4], 3), self.yolo.label(b[5]) if self.yolo else b[5]] for b in boxes],
                       background=round(det["background"]), contrast=round(det["contrast"]), cutoff=None if det["cutoff"] is None else round(det["cutoff"]),
                       n_sources=len(sources), sources=[{"x": round(s["x"], 1), "y": round(s["y"], 1), "r": round(float(np.sqrt(s["area"] / np.pi)), 1), "src": s["src"]} for s in sources[:32]])
            target = None
            if mode == "swarm":
                swarms = cluster_sources(sources, float(self.cfg.get("cluster_radius", 80)))
                with self.lock:
                    state, chosen = self.tracker.update(swarms, float(self.cfg.get("swarm_match", 60)))
                    self._swarms = swarms
                res["swarms"] = [{"id": s["id"], "x": round(s["x"], 1), "y": round(s["y"], 1), "n": s["n"], "spread": round(s["spread"], 1),
                                  "hull": [[round(a, 1), round(b, 1)] for a, b in s["hull"]], "selected": chosen is not None and s["id"] == chosen.get("id")} for s in swarms]
                if chosen is not None:
                    target = {"x": chosen["x"], "y": chosen["y"], "radius_px": max(6.0, chosen["spread"]),
                              "conf": float(min(0.97, 0.3 + 0.12 * min(5, chosen["n"]) + 0.4 * chosen["conf"])), "source": "swarm"}
                    res["swarm_id"] = chosen.get("id"); res["swarm_n"] = chosen.get("n", 0)
                    if state == "hold":
                        res["state"] = "hold"
            else:
                with self.lock:
                    self._swarms = []
                ylist = [s for s in sources if s["src"] == "yolo"]
                best = ylist[0] if ylist else (sources[0] if sources else None)
                if best is not None:
                    target = {"x": best["x"], "y": best["y"], "radius_px": float(np.sqrt(best["area"] / np.pi)) if best["src"] == "ir" else max(best["box"][2] - best["box"][0], best["box"][3] - best["box"][1]) / 2,
                              "conf": best["conf"], "source": best["src"]}
            if target is not None:
                res.update(found=True, x=round(target["x"], 1), y=round(target["y"], 1), conf=round(target["conf"], 3), radius_px=round(target["radius_px"], 1), source=target["source"])
                res["ex_px"] = round(target["x"] - w / 2, 1)
                res["ey_px"] = round(target["y"] - h / 2, 1)
                self.frames_seen = self.frames_seen + 1 if target["conf"] >= self.cfg["conf_needed"] else 0
                self.last_seen_t = now
                if res["state"] != "hold":
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
        grey, dim, hot, cyan = (150, 160, 170), (150, 90, 170), (92, 154, 255), (255, 211, 127)
        for s in res["sources"]:
            cv2.circle(img, (int(s["x"]), int(s["y"])), int(max(4, s["r"] + 3)), grey, 1)
        for b in res["boxes"]:
            x1, y1, x2, y2 = [int(v) for v in b[:4]]
            cv2.rectangle(img, (x1, y1), (x2, y2), hot, 1)
            cv2.putText(img, "%s %.2f" % (b[5], b[4]), (x1, max(10, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, hot, 1, cv2.LINE_AA)
        for sw in res["swarms"]:
            col = hot if sw["selected"] else dim
            pts = np.array(sw["hull"], np.int32).reshape(-1, 1, 2)
            if len(pts) >= 2:
                cv2.polylines(img, [pts], True, col, 2 if sw["selected"] else 1)
            if sw["selected"]:
                for p in sw["hull"]:
                    cv2.line(img, (int(sw["x"]), int(sw["y"])), (int(p[0]), int(p[1])), col, 1)
                cv2.circle(img, (int(sw["x"]), int(sw["y"])), int(max(6, sw["spread"])), col, 1)
            cv2.putText(img, "S%d x%d" % (sw["id"], sw["n"]), (int(sw["x"]) + 8, int(sw["y"]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
        cv2.line(img, (cx - 16, cy), (cx + 16, cy), (200, 215, 230), 1)
        cv2.line(img, (cx, cy - 16), (cx, cy + 16), (200, 215, 230), 1)
        cv2.circle(img, (cx, cy), int(self.cfg["deadband_px"]), (200, 215, 230), 1)
        if res["found"]:
            x, y = int(res["x"]), int(res["y"]); r = int(max(6, res["radius_px"]))
            if res["mode"] != "swarm":
                cv2.rectangle(img, (x - r - 6, y - r - 6), (x + r + 6, y + r + 6), hot, 2)
            label = "%s %.2f" % ({"ir": "beacon", "yolo": "yolo", "swarm": "swarm S%s" % res["swarm_id"]}.get(res["source"], res["source"]), res["conf"])
            cv2.rectangle(img, (x - r - 6, y - r - 24), (x - r - 6 + 8 * len(label) + 8, y - r - 6), hot, -1)
            cv2.putText(img, label, (x - r - 2, y - r - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.line(img, (cx, cy), (x, y), cyan, 1)
            cv2.circle(img, (x, y), 3, cyan, -1)
        cv2.putText(img, "IR %dx%d %s %s %d src" % (w, h, res["mode"].upper(), res["state"].upper(), res["n_sources"]), (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 215, 230), 1, cv2.LINE_AA)
        ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return enc.tobytes() if ok else None

    def latest(self):
        with self.lock:
            return dict(self.result), self.jpeg


# ---------------------------------------------------------------------------
#  Standalone check: python -m gcs.vision --image frame.jpg [--yolo w.onnx] [--mode swarm]
# ---------------------------------------------------------------------------
def _main(argv=None):
    import argparse
    import json
    p = argparse.ArgumentParser(description="Run the detector on one image or an MJPEG stream and print the result")
    p.add_argument("--image", help="a still frame (jpg/png)")
    p.add_argument("--stream", help="MJPEG URL, e.g. http://172.20.10.13:81/stream")
    p.add_argument("--yolo", default=None)
    p.add_argument("--mode", choices=["beacon", "swarm"], default="swarm")
    p.add_argument("--classes", default="")
    p.add_argument("--frames", type=int, default=20)
    p.add_argument("--save", help="write the annotated frame here")
    a = p.parse_args(argv)
    cfg = {"blur": 9, "min_area": 4, "thresh_mode": "relative", "rel_threshold": 0.5, "min_contrast": 40, "threshold": 150,
           "max_sources": 32, "vision_mode": a.mode, "cluster_radius": 80, "swarm_match": 60, "conf_needed": 0.6,
           "deadband_px": 6, "yolo_conf": 0.25, "yolo_imgsz": 320, "yolo_classes": a.classes}

    class Still:
        def __init__(self, img): self.img, self.seq, self.fps, self.connected = img, 0, 0.0, True
        def latest(self): self.seq += 1; return None if self.img is None else self.img.copy()

    if a.image:
        src = Still(cv2.imread(a.image))
        if src.img is None:
            raise SystemExit("cannot read " + a.image)
    elif a.stream:
        src = MjpegSource(a.stream); src.start()
    else:
        raise SystemExit("give --image or --stream")
    v = Vision(src, cfg, yolo_weights=a.yolo)
    if v.yolo_error:
        print("note:", v.yolo_error)
    v.start()
    n = 0
    t_end = time.time() + 30
    while n < a.frames and time.time() < t_end:
        time.sleep(0.05)
        res, jpeg = v.latest()
        if res["state"] == "none" or not res["n_sources"] and res["state"] == "searching" and n == 0 and time.time() < t_end - 29:
            continue
        n += 1
        print(json.dumps({k: res[k] for k in ("mode", "state", "n_sources", "found", "x", "y", "conf", "swarm_id", "swarm_n")}),
              "yolo %.0f ms" % v.yolo.last_ms if v.yolo else "")
    if a.save and v.jpeg:
        open(a.save, "wb").write(v.jpeg); print("saved", a.save)
    v.stop()


if __name__ == "__main__":
    _main()
