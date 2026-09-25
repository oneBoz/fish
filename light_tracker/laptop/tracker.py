"""
Light-source tracker — laptop compute unit.

Pipeline (per frame):
  1. Pull MJPEG frames from the ESP32-CAM           (MjpegReader thread)
  2. Find every light source with OpenCV            (find_bright_spot)
     swarm mode: group sources into clusters        (cluster_sources)
     and follow the one the user selected           (SwarmTracker)
  3. Convert pixel error -> an actuator command:
       --output pantilt   pan/tilt bracket angles          (Controller)
       --output fins      yaw/pitch fin deflections        (FinController)
  4. Send "a,b\n" over UDP to the servo / fin ESP32        (ServoLink)
  5. Serve an annotated stream + live numbers to the browser dashboard (Flask)

Run:   python tracker.py --cam 192.168.1.50 --servo 192.168.1.51                # pan/tilt bracket
Fins:  python tracker.py --cam 172.20.10.13 --servo 172.20.10.12 --output fins  # fish (pid_fins.ino)
Fake:  python tracker.py --cam fake --fake-swarms 3     (no hardware needed)
Open:  http://localhost:9000
"""

import argparse
import json
import socket
import threading
import time
from collections import deque

import cv2
import numpy as np
import requests
from flask import Flask, Response, jsonify, render_template, request

# =====================================================================
#  1. MJPEG reader
# =====================================================================
class MjpegReader(threading.Thread):
    """Continuously decodes the ESP32-CAM multipart stream into the latest frame."""

    def __init__(self, url):
        super().__init__(daemon=True)
        self.url = url
        self.frame = None
        self.seq = 0                      # increments on every new decoded frame
        self.new_frame = threading.Event()
        self.lock = threading.Lock()
        self.fps = 0.0
        self.connected = False
        self.dropped = 0
        self._times = deque(maxlen=30)

    def run(self):
        while True:
            try:
                resp = requests.get(self.url, stream=True, timeout=5)
                self.connected = True
                buf = b""
                for chunk in resp.iter_content(chunk_size=4096):
                    buf += chunk

                    # Drain every complete JPEG currently sitting in the buffer,
                    # but only decode the newest one. If CV/Flask processing on
                    # the main thread ever falls behind the camera's frame
                    # rate, older frames pile up here — decoding them in order
                    # would make the displayed frame drift further and further
                    # behind real time. Instead we throw away the backlog and
                    # keep only what just arrived, so the feed stays "live"
                    # even if some frames are skipped.
                    newest_jpg = None
                    drained = 0
                    while True:
                        start = buf.find(b"\xff\xd8")      # JPEG SOI
                        end = buf.find(b"\xff\xd9", start)  # JPEG EOI
                        if start == -1 or end == -1:
                            break
                        newest_jpg = buf[start:end + 2]
                        buf = buf[end + 2:]
                        drained += 1
                    if drained > 1:
                        self.dropped += drained - 1

                    if newest_jpg is not None:
                        img = cv2.imdecode(np.frombuffer(newest_jpg, np.uint8), cv2.IMREAD_COLOR)
                        if img is not None:
                            with self.lock:
                                self.frame = img
                                self.seq += 1
                            self.new_frame.set()
                            self._times.append(time.time())
                            if len(self._times) > 1:
                                self.fps = (len(self._times) - 1) / (self._times[-1] - self._times[0])
                    if len(buf) > 2_000_000:   # runaway buffer guard
                        buf = b""
            except Exception as e:
                self.connected = False
                print(f"[cam] stream error: {e} — retrying in 2s")
                time.sleep(2)

    def latest(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def wait_new(self, timeout=0.5):
        """Block until a frame newer than the last one handed out arrives.
        Returns the frame copy, or None on timeout. Guarantees the control
        loop runs exactly once per camera frame, so a servo step really is
        'per frame' and not 'per spin of the loop'."""
        if not self.new_frame.wait(timeout):
            return None
        self.new_frame.clear()
        return self.latest()


class FakeCam(threading.Thread):
    """
    Synthetic stand-in for the ESP32-CAM so the whole pipeline (detection,
    clustering, swarm IDs, controller, dashboard) can be exercised with no
    hardware. Drop-in for MjpegReader: same frame / seq / new_frame / fps /
    connected / dropped attributes and the same latest() / wait_new() calls.

    The scene is `n_swarms` swarms of lights living in *angle space* (pan,
    tilt in degrees, the frame the servos work in). Each swarm wanders
    slowly and randomly inside the servo range and bounces off its edges;
    its lights sit at fixed offsets around the swarm centre (within
    `swarm_radius_deg`) and twinkle a little. Swarms start well apart so the
    clustering step sees them as separate groups.

    The camera renders whatever lies inside its field of view around the
    current (pan, tilt) that the tracker feeds back through `set_view()`,
    so when the controller pans toward a swarm the swarm really does slide
    to the centre of the picture: the control loop is closed.
    """

    def __init__(self, n_swarms=3, lights_per_swarm=4, width=640, height=480,
                 fps=20.0, hfov=62.0, vfov=49.0, swarm_radius_deg=3.0,
                 speed_deg_s=3.0, area_deg=(50.0, 30.0), seed=None):
        super().__init__(daemon=True)
        self.w, self.h = width, height
        self.period = 1.0 / max(1.0, fps)
        self.hfov, self.vfov = hfov, vfov
        self.speed = speed_deg_s
        self.rng = np.random.default_rng(seed)
        # region the swarms roam in: `area_deg` (pan x tilt) centred on the
        # home position, clipped to what the servos can reach. The default
        # fits inside one field of view so every swarm is visible from home
        # and can be picked on the dashboard; make it bigger to see the
        # camera hunt for a swarm that has wandered out of the picture.
        self.pan_lim = (max(PAN_RANGE[0] + 4, PAN_HOME - area_deg[0] / 2),
                        min(PAN_RANGE[1] - 4, PAN_HOME + area_deg[0] / 2))
        self.tilt_lim = (max(TILT_RANGE[0] + 4, TILT_HOME - area_deg[1] / 2),
                         min(TILT_RANGE[1] - 4, TILT_HOME + area_deg[1] / 2))
        self.frame = None
        self.seq = 0
        self.new_frame = threading.Event()
        self.lock = threading.Lock()
        self.fps = 0.0
        self.connected = False
        self.dropped = 0
        self._times = deque(maxlen=30)
        self.view_pan, self.view_tilt = PAN_HOME, TILT_HOME
        n_swarms, lights_per_swarm = max(1, n_swarms), max(1, lights_per_swarm)
        self.swarms = [self._make_swarm(i, n_swarms, lights_per_swarm, swarm_radius_deg)
                       for i in range(n_swarms)]

    # ---- scene ---------------------------------------------------------
    def _make_swarm(self, i, n, n_lights, radius_deg):
        # spread the starting centres evenly across the roaming area, at
        # alternating heights, so no two swarms start on top of each other
        pan_lo, pan_hi = self.pan_lim
        tilt_lo, tilt_hi = self.tilt_lim
        pan = pan_lo + (pan_hi - pan_lo) * (i + 0.5) / n
        tilt = tilt_lo + (tilt_hi - tilt_lo) * (0.3 if i % 2 == 0 else 0.7)
        ang = self.rng.uniform(0, 2 * np.pi)
        lights = []
        for _ in range(n_lights):
            r = radius_deg * np.sqrt(self.rng.uniform(0.15, 1.0))
            a = self.rng.uniform(0, 2 * np.pi)
            lights.append({
                "dpan": float(r * np.cos(a)), "dtilt": float(r * np.sin(a)),
                "radius": int(self.rng.integers(7, 13)),
                "bright": float(self.rng.uniform(225, 255)),
            })
        return {"pan": pan, "tilt": tilt,
                "vpan": self.speed * np.cos(ang), "vtilt": self.speed * np.sin(ang),
                "lights": lights}

    def _step(self, dt):
        for sw in self.swarms:
            # random walk on the velocity, capped at `speed`
            sw["vpan"] += self.rng.normal(0, self.speed * 0.6) * dt
            sw["vtilt"] += self.rng.normal(0, self.speed * 0.6) * dt
            v = float(np.hypot(sw["vpan"], sw["vtilt"]))
            if v > self.speed:
                sw["vpan"] *= self.speed / v
                sw["vtilt"] *= self.speed / v
            sw["pan"] += sw["vpan"] * dt
            sw["tilt"] += sw["vtilt"] * dt
            # bounce off the roaming area so every swarm stays reachable
            if not self.pan_lim[0] <= sw["pan"] <= self.pan_lim[1]:
                sw["vpan"] = -sw["vpan"]
                sw["pan"] = float(np.clip(sw["pan"], *self.pan_lim))
            if not self.tilt_lim[0] <= sw["tilt"] <= self.tilt_lim[1]:
                sw["vtilt"] = -sw["vtilt"]
                sw["tilt"] = float(np.clip(sw["tilt"], *self.tilt_lim))

    def _render(self):
        w, h = self.w, self.h
        cx, cy = w / 2.0, h / 2.0
        img = np.full((h, w, 3), (10, 8, 6), np.uint8)                          # dark room
        noise = np.repeat(self.rng.integers(0, 10, (h, w, 1), dtype=np.uint8), 3, axis=2)
        img = cv2.add(img, noise)                                                # sensor noise
        for sw in self.swarms:
            for lt in sw["lights"]:
                # camera pans right -> world slides left; tilts up -> world slides down
                px = cx + (sw["pan"] + lt["dpan"] - self.view_pan) / (self.hfov / 2) * cx
                py = cy + (self.view_tilt - sw["tilt"] - lt["dtilt"]) / (self.vfov / 2) * cy
                if px < -30 or px > w + 30 or py < -30 or py > h + 30:
                    continue
                b = lt["bright"] * self.rng.uniform(0.9, 1.0)         # twinkle
                col = (int(b * 0.85), int(b * 0.95), int(b))           # warm white, BGR
                cv2.circle(img, (int(px), int(py)), lt["radius"], col, -1, cv2.LINE_AA)
        glow = cv2.GaussianBlur(img, (0, 0), 4)
        return cv2.addWeighted(img, 1.0, glow, 0.5, 0)                # bloom

    # ---- MjpegReader interface -----------------------------------------
    def set_view(self, pan, tilt):
        """Where the (fake) servos are pointing the camera right now."""
        self.view_pan, self.view_tilt = float(pan), float(tilt)

    def run(self):
        self.connected = True
        last = time.time()
        while True:
            t = time.time()
            self._step(min(t - last, 0.2))
            last = t
            img = self._render()
            with self.lock:
                self.frame = img
                self.seq += 1
            self.new_frame.set()
            self._times.append(t)
            if len(self._times) > 1:
                self.fps = (len(self._times) - 1) / (self._times[-1] - self._times[0])
            time.sleep(max(0.0, self.period - (time.time() - t)))

    def latest(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def wait_new(self, timeout=0.5):
        if not self.new_frame.wait(timeout):
            return None
        self.new_frame.clear()
        return self.latest()


# =====================================================================
#  2. Computer vision
# =====================================================================
def find_bright_spot(frame, blur_ksize=9, threshold=150, min_area=4,
                     prev=None, lock_radius=40, max_sources=32, mode="single",
                     cluster_radius=80, thresh_mode="relative", rel_threshold=0.5,
                     min_contrast=40):
    """
    Detects EVERY light source that stands out from the background and
    reduces them to ONE target (x, y) for the controller.

    Detection:
      grayscale -> Gaussian blur (kills hot pixels) -> cutoff -> contours.
      Each blob >= `min_area` becomes a candidate with its intensity-weighted
      centroid, area, peak brightness and `mass` (sum of pixel intensities).
      Candidates are sorted brightest first and capped at `max_sources`.

    The cutoff (`thresh_mode`):
      "relative" (default, meant for an IR-filtered camera where the scene is
        dark and only the IR sources show):
          background = median of the blurred frame     (what "dark" is now)
          contrast   = brightest pixel - background    (how much light there is)
          cutoff     = background + rel_threshold * contrast
        i.e. a pixel counts as light when it is `rel_threshold` of the way
        from the background level to the brightest pixel in the frame. This
        follows exposure and distance automatically: a far, dim source with
        nothing brighter in view is still found. If contrast < `min_contrast`
        the frame is treated as empty, so noise in an all-dark frame is never
        promoted to a target.
      "absolute": the old behaviour, cutoff = `threshold` (0-255).

    mode == "single"  (follow one light):
      * If `prev` (last frame's target) is given and a candidate lies within
        `lock_radius` px of it, take the nearest one -> "sticky". This stops a
        second light that enters the frame from stealing the lock.
      * Otherwise take the candidate with the highest peak brightness.

    mode == "swarm"  (follow a group of lights):
      * Candidates are grouped into swarms by `cluster_sources`: two sources
        belong to the same swarm when they are within `cluster_radius` px of
        each other (chains count, so a line of lights is one swarm).
        `cluster_radius` <= 0 puts every source in one swarm.
      * Each swarm gets a mass-weighted centroid (brighter/bigger lights pull
        harder), `spread` (mass-weighted RMS radius, px) and `hull` (convex
        hull of its sources) for the dashboard.
      * No target is picked here: `result["swarms"]` is handed to a
        SwarmTracker which assigns persistent IDs and applies the user's
        selection, so `found` stays False for now.

    Either mode: if no blob is big enough but the frame is still "lit",
    fall back to the raw brightest pixel (tiny/distant sources).

    `candidates` in the result is contour-free and JSON-safe for telemetry.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    k = blur_ksize | 1  # kernel must be odd
    blurred = cv2.GaussianBlur(gray, (k, k), 0)

    _, max_val, _, max_loc = cv2.minMaxLoc(blurred)

    # Background level = median of a 4x subsampled blurred frame (a few
    # thousand pixels, microseconds). The median ignores the light sources
    # themselves as long as they cover well under half the picture.
    background = float(np.median(blurred[::4, ::4]))
    contrast = float(max_val) - background
    if thresh_mode == "relative":
        cutoff = background + float(rel_threshold) * contrast
        lit = contrast >= min_contrast
    else:
        cutoff = float(threshold)
        lit = max_val >= cutoff

    result = {
        "found": False,
        "x": None, "y": None,
        "max_val": float(max_val),
        "background": background,     # median brightness of the frame
        "contrast": contrast,         # brightest pixel above background
        "cutoff": cutoff,             # brightness cutoff actually used this frame
        "thresh_mode": thresh_mode,
        "method": "none",
        "area": 0,
        "contour": None,
        "mask": None,
        "candidates": [],
        "n_sources": 0,
        "mode": mode,
        "spread": 0.0,     # swarm: mass-weighted RMS radius around the centroid, px
        "hull": None,      # swarm: convex hull of source centroids (np array, for drawing)
        "swarms": [],      # swarm: every cluster found this frame (see cluster_sources)
        "swarm_id": None,  # swarm: ID of the swarm being followed
    }

    if not lit:
        return result, blurred

    # ---- detect all blobs ------------------------------------------------
    _, mask = cv2.threshold(blurred, cutoff, 255, cv2.THRESH_BINARY)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=2)
    result["mask"] = mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cands = []
    blob_mask = np.zeros_like(mask)
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        blob_mask[:] = 0
        cv2.drawContours(blob_mask, [c], -1, 255, -1)
        m = cv2.moments(cv2.bitwise_and(blurred, blurred, mask=blob_mask))
        if m["m00"] <= 0:
            continue
        _, peak, _, _ = cv2.minMaxLoc(blurred, mask=blob_mask)
        cands.append({
            "x": m["m10"] / m["m00"],
            "y": m["m01"] / m["m00"],
            "area": float(area),
            "peak": float(peak),
            "mass": float(m["m00"]),   # total intensity: the blob's "pull" in swarm mode
            "contour": c,
        })

    # brightest first, then biggest; cap so telemetry stays small
    cands.sort(key=lambda d: (d["peak"], d["area"]), reverse=True)
    cands = cands[:max_sources]
    result["n_sources"] = len(cands)
    result["candidates"] = [{k2: round(v, 1) for k2, v in d.items() if k2 != "contour"}
                            for d in cands]

    # ---- swarm: group the sources into clusters ---------------------------
    if cands and mode == "swarm":
        result["swarms"] = cluster_sources(cands, cluster_radius)
        return result, blurred

    # ---- single: pick one target ------------------------------------------
    if cands:
        chosen, method = None, "blob-brightest"
        if prev is not None and lock_radius > 0:   # lock_radius 0 = sticky off
            px, py = prev
            near = min(cands, key=lambda d: (d["x"] - px) ** 2 + (d["y"] - py) ** 2)
            if np.hypot(near["x"] - px, near["y"] - py) <= lock_radius:
                chosen, method = near, "blob-sticky"
        if chosen is None:
            chosen = cands[0]
        result.update(found=True, x=chosen["x"], y=chosen["y"], method=method,
                      area=chosen["area"], contour=chosen["contour"])
        return result, blurred

    # ---- fallback: raw brightest pixel ----------------------------------
    result.update(found=True, x=float(max_loc[0]), y=float(max_loc[1]),
                  method="minMaxLoc")
    return result, blurred


def cluster_sources(cands, cluster_radius=80):
    """
    Single-linkage clustering of light sources into swarms.

    Two sources are in the same swarm if they are within `cluster_radius` px
    of each other, directly or through a chain of other sources. A radius of
    0 (or less) puts everything into one swarm, which is the old behaviour.

    Returns a list of swarms sorted biggest (highest total mass) first. Each:
      x, y     mass-weighted centroid
      n        number of sources
      mass     sum of source masses (total light)
      area     sum of source areas
      peak     brightest pixel in the swarm
      spread   mass-weighted RMS distance of the sources from the centroid
      hull     convex hull of the source centroids (np array, for drawing)
      members  indices into `cands`
    """
    n = len(cands)
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    r2 = cluster_radius * cluster_radius
    for i in range(n):
        for j in range(i + 1, n):
            if cluster_radius <= 0 or \
               (cands[i]["x"] - cands[j]["x"]) ** 2 + (cands[i]["y"] - cands[j]["y"]) ** 2 <= r2:
                parent[find(i)] = find(j)

    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)

    swarms = []
    for members in groups.values():
        xs = np.array([cands[i]["x"] for i in members])
        ys = np.array([cands[i]["y"] for i in members])
        ws = np.array([cands[i]["mass"] for i in members])
        W = ws.sum()
        cx, cy = float((ws * xs).sum() / W), float((ws * ys).sum() / W)
        spread = float(np.sqrt((ws * ((xs - cx) ** 2 + (ys - cy) ** 2)).sum() / W))
        pts = np.column_stack([xs, ys]).astype(np.int32).reshape(-1, 1, 2)
        hull = cv2.convexHull(pts) if len(members) >= 3 else pts
        swarms.append({
            "x": cx, "y": cy, "n": len(members),
            "mass": float(W),
            "area": float(sum(cands[i]["area"] for i in members)),
            "peak": float(max(cands[i]["peak"] for i in members)),
            "spread": spread, "hull": hull, "members": members,
        })
    swarms.sort(key=lambda d: d["mass"], reverse=True)
    return swarms


class SwarmTracker:
    """
    Gives every swarm a persistent ID across frames and remembers which one
    the user wants to follow.

    Matching: each swarm seen this frame is paired with the closest remembered
    swarm within `match_radius` px (closest pairs first, one-to-one).
    Unmatched swarms get a fresh ID. A remembered swarm that is not seen for
    more than `max_missed` frames is forgotten, so a brief flicker does not
    change IDs but a swarm that really left does not linger in the list.

    Selection (`selected` is a swarm ID):
      * visible          -> follow it                       (state "locked")
      * missing briefly  -> no target, servos hold          (state "hold")
      * gone / none set  -> follow the biggest swarm and    (state "locked")
                            move the selection to it
    `select_next/prev/id/at` change the selection; IDs cycle in numeric order.
    """

    def __init__(self, match_radius=60.0, max_missed=10):
        self.match_radius = match_radius
        self.max_missed = max_missed
        self.tracks = {}          # id -> {"x", "y", "missed"}
        self.next_id = 1
        self.selected = None
        self.visible = []         # IDs seen last frame, numeric order
        self.lock = threading.Lock()

    def reset(self):
        with self.lock:
            self.tracks.clear()
            self.visible = []
            self.selected = None

    def update(self, swarms):
        """Assigns `id` to every swarm in place. Returns (chosen_swarm, state)."""
        with self.lock:
            pairs = []
            for si, sw in enumerate(swarms):
                for tid, t in self.tracks.items():
                    d = float(np.hypot(sw["x"] - t["x"], sw["y"] - t["y"]))
                    if d <= self.match_radius:
                        pairs.append((d, si, tid))
            pairs.sort()
            used_s, used_t = set(), set()
            for d, si, tid in pairs:
                if si in used_s or tid in used_t:
                    continue
                swarms[si]["id"] = tid
                used_s.add(si)
                used_t.add(tid)
            for si, sw in enumerate(swarms):
                if si not in used_s:
                    sw["id"] = self.next_id
                    self.next_id += 1

            seen = set()
            for sw in swarms:
                self.tracks[sw["id"]] = {"x": sw["x"], "y": sw["y"], "missed": 0}
                seen.add(sw["id"])
            for tid in list(self.tracks):
                if tid not in seen:
                    self.tracks[tid]["missed"] += 1
                    if self.tracks[tid]["missed"] > self.max_missed:
                        del self.tracks[tid]
            self.visible = sorted(seen)

            if not swarms:
                return None, "none"
            chosen = next((sw for sw in swarms if sw["id"] == self.selected), None)
            if chosen is not None:
                return chosen, "locked"
            if self.selected in self.tracks:        # briefly missing: hold
                return None, "hold"
            chosen = swarms[0]                        # biggest swarm
            self.selected = chosen["id"]
            return chosen, "locked"

    def select_next(self, step=1):
        with self.lock:
            if not self.visible:
                return self.selected
            if self.selected in self.visible:
                i = (self.visible.index(self.selected) + step) % len(self.visible)
            else:
                i = 0
            self.selected = self.visible[i]
            return self.selected

    def select_id(self, sid):
        with self.lock:
            if sid in self.tracks:
                self.selected = sid
            return self.selected

    def select_at(self, x, y):
        """Select the visible swarm whose centroid is nearest to (x, y)."""
        with self.lock:
            best = None
            for tid in self.visible:
                t = self.tracks[tid]
                d = (t["x"] - x) ** 2 + (t["y"] - y) ** 2
                if best is None or d < best[0]:
                    best = (d, tid)
            if best is not None:
                self.selected = best[1]
            return self.selected


# =====================================================================
#  3. Controller  (pixel error -> servo angles)
# =====================================================================
# Mechanical limits of the optics bracket and its rest position (degrees).
# MUST match PAN_MIN/MAX, TILT_MIN/MAX, PAN_HOME/TILT_HOME in esp32_servo.ino.
# Clipping here (not just on the ESP32) stops the controller winding up past
# what the servo can physically reach.
PAN_RANGE  = (5.0, 175.0)
TILT_RANGE = (5.0, 105.0)
PAN_HOME, TILT_HOME = 90.0, 55.0


class Controller:
    """
    Proportional-integral controller working in *angle* space.

    error_x_norm  = (spot_x - cx) / (W/2)        range -1..+1
    pan_correction = Kp * error_x_norm * (HFOV/2)  [deg]   (+ Ki term)

    i.e. if the spot is at the very right edge, the camera needs to pan by
    roughly half the horizontal field of view to center it.
    """
    labels = ("pan", "tilt")

    def __init__(self, hfov=62.0, vfov=49.0, kp=0.6, ki=0.05,
                 deadband_px=6, max_step=8.0,
                 invert_pan=False, invert_tilt=False):
        self.hfov, self.vfov = hfov, vfov
        self.kp, self.ki = kp, ki
        self.deadband_px = deadband_px
        self.max_step = max_step
        self.invert_pan, self.invert_tilt = invert_pan, invert_tilt
        self.pan, self.tilt = PAN_HOME, TILT_HOME
        self.ix = self.iy = 0.0
        self.last = {}

    @property
    def ranges(self):
        return PAN_RANGE, TILT_RANGE

    def home(self):
        self.pan, self.tilt = PAN_HOME, TILT_HOME
        self.ix = self.iy = 0.0

    def update(self, spot, w, h, dt):
        cx, cy = w / 2.0, h / 2.0
        if not spot["found"]:
            self.ix = self.iy = 0.0
            self.last = {"ex_px": 0, "ey_px": 0, "ex_norm": 0, "ey_norm": 0,
                         "dpan": 0, "dtilt": 0, "in_deadband": True}
            return self.pan, self.tilt

        ex_px = spot["x"] - cx          # +ve = spot is right of center
        ey_px = spot["y"] - cy          # +ve = spot is below center
        ex_norm = ex_px / cx
        ey_norm = ey_px / cy
        err_px = float(np.hypot(ex_px, ey_px))
        in_deadband = err_px < self.deadband_px

        dpan = dtilt = 0.0
        if not in_deadband:
            self.ix = float(np.clip(self.ix + ex_norm * dt, -2, 2))
            self.iy = float(np.clip(self.iy + ey_norm * dt, -2, 2))

            # Spot to the RIGHT -> camera must pan RIGHT -> (by default) angle increases
            dpan = (self.kp * ex_norm + self.ki * self.ix) * (self.hfov / 2)
            # Spot BELOW center -> camera must tilt DOWN -> (by default) angle decreases
            dtilt = -(self.kp * ey_norm + self.ki * self.iy) * (self.vfov / 2)

            if self.invert_pan:
                dpan = -dpan
            if self.invert_tilt:
                dtilt = -dtilt

            dpan = float(np.clip(dpan, -self.max_step, self.max_step))
            dtilt = float(np.clip(dtilt, -self.max_step, self.max_step))

            self.pan = float(np.clip(self.pan + dpan, *PAN_RANGE))
            self.tilt = float(np.clip(self.tilt + dtilt, *TILT_RANGE))
        else:
            self.ix = self.iy = 0.0

        self.last = {
            "ex_px": round(ex_px, 1), "ey_px": round(ey_px, 1),
            "ex_norm": round(ex_norm, 3), "ey_norm": round(ey_norm, 3),
            "err_px": round(err_px, 1),
            "dpan": round(dpan, 2), "dtilt": round(dtilt, 2),
            "in_deadband": in_deadband,
        }
        return self.pan, self.tilt


class FinController:
    """
    Direct fin steering for the fish (--output fins), no IMU involved.

    The pan/tilt Controller accumulates an absolute bracket angle. Fins are
    different: the command is a *deflection*, how hard to push right now. The
    fish's body turns, the camera mounted on it sees the light drift back
    towards the centre, and the deflection relaxes to zero. That is the loop.

        yaw_fin   =  (Kp·ex_norm + Ki·∫ex) · fin_max     spot right -> yaw right
        pitch_fin = -(Kp·ey_norm + Ki·∫ey) · fin_max     spot below -> pitch down

    Inside the deadband, or with no target at all, the fins go to neutral.
    `max_step` limits how far the deflection may change per frame so a
    flickering detection does not slam the fins. `invert_pan` / `invert_tilt`
    flip yaw / pitch, same flags as the bracket.

    The outputs are stored as `pan` / `tilt` (= yaw / pitch fin, degrees from
    neutral) so ServoLink, the overlay and the telemetry are shared with the
    bracket controller; `labels` / `ranges` tell the dashboard what they mean.
    """
    labels = ("yaw fin", "pitch fin")

    def __init__(self, fin_max=30.0, kp=0.6, ki=0.05, deadband_px=6, max_step=8.0,
                 invert_pan=False, invert_tilt=False):
        self.fin_max = fin_max
        self.kp, self.ki = kp, ki
        self.deadband_px = deadband_px
        self.max_step = max_step
        self.invert_pan, self.invert_tilt = invert_pan, invert_tilt
        self.pan = self.tilt = 0.0          # yaw / pitch fin deflection, deg
        self.ix = self.iy = 0.0
        self.last = {}

    @property
    def ranges(self):
        return (-self.fin_max, self.fin_max), (-self.fin_max, self.fin_max)

    def home(self):
        self.pan = self.tilt = 0.0
        self.ix = self.iy = 0.0

    def _slew(self, yaw_t, pitch_t):
        self.pan  += float(np.clip(yaw_t   - self.pan,  -self.max_step, self.max_step))
        self.tilt += float(np.clip(pitch_t - self.tilt, -self.max_step, self.max_step))
        return self.pan, self.tilt

    def update(self, spot, w, h, dt):
        cx, cy = w / 2.0, h / 2.0
        if not spot["found"]:
            self.ix = self.iy = 0.0
            self.last = {"ex_px": 0, "ey_px": 0, "ex_norm": 0, "ey_norm": 0, "err_px": 0,
                         "dpan": 0, "dtilt": 0, "in_deadband": True}
            return self._slew(0.0, 0.0)        # no light: glide straight

        ex_px = spot["x"] - cx
        ey_px = spot["y"] - cy
        ex_norm = ex_px / cx
        ey_norm = ey_px / cy
        err_px = float(np.hypot(ex_px, ey_px))
        in_deadband = err_px < self.deadband_px

        yaw_t = pitch_t = 0.0
        if in_deadband:
            self.ix = self.iy = 0.0
        else:
            self.ix = float(np.clip(self.ix + ex_norm * dt, -2, 2))
            self.iy = float(np.clip(self.iy + ey_norm * dt, -2, 2))
            yaw_t   =  (self.kp * ex_norm + self.ki * self.ix) * self.fin_max
            pitch_t = -(self.kp * ey_norm + self.ki * self.iy) * self.fin_max
            if self.invert_pan:
                yaw_t = -yaw_t
            if self.invert_tilt:
                pitch_t = -pitch_t
            yaw_t   = float(np.clip(yaw_t,   -self.fin_max, self.fin_max))
            pitch_t = float(np.clip(pitch_t, -self.fin_max, self.fin_max))

        self.last = {
            "ex_px": round(ex_px, 1), "ey_px": round(ey_px, 1),
            "ex_norm": round(ex_norm, 3), "ey_norm": round(ey_norm, 3),
            "err_px": round(err_px, 1),
            "dpan": round(yaw_t, 2), "dtilt": round(pitch_t, 2),   # commanded deflection
            "in_deadband": in_deadband,
        }
        return self._slew(yaw_t, pitch_t)


# =====================================================================
#  4. UDP link to the servo ESP32
# =====================================================================
class ServoLink:
    def __init__(self, ip, port=4210):
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.05)
        self.sent = 0
        self.acks = 0
        self.last_ack = None
        self.last_ack_time = 0.0
        self.rtt_ms = None

    def send(self, pan, tilt):
        msg = f"{pan:.1f},{tilt:.1f}\n".encode()
        t0 = time.time()
        try:
            self.sock.sendto(msg, self.addr)
            self.sent += 1
            data, _ = self.sock.recvfrom(64)
            if data.startswith(b"ACK"):
                self.acks += 1
                self.last_ack = data.decode(errors="ignore").strip()
                self.last_ack_time = time.time()
                self.rtt_ms = (self.last_ack_time - t0) * 1000
        except socket.timeout:
            pass
        except OSError as e:
            hint = ""
            if getattr(e, "errno", None) == 13:   # EACCES on sendto
                hint = ("  (Permission denied on a UDP send usually means the target IP is the "
                        "subnet's broadcast address, e.g. .15 on a /28 hotspot; give the board "
                        "a different static IP)")
            print(f"[servo] send error: {e}{hint}")

    def home(self):
        try:
            self.sock.sendto(b"HOME\n", self.addr)
        except OSError:
            pass

    @property
    def alive(self):
        return (time.time() - self.last_ack_time) < 1.5


class FakeServoLink:
    """
    Stand-in for ServoLink when no servo ESP32 is given (--servo omitted).
    Behaves like the servo node: slews toward the commanded angles at the
    same rate (<= 3 deg per 20 ms) and always "ACKs", so the dashboard shows
    a live link. With a FakeCam it also points the camera's view at the
    resulting angles, closing the loop; with a real camera it just tracks
    the angles so the tracker can run without the servo board.
    """
    SLEW_DEG_PER_S = 3.0 / 0.020

    def __init__(self, cam):
        self.cam = cam
        self.pan, self.tilt = PAN_HOME, TILT_HOME
        self._target_pan, self._target_tilt = PAN_HOME, TILT_HOME
        self.sent = 0
        self.acks = 0
        self.last_ack = None
        self.last_ack_time = 0.0
        self.rtt_ms = 0.5
        self._t = time.time()
        threading.Thread(target=self._slew_loop, daemon=True).start()

    def _slew_loop(self):
        while True:
            now = time.time()
            step = self.SLEW_DEG_PER_S * (now - self._t)
            self._t = now
            self.pan += float(np.clip(self._target_pan - self.pan, -step, step))
            self.tilt += float(np.clip(self._target_tilt - self.tilt, -step, step))
            if hasattr(self.cam, "set_view"):      # only the fake camera can be steered
                self.cam.set_view(self.pan, self.tilt)
            time.sleep(0.02)

    def send(self, pan, tilt):
        self._target_pan = float(np.clip(pan, *PAN_RANGE))
        self._target_tilt = float(np.clip(tilt, *TILT_RANGE))
        self.sent += 1
        self.acks += 1
        self.last_ack_time = time.time()
        self.last_ack = f"ACK {self.pan:.1f} {self.tilt:.1f} {self.acks}"

    def home(self):
        self._target_pan, self._target_tilt = PAN_HOME, TILT_HOME

    @property
    def alive(self):
        return True


class FakeFinLink:
    """
    Stand-in for the fin board (--output fins with --servo omitted). Slews the
    fin deflection like pid_fins.ino (<= 3 deg per 20 ms) and always "ACKs".
    With a FakeCam it also turns the simulated fish: heading rate is
    TURN_DEG_S_PER_FIN_DEG times the deflection, and the camera view follows
    the heading, so the loop is closed and you can tune Kp on the bench.
    """
    SLEW_DEG_PER_S = 3.0 / 0.020
    TURN_DEG_S_PER_FIN_DEG = 1.0      # 30 deg of fin -> 30 deg/s of yaw

    def __init__(self, cam):
        self.cam = cam
        self.pan = self.tilt = 0.0            # actual fin deflection (yaw, pitch)
        self._target_pan = self._target_tilt = 0.0
        self.heading_pan, self.heading_tilt = PAN_HOME, TILT_HOME   # where the fish points
        self.sent = 0
        self.acks = 0
        self.last_ack = None
        self.last_ack_time = 0.0
        self.rtt_ms = 0.5
        self._t = time.time()
        threading.Thread(target=self._slew_loop, daemon=True).start()

    def _slew_loop(self):
        while True:
            now = time.time()
            dt = now - self._t
            self._t = now
            step = self.SLEW_DEG_PER_S * dt
            self.pan += float(np.clip(self._target_pan - self.pan, -step, step))
            self.tilt += float(np.clip(self._target_tilt - self.tilt, -step, step))
            self.heading_pan = float(np.clip(self.heading_pan + self.pan * self.TURN_DEG_S_PER_FIN_DEG * dt, *PAN_RANGE))
            self.heading_tilt = float(np.clip(self.heading_tilt + self.tilt * self.TURN_DEG_S_PER_FIN_DEG * dt, *TILT_RANGE))
            if hasattr(self.cam, "set_view"):
                self.cam.set_view(self.heading_pan, self.heading_tilt)
            time.sleep(0.02)

    def send(self, yaw, pitch):
        self._target_pan, self._target_tilt = float(yaw), float(pitch)
        self.sent += 1
        self.acks += 1
        self.last_ack_time = time.time()
        self.last_ack = f"ACK {self.pan:.1f} {self.tilt:.1f} {self.acks}"

    def home(self):
        self._target_pan = self._target_tilt = 0.0

    @property
    def alive(self):
        return True


# =====================================================================
#  5. Main tracker loop + dashboard
# =====================================================================
class Tracker:
    def __init__(self, args):
        self.fake = args.cam.lower() == "fake"
        if self.fake:
            w, h = (int(v) for v in args.fake_size.lower().split("x"))
            area = tuple(float(v) for v in args.fake_area.lower().split("x"))
            self.cam = FakeCam(n_swarms=args.fake_swarms, lights_per_swarm=args.fake_lights,
                               width=w, height=h, fps=args.fake_fps, hfov=args.hfov, vfov=args.vfov,
                               swarm_radius_deg=args.fake_spread, speed_deg_s=args.fake_speed,
                               area_deg=area, seed=args.fake_seed)
        else:
            self.cam = MjpegReader(f"http://{args.cam}:{args.cam_port}/stream")
        # --servo is optional: without it a simulated servo link is used, so
        # the tracker runs (and the dashboard works) with no servo board.
        self.output = args.output
        if args.servo:
            self.servo = ServoLink(args.servo, args.servo_port)
        elif self.output == "fins":
            print("[fins] no --servo given: using simulated fish (nothing is driven)")
            self.servo = FakeFinLink(self.cam)
        else:
            print("[servo] no --servo given: using simulated servo (nothing is driven)")
            self.servo = FakeServoLink(self.cam)
        if self.output == "fins":
            self.ctrl = FinController(invert_pan=args.invert_pan, invert_tilt=args.invert_tilt)
        else:
            self.ctrl = Controller(hfov=args.hfov, vfov=args.vfov,
                                   invert_pan=args.invert_pan, invert_tilt=args.invert_tilt)
        # Detection sensitivity. thresh_mode "relative": a pixel is light when
        # it is rel_threshold of the way from the frame's background level to
        # its brightest pixel, and the frame needs at least min_contrast of
        # range or it is treated as empty. "absolute": fixed cutoff `threshold`.
        # min_area = smallest blob in px; blur = smoothing kernel (bigger kills
        # hot pixels but also dims small far-away sources).
        self.cfg = {"blur": 9, "min_area": 4, "lock_radius": 40,
                    "thresh_mode": args.thresh_mode,       # "relative" | "absolute"
                    "rel_threshold": args.rel_threshold,   # relative: 0..1 of background->peak
                    "min_contrast": args.min_contrast,     # relative: peak must beat background by this
                    "threshold": args.threshold,           # absolute: fixed cutoff 0..255
                    "mode": "single",          # "single" = one light, "swarm" = follow a cluster
                    "max_sources": 32,
                    "cluster_radius": 80,      # swarm: sources closer than this are one swarm (0 = all)
                    "swarm_match": 60,         # swarm: max px a swarm may move per frame and keep its ID
                    "kp": 0.6, "ki": 0.05, "deadband": 6, "max_step": 8.0,
                    "fin_max": 30.0,           # fins: deflection limit, deg (--output fins)
                    "tracking": True, "send_rate_hz": 20, "show_mask": False}
        self.annotated = None
        self.jpeg = None
        self.telemetry = {}
        self.lock = threading.Lock()
        self._last_send = 0.0
        self._loop_times = deque(maxlen=30)
        self._prev_xy = None   # last frame's target, for sticky selection
        self.swarms = SwarmTracker()   # swarm IDs + which one is selected

    def start(self):
        self.cam.start()
        threading.Thread(target=self._loop, daemon=True).start()

    # ---- drawing helpers -------------------------------------------------
    @staticmethod
    def _crosshair(img, x, y, color, size=14, thick=2):
        x, y = int(x), int(y)
        cv2.circle(img, (x, y), size, color, thick, cv2.LINE_AA)
        cv2.line(img, (x - size - 6, y), (x - 4, y), color, thick)
        cv2.line(img, (x + 4, y), (x + size + 6, y), color, thick)
        cv2.line(img, (x, y - size - 6), (x, y - 4), color, thick)
        cv2.line(img, (x, y + 4), (x, y + size + 6), color, thick)

    def _draw(self, frame, spot, blurred):
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        out = frame.copy()

        if self.cfg["show_mask"] and spot.get("mask") is not None:
            mask_bgr = cv2.cvtColor(spot["mask"], cv2.COLOR_GRAY2BGR)
            out = cv2.addWeighted(out, 0.6, mask_bgr, 0.4, 0)

        # frame-center reticle + deadband circle
        cv2.drawMarker(out, (cx, cy), (200, 200, 200), cv2.MARKER_CROSS, 24, 1, cv2.LINE_AA)
        cv2.circle(out, (cx, cy), int(self.cfg["deadband"]), (200, 200, 200), 1, cv2.LINE_AA)

        # every detected light source: dim circle + index, so you can see what
        # the detector is considering even though only one drives the servos
        for i, cnd in enumerate(spot.get("candidates", [])):
            px, py = int(cnd["x"]), int(cnd["y"])
            cv2.circle(out, (px, py), 8, (160, 160, 160), 1, cv2.LINE_AA)
            cv2.putText(out, f"{i}:{cnd['peak']:.0f}", (px + 10, py + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 160, 160), 1, cv2.LINE_AA)

        # every swarm: hull + ID label. The selected one is drawn bright
        # magenta, the others dim purple so you can see what you can switch to.
        sel_id = spot.get("swarm_id")
        for sw in spot.get("swarms", []):
            is_sel = sw.get("id") == sel_id
            hull_col = (255, 120, 230) if is_sel else (140, 70, 130)
            txt_col = (255, 0, 200) if is_sel else (150, 90, 150)
            if sw.get("hull") is not None and len(sw["hull"]) >= 2:
                cv2.polylines(out, [sw["hull"]], True, hull_col, 1, cv2.LINE_AA)
            sx, sy = int(sw["x"]), int(sw["y"])
            if is_sel:
                for i in sw["members"]:
                    cnd = spot["candidates"][i]
                    cv2.line(out, (sx, sy), (int(cnd["x"]), int(cnd["y"])), (120, 60, 110), 1, cv2.LINE_AA)
            else:
                cv2.circle(out, (sx, sy), 5, txt_col, 1, cv2.LINE_AA)
            cv2.putText(out, f"S{sw.get('id', '?')} n={sw['n']}", (sx + 8, sy + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, txt_col, 1, cv2.LINE_AA)

        if spot["found"]:
            x, y = spot["x"], spot["y"]
            swarm = spot["method"] == "swarm-centroid"
            col = (255, 0, 200) if swarm else (0, 60, 255)   # magenta for swarm, orange for single
            if swarm:
                cv2.circle(out, (int(x), int(y)), int(spot["spread"]), col, 1, cv2.LINE_AA)
                label = f"S{sel_id} {spot['n_sources']}src  r={spot['spread']:.0f}px"
            else:
                if spot.get("contour") is not None:
                    cv2.drawContours(out, [spot["contour"]], -1, (0, 200, 255), 1, cv2.LINE_AA)
                label = f"{spot['max_val']:.0f}"
            self._crosshair(out, x, y, col)
            cv2.arrowedLine(out, (cx, cy), (int(x), int(y)), (0, 255, 120), 1, cv2.LINE_AA, tipLength=0.08)
            cv2.putText(out, label, (int(x) + 18, int(y) - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
        elif spot["method"] == "swarm-hold":
            cv2.putText(out, f"S{sel_id} LOST - holding", (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1, cv2.LINE_AA)
        else:
            cv2.putText(out, "NO TARGET", (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1, cv2.LINE_AA)

        la, lb = self.ctrl.labels
        cv2.putText(out, f"{la} {self.ctrl.pan:5.1f}  {lb} {self.ctrl.tilt:5.1f}", (10, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        n = len(spot.get("candidates", []))
        if self.cfg["mode"] == "swarm":
            hud = f"swarm {len(spot.get('swarms', []))}  sources {n}"
        else:
            hud = f"single  sources {n}"
        cv2.putText(out, hud, (w - 170, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        # what the detector used this frame: background level and cutoff
        cv2.putText(out, f"bg {spot['background']:.0f}  cut {spot['cutoff']:.0f}  peak {spot['max_val']:.0f}",
                    (w - 200, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1, cv2.LINE_AA)
        return out

    # ---- main loop ----------------------------------------------------------
    def _loop(self):
        last_t = time.time()
        while True:
            frame = self.cam.wait_new()   # one control step per camera frame
            if frame is None:
                continue
            t0 = time.time()
            dt = t0 - last_t
            last_t = t0

            c = self.cfg
            self.ctrl.kp, self.ctrl.ki = c["kp"], c["ki"]
            self.ctrl.deadband_px, self.ctrl.max_step = c["deadband"], c["max_step"]
            if self.output == "fins":
                self.ctrl.fin_max = c["fin_max"]

            spot, blurred = find_bright_spot(frame, c["blur"], c["threshold"], c["min_area"],
                                             prev=self._prev_xy, lock_radius=c["lock_radius"],
                                             max_sources=c["max_sources"], mode=c["mode"],
                                             cluster_radius=c["cluster_radius"],
                                             thresh_mode=c["thresh_mode"],
                                             rel_threshold=c["rel_threshold"],
                                             min_contrast=c["min_contrast"])
            if c["mode"] == "swarm":
                # persistent IDs + user selection -> one target
                self.swarms.match_radius = c["swarm_match"]
                chosen, state = self.swarms.update(spot["swarms"])
                spot["swarm_id"] = self.swarms.selected
                if chosen is not None:
                    spot.update(found=True, x=chosen["x"], y=chosen["y"],
                                method="swarm-centroid", area=chosen["area"],
                                spread=chosen["spread"], hull=chosen["hull"],
                                n_sources=chosen["n"])
                elif state == "hold":
                    spot["method"] = "swarm-hold"
            self._prev_xy = (spot["x"], spot["y"]) if spot["found"] else None
            h, w = frame.shape[:2]

            if c["tracking"]:
                pan, tilt = self.ctrl.update(spot, w, h, dt)
            else:
                pan, tilt = self.ctrl.pan, self.ctrl.tilt

            if t0 - self._last_send >= 1.0 / max(1, c["send_rate_hz"]):
                self.servo.send(pan, tilt)
                self._last_send = t0

            annotated = self._draw(frame, spot, blurred)
            ok, jpg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])

            self._loop_times.append(time.time())
            proc_fps = 0.0
            if len(self._loop_times) > 1:
                proc_fps = (len(self._loop_times) - 1) / (self._loop_times[-1] - self._loop_times[0])

            tele = {
                "time": t0,
                "frame": {"w": w, "h": h, "cam_fps": round(self.cam.fps, 1),
                          "proc_fps": round(proc_fps, 1), "cam_connected": self.cam.connected,
                          "cv_ms": round((time.time() - t0) * 1000, 1),
                          "dropped_frames": self.cam.dropped, "fake": self.fake},
                "spot": {"found": spot["found"],
                         "x": None if spot["x"] is None else round(spot["x"], 1),
                         "y": None if spot["y"] is None else round(spot["y"], 1),
                         "max_val": round(spot["max_val"], 1),
                         "background": round(spot["background"], 1),
                         "contrast": round(spot["contrast"], 1),
                         "cutoff": round(spot["cutoff"], 1),
                         "thresh_mode": spot["thresh_mode"],
                         "area": spot["area"], "method": spot["method"],
                         "n_sources": spot["n_sources"], "candidates": spot["candidates"],
                         "mode": spot["mode"], "spread": round(spot["spread"], 1),
                         "swarm_id": spot["swarm_id"],
                         "swarms": [{"id": sw.get("id"), "x": round(sw["x"], 1), "y": round(sw["y"], 1),
                                     "n": sw["n"], "mass": round(sw["mass"]), "spread": round(sw["spread"], 1),
                                     "area": round(sw["area"]), "peak": round(sw["peak"]),
                                     "selected": sw.get("id") == spot["swarm_id"]}
                                    for sw in spot["swarms"]]},
                "error": self.ctrl.last,
                "servo": {"pan": round(self.ctrl.pan, 1), "tilt": round(self.ctrl.tilt, 1),
                          "pan_range": self.ctrl.ranges[0], "tilt_range": self.ctrl.ranges[1],
                          "labels": self.ctrl.labels, "output": self.output,
                          "sent": self.servo.sent, "acks": self.servo.acks,
                          "alive": self.servo.alive, "last_ack": self.servo.last_ack,
                          "rtt_ms": None if self.servo.rtt_ms is None else round(self.servo.rtt_ms, 1)},
                "cfg": {k: v for k, v in c.items()},
            }
            with self.lock:
                if ok:
                    self.jpeg = jpg.tobytes()
                self.telemetry = tele

    def mjpeg_generator(self):
        while True:
            with self.lock:
                jpg = self.jpeg
            if jpg is None:
                time.sleep(0.05)
                continue
            yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                   + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
            time.sleep(0.03)


# ---------------------------------------------------------------------
app = Flask(__name__)
tracker: Tracker = None


@app.route("/")
def index():
    return render_template("dashboard.html")


@app.route("/video_feed")
def video_feed():
    return Response(tracker.mjpeg_generator(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/telemetry")
def telemetry():
    with tracker.lock:
        return jsonify(tracker.telemetry)


@app.route("/config", methods=["POST"])
def config():
    data = request.get_json(force=True) or {}
    for k, v in data.items():
        if k not in tracker.cfg:
            continue
        if k == "thresh_mode" and v not in ("relative", "absolute"):
            continue
        tracker.cfg[k] = type(tracker.cfg[k])(v)
    if "mode" in data:
        tracker.swarms.reset()      # stale IDs from a previous session would mis-match
    return jsonify(tracker.cfg)


@app.route("/swarm", methods=["POST"])
def swarm():
    """Change which swarm is followed (swarm mode only).
       {"action": "next"|"prev"}  cycle through visible swarms by ID
       {"id": 3}                  select swarm 3
       {"x": 120, "y": 80}        select the swarm nearest to that pixel"""
    data = request.get_json(force=True) or {}
    st = tracker.swarms
    if data.get("action") == "next":
        st.select_next(+1)
    elif data.get("action") == "prev":
        st.select_next(-1)
    elif "id" in data:
        st.select_id(int(data["id"]))
    elif "x" in data and "y" in data:
        st.select_at(float(data["x"]), float(data["y"]))
    return jsonify(selected=st.selected, visible=st.visible)


@app.route("/home", methods=["POST"])
def home():
    tracker.ctrl.home()
    tracker.servo.home()
    return jsonify(ok=True)


def main():
    global tracker
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", required=True, help="ESP32-CAM IP, or 'fake' for a synthetic scene")
    ap.add_argument("--cam-port", type=int, default=81)
    ap.add_argument("--servo", default=None,
                    help="Servo ESP32 IP (optional: without it a simulated servo is used and nothing is driven)")
    ap.add_argument("--servo-port", type=int, default=4210)
    ap.add_argument("--output", choices=["pantilt", "fins"], default="pantilt",
                    help="pantilt = drive the camera bracket (esp32_servo.ino); "
                         "fins = steer the fish's fins (pid_fins.ino)")
    ap.add_argument("--hfov", type=float, default=62.0, help="camera horizontal FOV, deg")
    ap.add_argument("--vfov", type=float, default=49.0, help="camera vertical FOV, deg")
    ap.add_argument("--invert-pan", action="store_true")
    ap.add_argument("--invert-tilt", action="store_true")
    ap.add_argument("--port", type=int, default=9000, help="dashboard port")
    det = ap.add_argument_group("detection (all adjustable live on the dashboard)")
    det.add_argument("--thresh-mode", choices=["relative", "absolute"], default="relative",
                     help="relative: cutoff follows the frame's background and brightest pixel "
                          "(for IR-filtered cameras); absolute: fixed cutoff --threshold")
    det.add_argument("--rel-threshold", type=float, default=0.5,
                     help="relative mode: a pixel is light when it is this fraction of the way "
                          "from the background level to the brightest pixel (0-1)")
    det.add_argument("--min-contrast", type=int, default=40,
                     help="relative mode: brightest pixel must exceed the background by this "
                          "much (0-255) or the frame counts as empty")
    det.add_argument("--threshold", type=int, default=150,
                     help="absolute mode: fixed brightness cutoff 0-255")
    fk = ap.add_argument_group("fake camera (--cam fake)")
    fk.add_argument("--fake-swarms", type=int, default=3, help="number of swarms in the scene")
    fk.add_argument("--fake-lights", type=int, default=4, help="lights per swarm")
    fk.add_argument("--fake-spread", type=float, default=3.0, help="swarm radius, degrees")
    fk.add_argument("--fake-speed", type=float, default=3.0, help="how fast swarms drift, deg/s")
    fk.add_argument("--fake-area", default="50x30",
                    help="region the swarms roam in, pan x tilt degrees around home "
                         "(default fits in one view; e.g. 160x90 to make the camera hunt)")
    fk.add_argument("--fake-size", default="640x480", help="frame size WxH")
    fk.add_argument("--fake-fps", type=float, default=20.0)
    fk.add_argument("--fake-seed", type=int, default=None, help="RNG seed for a repeatable scene")
    args = ap.parse_args()

    tracker = Tracker(args)
    tracker.start()
    print(f"Output: {args.output}  ->  {args.servo or 'simulated'}:{args.servo_port}")
    print(f"Dashboard: http://localhost:{args.port}")
    app.run(host="0.0.0.0", port=args.port, threaded=True, debug=False)


if __name__ == "__main__":
    main()
