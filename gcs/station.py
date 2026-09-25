"""
GroundStation: owns the link, the estimator, the vision thread and the guidance
loop; builds the telemetry snapshot the browser receives at 20 Hz; executes
operator commands with plain-language errors.

Guidance modes
  laptop (default): the laptop steers in every phase (IMU glide from the
      streamed ICM20948 samples, vision homing from the camera). The fish node
      only moves servos and streams the IMU.
  fish: the fish node steers itself during IMU glide from the mission it was
      sent ("MISSION,..." / "LAUNCH"); the laptop only monitors until hand-over.
      fish_node.ino accepts those commands but its onboardGuidance() is a stub.
"""
import csv
import io
import json
import math
import threading
import time
from collections import deque

import numpy as np

from .guidance import FLYING, Guidance
from .imu import G, ImuEstimator
from .imu import euler_deg
from .link import FakeFish, FishLink, SimCamera
from .vision import MjpegSource, Vision

DEFAULT_CFG = {
    "handover_threshold": 0.80, "beacon_frames_needed": 5, "conf_needed": 0.60, "handover_blend_s": 1.0,
    "cv_lost_timeout_s": 2.0, "arrive_dist_m": 0.3, "arrive_radius_px": 90,
    "kp_imu": 1.2, "kp": 0.6, "ki": 0.05, "deadband_px": 6, "max_step": 8.0, "fin_max": 30.0,
    "invert_yaw": False, "invert_pitch": False,
    "blur": 9, "min_area": 4, "thresh_mode": "relative", "rel_threshold": 0.5, "min_contrast": 40, "threshold": 150,
    "calib_s": 3.0, "batt_min_v": 7.0, "launch_detect_g": 2.0, "launch_wait_s": 10.0,
    # vision mode (design section 15): "beacon" = brightest source, "swarm" = follow a cluster's centroid
    "vision_mode": "beacon", "cluster_radius": 200, "swarm_match": 60, "max_sources": 32,
    "yolo_conf": 0.35, "yolo_imgsz": 320, "yolo_classes": "", "sim_quads": 4,
}
TUNABLE = {"kp_imu", "kp", "ki", "deadband_px", "max_step", "fin_max", "invert_yaw", "invert_pitch",
           "blur", "min_area", "thresh_mode", "rel_threshold", "min_contrast", "threshold",
           "beacon_frames_needed", "conf_needed", "handover_blend_s",
           "vision_mode", "cluster_radius", "swarm_match", "max_sources", "yolo_conf", "yolo_classes", "sim_quads"}


class GroundStation:
    def __init__(self, args):
        self.args = args
        self.cfg = dict(DEFAULT_CFG)
        self.lock = threading.RLock()
        self.imu = ImuEstimator(axes=args.imu_axes)
        self.guidance = Guidance(self.cfg)
        self.link = None
        self.cam = None
        self.vision = None
        self.connected = False
        self.calibrated = False
        self.selftest_ok = False
        self.selftest_cmd = None
        self.arm_pending_until = None
        self.autolaunch = True
        self.launch_pending_until = None
        self.override = {"on": False, "yaw": 0.0, "pitch": 0.0}
        self.mission = {"speed": None, "target": None, "set": False, "source": None}
        self.batt_v = None
        self.events = deque(maxlen=500)
        self.event_seq = 0
        self.history = deque(maxlen=12000)
        self.launch_time = None
        self.telemetry = {}
        self.fin_cmd = (0.0, 0.0)
        self._stop = threading.Event()
        self._last_hist = 0.0
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.log("Ground station started%s." % (" in FAKE mode: no hardware, a simulated fish" if args.fake else ""))
        if args.yolo:
            self.log("YOLO weights: %s" % args.yolo)

    # ------------------------------------------------------------------ helpers
    def log(self, msg, level="info"):
        with self.lock:
            self.event_seq += 1
            ev = {"seq": self.event_seq, "t": time.time(), "t_mission": self.t_mission(), "level": level, "msg": msg}
            self.events.append(ev)
        print("[%s] %s" % (level, msg), flush=True)
        return ev

    def t_mission(self):
        return None if self.launch_time is None else time.time() - self.launch_time

    def start(self):
        self.thread.start()

    def stop(self):
        self._stop.set()
        self.disconnect(quiet=True)

    @property
    def phase(self):
        return self.guidance.phase

    def set_phase(self, phase, why=None, level="info"):
        if self.guidance.phase == phase:
            return
        self.guidance.set_phase(phase)
        names = {"disconnected": "Disconnected", "ready": "Ready", "armed": "Armed", "imu_glide": "IMU glide",
                 "handover": "Hand-over", "cv_homing": "Vision homing", "arrived": "Arrived", "aborted": "Aborted · safe"}
        self.log("Phase: %s%s" % (names[phase], (" — " + why) if why else ""), level)
        if phase in ("arrived", "aborted"):
            self.imu.stop()
            if self.link:
                self.link.home()

    # ------------------------------------------------------------------ connect
    def connect(self, fin_ip, cam_ip):
        if self.connected:
            return {"ok": False, "error": "Already connected. Disconnect first."}
        if self.args.fake:
            fish = FakeFish(on_imu=self.imu.update, on_batt=self._on_batt, axes=self.args.imu_axes)
            fish.set_swarm(self.cfg["vision_mode"] == "swarm", self.cfg["sim_quads"])
            self.link = fish
            self.cam = SimCamera(fish.cam)      # browser-rendered fish view when the page streams it
            fish.start(); self.cam.start()
        else:
            self.link = FishLink(fin_ip, on_imu=self.imu.update, on_batt=self._on_batt)
            self.link.start()
            self.cam = MjpegSource("http://%s:81/stream" % cam_ip)
            self.cam.start()
        self.vision = Vision(self.cam, self.cfg, yolo_weights=self.args.yolo)
        self.vision.start()
        if self.vision.yolo_error:
            self.log(self.vision.yolo_error, "warn")
        # probe the fin board: send neutral and wait for an ACK
        deadline = time.time() + 2.5
        while time.time() < deadline and not self.link.alive:
            self.link.send(0.0, 0.0)
            time.sleep(0.1)
        if not self.link.alive:
            err = self.link.error or "No ACK from %s in 2.5 s. Check the board is on the hotspot and its IP." % fin_ip
            self.disconnect(quiet=True)
            self.log("Connect failed: " + err, "warn")
            return {"ok": False, "error": err}
        self.connected = True
        cam_deadline = time.time() + 3.0
        while time.time() < cam_deadline and self.cam.latest() is None:
            time.sleep(0.1)
        cam_ok = self.cam.latest() is not None
        self.log("Fin board ACK %.0f ms.%s" % (self.link.rtt_ms or 0, " Camera streaming." if cam_ok else " Camera: no frame yet from %s (stream keeps retrying)." % cam_ip), "good" if cam_ok else "warn")
        if self.mission["set"] and self.link.fake:
            self.link.set_mission(self.mission["speed"], self.mission["target"])
        self.set_phase("ready")
        return {"ok": True, "cam": cam_ok, "rtt_ms": self.link.rtt_ms}

    def disconnect(self, quiet=False):
        if self.phase in FLYING:
            return {"ok": False, "error": "Cannot disconnect while flying. Abort first."}
        for obj in (self.vision, self.cam, self.link):
            if obj is not None:
                try:
                    obj.stop()
                except Exception:  # noqa: BLE001
                    pass
        self.vision = self.cam = self.link = None
        self.connected = False
        self.calibrated = False
        self.selftest_ok = False
        if not quiet:
            self.set_phase("disconnected", "link closed by operator")
        return {"ok": True}

    def _on_batt(self, v):
        self.batt_v = v

    def push_sim_frame(self, data):
        """A JPEG of the fish's point of view rendered by the page (fake mode only)."""
        cam = self.cam
        if not self.args.fake or not isinstance(cam, SimCamera):
            return False
        return cam.push(data)

    # ------------------------------------------------------------------ commands
    def command(self, d):
        cmd = d.get("cmd")
        try:
            fn = getattr(self, "cmd_" + str(cmd), None)
            if fn is None:
                return {"ok": False, "error": "Unknown command '%s'." % cmd}
            with self.lock:
                return fn(d)
        except Exception as e:  # noqa: BLE001
            self.log("Command %s failed: %s" % (cmd, e), "warn")
            return {"ok": False, "error": str(e)}

    def cmd_connect(self, d):
        return self.connect(d.get("fin_ip", self.args.fin_ip), d.get("cam_ip", self.args.cam_ip))

    def cmd_disconnect(self, d):
        return self.disconnect()

    def cmd_mission(self, d):
        if self.phase in FLYING:
            return {"ok": False, "error": "Cannot change the mission while the fish is flying. Stop the mission first."}
        if self.phase == "armed":
            return {"ok": False, "error": "Disarm to change the mission."}
        try:
            speed = float(d["speed"]); target = [float(v) for v in d["target"]]
        except (KeyError, TypeError, ValueError):
            return {"ok": False, "error": "Mission needs speed (m/s) and target [x, y, z] in metres."}
        if not speed > 0:
            return {"ok": False, "error": "Launch speed must be above 0 m/s."}
        if len(target) != 3 or math.hypot(*target) < 0.5:
            return {"ok": False, "error": "Target must be at least 0.5 m from the launch point."}
        self.mission = {"speed": speed, "target": target, "set": True, "source": d.get("source") or "typed in"}
        if self.link is not None:
            if self.link.fake:
                self.link.set_mission(speed, target)
            elif self.args.guidance == "fish":
                self.link.send_text("MISSION,%.2f,%.2f,%.2f,%.2f" % (speed, *target))
        plen = math.hypot(*target)
        self.log("Mission set: target (%.2f, %.2f, %.2f) m, %.2f m at %.1f m/s, expected %.1f s. Hand-over at %.0f %%." % (*target, plen, speed, plen / speed, self.cfg["handover_threshold"] * 100), "good")
        return {"ok": True, "path_len": plen, "eta_s": plen / speed}

    def cmd_calibrate(self, d):
        if self.phase != "ready":
            return {"ok": False, "error": "Calibrate only when Ready (connected, not armed)."}
        if self.imu.last_sample_time is None:
            return {"ok": False, "error": "No IMU samples yet. Is the fish node streaming IMU lines?"}
        self.imu.start_calibration(self.cfg["calib_s"])
        self.log("IMU calibration started: hold the fish still for %.0f s." % self.cfg["calib_s"])
        threading.Thread(target=self._finish_calibration, daemon=True).start()
        return {"ok": True}

    def _finish_calibration(self):
        time.sleep(self.cfg["calib_s"] + 0.2)
        self.calibrated = self.imu._cal is None and self.imu.last_sample_time is not None
        if self.calibrated:
            gb = self.imu.gyro_bias
            self.log("IMU calibrated: gyro bias (%.2f, %.2f, %.2f) deg/s, accel residual %.2f m/s2." % (gb[0], gb[1], gb[2], float(np.linalg.norm(self.imu.accel_bias))), "good")
        else:
            self.log("IMU calibration did not finish (no samples).", "warn")

    def cmd_selftest(self, d):
        if self.phase != "ready":
            return {"ok": False, "error": "Self-test only when Ready."}
        threading.Thread(target=self._run_selftest, daemon=True).start()
        return {"ok": True}

    def _run_selftest(self):
        self.log("Fin self-test: wiggling yaw pair then pitch pair.")
        acked0 = self.link.acked if self.link else 0
        for step in [(25, 0), (-25, 0), (0, 25), (0, -25), (0, 0)]:
            self.selftest_cmd = step
            time.sleep(0.45)
        self.selftest_cmd = None
        moved = (self.link is not None) and (self.link.acked > acked0)
        self.selftest_ok = moved
        self.log("Fin self-test passed: fins acknowledged every step." if moved else "Fin self-test failed: no ACKs while wiggling.", "good" if moved else "warn")

    ADVISORY_NAMES = {"calibrated": "IMU calibration", "selftest": "fin self-test", "batt": "battery check"}

    def preflight(self):
        """Required items gate Arm; advisory items only produce a warning (design section 12)."""
        pf = {
            "link": bool(self.connected and self.link and self.link.alive),
            "mission": bool(self.mission["set"]),
            "calibrated": self.calibrated,
            "selftest": self.selftest_ok,
            "batt": (self.batt_v is None and self.connected) or (self.batt_v is not None and self.batt_v >= self.cfg["batt_min_v"]),
        }
        pf["can_arm"] = pf["link"] and pf["mission"]
        pf["skipped"] = [name for key, name in self.ADVISORY_NAMES.items() if not pf[key]]
        return pf

    def cmd_arm(self, d):
        if self.phase == "armed":
            return {"ok": False, "error": "Already armed."}
        if self.phase != "ready":
            return {"ok": False, "error": "Arm only when Ready."}
        pf = self.preflight()
        if not pf["can_arm"]:
            missing = [n for k, n in (("link", "fin board link"), ("mission", "mission target and speed")) if not pf[k]]
            return {"ok": False, "error": "Cannot arm without " + " and ".join(missing) + "."}
        now = time.time()
        if self.arm_pending_until and now < self.arm_pending_until:
            self.arm_pending_until = None
            if pf["skipped"]:
                self.log("Armed without " + ", ".join(pf["skipped"]) + ".", "warn")
            self.set_phase("armed", "armed by operator")
            return {"ok": True, "armed": True, "skipped": pf["skipped"]}
        self.arm_pending_until = now + 5.0
        self.log("Arm requested: press Confirm arm within 5 s." + (" Skipped: " + ", ".join(pf["skipped"]) + "." if pf["skipped"] else ""))
        return {"ok": True, "armed": False, "pending_s": 5.0, "skipped": pf["skipped"]}

    def cmd_disarm(self, d):
        self.arm_pending_until = None
        if self.phase == "armed":
            self.launch_pending_until = None
            self.set_phase("ready", "disarmed by operator")
        return {"ok": True}

    def cmd_autolaunch(self, d):
        self.autolaunch = bool(d.get("on", True))
        return {"ok": True}

    def cmd_launch(self, d):
        if self.phase != "armed":
            return {"ok": False, "error": "Launch only when Armed."}
        if self.link and self.link.fake:
            self.link.launch()
        elif self.args.guidance == "fish" and self.link:
            self.link.send_text("LAUNCH")
        if self.autolaunch and not (self.link and self.link.fake and False):
            self.launch_pending_until = time.time() + self.cfg["launch_wait_s"]
            self.log("Launch commanded: waiting for the accelerometer to see the launch (up to %.0f s)." % self.cfg["launch_wait_s"])
            return {"ok": True, "pending": True}
        self._do_launch("launch command")
        return {"ok": True, "pending": False}

    def _do_launch(self, why):
        self.launch_pending_until = None
        self.launch_time = time.time()
        self.history.clear()
        self.imu.launch(self.mission["speed"])
        self.guidance.fault = None
        self.set_phase("imu_glide", why)

    def cmd_abort(self, d):
        """Operator abort: fins to neutral and straight back to Ready, like Disarm (design section 14).
        The Aborted phase is reserved for automatic faults (link lost, path overrun)."""
        if self.phase == "armed":
            return self.cmd_stop(d)
        if self.phase not in FLYING:
            return {"ok": False, "error": "Nothing to abort."}
        self._end_mission("ABORT pressed, fins to neutral, back to Ready", "crit")
        return {"ok": True, "from": "flight"}

    def cmd_fins_neutral(self, d):
        if not self.link:
            return {"ok": False, "error": "Not connected."}
        self.override["yaw"] = self.override["pitch"] = 0.0
        self.guidance.prev_yaw = self.guidance.prev_pitch = 0.0
        self.link.home()
        self.log("Fins commanded to neutral.")
        return {"ok": True}

    def cmd_override(self, d):
        on = bool(d.get("on", self.override["on"]))
        if on and self.phase not in FLYING:
            return {"ok": False, "error": "Manual override is only available while flying."}
        was = self.override["on"]
        self.override["on"] = on
        fm = self.cfg["fin_max"]
        self.override["yaw"] = max(-fm, min(fm, float(d.get("yaw", self.override["yaw"]))))
        self.override["pitch"] = max(-fm, min(fm, float(d.get("pitch", self.override["pitch"]))))
        if on != was:
            self.log("Manual fin override ON. Guidance paused." if on else "Manual fin override off. Guidance resumed.", "warn" if on else "info")
        return {"ok": True}

    def cmd_handover_threshold(self, d):
        v = float(d.get("value", 80))
        if not 50 <= v <= 99:
            return {"ok": False, "error": "Threshold must be between 50 and 99 %."}
        self.cfg["handover_threshold"] = v / 100.0
        self.log("Hand-over threshold set to %.0f %%." % v)
        return {"ok": True}

    def cmd_tune(self, d):
        k = d.get("key")
        if k not in TUNABLE:
            return {"ok": False, "error": "'%s' is not a tunable parameter." % k}
        cur = self.cfg[k]
        v = d.get("value")
        if isinstance(cur, bool):
            v = bool(v)
        elif isinstance(cur, int) and not isinstance(cur, bool):
            v = int(float(v))
        elif isinstance(cur, float):
            v = float(v)
        elif isinstance(cur, str):
            v = str(v)
        if k == "vision_mode" and v not in ("beacon", "swarm"):
            return {"ok": False, "error": "vision_mode must be 'beacon' or 'swarm'."}
        self.cfg[k] = v
        if k in ("vision_mode", "sim_quads"):
            if self.link is not None and self.link.fake:
                self.link.set_swarm(self.cfg["vision_mode"] == "swarm", self.cfg["sim_quads"])
            if k == "vision_mode":
                if self.vision is not None:
                    self.vision.tracker.reset()
                self.log("Vision mode: %s." % ("swarm, following the biggest cluster's centroid" if v == "swarm" else "beacon, brightest source"))
        return {"ok": True, "key": k, "value": v}

    def cmd_swarm(self, d):
        """Pick which swarm to follow: {"id": 3} | {"action": "next"|"prev"|"auto"} | {"x": px, "y": px}."""
        if self.vision is None:
            return {"ok": False, "error": "Not connected."}
        if self.cfg["vision_mode"] != "swarm":
            return {"ok": False, "error": "Switch the vision mode to swarm first."}
        r = self.vision.select(d)
        if r["ok"]:
            self.log("Following swarm %s." % ("S%d" % r["selected"] if r["selected"] is not None else "auto (biggest)"))
        return r

    def cmd_force_handover(self, d):
        if self.phase != "imu_glide":
            return {"ok": False, "error": "Force hand-over only during IMU glide."}
        self.set_phase("handover", "forced by operator", "warn")
        return {"ok": True}

    def cmd_return_to_imu(self, d):
        if self.phase not in ("handover", "cv_homing"):
            return {"ok": False, "error": "Return to IMU only during hand-over or vision homing."}
        self.set_phase("imu_glide", "returned to IMU by operator", "warn")
        return {"ok": True}

    def cmd_stop(self, d):
        """Routine end of a mission from any phase after Ready: fins neutral, back to Ready.
        Keeps the link, calibration, self-test, mission parameters and the flight history."""
        if self.phase in ("disconnected", "ready"):
            return {"ok": False, "error": "No mission to stop."}
        was = self.phase
        why = {"armed": "disarmed by operator", "arrived": "back to Ready", "aborted": "back to Ready"}.get(was, "mission stopped by operator, fins to neutral")
        self._end_mission(why, "warn" if was in FLYING else "info")
        return {"ok": True, "from": was}

    def _end_mission(self, why, level):
        self.launch_pending_until = None
        self.arm_pending_until = None
        self.override["on"] = False
        self.selftest_cmd = None
        self.guidance.prev_yaw = self.guidance.prev_pitch = 0.0
        self.imu.reset_flight()
        self.launch_time = None
        if self.link:
            self.link.home()
            if self.link.fake:
                self.link.reset()
        self.set_phase("ready", why, level)

    def cmd_new_mission(self, d):
        return self.cmd_stop(d)

    # ------------------------------------------------------------------ loop
    def loop(self):
        last = time.time()
        while not self._stop.is_set():
            t0 = time.time()
            dt = min(0.2, t0 - last); last = t0
            try:
                self.tick(dt)
            except Exception as e:  # noqa: BLE001
                self.log("Control loop error: %s" % e, "crit")
            time.sleep(max(0.0, 0.05 - (time.time() - t0)))

    def tick(self, dt):
        with self.lock:
            link = self.link
            now = time.time()
            if self.arm_pending_until and now > self.arm_pending_until:
                self.arm_pending_until = None
                self.log("Arm timed out. Press Arm again.")
            if self.connected and link is not None and not link.alive and self.phase in FLYING:
                self.set_phase("aborted", "fin link lost for more than 1.5 s (fins go neutral on the fish)", "crit")
            if self.phase == "armed" and self.launch_pending_until:
                spike = abs(float(self.imu.last_accel[0])) > self.cfg["launch_detect_g"] * G
                if spike:
                    self._do_launch("launch detected by accelerometer")
                elif now > self.launch_pending_until:
                    self.log("No launch seen on the accelerometer in %.0f s; starting IMU glide anyway." % self.cfg["launch_wait_s"], "warn")
                    self._do_launch("launch command (no accelerometer spike)")

            cv, _ = self.vision.latest() if self.vision else (Vision._empty(), None)
            target = self.mission["target"] or [0.0, 1.0, 0.0]
            est = self.imu.snapshot(target)
            link_ok = bool(link and link.alive)
            yaw_cmd = pitch_cmd = 0.0
            if self.phase in FLYING:
                ovr = (self.override["yaw"], self.override["pitch"]) if self.override["on"] else None
                if self.args.guidance == "fish" and self.phase == "imu_glide" and ovr is None:
                    # the fish steers itself; still evaluate hand-over
                    _, _, events = self.guidance.step(dt, est, self.imu.rotation(), target, cv, link_ok, override=(0.0, 0.0))
                    yaw_cmd = pitch_cmd = 0.0
                else:
                    yaw_cmd, pitch_cmd, events = self.guidance.step(dt, est, self.imu.rotation(), target, cv, link_ok, override=ovr)
                for level, msg in events:
                    self.log(msg, level)
                    if self.phase in ("arrived", "aborted"):
                        self.imu.stop()
            if self.selftest_cmd is not None:
                yaw_cmd, pitch_cmd = self.selftest_cmd
            self.fin_cmd = (yaw_cmd, pitch_cmd)
            if link is not None and self.connected:
                if not (self.args.guidance == "fish" and self.phase == "imu_glide" and not self.override["on"]):
                    link.send(yaw_cmd, pitch_cmd)
                else:
                    link.send_text("PING")
            self.telemetry = self.build_telemetry(est, cv, link, link_ok, yaw_cmd, pitch_cmd)
            if self.phase in FLYING and now - self._last_hist >= 0.1:
                self._last_hist = now
                self.history.append({"t": round(self.t_mission(), 2), "phase": self.phase, "x": est["x"], "y": est["y"], "z": est["z"],
                                     "progress": est["progress"], "dist": est["dist"], "roll": est["roll"], "pitch": est["pitch"], "yaw": est["yaw"],
                                     "yaw_cmd": round(yaw_cmd, 1), "pitch_cmd": round(pitch_cmd, 1),
                                     "fin_yaw": round(link.fin_yaw, 1) if link else 0, "fin_pitch": round(link.fin_pitch, 1) if link else 0,
                                     "cv_state": cv["state"], "conf": cv["conf"], "ex_px": cv["ex_px"], "ey_px": cv["ey_px"]})

    def build_telemetry(self, est, cv, link, link_ok, yaw_cmd, pitch_cmd):
        m = self.mission
        plen = math.hypot(*m["target"]) if m["set"] else None
        state = cv["state"]
        if state == "seen" and self.phase == "cv_homing" and self.guidance.cv_locked_frames >= 5:
            state = "locked"
        if self.phase == "arrived" and cv["found"]:
            state = "locked"
        return {
            "type": "telemetry", "t": round(time.time(), 3), "t_mission": None if self.t_mission() is None else round(self.t_mission(), 2),
            "phase": self.phase, "progress": est.get("progress", 0.0) if self.phase in FLYING or self.phase in ("arrived",) else 0.0,
            "connected": self.connected, "fake": bool(self.args.fake), "guidance_mode": self.args.guidance,
            "link": {"fin_rtt_ms": None if not link or link.rtt_ms is None else round(link.rtt_ms, 1), "sent": link.sent if link else 0, "acked": link.acked if link else 0,
                     "alive": link_ok, "imu_age_ms": None if not link or link.imu_age_ms is None else round(link.imu_age_ms), "imu_rate": round(link.imu_rate, 1) if link else 0.0, "error": link.error if link else None},
            "cam": {"connected": bool(self.cam and self.cam.connected), "fps": round(self.vision.fps, 1) if self.vision else 0.0, "cv_ms": round(self.vision.proc_ms, 1) if self.vision else 0.0,
                    "error": getattr(self.cam, "error", None) if self.cam else None, "yolo": bool(self.vision and self.vision.yolo),
                    "yolo_kind": self.vision.yolo.kind if self.vision and self.vision.yolo else None, "yolo_error": self.vision.yolo_error if self.vision else None},
            "imu": {"roll": est["roll"], "pitch": est["pitch"], "yaw": est["yaw"], "samples": est["samples"]},
            "est": {"x": est["x"], "y": est["y"], "z": est["z"], "dist": est.get("dist"), "drift": est["drift"], "drift_warn": est["drift"] > 0.4},
            "cv": {"state": state, "conf": cv["conf"], "frames": cv["frames"], "ex_px": cv["ex_px"], "ey_px": cv["ey_px"], "w": cv["w"], "h": cv["h"],
                   "radius_px": cv["radius_px"], "boxes": cv["boxes"], "source": cv["source"], "lost_s": cv["lost_s"],
                   "mode": cv.get("mode", "beacon"), "n_sources": cv.get("n_sources", 0), "swarm_id": cv.get("swarm_id"), "swarm_n": cv.get("swarm_n", 0),
                   "swarms": [{k: sw[k] for k in ("id", "x", "y", "n", "spread", "selected")} for sw in cv.get("swarms", [])]},
            "sim": self._sim_block(link),
            "fins": {"yaw_cmd": round(yaw_cmd, 1), "pitch_cmd": round(pitch_cmd, 1), "yaw": round(link.fin_yaw, 1) if link else 0.0, "pitch": round(link.fin_pitch, 1) if link else 0.0},
            "handover": {"threshold": self.cfg["handover_threshold"], "ok": self.guidance.last_conditions, "frames_needed": self.cfg["beacon_frames_needed"], "conf_needed": self.cfg["conf_needed"]},
            "mission": {"speed": m["speed"], "target": m["target"], "set": m["set"], "path_len": plen, "eta_s": None if not plen else round(plen / m["speed"], 1), "source": m["source"]},
            "preflight": self.preflight(), "batt_v": None if self.batt_v is None else round(self.batt_v, 2),
            "override": dict(self.override), "autolaunch": self.autolaunch,
            "arm_pending_s": None if not self.arm_pending_until else max(0.0, round(self.arm_pending_until - time.time(), 1)),
            "launch_pending_s": None if not self.launch_pending_until else max(0.0, round(self.launch_pending_until - time.time(), 1)),
            "calibrating_s": self.imu.calibrating_s, "selftest_running": self.selftest_cmd is not None, "fault": self.guidance.fault,
            "cfg": {k: self.cfg[k] for k in TUNABLE},
        }

    def _sim_block(self, link):
        """Fake mode only: true quad positions and the true fish pose, for the 3D demo and the simulated camera."""
        if link is None or not getattr(link, "fake", False):
            return {"quads": [], "fish": None, "cam_source": None}
        with link.lock:
            pos = link.pos.tolist(); yaw, pitch, roll = euler_deg(link.R)
        return {"quads": link.quad_positions() if self.cfg["vision_mode"] == "swarm" else [],
                "fish": {"pos": [round(v, 3) for v in pos], "yaw": round(yaw, 1), "pitch": round(pitch, 1), "roll": round(roll, 1), "launched": bool(link.launched)},
                "cam_source": self.cam.source if isinstance(self.cam, SimCamera) else None}

    # ------------------------------------------------------------------ logs
    def export_json(self):
        return json.dumps({"mission": self.mission, "cfg": self.cfg, "events": list(self.events), "history": list(self.history)}, indent=1)

    def export_csv(self):
        buf = io.StringIO()
        rows = list(self.history)
        if not rows:
            return "t\n"
        w = csv.DictWriter(buf, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
        return buf.getvalue()
