"""
Mission state machine and the two steering laws.

Phases (mirrored one-to-one in the frontend stepper):
  disconnected -> ready -> armed -> imu_glide -> handover -> cv_homing -> arrived
                                       \\_____________ aborted ______________/

Steering
  IMU glide    : point the nose at the target using the dead-reckoned position
                 and the IMU attitude, with the target direction expressed in the
                 body frame.                  yaw_fin = Kp_imu * angle_right_of_nose
  Vision homing: keep the beacon centred.     yaw_fin = (Kp*ex_norm + Ki*sum) * fin_max
  Hand-over    : ~1 s linear blend of the two.
Sign convention (from pid_fins.ino / tracker.py): positive yaw fin turns the
nose right, positive pitch fin lifts the nose.
"""
import math
import time

import numpy as np

PHASES = ["disconnected", "ready", "armed", "imu_glide", "handover", "cv_homing", "arrived", "aborted"]
FLYING = {"imu_glide", "handover", "cv_homing"}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Guidance:
    def __init__(self, cfg):
        self.cfg = cfg                      # dict, shared with the station (live tunable)
        self.phase = "disconnected"
        self.handover_t = None
        self.cv_locked_frames = 0
        self.int_ex = 0.0
        self.int_ey = 0.0
        self.prev_yaw = 0.0
        self.prev_pitch = 0.0
        self.launch_t = None
        self.last_conditions = [False, False, False, False]
        self.fault = None
        self.overrun_warned = False

    # ------------------------------------------------------------------ phases
    def set_phase(self, phase):
        self.phase = phase
        if phase == "imu_glide":
            self.launch_t = time.time()
            self.overrun_warned = False
            self.int_ex = self.int_ey = 0.0
            self.cv_locked_frames = 0
        if phase == "handover":
            self.handover_t = time.time()

    def handover_conditions(self, progress, cv, link_ok):
        c = self.cfg
        return [
            progress >= c["handover_threshold"],
            cv["frames"] >= c["beacon_frames_needed"],
            cv["conf"] >= c["conf_needed"],
            bool(link_ok),
        ]

    # ------------------------------------------------------------------ laws
    def imu_law(self, est, R, target):
        """
        est: world position (3,), R: body->world rotation (columns f, r, u), target: (3,).
        The target direction is expressed in the BODY frame and the nose is steered onto it,
        exactly like the vision law steers onto the beacon's pixel offset. Working in the
        body frame avoids the heading singularity of a nose-up flight: a yaw fin rotates
        the nose about the body up axis, which at 75 deg pitch is nearly horizontal.
        """
        to_t = np.asarray(target, dtype=float) - np.asarray(est, dtype=float)
        d = np.linalg.norm(to_t)
        if d < 1e-3:
            return 0.0, 0.0
        b = np.asarray(R).T @ (to_t / d)          # (forward, right, up) components of the target direction
        yaw_err = math.degrees(math.atan2(b[1], b[0]))     # + = target right of the nose -> turn right
        pitch_err = math.degrees(math.atan2(b[2], b[0]))   # + = target above the nose   -> nose up
        k = self.cfg["kp_imu"]
        fm = self.cfg["fin_max"]
        return clamp(k * yaw_err, -fm, fm), clamp(k * pitch_err, -fm, fm)

    def cv_law(self, cv, dt):
        """cv: dict with found, ex_px, ey_px, w, h."""
        c = self.cfg
        fm = c["fin_max"]
        if not cv["found"]:
            self.int_ex = self.int_ey = 0.0
            return 0.0, 0.0, True
        ex_n = cv["ex_px"] / (cv["w"] / 2.0)
        ey_n = cv["ey_px"] / (cv["h"] / 2.0)
        in_db = math.hypot(cv["ex_px"], cv["ey_px"]) < c["deadband_px"]
        if in_db:
            self.int_ex = self.int_ey = 0.0
            return 0.0, 0.0, True
        self.int_ex = clamp(self.int_ex + ex_n * dt, -2.0, 2.0)
        self.int_ey = clamp(self.int_ey + ey_n * dt, -2.0, 2.0)
        yaw = (c["kp"] * ex_n + c["ki"] * self.int_ex) * fm
        pitch = -(c["kp"] * ey_n + c["ki"] * self.int_ey) * fm
        if c.get("invert_yaw"): yaw = -yaw
        if c.get("invert_pitch"): pitch = -pitch
        return clamp(yaw, -fm, fm), clamp(pitch, -fm, fm), False

    # ------------------------------------------------------------------ step
    def step(self, dt, est, R, target, cv, link_ok, override=None):
        """
        Returns (yaw_cmd, pitch_cmd, events) and advances the phase.
        est: {'x','y','z','progress','dist'}; R: body->world rotation; cv: detector result; override: (yaw, pitch) or None.
        """
        events = []
        c = self.cfg
        pos = np.array([est["x"], est["y"], est["z"]])
        progress = est.get("progress", 0.0)
        conds = self.handover_conditions(progress, cv, link_ok)
        self.last_conditions = conds
        yaw = pitch = 0.0

        if self.phase == "imu_glide":
            if all(conds):
                self.set_phase("handover"); events.append(("info", "Phase: Hand-over — all four conditions met"))
            elif progress > 1.15:
                self.set_phase("aborted"); self.fault = "overran the path without a beacon lock"
                events.append(("crit", "Aborted: progress passed 115 % with no hand-over. Fins to neutral."))
            if progress >= 1.0 and self.phase == "imu_glide":
                # the estimate says we are past the target: do not chase a point behind the nose,
                # glide straight and give vision until 115 % to find the beacon
                if not self.overrun_warned:
                    self.overrun_warned = True
                    events.append(("warn", "Past 100 % of the path on IMU with no beacon lock. Fins neutral; abort at 115 %."))
                yaw = pitch = 0.0
            else:
                yaw, pitch = self.imu_law(pos, R, target)
        elif self.phase == "handover":
            w = clamp((time.time() - self.handover_t) / c["handover_blend_s"], 0.0, 1.0)
            y1, p1 = self.imu_law(pos, R, target)
            y2, p2, _ = self.cv_law(cv, dt)
            yaw, pitch = (1 - w) * y1 + w * y2, (1 - w) * p1 + w * p2
            if w >= 1.0:
                self.set_phase("cv_homing"); events.append(("info", "Phase: Vision homing — blend complete, laptop vision now steers"))
        elif self.phase == "cv_homing":
            yaw, pitch, in_db = self.cv_law(cv, dt)
            self.cv_locked_frames = self.cv_locked_frames + 1 if (cv["found"] and in_db) else 0
            if not cv["found"] and cv.get("lost_s", 0.0) > c["cv_lost_timeout_s"]:
                self.set_phase("imu_glide"); events.append(("warn", "Beacon lost for %.1f s, back to IMU glide" % cv["lost_s"]))
            if progress >= 1.0 or est.get("dist", 9e9) < c["arrive_dist_m"] or (cv["found"] and cv.get("radius_px", 0) >= c["arrive_radius_px"]):
                self.set_phase("arrived")
                events.append(("good", "Phase: Arrived — %s" % ("beacon fills the frame" if cv["found"] else "planned distance covered")))
                yaw = pitch = 0.0

        if override is not None and self.phase in FLYING:
            yaw, pitch = override

        # rate limit (max_step is degrees per 20 Hz tick, as in the tracker)
        ms = c["max_step"]
        yaw = self.prev_yaw + clamp(yaw - self.prev_yaw, -ms, ms)
        pitch = self.prev_pitch + clamp(pitch - self.prev_pitch, -ms, ms)
        if self.phase not in FLYING:
            yaw = pitch = 0.0
        self.prev_yaw, self.prev_pitch = yaw, pitch
        return yaw, pitch, events
