"""
Camera gimbal controller: keep the detected target (beacon or swarm centroid) centred.

Pan (GPIO 18) and tilt (GPIO 17) are absolute servo angles with hard limits on the board and
here. Per tick the pixel error of the target is turned into a small angle step, like the pan/tilt
mode of the old light tracker:

    pan  += sign_pan  * (Kp * ex_norm + Ki * sum_ex) * HFOV/2        (ex_norm = ex_px / (w/2))
    tilt += sign_tilt * -(Kp * ey_norm + Ki * sum_ey) * VFOV/2       (ey positive = target below)

steps are limited to `gimbal_max_step` degrees per tick and the result is clamped to the limits.
With no target for `gimbal_return_s` seconds the camera glides back to its centre position.

For the fins: `camera_offset()` returns how far the camera looks right of and above the nose,
in degrees; the vision law adds that to the residual pixel error so the fish turns until the
camera is centred again (the target then sits straight ahead).
"""


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Gimbal:
    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        c = self.cfg
        self.pan = c["pan_home"]
        self.tilt = c["tilt_home"]
        self.int_ex = self.int_ey = 0.0
        self.lost_t = 0.0
        self.tracking = False

    def home(self):
        self.reset()

    def step(self, cv, dt):
        """cv: detector result (found, ex_px, ey_px, w, h). Returns (pan, tilt) to command."""
        c = self.cfg
        if not c.get("gimbal_on", True):
            self.tracking = False
            return self.pan, self.tilt
        ms = float(c.get("gimbal_max_step", 6.0))
        if cv.get("found"):
            self.lost_t = 0.0
            self.tracking = True
            ex_n = cv["ex_px"] / (cv["w"] / 2.0)
            ey_n = cv["ey_px"] / (cv["h"] / 2.0)
            if abs(cv["ex_px"]) < c.get("gimbal_deadband_px", 6):
                ex_n = 0.0; self.int_ex = 0.0
            if abs(cv["ey_px"]) < c.get("gimbal_deadband_px", 6):
                ey_n = 0.0; self.int_ey = 0.0
            self.int_ex = clamp(self.int_ex + ex_n * dt, -2.0, 2.0)
            self.int_ey = clamp(self.int_ey + ey_n * dt, -2.0, 2.0)
            sp = -1.0 if c.get("invert_pan") else 1.0
            st = -1.0 if c.get("invert_tilt") else 1.0
            dpan = sp * (c["gimbal_kp"] * ex_n + c["gimbal_ki"] * self.int_ex) * c["cam_hfov"] / 2.0
            dtilt = st * -(c["gimbal_kp"] * ey_n + c["gimbal_ki"] * self.int_ey) * c["cam_vfov"] / 2.0
            self.pan += clamp(dpan, -ms, ms)
            self.tilt += clamp(dtilt, -ms, ms)
        else:
            self.lost_t += dt
            self.int_ex = self.int_ey = 0.0
            if self.lost_t >= c.get("gimbal_return_s", 1.0):
                self.tracking = False
                self.pan += clamp(c["pan_home"] - self.pan, -ms, ms)
                self.tilt += clamp(c["tilt_home"] - self.tilt, -ms, ms)
        self.pan = clamp(self.pan, c["pan_min"], c["pan_max"])
        self.tilt = clamp(self.tilt, c["tilt_min"], c["tilt_max"])
        return self.pan, self.tilt

    def camera_offset(self, actual_pan=None, actual_tilt=None):
        """(right_deg, up_deg): where the camera looks relative to the nose, from the board's
        reported angles when available, else from the commanded ones."""
        c = self.cfg
        pan = self.pan if actual_pan is None else actual_pan
        tilt = self.tilt if actual_tilt is None else actual_tilt
        sp = -1.0 if c.get("invert_pan") else 1.0
        st = -1.0 if c.get("invert_tilt") else 1.0
        return sp * (pan - c["pan_home"]), st * (tilt - c["tilt_home"])
