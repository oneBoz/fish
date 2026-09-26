"""
ICM20948 attitude + dead-reckoning estimator.

Frames
------
Body frame (f, r, u): f = nose/forward, r = right, u = up. Right-handed (f x r = u).
The raw sensor axes are mapped onto it with `axes` (config --imu-axes; default
"x,y,z" = sensor x forward, sensor y right, sensor z up). The mapping must stay
right-handed: "x,-y,-z" is a valid re-mounting, "x,-y,z" is a mirror image and is
refused, because a mirrored frame reverses every rotation.

World frame (x, y, z): y is up, z is the direction the nose pointed at launch,
x is to the right of that. The mission target is entered in this frame, so
"target (0, 8, 0)" means "8 m straight up from the launch point" and
"(2, 8, 1)" means 2 m right, 8 m up, 1 m ahead.

Attitude is a rotation matrix R (body -> world; columns are f, r, u expressed in
world coordinates). Each gyro sample rotates R by the body rates; the
accelerometer pulls the estimated "up" towards the measured gravity direction
(complementary filter, gain `acc_gain`). This is correct at any pitch, which
matters because the fish flies nose-up. Euler angles are derived for display:
yaw = nose right of the launch heading, pitch = nose above horizontal,
roll = right side down (degrees). Yaw is gyro-only and zeroed at launch.

Dead reckoning: velocity starts as `speed` along the nose at launch and is
nudged by the integrated linear acceleration, with a leak back towards the
entered speed so noise cannot run away. `progress` is the component of the
estimated displacement along the planned path, divided by its length.
"""
import math
import threading
import time

import numpy as np

G = 9.80665
UP_W = np.array([0.0, 1.0, 0.0])
# Level, nose along +z: columns f=(0,0,1), r=(1,0,0), u=(0,1,0).
R_LEVEL = np.array([[0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [1.0, 0.0, 0.0]])


def parse_axes(spec):
    """'x,-y,-z' -> list of (sensor index, sign) for (forward, right, up). Right-handed only."""
    out = []
    for tok in spec.split(","):
        tok = tok.strip().lower()
        sign = -1.0 if tok.startswith("-") else 1.0
        name = tok.lstrip("+-")
        if name not in "xyz" or len(name) != 1:
            raise ValueError("imu axes must be three of x,y,z like 'x,y,z' or 'x,-y,-z'")
        out.append(("xyz".index(name), sign))
    if len(out) != 3 or len({i for i, _ in out}) != 3:
        raise ValueError("imu axes must name x, y and z once each, like 'x,y,z'")
    m = np.zeros((3, 3))
    for row, (idx, sign) in enumerate(out):
        m[row, idx] = sign
    if np.linalg.det(m) < 0:
        raise ValueError("--imu-axes '%s' is a mirror image (left-handed). Flip one more sign, e.g. '%s'."
                         % (spec, ",".join(("-" if s > 0 else "") + "xyz"[i] if k == 2 else ("-" if s < 0 else "") + "xyz"[i]
                                            for k, (i, s) in enumerate(out))))
    return out


_parse_axes = parse_axes


def skew(w):
    return np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])


def exp_so3(w):
    """Rotation matrix for a rotation vector w (radians)."""
    th = float(np.linalg.norm(w))
    if th < 1e-9:
        return np.eye(3) + skew(w)
    k = skew(w / th)
    return np.eye(3) + math.sin(th) * k + (1.0 - math.cos(th)) * (k @ k)


def orthonormalize(R):
    f = R[:, 0] / max(1e-9, np.linalg.norm(R[:, 0]))
    r = R[:, 1] - np.dot(R[:, 1], f) * f
    r /= max(1e-9, np.linalg.norm(r))
    u = np.cross(f, r)
    return np.column_stack([f, r, u])


def euler_deg(R):
    """(yaw, pitch, roll) in degrees from a body->world matrix, see module doc."""
    f, r, u = R[:, 0], R[:, 1], R[:, 2]
    yaw = math.degrees(math.atan2(f[0], f[2]))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, f[1]))))
    roll = math.degrees(math.atan2(-r[1], u[1]))
    return yaw, pitch, roll


def rot_y(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


class ImuEstimator:
    def __init__(self, axes="x,y,z", acc_gain=1.0, leak=4.0, alpha=None):
        self.axes = parse_axes(axes)
        # `alpha` kept for compatibility: an old-style 0.98 filter weight at 50 Hz ~ gain 1.0 rad/s
        self.acc_gain = acc_gain if alpha is None else (1.0 - alpha) * 50.0
        self.leak = leak            # 1/s, pull of velocity back to `speed` along the nose (0.25 s memory for accel deviations)
        self.lock = threading.Lock()
        self.reset()

    def reset(self):
        with self.lock:
            self.R = R_LEVEL.copy()
            self.roll = self.pitch = self.yaw = 0.0
            self.gyro_bias = np.zeros(3)
            self.accel_bias = np.zeros(3)      # body-frame residual at rest after scaling gravity to G
            self.pos = np.zeros(3)             # world x, y, z (m)
            self.vel = np.zeros(3)
            self.speed = 0.0
            self.launched = False
            self.launch_time = None
            self.integrate_after = 0.0         # ignore accel until the launch spike is over
            self.drift = 0.0                   # rough uncertainty of pos (m)
            self.last_sample_time = None
            self.last_accel = np.zeros(3)
            self.last_gyro = np.zeros(3)
            self.samples = 0
            self.calibrated = False
            self._cal = None

    # ---- calibration (fish held still, 3 s) ----------------------------------
    def start_calibration(self, seconds=3.0):
        with self.lock:
            self._cal = {"until": time.time() + seconds, "g": [], "a": []}

    @property
    def calibrating_s(self):
        c = self._cal
        return None if not c else max(0.0, c["until"] - time.time())

    def _map(self, raw):
        return np.array([raw[i] * s for i, s in self.axes])

    def _align_to_gravity(self, a_body):
        """Rotate R so that its 'up' matches the measured gravity direction, keeping yaw at 0."""
        v = a_body / max(1e-9, np.linalg.norm(a_body))
        v_pred = self.R.T @ UP_W
        axis = np.cross(v, v_pred)                      # body-frame rotation that carries v_pred onto v
        s = float(np.linalg.norm(axis)); c = float(np.dot(v, v_pred))
        if s > 1e-9:
            self.R = orthonormalize(self.R @ exp_so3(axis / s * math.atan2(s, c)))
        self._zero_yaw()

    def _zero_yaw(self):
        yaw = math.atan2(self.R[0, 0], self.R[2, 0])
        self.R = orthonormalize(rot_y(-yaw) @ self.R)

    # ---- sample ingest (from the UDP receiver thread) -------------------------
    def update(self, accel_xyz, gyro_xyz, t=None):
        """accel in m/s^2 (sensor axes), gyro in deg/s (sensor axes)."""
        t = time.time() if t is None else t
        a = self._map(np.asarray(accel_xyz, dtype=float))   # (f, r, u)
        g = self._map(np.asarray(gyro_xyz, dtype=float))    # rates about (f, r, u), deg/s
        with self.lock:
            self.samples += 1
            if self._cal is not None:
                self._cal["g"].append(g)
                self._cal["a"].append(a)
                if t >= self._cal["until"]:
                    gm = np.mean(self._cal["g"], axis=0)
                    am = np.mean(self._cal["a"], axis=0)
                    self.gyro_bias = gm
                    self.accel_bias = am - am / max(1e-6, np.linalg.norm(am)) * G
                    self._cal = None
                    self.calibrated = True
                    self._align_to_gravity(am - self.accel_bias)
            g = g - self.gyro_bias
            a = a - self.accel_bias
            dt = 0.0 if self.last_sample_time is None else min(0.1, max(0.0, t - self.last_sample_time))
            self.last_sample_time = t
            self.last_accel, self.last_gyro = a, g

            # attitude: gyro rotates the body, accelerometer pulls 'up' back onto gravity
            w = np.radians(g)
            norm = float(np.linalg.norm(a))
            if 0.5 * G < norm < 1.5 * G and dt > 0:
                v_meas = a / norm
                v_pred = self.R.T @ UP_W
                # rotating the body estimate by d changes v_pred by -d x v_pred, so d = v_meas x v_pred
                # moves the estimated 'up' towards the measured one
                w = w + self.acc_gain * np.cross(v_meas, v_pred)
            if dt > 0:
                self.R = self.R @ exp_so3(w * dt)
                if self.samples % 25 == 0:
                    self.R = orthonormalize(self.R)
            self.yaw, self.pitch, self.roll = euler_deg(self.R)

            if self.launched and dt > 0:
                fwd = self.R[:, 0]
                if t < self.integrate_after:
                    # launch spike window: the entered speed is the truth, accel is the throw itself
                    self.vel = fwd * self.speed
                    a_world = np.zeros(3)
                else:
                    a_world = self.R @ a - UP_W * G
                    self.vel += a_world * dt
                    self.vel += (fwd * self.speed - self.vel) * min(1.0, self.leak * dt)
                    vmax = 2.0 * self.speed + 0.5
                    vn = float(np.linalg.norm(self.vel))
                    if vn > vmax:
                        self.vel *= vmax / vn
                self.pos += self.vel * dt
                self.drift += (0.02 + 0.05 * float(np.linalg.norm(a_world)) * dt) * dt

    # ---- geometry --------------------------------------------------------------
    def rotation(self):
        """Body (f, r, u) -> world (x right, y up, z forward) rotation matrix."""
        return self.R.copy()

    def forward(self):
        return self.R[:, 0].copy()

    # ---- mission hooks ---------------------------------------------------------
    def launch(self, speed):
        with self.lock:
            self.launched = True
            self.launch_time = time.time()
            self.integrate_after = self.launch_time + 0.6
            self.speed = float(speed)
            self._zero_yaw()
            self.yaw, self.pitch, self.roll = euler_deg(self.R)
            self.pos = np.zeros(3)
            self.vel = self.R[:, 0] * self.speed
            self.drift = 0.0

    def stop(self):
        with self.lock:
            self.launched = False

    def advance_along(self, target, speed, dt):
        """No IMU samples: move the estimate along the planned path at the entered speed (time-based)."""
        with self.lock:
            tgt = np.asarray(target, dtype=float)
            n = float(np.linalg.norm(tgt))
            if n < 1e-6 or not speed or dt <= 0:
                return
            self.pos += tgt / n * float(speed) * dt
            self.drift += 0.05 * dt

    def reset_flight(self):
        """Back to the launch point for the next mission; keeps attitude, biases and calibration."""
        with self.lock:
            self.launched = False
            self.launch_time = None
            self.pos = np.zeros(3)
            self.vel = np.zeros(3)
            self.drift = 0.0

    def snapshot(self, target=None):
        with self.lock:
            pos = self.pos.copy()
            out = {
                "roll": round(self.roll, 1), "pitch": round(self.pitch, 1), "yaw": round(self.yaw, 1),
                "af": round(float(self.last_accel[0]), 2), "ar": round(float(self.last_accel[1]), 2), "au": round(float(self.last_accel[2]), 2),
                "age_ms": None if self.last_sample_time is None else round((time.time() - self.last_sample_time) * 1000),
                "samples": self.samples,
                "x": round(float(pos[0]), 3), "y": round(float(pos[1]), 3), "z": round(float(pos[2]), 3),
                "drift": round(self.drift, 3),
            }
        if target is not None:
            tgt = np.asarray(target, dtype=float)
            length = float(np.linalg.norm(tgt))
            out["dist"] = round(float(np.linalg.norm(tgt - pos)), 3)
            out["progress"] = 0.0 if length < 1e-6 else round(float(np.dot(pos, tgt) / (length * length)), 4)
        return out
