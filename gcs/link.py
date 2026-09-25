"""
UDP link to the fish node (ESP32-S3 running firmware/fish_node/fish_node.ino)
and a complete stand-in (FakeFish + FakeIRCam) for the bench.

Wire protocol (text lines over UDP 4210, all to/from the fin board):
  laptop -> fish   "yaw,pitch\\n"          fin deflection, degrees from neutral
                   "HOME\\n"               fins to neutral
                   "PING\\n"               keep-alive, answered with ACK (new firmware)
  fish -> laptop   "ACK yaw pitch n\\n"    actual fin position + packet count
                   "IMU,ms,ax,ay,az,gx,gy,gz\\n"   accel m/s^2, gyro deg/s, sensor axes, ~50 Hz
                   "BATT,volts\\n"          optional
"""
import math
import random
import socket
import threading
import time

import cv2
import numpy as np

from .imu import G, R_LEVEL, euler_deg, exp_so3, orthonormalize, parse_axes


class FishLink(threading.Thread):
    def __init__(self, ip, port=4210, on_imu=None, on_batt=None):
        super().__init__(daemon=True)
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)
        self.on_imu = on_imu
        self.on_batt = on_batt
        self.sent = 0
        self.acked = 0
        self.rtt_ms = None
        self.last_ack_time = 0.0
        self.last_ack = None
        self.last_imu_time = None
        self.imu_rate = 0.0
        self.batt_v = None
        self.error = None
        self.fin_yaw = 0.0
        self.fin_pitch = 0.0
        self._last_send = 0.0
        self._imu_count = 0
        self._imu_t0 = time.time()
        self._stop = threading.Event()
        self.fake = False

    def stop(self):
        self._stop.set()

    def send(self, yaw, pitch):
        msg = f"{yaw:.1f},{pitch:.1f}\n".encode()
        self._send_raw(msg)

    def home(self):
        self._send_raw(b"HOME\n")

    def send_text(self, text):
        self._send_raw((text.strip() + "\n").encode())

    def _send_raw(self, msg):
        try:
            self.sock.sendto(msg, self.addr)
            self.sent += 1
            self._last_send = time.time()
            self.error = None
        except OSError as e:
            hint = " (is the IP the hotspot broadcast address, e.g. .15?)" if getattr(e, "errno", None) == 13 else ""
            self.error = f"{e}{hint}"

    def run(self):
        while not self._stop.is_set():
            try:
                data, _ = self.sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                continue
            for line in data.decode(errors="ignore").splitlines():
                self._handle(line.strip())

    def _handle(self, line):
        now = time.time()
        if line.startswith("ACK"):
            parts = line.split()
            self.acked += 1
            self.last_ack = line
            self.last_ack_time = now
            self.rtt_ms = (now - self._last_send) * 1000
            if len(parts) >= 3:
                try:
                    self.fin_yaw, self.fin_pitch = float(parts[1]), float(parts[2])
                except ValueError:
                    pass
        elif line.startswith("IMU,"):
            p = line.split(",")
            if len(p) >= 8:
                try:
                    a = (float(p[2]), float(p[3]), float(p[4])); g = (float(p[5]), float(p[6]), float(p[7]))
                except ValueError:
                    return
                self.last_imu_time = now
                self._imu_count += 1
                if now - self._imu_t0 >= 1.0:
                    self.imu_rate = self._imu_count / (now - self._imu_t0); self._imu_count = 0; self._imu_t0 = now
                if self.on_imu:
                    self.on_imu(a, g, now)
        elif line.startswith("BATT,"):
            try:
                self.batt_v = float(line.split(",")[1])
            except (ValueError, IndexError):
                pass
            if self.on_batt:
                self.on_batt(self.batt_v)

    @property
    def alive(self):
        return (time.time() - self.last_ack_time) < 1.5

    @property
    def imu_age_ms(self):
        return None if self.last_imu_time is None else (time.time() - self.last_imu_time) * 1000


# =============================================================================
#  Bench stand-ins
# =============================================================================
class FakeFish(threading.Thread):
    """
    Simulated fish + fin node + ICM20948. Same public surface as FishLink.

    Dynamics: the nose turns at `turn_rate` deg/s per degree of fin deflection
    (yaw fin -> rate about the body up axis, pitch fin -> nose up), integrated
    on a body->world rotation matrix with the *measured* loop period; the fish
    moves at `speed` m/s along its nose once launched. A slow random
    disturbance keeps both guidance phases busy. Frames are rendered by
    FakeIRCam from the true pose. World frame as in imu.py: x right, y up,
    z forward (nose at launch). The IMU it streams is what a real ICM20948
    would report for that motion: specific force (gravity reaction + the
    launch throw) and right-handed body rates, written back into sensor axes
    according to `axes` so any --imu-axes setting round-trips.
    """

    def __init__(self, on_imu=None, on_batt=None, turn_rate=1.0, hfov=62.0, vfov=49.0, seed=None, axes="x,y,z"):
        super().__init__(daemon=True)
        self.fake = True
        self.on_imu, self.on_batt = on_imu, on_batt
        self.rng = random.Random(seed)
        self.turn_rate = turn_rate
        self.axes = parse_axes(axes)
        self.sent = self.acked = 0
        self.rtt_ms = 21.0
        self.last_ack_time = 0.0
        self.last_ack = None
        self.last_imu_time = None
        self.imu_rate = 50.0
        self.batt_v = 7.8
        self.error = None
        self.fin_yaw = self.fin_pitch = 0.0          # actual (slewed)
        self.cmd_yaw = self.cmd_pitch = 0.0
        self.last_cmd_time = 0.0
        self.pos = np.zeros(3)
        self.R = R_LEVEL.copy()
        self.speed = 0.0
        self.launched = False
        self.target = np.array([2.0, 8.0, 1.0])
        self.dist_yaw = self.dist_pitch = 0.0
        self.launch_spike = 0.0
        self.cam = FakeIRCam(self, hfov, vfov)
        self._stop = threading.Event()
        self.lock = threading.RLock()          # reset() calls set_mission() under the lock

    # ---- FishLink surface ----
    def stop(self): self._stop.set()

    def send(self, yaw, pitch):
        self.sent += 1
        self.cmd_yaw, self.cmd_pitch = max(-30.0, min(30.0, yaw)), max(-30.0, min(30.0, pitch))
        self.last_cmd_time = time.time()
        self.acked += 1
        self.last_ack_time = time.time()
        self.rtt_ms = 18.0 + self.rng.random() * 8.0
        self.last_ack = "ACK %.1f %.1f %d" % (self.fin_yaw, self.fin_pitch, self.acked)

    def home(self): self.send(0.0, 0.0)
    def send_text(self, text): pass

    @property
    def alive(self): return (time.time() - self.last_ack_time) < 1.5

    @property
    def imu_age_ms(self): return None if self.last_imu_time is None else (time.time() - self.last_imu_time) * 1000

    # ---- true pose (for diagnostics and the fake camera) ----
    @property
    def yaw(self): return euler_deg(self.R)[0]
    @property
    def pitch(self): return euler_deg(self.R)[1]
    @property
    def roll(self): return euler_deg(self.R)[2]

    def basis(self):
        return self.R[:, 0].copy(), self.R[:, 1].copy(), self.R[:, 2].copy()

    # ---- bench controls ----
    def set_mission(self, speed, target):
        with self.lock:
            self.target = np.asarray(target, dtype=float)
            self.speed = float(speed)
            # the fish starts level with its nose along +z (the estimator's world frame);
            # whatever the entered target implies is the pointing error the IMU phase must correct
            self.R = R_LEVEL.copy()

    def launch(self):
        with self.lock:
            self.launched = True
            self.launch_spike = 0.25       # seconds of +3 g forward acceleration
            self.pos = np.zeros(3)

    def reset(self):
        with self.lock:
            self.launched = False
            self.pos = np.zeros(3)
            self.fin_yaw = self.fin_pitch = self.cmd_yaw = self.cmd_pitch = 0.0
            self.set_mission(self.speed, self.target)

    def run(self):
        period = 0.02
        last = time.time()
        last_batt = 0.0
        n = 0
        while not self._stop.is_set():
            t0 = time.time()
            dt = min(0.1, max(0.0, t0 - last)); last = t0
            n += 1
            with self.lock:
                # fin node behaviour: link timeout -> neutral, slew 3 deg / 20 ms
                if t0 - self.last_cmd_time > 1.0:
                    self.cmd_yaw = self.cmd_pitch = 0.0
                self.fin_yaw += max(-3.0, min(3.0, self.cmd_yaw - self.fin_yaw))
                self.fin_pitch += max(-3.0, min(3.0, self.cmd_pitch - self.fin_pitch))
                # disturbance random walk (deg/s)
                self.dist_yaw = max(-1.5, min(1.5, self.dist_yaw + self.rng.gauss(0, 0.4) * dt))
                self.dist_pitch = max(-1.0, min(1.0, self.dist_pitch + self.rng.gauss(0, 0.3) * dt))
                rates = np.zeros(3)                        # body rates (p, q, r) about (f, r, u), deg/s
                if self.launched:
                    nose_up = self.turn_rate * self.fin_pitch + self.dist_pitch
                    nose_right = self.turn_rate * self.fin_yaw + self.dist_yaw
                    rates = np.array([0.0, -nose_up, nose_right])   # +q about r pitches the nose DOWN
                    self.R = self.R @ exp_so3(np.radians(rates) * dt)
                    if n % 25 == 0:
                        self.R = orthonormalize(self.R)
                    self.pos = self.pos + self.R[:, 0] * self.speed * dt
                    self.batt_v = max(6.5, self.batt_v - 0.0015 * dt)
                # IMU in body (f, r, u): gravity reaction + launch throw + noise
                a_body = self.R.T @ np.array([0.0, G, 0.0])
                if self.launch_spike > 0:
                    a_body = a_body + np.array([3.0 * G, 0.0, 0.0]); self.launch_spike -= dt
                a_body = a_body + np.array([self.rng.gauss(0, 0.05) for _ in range(3)])
                g_body = rates + np.array([self.rng.gauss(0, 0.2) for _ in range(3)])
            a_sensor = np.zeros(3); g_sensor = np.zeros(3)
            for i, (idx, sign) in enumerate(self.axes):
                a_sensor[idx] = sign * a_body[i]; g_sensor[idx] = sign * g_body[i]
            self.last_imu_time = time.time()
            if self.on_imu:
                self.on_imu(tuple(a_sensor), tuple(g_sensor), self.last_imu_time)
            if self.last_imu_time - last_batt > 2.0:
                last_batt = self.last_imu_time
                if self.on_batt: self.on_batt(self.batt_v)
            time.sleep(max(0.0, period - (time.time() - t0)))


class FakeIRCam(threading.Thread):
    """Renders a 640x480 'IR' frame of the beacon as the fake fish's camera sees it."""

    def __init__(self, fish, hfov=62.0, vfov=49.0, w=640, h=480, fps=20.0, ir_range=6.0):
        super().__init__(daemon=True)
        self.fish, self.w, self.h, self.period = fish, w, h, 1.0 / fps
        self.tan_h, self.tan_v = math.tan(math.radians(hfov / 2)), math.tan(math.radians(vfov / 2))
        self.ir_range = ir_range
        self.frame = None
        self.seq = 0
        self.lock = threading.Lock()
        self.fps = fps
        self.connected = True
        self.error = None
        self._stop = threading.Event()
        self.rng = np.random.default_rng()

    def stop(self): self._stop.set()

    def latest(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def run(self):
        while not self._stop.is_set():
            t0 = time.time()
            img = np.full((self.h, self.w), 8, np.uint8)
            n = 60
            xs = self.rng.integers(0, self.w, n); ys = self.rng.integers(0, self.h, n)
            img[ys, xs] = self.rng.integers(20, 50, n)
            with self.fish.lock:
                fwd, right, up = self.fish.basis()
                rel = self.fish.target - self.fish.pos
            d = float(np.linalg.norm(rel))
            f, r, u = float(np.dot(rel, fwd)), float(np.dot(rel, right)), float(np.dot(rel, up))
            if f > 0.05 and d < self.ir_range:
                px = self.w / 2 + (r / f) / self.tan_h * self.w / 2
                py = self.h / 2 - (u / f) / self.tan_v * self.h / 2
                if -40 < px < self.w + 40 and -40 < py < self.h + 40:
                    rad = int(max(4, min(120, 22 / max(0.15, d))))
                    bright = int(170 + 85 * (1 - min(1.0, d / self.ir_range)))
                    cv2.circle(img, (int(px), int(py)), rad, bright, -1)
                    img = cv2.GaussianBlur(img, (0, 0), max(1, rad / 3))
            frame = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            with self.lock:
                self.frame = frame
                self.seq += 1
            time.sleep(max(0.0, self.period - (time.time() - t0)))
