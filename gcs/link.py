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
        self.cam_pan = None            # reported by the board's ACK when it has the gimbal
        self.cam_tilt = None
        self.fin_board_ok = None       # None = single board; True/False = head node's link to a separate fin board
        self._last_send = 0.0
        self._imu_count = 0
        self._imu_t0 = time.time()
        self._stop = threading.Event()
        self.fake = False

    def stop(self):
        self._stop.set()

    def send(self, yaw, pitch, pan=None, tilt=None):
        if pan is None or tilt is None:
            msg = f"{yaw:.1f},{pitch:.1f}\n".encode()
        else:
            msg = f"{yaw:.1f},{pitch:.1f},{pan:.1f},{tilt:.1f}\n".encode()
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
                    if len(parts) >= 6:
                        self.cam_pan, self.cam_tilt = float(parts[3]), float(parts[4])
                    if len(parts) >= 7:
                        self.fin_board_ok = parts[6] == "1"     # two-board layout: the head node reports its UART link to the fin board
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
        self.cam_pan, self.cam_tilt = 90.0, 55.0     # camera gimbal, absolute servo angles (slewed)
        self.cmd_pan, self.cmd_tilt = 90.0, 55.0
        self.cam_limits = ((5.0, 175.0, 90.0), (35.0, 75.0, 55.0))   # (min, max, home) for pan, tilt
        self.last_cmd_time = 0.0
        self.pos = np.zeros(3)
        self.R = R_LEVEL.copy()
        self.speed = 0.0
        self.launched = False
        self.target = np.array([2.0, 8.0, 1.0])
        self.dist_yaw = self.dist_pitch = 0.0
        self.launch_spike = 0.0
        self.swarm_on = False
        self.quads = []                 # [{off: world offset from target (m), vel}] simulated quad swarm
        self.set_swarm(False, 4)
        self.cam = FakeIRCam(self, hfov, vfov)
        self._stop = threading.Event()
        self.lock = threading.RLock()          # reset() calls set_mission() under the lock

    # ---- FishLink surface ----
    def stop(self): self._stop.set()

    def send(self, yaw, pitch, pan=None, tilt=None):
        self.sent += 1
        self.cmd_yaw, self.cmd_pitch = max(-30.0, min(30.0, yaw)), max(-30.0, min(30.0, pitch))
        if pan is not None and tilt is not None:
            (pl, ph, _), (tl, th, _) = self.cam_limits
            self.cmd_pan, self.cmd_tilt = max(pl, min(ph, pan)), max(tl, min(th, tilt))
        self.last_cmd_time = time.time()
        self.acked += 1
        self.last_ack_time = time.time()
        self.rtt_ms = 18.0 + self.rng.random() * 8.0
        self.last_ack = "ACK %.1f %.1f %.1f %.1f %d" % (self.fin_yaw, self.fin_pitch, self.cam_pan, self.cam_tilt, self.acked)

    def home(self): self.send(0.0, 0.0, self.cam_limits[0][2], self.cam_limits[1][2])
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

    def cam_basis(self):
        """Camera axes: the body axes turned by the gimbal (pan about up, tilt about right).
        Pan above 90 looks right, tilt above 55 looks up (the laptop's invert tunables flip this)."""
        f, r, u = self.basis()
        pa = math.radians(self.cam_pan - self.cam_limits[0][2]); ta = math.radians(self.cam_tilt - self.cam_limits[1][2])
        f1 = f * math.cos(pa) + r * math.sin(pa); r1 = r * math.cos(pa) - f * math.sin(pa)
        f2 = f1 * math.cos(ta) + u * math.sin(ta); u2 = u * math.cos(ta) - f1 * math.sin(ta)
        return f2, r1, u2

    # ---- simulated quad swarm (design section 15) ----
    def set_swarm(self, on, n=4, radius=0.4):
        """n quads hovering around the mission target, each with its own IR LED."""
        with getattr(self, "lock", threading.RLock()):
            self.swarm_on = bool(on)
            n = max(1, int(n))
            if len(self.quads) != n:
                self.quads = []
                for i in range(n):
                    ang = 2 * math.pi * i / n
                    r = radius * (0.5 + 0.5 * self.rng.random())
                    self.quads.append({"off": np.array([r * math.cos(ang), 0.25 * (self.rng.random() - 0.5), r * math.sin(ang)]),
                                       "vel": np.zeros(3), "radius": radius})

    def _step_quads(self, dt):
        for q in self.quads:
            q["vel"] += np.array([self.rng.gauss(0, 0.15), self.rng.gauss(0, 0.05), self.rng.gauss(0, 0.15)]) * dt
            q["vel"] *= 0.98
            q["off"] = q["off"] + q["vel"] * dt
            r = float(np.linalg.norm(q["off"]))
            if r > q["radius"]:
                q["off"] *= q["radius"] / r; q["vel"] *= -0.5

    def quad_positions(self):
        """True world positions of the quads (fake mode only, for the 3D demo)."""
        with self.lock:
            return [(self.target + q["off"]).tolist() for q in self.quads] if self.swarm_on else []

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
            self.cam_pan = self.cmd_pan = self.cam_limits[0][2]; self.cam_tilt = self.cmd_tilt = self.cam_limits[1][2]
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
                    self.cmd_pan, self.cmd_tilt = self.cam_limits[0][2], self.cam_limits[1][2]
                self.fin_yaw += max(-3.0, min(3.0, self.cmd_yaw - self.fin_yaw))
                self.fin_pitch += max(-3.0, min(3.0, self.cmd_pitch - self.fin_pitch))
                self.cam_pan += max(-4.0, min(4.0, self.cmd_pan - self.cam_pan))
                self.cam_tilt += max(-4.0, min(4.0, self.cmd_tilt - self.cam_tilt))
                # disturbance random walk (deg/s)
                self.dist_yaw = max(-1.5, min(1.5, self.dist_yaw + self.rng.gauss(0, 0.4) * dt))
                self.dist_pitch = max(-1.0, min(1.0, self.dist_pitch + self.rng.gauss(0, 0.3) * dt))
                rates = np.zeros(3)                        # body rates (p, q, r) about (f, r, u), deg/s
                if self.swarm_on:
                    self._step_quads(dt)
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


class SimCamera:
    """
    Fake-mode camera source fed by the browser: the GCS page renders the fish's point of view
    with Three.js (IR-style or visible light) and pushes JPEG frames over /camsim. Frames older
    than `stale_s` fall back to the FakeIRCam dot picture, so the pipeline keeps running when
    no page is open. Same surface as FakeIRCam / MjpegSource: latest(), seq, fps, connected.
    """

    def __init__(self, fallback, stale_s=1.0):
        self.fallback = fallback
        self.stale_s = stale_s
        self.lock = threading.Lock()
        self.frame = None
        self.t = 0.0
        self._seq = 0
        self.pushed = 0
        self.error = None
        self.connected = True
        self._times = []

    def start(self): self.fallback.start()
    def stop(self): self.fallback.stop()

    def push(self, jpeg_bytes):
        img = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            self.error = "bad frame from the browser"
            return False
        now = time.time()
        with self.lock:
            self.frame = img; self.t = now; self._seq += 1; self.pushed += 1
            self._times.append(now); self._times = [t for t in self._times if now - t < 2.0]
        return True

    @property
    def fresh(self):
        return self.frame is not None and (time.time() - self.t) < self.stale_s

    @property
    def source(self):
        return "browser" if self.fresh else "fake"

    @property
    def seq(self):
        return (1_000_000_000 + self._seq) if self.fresh else self.fallback.seq

    @property
    def fps(self):
        if self.fresh:
            return round(len(self._times) / 2.0, 1)
        return self.fallback.fps

    def latest(self):
        if self.fresh:
            with self.lock:
                return self.frame.copy()
        return self.fallback.latest()


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
                fwd, right, up = self.fish.cam_basis()
                points = [self.fish.target + q["off"] for q in self.fish.quads] if self.fish.swarm_on else [self.fish.target]
                pos = self.fish.pos.copy()
            led = 22.0 if len(points) == 1 else 9.0          # a quad's LED is smaller than the beacon
            drawn = 0
            for pt in points:
                rel = pt - pos
                d = float(np.linalg.norm(rel))
                f, r, u = float(np.dot(rel, fwd)), float(np.dot(rel, right)), float(np.dot(rel, up))
                if f > 0.05 and d < self.ir_range:
                    px = self.w / 2 + (r / f) / self.tan_h * self.w / 2
                    py = self.h / 2 - (u / f) / self.tan_v * self.h / 2
                    if -40 < px < self.w + 40 and -40 < py < self.h + 40:
                        rad = int(max(3, min(120, led / max(0.15, d))))
                        bright = int(170 + 85 * (1 - min(1.0, d / self.ir_range)))
                        cv2.circle(img, (int(px), int(py)), rad, bright, -1)
                        drawn = max(drawn, rad)
            if drawn:
                img = cv2.GaussianBlur(img, (0, 0), max(1, drawn / 3))
            frame = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            with self.lock:
                self.frame = frame
                self.seq += 1
            time.sleep(max(0.0, self.period - (time.time() - t0)))
