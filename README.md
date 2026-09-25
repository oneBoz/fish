# Fish GCS

Ground control station for the fin-steered fish: the laptop receives the ICM20948 stream and
the IR camera over the phone hotspot, steers the fins over UDP, and shows everything in a browser.

```
fish (ESP32-S3 fish_node.ino + ICM20948 + 4 fins, ESP32-CAM IR)
   │ UDP 4210: fin commands ⇄ ACK + IMU lines     MJPEG :81/stream
   ▼
laptop  python -m gcs   →  http://127.0.0.1:9000  (the GCS page)
```

## Run

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python -m gcs --fake --open              # no hardware: simulated fish, IMU and IR camera
.venv/bin/python -m gcs --fin-ip 172.20.10.12 --cam-ip 172.20.10.13 --open   # real fish
```

Options: `--yolo weights.pt` (needs `pip install ultralytics`; IR blob detection is the fallback),
`--imu-axes x,y,z` (which ICM20948 sensor axes point forward, right, up; the triple must stay right-handed, so flip signs in pairs, e.g. `x,-y,-z`), `--guidance laptop|fish`
(who steers during IMU glide; default laptop), `--port 9000`.

In the page: Connect → Apply mission (speed and target x right, y up, z ahead of the nose at
launch) → Calibrate IMU (hold still 3 s) → Fin self-test → Arm, Confirm arm → Launch (countdown).
The fish glides under IMU dead reckoning, hands over to vision at the threshold (default 80 %)
once the beacon has been seen for 5 frames at confidence ≥ 0.6, and homes on the beacon.
Abort puts the fins to neutral and keeps the link. After the flight, scrub the replay or
download the log as JSON or CSV.

## Bench test (no hardware)

```bash
.venv/bin/python tests/flight_sim.py            # full simulated flight, exits 0 when the fish arrives
.venv/bin/python tests/flight_sim.py 0 6 0 1.0  # other target x y z and speed
.venv/bin/python tests/flight_sim.py --abort    # abort mid-glide, then New mission
```

The simulated fish turns 1 deg/s per degree of fin, so with `fin_max` 30 it needs about 3 s to
swing its nose onto a target 80 deg off the launch heading. The IMU law steers in the body frame
(target direction expressed as angles right of and above the nose), which is what keeps a nose-up
flight stable; the vision law does the same with pixel offsets.

## Layout

| Path | What |
|---|---|
| `gcs/server.py` | FastAPI: `/` page, `/ws` telemetry 20 Hz + events, `POST /cmd`, `/video_feed`, `/log.json`, `/log.csv` |
| `gcs/station.py` | GroundStation: connect, mission, calibrate, self-test, arm/launch/abort, control loop, telemetry |
| `gcs/guidance.py` | phase state machine, IMU steering law, vision steering law, hand-over blend |
| `gcs/imu.py` | ICM20948 attitude (complementary filter) + dead reckoning + progress along the path |
| `gcs/vision.py` | MJPEG reader, IR blob detector (same method as `light_tracker`), optional YOLO nano, annotated stream |
| `gcs/link.py` | UDP link to the fish node; `FakeFish` + `FakeIRCam` for the bench |
| `tests/flight_sim.py` | bench flight through the real command path, prints true vs estimated position |
| `gcs/static/index.html` | the GCS page (Three.js 3D view, vision, fins, log; WCAG 2.2 AA choices in `docs/FRONTEND_DESIGN.md`) |
| `firmware/fish_node/fish_node.ino` | `pid_fins.ino` + ICM20948 streaming + PING/MISSION/LAUNCH. Untested on hardware. |
| `docs/FRONTEND_DESIGN.md` | the agreed design, data contract and decisions |

## Wire protocol (UDP 4210)

```
laptop → fish   "yaw,pitch\n"   "HOME\n"   "PING\n"   "MISSION,speed,x,y,z\n"   "LAUNCH\n"
fish → laptop   "ACK yaw pitch n\n"   "IMU,ms,ax,ay,az,gx,gy,gz\n" (m/s², deg/s, ~50 Hz)   "BATT,v\n"
```

## Flash the fish node

Arduino IDE → board ESP32-S3, libraries **ESP32Servo** and **SparkFun 9DoF IMU Breakout - ICM 20948**.
Set Wi-Fi credentials and I2C pins in `fish_node.ino`. The old `pid_fins.ino` still works with the
GCS for fins only (no IMU lines, so Calibrate reports "no IMU samples").
