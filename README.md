# Maelstrom GCS

Ground control station for Maelstrom, the fin-steered fish: the laptop receives the ICM20948 stream and
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

In the page: Connect → Apply mission (speed in cm/s and target x right, y up, z ahead of the nose at
launch in cm; the station works in m/s and metres) → optionally Calibrate IMU (hold still 3 s) and Fin self-test → Arm, Confirm arm → Launch
(countdown). Only the link and an applied mission are required to arm; skipped checks are listed.
**Stop mission** (or `S`) ends any mission and returns to Ready with the fins neutral, keeping the
link, calibration and parameters, so you can edit the target or speed, Apply again and re-arm.
The fish glides under IMU dead reckoning, hands over to vision at the threshold (default 80 %)
once the beacon has been seen for 5 frames at confidence ≥ 0.6, and homes on the beacon.
Abort puts the fins to neutral and keeps the link. After the flight, scrub the replay or
download the log as JSON or CSV.

## Simulated fish camera (fake mode)

With `--fake`, the page renders what the fish's camera would see (from the true simulated pose,
IR-style by default, or visible light) and streams it to the ground station, which runs the blob
detector and YOLO on it exactly as on the real ESP32-CAM stream. Choose the style in the Vision
panel; close the page and the station falls back to its own dot picture within a second.

## Bench test (no hardware)

```bash
.venv/bin/python tests/flight_sim.py            # full simulated flight, exits 0 when the fish arrives
.venv/bin/python tests/flight_sim.py 0 6 0 1.0  # other target x y z and speed
.venv/bin/python tests/flight_sim.py --abort    # abort mid-glide, then Stop back to Ready
.venv/bin/python tests/flight_sim.py --stop-restart  # arm without checks, stop mid-glide, re-apply, fly again
```

The simulated fish turns 1 deg/s per degree of fin, so with `fin_max` 30 it needs about 3 s to
swing its nose onto a target 80 deg off the launch heading. The IMU law steers in the body frame
(target direction expressed as angles right of and above the nose), which is what keeps a nose-up
flight stable; the vision law does the same with pixel offsets.

## Swarm mode and YOLO (laptop or Raspberry Pi)

In the Vision panel switch **Beacon → Swarm**: every light source (or YOLO box) becomes a source,
sources closer than *cluster radius* px form a swarm, swarms keep IDs across frames, and the fish
homes on the **centroid** of the followed swarm (the biggest, or the one you pick with the list,
the `[` `]` keys, or a click on the frame). In fake mode the target becomes a hovering swarm of
*simulated quads* (slider in Detection tuning) drawn as quadcopters in the 3D view.

YOLO is optional and plugs into the same pipeline as a second source of boxes:

| Where | Install | Run |
|---|---|---|
| laptop | `pip install ultralytics` | `python -m gcs --yolo yolo11n.pt` |
| Raspberry Pi | `pip install onnxruntime opencv-python numpy` | copy `gcs/yolo_backend.py` + `gcs/vision.py`, or run `python -m gcs.vision --stream http://<cam>:81/stream --yolo yolo11n.onnx --mode swarm` |

Export once on the laptop: `yolo export model=yolo11n.pt format=onnx imgsz=320 opset=12`. The same
`.onnx` is what the AI HAT / Hailo compiler takes later. Restrict classes with the `yolo_classes`
tunable (names or ids, e.g. `airplane,kite` on a COCO model). Pretrained COCO weights have no
"drone" class and see little in an IR-filtered frame, so for real quads either fit IR LEDs (works
today) or fine-tune: label a few hundred frames from your own camera, then
`yolo train model=yolo11n.pt data=quads.yaml imgsz=320 epochs=50` and export as above.

Standalone check on any machine: `python -m gcs.vision --image frame.jpg --mode swarm --save out.jpg`.

### Train on synthetic Three.js drones (no real dataset needed)

```bash
.venv/bin/python tools/synth/server.py          # then open http://127.0.0.1:9100
```

The page renders random scenes of procedurally varied quadcopters (five base frames: X, H, plus,
hex, racer; random arms, hubs, rotors, prop guards, legs, colours, LEDs) against sky, ground or
plain backgrounds, half of them IR-style (dark, bright LEDs, sensor noise, bloom) to match the
fish's filtered camera, with unlabelled distractors (birds, balls, boxes, poles, stray lights) and
some empty frames. Labels come from the projected 3D geometry, so every box is exact. Set the count,
press **Generate and save**; files land in `datasets/quads/{images,labels}/{train,val}` with a
`data.yaml`. About 2 000 to 5 000 images is a sensible first run. Then:

```bash
.venv/bin/pip install ultralytics
.venv/bin/python tools/train_quads.py            # ~50 epochs at 320 px -> models/quads.pt + models/quads.onnx
.venv/bin/python tools/train_quads.py --check    # detections on a few val images
.venv/bin/python -m gcs --fake --yolo models/quads.pt
```

**Trained model in the repo.** `models/quads.pt` and `models/quads.onnx` were trained on 26 Sep 2026
from 1 500 synthetic images (1 296 train / 204 val, 50 epochs at 320 px, about 8 min on an M4 Pro
with MPS): mAP50 0.92, mAP50-95 0.58 on the synthetic validation set; the ONNX runs in 4 to 10 ms
per frame on the laptop CPU. Full run in `runs/quads/` (curves, confusion matrix, prediction grids).
Regenerate with a different seed to grow the set; retraining resumes from `yolo11n.pt`, not from
the previous model, so add real frames before the next run rather than after.

Synthetic-only models transfer imperfectly to real footage; when you have real frames, label a
hundred or so and add them to the same folders before retraining (the synthetic set stays in).

## Layout

| Path | What |
|---|---|
| `gcs/server.py` | FastAPI: `/` page, `/ws` telemetry 20 Hz + events, `POST /cmd`, `/video_feed`, `/log.json`, `/log.csv` |
| `gcs/station.py` | GroundStation: connect, mission, calibrate, self-test, arm/launch/abort, control loop, telemetry |
| `gcs/guidance.py` | phase state machine, IMU steering law, vision steering law, hand-over blend |
| `gcs/imu.py` | ICM20948 attitude (complementary filter) + dead reckoning + progress along the path |
| `gcs/vision.py` | MJPEG reader, IR blob sources, swarm clustering + tracking, optional YOLO, annotated stream, standalone CLI |
| `gcs/yolo_backend.py` | YOLO boxes via ultralytics (.pt, laptop) or onnxruntime (.onnx, Raspberry Pi); standalone |
| `gcs/link.py` | UDP link to the fish node; `FakeFish` + `FakeIRCam` (with a simulated quad swarm) for the bench |
| `tests/flight_sim.py` | bench flight through the real command path, prints true vs estimated position |
| `tools/synth/` | Three.js synthetic drone image generator + save server (YOLO dataset) |
| `tools/train_quads.py` | fine-tune YOLO nano on that dataset, export ONNX for the Pi |
| `gcs/static/index.html` | the ground station page (Three.js flight view, vision, fins, log; WCAG 2.2 AA and the Apple HIG redesign in `docs/FRONTEND_DESIGN.md`, audit in `docs/HIG_AUDIT.md`) |
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
