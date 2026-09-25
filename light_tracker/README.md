# Light-source tracker: pan/tilt camera **or** fish fins

```
ESP32-CAM  ──MJPEG over HTTP (:81/stream)──▶  Laptop  ──UDP "pan,tilt" (:4210)──▶  ESP32 + 2 servos   (--output pantilt)
                                               │                   or
                                               │      ──UDP "yaw,pitch" (:4210)──▶  ESP32-S3 + 4 fins  (--output fins)
                                               │ OpenCV: detect every light, group into swarms
                                               │ follow one light or one selected swarm
                                               │ P-I controller: px error → degrees
                                               └ Flask dashboard  http://localhost:9000
```
Two ways to use the same tracker:

- **pan/tilt** — the camera sits on a 2-servo bracket and turns to keep the light centred (`esp32_servo/esp32_servo.ino`).
- **fins** — the camera sits **on the fish**, facing forward, and the four fins steer the whole fish towards the light (`../pid_fins/pid_fins.ino`). No IMU is involved: the camera seeing the light drift back to centre *is* the feedback.

# NOTEE
2 Servos, 1 pan 1 tilt.
For the pan servo, 


## Files

| Path | Runs on | What it does |
|---|---|---|
| `esp32_cam/esp32_cam.ino` | AI-Thinker ESP32-CAM | Streams MJPEG at `http://<cam-ip>:81/stream` |
| `esp32_servo/esp32_servo.ino` | Any ESP32 dev board | Listens on UDP 4210 for `pan,tilt`, drives servos with slew limiting, ACKs back |
| `../pid_fins/pid_fins.ino` | ESP32-S3 on the fish | Listens on UDP 4210 for `yaw,pitch` fin deflections, drives 4 fins (2 pairs) with slew limiting, ACKs back, fins to neutral if the link drops |
| `laptop/tracker.py` | Laptop | CV + control loop + dashboard server |
| `laptop/templates/dashboard.html` | Laptop (browser) | Live annotated stream, crosshair, swarm picker, every calculation step, tuning sliders |

## 1. Flash the ESP32-CAM

- Arduino IDE → Board **AI Thinker ESP32-CAM**, Partition **Huge APP (3MB No OTA)**.
- Edit `WIFI_SSID` / `WIFI_PASS` in `esp32_cam.ino`. No libraries to install.
- Flash (GPIO0 → GND during upload, then release and reset). Serial monitor @115200 prints the stream URL.
- Test in a browser: `http://<cam-ip>:81/stream`.

Tip: `set_ae_level(s, -2)` biases exposure dark so a lamp/torch saturates while the room doesn't. If the whole frame blows out, set `set_exposure_ctrl(s, 0)` and `s->set_aec_value(s, 100..300)` for fixed exposure.

## 2. Flash the servo ESP32

- Library Manager → install **ESP32Servo**.
- Wiring: pan signal → GPIO13, tilt signal → GPIO12, servo GND ↔ ESP32 GND. **Power the servos from a separate 5 V supply** (≥1 A), not the ESP32's 3V3/5V pin — the brownouts will reset the board.
- Edit Wi-Fi credentials and the `attach(pin, 500, 2500)` pulse range for your servos (SG90 ≈ 500–2400).
- Bracket limits are pan 5–175° (home 90°) and tilt 5–105° (home 55°). They are defined in **both** `esp32_servo.ino` and `laptop/tracker.py` (`PAN_RANGE`, `TILT_RANGE`, `*_HOME`) and must be kept identical.
- Serial monitor prints the node's IP.

## 2b. Flash the fin board instead (fish mode)

- Same **ESP32Servo** library. Board: your ESP32-S3, upload via the bottom (UART) USB-C port, serial monitor @115200.
- Servos on GPIO 4, 5, 6, 7, servo rail from the UBEC, grounds common. **Servos 0 & 2 are the pitch pair (horizontal fins), 1 & 3 the yaw pair (vertical fins).** Flip `servoDirection[i]` to -1 (or type `f<i>` in the serial monitor) for any fin that moves the wrong way; `t<i> <deg>` sets trim; `m<deg>` sets the deflection limit.
- Edit Wi-Fi credentials. The static IP defaults to `172.20.10.12` (cam is `.13`, pan/tilt node `.14`). Do **not** use `.15`: on a phone hotspot's /28 subnet that is the broadcast address, and the laptop will refuse to send to it (`Permission denied`).
- Packet format is `"yaw,pitch\n"` in degrees from neutral, clipped to ±`FIN_MAX` (30°). `HOME\n` centres the fins. If no packet arrives for 1 s the fins **return to neutral** so a dropped link never leaves the fish stuck in a turn.
- The MPU6050 is not used by this sketch.

## 3. Run the laptop tracker

```bash
cd laptop
pip install -r requirements.txt
python tracker.py --cam 192.168.1.50 --servo 192.168.1.51                  # pan/tilt bracket
python tracker.py --cam 172.20.10.13 --servo 172.20.10.12 --output fins    # fish fins (pid_fins.ino)
```

Open **http://localhost:9000**.

Options:
- `--output pantilt|fins` — what the tracker drives. `pantilt` (default) sends bracket angles to `esp32_servo`; `fins` sends yaw/pitch fin deflections to `pid_fins`. Without `--servo`, fins mode uses a simulated fish (`FakeFinLink`) that turns in proportion to the fin deflection, so `--cam fake --output fins` closes the loop on the bench.
- `--servo` is optional. Leave it out to run camera + dashboard with no servo board: a simulated servo link (`FakeServoLink`) accepts the commands, slews like the real node and always reports alive, but nothing physical moves.
- **Detection cutoff** (`--thresh-mode relative|absolute`, default relative; all live on the dashboard):
  - `relative` — made for an IR-filtered camera, where the scene is dark and only the IR sources show, but *how* bright they are depends on distance and exposure. Each frame: `background = median(blurred)`, `contrast = peak − background`, `cutoff = background + rel_threshold · contrast`. A pixel counts as light when it is `--rel-threshold` (0.5) of the way from the background to the brightest pixel, so the cutoff follows exposure automatically. If `contrast < --min-contrast` (40) the frame is treated as empty, so an all-dark frame never turns noise into a target. Lower `rel_threshold` to keep dimmer sources when a bright one is also in view (swarm mode); raise `min_contrast` if noise gets picked up.
  - `absolute` — fixed cutoff `--threshold` (150) in 0–255, the old behaviour.
  - `min area` and `blur` sliders apply to both modes. The dashboard shows background, contrast and the cutoff actually used every frame.
- `--invert-pan` / `--invert-tilt` — if the camera (or the fish) turns *away* from the light, flip the sign. In fins mode these flip yaw / pitch.
- `--hfov 62 --vfov 49` — OV2640 default lens; adjust for wide-angle lenses. Pan/tilt only: affects how aggressive one step is.

### No hardware? Fake camera with multiple swarms

```bash
python tracker.py --cam fake --fake-swarms 3
```

`--cam fake` replaces the ESP32-CAM with a synthetic scene (`FakeCam`): several swarms of lights that drift around slowly and twinkle. If `--servo` is omitted a simulated servo node (`FakeServoLink`) is used too; it slews at the real node's rate and points the fake camera, so the loop is closed and a tracked swarm really slides to the centre of the picture. Pass `--servo <ip>` as well to drive the real servos from the fake picture. Everything else (detection, clustering, swarm IDs, the dashboard, `/swarm` selection) runs unchanged; the dashboard telemetry has `frame.fake = true`.

| Flag | Default | Meaning |
|---|---|---|
| `--fake-swarms N` | 3 | number of swarms in the scene |
| `--fake-lights N` | 4 | lights per swarm |
| `--fake-spread DEG` | 3 | swarm radius in degrees (≈10 px/deg at 640×62°); keep it under `cluster_radius` |
| `--fake-speed DEG_S` | 3 | how fast swarms drift |
| `--fake-area PANxTILT` | 50x30 | region (degrees, around home) the swarms roam in. The default fits in one view so every swarm is visible from home; use e.g. `160x90` to make the camera hunt for swarms that leave the picture |
| `--fake-size WxH`, `--fake-fps` | 640x480, 20 | frame size and rate |
| `--fake-seed N` | random | repeatable scene |

Switch the dashboard to **swarm** mode, then use the swarm list / **N** / **P** / click on the stream to pick which swarm the camera follows.

## How the tracking works

1. **Detect** — grayscale → Gaussian blur (kills hot pixels) → threshold → contours. *Every* blob above threshold and `min_area` becomes a candidate light source with its intensity-weighted centroid, area and peak brightness (all shown on the dashboard and drawn as grey circles on the stream). Falls back to the raw peak pixel for tiny sources.
1. **Reduce to one target** (dashboard button **mode**):
   - **single** — if a candidate lies within `lock_radius` px of last frame's target, take the nearest one (*sticky*: a second light entering the frame does not steal the lock). Otherwise take the brightest candidate. Set `lock_radius` to 0 to always chase the brightest.
   - **swarm** — sources are first *grouped into swarms*: two sources belong to the same swarm if they are within `cluster_radius` px of each other, directly or through a chain of other sources (single-linkage clustering, `cluster_sources`). Set `cluster_radius` to 0 to put every source in one swarm. Each swarm's target point is the *mass-weighted centroid of its sources* (mass = sum of pixel intensities in the blob, so bigger/brighter lights pull harder), with a `spread` (mass-weighted RMS radius) and a convex hull. Up to `max_sources` (32) blobs are used.

     Swarms keep a **persistent ID** across frames (`SwarmTracker`): each swarm is matched to the closest one from the previous frame within `swarm_match` px, otherwise it gets a new ID. A swarm that vanishes is remembered for ~10 frames so a flicker does not renumber everything. **You choose which swarm to follow**: the dashboard lists every visible swarm (`S1`, `S2`, …), and you can click a row, click the swarm on the stream, use the **prev / next swarm** buttons, or press **N / P**. If nothing is selected, or the selected swarm has been gone for more than ~10 frames, the biggest swarm (highest total mass) is followed. While the selected swarm is only briefly missing the servos **hold** position instead of jumping to another swarm.

     On the stream the followed swarm is drawn bright magenta (hull, spokes to each source, crosshair, circle of radius `spread`); the other swarms are drawn dim purple with their `S<id>` label so you can see what you can switch to.
2. **Error** — `ex = x − W/2`, `ey = y − H/2`, normalised to −1..+1.
3. **Control**
   - **pan/tilt** — `Δpan = (Kp·ex_norm + Ki·∫ex)·HFOV/2`, same for tilt with the vertical FOV, *added* to the current bracket angle. Deadband stops hunting once the spot is centred; `max_step` caps degrees per frame.
   - **fins** — `yaw = (Kp·ex_norm + Ki·∫ex)·fin_max`, `pitch = -(Kp·ey_norm + Ki·∫ey)·fin_max`. This is a *deflection*, not an accumulated angle: light on the right → push the yaw fins right; as the fish turns and the light drifts back to centre the deflection relaxes to zero. Inside the deadband, or with no light in view, the fins go to neutral so the fish glides straight. `max_step` limits how fast the deflection may change per frame; **fin max deflection** (dashboard slider) is the clip.
4. **Send** — `"a,b\n"` over UDP at 20 Hz (`pan,tilt` or `yaw,pitch`); the node slews at ≤3°/20 ms and replies `ACK a b count` so the dashboard shows link RTT.

### Swarm mode quick reference

| Control | Where | Effect |
|---|---|---|
| **mode** button | Tuning panel | toggles `single` ↔ `swarm` (also resets swarm IDs) |
| **cluster radius** slider | Tuning panel | max gap (px) between two lights that still count as one swarm; 0 = everything is one swarm |
| **swarm match** slider | Tuning panel | how far (px) a swarm may move between frames and keep its ID; raise it for fast-moving swarms |
| **prev / next swarm**, **N / P** keys | Swarms panel | cycle the followed swarm through the visible IDs |
| click a swarm row / click on the stream | Swarms panel / stream | follow that swarm |
| `POST /swarm` `{"action":"next"}` / `{"id":3}` / `{"x":120,"y":80}` | HTTP | same thing from a script |

Notes: if two swarms drift closer than `cluster_radius` they merge into one swarm (and the merged swarm keeps whichever ID was nearest to its centroid); when they separate again one half gets a new ID. Keep `cluster_radius` a bit smaller than the gap between the groups you want to tell apart, and a bit larger than the gap between lights inside a group.

## Tuning order (fins)

1. Fish on the bench, tracking **OFF**, camera pointed at a lamp. Move the lamp right: the dashboard's *yaw fin* should go positive. Move it down: *pitch fin* negative. If a fin moves the wrong way physically, flip it with `f<i>` on the fin board's serial monitor; if the *sign of the command* is wrong, use `--invert-pan` / `--invert-tilt`.
2. Start with Kp≈0.5, Ki=0, fin max 20°. In water, raise Kp until the fish overshoots and wags, then back off. Add a little Ki only if it consistently stops short of the light. Raise **deadband** if the fins twitch when the light is already centred.
3. Water is slow: if the fish reacts late and oscillates, lower **max step** so the fins ramp instead of slamming.
4. No water handy? `python tracker.py --cam fake --output fins` simulates a fish that turns at 1°/s per degree of fin.

## Tuning order (pan/tilt)

1. Cover the camera so only the light is bright → confirm `LOCKED` and the crosshair sits on the source.
2. Turn tracking OFF, move the light, check the arrow points the right way. If pan runs away → `--invert-pan`.
3. Tracking ON. Start Kp≈0.4, Ki=0. Raise Kp until it oscillates, back off 30 %. Add a little Ki if it stops short of centre. Widen the deadband if servos twitch.
4. For swarms: switch mode to **swarm**, put two groups of lights in view, and adjust **cluster radius** until the stream shows the number of hulls you expect. Then cycle with **next swarm** and confirm the crosshair jumps between groups.

## Troubleshooting

- **Cam offline** — check `http://<cam-ip>:81/stream` in a browser; the ESP32-CAM only serves one stream at a time, so close the browser tab before running the tracker.
- **Servo no ACK** — both devices on the same Wi-Fi/subnet; laptop firewall may block inbound UDP replies (allow Python). In fins mode make sure `--servo` points at the fin board (`.12`), not the pan/tilt node.
- **Fins snap to neutral every second** — the fin board is not receiving packets (1 s link timeout). Check the IP and that the tracker prints `Output: fins -> <ip>:4210`.
- **Locks onto a window/reflection** — raise threshold, or lower `ae_level` on the camera.
- **Jittery** — increase blur kernel and deadband, lower Kp.
- **Swarm IDs keep changing** — the swarm moves more than `swarm_match` px per frame; raise the slider. Or the group is splitting/merging at the `cluster_radius` boundary; adjust it.
- **Two groups show as one swarm** — lower `cluster_radius` below the gap between the groups.
- **One group shows as several swarms** — raise `cluster_radius` above the gap between the lights inside it, or raise the blur kernel so neighbouring lights merge into one blob.
