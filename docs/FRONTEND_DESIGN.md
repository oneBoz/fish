# Fish Ground Control Station — Frontend Design

Status: **design agreed, 25 Sep 2026**. Nothing below is implemented yet. The clickable prototype
(https://claude.ai/artifact/9WDoVxCu2Kdy8eXxyeiPQe) shows every screen with simulated data. The
decisions in section 10 were made with the operator and implementation starts from this spec.

## 1. Purpose and scope

The GCS is the operator's single screen for one flight of the fish: connect, load the initial
readings, arm, launch, watch the fish climb under IMU guidance, watch the hand-over to computer
vision for the last ~20 %, and abort at any moment. It runs in a browser on the laptop and is
served by the same Python ground station process that does the computation (telemetry, YOLO nano
inference on the IR stream, guidance, and the UDP link to the fin board).

Out of scope for this document: the firmware on the ESP32-S3, the ICM20948 driver, the YOLO
training, and the guidance maths. The frontend only *shows* those and sends commands to them.

## 2. System context

```
   FISH                                    HOTSPOT (phone, 172.20.10.0/28)              LAPTOP
   ┌──────────────────────────┐            ┌───────────────┐            ┌──────────────────────────────────────┐
   │ ESP32-S3  + ICM20948     │ UDP 4210   │               │            │ ground station (Python)              │
   │  4 fin servos (pid_fins) │◀──────────▶│   Wi-Fi AP    │◀──────────▶│  • telemetry in (IMU, fins, link)    │
   │  microSD: init readings  │ telemetry  │               │            │  • MJPEG in → YOLO nano + IR blob    │
   │ ESP32-CAM (IR)  :81      │ MJPEG      │               │            │  • guidance: IMU → CV hand-over      │
   └──────────────────────────┘            └───────────────┘            │  • WebSocket + HTTP to the browser   │
                                                                        └──────────────┬───────────────────────┘
                                                                                       │ ws://localhost:9000
                                                                        ┌──────────────▼───────────────────────┐
                                                                        │ GCS frontend (this document)         │
                                                                        └──────────────────────────────────────┘
```

The laptop replaces the earlier Raspberry Pi AI HAT + Hailo plan. The existing `light_tracker`
dashboard (Flask, vanilla JS) already does the vision half of this; the GCS supersedes it.

## 3. Mission phases

The frontend mirrors the guidance state machine one-to-one. Every phase has a name, a colour that is
**never used alone** (always with an icon and the phase name), and the controls that are enabled in it.

| # | Phase          | Who steers                     | Enters when                                              | Operator controls enabled                  |
|---|----------------|--------------------------------|----------------------------------------------------------|--------------------------------------------|
| 0 | Disconnected   | nobody, fins at neutral        | app opens / link lost                                    | Connect                                    |
| 1 | Ready          | nobody                         | fin board ACKs and camera streams                        | Load init file, Calibrate, Arm             |
| 2 | Armed          | nobody                         | Arm confirmed (two-step)                                 | Launch, Disarm, Abort                      |
| 3 | IMU glide      | ESP32 from microSD init + IMU  | Launch pressed / launch detected by accelerometer        | Abort, Fins neutral, Force hand-over       |
| 4 | Hand-over      | both (blend window, ~1 s)      | progress ≥ threshold (default 80 %, progress = dead-reckoned distance ÷ path length) AND beacon seen ≥ N frames AND confidence ≥ c | Abort, Return to IMU |
| 5 | Vision homing  | laptop CV (YOLO nano + IR)     | hand-over complete                                       | Abort, Return to IMU, Fins neutral         |
| 6 | Arrived        | nobody, fins neutral           | CV error inside deadband for M frames / range reached    | Save log, Stop (back to Ready)             |
| 7 | Aborted / Safe | nobody, fins neutral           | link timeout or guidance fault (automatic only, see section 14) | Save log, Stop (back to Ready), Connect |

Abort is reachable from every phase after Ready, is a single action (no confirmation), and is
visually the largest control on the screen.

## 4. Screen layout

Desktop (≥ 1100 px): a fixed status bar and a three-column deck. Narrow (< 800 px): the same regions
stack in the order Status → Guidance → 3D view → Vision → Fins → Mission setup → Log, with a
"Jump to" navigation and skip link at the top.

```
┌ Status bar ─────────────────────────────────────────────────────────────────────────────────┐
│ Fish GCS   [Fin board ● ok 21 ms] [Camera ● 19 fps] [IMU ● ok]  ▶ IMU glide 63 %   T+00:12  [ ABORT ] │
├ Mission & guidance ───────┬ 3D flight view ────────────────────────┬ Vision ─────────────────┤
│ Connect / target + speed  │  Three.js scene: fish, planned path,   │ IR frame + YOLO box     │
│ Calibrate / Arm / Launch  │  hand-over ring, beacon, trail         │ lock, confidence, error │
│ Phase stepper 0-7         │  View: follow | side | top   Pause     │ hand-over checklist     │
│ Hand-over threshold ──○── │  Text readout of position (live)       ├ Fins ───────────────────┤
│ IMU: roll pitch yaw, est. │  Distance-to-target & vision-error     │ 4 fin gauges, yaw/pitch │
│ position, drift warning   │  sparklines                            │ link RTT, neutral, override │
├ Event log (live region) ────────────────────────────────────────────────────────────────────┤
│ 00:12.4  Hand-over conditions: progress 80 % ✔  beacon 7 frames ✔  confidence 0.91 ✔        │
└──────────────────────────────────────────────────────────────────────────────────────────────┘
```

## 5. Functional specification

Each item has an ID so review comments can point at it.

### F1 Status bar
- F1.1 Three link pills: fin board (ACK round-trip), camera (fps), IMU (last packet age). Each pill
  shows an icon, a word (ok / slow / lost) and the number. Colour is supplementary.
- F1.2 Current phase with progress percentage and an icon.
- F1.3 Mission clock (T− before launch, T+ after).
- F1.4 Abort button: 56 px tall, red, labelled "ABORT", always visible, keyboard shortcut, single press.
- F1.5 Buttons for Settings and Keyboard shortcuts.
- F1.6 Visual cue strip under the status bar: shows the launch countdown (large digits, one update per
  second, Cancel launch button) and a phase-change banner for a few seconds. No sound, no flashing.

### F2 Mission setup
- F2.1 Connect: fields for fin board IP and camera IP (defaults 172.20.10.12 / .13), Connect button,
  per-device result with a plain-language error ("No ACK from 172.20.10.12 in 2 s. Check the board is
  on the hotspot.").
- F2.2 Mission target and speed: numeric fields for launch speed (m/s) and target x, y, z in metres
  from the launch point (y up). "Apply mission" validates them (speed > 0, target ≥ 0.5 m away, not
  while flying), shows distance, expected flight time and hand-over point, and sends them to the fin
  board before arming.
- F2.2a Optional microSD init file (CSV or JSON) only pre-fills those fields; the operator still
  presses Apply. Refuses a file with no data rows or missing columns and says which.
- F2.3 Calibrate IMU: 3 s "hold still" progress with a countdown in text, then bias values shown.
- F2.4 Arm: two-step. First press turns the button into "Confirm arm (5 s)"; a second press arms.
  Cancels itself after 5 s. Disarm is one press.
- F2.5 Launch: enabled only when Armed. Starts a visible countdown (3, 5 or 10 s from Settings) in
  the cue strip; Cancel launch or Abort stops it. Optional "auto-detect launch from accelerometer" toggle.
- F2.6 Pre-flight checklist auto-filled, in two groups (revised in section 12): required to arm (link, mission applied) and
  recommended (calibrated, fins responded to self-test, battery ok).

### F3 Guidance
- F3.1 Phase stepper listing all phases; current one marked with `aria-current="step"`.
- F3.2 Hand-over threshold slider 60–95 %, default 80 %, with a numeric input twin.
- F3.3 Hand-over checklist, live: progress ≥ threshold, beacon seen ≥ N consecutive frames,
  confidence ≥ c, link ok. Each row shows ✔/✘ plus the current value.
- F3.4 Force hand-over / Return to IMU (advanced, behind a disclosure).
- F3.6 Stop mission button (section 12): from any phase after Ready back to Ready, fins neutral, shortcut `S`.
- F3.5 IMU block: roll, pitch, yaw; estimated position (x, y, z) and distance to target;
  dead-reckoning drift warning when the estimate's age or accumulated error exceeds a limit.

### F4 3D flight view
- F4.1 Scene: ground grid, launch point, planned path from launch to target, hand-over ring at the
  threshold, beacon marker at the target, the fish (body + 4 fins that deflect with the commands),
  and its trail. Estimated position (IMU) and vision-corrected position drawn differently
  once both exist.
- F4.2 Camera presets: Follow, Side, Top; orbit with mouse drag and with arrow keys when the view is focused.
- F4.3 Pause view updates (WCAG 2.2.2). Telemetry continues in text while paused.
- F4.4 Text equivalent: a live readout under the view ("3.4 m up, 1.2 m from target, nose 6° left of
  target, IMU glide 63 %") updated at most every 2 s so screen readers are not flooded.
- F4.5 Playback scrubber after the flight (replay from the in-memory history; loading a saved log is later).

### F5 Vision
- F5.1 Live IR frame with YOLO nano boxes, IR blob centroid, crosshair, deadband circle and error vector.
- F5.2 Lock state (searching / seen / locked / lost) as text with icon; confidence as a bar with the number.
- F5.3 Pixel error (ex, ey), normalised error, and the fin deflection it produces.
- F5.4 Detection tuning behind a disclosure: threshold mode, relative threshold, min contrast,
  min area, blur (same parameters as the existing tracker).
- F5.5 Text equivalent updated with the lock state and error.

### F6 Fins
- F6.1 Four fin gauges (0 & 2 pitch pair, 1 & 3 yaw pair) showing commanded and slewed deflection.
- F6.2 Yaw / pitch commanded vs actual, link RTT, packets sent / acked, last ACK age.
- F6.3 Fins to neutral (one press). Fin self-test (wiggle).
- F6.4 Manual override: toggle exposes yaw and pitch sliders; while on, guidance is paused and the
  status bar says so.
- F6.5 PID tuning behind a disclosure: Kp, Ki, deadband, max step, fin max.

### F7 Event log and status messages
- F7.1 Timestamped log of state changes, warnings and operator actions. Newest at the bottom,
  auto-scroll can be switched off.
- F7.2 Live regions: `polite` for routine events, `assertive` for abort, link lost, fault.
- F7.3 Save log (JSON) and export a CSV of telemetry.

### F8 Charts
- F8.1 Distance to target vs time and vision error vs time, each its own chart (no dual axis),
  2 px line, endpoint dot, hover / focus tooltip, and a table view.

### F9 Settings (persist per browser)
- Theme: system / light / dark. High contrast. Text size 100 / 125 / 150 %. Reduce motion.
  Single-key shortcuts on / off. Launch countdown 3 / 5 / 10 s. Units m / ft. Live-region verbosity.

## 6. Data contract (proposed)

Telemetry from the ground station over WebSocket at 20 Hz (one JSON object per message):

```json
{
  "t": 12.42,
  "phase": "imu_glide",           
  "progress": 0.63,
  "link":  {"fin_rtt_ms": 21, "sent": 248, "acked": 247, "imu_age_ms": 48, "rssi": -61},
  "cam":   {"connected": true, "fps": 19.2, "cv_ms": 31},
  "imu":   {"roll": 2.1, "pitch": 14.8, "yaw": -6.3, "ax": 0.1, "ay": 0.0, "az": 9.7},
  "est":   {"x": 0.9, "y": 5.1, "z": 0.4, "dist": 3.2, "drift_warn": false},
  "cv":    {"state": "seen", "conf": 0.91, "frames": 7, "ex_px": 12, "ey_px": -4, "boxes": [[300,180,340,220]]},
  "fins":  {"yaw_cmd": 6.0, "pitch_cmd": -2.5, "yaw": 5.5, "pitch": -2.0, "servo": [88, 96, 92, 84]},
  "handover": {"threshold": 0.8, "beacon_frames_needed": 5, "conf_needed": 0.6, "ok": [true, true, true, true]},
  "batt_v": 7.6
}
```

IMU samples reach the ground station from the ESP32-S3 over the existing UDP 4210 socket, one
line per sample at about 50 Hz, proposed as `IMU,<ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<mx>,<my>,<mz>\n`,
interleaved with the existing `ACK` lines. The ground station integrates them into `est`.

Commands from the frontend, HTTP POST `/cmd` with `{"cmd": "...", ...}`:
`connect`, `mission` `{speed, target:[x,y,z]}`, `calibrate`, `arm`, `disarm`, `launch`, `abort`,
`fins_neutral`, `selftest`, `override` `{on, yaw, pitch}`, `handover_threshold` `{value}`,
`force_handover`, `return_to_imu`, `tune` `{key, value}`, `save_log`.

Every command returns `{ok: true}` or `{ok: false, error: "plain sentence"}` and the frontend
shows the sentence next to the control that sent it.

## 7. WCAG 2.2 AA mapping

| Criterion | How the GCS meets it |
|---|---|
| 1.1.1 Non-text content | 3D view and IR frame have text equivalents (F4.4, F5.5); charts have table views; icons have labels. |
| 1.3.1 Info and relationships | Landmarks (`header`, `nav`, `main`, `section` with headings, `aside` log); tables for data; labelled form controls. |
| 1.4.1 Use of colour | Every phase / status carries an icon and a word; charts have one series each or a legend. |
| 1.4.3 / 1.4.11 Contrast | Text ≥ 4.5:1, UI borders and focus rings ≥ 3:1 in both themes; high-contrast setting raises borders further. |
| 1.4.4 / 1.4.10 / 1.4.12 | Text size setting to 150 %; layout reflows to one column at 400 px; spacing uses rem. |
| 1.4.13 Hover content | Tooltips also appear on focus, dismiss with Esc, and are hoverable. |
| 2.1.1 Keyboard | Every control is a real button / input; orbiting the 3D view works with arrow keys. |
| 2.1.4 Character key shortcuts | Single-key shortcuts can be switched off in Settings; all also exist as buttons. |
| 2.2.2 Pause, stop, hide | Pause button for the 3D view and charts; log auto-scroll can be turned off. |
| 2.3.1 Flashes | Nothing flashes; alerts use a steady colour change plus text and an optional sound. |
| 2.4.1 / 2.4.3 / 2.4.7 | Skip link, jump navigation, logical order, 3 px focus ring on every control. |
| 2.5.5 / 2.5.8 Target size | All targets ≥ 44 × 44 px; Abort is 56 px tall. |
| 3.2.2 On input | Sliders apply on release; nothing navigates or launches on a change event. |
| 3.3.1 / 3.3.3 Errors | Errors in words next to the control, with what to do. |
| 3.3.4 Error prevention | Arm is two-step; Abort is not (safety beats prevention here and is documented in the help). |
| 4.1.3 Status messages | Event log uses live regions with the right politeness. |

## 8. Keyboard map (single-key set can be switched off)

| Key | Action | Notes |
|---|---|---|
| `X` | Abort | works from any focus except text inputs |
| `S` | Stop mission, back to Ready | single press; keeps link, calibration and parameters |
| `A` | Arm / confirm arm | |
| `L` | Launch | only when Armed |
| `N` | Fins to neutral | |
| `P` | Pause / resume 3D view | |
| `1` `2` `3` | 3D view: Follow / Side / Top | |
| `←` `→` `↑` `↓` | Orbit the 3D view when it has focus | |
| `?` | Keyboard shortcuts dialog | |
| `Esc` | Close dialog | never aborts |

## 9. Visual system

- Type: Atkinson Hyperlegible (UI), JetBrains Mono (telemetry numbers, `tabular-nums`).
- Light tokens: ground `#F2F5F8`, surface `#FFFFFF`, ink `#12202C`, muted `#55667A`,
  line `#CBD5E0`, accent `#0B6E8E`.
- Dark tokens: ground `#0D141B`, surface `#172230`, ink `#E9EFF4`, muted `#9DACBB`,
  line `#2A3846`, accent `#5EC6E4`.
- Phase hues (validated for colour-vision deficiency): IMU glide blue `#2a78d6` / `#3987e5`,
  vision homing orange `#eb6834` / `#d95926`.
- Status (fixed): good `#0ca30c`, warning `#fab219`, critical `#d03b3b`; never without icon + label.
- Motion: only functional motion (fish pose, gauges). Camera easing and trail glow are off under
  `prefers-reduced-motion` or the Reduce motion setting.

## 10. Decisions (agreed 25 Sep 2026)

| # | Question | Decision |
|---|----------|----------|
| 1 | Initialised location | A 3D point in metres relative to the launch spot, plus initial attitude. |
| 2 | Flight geometry | Launch speed and target location are typed in by the operator (F2.2); the microSD file only pre-fills them. Scene scale follows the entered target. |
| 3 | Vision target | An IR beacon at the destination, seen from the fish's own camera. Matches the existing light tracker. |
| 4 | IMU telemetry | The ESP32-S3 streams ICM20948 samples over the same UDP socket as the fin ACKs. |
| 5 | Hand-over rule | Progress = dead-reckoned distance travelled ÷ planned path length; threshold default 80 %. Force hand-over and Return to IMU stay as advanced controls. |
| 6 | Abort | Fins to neutral and keep the link. |
| 7 | Replay | In scope for the hackathon (scrubber over the in-memory flight history). |
| 8 | Phone / tablet view | Not in the hackathon build. |
| 9 | Cues | No sound. A visual cue strip with a launch countdown and phase-change banners. |

Save log to JSON/CSV was not selected and is left for after the hackathon; the prototype's "Copy log" stands in for it.

## 11. Suggested implementation stack (after clarification)

Plain HTML/CSS/JS with Three.js (no build step), served by a Python ground station
(FastAPI + WebSocket, or the existing Flask app extended). This matches the existing
`light_tracker` code and keeps the hackathon setup to one `python gcs.py` command.

## 12. Change request, 26 Sep 2026: stop, edit, re-apply without gates

Status: **agreed and implemented, 26 Sep 2026**. Decisions: Stop in flight is a single press; Arm requires only the link and an applied mission; Apply mission is refused while Armed.

### Problem

Today the only ways out of a running or finished mission are Abort (lands in *Aborted*) and
New mission (only from *Arrived* / *Aborted*), and Arm is gated on the whole pre-flight list
(calibration, fin self-test, battery). On the bench that means: to change the target you must
abort, press New mission, re-calibrate, re-run the self-test, then arm again. Calibration and
self-test should be optional checks the operator can run before a mission, not a gate.

### Behaviour after the change

**Stop mission** (new, F3.6). One control, enabled in every phase except *Disconnected* and *Ready*:

| From phase | What Stop does |
|---|---|
| Armed | disarm, back to Ready |
| IMU glide / Hand-over / Vision homing | fins to neutral, guidance off, dead reckoning frozen, back to Ready (log: "Mission stopped by operator") |
| Arrived / Aborted | back to Ready (replaces today's "New mission") |

Stop keeps the link, the calibration, the self-test result, the mission parameters and the
last flight's history (replay scrubber stays available until the next launch). It sits in the
Guidance panel next to the phase stepper and has the shortcut `S`. Abort stays the emergency
control: single press, red, lands in *Aborted*, logged as critical. Stop is the routine one.

**Edit and re-apply** (F2.2 revised). In *Ready* every mission field (speed, target x y z), the
hand-over threshold and the tuning sliders are editable and *Apply mission* re-sends the mission,
moves the target, hand-over ring and planned path in the 3D view, and clears the old trail.
While *Armed* or flying, the mission fields and Apply are disabled with the hint
"Stop the mission to change parameters" (Armed: "Disarm to change parameters"). Threshold and
tuning sliders stay live during flight, as today.

**Pre-flight without gates** (F2.6 revised). The list splits into two groups:

| Group | Items | Effect on Arm |
|---|---|---|
| Required | fin board link, mission applied | Arm disabled until both are true |
| Recommended | IMU calibrated, fin self-test passed, battery above 7.0 V | Arm stays enabled; skipped items are listed under the Arm button and in the confirm step ("Confirm arm (5 s) · 2 checks skipped"); arming logs a warning naming them |

Calibrate IMU and Fin self-test are enabled only in *Ready* (before arming), can be re-run any
number of times, and are disabled from *Armed* onward. Their results survive Stop and are only
cleared by Disconnect.

**Live view.** Stop leaves the 3D view showing the finished trail with the replay scrubber;
Apply mission resets the scene to the launch point with the new target; Launch clears the trail.

### Data contract additions

- `POST /cmd {"cmd":"stop"}` → `{ok:true}` from any phase except Disconnected. `new_mission` becomes an alias.
- Telemetry `preflight` gains `required: {link, mission}`, `advisory: {calibrated, selftest, batt}`,
  `can_arm: bool`, `skipped: ["IMU calibration", ...]`.

### Decisions (confirmed 26 Sep 2026)

1. **Stop while flying** is a single press. It has the same physical effect as Abort (fins neutral) but lands in Ready.
2. **Arm requires** the fin board link and an applied mission. Calibration, self-test and battery are advisory and are listed when skipped.
3. **Apply mission while Armed** is refused with "Disarm to change the mission."; the fields are disabled in the page.

## 13. Additions, 26 Sep 2026: end-of-mission summary and speed in cm/s

- **F4.6 End-of-mission summary.** When the phase becomes *Arrived* or *Aborted*, a non-modal
  panel slides over the bottom of the 3D view: title (Arrived / Aborted · safe), the reason from
  the phase log, flight time, final estimated distance, path covered, when the hand-over happened,
  best beacon confidence and the largest drift estimate. Its primary button is **Back to Ready**
  (same as Stop mission). *Keep viewing* or `Esc` closes it and leaves the replay scrubber; the
  Stop mission button in the Guidance panel still works. Focus moves to Back to Ready when the
  panel opens and returns to Stop mission when it closes; a polite live-region message announces it.
  The panel is not modal, so nothing on the page is blocked.
- **F2.2 Units.** The launch speed field is entered and shown in **cm/s** (80 cm/s instead of
  0.8 m/s) and the target x, y, z fields in **cm** (200, 800, 100 instead of 2.0, 8.0, 1.0). The page
  divides by 100 before sending; the station, the telemetry, the 3D view scale, the log files and
  the microSD init file stay in **m/s** and **metres**.

## 14. Change, 26 Sep 2026: operator Abort returns to Ready

- **Abort while Armed** is a disarm. **Abort while flying** puts the fins to neutral and returns
  straight to *Ready*, logged as critical, keeping the link, calibration, parameters and the flight
  history. It is the same transition as Stop mission; the difference is only the log level and the
  red single-press control. The *Aborted* phase is now reached only by automatic faults (link lost
  for more than 1.5 s, path overrun without a beacon lock), where Back to Ready is still needed.
- **Summary panel** now appears at every mission end: *Arrived*, *Aborted* (fault) with Back to
  Ready, and *Stopped* / operator *Aborted* where the fish is already in Ready, so the panel only
  offers Close. Pointer, wheel and key events inside the panel no longer reach the 3D view's orbit
  handlers, which had been swallowing clicks on its buttons.

## 15. Swarm vision, 26 Sep 2026: follow a quad swarm's centroid

Status: **implemented 26 Sep 2026** (defaults: cluster radius 200 px, swarm match 60 px, 4 simulated quads within 0.4 m of the target). Decisions: IR LEDs on the quads are the primary detector, with a
YOLO nano path that must also run on a Raspberry Pi; the fish homes on the swarm centroid; fake mode
simulates a swarm; the 3D view shows the quads as models for the demo.

### Detection pipeline (station, `gcs/vision.py`)

1. **Sources.** Every frame yields a list of sources, each with pixel position, size, mass
   (summed intensity) and confidence. Two producers, merged: the IR blob detector (all blobs above
   the cutoff, up to `max_sources`, as in the light tracker) and, when weights are loaded, YOLO
   boxes filtered to the configured classes (box centre, box size, model score).
2. **Clustering.** Sources closer than `cluster_radius` px (directly or through a chain) form a
   swarm: mass-weighted centroid, member count, spread (RMS radius) and convex hull.
3. **Tracking.** Swarms keep a persistent ID across frames (`swarm_match` px); one that vanishes is
   remembered for ~10 frames so a flicker does not renumber. The followed swarm is the operator's
   choice, or the biggest by mass when nothing is chosen. While the followed swarm is briefly
   missing the fins hold (state *hold*) instead of jumping to another one.
4. **Target.** In *swarm* mode the "beacon" handed to guidance is the followed swarm's centroid;
   its confidence blends member count and contrast; its radius is the spread. *Beacon* mode
   (single brightest source) stays the default; the mode is a live tunable.

### YOLO on the laptop and on a Raspberry Pi

`--yolo weights.pt` uses the ultralytics package (laptop). `--yolo weights.onnx` uses onnxruntime,
which installs on a Pi in one line and runs a nano model at 320 px at a few frames per second on
the CPU; the same file is the input for the AI HAT / Hailo compiler later. Both backends return the
same box list, so the swarm logic does not care which one ran. Class filter: `yolo_classes`
(names or ids, comma separated). No drone dataset exists yet, so the README carries the recipe:
label frames, train `yolo11n`, export to ONNX at 320 px.

### Fake mode

The simulated fish's target becomes a swarm of `n` quads (default 4) hovering around the mission
target with a slow drift; the fake IR camera renders one LED per quad. Their true positions are
sent in telemetry (`sim.quads`) only in fake mode.

### Page changes

- **F5.6 Mode switch** Beacon / Swarm (segmented, `aria-pressed`), live.
- **F5.7 Swarm list**: one row per visible swarm (ID, quads, spread px), the followed one marked;
  Prev / Next buttons, keys `[` and `]`, click a swarm on the IR frame to follow it, *Auto* returns
  to "biggest". Text equivalent names the followed swarm and its quad count.
- **F5.1** annotated frame shows every source as a small circle, other swarms dim with their ID,
  the followed swarm's hull, spokes and crosshair, and YOLO boxes when present.
- **F3.3** hand-over row reads "Swarm seen N frames in a row" in swarm mode.
- **F4.7 Quads in the 3D view**: simple quadcopter models (cross frame, four rotors, one LED)
  at the true positions in fake mode, or a ring of *n* around the target marker in real flights
  where *n* is the detected quad count. Rotors spin unless motion is reduced. Legend entry added.
- Detection tuning gains cluster radius and swarm match sliders.

## 16. Synthetic training data, 26 Sep 2026

`tools/synth/index.html` (served by `tools/synth/server.py`) is a separate, single-purpose page
that renders procedurally varied quadcopters with Three.js and writes YOLO labels from the
projected geometry. Controls: count, image size, IR-style share, max drones per image, share of
empty frames, distractor share, validation share, seed; Preview, Generate and save, Stop; a
progress bar with a polite live-region status. `tools/train_quads.py` fine-tunes YOLO nano on the
result and exports ONNX for the Raspberry Pi. The GCS page itself is unchanged.

## 17. Simulated fish camera, 26 Sep 2026

In fake mode the page renders the fish's own point of view (62° × 49°, 640 × 480, from 0.3 m ahead
of the true simulated pose that the station now sends in telemetry) and streams it as JPEG frames
over `ws://…/camsim` at about 10 per second. The ground station treats those frames as the camera:
the annotated feed, the IR blob detector and YOLO all run on them, and it falls back to the dot
picture within a second if the page stops streaming. A select in the Vision panel (fake mode only)
chooses **IR-style** (black world, dark bodies, white LEDs, sensor noise and bloom, matching the
training data), **visible light** (sky and ground), or **off**. Operator overlays (grid, planned
path, hand-over ring, trails, estimate marker, halo) live on a separate render layer so they never
appear in the fish's view, and the fish model is hidden for that pass. The Vision panel's source
line says whether frames come from the page or from the station. Real mode is unaffected.

## 18. HIG redesign, 26 Sep 2026 (approved from the mock, implemented)

Source: `docs/HIG_AUDIT.md` (38 verified findings). Mock approved by the owner as shown. Changes on the live page:

- **Naming.** The fish is **Maelstrom**; the page title is "Maelstrom Ground Station", the brand block says Maelstrom / Ground station, sections are Mission, Guidance, Flight view, Trends, Vision, Fins, Log. Spec IDs left the UI. Downloads are `maelstrom-flight.json/csv`. Simulation shows as a "Simulated" pill, never in the brand.
- **Document.** Real doctype, `lang`, charset, viewport (`viewport-fit=cover`) and `color-scheme` meta.
- **Toolbar.** One 64 px row, three groups: brand + bordered link cluster (tabular values), phase chip + clock centred, Keys / Settings icon buttons and Abort trailing. Never wraps; at phone width the pill words drop and the dots and values stay.
- **Cue strip** only for time-critical or actionable messages: launch countdown, accelerometer wait, automatic Aborted (fault), operator abort, ground-station link lost / restored, a safety command the station refused. Routine phase changes go to the chip, the stepper, the log and the live region.
- **Stage-ordered deck.** `body[data-stage]` (disconnected, ready, armed, flying, ended) swaps the left column: Mission leads in Ready, Guidance leads from Armed on. Flight view and Trends share the centre column.
- **One primary action at a time**: Connect → Apply mission → Arm → Launch → Stop / Back to Ready, removed while disabled.
- **Mission fields** with the unit inside the field (cm/s, cm) and visible x / y / z labels; IP fields disabled while connected; init-file import behind a disclosure; step badges 1-2-3 turn into ticks.
- **Controls.** Joined segmented controls (camera, Beacon/Swarm, simulated camera, every Settings choice); disclosures with a rotating chevron; compact buttons are 44 px; every tuning and PID slider has a number field and printed range ends; the hand-over threshold has a % field; the override sliders show −30 / 0 / +30.
- **Feedback.** Settings and Keys are non-modal popovers (Esc, outside click, close button); Abort and the X shortcut work while open. Held keys no longer auto-repeat through the two-step Arm. Link pills turn "stale N s" when telemetry stops and "lost" after 3 s with a critical log line and cue; recovery is logged. A refused safety command shows a critical cue naming the command and reason.
- **Gauges, meters, charts.** Scale row over the fin gauges, `role="meter"` with live values; confidence meter with the threshold mark; charts with fixed axes (distance 0 to the path length with a hand-over reference, vision error 0 to half the frame width with the deadband reference).
- **Symbols.** One inline SVG sprite replaces the Unicode glyphs: phases, checks, lock states, log levels, camera/IMU/link, settings, keyboard, chevrons.
- **Colour and type.** Light-mode vision orange darkened to #D9531F; status text tokens used for dots and borders in both themes; `prefers-contrast: more` honoured with a tri-state setting; type scale tokens (h1 1.375 rem … footnote .8125 rem); buttons regular weight, primary bold.
- **Not changed.** WCAG AA, 44 px targets, single-press Abort, keyboard shortcuts, the summary panel over the flight view (now a compact single row).
