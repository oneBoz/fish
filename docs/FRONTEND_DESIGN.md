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
| 6 | Arrived        | nobody, fins neutral           | CV error inside deadband for M frames / range reached    | Save log, New mission                      |
| 7 | Aborted / Safe | nobody, fins neutral           | Abort pressed, link timeout, or guidance fault           | Save log, New mission, Connect             |

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
- F2.6 Pre-flight checklist auto-filled: link ok, init loaded, calibrated, fins responded to self-test,
  battery ok, camera sees ≥ 1 frame.

### F3 Guidance
- F3.1 Phase stepper listing all phases; current one marked with `aria-current="step"`.
- F3.2 Hand-over threshold slider 60–95 %, default 80 %, with a numeric input twin.
- F3.3 Hand-over checklist, live: progress ≥ threshold, beacon seen ≥ N consecutive frames,
  confidence ≥ c, link ok. Each row shows ✔/✘ plus the current value.
- F3.4 Force hand-over / Return to IMU (advanced, behind a disclosure).
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
