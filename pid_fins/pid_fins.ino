/*
  ESP32-S3 : Maelstrom fin node -> 4x SG90 fin servos + 2x camera gimbal servos

  The laptop ground station (python -m gcs) looks at the ESP32-CAM stream, finds the
  beacon or the swarm centroid, and sends one packet per tick over Wi-Fi/UDP:

      "YAW,PITCH,PAN,TILT\n"  e.g. "12.5,-4.0,96.0,52.0"
                              fins in degrees from neutral (+/- FIN_MAX),
                              camera pan / tilt as absolute servo angles (clamped to their limits)
      "YAW,PITCH\n"           fins only (older tracker); the camera keeps its last target
      "HOME\n"                fins to neutral, camera to its centre position

  Reply to every fin/camera packet:  "ACK yaw pitch pan tilt n\n"  (slewed, actual angles)

  Fin layout (4-fin cross):
      PITCH pair = servos 0 & 2  (horizontal fins: nose up / down)      GPIO 4, 6
      YAW   pair = servos 1 & 3  (vertical fins:   nose left / right)   GPIO 5, 7
  Camera gimbal:
      PAN  = GPIO 18, 5..175 deg, centre 90    (camera looks left / right)
      TILT = GPIO 17, 35..75 deg, centre 55    (camera looks down / up)
  Flip servoDirection[] entries to -1 for mirrored fin mountings. The camera's sense of
  direction is set on the laptop (invert pan / invert tilt tunables).

  If no packet arrives for LINK_TIMEOUT_MS the fins return to neutral and the camera to
  centre, so a dropped link never leaves the fish stuck in a hard turn.

  POWER:  2S LiPo -> UBEC 5V (>= 2 A for six SG90s) -> servo rail, grounds common with the
          board. Never feed the servos from the board's 3V3/5V pin: the board browns out
          and resets the moment a servo starts to move.
  BOARD:  ESP32-S3 with two USB-C ports. Use the BOTTOM (UART) port. Serial monitor 115200.
  Library: ESP32Servo (Library Manager -> "ESP32Servo" by Kevin Harrington)

  LIVE TUNING over serial -- type and press enter:
      t2 3.5   trim fin servo 2 by +3.5 deg     f1     flip fin servo 1 direction
      m25      set fin max deflection to 25 deg  z      fins neutral, camera centre
      w2       wiggle fin servo 2 (index 0-3)    w4     wiggle camera pan     w5   wiggle camera tilt
      c120 60  point the camera: pan 120, tilt 60 (ignores the link until the next packet)
      a        wiggle everything in turn (same as the boot self-test)
      l        toggle the movement log on/off      ?      print settings

  LOG (every 500 ms, toggle with 'l'):
      link ok  yaw 12.5->12.0  pitch -4.0->-3.0  cam 96.0->95.0 52.0->52.0 | S0(g4) 102.0 1588us | ... | PAN(g18) 95.0 1508us | TILT(g17) 52.0 1049us
      "a->b" is target -> slewed value. If a servo's number changes but the servo doesn't,
      it is wiring/power; if the number never changes, it is the command path.
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

#define USE_USB_CDC 0        // 0 = bottom UART port (correct for your board)

#if USE_USB_CDC
  #define DBG Serial
#elif defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  #define DBG Serial0
#else
  #define DBG Serial
#endif

// ==================== USER CONFIG ====================

const char* WIFI_SSID = "DING DONG";
const char* WIFI_PASS = "12345679";

// Static IP so the address never changes on reconnect (phone hotspots have no
// DHCP reservation UI). Set STATIC_IP to all zeros to fall back to DHCP.
// .13 = ESP32-CAM, .12 = this fin board. The hotspot subnet is a /28 (.0-.15):
// .15 is the broadcast address and must NOT be used as a device IP.
IPAddress STATIC_IP (172, 20, 10, 12);
IPAddress GATEWAY   (172, 20, 10, 1);
IPAddress SUBNET     (255, 255, 255, 240);
IPAddress DNS_SERVER (172, 20, 10, 1);

const int UDP_PORT = 4210;

// ==================== PINS ====================

#define SERVO_COUNT 4
const int SERVO_PINS[SERVO_COUNT] = { 4, 5, 6, 7 };
const int PITCH_FINS[2] = { 0, 2 };    // horizontal fins
const int YAW_FINS[2]   = { 1, 3 };    // vertical fins

#define CAM_PAN_PIN  18
#define CAM_TILT_PIN 17

#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2400

// ==================== CONTROL CONFIG ====================

float servoNeutral = 90.0f;
float servoDirection[SERVO_COUNT] = { 1, 1, 1, 1 };   // flip to -1 as needed
float servoTrim[SERVO_COUNT]      = { 0, 0, 0, 0 };   // mechanical squaring

float FIN_MAX = 30.0f;                 // deflection limit either side of neutral, deg
#define SELF_TEST_ON_BOOT 1            // 1 = wiggle each servo in turn at power-up
const float SLEW_DEG_PER_TICK = 3.0f;  // max fin movement per 20 ms tick (smooth, no brownouts)
const unsigned long LINK_TIMEOUT_MS = 1000;   // no packet for this long -> fins neutral, camera centre

// camera gimbal limits (absolute servo angles) and centre position
const float PAN_MIN = 5.0f,   PAN_MAX = 175.0f, PAN_HOME = 90.0f;
const float TILT_MIN = 35.0f, TILT_MAX = 75.0f, TILT_HOME = 55.0f;
const float CAM_SLEW_DEG_PER_TICK = 4.0f;     // camera moves a little faster than the fins

// ==================== STATE ====================

WiFiUDP udp;
Servo servos[SERVO_COUNT];
Servo camPan, camTilt;

float targetYaw = 0, targetPitch = 0;    // last fin command from the ground station
float yaw = 0, pitch = 0;                // slew-limited, what the fins actually do
float targetPan = PAN_HOME, targetTilt = TILT_HOME;   // last camera command
float pan = PAN_HOME, tilt = TILT_HOME;               // slew-limited camera angles
unsigned long lastPacketMs = 0;
unsigned long lastTickMs = 0;
unsigned long lastPrintMs = 0;
uint32_t packetsRx = 0;
bool linkWasAlive = false;
bool logEnabled = true;

float servoCmdDeg[SERVO_COUNT] = { 0, 0, 0, 0 };   // last angle asked of each fin servo
int   servoCmdUs[SERVO_COUNT]  = { 0, 0, 0, 0 };   // last pulse width written
float camCmdDeg[2] = { PAN_HOME, TILT_HOME };      // [0] pan, [1] tilt
int   camCmdUs[2]  = { 0, 0 };

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static int degToUs(float angleDeg) {
  angleDeg = clampf(angleDeg, 0.0f, 180.0f);
  return (int)(SERVO_MIN_PULSE + (angleDeg / 180.0f) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
}

// Write in microseconds rather than whole degrees -- SG90s resolve finer
// than 1 deg, and integer degrees make the motion look steppy.
void writeServoAngle(int i, float angleDeg) {
  angleDeg = clampf(angleDeg, 0.0f, 180.0f);
  int us = degToUs(angleDeg);
  servos[i].writeMicroseconds(us);
  servoCmdDeg[i] = angleDeg;
  servoCmdUs[i]  = us;
}

// Camera servos are always clamped to their mechanical limits, whatever is asked.
void writeCam(int which, float angleDeg) {      // which: 0 = pan, 1 = tilt
  if (which == 0) angleDeg = clampf(angleDeg, PAN_MIN, PAN_MAX);
  else            angleDeg = clampf(angleDeg, TILT_MIN, TILT_MAX);
  int us = degToUs(angleDeg);
  (which == 0 ? camPan : camTilt).writeMicroseconds(us);
  camCmdDeg[which] = angleDeg;
  camCmdUs[which]  = us;
}

void applyCam(float panDeg, float tiltDeg) { writeCam(0, panDeg); writeCam(1, tiltDeg); }

// One line with every servo's commanded angle + pulse, for the log.
void printServos() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    DBG.print(" | S"); DBG.print(i);
    DBG.print("(g"); DBG.print(SERVO_PINS[i]); DBG.print(") ");
    DBG.print(servoCmdDeg[i], 1); DBG.print(" ");
    DBG.print(servoCmdUs[i]); DBG.print("us");
    if (!servos[i].attached()) DBG.print(" NOT-ATTACHED");
  }
  DBG.print(" | PAN(g"); DBG.print(CAM_PAN_PIN); DBG.print(") "); DBG.print(camCmdDeg[0], 1); DBG.print(" "); DBG.print(camCmdUs[0]); DBG.print("us");
  if (!camPan.attached()) DBG.print(" NOT-ATTACHED");
  DBG.print(" | TILT(g"); DBG.print(CAM_TILT_PIN); DBG.print(") "); DBG.print(camCmdDeg[1], 1); DBG.print(" "); DBG.print(camCmdUs[1]); DBG.print("us");
  if (!camTilt.attached()) DBG.print(" NOT-ATTACHED");
  DBG.println();
}

// Move one fin servo through -30 / +30 / neutral, blocking, printing each step.
// Bypasses the link so you can tell a wiring fault from a command fault.
void wiggleServo(int i) {
  if (i == 4 || i == 5) { wiggleCam(i - 4); return; }
  if (i < 0 || i >= SERVO_COUNT) return;
  DBG.print("wiggle S"); DBG.print(i); DBG.print(" (GPIO "); DBG.print(SERVO_PINS[i]);
  DBG.print(") attached="); DBG.println(servos[i].attached() ? "yes" : "NO");
  const float steps[3] = { -30.0f, 30.0f, 0.0f };
  for (int k = 0; k < 3; k++) {
    writeServoAngle(i, servoNeutral + servoTrim[i] + steps[k] * servoDirection[i]);
    DBG.print("   -> "); DBG.print(servoCmdDeg[i], 1); DBG.print(" deg  ");
    DBG.print(servoCmdUs[i]); DBG.println(" us");
    delay(500);
  }
}

// Camera servo through its range: low limit (+25 for pan) / high limit / centre.
void wiggleCam(int which) {
  const bool isPan = (which == 0);
  DBG.print("wiggle "); DBG.print(isPan ? "PAN (GPIO 18)" : "TILT (GPIO 17)");
  DBG.print(" attached="); DBG.println((isPan ? camPan : camTilt).attached() ? "yes" : "NO");
  const float steps[3] = { isPan ? PAN_MIN + 25.0f : TILT_MIN, isPan ? PAN_MAX - 25.0f : TILT_MAX, isPan ? PAN_HOME : TILT_HOME };
  for (int k = 0; k < 3; k++) {
    writeCam(which, steps[k]);
    DBG.print("   -> "); DBG.print(camCmdDeg[which], 1); DBG.print(" deg  ");
    DBG.print(camCmdUs[which]); DBG.println(" us");
    delay(500);
  }
  if (isPan) pan = targetPan = PAN_HOME; else tilt = targetTilt = TILT_HOME;
}

void wiggleAll() {
  DBG.println("self-test: wiggling each fin servo, then the camera pan and tilt");
  for (int i = 0; i < SERVO_COUNT; i++) wiggleServo(i);
  wiggleCam(0);
  wiggleCam(1);
  parkFins();
  parkCam();
  DBG.println("self-test done, fins neutral, camera centred");
}

void applyFins(float yawDeg, float pitchDeg) {
  for (int k = 0; k < 2; k++) {
    int p = PITCH_FINS[k], y = YAW_FINS[k];
    writeServoAngle(p, servoNeutral + servoTrim[p] + pitchDeg * servoDirection[p]);
    writeServoAngle(y, servoNeutral + servoTrim[y] + yawDeg   * servoDirection[y]);
  }
}

void parkFins() {
  targetYaw = targetPitch = yaw = pitch = 0;
  applyFins(0, 0);
}

void parkCam() {
  targetPan = pan = PAN_HOME;
  targetTilt = tilt = TILT_HOME;
  applyCam(pan, tilt);
}

// ==================== WIFI / UDP ====================

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  if (STATIC_IP != IPAddress(0, 0, 0, 0)) {
    if (!WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS_SERVER))
      DBG.println("Static IP config failed, falling back to DHCP");
  }
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  DBG.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); DBG.print("."); }
  DBG.printf("\nFin node IP: %s  (UDP port %d)\n", WiFi.localIP().toString().c_str(), UDP_PORT);
}

void sendAck() {
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.printf("ACK %.1f %.1f %.1f %.1f %lu\n", yaw, pitch, pan, tilt, (unsigned long)packetsRx);
  udp.endPacket();
}

void handlePacket() {
  int len = udp.parsePacket();
  if (len <= 0) return;

  char buf[96];
  int n = udp.read(buf, sizeof(buf) - 1);
  if (n <= 0) return;
  buf[n] = '\0';

  float y, p, cp, ct;
  int got = sscanf(buf, "%f,%f,%f,%f", &y, &p, &cp, &ct);
  if (got >= 2) {
    targetYaw   = clampf(y, -FIN_MAX, FIN_MAX);
    targetPitch = clampf(p, -FIN_MAX, FIN_MAX);
    if (got == 4) {
      targetPan  = clampf(cp, PAN_MIN, PAN_MAX);
      targetTilt = clampf(ct, TILT_MIN, TILT_MAX);
    }
    lastPacketMs = millis();
    packetsRx++;
    sendAck();                            // actual (slewed) angles, so the page shows link health
  } else if (strncmp(buf, "HOME", 4) == 0) {
    targetYaw = targetPitch = 0;
    targetPan = PAN_HOME; targetTilt = TILT_HOME;
    lastPacketMs = millis();
    packetsRx++;
    sendAck();
  } else if (strncmp(buf, "PING", 4) == 0) {
    lastPacketMs = millis();
    sendAck();
  }
}

void tickFins() {
  unsigned long now = millis();
  if (now - lastTickMs < 20) return;     // 50 Hz servo update
  lastTickMs = now;

  bool linkAlive = (now - lastPacketMs) < LINK_TIMEOUT_MS;
  if (!linkAlive) {
    targetYaw = targetPitch = 0;         // ground station went quiet: glide straight, look ahead
    targetPan = PAN_HOME; targetTilt = TILT_HOME;
    if (linkWasAlive) DBG.println("link lost -> fins neutral, camera centred");
  }
  linkWasAlive = linkAlive;

  yaw   += clampf(targetYaw   - yaw,   -SLEW_DEG_PER_TICK, SLEW_DEG_PER_TICK);
  pitch += clampf(targetPitch - pitch, -SLEW_DEG_PER_TICK, SLEW_DEG_PER_TICK);
  applyFins(yaw, pitch);
  pan   += clampf(targetPan   - pan,   -CAM_SLEW_DEG_PER_TICK, CAM_SLEW_DEG_PER_TICK);
  tilt  += clampf(targetTilt  - tilt,  -CAM_SLEW_DEG_PER_TICK, CAM_SLEW_DEG_PER_TICK);
  applyCam(pan, tilt);
}

// ==================== SERIAL TUNING ====================

void printSettings() {
  DBG.print("FIN_MAX "); DBG.print(FIN_MAX, 1);
  DBG.print("  yaw "); DBG.print(yaw, 1);
  DBG.print("  pitch "); DBG.print(pitch, 1);
  DBG.print("  pan "); DBG.print(pan, 1); DBG.print(" ["); DBG.print(PAN_MIN, 0); DBG.print("-"); DBG.print(PAN_MAX, 0); DBG.print("]");
  DBG.print("  tilt "); DBG.print(tilt, 1); DBG.print(" ["); DBG.print(TILT_MIN, 0); DBG.print("-"); DBG.print(TILT_MAX, 0); DBG.print("]");
  DBG.print("  packets "); DBG.println(packetsRx);
  for (int i = 0; i < SERVO_COUNT; i++) {
    DBG.print("  servo "); DBG.print(i);
    DBG.print(" dir "); DBG.print(servoDirection[i], 0);
    DBG.print(" trim "); DBG.println(servoTrim[i], 1);
  }
}

void handleSerial() {
  if (!DBG.available()) return;
  char c = DBG.read();

  if (c == '?') { printSettings(); return; }
  if (c == 'z') { parkFins(); parkCam(); DBG.println("fins neutral, camera centred"); return; }
  if (c == 'm') { FIN_MAX = DBG.parseFloat(); DBG.print("FIN_MAX = "); DBG.println(FIN_MAX, 1); return; }
  if (c == 'f') {
    int i = DBG.parseInt();
    if (i >= 0 && i < SERVO_COUNT) { servoDirection[i] = -servoDirection[i];
      DBG.print("servo "); DBG.print(i); DBG.print(" dir "); DBG.println(servoDirection[i], 0); }
    return;
  }
  if (c == 't') {
    int i = DBG.parseInt();
    float v = DBG.parseFloat();
    if (i >= 0 && i < SERVO_COUNT) { servoTrim[i] = v;
      DBG.print("servo "); DBG.print(i); DBG.print(" trim "); DBG.println(v, 1); }
    return;
  }
  if (c == 'c') {
    float cp = DBG.parseFloat(); float ct = DBG.parseFloat();
    targetPan = pan = clampf(cp, PAN_MIN, PAN_MAX); targetTilt = tilt = clampf(ct, TILT_MIN, TILT_MAX);
    applyCam(pan, tilt);
    DBG.print("camera pan "); DBG.print(pan, 1); DBG.print(" tilt "); DBG.println(tilt, 1);
    return;
  }
  if (c == 'w') { wiggleServo(DBG.parseInt()); return; }
  if (c == 'a') { wiggleAll(); return; }
  if (c == 'l') { logEnabled = !logEnabled; DBG.print("log "); DBG.println(logEnabled ? "on" : "off"); return; }
}

// ==================== SETUP / LOOP ====================

void setup() {
#if USE_USB_CDC
  DBG.begin(115200);
#else
  DBG.begin(115200, SERIAL_8N1, 44, 43);   // RX 44, TX 43
#endif
  delay(200);
  DBG.println("=== Maelstrom fin + camera gimbal node ===");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  for (int i = 0; i < SERVO_COUNT; i++) {
    servos[i].setPeriodHertz(50);
    int ch = servos[i].attach(SERVO_PINS[i], SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    DBG.print("servo "); DBG.print(i); DBG.print(" on GPIO "); DBG.print(SERVO_PINS[i]);
    DBG.print(" -> attach "); DBG.println(ch ? "ok" : "FAILED (no free timer/channel?)");
  }
  camPan.setPeriodHertz(50);
  DBG.print("camera PAN on GPIO "); DBG.print(CAM_PAN_PIN); DBG.print(" -> attach ");
  DBG.println(camPan.attach(CAM_PAN_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE) ? "ok" : "FAILED");
  camTilt.setPeriodHertz(50);
  DBG.print("camera TILT on GPIO "); DBG.print(CAM_TILT_PIN); DBG.print(" -> attach ");
  DBG.println(camTilt.attach(CAM_TILT_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE) ? "ok" : "FAILED");
  parkFins();
  parkCam();
  delay(600);
#if SELF_TEST_ON_BOOT
  wiggleAll();
#endif

  connectWiFi();
  udp.begin(UDP_PORT);
  lastPacketMs = 0;                      // link starts "dead": fins neutral, camera centred
  DBG.println("Waiting for the ground station. Type ? for settings.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) { connectWiFi(); udp.begin(UDP_PORT); }
  handleSerial();
  handlePacket();
  tickFins();

  if (logEnabled && millis() - lastPrintMs >= 500) {
    lastPrintMs = millis();
    DBG.print("link "); DBG.print(linkWasAlive ? "ok" : "--");
    DBG.print("  yaw ");   DBG.print(targetYaw, 1);   DBG.print("->"); DBG.print(yaw, 1);
    DBG.print("  pitch "); DBG.print(targetPitch, 1); DBG.print("->"); DBG.print(pitch, 1);
    DBG.print("  cam ");   DBG.print(targetPan, 1);   DBG.print("->"); DBG.print(pan, 1);
    DBG.print(" ");        DBG.print(targetTilt, 1);  DBG.print("->"); DBG.print(tilt, 1);
    printServos();
  }
}
