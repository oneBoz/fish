/*
  ESP32-S3 : light tracker -> 4x SG90 fin servos

  The MPU6050 is no longer used. The laptop light tracker
  (light_tracker/laptop/tracker.py --output fins) looks at the ESP32-CAM
  stream, finds the light, and sends fin deflections over Wi-Fi/UDP:

      "YAW,PITCH\n"     e.g. "12.5,-4.0"   degrees from neutral, +/- FIN_MAX
      "HOME\n"          fins to neutral

  Fin layout (4-fin cross):
      PITCH pair = servos 0 & 2  (horizontal fins: nose up / down)
      YAW   pair = servos 1 & 3  (vertical fins:   nose left / right)
  Flip servoDirection[] entries to -1 for mirrored mountings.

  If no packet arrives for LINK_TIMEOUT_MS the fins return to neutral, so a
  dropped link never leaves the fish stuck in a hard turn.

  POWER:  2S LiPo -> UBEC 5V -> servo rail + ESP32 5V pin, grounds common.
  BOARD:  ESP32-S3 with two USB-C ports. Use the BOTTOM (UART) port.
          Serial monitor at 115200.
  Servos: GPIO 4, 5, 6, 7
  Library: ESP32Servo (Library Manager -> "ESP32Servo" by Kevin Harrington)

  LIVE TUNING over serial -- type and press enter:
      t2 3.5   trim servo 2 by +3.5 deg       f1     flip servo 1 direction
      m25      set max deflection to 25 deg    z      fins to neutral
      w2       wiggle servo 2 (index, not GPIO) -30/+30/neutral, ignoring the tracker
      a        wiggle all four servos in turn (same as the boot self-test)
      l        toggle the per-servo movement log on/off
      ?        print settings

  LOG (every 500 ms, toggle with 'l'):
      link ok  yaw  12.5->12.0  pitch -4.0->-3.0 | S0(g4) 102.0 1588us | S1(g5) ...
      "a->b" is target -> slewed value. Each S<i> shows the servo INDEX, its
      GPIO, the angle commanded and the pulse width actually written.
      If a servo's number changes but the fin doesn't, it is wiring/power;
      if the number never changes, it is the command path.
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

#define USE_USB_CDC 0        // 0 = bottom UART port (correct for your board)

// Which object is the bottom UART port depends on the board option
// Tools -> "USB CDC On Boot": enabled -> Serial is USB and Serial0 is the UART;
// disabled -> Serial *is* the UART and Serial0 does not exist.
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
// .13 = ESP32-CAM, .14 = pan/tilt node, .12 = this fin board.
// The hotspot subnet is a /28 (.0-.15): .15 is the broadcast address and
// must NOT be used as a device IP (the laptop refuses to send to it).
IPAddress STATIC_IP (172, 20, 10, 12);
IPAddress GATEWAY   (172, 20, 10, 1);
IPAddress SUBNET     (255, 255, 255, 240);
IPAddress DNS_SERVER (172, 20, 10, 1);

const int UDP_PORT = 4210;             // same port as the pan/tilt node; different IP

// ==================== PINS ====================

#define SERVO_COUNT 4
const int SERVO_PINS[SERVO_COUNT] = { 4, 5, 6, 7 };
const int PITCH_FINS[2] = { 0, 2 };    // horizontal fins
const int YAW_FINS[2]   = { 1, 3 };    // vertical fins

#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2400

// ==================== CONTROL CONFIG ====================

float servoNeutral = 90.0f;
float servoDirection[SERVO_COUNT] = { 1, 1, 1, 1 };   // flip to -1 as needed
float servoTrim[SERVO_COUNT]      = { 0, 0, 0, 0 };   // mechanical squaring

float FIN_MAX = 30.0f;                 // deflection limit either side of neutral, deg
#define SELF_TEST_ON_BOOT 1            // 1 = wiggle each servo in turn at power-up
const float SLEW_DEG_PER_TICK = 3.0f;  // max fin movement per 20 ms tick (smooth, no brownouts)
const unsigned long LINK_TIMEOUT_MS = 1000;   // no packet for this long -> fins to neutral

// ==================== STATE ====================

WiFiUDP udp;
Servo servos[SERVO_COUNT];

float targetYaw = 0, targetPitch = 0;    // last command from the tracker
float yaw = 0, pitch = 0;                // slew-limited, what the fins actually do
unsigned long lastPacketMs = 0;
unsigned long lastTickMs = 0;
unsigned long lastPrintMs = 0;
uint32_t packetsRx = 0;
bool linkWasAlive = false;
bool logEnabled = true;

float servoCmdDeg[SERVO_COUNT] = { 0, 0, 0, 0 };   // last angle asked of each servo
int   servoCmdUs[SERVO_COUNT]  = { 0, 0, 0, 0 };   // last pulse width written

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Write in microseconds rather than whole degrees -- SG90s resolve finer
// than 1 deg, and integer degrees make the motion look steppy.
void writeServoAngle(int i, float angleDeg) {
  angleDeg = clampf(angleDeg, 0.0f, 180.0f);
  int us = (int)(SERVO_MIN_PULSE +
                 (angleDeg / 180.0f) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
  servos[i].writeMicroseconds(us);
  servoCmdDeg[i] = angleDeg;
  servoCmdUs[i]  = us;
}

// One line with every servo's commanded angle + pulse, for the log.
void printServos() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    DBG.print(" | S"); DBG.print(i);
    DBG.print("(g"); DBG.print(SERVO_PINS[i]); DBG.print(") ");
    DBG.print(servoCmdDeg[i], 1); DBG.print(" ");
    DBG.print(servoCmdUs[i]); DBG.print("us");
    if (!servos[i].attached()) DBG.print(" NOT-ATTACHED");
  }
  DBG.println();
}

// Move one servo through -30 / +30 / neutral, blocking, printing each step.
// Bypasses the tracker so you can tell a wiring fault from a command fault.
void wiggleServo(int i) {
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

void wiggleAll() {
  DBG.println("self-test: wiggling each servo in turn");
  for (int i = 0; i < SERVO_COUNT; i++) wiggleServo(i);
  parkFins();
  DBG.println("self-test done, fins neutral");
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

void handlePacket() {
  int len = udp.parsePacket();
  if (len <= 0) return;

  char buf[64];
  int n = udp.read(buf, sizeof(buf) - 1);
  if (n <= 0) return;
  buf[n] = '\0';

  float y, p;
  if (sscanf(buf, "%f,%f", &y, &p) == 2) {
    targetYaw   = clampf(y, -FIN_MAX, FIN_MAX);
    targetPitch = clampf(p, -FIN_MAX, FIN_MAX);
    lastPacketMs = millis();
    packetsRx++;

    // ACK with the *actual* fin position so the dashboard shows link health
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.printf("ACK %.1f %.1f %lu\n", yaw, pitch, packetsRx);
    udp.endPacket();
  } else if (strncmp(buf, "HOME", 4) == 0) {
    targetYaw = targetPitch = 0;
    lastPacketMs = millis();
  }
}

void tickFins() {
  unsigned long now = millis();
  if (now - lastTickMs < 20) return;     // 50 Hz servo update
  lastTickMs = now;

  bool linkAlive = (now - lastPacketMs) < LINK_TIMEOUT_MS;
  if (!linkAlive) {
    targetYaw = targetPitch = 0;         // tracker went quiet: glide straight
    if (linkWasAlive) DBG.println("link lost -> fins to neutral");
  }
  linkWasAlive = linkAlive;

  yaw   += clampf(targetYaw   - yaw,   -SLEW_DEG_PER_TICK, SLEW_DEG_PER_TICK);
  pitch += clampf(targetPitch - pitch, -SLEW_DEG_PER_TICK, SLEW_DEG_PER_TICK);
  applyFins(yaw, pitch);
}

// ==================== SERIAL TUNING ====================

void printSettings() {
  DBG.print("FIN_MAX "); DBG.print(FIN_MAX, 1);
  DBG.print("  yaw "); DBG.print(yaw, 1);
  DBG.print("  pitch "); DBG.print(pitch, 1);
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
  if (c == 'z') { parkFins(); DBG.println("fins neutral"); return; }
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
  DBG.println("=== Light-tracker fin controller ===");

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
  parkFins();
  delay(600);
#if SELF_TEST_ON_BOOT
  wiggleAll();
#endif

  connectWiFi();
  udp.begin(UDP_PORT);
  lastPacketMs = 0;                      // link starts "dead": fins stay neutral
  DBG.println("Waiting for tracker. Type ? for settings.");
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
    printServos();
  }
}
