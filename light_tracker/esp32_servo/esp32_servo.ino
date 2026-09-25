/*
 * ESP32 (DevKit) — pan/tilt servo controller
 *
 * Listens for UDP packets on port 4210 in the form:
 *      "PAN,TILT\n"      e.g. "97.5,84.0"
 * Angles are in degrees (0-180). The laptop tracker sends these.
 *
 * Also replies "ACK pan tilt" to the sender so the dashboard can show link health.
 *
 * Library: ESP32Servo  (Library Manager -> "ESP32Servo" by Kevin Harrington)
 * Board: any ESP32 dev board
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// ---------- USER CONFIG ----------
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// Static IP so the address never changes on reconnect (e.g. to a phone hotspot,
// which has no DHCP-reservation UI). Pick an address outside what your
// router/hotspot hands out via DHCP, or one you've confirmed is free.
// Set STATIC_IP to all zeros to fall back to normal DHCP.
IPAddress STATIC_IP (172, 20, 10, 14);   // top of the /28 range: hotspot DHCP hands out .2 upward, so this won't collide
IPAddress GATEWAY   (172, 20, 10, 1);
IPAddress SUBNET     (255, 255, 255, 240);
IPAddress DNS_SERVER (172, 20, 10, 1);

const int   UDP_PORT   = 4210;
const int   PAN_PIN    = 13;
const int   TILT_PIN   = 12;

// Mechanical limits of the optics bracket (degrees) and the rest position.
// These MUST match PAN_RANGE / TILT_RANGE / *_HOME in laptop/tracker.py.
const float PAN_MIN  = 5,   PAN_MAX  = 175;
const float TILT_MIN = 5,   TILT_MAX = 105;
const float PAN_HOME = 90,  TILT_HOME = 55;

// Smoothing: max degrees the servo may move per 20 ms tick (prevents jerks / brownouts)
const float MAX_STEP_DEG = 3.0;
// If no packet arrives for this long, stop moving (hold last target)
const unsigned long LINK_TIMEOUT_MS = 2000;
// ---------------------------------

WiFiUDP udp;
Servo panServo, tiltServo;

float targetPan = PAN_HOME, targetTilt = TILT_HOME;
float currentPan = PAN_HOME, currentTilt = TILT_HOME;
unsigned long lastPacketMs = 0;
unsigned long lastTickMs = 0;
uint32_t packetsRx = 0;

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  if (STATIC_IP != IPAddress(0, 0, 0, 0)) {
    if (!WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS_SERVER)) {
      Serial.println("Static IP config failed, falling back to DHCP");
    }
  }
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.printf("\nServo node IP: %s  (UDP port %d)\n", WiFi.localIP().toString().c_str(), UDP_PORT);
}

void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(PAN_PIN, 500, 2500);    // adjust pulse range for your servos (SG90: 500-2400)
  tiltServo.attach(TILT_PIN, 500, 2500);
  panServo.write(PAN_HOME);
  tiltServo.write(TILT_HOME);

  connectWiFi();
  udp.begin(UDP_PORT);
  lastPacketMs = millis();
}

void handlePacket() {
  int len = udp.parsePacket();
  if (len <= 0) return;

  char buf[64];
  int n = udp.read(buf, sizeof(buf) - 1);
  if (n <= 0) return;
  buf[n] = '\0';

  float p, t;
  if (sscanf(buf, "%f,%f", &p, &t) == 2) {
    targetPan  = clampf(p, PAN_MIN, PAN_MAX);
    targetTilt = clampf(t, TILT_MIN, TILT_MAX);
    lastPacketMs = millis();
    packetsRx++;

    // ACK back to sender with the *actual* servo position
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.printf("ACK %.1f %.1f %lu\n", currentPan, currentTilt, packetsRx);
    udp.endPacket();
  } else if (strncmp(buf, "HOME", 4) == 0) {
    targetPan = PAN_HOME; targetTilt = TILT_HOME;
    lastPacketMs = millis();
  }
}

void tickServos() {
  unsigned long now = millis();
  if (now - lastTickMs < 20) return;     // 50 Hz servo update
  lastTickMs = now;

  bool linkAlive = (now - lastPacketMs) < LINK_TIMEOUT_MS;
  if (!linkAlive) return;                // hold position when the laptop goes quiet

  // Slew-rate limited move toward target
  float dp = clampf(targetPan  - currentPan,  -MAX_STEP_DEG, MAX_STEP_DEG);
  float dt = clampf(targetTilt - currentTilt, -MAX_STEP_DEG, MAX_STEP_DEG);
  currentPan  += dp;
  currentTilt += dt;

  panServo.write((int)(currentPan + 0.5f));
  tiltServo.write((int)(currentTilt + 0.5f));
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) { connectWiFi(); udp.begin(UDP_PORT); }
  handlePacket();
  tickServos();
}
