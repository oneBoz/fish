/*
  fish_node.ino -- ESP32-S3 fish node for the Fish GCS
  = pid_fins.ino (4 fin servos over UDP) + ICM20948 streaming + PING/MISSION/LAUNCH.

  UDP 4210, text lines:
    laptop -> fish   "yaw,pitch\n"  fin deflection (deg from neutral, +/-FIN_MAX), answered with ACK
                     "HOME\n"       fins neutral
                     "PING\n"       keep-alive, answered with ACK (fins unchanged)
                     "MISSION,speed,x,y,z\n"  stored for onboard guidance (--guidance fish)
                     "LAUNCH\n"     start onboard guidance (--guidance fish)
    fish -> laptop   "ACK yaw pitch n\n"
                     "IMU,ms,ax,ay,az,gx,gy,gz\n"  accel m/s^2, gyro deg/s, SENSOR axes, IMU_RATE_HZ
                     "BATT,volts\n" every 2 s (if BATT_PIN is wired, else 0.0)
  IMU lines go to whoever sent the last packet, so the laptop just has to talk first.

  Fin layout (4-fin cross): PITCH pair = servos 0 & 2 (horizontal fins), YAW pair = 1 & 3 (vertical).
  Positive yaw fin = nose right, positive pitch fin = nose up. Flip servoDirection[] for mirrored mounts.
  If no packet arrives for LINK_TIMEOUT_MS the fins return to neutral.

  Libraries (Library Manager): "ESP32Servo" (Kevin Harrington),
                               "SparkFun 9DoF IMU Breakout - ICM 20948" (SparkFun).
  Wiring: ICM20948 on I2C (SDA/SCL below, 3V3, GND, AD0 -> GND so address = 0x68).
  Board: ESP32-S3, upload via the bottom UART USB-C port, serial monitor 115200.

  IMU axis mapping is done on the laptop (--imu-axes); this sketch streams raw sensor axes.
  onboardGuidance() is a STUB: with --guidance fish the fins stay neutral until you fill it in.
  Default ground-station mode is --guidance laptop, which needs nothing from this stub.
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include "ICM_20948.h"

#define USE_USB_CDC 0
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
IPAddress STATIC_IP (172, 20, 10, 12);
IPAddress GATEWAY   (172, 20, 10, 1);
IPAddress SUBNET     (255, 255, 255, 240);
IPAddress DNS_SERVER (172, 20, 10, 1);
const int UDP_PORT = 4210;

#define SERVO_COUNT 4
const int SERVO_PINS[SERVO_COUNT] = { 4, 5, 6, 7 };
const int PITCH_FINS[2] = { 0, 2 };
const int YAW_FINS[2]   = { 1, 3 };
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2400

#define I2C_SDA 8
#define I2C_SCL 9
#define AD0_VAL 0                 // ICM20948 AD0 pin low -> 0x68
const int IMU_RATE_HZ = 50;
const int BATT_PIN = -1;          // ADC pin through a divider, -1 = not wired
const float BATT_DIVIDER = 2.0f;  // (R1+R2)/R2

float servoNeutral = 90.0f;
float servoDirection[SERVO_COUNT] = { 1, 1, 1, 1 };
float servoTrim[SERVO_COUNT]      = { 0, 0, 0, 0 };
float FIN_MAX = 30.0f;
#define SELF_TEST_ON_BOOT 1
const float SLEW_DEG_PER_TICK = 3.0f;
const unsigned long LINK_TIMEOUT_MS = 1000;

// ==================== STATE ====================
WiFiUDP udp;
Servo servos[SERVO_COUNT];
ICM_20948_I2C imu;
bool imuOk = false;

float targetYaw = 0, targetPitch = 0;
float yaw = 0, pitch = 0;
unsigned long lastPacketMs = 0, lastTickMs = 0, lastPrintMs = 0, lastImuMs = 0, lastBattMs = 0;
uint32_t packetsRx = 0;
bool linkWasAlive = false;
bool logEnabled = true;
IPAddress peerIp; uint16_t peerPort = 0;   // whoever talked to us last gets the IMU stream

// mission for onboard guidance (only used with --guidance fish)
float missionSpeed = 0, missionX = 0, missionY = 0, missionZ = 0;
bool missionSet = false, onboardActive = false;

float servoCmdDeg[SERVO_COUNT] = { 0, 0, 0, 0 };
int   servoCmdUs[SERVO_COUNT]  = { 0, 0, 0, 0 };

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void writeServoAngle(int i, float angleDeg) {
  angleDeg = clampf(angleDeg, 0.0f, 180.0f);
  int us = (int)(SERVO_MIN_PULSE + (angleDeg / 180.0f) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
  servos[i].writeMicroseconds(us);
  servoCmdDeg[i] = angleDeg; servoCmdUs[i] = us;
}

void applyFins(float yawDeg, float pitchDeg) {
  for (int k = 0; k < 2; k++) {
    int p = PITCH_FINS[k], y = YAW_FINS[k];
    writeServoAngle(p, servoNeutral + servoTrim[p] + pitchDeg * servoDirection[p]);
    writeServoAngle(y, servoNeutral + servoTrim[y] + yawDeg   * servoDirection[y]);
  }
}

void parkFins() { targetYaw = targetPitch = yaw = pitch = 0; applyFins(0, 0); }

void wiggleServo(int i) {
  if (i < 0 || i >= SERVO_COUNT) return;
  const float steps[3] = { -30.0f, 30.0f, 0.0f };
  for (int k = 0; k < 3; k++) {
    writeServoAngle(i, servoNeutral + servoTrim[i] + steps[k] * servoDirection[i]);
    delay(400);
  }
}
void wiggleAll() { for (int i = 0; i < SERVO_COUNT; i++) wiggleServo(i); parkFins(); }

// ==================== WIFI / UDP ====================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  if (STATIC_IP != IPAddress(0, 0, 0, 0)) WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS_SERVER);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  DBG.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); DBG.print("."); }
  DBG.printf("\nFish node IP: %s  (UDP %d)\n", WiFi.localIP().toString().c_str(), UDP_PORT);
}

void sendAck() {
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.printf("ACK %.1f %.1f %lu\n", yaw, pitch, (unsigned long)packetsRx);
  udp.endPacket();
}

void handlePacket() {
  int len = udp.parsePacket();
  if (len <= 0) return;
  char buf[96];
  int n = udp.read(buf, sizeof(buf) - 1);
  if (n <= 0) return;
  buf[n] = '\0';
  peerIp = udp.remoteIP(); peerPort = udp.remotePort();
  lastPacketMs = millis();
  packetsRx++;

  float y, p;
  if (strncmp(buf, "PING", 4) == 0) { sendAck(); return; }
  if (strncmp(buf, "HOME", 4) == 0) { targetYaw = targetPitch = 0; onboardActive = false; sendAck(); return; }
  if (strncmp(buf, "LAUNCH", 6) == 0) { onboardActive = missionSet; sendAck(); return; }
  if (strncmp(buf, "MISSION,", 8) == 0) {
    if (sscanf(buf + 8, "%f,%f,%f,%f", &missionSpeed, &missionX, &missionY, &missionZ) == 4) missionSet = true;
    sendAck(); return;
  }
  if (sscanf(buf, "%f,%f", &y, &p) == 2) {
    targetYaw = clampf(y, -FIN_MAX, FIN_MAX);
    targetPitch = clampf(p, -FIN_MAX, FIN_MAX);
    onboardActive = false;               // the laptop is steering
    sendAck();
  }
}

// ==================== ONBOARD GUIDANCE (stub) ====================
// Called at 50 Hz while onboardActive. Fill in with your IMU dead-reckoning
// towards (missionX, missionY, missionZ) at missionSpeed if you run the ground
// station with --guidance fish. Until then it holds the fins neutral.
void onboardGuidance() {
  targetYaw = 0; targetPitch = 0;
}

void tickFins() {
  unsigned long now = millis();
  if (now - lastTickMs < 20) return;
  lastTickMs = now;
  bool linkAlive = (now - lastPacketMs) < LINK_TIMEOUT_MS;
  if (!linkAlive) { targetYaw = targetPitch = 0; onboardActive = false; if (linkWasAlive) DBG.println("link lost -> fins neutral"); }
  linkWasAlive = linkAlive;
  if (onboardActive) onboardGuidance();
  yaw   += clampf(targetYaw   - yaw,   -SLEW_DEG_PER_TICK, SLEW_DEG_PER_TICK);
  pitch += clampf(targetPitch - pitch, -SLEW_DEG_PER_TICK, SLEW_DEG_PER_TICK);
  applyFins(yaw, pitch);
}

// ==================== IMU STREAM ====================
void tickImu() {
  unsigned long now = millis();
  if (!imuOk || peerPort == 0) return;
  if (now - lastImuMs < (1000 / IMU_RATE_HZ)) return;
  if (!imu.dataReady()) return;
  lastImuMs = now;
  imu.getAGMT();
  // accX() is in mg, gyrX() in deg/s (SparkFun library)
  float ax = imu.accX() * 9.80665f / 1000.0f, ay = imu.accY() * 9.80665f / 1000.0f, az = imu.accZ() * 9.80665f / 1000.0f;
  float gx = imu.gyrX(), gy = imu.gyrY(), gz = imu.gyrZ();
  udp.beginPacket(peerIp, peerPort);
  udp.printf("IMU,%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n", now, ax, ay, az, gx, gy, gz);
  udp.endPacket();
  if (now - lastBattMs > 2000) {
    lastBattMs = now;
    float v = 0.0f;
    if (BATT_PIN >= 0) v = analogReadMilliVolts(BATT_PIN) / 1000.0f * BATT_DIVIDER;
    udp.beginPacket(peerIp, peerPort);
    udp.printf("BATT,%.2f\n", v);
    udp.endPacket();
  }
}

// ==================== SERIAL TUNING (same keys as pid_fins.ino) ====================
void handleSerial() {
  if (!DBG.available()) return;
  char c = DBG.read();
  if (c == 'z') { parkFins(); DBG.println("fins neutral"); return; }
  if (c == 'm') { FIN_MAX = DBG.parseFloat(); DBG.print("FIN_MAX = "); DBG.println(FIN_MAX, 1); return; }
  if (c == 'f') { int i = DBG.parseInt(); if (i >= 0 && i < SERVO_COUNT) servoDirection[i] = -servoDirection[i]; return; }
  if (c == 't') { int i = DBG.parseInt(); float v = DBG.parseFloat(); if (i >= 0 && i < SERVO_COUNT) servoTrim[i] = v; return; }
  if (c == 'w') { wiggleServo(DBG.parseInt()); return; }
  if (c == 'a') { wiggleAll(); return; }
  if (c == 'l') { logEnabled = !logEnabled; return; }
  if (c == '?') { DBG.printf("FIN_MAX %.1f yaw %.1f pitch %.1f packets %lu imu %s mission %s\n", FIN_MAX, yaw, pitch, (unsigned long)packetsRx, imuOk ? "ok" : "MISSING", missionSet ? "set" : "none"); return; }
}

// ==================== SETUP / LOOP ====================
void setup() {
#if USE_USB_CDC
  DBG.begin(115200);
#else
  DBG.begin(115200, SERIAL_8N1, 44, 43);
#endif
  delay(200);
  DBG.println("=== Fish node: fins + ICM20948 ===");

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1); ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  for (int i = 0; i < SERVO_COUNT; i++) { servos[i].setPeriodHertz(50); servos[i].attach(SERVO_PINS[i], SERVO_MIN_PULSE, SERVO_MAX_PULSE); }
  parkFins();
  delay(600);
#if SELF_TEST_ON_BOOT
  wiggleAll();
#endif

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  for (int tries = 0; tries < 5 && !imuOk; tries++) {
    imu.begin(Wire, AD0_VAL);
    imuOk = (imu.status == ICM_20948_Stat_Ok);
    if (!imuOk) { DBG.println("ICM20948 not found, retrying"); delay(400); }
  }
  DBG.println(imuOk ? "ICM20948 ok" : "ICM20948 MISSING: fins still work, no IMU stream");

  connectWiFi();
  udp.begin(UDP_PORT);
  lastPacketMs = 0;
  DBG.println("Waiting for the ground station. Type ? for status.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) { connectWiFi(); udp.begin(UDP_PORT); }
  handleSerial();
  handlePacket();
  tickFins();
  tickImu();
  if (logEnabled && millis() - lastPrintMs >= 1000) {
    lastPrintMs = millis();
    DBG.printf("link %s yaw %.1f->%.1f pitch %.1f->%.1f imu %s\n", linkWasAlive ? "ok" : "--", targetYaw, yaw, targetPitch, pitch, imuOk ? "ok" : "--");
  }
}
