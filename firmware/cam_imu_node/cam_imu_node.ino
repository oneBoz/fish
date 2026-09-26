/*
  ESP32-S3 : Maelstrom "head" node -- Wi-Fi link, camera gimbal, ICM20948, fin board over UART

  This is pid_fins.ino split across two boards. This board keeps everything the laptop talks to:

      laptop <-Wi-Fi/UDP 4210->  THIS BOARD (ESP1)  <-UART 115200->  fin board (ESP2, fin_node_uart.ino)
                                  camera pan/tilt servos                 4 fin servos
                                  ICM20948 (I2C)

  UDP protocol (unchanged from pid_fins.ino / fish_node.ino):
      laptop -> "YAW,PITCH,PAN,TILT\n"  fins deg from neutral (+/-FIN_MAX), camera absolute angles
                "YAW,PITCH\n"           fins only            "HOME\n"  fins neutral + camera centre
                "PING\n"                keep-alive           "MISSION,speed,x,y,z\n"  "LAUNCH\n"  (stored, stubs)
      this   -> "ACK yaw pitch pan tilt n finlink\n"   yaw/pitch = the fin board's ACTUAL angles when its
                                                       link is up (finlink = 1), else the forwarded targets (finlink = 0)
                "IMU,ms,ax,ay,az,gx,gy,gz\n"           accel m/s^2, gyro deg/s, sensor axes, IMU_RATE_HZ
                "BATT,volts\n"                         every 2 s (0.0 if BATT_PIN is not wired)

  UART protocol to the fin board (Serial1, TX GPIO 21 -> ESP2 RX 16, RX GPIO 20 <- ESP2 TX 17, common GND):
      this -> fin  "F,yaw,pitch\n"   every 20 ms (doubles as the heartbeat)     "W\n"  wiggle all fins (self-test)
                   "C:<cmd>\n"       forward a serial tuning command (t2 3.5 / f1 / m25 / w2 / z / ? / l)
      fin -> this  "A,yaw,pitch,n\n" every 20 ms: actual (slewed) fin angles + packet count

  Camera gimbal: PAN = GPIO 18 (5..175, centre 90), TILT = GPIO 17 (35..75, centre 55).
  IMU: ICM20948 on I2C, SDA GPIO 6, SCL GPIO 5, INT GPIO 4 (data-ready, optional; polled if unused).
  If no laptop packet for LINK_TIMEOUT_MS: fins to neutral (forwarded) and camera to centre.

  POWER:  servo rail from a 5 V supply able to give >= 2 A, grounds common with BOTH boards.
  BOARD:  ESP32-S3 with two USB-C ports. Use the BOTTOM (UART) port, serial monitor 115200.
  Libraries: ESP32Servo (Kevin Harrington), SparkFun 9DoF IMU Breakout - ICM 20948 (SparkFun).

  LIVE TUNING over serial -- type and press enter:
      c120 60   point the camera (pan, tilt)      w4 / w5   wiggle pan / tilt     z   fins neutral, camera centre
      a         self-test: camera here, fins on the fin board
      t2 3.5 / f1 / m25 / w2 / ?   forwarded to the fin board (its own monitor shows the result)
      l         toggle this board's log on/off

  LOG (every 500 ms):
      link ok  fins 12.5,-4.0 -> 12.0,-3.0 (finlink ok)  cam 96.0->95.0 52.0->52.0  imu ok 50 Hz | PAN(g18) 95.0 1508us | TILT(g17) 52.0 1049us
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

// camera gimbal
#define CAM_PAN_PIN  18
#define CAM_TILT_PIN 17
const float PAN_MIN = 5.0f,   PAN_MAX = 175.0f, PAN_HOME = 90.0f;
const float TILT_MIN = 35.0f, TILT_MAX = 75.0f, TILT_HOME = 55.0f;
const float CAM_SLEW_DEG_PER_TICK = 4.0f;
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2400

// UART to the fin board
#define FIN_UART_RX 20        // <- ESP2 TX (GPIO 17)
#define FIN_UART_TX 21        // -> ESP2 RX (GPIO 16)
const unsigned long FIN_UART_BAUD = 115200;
const unsigned long FINLINK_TIMEOUT_MS = 500;   // no "A," line for this long -> finlink = 0

// IMU
#define I2C_SDA 6
#define I2C_SCL 5
#define IMU_INT_PIN 4
#define AD0_VAL 0                       // ICM20948 AD0 low -> address 0x68 (1 -> 0x69)
const int IMU_RATE_HZ = 50;
const int BATT_PIN = -1;                // ADC pin through a divider, -1 = not wired
const float BATT_DIVIDER = 2.0f;

float FIN_MAX = 30.0f;                  // fin deflection limit forwarded to the fin board
const float SLEW_DEG_PER_TICK = 3.0f;   // mirrors the fin board's slew, for the ACK when its link is down
const unsigned long LINK_TIMEOUT_MS = 1000;
#define SELF_TEST_ON_BOOT 1

// ==================== STATE ====================
WiFiUDP udp;
Servo camPan, camTilt;
ICM_20948_I2C imu;
bool imuOk = false;

float targetYaw = 0, targetPitch = 0;            // fins, as commanded by the laptop
float fwdYaw = 0, fwdPitch = 0;                  // slewed copy (used in the ACK when the fin board is silent)
float finYawAct = 0, finPitchAct = 0;            // reported by the fin board
unsigned long finCount = 0, lastFinRxMs = 0;
float targetPan = PAN_HOME, targetTilt = TILT_HOME, pan = PAN_HOME, tilt = TILT_HOME;
unsigned long lastPacketMs = 0, lastTickMs = 0, lastPrintMs = 0, lastImuMs = 0, lastBattMs = 0;
uint32_t packetsRx = 0;
bool linkWasAlive = false, finLinkWasAlive = false, logEnabled = true;
IPAddress peerIp; uint16_t peerPort = 0;         // whoever talked to us last gets the IMU stream
float camCmdDeg[2] = { PAN_HOME, TILT_HOME }; int camCmdUs[2] = { 0, 0 };
float missionSpeed = 0, missionX = 0, missionY = 0, missionZ = 0; bool missionSet = false;
char uartBuf[96]; int uartLen = 0;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static int degToUs(float a) { a = clampf(a, 0, 180); return (int)(SERVO_MIN_PULSE + (a / 180.0f) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)); }

// ==================== CAMERA ====================
void writeCam(int which, float deg) {
  deg = (which == 0) ? clampf(deg, PAN_MIN, PAN_MAX) : clampf(deg, TILT_MIN, TILT_MAX);
  int us = degToUs(deg);
  (which == 0 ? camPan : camTilt).writeMicroseconds(us);
  camCmdDeg[which] = deg; camCmdUs[which] = us;
}
void applyCam(float p, float t) { writeCam(0, p); writeCam(1, t); }
void parkCam() { targetPan = pan = PAN_HOME; targetTilt = tilt = TILT_HOME; applyCam(pan, tilt); }

void wiggleCam(int which) {
  const bool isPan = (which == 0);
  DBG.print("wiggle "); DBG.print(isPan ? "PAN (GPIO 18)" : "TILT (GPIO 17)");
  DBG.print(" attached="); DBG.println((isPan ? camPan : camTilt).attached() ? "yes" : "NO");
  const float steps[3] = { isPan ? PAN_MIN + 25.0f : TILT_MIN, isPan ? PAN_MAX - 25.0f : TILT_MAX, isPan ? PAN_HOME : TILT_HOME };
  for (int k = 0; k < 3; k++) {
    writeCam(which, steps[k]);
    DBG.print("   -> "); DBG.print(camCmdDeg[which], 1); DBG.print(" deg  "); DBG.print(camCmdUs[which]); DBG.println(" us");
    delay(500);
  }
  if (isPan) pan = targetPan = PAN_HOME; else tilt = targetTilt = TILT_HOME;
}

// ==================== FIN BOARD (UART) ====================
void finSend(const char* line) { Serial1.print(line); Serial1.print('\n'); }

void finSendTargets() {
  char b[40]; snprintf(b, sizeof(b), "F,%.1f,%.1f", targetYaw, targetPitch); finSend(b);
}

void finHandleLine(const char* s) {
  if (strncmp(s, "A,", 2) == 0) {
    float y, p; unsigned long n;
    if (sscanf(s + 2, "%f,%f,%lu", &y, &p, &n) == 3) { finYawAct = y; finPitchAct = p; finCount = n; lastFinRxMs = millis(); }
  }
}

void finPoll() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') { if (uartLen) { uartBuf[uartLen] = 0; finHandleLine(uartBuf); uartLen = 0; } }
    else if (uartLen < (int)sizeof(uartBuf) - 1) uartBuf[uartLen++] = c;
    else uartLen = 0;
  }
}

bool finLinkAlive() { return lastFinRxMs != 0 && (millis() - lastFinRxMs) < FINLINK_TIMEOUT_MS; }

void selfTest() {
  DBG.println("self-test: fins on the fin board, then camera pan and tilt");
  finSend("W");                       // fin board wiggles its four servos (takes ~6 s)
  wiggleCam(0); wiggleCam(1);
  parkCam();
  DBG.println("self-test: camera done (watch the fin board's monitor for the fins)");
}

// ==================== WIFI / UDP ====================
void connectWiFi() {
  WiFi.mode(WIFI_STA); WiFi.setSleep(false);
  if (STATIC_IP != IPAddress(0, 0, 0, 0)) WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS_SERVER);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  DBG.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); DBG.print("."); }
  DBG.printf("\nHead node IP: %s  (UDP %d)\n", WiFi.localIP().toString().c_str(), UDP_PORT);
}

void sendAck() {
  bool fl = finLinkAlive();
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.printf("ACK %.1f %.1f %.1f %.1f %lu %d\n", fl ? finYawAct : fwdYaw, fl ? finPitchAct : fwdPitch, pan, tilt, (unsigned long)packetsRx, fl ? 1 : 0);
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

  float y, p, cp, ct;
  int got = sscanf(buf, "%f,%f,%f,%f", &y, &p, &cp, &ct);
  if (got >= 2) {
    targetYaw = clampf(y, -FIN_MAX, FIN_MAX); targetPitch = clampf(p, -FIN_MAX, FIN_MAX);
    if (got == 4) { targetPan = clampf(cp, PAN_MIN, PAN_MAX); targetTilt = clampf(ct, TILT_MIN, TILT_MAX); }
    lastPacketMs = millis(); packetsRx++; sendAck(); return;
  }
  if (strncmp(buf, "HOME", 4) == 0) { targetYaw = targetPitch = 0; targetPan = PAN_HOME; targetTilt = TILT_HOME; lastPacketMs = millis(); packetsRx++; sendAck(); return; }
  if (strncmp(buf, "PING", 4) == 0) { lastPacketMs = millis(); sendAck(); return; }
  if (strncmp(buf, "LAUNCH", 6) == 0) { sendAck(); return; }
  if (strncmp(buf, "MISSION,", 8) == 0) {
    if (sscanf(buf + 8, "%f,%f,%f,%f", &missionSpeed, &missionX, &missionY, &missionZ) == 4) missionSet = true;
    sendAck(); return;
  }
}

// ==================== TICK ====================
void tick() {
  unsigned long now = millis();
  if (now - lastTickMs < 20) return;
  lastTickMs = now;
  bool linkAlive = (now - lastPacketMs) < LINK_TIMEOUT_MS;
  if (!linkAlive) {
    targetYaw = targetPitch = 0; targetPan = PAN_HOME; targetTilt = TILT_HOME;
    if (linkWasAlive) DBG.println("laptop link lost -> fins neutral, camera centred");
  }
  linkWasAlive = linkAlive;
  bool fl = finLinkAlive();
  if (fl != finLinkWasAlive) { DBG.println(fl ? "fin board link ok" : "fin board link LOST (no A, lines on the UART)"); finLinkWasAlive = fl; }

  fwdYaw   += clampf(targetYaw   - fwdYaw,   -SLEW_DEG_PER_TICK, SLEW_DEG_PER_TICK);
  fwdPitch += clampf(targetPitch - fwdPitch, -SLEW_DEG_PER_TICK, SLEW_DEG_PER_TICK);
  finSendTargets();                                          // 50 Hz, also the fin board's heartbeat

  pan  += clampf(targetPan  - pan,  -CAM_SLEW_DEG_PER_TICK, CAM_SLEW_DEG_PER_TICK);
  tilt += clampf(targetTilt - tilt, -CAM_SLEW_DEG_PER_TICK, CAM_SLEW_DEG_PER_TICK);
  applyCam(pan, tilt);
}

// ==================== IMU STREAM ====================
void tickImu() {
  unsigned long now = millis();
  if (!imuOk || peerPort == 0) return;
  if (now - lastImuMs < (1000 / IMU_RATE_HZ)) return;
  if (!imu.dataReady()) return;
  lastImuMs = now;
  imu.getAGMT();
  float ax = imu.accX() * 9.80665f / 1000.0f, ay = imu.accY() * 9.80665f / 1000.0f, az = imu.accZ() * 9.80665f / 1000.0f;
  float gx = imu.gyrX(), gy = imu.gyrY(), gz = imu.gyrZ();
  udp.beginPacket(peerIp, peerPort);
  udp.printf("IMU,%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n", now, ax, ay, az, gx, gy, gz);
  udp.endPacket();
  if (now - lastBattMs > 2000) {
    lastBattMs = now;
    float v = (BATT_PIN >= 0) ? analogReadMilliVolts(BATT_PIN) / 1000.0f * BATT_DIVIDER : 0.0f;
    udp.beginPacket(peerIp, peerPort); udp.printf("BATT,%.2f\n", v); udp.endPacket();
  }
}

// ==================== SERIAL ====================
void handleSerial() {
  if (!DBG.available()) return;
  String line = DBG.readStringUntil('\n'); line.trim();
  if (!line.length()) return;
  char c = line[0];
  if (c == 'c') { float cp = 0, ct = 0; sscanf(line.c_str() + 1, "%f %f", &cp, &ct); targetPan = pan = clampf(cp, PAN_MIN, PAN_MAX); targetTilt = tilt = clampf(ct, TILT_MIN, TILT_MAX); applyCam(pan, tilt);
    DBG.print("camera pan "); DBG.print(pan, 1); DBG.print(" tilt "); DBG.println(tilt, 1); return; }
  if (line == "w4") { wiggleCam(0); return; }
  if (line == "w5") { wiggleCam(1); return; }
  if (c == 'z') { targetYaw = targetPitch = 0; parkCam(); finSend("C:z"); DBG.println("fins neutral, camera centred"); return; }
  if (c == 'a') { selfTest(); return; }
  if (c == 'l') { logEnabled = !logEnabled; DBG.print("log "); DBG.println(logEnabled ? "on" : "off"); return; }
  if (c == 'm') { FIN_MAX = line.substring(1).toFloat(); DBG.print("FIN_MAX = "); DBG.println(FIN_MAX, 1); }   // also forwarded below
  if (c == 't' || c == 'f' || c == 'm' || c == 'w' || c == '?') { String fwd = "C:" + line; finSend(fwd.c_str()); DBG.print("forwarded to the fin board: "); DBG.println(line); if (c == '?') { DBG.print("  here: pan "); DBG.print(pan, 1); DBG.print(" tilt "); DBG.print(tilt, 1); DBG.print(" imu "); DBG.print(imuOk ? "ok" : "MISSING"); DBG.print(" finlink "); DBG.println(finLinkAlive() ? "ok" : "lost"); } return; }
}

// ==================== SETUP / LOOP ====================
void setup() {
#if USE_USB_CDC
  DBG.begin(115200);
#else
  DBG.begin(115200, SERIAL_8N1, 44, 43);
#endif
  delay(200);
  DBG.println("=== Maelstrom head node: Wi-Fi + camera gimbal + ICM20948 + fin board over UART ===");

  Serial1.begin(FIN_UART_BAUD, SERIAL_8N1, FIN_UART_RX, FIN_UART_TX);

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1); ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  camPan.setPeriodHertz(50);  DBG.print("camera PAN on GPIO 18 -> attach ");  DBG.println(camPan.attach(CAM_PAN_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE) ? "ok" : "FAILED");
  camTilt.setPeriodHertz(50); DBG.print("camera TILT on GPIO 17 -> attach "); DBG.println(camTilt.attach(CAM_TILT_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE) ? "ok" : "FAILED");
  parkCam();

  pinMode(IMU_INT_PIN, INPUT);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  for (int tries = 0; tries < 5 && !imuOk; tries++) {
    imu.begin(Wire, AD0_VAL);
    imuOk = (imu.status == ICM_20948_Stat_Ok);
    if (!imuOk) { DBG.println("ICM20948 not found, retrying"); delay(400); }
  }
  DBG.println(imuOk ? "ICM20948 ok" : "ICM20948 MISSING: everything else still works, no IMU stream");

  delay(600);
#if SELF_TEST_ON_BOOT
  selfTest();
#endif

  connectWiFi();
  udp.begin(UDP_PORT);
  lastPacketMs = 0;
  DBG.println("Waiting for the ground station. Type ? for settings.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) { connectWiFi(); udp.begin(UDP_PORT); }
  handleSerial();
  handlePacket();
  finPoll();
  tick();
  tickImu();
  if (logEnabled && millis() - lastPrintMs >= 500) {
    lastPrintMs = millis();
    DBG.print("link "); DBG.print(linkWasAlive ? "ok" : "--");
    DBG.print("  fins "); DBG.print(targetYaw, 1); DBG.print(","); DBG.print(targetPitch, 1); DBG.print(" -> "); DBG.print(finYawAct, 1); DBG.print(","); DBG.print(finPitchAct, 1);
    DBG.print(" (finlink "); DBG.print(finLinkAlive() ? "ok" : "LOST"); DBG.print(")");
    DBG.print("  cam "); DBG.print(targetPan, 1); DBG.print("->"); DBG.print(pan, 1); DBG.print(" "); DBG.print(targetTilt, 1); DBG.print("->"); DBG.print(tilt, 1);
    DBG.print("  imu "); DBG.print(imuOk ? "ok" : "--");
    DBG.print(" | PAN(g18) "); DBG.print(camCmdDeg[0], 1); DBG.print(" "); DBG.print(camCmdUs[0]); DBG.print("us");
    DBG.print(" | TILT(g17) "); DBG.print(camCmdDeg[1], 1); DBG.print(" "); DBG.print(camCmdUs[1]); DBG.println("us");
  }
}
