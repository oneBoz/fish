/*
  ESP32 (classic) : Maelstrom fin node -- 4 fin servos, commanded over UART by the head node

  No Wi-Fi here. The head node (ESP32-S3, cam_imu_node.ino) talks to the laptop and sends us:

      "F,yaw,pitch\n"   every 20 ms   fin deflection in degrees from neutral (+/- FIN_MAX)  -- also the heartbeat
      "W\n"                           wiggle all four servos (self-test)
      "C:<cmd>\n"                     a tuning command exactly as you would type it on our serial monitor

  We reply every 20 ms with   "A,yaw,pitch,n\n"   the actual (slew-limited) angles and a packet count,
  so the head node can report real fin angles to the laptop and notice if we drop out.

  UART: Serial2, RX = GPIO 16 (<- head TX 21), TX = GPIO 17 (-> head RX 20), 115200, GROUND SHARED.
  Fins (4-fin cross), SERVO INDEX -> GPIO:   0 -> 26,  1 -> 27,  2 -> 14,  3 -> 12
      PITCH pair = servos 0 & 2 (GPIO 26, 14)   YAW pair = servos 1 & 3 (GPIO 27, 12)
  Note: GPIO 12 is a strapping pin on the classic ESP32. A servo signal lead is driven by us, so it is
  normally fine, but if the board fails to boot with that servo plugged in, move it to GPIO 13 or 4.
  If no "F," line arrives for LINK_TIMEOUT_MS the fins return to neutral (same as before).

  POWER:  servo rail from a 5 V supply able to give >= 2 A, ground common with this board AND the head node.
  Serial monitor 115200 (USB). Library: ESP32Servo (Kevin Harrington).

  LIVE TUNING over serial (or forwarded from the head node as C:<cmd>):
      t2 3.5   trim servo 2 by +3.5 deg       f1     flip servo 1 direction
      m25      set max deflection to 25 deg    z      fins to neutral
      w2       wiggle servo 2 (index 0-3)      a      wiggle all four (same as the boot self-test)
      l        toggle the log on/off           ?      print settings

  LOG (every 500 ms):
      link ok  yaw 12.5->12.0  pitch -4.0->-3.0 | S0(g26) 102.0 1588us | S1(g27) ... (NOT-ATTACHED if a servo failed to attach)
*/

#include <ESP32Servo.h>

#define SERVO_COUNT 4
const int SERVO_PINS[SERVO_COUNT] = { 26, 27, 14, 12 };
const int PITCH_FINS[2] = { 0, 2 };
const int YAW_FINS[2]   = { 1, 3 };
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2400

#define UART_RX 16
#define UART_TX 17
const unsigned long UART_BAUD = 115200;

float servoNeutral = 90.0f;
float servoDirection[SERVO_COUNT] = { 1, 1, 1, 1 };
float servoTrim[SERVO_COUNT]      = { 0, 0, 0, 0 };
float FIN_MAX = 30.0f;
#define SELF_TEST_ON_BOOT 1
const float SLEW_DEG_PER_TICK = 3.0f;
const unsigned long LINK_TIMEOUT_MS = 1000;

Servo servos[SERVO_COUNT];
float targetYaw = 0, targetPitch = 0, yaw = 0, pitch = 0;
unsigned long lastPacketMs = 0, lastTickMs = 0, lastPrintMs = 0;
unsigned long packetsRx = 0;
bool linkWasAlive = false, logEnabled = true;
float servoCmdDeg[SERVO_COUNT] = { 0, 0, 0, 0 }; int servoCmdUs[SERVO_COUNT] = { 0, 0, 0, 0 };
char uartBuf[96]; int uartLen = 0;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void writeServoAngle(int i, float angleDeg) {
  angleDeg = clampf(angleDeg, 0.0f, 180.0f);
  int us = (int)(SERVO_MIN_PULSE + (angleDeg / 180.0f) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
  servos[i].writeMicroseconds(us);
  servoCmdDeg[i] = angleDeg; servoCmdUs[i] = us;
}

void printServos() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.print(" | S"); Serial.print(i); Serial.print("(g"); Serial.print(SERVO_PINS[i]); Serial.print(") ");
    Serial.print(servoCmdDeg[i], 1); Serial.print(" "); Serial.print(servoCmdUs[i]); Serial.print("us");
    if (!servos[i].attached()) Serial.print(" NOT-ATTACHED");
  }
  Serial.println();
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
  Serial.print("wiggle S"); Serial.print(i); Serial.print(" (GPIO "); Serial.print(SERVO_PINS[i]);
  Serial.print(") attached="); Serial.println(servos[i].attached() ? "yes" : "NO");
  const float steps[3] = { -30.0f, 30.0f, 0.0f };
  for (int k = 0; k < 3; k++) {
    writeServoAngle(i, servoNeutral + servoTrim[i] + steps[k] * servoDirection[i]);
    Serial.print("   -> "); Serial.print(servoCmdDeg[i], 1); Serial.print(" deg  "); Serial.print(servoCmdUs[i]); Serial.println(" us");
    delay(500);
  }
}

void wiggleAll() {
  Serial.println("self-test: wiggling each servo in turn");
  for (int i = 0; i < SERVO_COUNT; i++) wiggleServo(i);
  parkFins();
  Serial.println("self-test done, fins neutral");
}

void printSettings() {
  Serial.print("FIN_MAX "); Serial.print(FIN_MAX, 1); Serial.print("  yaw "); Serial.print(yaw, 1); Serial.print("  pitch "); Serial.print(pitch, 1);
  Serial.print("  packets "); Serial.println(packetsRx);
  for (int i = 0; i < SERVO_COUNT; i++) { Serial.print("  servo "); Serial.print(i); Serial.print(" GPIO "); Serial.print(SERVO_PINS[i]); Serial.print(" dir "); Serial.print(servoDirection[i], 0); Serial.print(" trim "); Serial.println(servoTrim[i], 1); }
}

// one tuning command, from our own monitor or forwarded by the head node
void handleCommand(const char* s) {
  char c = s[0];
  if (c == '?') { printSettings(); return; }
  if (c == 'z') { parkFins(); Serial.println("fins neutral"); return; }
  if (c == 'm') { FIN_MAX = atof(s + 1); Serial.print("FIN_MAX = "); Serial.println(FIN_MAX, 1); return; }
  if (c == 'f') { int i = atoi(s + 1); if (i >= 0 && i < SERVO_COUNT) { servoDirection[i] = -servoDirection[i]; Serial.print("servo "); Serial.print(i); Serial.print(" dir "); Serial.println(servoDirection[i], 0); } return; }
  if (c == 't') { int i = 0; float v = 0; if (sscanf(s + 1, "%d %f", &i, &v) == 2 && i >= 0 && i < SERVO_COUNT) { servoTrim[i] = v; Serial.print("servo "); Serial.print(i); Serial.print(" trim "); Serial.println(v, 1); } return; }
  if (c == 'w') { wiggleServo(atoi(s + 1)); return; }
  if (c == 'a') { wiggleAll(); return; }
  if (c == 'l') { logEnabled = !logEnabled; Serial.print("log "); Serial.println(logEnabled ? "on" : "off"); return; }
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n'); line.trim();
  if (line.length()) handleCommand(line.c_str());
}

// ---- UART from the head node ----
void uartHandleLine(const char* s) {
  if (strncmp(s, "F,", 2) == 0) {
    float y, p;
    if (sscanf(s + 2, "%f,%f", &y, &p) == 2) { targetYaw = clampf(y, -FIN_MAX, FIN_MAX); targetPitch = clampf(p, -FIN_MAX, FIN_MAX); lastPacketMs = millis(); packetsRx++; }
  } else if (strncmp(s, "W", 1) == 0 && strlen(s) == 1) {
    wiggleAll();
  } else if (strncmp(s, "C:", 2) == 0) {
    Serial.print("head node: "); Serial.println(s + 2); handleCommand(s + 2);
  }
}

void uartPoll() {
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n' || c == '\r') { if (uartLen) { uartBuf[uartLen] = 0; uartHandleLine(uartBuf); uartLen = 0; } }
    else if (uartLen < (int)sizeof(uartBuf) - 1) uartBuf[uartLen++] = c;
    else uartLen = 0;
  }
}

void tickFins() {
  unsigned long now = millis();
  if (now - lastTickMs < 20) return;
  lastTickMs = now;
  bool linkAlive = (now - lastPacketMs) < LINK_TIMEOUT_MS;
  if (!linkAlive) { targetYaw = targetPitch = 0; if (linkWasAlive) Serial.println("head node link lost -> fins to neutral"); }
  linkWasAlive = linkAlive;
  yaw   += clampf(targetYaw   - yaw,   -SLEW_DEG_PER_TICK, SLEW_DEG_PER_TICK);
  pitch += clampf(targetPitch - pitch, -SLEW_DEG_PER_TICK, SLEW_DEG_PER_TICK);
  applyFins(yaw, pitch);
  Serial2.printf("A,%.1f,%.1f,%lu\n", yaw, pitch, packetsRx);       // actual angles back to the head node
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("=== Maelstrom fin node (UART) ===");
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1); ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  for (int i = 0; i < SERVO_COUNT; i++) {
    servos[i].setPeriodHertz(50);
    int ch = servos[i].attach(SERVO_PINS[i], SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    Serial.print("servo "); Serial.print(i); Serial.print(" on GPIO "); Serial.print(SERVO_PINS[i]); Serial.print(" -> attach "); Serial.println(ch ? "ok" : "FAILED (no free timer/channel?)");
  }
  parkFins();
  delay(600);
#if SELF_TEST_ON_BOOT
  wiggleAll();
#endif
  lastPacketMs = 0;
  Serial.println("Waiting for the head node on UART (RX 16 / TX 17). Type ? for settings.");
}

void loop() {
  handleSerial();
  uartPoll();
  tickFins();
  if (logEnabled && millis() - lastPrintMs >= 500) {
    lastPrintMs = millis();
    Serial.print("link "); Serial.print(linkWasAlive ? "ok" : "--");
    Serial.print("  yaw "); Serial.print(targetYaw, 1); Serial.print("->"); Serial.print(yaw, 1);
    Serial.print("  pitch "); Serial.print(targetPitch, 1); Serial.print("->"); Serial.print(pitch, 1);
    printServos();
  }
}
