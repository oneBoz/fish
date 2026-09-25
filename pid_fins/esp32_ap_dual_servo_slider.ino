/*
  ESP32-WROOM Access Point Dual Servo Slider Controller
  - Creates its own Wi-Fi AP (no router required)
  - Serves a webpage with two sliders (0-180 deg)
  - Controls two MG90S servos via ESP32Servo library

  Library required: ESP32Servo (install via Library Manager)

  Wiring:
    Servo 1 signal -> GPIO 4
    Servo 2 signal -> GPIO 16
    Servo V+ (red)   -> External 5V supply
    Servo GND (brown)-> External 5V supply GND, AND common with ESP32 GND
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// ---------- Wi-Fi AP credentials ----------
const char* AP_SSID     = "ESP32-ServoCtrl";
const char* AP_PASSWORD = "servo1234";   // min 8 chars, or set to "" for open network

// ---------- Servo setup ----------
#define SERVO1_PIN 4
#define SERVO2_PIN 16

Servo servo1;
Servo servo2;

int servo1Angle = 55;
int servo2Angle = 90;

WebServer server(80);

// ---------- HTML Page (served from PROGMEM) ----------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 Servo Control</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      text-align: center;
      background: #1e1e2f;
      color: #f0f0f0;
      margin: 0;
      padding: 20px;
    }
    h2 { margin-bottom: 30px; }
    .slider-block {
      margin: 30px auto;
      width: 85%;
      max-width: 400px;
      background: #2a2a40;
      border-radius: 12px;
      padding: 20px;
      box-shadow: 0 4px 10px rgba(0,0,0,0.4);
    }
    input[type=range] {
      width: 100%;
      height: 30px;
    }
    .value {
      font-size: 22px;
      font-weight: bold;
      color: #4dd0e1;
      margin-top: 10px;
    }
    label { font-size: 18px; }
  </style>
</head>
<body>
  <h2>ESP32 Dual Servo Controller</h2>

  <div class="slider-block">
    <label for="s1">Servo Top</label><br>
    <input type="range" min="5" max="105" value="55" id="s1" oninput="sendVal(1,this.value)">
    <div class="value" id="s1val">55&deg;</div>
  </div>

  <div class="slider-block">
    <label for="s2">Servo Bottom</label><br>
    <input type="range" min="0" max="180" value="90" id="s2" oninput="sendVal(2,this.value)">
    <div class="value" id="s2val">90&deg;</div>
  </div>

  <script>
    function sendVal(servoNum, val) {
      document.getElementById('s' + servoNum + 'val').innerHTML = val + '&deg;';
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/setServo?servo=" + servoNum + "&angle=" + val, true);
      xhr.send();
    }
  </script>
</body>
</html>
)HTML";

// ---------- Route handlers ----------
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleSetServo() {
  if (server.hasArg("servo") && server.hasArg("angle")) {
    int servoNum = server.arg("servo").toInt();
    int angle = server.arg("angle").toInt();
    angle = constrain(angle, 0, 180);

    if (servoNum == 1) {
      servo1Angle = angle;
      servo1.write(servo1Angle);
      Serial.print("Servo1 set to: ");
      Serial.println(servo1Angle);
    } else if (servoNum == 2) {
      servo2Angle = angle;
      servo2.write(servo2Angle);
      Serial.print("Servo2 set to: ");
      Serial.println(servo2Angle);
    }
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Missing arguments");
  }
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void setup() {
  Serial.begin(115200);

  // Allow allocation of all timers for ESP32Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo1.setPeriodHertz(50);       // Standard 50Hz servo
  servo2.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, 500, 2400); // MG90S pulse range ~500-2400us
  servo2.attach(SERVO2_PIN, 500, 2400);

  servo1.write(servo1Angle);
  servo2.write(servo2Angle);

  // Start Access Point
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  IPAddress apIP = WiFi.softAPIP();
  Serial.print("AP started. Connect to SSID: ");
  Serial.println(AP_SSID);
  Serial.print("Then open browser at: http://");
  Serial.println(apIP);

  server.on("/", handleRoot);
  server.on("/setServo", handleSetServo);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
