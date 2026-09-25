/*
 * ESP32-CAM (AI-Thinker) — MJPEG stream server
 *
 * Streams JPEG frames over Wi-Fi at   http://<cam-ip>:81/stream
 * Single-frame snapshot at            http://<cam-ip>:81/capture
 *
 * Board: "AI Thinker ESP32-CAM"  (Tools -> Board -> ESP32 Arduino)
 * Partition scheme: "Huge APP (3MB No OTA)"
 * Needs no extra libraries — esp_camera / esp_http_server ship with the ESP32 core.
 */

#include "esp_camera.h"
#include "esp_http_server.h"
#include <WiFi.h>

// ---------- USER CONFIG ----------
const char* WIFI_SSID = "DING DONG";
const char* WIFI_PASS = "12345679";

// Static IP so the address never changes when this board reconnects (e.g. to a
// phone hotspot, which has no DHCP-reservation UI). Pick an address outside
// what your router/hotspot hands out via DHCP, or one you've confirmed is free.
// Set STATIC_IP to all zeros to fall back to normal DHCP.
IPAddress STATIC_IP (172, 20, 10, 13);   // top of the /28 range: hotspot DHCP hands out .2 upward, so this won't collide
IPAddress GATEWAY   (172, 20, 10, 1);
IPAddress SUBNET     (255, 255, 255, 240);
IPAddress DNS_SERVER (172, 20, 10, 1);

// Lower resolution = higher FPS, lower latency. QVGA (320x240) is plenty for a bright-spot tracker.
#define STREAM_FRAMESIZE   FRAMESIZE_QVGA   // FRAMESIZE_QVGA / FRAMESIZE_VGA / FRAMESIZE_HVGA
#define STREAM_JPEG_QUALITY 20              // 0-63, lower = better quality, bigger frames
#define STREAM_PORT         81
// ---------------------------------

// AI-Thinker ESP32-CAM pin map
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_LED_PIN      4

static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static const char* STREAM_BOUNDARY     = "\r\n--frame\r\n";
static const char* STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

// ---------------- /stream : endless MJPEG ----------------
static esp_err_t stream_handler(httpd_req_t* req) {
  camera_fb_t* fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[64];

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
      break;
    }
    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;   // client disconnected
  }
  return res;
}

// ---------------- /capture : single JPEG ----------------
static esp_err_t capture_handler(httpd_req_t* req) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

// ---------------- / : tiny status page ----------------
static esp_err_t index_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_sendstr(req,
    "<html><body style='font-family:sans-serif'>"
    "<h3>ESP32-CAM light tracker</h3>"
    "<p>Stream: <a href='/stream'>/stream</a> &nbsp; Snapshot: <a href='/capture'>/capture</a></p>"
    "</body></html>");
}

void start_server() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = STREAM_PORT;
  config.ctrl_port   = STREAM_PORT + 1000;
  config.max_open_sockets = 3;

  httpd_uri_t index_uri   = { "/",        HTTP_GET, index_handler,   NULL };
  httpd_uri_t stream_uri  = { "/stream",  HTTP_GET, stream_handler,  NULL };
  httpd_uri_t capture_uri = { "/capture", HTTP_GET, capture_handler, NULL };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    httpd_register_uri_handler(stream_httpd, &capture_uri);
  }
}

bool init_camera() {
  camera_config_t c;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 10000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = STREAM_FRAMESIZE;
  c.jpeg_quality = STREAM_JPEG_QUALITY;
  c.fb_count     = psramFound() ? 2 : 1;
  c.fb_location  = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;   // always serve the freshest frame (low latency)

  if (esp_camera_init(&c) != ESP_OK) return false;

  // Sensor tuning for light tracking: fixed-ish exposure so the bright spot
  // doesn't get auto-exposed away. Tweak on the sensor object.
  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, STREAM_FRAMESIZE);
  s->set_brightness(s, 1);
  s->set_contrast(s, 1);
  s->set_saturation(s, 0);       // we only care about luminance
  s->set_whitebal(s, 1);
  s->set_exposure_ctrl(s, 0);     // 1 = auto exposure on. Set to 0 + set_aec_value() for manual.
  s->set_aec2(s, 0);
  s->set_ae_level(s, 1);         // bias exposure darker so the light source stands out
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 0);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);
  return true;
}

void setup() {
  Serial.begin(115200);
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);   // keep the flash LED off — it would be our own "brightest spot"

  if (!init_camera()) {
    Serial.println("Camera init failed — check board/PSRAM/partition settings");
    while (true) delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);               // keeps latency low
  if (STATIC_IP != IPAddress(0, 0, 0, 0)) {
    if (!WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS_SERVER)) {
      Serial.println("Static IP config failed, falling back to DHCP");
    }
  }
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println();

  start_server();
  Serial.printf("Stream ready:  http://%s:%d/stream\n", WiFi.localIP().toString().c_str(), STREAM_PORT);
}

void loop() {
  // Everything runs inside the HTTP server task. Reconnect Wi-Fi if it drops.
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost, reconnecting...");
    WiFi.reconnect();
    delay(2000);
  }
  delay(500);
}
