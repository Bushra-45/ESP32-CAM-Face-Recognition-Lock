// CyberLock - ESP32-CAM AI face detection door lock
// Hotspot (AP) + optional router (STA), fail-safe lock, safer pins.
//
// Arduino IDE settings (important for face detection):
//   Board:            "ESP32 Wrover Module"  (or AI Thinker ESP32-CAM)
//   PSRAM:            Enabled
//   Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
//   ESP32 core:       1.0.6 (face detection/recognition was dropped from newer cores)

#include "esp_camera.h"
#include <WiFi.h>

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// ---------------- Pins ----------------
// Moved away from GPIO 2 / 12 (strapping pins) to avoid boot problems.
// GPIO 13 has no boot-time glitch, so it drives the relay. Do NOT use GPIO 16 (PSRAM).
#define Relay 13
#define Green 14
#define Red   15

// ---------------- Relay (active-LOW module) ----------------
// 1 = switch OFF by releasing the pin (input mode). Fixes relay chatter / lock flicker on
//     5 V active-LOW modules, which cannot be switched off properly by the ESP32's 3.3 V HIGH.
// 0 = switch OFF by driving the pin HIGH (plain push-pull).
#define RELAY_OPEN_DRAIN 1

void relayOn() {
  pinMode(Relay, OUTPUT);
  digitalWrite(Relay, LOW);
}

void relayOff() {
#if RELAY_OPEN_DRAIN
  pinMode(Relay, INPUT);
#else
  digitalWrite(Relay, HIGH);
  pinMode(Relay, OUTPUT);
#endif
}

// ---------------- WiFi ----------------
#define USE_ROUTER 1   // 1 = hotspot + try router, 0 = hotspot only

const char* ssid     = "#-#";            // Router WiFi name
const char* password = "Nosign@l00";     // Router WiFi password

const char* ap_ssid     = "CyberLock-CAM";  // Hotspot name
const char* ap_password = "79101103";       // Min 8 chars - CHANGE IT
const int   ap_channel  = 1;
const int   ap_max_conn = 2;                // Fewer clients = smoother face detection

// Web page login (used by auth.h in app_httpd.cpp) - CHANGE BOTH
const char* web_user = "admin";
const char* web_pass = "12345678";

const unsigned long wifiTimeout = 15000;    // router connect timeout (ms)

// ---------------- State ----------------
void startCameraServer();

boolean matchFace = false;            // set to true by app_httpd.cpp on a recognised face
boolean activateRelay = false;
boolean cameraOK = false;
unsigned long prevMillis = 0;
const unsigned long interval = 5000;  // door stays unlocked for 5 s

#define DEBUG_STATS 1                 // print heap/PSRAM every 10 s (helps find freezes)

// Door locked, red LED on
void lockClosed() {
  relayOff();
  digitalWrite(Green, LOW);
  digitalWrite(Red, HIGH);
}

// Camera/system error: keep the door locked, blink red, then reboot.
void failSafe(const char* msg) {
  Serial.println(msg);
  lockClosed();
  for (int i = 0; i < 20; i++) {      // ~10 s of blinking
    digitalWrite(Red, !digitalRead(Red));
    delay(500);
  }
  lockClosed();
  ESP.restart();
}

void startHotspot() {
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1),
                    IPAddress(192, 168, 4, 1),
                    IPAddress(255, 255, 255, 0));
  if (WiFi.softAP(ap_ssid, ap_password, ap_channel, 0, ap_max_conn)) {
    Serial.print("Hotspot: ");
    Serial.print(ap_ssid);
    Serial.print("  http://");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Hotspot failed to start!");
  }
}

bool connectToRouter() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to router");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < wifiTimeout) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Router IP: http://");
    Serial.println(WiFi.localIP());
    return true;
  }
  // Router not available: stop scanning. Background STA scans hop channels and
  // stall the video stream / face detection.
  Serial.println("Router not found - hotspot only");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  return false;
}

void setup() {
  // Safe levels first so the relay never glitches at boot
  relayOff();
  digitalWrite(Green, LOW);
  digitalWrite(Red, HIGH);
  pinMode(Green, OUTPUT);
  pinMode(Red, OUTPUT);
  lockClosed();

  Serial.begin(115200);
  Serial.setDebugOutput(false);   // debug spam slows the stream
  Serial.println();

  if (!psramFound()) {
    Serial.println("WARNING: PSRAM not found. Select 'ESP32 Wrover Module' and enable PSRAM, "
                   "otherwise face detection will freeze.");
  }

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Start directly at QVGA (320x240). Face detection only works at CIF (400x296)
  // or lower, and starting at UXGA wasted PSRAM that the face detector needs.
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count     = psramFound() ? 2 : 1;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 2
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.grab_mode   = CAMERA_GRAB_LATEST;   // always process the newest frame (no lag)
#endif

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Camera init failed with error 0x%x - door stays locked", err);
    failSafe(buf);   // never returns
  }
  cameraOK = true;

  sensor_t* s = esp_camera_sensor_get();
  if (s == NULL) failSafe("Camera sensor not found - door stays locked");
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  s->set_framesize(s, FRAMESIZE_QVGA);

  // ---- WiFi ----
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);          // no background reconnect scans
  WiFi.mode(USE_ROUTER ? WIFI_AP_STA : WIFI_AP);
  startHotspot();
#if USE_ROUTER
  connectToRouter();
#endif
  WiFi.setSleep(false);                  // smoother video

  if (strcmp(web_pass, "ChangeThisWebPass") == 0 ||
      strcmp(ap_password, "ChangeThisHotspotPass") == 0) {
    Serial.println("WARNING: default passwords in use - change them before installing!");
  }

  startCameraServer();
  Serial.println("Camera Ready!");
}

void loop() {
  // Never open the door if the camera is not healthy
  if (!cameraOK) {
    matchFace = false;
    lockClosed();
    return;
  }

  if (matchFace == true && activateRelay == false) {
    activateRelay = true;
    relayOn();
    digitalWrite(Green, HIGH);
    digitalWrite(Red, LOW);
    prevMillis = millis();
  }
  if (activateRelay == true && millis() - prevMillis > interval) {
    activateRelay = false;
    matchFace = false;
    lockClosed();
  }

#if DEBUG_STATS
  static unsigned long lastStat = 0;
  if (millis() - lastStat > 10000) {
    lastStat = millis();
    Serial.printf("Free heap: %u  Free PSRAM: %u  AP clients: %d\n",
                  ESP.getFreeHeap(), ESP.getFreePsram(), WiFi.softAPgetStationNum());
  }
#endif
  delay(10);   // let other tasks (camera/web server) run
}