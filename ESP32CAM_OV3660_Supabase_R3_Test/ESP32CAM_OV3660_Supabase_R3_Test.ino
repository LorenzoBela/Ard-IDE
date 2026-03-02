/**
 * ESP32-CAM OV3660 → Supabase R3 (Person-Detection Triggered)
 * DOWNGRADED TO ESP32 CORE 2.0.X & ESP-FACE
 *
 * Behaviour:
 * - Continuously grabs QVGA frames and runs ESP-Face detection.
 * - Person detected  → captures a hi-res JPEG and uploads to Supabase.
 * - No person        → does nothing.
 *
 * Requirements:
 * - ESP32 Arduino Core 2.0.17
 * - PSRAM enabled (Tools → PSRAM → Enabled)
 * - AI-Thinker ESP32-CAM board
 * - Partition Scheme: Huge APP (3MB No OTA / 1MB SPIFFS)
 *
 * Serial commands:
 * c = force manual capture & upload (bypass detection)
 * h = show help
 */

#include <HTTPClient.h>
#include <WiFi.h>
#include <esp_arduino_version.h>
#include <esp_camera.h>
#include <esp_log.h>
#include <img_converters.h>
#include <time.h>

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR != 2
#error                                                                         \
    "This sketch strictly requires ESP32 Core 2.0.x (e.g. 2.0.17). Please downgrade in Boards Manager."
#endif

// Advanced Local Face Detection (built-in to ESP32 Core 2.x camera driver)
#include "human_face_detect_mnp01.hpp"
#include "human_face_detect_msr01.hpp"

// ===================== USER CONFIG =====================
// WiFi: connect to the hotspot broadcast by the GPS/LTE board
#define WIFI_SSID     "SmartTopBox_AP"
#define WIFI_PASSWORD "topbox123"

// Proxy endpoint on the GPS/LTE board (192.168.4.1 is default SoftAP IP)
// Images are POSTed here; the GPS/LTE board relays them to Supabase via LTE.
#define PROXY_HOST    "192.168.4.1"
#define PROXY_PORT    8080
#define PROXY_PATH    "/upload"

// Supabase credentials kept here for reference (used by GPS/LTE board for relay)
#define SUPABASE_URL "https://lvpneakciqegwyymtqno.supabase.co"
#define SUPABASE_BUCKET "r3"
#define SUPABASE_API_KEY                                                       \
  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9."                                      \
  "eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Imx2cG5lYWtjaXFlZ3d5eW10cW5vIiwicm9sZSI6Im" \
  "Fub24iLCJpYXQiOjE3Njc5MDYzNzgsImV4cCI6MjA4MzQ4MjM3OH0."                     \
  "liZ3l1u18H7WwIc72P9JgBTp9b7zUlLfPUhCAndW9uU"
#define SUPABASE_SERVICE_ROLE_KEY ""

#define DEVICE_ID "OV3660_CAM_001"
#define FILE_PREFIX "capture"
#define MAX_UPLOAD_RETRIES 3

// ===================== DETECTION CONFIG =====================
#define DETECTION_COOLDOWN_MS 5000     // Min time between uploads
#define UPLOAD_FRAME_SIZE FRAMESIZE_HD // 1280×720
#define UPLOAD_JPEG_QUALITY 8

// ===================== CAMERA PINS: AI THINKER =====================
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// ===================== STATE =====================
static unsigned long lastUploadAt = 0;
static unsigned long scanCount = 0;
static unsigned long personCount = 0;
static bool cameraInDetectMode = true;

// Initialize ESP-Face models
HumanFaceDetectMSR01 *s1;

// ===================== HELPERS =====================

void printCommands() {
  Serial.println(F("Commands:"));
  Serial.println(F("  c = force capture & upload (bypass detection)"));
  Serial.println(F("  h or ? = show commands"));
}

bool hasUnsetConfig() {
  return strlen(WIFI_SSID) == 0 || strlen(WIFI_PASSWORD) == 0 ||
         strlen(SUPABASE_URL) == 0 || strlen(SUPABASE_API_KEY) == 0;
}

const char *getSupabaseBearerToken() {
  if (strlen(SUPABASE_SERVICE_ROLE_KEY) > 0) {
    return SUPABASE_SERVICE_ROLE_KEY;
  }
  return SUPABASE_API_KEY;
}

// ===================== WIFI =====================

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // The GPS/LTE board starts its AP after LTE connects, which can take
  // 30-60 s from power-on. Retry until the AP is reachable (up to 90 s).
  Serial.print(F("Connecting to hotspot '"));
  Serial.print(WIFI_SSID);
  Serial.print(F("'"));
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 90000) {
    delay(500);
    Serial.print('.');
    // If WiFi can't associate yet, disconnect+reconnect to trigger a fresh scan
    if (millis() - start > 15000 &&
        (millis() - start) % 15000 < 600 &&
        WiFi.status() != WL_CONNECTED) {
      WiFi.disconnect();
      delay(200);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("WiFi connected to proxy AP. IP: "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("WiFi connection failed. Will retry on next upload."));
  }
}

void initTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 5000)) {
    Serial.println(F("NTP time synced."));
  } else {
    Serial.println(F("NTP sync failed (millis fallback)."));
  }
}

// ===================== CAMERA =====================

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000; // 10 MHz: reduces EV-VSYNC-OVF & FB-SIZE corruption when WiFi is active
  config.ledc_timer = LEDC_TIMER_0;
  config.ledc_channel = LEDC_CHANNEL_0;

  // For ESP-Face, PIXFORMAT_RGB565 directly feeds the algorithm without DMA
  // timeouts on 2.0.x
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QVGA;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.jpeg_quality = 12;
  config.fb_count = 2; // 2 buffers + GRAB_LATEST silently drops stale frames,
                       // preventing EV-VSYNC-OVF log spam

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor) {
    if (sensor->id.PID == OV3660_PID) {
      Serial.println(F("OV3660 detected."));
    }
    // Optimise for clarity and detection
    sensor->set_vflip(sensor, 1);
    sensor->set_hmirror(sensor, 1);
    sensor->set_brightness(sensor, 0);
    sensor->set_contrast(sensor, 0);
    sensor->set_saturation(sensor, 0);
    sensor->set_sharpness(sensor, 1);
    sensor->set_exposure_ctrl(sensor, 1);
  }

  cameraInDetectMode = true;
  Serial.println(F("Camera initialised (QVGA RGB565 for ESP-Face)."));
  return true;
}

void switchToUploadMode() {
  if (!cameraInDetectMode)
    return;

  esp_camera_deinit();
  delay(100);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000; // 10 MHz: prevents PSRAM starvation during WiFi bursts
  config.ledc_timer = LEDC_TIMER_0;
  config.ledc_channel = LEDC_CHANNEL_0;

  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = UPLOAD_FRAME_SIZE;
  config.jpeg_quality = UPLOAD_JPEG_QUALITY;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.fb_count = 2; // Need 2 for JPEG DMA

  esp_camera_init(&config);

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor) {
    sensor->set_vflip(sensor, 1);
    sensor->set_hmirror(sensor, 1);
  }

  cameraInDetectMode = false;
}

void switchToDetectMode() {
  if (cameraInDetectMode)
    return;

  esp_camera_deinit();
  delay(100);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000; // 10 MHz: prevents PSRAM starvation during WiFi bursts
  config.ledc_timer = LEDC_TIMER_0;
  config.ledc_channel = LEDC_CHANNEL_0;

  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.fb_count = 2;

  esp_camera_init(&config);

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor) {
    sensor->set_vflip(sensor, 1);
    sensor->set_hmirror(sensor, 1);
  }

  cameraInDetectMode = true;
}

// ===================== DETECTION =====================

/**
 * Returns true if a face is detected
 */
bool runFaceDetection() {
  camera_fb_t *fb = nullptr;
  for (int i = 0; i < 3; i++) {
    fb = esp_camera_fb_get();
    if (fb)
      break;
    delay(50);
  }

  if (!fb) {
    Serial.println(F("Detection: frame grab failed. Resetting..."));
    esp_camera_deinit();
    delay(100);
    initCamera();
    return false;
  }

  // Convert RGB565 to RGB888 for ESP-Face (MSR01 expects 24-bit RGB)
  size_t rgbBytes = (size_t)fb->width * (size_t)fb->height * 3;
  uint8_t *rgbBuffer = (uint8_t *)ps_malloc(rgbBytes);
  if (!rgbBuffer) {
    Serial.println(F("Face detect skipped: RGB buffer alloc failed."));
    esp_camera_fb_return(fb);
    return false;
  }

  bool converted = fmt2rgb888(fb->buf, fb->len, fb->format, rgbBuffer);
  esp_camera_fb_return(fb); // Free fb early

  if (!converted) {
    free(rgbBuffer);
    return false;
  }

  // Run MSR01 Face Detection
  std::list<dl::detect::result_t> &results =
      s1->infer(rgbBuffer, {(int)fb->height, (int)fb->width, 3});

  bool detected = false;
  if (results.size() > 0) {
    detected = true;
  }

  free(rgbBuffer);
  return detected;
}

// ===================== UPLOAD =====================

String makeObjectPath() {
  char filename[96];
  snprintf(filename, sizeof(filename), "%s_%lu.jpg", FILE_PREFIX, millis());
  return String("esp32cam/") + DEVICE_ID + "/" + filename;
}

bool uploadToSupabase(const uint8_t *data, size_t len,
                      const String &objectPath) {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println(F("Upload skipped: WiFi (proxy AP) disconnected."));
      return false;
    }
  }

  // POST image data to the GPS/LTE board proxy which relays it to Supabase via LTE.
  HTTPClient http;
  String endpoint = String("http://") + PROXY_HOST + ":" +
                    String(PROXY_PORT) + PROXY_PATH;

  http.setTimeout(90000); // LTE relay can take up to ~60 s for large frames
  http.begin(endpoint);
  http.addHeader("Content-Type", "image/jpeg");
  // Tell the proxy the Supabase storage path for this image
  http.addHeader("X-Object-Path", objectPath);

  Serial.printf("Uploading via LTE proxy (%s)...\n", endpoint.c_str());
  int code = http.POST((uint8_t *)data, len);
  String response = http.getString();
  http.end();

  // Always print the relay diagnostic body so the proxy's internals are
  // visible on this serial monitor even when the LilyGO has no USB attached.
  Serial.printf("[PROXY] HTTP %d | %s\n", code,
                response.length() > 0 ? response.c_str() : "(no body)");

  if (code == 200 || code == 201) {
    Serial.print(F("Proxy upload success -> "));
    Serial.println(objectPath);
    return true;
  }

  Serial.println(F("Proxy upload failed."));
  return false;
}

void captureHighResAndUpload() {
  unsigned long t0 = millis();

  // Switch to JPEG
  switchToUploadMode();

  // Give the sensor a tiny moment to adjust its exposure in the new resolution
  delay(150);

  // Flush 1 stale frame that might have been queued during the switch
  camera_fb_t *stale = esp_camera_fb_get();
  if (stale) {
    esp_camera_fb_return(stale);
  }

  // Grab the actual high-res photo
  camera_fb_t *photo = esp_camera_fb_get();

  // If the frame is suspiciously small (corrupted JPEG), try one more time
  if (photo && photo->len < 5000) {
    esp_camera_fb_return(photo);
    delay(50);
    photo = esp_camera_fb_get();
  }

  if (!photo) {
    Serial.println(F("ERROR: Hi-res capture failed."));
    switchToDetectMode();
    return;
  }

  Serial.printf("Captured JPEG: %u bytes in %lu ms\n", photo->len,
                millis() - t0);

  String objectPath = makeObjectPath();
  bool uploaded = false;
  for (int attempt = 1; attempt <= MAX_UPLOAD_RETRIES; attempt++) {
    Serial.printf("Upload attempt %d/%d\n", attempt, MAX_UPLOAD_RETRIES);
    if (uploadToSupabase(photo->buf, photo->len, objectPath)) {
      uploaded = true;
      break;
    }
    delay(1000);
  }

  esp_camera_fb_return(photo);
  lastUploadAt = millis();

  // Switch back
  switchToDetectMode();
}

// ===================== MAIN LOOP =====================

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char command = (char)Serial.read();

    if (command == 'c' || command == 'C') {
      Serial.println(F("Manual capture (bypass detection)."));
      captureHighResAndUpload();
    } else if (command == 'h' || command == 'H' || command == '?') {
      printCommands();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println(F("\n============================================="));
  Serial.println(F(" ESP32-CAM OV3660 Face-Detection Capture"));
  Serial.println(F(" ESP-Face (Core 2.0.x) -> Supabase R3 bucket"));
  Serial.println(F("============================================="));

  esp_log_level_set(
      "*",
      ESP_LOG_ERROR); // Suppress all warnings including cam_hal: EV-VSYNC-OVF
  esp_log_level_set("camera", ESP_LOG_ERROR);

  Serial.printf("PSRAM size: %d bytes\n", ESP.getPsramSize());
  Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());

  if (hasUnsetConfig()) {
    Serial.println(
        F("ERROR: Missing WiFi or Supabase config. Please edit sketch."));
    while (true)
      delay(1000);
  }

  connectWiFi();
  initTime();

  if (!initCamera()) {
    Serial.println(F("Camera setup failed. Halting."));
    while (true)
      delay(1000);
  }

  // Allocate ESP-Face model
  s1 = new HumanFaceDetectMSR01(0.1F, 0.5F, 10, 0.2F);

  printCommands();
  Serial.println(F("\nFace detection active. Waiting for faces..."));
}

void loop() {
  handleSerialCommands();

  bool faceFound = runFaceDetection();
  scanCount++;

  if (faceFound) {
    personCount++;
    Serial.printf("=> FACE DETECTED! Frame #%lu. Uploading...\n", scanCount);

    if (millis() - lastUploadAt >= DETECTION_COOLDOWN_MS) {
      captureHighResAndUpload();
      Serial.println(F("\nFace detection resumed. Waiting..."));
    } else {
      Serial.println(F("Upload skipped (cooldown active)."));
    }
  }

  // A tiny yield so the ESP32 doesn't trigger FreeRTOS watchdogs
  delay(10);
}