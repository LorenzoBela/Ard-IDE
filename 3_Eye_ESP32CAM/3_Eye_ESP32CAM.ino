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
#include <WiFiUdp.h>
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
// WiFi: auto-discovered by scanning for SmartTopBox_AP_* SSIDs
char WIFI_SSID[24] = "";
#define WIFI_PASSWORD "topbox123"

// Proxy endpoint on the GPS/LTE board (192.168.4.1 is default SoftAP IP)
// Images are POSTed here; the GPS/LTE board relays them to Supabase via LTE.
#define PROXY_HOST "192.168.4.1"
#define PROXY_PORT 8080
#define PROXY_PATH "/upload"

// Supabase credentials kept here for reference (used by GPS/LTE board for
// relay)
#define SUPABASE_URL "https://lvpneakciqegwyymtqno.supabase.co"
#define SUPABASE_BUCKET "proof-photos"
#define SUPABASE_API_KEY                                                       \
  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9."                                      \
  "eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Imx2cG5lYWtjaXFlZ3d5eW10cW5vIiwicm9sZSI6Im" \
  "Fub24iLCJpYXQiOjE3Njc5MDYzNzgsImV4cCI6MjA4MzQ4MjM3OH0."                     \
  "liZ3l1u18H7WwIc72P9JgBTp9b7zUlLfPUhCAndW9uU"
#define SUPABASE_SERVICE_ROLE_KEY ""

char DEVICE_ID[24] = "OV3660_CAM_001";
#define FILE_PREFIX "capture"
#define MAX_UPLOAD_RETRIES 3

// ===================== DETECTION CONFIG =====================
#define DETECTION_COOLDOWN_MS 5000     // Min time between uploads
#define UPLOAD_FRAME_SIZE FRAMESIZE_HD // 1280×720
#define UPLOAD_JPEG_QUALITY 8
#define FACE_DETECT_WINDOW_MS 6000  // Repeated detect window per request
#define FACE_DETECT_RETRY_GAP_MS 30 // Gap between detect attempts
#define WIFI_BOOT_CONNECT_TIMEOUT_MS 120000
#define WIFI_RECONNECT_TIMEOUT_MS 20000
#define WIFI_MAINTAIN_INTERVAL_MS 8000

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

// Deferred upload flag — set by face-check handlers, executed in loop()
static bool pendingUpload = false;
static bool uploadInProgress = false;

static char currentDeliveryId[64] = "UNKNOWN_DELIVERY";

// Initialize ESP-Face models
HumanFaceDetectMSR01 *s1;

// HTTP server for face-status endpoint (port 80)
// Proxy board forwards GET /face-check here
WiFiServer faceServer(80);

// UART fallback to Tester board (Serial2)
// Wire: CAM TX(14) → Tester RX2(16), Tester TX2(17) → CAM RX(13), shared GND
#define TESTER_UART_RX 13
#define TESTER_UART_TX 14
#define TESTER_UART_BAUD 9600

// ===================== UDP LOGGING =====================
WiFiUDP udpClient;
#define UDP_LOG_PORT 5114

void netLog(const char *format, ...) {
  char buf[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  Serial.print(buf);

  if (WiFi.status() == WL_CONNECTED) {
    udpClient.beginPacket(PROXY_HOST, UDP_LOG_PORT);
    udpClient.print("[CAM] ");
    udpClient.print(buf);
    udpClient.endPacket();
  }
} // ===================== HELPERS =====================

bool queueSingleUpload(const char *source) {
  unsigned long now = millis();

  if (uploadInProgress || pendingUpload) {
    netLog("[UPLOAD] Skip queue from %s (busy)\n", source);
    return false;
  }

  if (lastUploadAt != 0 && (now - lastUploadAt) < DETECTION_COOLDOWN_MS) {
    netLog("[UPLOAD] Skip queue from %s (cooldown)\n", source);
    return false;
  }

  pendingUpload = true;
  netLog("[UPLOAD] Queued from %s\n", source);
  return true;
}

void printCommands() {
  Serial.println(F("Commands:"));
  Serial.println(F("  c = force capture & upload (bypass detection)"));
  Serial.println(F("  h or ? = show commands"));
}

bool hasUnsetConfig() {
  return strlen(WIFI_PASSWORD) == 0 ||
         strlen(SUPABASE_URL) == 0 || strlen(SUPABASE_API_KEY) == 0;
}

const char *getSupabaseBearerToken() {
  if (strlen(SUPABASE_SERVICE_ROLE_KEY) > 0) {
    return SUPABASE_SERVICE_ROLE_KEY;
  }
  return SUPABASE_API_KEY;
}

// ===================== WIFI =====================

void scanForProxyAP() {
  Serial.println(F("[WIFI] Scanning for SmartTopBox_AP_*..."));
  int n = WiFi.scanNetworks();

  int bestIdx = -1;
  int bestRssi = -999;
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    if (ssid.startsWith("SmartTopBox_AP_")) {
      if (WiFi.RSSI(i) > bestRssi) {
        bestRssi = WiFi.RSSI(i);
        bestIdx = i;
      }
    }
  }

  if (bestIdx >= 0) {
    String ssid = WiFi.SSID(bestIdx);
    strncpy(WIFI_SSID, ssid.c_str(), sizeof(WIFI_SSID) - 1);
    WIFI_SSID[sizeof(WIFI_SSID) - 1] = '\0';

    int num = ssid.substring(15).toInt();
    snprintf(DEVICE_ID, sizeof(DEVICE_ID), "OV3660_CAM_%03d", num);

    Serial.printf("[WIFI] Found: %s (%d dBm) -> %s\n",
                  WIFI_SSID, bestRssi, DEVICE_ID);
  } else {
    Serial.printf("[WIFI] No SmartTopBox_AP_* found (%d APs scanned)\n", n);
  }

  WiFi.scanDelete();
}

const char *wifiStatusText(wl_status_t status) {
  switch (status) {
  case WL_IDLE_STATUS:
    return "IDLE";
  case WL_NO_SSID_AVAIL:
    return "NO_SSID";
  case WL_SCAN_COMPLETED:
    return "SCAN_COMPLETED";
  case WL_CONNECTED:
    return "CONNECTED";
  case WL_CONNECT_FAILED:
    return "CONNECT_FAILED";
  case WL_CONNECTION_LOST:
    return "CONNECTION_LOST";
  case WL_DISCONNECTED:
    return "DISCONNECTED";
  default:
    return "UNKNOWN";
  }
}

void logTargetApVisibility() {
  int found = WiFi.scanNetworks();
  if (found <= 0) {
    Serial.println(F("[WIFI] Scan result: no APs found"));
    return;
  }

  bool targetFound = false;
  for (int i = 0; i < found; i++) {
    if (WiFi.SSID(i) == String(WIFI_SSID)) {
      targetFound = true;
      Serial.printf("[WIFI] Target AP visible | RSSI %d dBm | CH %d\n",
                    WiFi.RSSI(i), WiFi.channel(i));
      break;
    }
  }

  if (!targetFound) {
    Serial.printf("[WIFI] Target AP '%s' not visible in scan (%d APs)\n",
                  WIFI_SSID, found);
  }

  WiFi.scanDelete();
}

bool connectWiFi(unsigned long timeoutMs = WIFI_BOOT_CONNECT_TIMEOUT_MS) {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);

  WiFi.disconnect(true, false);
  delay(150);

  if (WIFI_SSID[0] == '\0') {
    scanForProxyAP();
  }
  if (WIFI_SSID[0] == '\0') {
    Serial.println(F("[WIFI] No proxy AP found, cannot connect"));
    return false;
  }

  // Static IP so the LilyGO proxy can always reach us on 192.168.4.10
  // (matches the hardcoded fallback in forwardFaceCheck() on the LilyGO side)
  IPAddress staticIP(192, 168, 4, 10);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(staticIP, gateway, subnet, gateway);

  WiFi.begin((const char *)WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("[WIFI] Connecting to '%s'\n", WIFI_SSID);
  unsigned long start = millis();
  unsigned long lastRetryAt = start;
  wl_status_t lastStatus = WiFi.status();

  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    wl_status_t statusNow = WiFi.status();
    if (statusNow != lastStatus) {
      Serial.printf("[WIFI] Status -> %s (%d)\n", wifiStatusText(statusNow),
                    (int)statusNow);
      lastStatus = statusNow;
    }

    if (millis() - lastRetryAt >= 12000) {
      Serial.println(F("[WIFI] Re-associate attempt..."));
      WiFi.disconnect();
      delay(100);
      WiFi.begin((const char *)WIFI_SSID, WIFI_PASSWORD);
      lastRetryAt = millis();
    }

    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WIFI] Connected. IP: %s | RSSI: %d dBm | CH: %d\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI(),
                  WiFi.channel());
    return true;
  } else {
    wl_status_t statusNow = WiFi.status();
    Serial.printf("[WIFI] Connect FAILED. Final status: %s (%d)\n",
                  wifiStatusText(statusNow), (int)statusNow);
    logTargetApVisibility();
    return false;
  }
}

void maintainWiFiConnection() {
  static unsigned long lastAttemptAt = 0;
  unsigned long now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  if (now - lastAttemptAt < WIFI_MAINTAIN_INTERVAL_MS) {
    return;
  }

  lastAttemptAt = now;
  Serial.printf("[WIFI] Link down (%s). Reconnecting...\n",
                wifiStatusText(WiFi.status()));
  connectWiFi(WIFI_RECONNECT_TIMEOUT_MS);
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
  config.xclk_freq_hz = 10000000; // 10 MHz: reduces EV-VSYNC-OVF & FB-SIZE
                                  // corruption when WiFi is active
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
  config.xclk_freq_hz =
      10000000; // 10 MHz: prevents PSRAM starvation during WiFi bursts
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
  config.xclk_freq_hz =
      10000000; // 10 MHz: prevents PSRAM starvation during WiFi bursts
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

  int fbWidth  = (int)fb->width;
  int fbHeight = (int)fb->height;

  bool converted = fmt2rgb888(fb->buf, fb->len, fb->format, rgbBuffer);
  esp_camera_fb_return(fb); // Free fb early — do not access fb after this
  fb = nullptr;

  if (!converted) {
    free(rgbBuffer);
    return false;
  }

  // Run MSR01 Face Detection
  std::list<dl::detect::result_t> &results =
      s1->infer(rgbBuffer, {fbHeight, fbWidth, 3});

  bool detected = false;
  if (results.size() > 0) {
    detected = true;
  }

  free(rgbBuffer);
  return detected;
}

bool runFaceDetectionWindow(unsigned long windowMs) {
  unsigned long detectStart = millis();
  while (millis() - detectStart < windowMs) {
    if (runFaceDetection()) {
      scanCount++;
      return true;
    }
    scanCount++;
    delay(FACE_DETECT_RETRY_GAP_MS);
  }
  return false;
}

// ===================== UPLOAD =====================

String makeObjectPath() {
  char filename[128];
  snprintf(filename, sizeof(filename), "%s_%lu.jpg", currentDeliveryId,
           millis());
  return String("deliveries/") + DEVICE_ID + "/" + filename;
}

bool uploadToSupabase(const uint8_t *data, size_t len,
                      const String &objectPath) {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi(WIFI_RECONNECT_TIMEOUT_MS);
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println(F("Upload skipped: WiFi (proxy AP) disconnected."));
      return false;
    }
  }

  // POST image data to the GPS/LTE board proxy which relays it to Supabase via
  // LTE.
  HTTPClient http;
  String endpoint =
      String("http://") + PROXY_HOST + ":" + String(PROXY_PORT) + PROXY_PATH;

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

  // DELAY UPLOAD TO PREVENT BROWNOUT DURING SOLENOID FIRING
  // The solenoid is actively firing right now (takes ~1000ms).
  // The Proxy LTE transmission draws huge current spikes up to 2A.
  // Delaying the HTTP POST ensures the physical lock actuates securely
  // before the modem blasts the network.
  delay(1500);

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

  connectWiFi(WIFI_BOOT_CONNECT_TIMEOUT_MS);
  initTime();

  if (!initCamera()) {
    Serial.println(F("Camera setup failed. Halting."));
    while (true)
      delay(1000);
  }

  // Allocate ESP-Face model
  s1 = new HumanFaceDetectMSR01(0.1F, 0.5F, 10, 0.2F);

  // Start HTTP server for /face-status endpoint
  faceServer.begin();
  Serial.println(F("[HTTP] Face-status server on port 80"));

  // Start Serial2 for UART fallback from Tester board
  Serial2.begin(TESTER_UART_BAUD, SERIAL_8N1, TESTER_UART_RX, TESTER_UART_TX);
  Serial.println(F("[UART] Serial2 ready (Tester fallback)"));

  printCommands();
  Serial.println(F("\nFlushing stale camera frames..."));
  for (int i = 0; i < 3; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb)
      esp_camera_fb_return(fb);
    delay(50);
  }
  Serial.println(F("Face detection active. Waiting for faces..."));
}

// ===================== FACE-STATUS HTTP HANDLER =====================
// Responds to GET /face-status from the GPS/LTE proxy board.
// Runs face detection on demand. Responds IMMEDIATELY, then defers upload
// to loop() to avoid deadlock (CAM uploads back to the same proxy).
void handleFaceStatusClient() {
  WiFiClient client = faceServer.available();
  if (!client)
    return;

  Serial.println(F("[HTTP] Face-status request received"));

  // Wait for data
  unsigned long timeout = millis() + 5000;
  while (!client.available() && millis() < timeout) {
    delay(10);
  }
  if (!client.available()) {
    client.stop();
    return;
  }

  String requestLine = client.readStringUntil('\n');
  requestLine.trim();

  if (!requestLine.startsWith("GET /face-status")) {
    netLog("[HTTP] Ignoring request: %s\n", requestLine.c_str());
    const char *body = "NOT_FOUND\r\n";
    char resp404[160];
    snprintf(resp404, sizeof(resp404),
             "HTTP/1.1 404 Not Found\r\n"
             "Content-Type: text/plain\r\n"
             "Content-Length: %d\r\n"
             "Connection: close\r\n\r\n"
             "%s",
             (int)strlen(body), body);
    client.print(resp404);
    client.flush();
    delay(50);
    client.stop();
    return;
  }

  // Parse deliveryId from query string if present
  int qIdx = requestLine.indexOf("?delivery_id=");
  if (qIdx >= 0) {
    int spIdx = requestLine.indexOf(' ', qIdx);
    if (spIdx > qIdx) {
      String delId = requestLine.substring(qIdx + 13, spIdx);
      if (delId.length() > 0 && delId.length() < sizeof(currentDeliveryId)) {
        strncpy(currentDeliveryId, delId.c_str(),
                sizeof(currentDeliveryId) - 1);
        currentDeliveryId[sizeof(currentDeliveryId) - 1] = '\0';
      }
    }
  }

  // Read and discard HTTP headers
  while (client.available()) {
    String line = client.readStringUntil('\n');
    line.trim();
    if (line.length() == 0)
      break; // End of headers
  }

  // Run repeated face detection in a window (old working logic style)
  bool faceFound = runFaceDetectionWindow(FACE_DETECT_WINDOW_MS);

  const char *result;
  if (faceFound) {
    personCount++;
    netLog("[HTTP] Face detected!\n");
    result = "FACE_OK";
    queueSingleUpload("HTTP");
  } else {
    netLog("[HTTP] No face detected\n");
    result = "NO_FACE";
  }

  // Send HTTP response IMMEDIATELY (before any upload)
  String fullResult = String(result) + "\r\n";
  char resp[128];
  snprintf(resp, sizeof(resp),
           "HTTP/1.1 200 OK\r\n"
           "Content-Type: text/plain\r\n"
           "Content-Length: %d\r\n"
           "Connection: close\r\n\r\n"
           "%s",
           (int)fullResult.length(), fullResult.c_str());
  client.print(resp);
  client.flush();
  delay(50);
  client.stop();
}

void loop() {
  handleSerialCommands();
  maintainWiFiConnection();
  handleFaceStatusClient(); // On-demand face-check via WiFi (from proxy)
  handleUartFaceCommand();  // On-demand face-check via UART (from Tester)

  // Deferred upload — runs AFTER face-check response was sent
  // This avoids deadlock (CAM uploads to the same proxy that requested
  // face-check)
  if (pendingUpload && !uploadInProgress) {
    pendingUpload = false;
    uploadInProgress = true;
    netLog("[UPLOAD] Deferred capture + upload starting...\n");
    captureHighResAndUpload();
    netLog("[UPLOAD] Done. One photo captured.\n");
    uploadInProgress = false;
  }

  // A tiny yield so the ESP32 doesn't trigger FreeRTOS watchdogs
  delay(10);
}

// ===================== UART FACE COMMAND HANDLER =====================
// Responds to "FACE?" from Tester board over Serial2 when WiFi is down.
void handleUartFaceCommand() {
  if (!Serial2.available())
    return;

  char cmd[16] = "";
  uint8_t len = 0;
  unsigned long start = millis();

  while (millis() - start < 200) {
    if (Serial2.available()) {
      char c = Serial2.read();
      if (c == '\n' || c == '\r') {
        if (len > 0)
          break;
      } else if (len < sizeof(cmd) - 1) {
        cmd[len++] = c;
      }
    }
  }
  cmd[len] = '\0';

  if (strncmp(cmd, "FACE?", 5) != 0)
    return;

  // Extract delivery_id if present
  if (cmd[5] == ',') {
    String delId = String(cmd + 6);
    delId.trim();
    if (delId.length() > 0 && delId.length() < sizeof(currentDeliveryId)) {
      strncpy(currentDeliveryId, delId.c_str(), sizeof(currentDeliveryId) - 1);
      currentDeliveryId[sizeof(currentDeliveryId) - 1] = '\0';
    }
  }

  netLog("[UART] Face check requested from Tester board\n");

  // Run repeated face detection in a window (old working logic style)
  bool detected = runFaceDetectionWindow(FACE_DETECT_WINDOW_MS);

  // Respond IMMEDIATELY
  if (detected) {
    netLog("[UART] Face DETECTED\n");
    Serial2.println("FACE_OK");
    queueSingleUpload("UART");
  } else {
    Serial.println(F("[UART] No face"));
    Serial2.println("NO_FACE");
  }
}