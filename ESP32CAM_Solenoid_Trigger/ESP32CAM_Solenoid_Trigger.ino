#include <HTTPClient.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <esp_log.h>
#include <time.h>


// ===================== USER CONFIG =====================
#define WIFI_SSID "GlobeAtHome_e9a28_2.4"
#define WIFI_PASSWORD "furymabaho912!"

#define SUPABASE_URL "https://lvpneakciqegwyymtqno.supabase.co"
#define SUPABASE_BUCKET "r3"
#define SUPABASE_API_KEY                                                       \
  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9."                                      \
  "eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Imx2cG5lYWtjaXFlZ3d5eW10cW5vIiwicm9sZSI6Im" \
  "Fub24iLCJpYXQiOjE3Njc5MDYzNzgsImV4cCI6MjA4MzQ4MjM3OH0."                     \
  "liZ3l1u18H7WwIc72P9JgBTp9b7zUlLfPUhCAndW9uU"
#define SUPABASE_SERVICE_ROLE_KEY ""

#define DEVICE_ID "OV3660_CAM_001"
#define FILE_PREFIX "solenoid_capture"

// --- SERIAL TRIGGER CONFIG ---
const unsigned long CAPTURE_COOLDOWN_MS =
    5000; // Minimum time between captures (5 seconds)
unsigned long lastCaptureTime = 0;

// Camera configuration
#define MAX_UPLOAD_RETRIES 3
#define FAST_CAPTURE_MODE true
#define FAST_DETAIL_PS_FRAME_SIZE FRAMESIZE_CIF
#define FAST_DETAIL_NO_PSRAM_FRAME_SIZE FRAMESIZE_QVGA
#define FAST_DETAIL_PS_JPEG_QUALITY 10
#define FAST_DETAIL_NO_PSRAM_JPEG_QUALITY 10
#define CAMERA_XCLK_HZ 10000000
#define CAPTURE_FLUSH_FRAMES 1
#define CAPTURE_FLUSH_DELAY_MS 25

// ===================== CAMERA MODEL: AI THINKER =====================
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

bool captureFallbackActive = false;

// ----------------- Helper Functions -----------------
bool hasUnsetConfig() {
  return String(WIFI_SSID).length() == 0 ||
         String(WIFI_PASSWORD).length() == 0 ||
         String(SUPABASE_URL).length() == 0 ||
         String(SUPABASE_API_KEY).length() == 0;
}

String getSupabaseBearerToken() {
  if (String(SUPABASE_SERVICE_ROLE_KEY).length() > 0) {
    return String(SUPABASE_SERVICE_ROLE_KEY);
  }
  return String(SUPABASE_API_KEY);
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed.");
  }
}

void initTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 5000)) {
    Serial.println("NTP time synced.");
  } else {
    Serial.println("NTP sync failed (using millis fallback for filenames).");
  }
}

camera_fb_t *getFrameWithRetry(int retries, int delayMs) {
  for (int attempt = 0; attempt < retries; attempt++) {
    camera_fb_t *frame = esp_camera_fb_get();
    if (frame) {
      return frame;
    }
    delay(delayMs);
  }
  return nullptr;
}

void flushCameraFrames(int count) {
  for (int i = 0; i < count; i++) {
    camera_fb_t *oldFrame = esp_camera_fb_get();
    if (oldFrame) {
      esp_camera_fb_return(oldFrame);
    }
    delay(CAPTURE_FLUSH_DELAY_MS);
  }
}

void applyRecoverySensorProfile() {
  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor)
    return;

  sensor->reset(sensor);
  delay(120);
  sensor->set_framesize(sensor, FRAMESIZE_QVGA);
  sensor->set_quality(sensor, 12);
  sensor->set_brightness(sensor, -1);
  sensor->set_contrast(sensor, 1);
  sensor->set_sharpness(sensor, 1);
  captureFallbackActive = true;
  Serial.println("Applied recovery profile: QVGA / quality 12.");
}

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
  config.xclk_freq_hz = CAMERA_XCLK_HZ;
  config.pixel_format = PIXFORMAT_JPEG;

  if (FAST_CAPTURE_MODE) {
    if (psramFound()) {
      config.frame_size = FAST_DETAIL_PS_FRAME_SIZE;
      config.jpeg_quality = FAST_DETAIL_PS_JPEG_QUALITY;
      config.fb_count = 2;
    } else {
      config.frame_size = FAST_DETAIL_NO_PSRAM_FRAME_SIZE;
      config.jpeg_quality = FAST_DETAIL_NO_PSRAM_JPEG_QUALITY;
      config.fb_count = 1;
    }
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  } else if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor) {
    sensor->set_brightness(sensor, -2);
    sensor->set_contrast(sensor, 1);
    sensor->set_saturation(sensor, 0);
    sensor->set_sharpness(sensor, 1);
    sensor->set_exposure_ctrl(sensor, 1);
    sensor->set_aec2(sensor, 1);
    sensor->set_ae_level(sensor, 0);
    sensor->set_aec_value(sensor, 300);
    sensor->set_gain_ctrl(sensor, 1);
    sensor->set_bpc(sensor, 1);
  }
  return true;
}

String makeObjectPath() {
  char filename[96];
  snprintf(filename, sizeof(filename), FILE_PREFIX "_%lu.jpg", millis());
  return String("esp32cam/") + DEVICE_ID + "/" + filename;
}

bool uploadToSupabase(const uint8_t *data, size_t len,
                      const String &objectPath) {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Upload skipped: WiFi not connected.");
      return false;
    }
  }

  HTTPClient http;
  String endpoint = String(SUPABASE_URL) + "/storage/v1/object/" +
                    SUPABASE_BUCKET + "/" + objectPath;

  http.setTimeout(25000);
  http.begin(endpoint);
  http.addHeader("Authorization", "Bearer " + getSupabaseBearerToken());
  http.addHeader("apikey", String(SUPABASE_API_KEY));
  http.addHeader("Content-Type", "image/jpeg");
  http.addHeader("x-upsert", "true");

  int code = http.POST((uint8_t *)data, len);
  String response = http.getString();
  http.end();

  if (code == 200 || code == 201) {
    Serial.print("Upload success: ");
    Serial.println(objectPath);
    return true;
  }

  Serial.printf("Upload failed. HTTP %d\n", code);
  if (response.length() > 0)
    Serial.println(response);
  return false;
}

void captureAndUpload() {
  unsigned long captureStart = millis();
  flushCameraFrames(CAPTURE_FLUSH_FRAMES);

  camera_fb_t *frame = getFrameWithRetry(2, 80);
  if (!frame) {
    Serial.println("No frame on first attempt.");
    if (!captureFallbackActive) {
      applyRecoverySensorProfile();
      flushCameraFrames(1);
      frame = getFrameWithRetry(3, 100);
    }
    if (!frame)
      frame = getFrameWithRetry(4, 120);

    if (!frame) {
      Serial.printf("Capture failed after %lu ms.\n", millis() - captureStart);
      return;
    }
  }

  Serial.printf("Captured JPEG bytes: %d\n", frame->len);
  String objectPath = makeObjectPath();

  bool uploaded = false;
  for (int attempt = 1; attempt <= MAX_UPLOAD_RETRIES; attempt++) {
    Serial.printf("Upload attempt %d/%d\n", attempt, MAX_UPLOAD_RETRIES);
    if (uploadToSupabase(frame->buf, frame->len, objectPath)) {
      uploaded = true;
      break;
    }
    delay(1000);
  }

  if (!uploaded) {
    Serial.println("Final upload status: FAILED");
  }
  esp_camera_fb_return(frame);
}

void handleSerial() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equalsIgnoreCase("CAPTURE") || command.equalsIgnoreCase("C")) {
      if (millis() - lastCaptureTime >= CAPTURE_COOLDOWN_MS) {
        Serial.println("\n>>> SERIAL TRIGGER RECEIVED! CAPTURING PHOTO... <<<");
        captureAndUpload();
        lastCaptureTime = millis();
      } else {
        Serial.println("\n>>> Serial trigger received, but still in cooldown. "
                       "Ignoring. <<<");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nESP32-CAM Serial RX/TX Trigger -> Supabase Upload");

  esp_log_level_set("cam_hal", ESP_LOG_NONE);
  esp_log_level_set("camera", ESP_LOG_WARN);

  if (hasUnsetConfig()) {
    Serial.println("Please edit WiFi/Supabase config values.");
    while (true)
      delay(1000);
  }

  connectWiFi();
  initTime();

  if (!initCamera()) {
    Serial.println("Camera setup failed. Halting.");
    while (true)
      delay(1000);
  }

  // Warmup
  camera_fb_t *warmup = esp_camera_fb_get();
  if (warmup)
    esp_camera_fb_return(warmup);

  Serial.println("Setup complete. Waiting for 'CAPTURE' over Serial RX...");
}

void loop() {
  handleSerial();
  delay(50); // Let WiFi and other background tasks run freely
}
