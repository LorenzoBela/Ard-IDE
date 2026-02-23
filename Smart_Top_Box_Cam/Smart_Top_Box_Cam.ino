/**
 * SMART TOP BOX - CAMERA SUBSYSTEM
 *
 * Hardware: ESP32-CAM (AI Thinker)
 * Interface: UART0 (RX=3, TX=1) connected to Main MCU via logic level
 * shifter/direct wire. Protocol:
 * - Listen for 'C' (Capture Command)
 * - Reply 'D' (Done/Success)
 * - Reply 'F' (Fail)
 *
 * Note: While debugging via USB, disconnect RX/TX from Main MCU to avoid
 * conflict.
 */

#include <HTTPClient.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <esp_log.h>
#include <time.h>


// ===================== USER CONFIG =====================
#define WIFI_SSID "RAJ_VIRUS2"
#define WIFI_PASSWORD "I@mjero4ever"

#define SUPABASE_URL "https://lvpneakciqegwyymtqno.supabase.co"
#define SUPABASE_BUCKET "r3"
#define SUPABASE_API_KEY                                                       \
  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9."                                      \
  "eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Imx2cG5lYWtjaXFlZ3d5eW10cW5vIiwicm9sZSI6Im" \
  "Fub24iLCJpYXQiOjE3Njc5MDYzNzgsImV4cCI6MjA4MzQ4MjM3OH0."                     \
  "liZ3l1u18H7WwIc72P9JgBTp9b7zUlLfPUhCAndW9uU"
#define SUPABASE_SERVICE_ROLE_KEY ""

#define DEVICE_ID "OV3660_CAM_001" // Or link to Box ID
#define FILE_PREFIX "evidence"
#define MAX_UPLOAD_RETRIES 3

// ===================== CAMERA PINS (AI THINKER) =====================
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

  // Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) {
    delay(400);
    // Serial.print(".");
  }
  // Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    // Serial.println("WiFi connected.");
  } else {
    // Serial.println("WiFi failed.");
  }
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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA; // UXGA logic
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    return false;
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
  if (WiFi.status() != WL_CONNECTED)
    connectWiFi();
  if (WiFi.status() != WL_CONNECTED)
    return false;

  HTTPClient http;
  String endpoint = String(SUPABASE_URL) + "/storage/v1/object/" +
                    SUPABASE_BUCKET + "/" + objectPath;

  http.setTimeout(20000);
  http.begin(endpoint);
  http.addHeader("Authorization", "Bearer " + getSupabaseBearerToken());
  http.addHeader("apikey", String(SUPABASE_API_KEY));
  http.addHeader("Content-Type", "image/jpeg");
  http.addHeader("x-upsert", "true");

  int code = http.POST((uint8_t *)data, len);
  http.end();

  return (code == 200 || code == 201);
}

void processCaptureCommand() {
  // Serial.println("Capturing...");

  camera_fb_t *fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.print('F'); // Fail
    return;
  }

  String path = makeObjectPath();
  bool success = false;

  // Try upload
  for (int i = 0; i < 3; i++) {
    if (uploadToSupabase(fb->buf, fb->len, path)) {
      success = true;
      break;
    }
    delay(500);
  }

  esp_camera_fb_return(fb);

  if (success) {
    Serial.print('D'); // Done - Signal to Main MCU to Unlock
  } else {
    Serial.print('F'); // Upload Failed
  }
}

void setup() {
  Serial.begin(115200); // Communicate with Main MCU

  // Disable brownout detector
  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  connectWiFi();
  initCamera();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'C') { // Standard command from Main MCU
      processCaptureCommand();
    }
  }
  delay(10);
}
