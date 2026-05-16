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
#include <Preferences.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_arduino_version.h>
#include <esp_camera.h>
#include <esp_log.h>
#include <time.h>

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR != 2
#error                                                                         \
    "This sketch strictly requires ESP32 Core 2.0.x (e.g. 2.0.17). Please downgrade in Boards Manager."
#endif

// Advanced Local Face Detection (built-in to ESP32 Core 2.x camera driver)
#include "human_face_detect_msr01.hpp"

// ===================== USER CONFIG =====================
// WiFi: production hotspot-first variant. Hotspots are tried in priority order.
// Empty SSIDs are ignored.
struct HotspotCredential {
  const char *ssid;
  const char *password;
};

static const HotspotCredential HOTSPOTS[] = {
    {"ZTE_2.4G_3GRHSf", "C539c7d4"},
    {"bibliyuh", "qwertyui"},
    {"Zooo :P", "Xpander19"},
    {"Vivviccc", "vivviccc"},
};
static const uint8_t HOTSPOT_COUNT = sizeof(HOTSPOTS) / sizeof(HOTSPOTS[0]);

char WIFI_SSID[33] = "";
char WIFI_PASSWORD[65] = "";

// Proxy endpoint on the LilyGO board; discovered dynamically on the hotspot LAN.
// Images are POSTed here; the GPS/LTE board relays them to Supabase via LTE.
char PROXY_HOST[16] = "";
#define PROXY_PORT 8080
#define PROXY_PATH "/upload"
#define PROXY_DISCOVERY_PORT 5115
#define PROXY_DISCOVERY_QUERY "SMART_TOP_BOX_PROXY?"
#define PROXY_DISCOVERY_REPLY "SMART_TOP_BOX_PROXY:"

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
#define UPLOAD_RETRY_BASE_MS 5000
#define UPLOAD_RETRY_MAX_MS 60000

// ===================== DETECTION CONFIG =====================
#define DETECTION_COOLDOWN_MS 5000 // Min time between uploads
#define PREVIEW_FRAME_SIZE FRAMESIZE_VGA
#define PREVIEW_JPEG_QUALITY 12
#define UPLOAD_FRAME_SIZE FRAMESIZE_SVGA
#define UPLOAD_JPEG_QUALITY 10
#define LOW_LIGHT_LUMA_THRESHOLD 38
#define LOW_LIGHT_REPORT_LUMA_THRESHOLD 28
#define LOW_LIGHT_REPORT_HIT_PERCENT 75
#define LOW_LIGHT_TRIGGER_HITS 2
#define BRIGHT_LIGHT_LUMA_THRESHOLD 170
#define UPLOAD_EXPOSURE_SETTLE_MS 900
#define LOW_LIGHT_EXPOSURE_SETTLE_MS 1400
#define DETECT_PROFILE_SETTLE_MS 250
#define DETECT_LOW_LIGHT_PROFILE_SETTLE_MS 700
#define UPLOAD_STALE_FLUSH_FRAMES 3
#define FACE_DETECT_WINDOW_MS 6000  // Repeated detect window per request
#define FACE_DETECT_RETRY_GAP_MS 30 // Gap between detect attempts
#define BROWSER_STREAM_MAX_MS 20000 // Diagnostic stream window; face-check uses the same server
#define WIFI_BOOT_CONNECT_TIMEOUT_MS 120000
#define WIFI_RECONNECT_TIMEOUT_MS 20000
#define WIFI_MAINTAIN_INTERVAL_MS 8000
#define WIFI_RETRY_BASE_MS 2000
#define WIFI_RETRY_MAX_MS 30000
#define WIFI_RETRY_JITTER_MS 400

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
static bool cameraSleepMode = false;
static bool camRebootPending = false;
static unsigned long camRebootAtMs = 0;
static uint8_t lastDetectLuma = 255;
static bool lastFaceWindowLowLight = false;
static bool lastFaceWindowBright = false;

// Deferred upload flag — set by face-check handlers, executed in loop()
static bool pendingUpload = false;
static bool uploadInProgress = false;
static unsigned long uploadRetryAtMs = 0;
static uint8_t pendingUploadRetryCount = 0;
static bool payloadStoreReady = false;
static char pendingObjectPath[128] = "";
static const char *CAM_PENDING_FILE = "/pending.jpg";

static char currentDeliveryId[64] = "UNKNOWN_DELIVERY";

static Preferences camPrefs;
static bool camPrefsReady = false;
static const char *CAM_NS = "camQueue";
static const char *CAM_KEY_PENDING = "pending";
static const char *CAM_KEY_DELIVERY = "delId";
static const char *CAM_KEY_QUEUED_AT = "queuedAt";
static const char *CAM_KEY_RETRY = "retry";
static const char *CAM_KEY_OBJECT = "obj";

// WiFi reconnect scheduler (non-blocking)
static bool wifiConnectInProgress = false;
static unsigned long wifiConnectStartedAt = 0;
static unsigned long wifiNextAttemptAt = 0;
static unsigned long wifiBackoffMs = WIFI_RETRY_BASE_MS;
static unsigned long wifiReconnectStartAt = 0;
static unsigned long lastWifiReconnectLatencyMs = 0;

// Write telemetry (hourly)
static unsigned long camNvsWritesThisWindow = 0;
static unsigned long camSpiffsWritesThisWindow = 0;
static unsigned long camMetricsWindowStartedAt = 0;
static const unsigned long CAM_WRITE_METRICS_WINDOW_MS = 3600000UL;

// Deferred face-status client intake to avoid blocking wait loops
static WiFiClient pendingFaceClient;
static bool pendingFaceClientActive = false;
static unsigned long pendingFaceClientDeadlineAt = 0;

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

void printBrowserUrls() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[HTTP] Browser URLs unavailable: WiFi not connected"));
    return;
  }

  String baseUrl = String("http://") + WiFi.localIP().toString();
  Serial.println(F("[HTTP] Browser live view URLs:"));
  Serial.printf("       Live page : %s/\n", baseUrl.c_str());
  Serial.printf("       Stream    : %s/stream\n", baseUrl.c_str());
  Serial.printf("       Snapshot  : %s/snapshot\n", baseUrl.c_str());
  Serial.printf("       Face test : %s/face-test\n", baseUrl.c_str());
  Serial.printf("       Status    : %s/status\n", baseUrl.c_str());
}

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

bool uploadToSupabase(const uint8_t *data, size_t len,
                      const String &objectPath,
                      const char *proofRole = "full");

static uint32_t fnv1a32(const char *text) {
  const char *src = text ? text : "";
  uint32_t hash = 2166136261u;
  while (*src) {
    hash ^= (uint8_t)*src++;
    hash *= 16777619u;
  }
  return hash;
}

static void bumpNvsWrites(unsigned long count) {
  if (count == 0) return;
  camNvsWritesThisWindow += count;
}

static void bumpSpiffsWrites(unsigned long count) {
  if (count == 0) return;
  camSpiffsWritesThisWindow += count;
}

static void maybePublishWriteMetrics(unsigned long now) {
  if (camMetricsWindowStartedAt == 0) {
    camMetricsWindowStartedAt = now;
    return;
  }

  unsigned long elapsed = now - camMetricsWindowStartedAt;
  if (elapsed < CAM_WRITE_METRICS_WINDOW_MS) {
    return;
  }

  netLog("[METRICS] nvs_writes=%lu spiffs_writes=%lu window_s=%lu wifi_reconnect_last_ms=%lu\n",
         camNvsWritesThisWindow,
         camSpiffsWritesThisWindow,
         elapsed / 1000UL,
         lastWifiReconnectLatencyMs);

  camNvsWritesThisWindow = 0;
  camSpiffsWritesThisWindow = 0;
  camMetricsWindowStartedAt = now;
}

static void persistUploadIntent(bool pending) {
  if (!camPrefsReady) return;
  camPrefs.putBool(CAM_KEY_PENDING, pending);
  bumpNvsWrites(1);
  if (pending) {
    camPrefs.putString(CAM_KEY_DELIVERY, currentDeliveryId);
    camPrefs.putULong(CAM_KEY_QUEUED_AT, millis());
    camPrefs.putUChar(CAM_KEY_RETRY, pendingUploadRetryCount);
    camPrefs.putString(CAM_KEY_OBJECT, pendingObjectPath);
    bumpNvsWrites(4);
  } else {
    camPrefs.remove(CAM_KEY_DELIVERY);
    camPrefs.remove(CAM_KEY_QUEUED_AT);
    camPrefs.remove(CAM_KEY_RETRY);
    camPrefs.remove(CAM_KEY_OBJECT);
    bumpNvsWrites(4);
    pendingUploadRetryCount = 0;
    pendingObjectPath[0] = '\0';
  }
}

static void clearPendingPayloadStorage() {
  if (!payloadStoreReady) {
    pendingObjectPath[0] = '\0';
    return;
  }
  if (SPIFFS.exists(CAM_PENDING_FILE)) {
    if (SPIFFS.remove(CAM_PENDING_FILE)) {
      bumpSpiffsWrites(1);
    }
  }
  pendingObjectPath[0] = '\0';
}

static bool savePendingPayload(const uint8_t *data, size_t len,
                               const String &objectPath) {
  if (!payloadStoreReady || !data || len == 0) {
    return false;
  }

  File f = SPIFFS.open(CAM_PENDING_FILE, FILE_WRITE);
  if (!f) {
    netLog("[UPLOAD] Pending payload open fail\n");
    return false;
  }

  size_t written = f.write(data, len);
  f.close();
  if (written != len) {
    netLog("[UPLOAD] Pending payload write fail %u/%u\n",
           (unsigned int)written, (unsigned int)len);
    return false;
  }

  bumpSpiffsWrites(1);

  strncpy(pendingObjectPath, objectPath.c_str(), sizeof(pendingObjectPath) - 1);
  pendingObjectPath[sizeof(pendingObjectPath) - 1] = '\0';
  return true;
}

static bool uploadPendingPayloadFromStorage() {
  if (!payloadStoreReady || !SPIFFS.exists(CAM_PENDING_FILE) ||
      pendingObjectPath[0] == '\0') {
    return false;
  }

  File f = SPIFFS.open(CAM_PENDING_FILE, FILE_READ);
  if (!f) {
    return false;
  }

  size_t len = (size_t)f.size();
  if (len == 0) {
    f.close();
    return false;
  }

  uint8_t *buf = (uint8_t *)ps_malloc(len);
  if (!buf) {
    buf = (uint8_t *)malloc(len);
  }
  if (!buf) {
    f.close();
    return false;
  }

  size_t rd = f.read(buf, len);
  f.close();
  if (rd != len) {
    free(buf);
    return false;
  }

  bool ok = uploadToSupabase(buf, len, String(pendingObjectPath), "full");
  free(buf);
  if (ok) {
    clearPendingPayloadStorage();
  }
  return ok;
}

static void initUploadIntentStore() {
  camPrefsReady = camPrefs.begin(CAM_NS, false);
  if (!camPrefsReady) {
    Serial.println(F("[UPLOAD] NVS unavailable; replay disabled"));
    return;
  }

  if (!camPrefs.getBool(CAM_KEY_PENDING, false)) {
    return;
  }

  String restoredDelivery = camPrefs.getString(CAM_KEY_DELIVERY, "UNKNOWN_DELIVERY");
  if (restoredDelivery.length() > 0 && restoredDelivery.length() < sizeof(currentDeliveryId)) {
    strncpy(currentDeliveryId, restoredDelivery.c_str(), sizeof(currentDeliveryId) - 1);
    currentDeliveryId[sizeof(currentDeliveryId) - 1] = '\0';
  }

  pendingUploadRetryCount =
      (uint8_t)camPrefs.getUChar(CAM_KEY_RETRY, 0);

  String obj = camPrefs.getString(CAM_KEY_OBJECT, "");
  if (obj.length() > 0 && obj.length() < sizeof(pendingObjectPath)) {
    strncpy(pendingObjectPath, obj.c_str(), sizeof(pendingObjectPath) - 1);
    pendingObjectPath[sizeof(pendingObjectPath) - 1] = '\0';
  }

  pendingUpload = true;
  unsigned long backoff = 0;
  if (pendingUploadRetryCount > 0) {
    backoff = UPLOAD_RETRY_BASE_MS;
    for (uint8_t i = 1; i < pendingUploadRetryCount; i++) {
      if (backoff >= (UPLOAD_RETRY_MAX_MS / 2)) {
        backoff = UPLOAD_RETRY_MAX_MS;
        break;
      }
      backoff *= 2;
    }
    if (backoff > UPLOAD_RETRY_MAX_MS) {
      backoff = UPLOAD_RETRY_MAX_MS;
    }
  }
  uploadRetryAtMs = millis() + backoff;
  Serial.printf("[UPLOAD] Restored pending intent for %s retry=%u\n",
                currentDeliveryId, (unsigned int)pendingUploadRetryCount);
}

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
  uploadRetryAtMs = now;
  pendingUploadRetryCount = 0;
  pendingObjectPath[0] = '\0';
  persistUploadIntent(true);
  netLog("[UPLOAD] Queued from %s\n", source);
  return true;
}

void printCommands() {
  Serial.println(F("Commands:"));
  Serial.println(F("  c = force capture & upload (bypass detection)"));
  Serial.println(F("  h or ? = show commands"));
}

bool hasUnsetConfig() {
  return strlen(SUPABASE_URL) == 0 || strlen(SUPABASE_API_KEY) == 0 || HOTSPOT_COUNT == 0;
}

const char *getSupabaseBearerToken() {
  if (strlen(SUPABASE_SERVICE_ROLE_KEY) > 0) {
    return SUPABASE_SERVICE_ROLE_KEY;
  }
  return SUPABASE_API_KEY;
}

// ===================== WIFI =====================

void scanForProxyAP() {
  Serial.println(F("[WIFI] Scanning for configured hotspots..."));
  int n = WiFi.scanNetworks();

  int bestIdx = -1;
  int bestRssi = -999;
  int bestCredential = -1;
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    Serial.printf("[WIFI]   %s (%d dBm)\n", ssid.c_str(), WiFi.RSSI(i));
  }

  for (uint8_t h = 0; h < HOTSPOT_COUNT && bestIdx < 0; h++) {
    if (!HOTSPOTS[h].ssid || HOTSPOTS[h].ssid[0] == '\0') continue;
    for (int i = 0; i < n; i++) {
      if (WiFi.SSID(i) == HOTSPOTS[h].ssid && WiFi.RSSI(i) > bestRssi) {
        bestRssi = WiFi.RSSI(i);
        bestIdx = i;
        bestCredential = h;
      }
    }
  }

  if (bestIdx >= 0 && bestCredential >= 0) {
    String ssid = WiFi.SSID(bestIdx);
    strncpy(WIFI_SSID, ssid.c_str(), sizeof(WIFI_SSID) - 1);
    WIFI_SSID[sizeof(WIFI_SSID) - 1] = '\0';
    strncpy(WIFI_PASSWORD, HOTSPOTS[bestCredential].password,
            sizeof(WIFI_PASSWORD) - 1);
    WIFI_PASSWORD[sizeof(WIFI_PASSWORD) - 1] = '\0';

    snprintf(DEVICE_ID, sizeof(DEVICE_ID), "OV3660_CAM_001");

    Serial.printf("[WIFI] Selected hotspot: %s (%d dBm) -> %s\n",
                  WIFI_SSID, bestRssi, DEVICE_ID);
  } else {
    Serial.printf("[WIFI] No configured hotspot found (%d APs scanned)\n", n);
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

bool discoverProxyUdp(unsigned long timeoutMs = 1500) {
  if (WiFi.status() != WL_CONNECTED) return false;
  WiFiUDP discovery;
  if (!discovery.begin(0)) {
    Serial.println(F("[DISCOVERY] UDP begin failed"));
    return false;
  }
  IPAddress broadcast((uint32_t)WiFi.localIP() | ~(uint32_t)WiFi.subnetMask());

  unsigned long deadline = millis() + timeoutMs;
  char packet[96];
  unsigned long nextQueryAt = 0;
  
  while (millis() < deadline) {
    unsigned long now = millis();
    if (now >= nextQueryAt) {
      discovery.beginPacket(broadcast, PROXY_DISCOVERY_PORT);
      discovery.print(PROXY_DISCOVERY_QUERY);
      discovery.endPacket();
      nextQueryAt = now + 250; // Resend every 250ms
    }

    int size = discovery.parsePacket();
    if (size > 0) {
      int len = discovery.read(packet, sizeof(packet) - 1);
      packet[len] = '\0';
      if (strncmp(packet, PROXY_DISCOVERY_REPLY,
                  strlen(PROXY_DISCOVERY_REPLY)) == 0) {
        
        String payload = String(packet + strlen(PROXY_DISCOVERY_REPLY));
        int firstColon = payload.indexOf(':');
        int secondColon = firstColon >= 0 ? payload.indexOf(':', firstColon + 1) : -1;
        String replyIp = firstColon >= 0 ? payload.substring(0, firstColon) : "";
        int replyPort = secondColon > firstColon
            ? payload.substring(firstColon + 1, secondColon).toInt()
            : PROXY_PORT;

        IPAddress ip;
        if (replyIp.length() == 0 || !ip.fromString(replyIp)) {
          ip = discovery.remoteIP();
          replyIp = ip.toString();
        }

        strncpy(PROXY_HOST, replyIp.c_str(), sizeof(PROXY_HOST) - 1);
        PROXY_HOST[sizeof(PROXY_HOST) - 1] = '\0';
        Serial.printf("[DISCOVERY] Proxy at %s:%d (%s)\n", PROXY_HOST, replyPort, packet);

        discovery.beginPacket(ip, PROXY_DISCOVERY_PORT);
        discovery.print("SMART_TOP_BOX_CAM:");
        discovery.print(DEVICE_ID);
        discovery.endPacket();
        
        discovery.stop();
        return true;
      }
    }
    delay(20);
  }
  discovery.stop();
  Serial.println(F("[DISCOVERY] Proxy not found"));
  return false;
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

  // Use DHCP from the phone/router hotspot. LilyGO learns this camera IP from
  // /face-status and /upload traffic.
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);

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
    discoverProxyUdp();
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
  unsigned long now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    if (PROXY_HOST[0] == '\0') {
      discoverProxyUdp(250);
    }
    if (wifiReconnectStartAt != 0) {
      lastWifiReconnectLatencyMs = now - wifiReconnectStartAt;
      netLog("[WIFI] Recovered in %lums\n", lastWifiReconnectLatencyMs);
      wifiReconnectStartAt = 0;
    }
    wifiConnectInProgress = false;
    wifiBackoffMs = WIFI_RETRY_BASE_MS;
    wifiNextAttemptAt = now + WIFI_RETRY_BASE_MS;
    return;
  }

  if (wifiReconnectStartAt == 0) {
    wifiReconnectStartAt = now;
  }

  if (wifiConnectInProgress) {
    if ((now - wifiConnectStartedAt) < WIFI_RECONNECT_TIMEOUT_MS) {
      return;
    }

    wl_status_t failedStatus = WiFi.status();
    netLog("[WIFI] Attempt timeout (%s)\n", wifiStatusText(failedStatus));
    WiFi.disconnect(false, false);
    wifiConnectInProgress = false;

    unsigned long jitter = (unsigned long)random(0, WIFI_RETRY_JITTER_MS + 1);
    wifiNextAttemptAt = now + wifiBackoffMs + jitter;
    if (wifiBackoffMs < WIFI_RETRY_MAX_MS) {
      wifiBackoffMs *= 2;
      if (wifiBackoffMs > WIFI_RETRY_MAX_MS) {
        wifiBackoffMs = WIFI_RETRY_MAX_MS;
      }
    }
    return;
  }

  if (now < wifiNextAttemptAt) {
    return;
  }

  if (WIFI_SSID[0] == '\0') {
    scanForProxyAP();
  }
  if (WIFI_SSID[0] == '\0') {
    wifiNextAttemptAt = now + wifiBackoffMs;
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.disconnect(false, false);

  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);

  netLog("[WIFI] Non-blocking reconnect attempt to '%s'\n", WIFI_SSID);
  WiFi.begin((const char *)WIFI_SSID, WIFI_PASSWORD);
  wifiConnectInProgress = true;
  wifiConnectStartedAt = now;
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

void applySensorTuning(sensor_t *sensor, bool uploadMode, bool lowLightMode,
                       bool brightLightMode = false) {
  if (!sensor) {
    return;
  }

  // Camera is mounted inverted in the box, so rotate the image 180 degrees.
  if (sensor->set_vflip) sensor->set_vflip(sensor, 1);
  if (sensor->set_hmirror) sensor->set_hmirror(sensor, 1);
  
  // Keep exposure and gain automatic, but bias normal/low-light scenes brighter.
  if (sensor->set_brightness) sensor->set_brightness(sensor, lowLightMode ? 2 : (brightLightMode ? -2 : 2));
  // Contrast: Higher contrast helps Haar/CNN extract facial edges reliably
  if (sensor->set_contrast) sensor->set_contrast(sensor, 2);
  // Saturation: Lower saturation helps focus on luma/edges rather than color noise
  if (sensor->set_saturation) sensor->set_saturation(sensor, brightLightMode ? -2 : (lowLightMode ? -1 : 0));
  if (sensor->set_sharpness) sensor->set_sharpness(sensor, uploadMode ? 3 : 1);
  
  if (sensor->set_exposure_ctrl) sensor->set_exposure_ctrl(sensor, 1);
  if (sensor->set_aec2) sensor->set_aec2(sensor, 1);
  // AE Level is the auto-exposure bias. Higher brightens dark live view.
  if (sensor->set_ae_level) sensor->set_ae_level(sensor, lowLightMode ? 2 : (brightLightMode ? -2 : 2));
  if (sensor->set_gain_ctrl) sensor->set_gain_ctrl(sensor, 1);
  // AGC is the closest OV3660 control to auto ISO. Let it climb in normal rooms.
  if (sensor->set_agc_gain) sensor->set_agc_gain(sensor, lowLightMode ? 22 : (brightLightMode ? 2 : 14));
  if (sensor->set_gainceiling) {
    sensor->set_gainceiling(sensor, lowLightMode ? GAINCEILING_64X
                                                 : (brightLightMode ? GAINCEILING_4X
                                                                    : GAINCEILING_32X));
  }
  
  if (sensor->set_whitebal) sensor->set_whitebal(sensor, 1);
  if (sensor->set_awb_gain) sensor->set_awb_gain(sensor, 1);
  if (sensor->set_wb_mode) sensor->set_wb_mode(sensor, brightLightMode ? 1 : 0);
  
  // Enable built-in ISP corrections to clean up the image
  if (sensor->set_bpc) sensor->set_bpc(sensor, 1);         // Black pixel correction
  if (sensor->set_wpc) sensor->set_wpc(sensor, 1);         // White pixel correction
  if (sensor->set_raw_gma) sensor->set_raw_gma(sensor, 1); // Gamma correction
  if (sensor->set_lenc) sensor->set_lenc(sensor, 1);       // Lens correction
}

uint8_t estimateRgb565Luma(const camera_fb_t *fb) {
  if (!fb || fb->format != PIXFORMAT_RGB565 || !fb->buf || fb->len < 2) {
    return 255;
  }

  const size_t pixelCount = (size_t)fb->width * (size_t)fb->height;
  if (pixelCount == 0 || fb->len < pixelCount * 2) {
    return 255;
  }

  size_t step = pixelCount / 384;
  if (step < 1) {
    step = 1;
  }
  uint32_t sum = 0;
  uint16_t samples = 0;

  for (size_t p = 0; p < pixelCount && samples < 512; p += step) {
    size_t offset = p * 2;
    uint16_t raw = ((uint16_t)fb->buf[offset + 1] << 8) | fb->buf[offset];
    uint8_t r = (uint8_t)(((raw >> 11) & 0x1F) * 255 / 31);
    uint8_t g = (uint8_t)(((raw >> 5) & 0x3F) * 255 / 63);
    uint8_t b = (uint8_t)((raw & 0x1F) * 255 / 31);
    sum += (uint32_t)r * 30 + (uint32_t)g * 59 + (uint32_t)b * 11;
    samples++;
  }

  if (samples == 0) {
    return 255;
  }
  return (uint8_t)((sum / samples) / 100);
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
    applySensorTuning(sensor, false, false);
  }

  cameraInDetectMode = true;
  Serial.println(F("Camera initialised (QVGA RGB565 for ESP-Face)."));
  return true;
}

bool switchToUploadMode(framesize_t frameSize, int jpegQuality,
                        bool lowLightMode, bool brightLightMode = false) {
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
  config.frame_size = frameSize;
  config.jpeg_quality = jpegQuality;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.fb_count = 2; // Need 2 for JPEG DMA

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Upload camera init failed: 0x%x\n", err);
    cameraInDetectMode = false;
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  applySensorTuning(sensor, true, lowLightMode, brightLightMode);

  cameraInDetectMode = false;
  return true;
}

bool switchToDetectMode() {
  if (cameraInDetectMode) {
    return true;
  }

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

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Detect camera init failed: 0x%x\n", err);
    cameraInDetectMode = false;
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  applySensorTuning(sensor, false, false);

  cameraInDetectMode = true;
  return true;
}

// ===================== DETECTION =====================

/**
 * Returns true if a face is detected
 */
bool runFaceDetection() {
  if (!s1) {
    Serial.println(F("Face detect skipped: model not allocated."));
    return false;
  }

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

  lastDetectLuma = estimateRgb565Luma(fb);

  if (fb->format != PIXFORMAT_RGB565) {
    Serial.println(F("Face detect skipped: frame is not RGB565."));
    esp_camera_fb_return(fb);
    return false;
  }

  int fbWidth  = (int)fb->width;
  int fbHeight = (int)fb->height;

  // Convert RGB565 to RGB888 to guarantee 100% correct color mapping for ESP-DL inference.
  // Direct uint16_t casting of RGB565 sometimes suffers from byte-order/endianness issues on OV3660.
  // We allocate in PSRAM since 320x240x3 = 230KB.
  uint8_t *rgb888_buf = (uint8_t *)heap_caps_malloc(fbWidth * fbHeight * 3, MALLOC_CAP_SPIRAM);
  bool detected = false;
  
  if (rgb888_buf) {
      if (fmt2rgb888(fb->buf, fb->len, fb->format, rgb888_buf)) {
          std::list<dl::detect::result_t> &results =
              s1->infer(rgb888_buf, {fbHeight, fbWidth, 3});
          detected = results.size() > 0;
      }
      heap_caps_free(rgb888_buf);
  } else {
      Serial.println(F("Failed to alloc RGB888 buffer, falling back to RGB565..."));
      std::list<dl::detect::result_t> &results =
          s1->infer((uint16_t *)fb->buf, {fbHeight, fbWidth, 3});
      detected = results.size() > 0;
  }

  esp_camera_fb_return(fb);
  return detected;

}

bool runFaceDetectionWindow(unsigned long windowMs, bool *lowLightOut = nullptr) {
  unsigned long detectStart = millis();
  uint32_t lumaSum = 0;
  uint16_t lumaSamples = 0;
  uint16_t lowLightHits = 0;
  uint16_t brightLightHits = 0;
  uint8_t profileIndex = 255;
  const char *profileName = "normal";

  sensor_t *sensor = esp_camera_sensor_get();
  applySensorTuning(sensor, false, false, false);

  while (millis() - detectStart < windowMs) {
    unsigned long elapsed = millis() - detectStart;
    uint8_t nextProfile =
        elapsed < (windowMs * 25UL / 100UL)
            ? 0
            : (elapsed < (windowMs * 85UL / 100UL) ? 2 : 1);

    if (lowLightHits >= LOW_LIGHT_TRIGGER_HITS &&
        elapsed < (windowMs * 85UL / 100UL)) {
      nextProfile = 2;
    }

    if (nextProfile != profileIndex) {
      profileIndex = nextProfile;
      profileName = profileIndex == 0 ? "normal" :
          (profileIndex == 1 ? "bright-glare" : "low-light");
      applySensorTuning(sensor, false, profileIndex == 2, profileIndex == 1);
      netLog("[FACE] Detection profile -> %s\n", profileName);
      delay(profileIndex == 2 ? DETECT_LOW_LIGHT_PROFILE_SETTLE_MS
                               : DETECT_PROFILE_SETTLE_MS);
    }

    if (runFaceDetection()) {
      scanCount++;
      lastFaceWindowLowLight = profileIndex == 2;
      lastFaceWindowBright = profileIndex == 1;
      netLog("[FACE] Detected using %s profile (luma=%u)\n", profileName,
             lastDetectLuma);
      if (lowLightOut) {
        *lowLightOut = false;
      }
      return true;
    }
    lumaSum += lastDetectLuma;
    lumaSamples++;
    if (lastDetectLuma <= LOW_LIGHT_LUMA_THRESHOLD) {
      lowLightHits++;
    }
    if (lastDetectLuma >= BRIGHT_LIGHT_LUMA_THRESHOLD) {
      brightLightHits++;
    }
    scanCount++;
    delay(FACE_DETECT_RETRY_GAP_MS);
  }

  uint8_t avgLuma = lumaSamples > 0 ? (uint8_t)(lumaSum / lumaSamples) : 255;
  bool reportLowLight =
      lumaSamples > 0 &&
      avgLuma <= LOW_LIGHT_REPORT_LUMA_THRESHOLD &&
      ((uint32_t)lowLightHits * 100UL) >=
          ((uint32_t)lumaSamples * LOW_LIGHT_REPORT_HIT_PERCENT);
  lastFaceWindowLowLight =
      reportLowLight;
  lastFaceWindowBright =
      lumaSamples > 0 &&
      (avgLuma >= BRIGHT_LIGHT_LUMA_THRESHOLD || brightLightHits >= (lumaSamples / 2));
  if (lowLightOut) {
    *lowLightOut = lastFaceWindowLowLight;
  }
  if (lastFaceWindowLowLight) {
    netLog("[FACE] Low light window avg=%u hits=%u/%u\n", avgLuma,
           (unsigned int)lowLightHits, (unsigned int)lumaSamples);
  } else if (lastFaceWindowBright) {
    netLog("[FACE] Bright/glare window avg=%u hits=%u/%u\n", avgLuma,
           (unsigned int)brightLightHits, (unsigned int)lumaSamples);
  }
  applySensorTuning(sensor, false, false, false);
  return false;
}

// ===================== UPLOAD =====================

String makeObjectPath(const char *proofRole = "full") {
  char filename[128];
  snprintf(filename, sizeof(filename), "%s_%s_%lu.jpg", currentDeliveryId,
           (proofRole && proofRole[0] != '\0') ? proofRole : "full", millis());
  return String("deliveries/") + DEVICE_ID + "/" + filename;
}

bool uploadToSupabase(const uint8_t *data, size_t len,
                      const String &objectPath,
                      const char *proofRole) {
  if (WiFi.status() != WL_CONNECTED) {
    maintainWiFiConnection();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println(F("Upload skipped: WiFi (proxy AP) disconnected."));
      return false;
    }
  }
  if (PROXY_HOST[0] == '\0' && !discoverProxyUdp()) {
    Serial.println(F("Upload skipped: proxy discovery failed."));
    return false;
  }

  // POST image data to the LilyGO proxy which relays it to Supabase.
  HTTPClient http;
  String endpoint =
      String("http://") + PROXY_HOST + ":" + String(PROXY_PORT) + PROXY_PATH;

  http.begin(endpoint);
  http.setTimeout(65000); // Max safe uint16_t value (~65s) to avoid overflow to 24.4s
  http.addHeader("Content-Type", "image/jpeg");
  // Tell the proxy the Supabase storage path for this image
  http.addHeader("X-Object-Path", objectPath);
  http.addHeader("X-Proof-Role",
                 (proofRole && proofRole[0] != '\0') ? proofRole : "full");
  char idempotencyKey[48];
  snprintf(idempotencyKey, sizeof(idempotencyKey), "%08lx-%u",
           (unsigned long)fnv1a32(objectPath.c_str()), (unsigned int)len);
  http.addHeader("X-Idempotency-Key", idempotencyKey);

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

bool captureHighResAndUpload() {
  unsigned long t0 = millis();
  bool lowLightMode =
      lastFaceWindowLowLight || lastDetectLuma <= LOW_LIGHT_LUMA_THRESHOLD;
  bool brightLightMode = !lowLightMode &&
      (lastFaceWindowBright || lastDetectLuma >= BRIGHT_LIGHT_LUMA_THRESHOLD);

  switchToUploadMode(PREVIEW_FRAME_SIZE, PREVIEW_JPEG_QUALITY, lowLightMode,
                     brightLightMode);

  unsigned long settleMs = lowLightMode ? LOW_LIGHT_EXPOSURE_SETTLE_MS
                                        : UPLOAD_EXPOSURE_SETTLE_MS;
  delay(settleMs);

  for (uint8_t i = 0; i < UPLOAD_STALE_FLUSH_FRAMES; i++) {
    camera_fb_t *stale = esp_camera_fb_get();
    if (stale) {
      esp_camera_fb_return(stale);
    }
    delay(70);
  }

  camera_fb_t *preview = esp_camera_fb_get();

  if (preview && preview->len < 4000) {
    esp_camera_fb_return(preview);
    delay(50);
    preview = esp_camera_fb_get();
  }

  if (!preview) {
    Serial.println(F("ERROR: Proof preview capture failed."));
    switchToDetectMode();
    return false;
  }

  Serial.printf("Captured preview JPEG: %u bytes in %lu ms\n", preview->len,
                millis() - t0);

  // Give the solenoid its power window before modem upload spikes begin.
  delay(1500);

  String previewObjectPath = makeObjectPath("preview");
  bool previewUploaded =
      uploadToSupabase(preview->buf, preview->len, previewObjectPath, "preview");
  esp_camera_fb_return(preview);

  if (!previewUploaded) {
    switchToDetectMode();
    return false;
  }

  switchToUploadMode(UPLOAD_FRAME_SIZE, UPLOAD_JPEG_QUALITY, lowLightMode,
                     brightLightMode);
  delay(settleMs);
  for (uint8_t i = 0; i < UPLOAD_STALE_FLUSH_FRAMES; i++) {
    camera_fb_t *stale = esp_camera_fb_get();
    if (stale) {
      esp_camera_fb_return(stale);
    }
    delay(70);
  }

  camera_fb_t *photo = esp_camera_fb_get();
  if (photo && photo->len < 5000) {
    esp_camera_fb_return(photo);
    delay(50);
    photo = esp_camera_fb_get();
  }

  if (!photo) {
    Serial.println(F("ERROR: Hi-res capture failed after preview upload."));
    switchToDetectMode();
    return false;
  }

  Serial.printf("Captured full JPEG: %u bytes in %lu ms\n", photo->len,
                millis() - t0);

  String objectPath =
      (pendingObjectPath[0] != '\0') ? String(pendingObjectPath)
                                     : makeObjectPath("full");
  if (payloadStoreReady) {
    if (!savePendingPayload(photo->buf, photo->len, objectPath)) {
      netLog("[UPLOAD] Warning: payload snapshot not persisted\n");
    }
  }

  bool uploaded = uploadToSupabase(photo->buf, photo->len, objectPath, "full");

  esp_camera_fb_return(photo);
  if (uploaded) {
    lastUploadAt = millis();
    clearPendingPayloadStorage();
  }

  // Switch back
  switchToDetectMode();
  return uploaded;
}

// ===================== MAIN LOOP =====================

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char command = (char)Serial.read();

    if (command == 'c' || command == 'C') {
      Serial.println(F("Manual capture (bypass detection)."));
      if (!captureHighResAndUpload()) {
        pendingUpload = true;
        uploadRetryAtMs = millis() + UPLOAD_RETRY_BASE_MS;
        if (pendingUploadRetryCount == 0) {
          pendingUploadRetryCount = 1;
        }
        persistUploadIntent(true);
        Serial.println(F("Manual upload failed; intent kept for retry."));
      }
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
  payloadStoreReady = SPIFFS.begin(true);
  if (!payloadStoreReady) {
    Serial.println(F("[UPLOAD] SPIFFS unavailable; payload replay disabled"));
  }
  initUploadIntentStore();
  initTime();

  if (!initCamera()) {
    Serial.println(F("Camera setup failed. Halting."));
    while (true)
      delay(1000);
  }

// Allocate ESP-Face model
    // 0.3F scale allows detecting faces that are very close (taking up the entire frame)
    s1 = new HumanFaceDetectMSR01(0.1F, 0.5F, 10, 0.3F);

  // Start HTTP server for /face-status endpoint
  faceServer.begin();
  Serial.println(F("[HTTP] Face-status and browser live-view server on port 80"));
  printBrowserUrls();

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

bool setCameraSleepMode(bool sleepMode) {
  if (sleepMode == cameraSleepMode) {
    return true;
  }

  if (sleepMode) {
    esp_camera_deinit();
    pinMode(PWDN_GPIO_NUM, OUTPUT);
    digitalWrite(PWDN_GPIO_NUM, HIGH);
    pendingUpload = false;
    uploadInProgress = false;
    uploadRetryAtMs = 0;
    pendingUploadRetryCount = 0;
    clearPendingPayloadStorage();
    persistUploadIntent(false);
    cameraSleepMode = true;
    cameraInDetectMode = false;
    netLog("[POWER] Camera sleep mode enabled\n");
    return true;
  }

  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, LOW);
  delay(20);
  bool ok = initCamera();
  cameraSleepMode = !ok;
  netLog("[POWER] Camera wake %s\n", ok ? "OK" : "FAIL");
  return ok;
}

// ===================== FACE-STATUS HTTP HANDLER =====================
// Responds to GET /face-status from the GPS/LTE proxy board.
// Runs face detection on demand. Responds IMMEDIATELY, then defers upload
// to loop() to avoid deadlock (CAM uploads back to the same proxy).
void sendBrowserLivePage(WiFiClient &client) {
  client.print(F(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/html; charset=utf-8\r\n"
      "Cache-Control: no-store\r\n"
      "Connection: close\r\n\r\n"
      "<!doctype html><html><head><meta name='viewport' "
      "content='width=device-width,initial-scale=1'>"
      "<title>ESP32-CAM Live View</title>"
      "<style>"
      "body{margin:0;background:#101418;color:#edf2f4;font-family:Arial,"
      "sans-serif}"
      "main{max-width:980px;margin:0 auto;padding:18px}"
      "header{display:flex;gap:12px;align-items:center;justify-content:"
      "space-between;flex-wrap:wrap;margin-bottom:14px}"
      "h1{font-size:22px;margin:0}.pill{font-size:13px;color:#b8c4cc}"
      ".viewer{background:#000;border:1px solid #303941;border-radius:8px;"
      "overflow:hidden;aspect-ratio:4/3;display:flex;align-items:center;"
      "justify-content:center}"
      "img{width:100%;height:100%;object-fit:contain;display:block}"
      ".actions{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}"
      "a,button{background:#1f6feb;color:white;border:0;border-radius:6px;"
      "padding:10px 12px;text-decoration:none;font-size:14px}"
      "button{cursor:pointer}.secondary{background:#30363d}"
      "pre{white-space:pre-wrap;background:#161b22;border:1px solid #30363d;"
      "border-radius:8px;padding:12px;color:#c9d1d9}"
      "</style></head><body><main>"
      "<header><div><h1>ESP32-CAM Live View</h1>"
      "<div class='pill'>Open this from a device connected to the same "
      "hotspot as the camera.</div></div>"
      "<a class='secondary' href='/status'>Status</a></header>"
      "<div class='viewer'><img id='cam' src='/stream' "
      "alt='ESP32-CAM MJPEG stream'></div>"
      "<div class='actions'>"
      "<a href='/stream'>Open raw stream</a>"
      "<a class='secondary' href='/snapshot'>Open snapshot</a>"
      "<a class='secondary' href='/face-test'>Test face detection</a>"
      "<button onclick=\"document.getElementById('cam').src='/stream?ts='+Date.now()\">"
      "Reconnect stream</button>"
      "</div><pre>The stream is for diagnostics and auto-closes after 20 seconds. "
      "Close it before face verification so the detector can use the camera.</pre>"
      "</main></body></html>"));
}

void sendBrowserStatus(WiFiClient &client) {
  String body = "{\n";
  body += "  \"device_id\": \"" + String(DEVICE_ID) + "\",\n";
  body += "  \"wifi_ssid\": \"" + String(WIFI_SSID) + "\",\n";
  body += "  \"ip\": \"" + WiFi.localIP().toString() + "\",\n";
  body += "  \"rssi_dbm\": " + String(WiFi.RSSI()) + ",\n";
  body += "  \"camera_sleep\": " + String(cameraSleepMode ? "true" : "false") + ",\n";
  body += "  \"camera_mode\": \"" + String(cameraInDetectMode ? "detect_rgb565" : "jpeg") + "\",\n";
  body += "  \"proxy_host\": \"" + String(PROXY_HOST) + "\",\n";
  body += "  \"pending_upload\": " + String(pendingUpload ? "true" : "false") + "\n";
  body += "}\n";

  client.print(F("HTTP/1.1 200 OK\r\n"));
  client.print(F("Content-Type: application/json\r\n"));
  client.printf("Content-Length: %u\r\n", body.length());
  client.print(F("Cache-Control: no-store\r\n"));
  client.print(F("Connection: close\r\n\r\n"));
  client.print(body);
}

void sendFaceTestResult(WiFiClient &client, bool detected, bool lowLight,
                        unsigned long elapsedMs, const char *error = nullptr) {
  String body = "{\n";
  body += "  \"detected\": " + String(detected ? "true" : "false") + ",\n";
  body += "  \"result\": \"" + String(error ? error : (detected ? "FACE_OK" : (lowLight ? "NO_FACE_LOW_LIGHT" : "NO_FACE"))) + "\",\n";
  body += "  \"elapsed_ms\": " + String(elapsedMs) + ",\n";
  body += "  \"last_luma\": " + String(lastDetectLuma) + ",\n";
  body += "  \"low_light\": " + String(lowLight ? "true" : "false") + ",\n";
  body += "  \"bright_light\": " + String(lastFaceWindowBright ? "true" : "false") + ",\n";
  body += "  \"camera_sleep\": " + String(cameraSleepMode ? "true" : "false") + ",\n";
  body += "  \"camera_mode\": \"" + String(cameraInDetectMode ? "detect_rgb565" : "jpeg") + "\",\n";
  body += "  \"scan_count\": " + String(scanCount) + "\n";
  body += "}\n";

  client.print(F("HTTP/1.1 200 OK\r\n"));
  client.print(F("Content-Type: application/json\r\n"));
  client.printf("Content-Length: %u\r\n", body.length());
  client.print(F("Cache-Control: no-store\r\n"));
  client.print(F("Connection: close\r\n\r\n"));
  client.print(body);
}

void handleFaceStatusClient() {
  if (!pendingFaceClientActive) {
    WiFiClient incoming = faceServer.available();
    if (!incoming) {
      return;
    }
    pendingFaceClient = incoming;
    pendingFaceClientActive = true;
    pendingFaceClientDeadlineAt = millis() + 500;
    Serial.println(F("[HTTP] Face-status request queued"));
  }

  WiFiClient &client = pendingFaceClient;
  if (!client.connected()) {
    client.stop();
    pendingFaceClientActive = false;
    return;
  }

  if (!client.available()) {
    if (millis() < pendingFaceClientDeadlineAt) {
      return;
    }
    client.stop();
    pendingFaceClientActive = false;
    return;
  }

  client.setTimeout(50);
  Serial.println(F("[HTTP] Face-status request received"));

  String requestLine = client.readStringUntil('\n');
  requestLine.trim();

  if (requestLine.startsWith("GET / ") || requestLine.startsWith("GET /live")) {
    while (client.available()) {
      String line = client.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) {
        break;
      }
    }

    sendBrowserLivePage(client);
    client.flush();
    yield();
    client.stop();
    pendingFaceClientActive = false;
    return;
  }

  if (requestLine.startsWith("GET /status")) {
    while (client.available()) {
      String line = client.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) {
        break;
      }
    }

    sendBrowserStatus(client);
    client.flush();
    yield();
    client.stop();
    pendingFaceClientActive = false;
    return;
  }

  if (requestLine.startsWith("GET /face-test")) {
    while (client.available()) {
      String line = client.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) {
        break;
      }
    }

    unsigned long startedAt = millis();
    bool lowLight = false;
    bool detected = false;
    const char *error = nullptr;

    if (cameraSleepMode) {
      error = "CAMERA_SLEEP";
    } else if (!cameraInDetectMode && !switchToDetectMode()) {
      error = "CAMERA_DETECT_MODE_FAILED";
    } else {
      detected = runFaceDetectionWindow(FACE_DETECT_WINDOW_MS, &lowLight);
    }

    sendFaceTestResult(client, detected, lowLight, millis() - startedAt, error);
    client.flush();
    yield();
    client.stop();
    pendingFaceClientActive = false;
    return;
  }

  if (requestLine.startsWith("GET /cam-power")) {
    int qIdx = requestLine.indexOf("?mode=");
    String mode = "";
    if (qIdx >= 0) {
      int spIdx = requestLine.indexOf(' ', qIdx);
      if (spIdx > qIdx) {
        mode = requestLine.substring(qIdx + 6, spIdx);
      }
    }
    mode.toLowerCase();

    while (client.available()) {
      String line = client.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) {
        break;
      }
    }

    String result = "CAM_MODE_INVALID";
    if (mode == "sleep") {
      result = setCameraSleepMode(true) ? "CAM_SLEEP_OK" : "CAM_SLEEP_FAIL";
    } else if (mode == "wake") {
      result = setCameraSleepMode(false) ? "CAM_WAKE_OK" : "CAM_WAKE_FAIL";
    } else if (mode == "reboot") {
      result = "CAM_REBOOTING";
      camRebootPending = true;
      camRebootAtMs = millis() + 200;
    }

    char resp[160];
    snprintf(resp, sizeof(resp),
             "HTTP/1.1 200 OK\r\n"
             "Content-Type: text/plain\r\n"
             "Content-Length: %d\r\n"
             "Connection: close\r\n\r\n"
             "%s",
             (int)result.length(), result.c_str());
    client.print(resp);
    client.flush();
    yield();
    client.stop();
    pendingFaceClientActive = false;
    return;
  }

  if (requestLine.startsWith("GET /preview") || requestLine.startsWith("GET /snapshot")) {
    netLog("[HTTP] Serving browser preview snapshot\n");

    // Read and discard HTTP headers
    while (client.available()) {
      String line = client.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) {
        break;
      }
    }

    if (cameraSleepMode) {
      const char *msg = "Camera is in sleep mode.";
      char resp[160];
      snprintf(resp, sizeof(resp),
               "HTTP/1.1 503 Service Unavailable\r\n"
               "Content-Type: text/plain\r\n"
               "Content-Length: %d\r\n"
               "Connection: close\r\n\r\n"
               "%s",
               (int)strlen(msg), msg);
      client.print(resp);
      client.flush();
      yield();
      client.stop();
      pendingFaceClientActive = false;
      return;
    }

    // Switch to JPEG mode temporarily for a clean snapshot
    if (!switchToUploadMode(FRAMESIZE_VGA, 12, false, false)) {
      const char *msg = "Camera JPEG mode failed.";
      char resp[160];
      snprintf(resp, sizeof(resp),
               "HTTP/1.1 500 Internal Server Error\r\n"
               "Content-Type: text/plain\r\n"
               "Content-Length: %d\r\n"
               "Connection: close\r\n\r\n"
               "%s",
               (int)strlen(msg), msg);
      client.print(resp);
      client.flush();
      yield();
      client.stop();
      pendingFaceClientActive = false;
      return;
    }
    delay(400); // Allow AE/AGC to adjust to current lighting

    // Flush stale frames
    for (uint8_t i = 0; i < 2; i++) {
      camera_fb_t *stale = esp_camera_fb_get();
      if (stale) {
        esp_camera_fb_return(stale);
      }
      delay(50);
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      const char *msg = "Failed to capture frame.";
      char resp[160];
      snprintf(resp, sizeof(resp),
               "HTTP/1.1 500 Internal Server Error\r\n"
               "Content-Type: text/plain\r\n"
               "Content-Length: %d\r\n"
               "Connection: close\r\n\r\n"
               "%s",
               (int)strlen(msg), msg);
      client.print(resp);
    } else {
      client.print("HTTP/1.1 200 OK\r\n");
      client.print("Content-Type: image/jpeg\r\n");
      client.printf("Content-Length: %u\r\n", fb->len);
      client.print("Connection: close\r\n\r\n");
      client.write(fb->buf, fb->len);
      client.flush();
      esp_camera_fb_return(fb);
    }

    // Restore detect mode
    switchToDetectMode();

    yield();
    client.stop();
    pendingFaceClientActive = false;
    return;
  }

  if (requestLine.startsWith("GET /stream")) {
    netLog("[HTTP] Starting MJPEG stream for browser\n");

    // Read and discard HTTP headers
    while (client.available()) {
      String line = client.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) {
        break;
      }
    }

    if (cameraSleepMode) {
      const char *msg = "Camera is in sleep mode.";
      char resp[160];
      snprintf(resp, sizeof(resp),
               "HTTP/1.1 503 Service Unavailable\r\n"
               "Content-Type: text/plain\r\n"
               "Content-Length: %d\r\n"
               "Connection: close\r\n\r\n"
               "%s",
               (int)strlen(msg), msg);
      client.print(resp);
      client.flush();
      yield();
      client.stop();
      pendingFaceClientActive = false;
      return;
    }

    // Switch to JPEG mode for video streaming
    if (!switchToUploadMode(FRAMESIZE_VGA, 12, false, false)) {
      const char *msg = "Camera JPEG mode failed.";
      char resp[160];
      snprintf(resp, sizeof(resp),
               "HTTP/1.1 500 Internal Server Error\r\n"
               "Content-Type: text/plain\r\n"
               "Content-Length: %d\r\n"
               "Connection: close\r\n\r\n"
               "%s",
               (int)strlen(msg), msg);
      client.print(resp);
      client.flush();
      yield();
      client.stop();
      pendingFaceClientActive = false;
      return;
    }

    // Send MJPEG stream headers after JPEG mode is confirmed.
    client.print("HTTP/1.1 200 OK\r\n");
    client.print("Content-Type: multipart/x-mixed-replace; boundary=frame\r\n");
    client.print("Connection: close\r\n\r\n");
    delay(200);

    // Continuous capture loop while browser tab remains open. Keep this finite
    // because face verification uses this same single WiFiServer.
    unsigned long streamStartedAt = millis();
    while (client.connected() && (millis() - streamStartedAt) < BROWSER_STREAM_MAX_MS) {
      camera_fb_t *fb = esp_camera_fb_get();
      if (fb) {
        client.print("--frame\r\n");
        client.print("Content-Type: image/jpeg\r\n");
        client.printf("Content-Length: %u\r\n\r\n", fb->len);
        client.write(fb->buf, fb->len);
        client.print("\r\n");
        client.flush();
        esp_camera_fb_return(fb);
      }
      
      // Yield to keep FreeRTOS watchdogs and WiFi layer responsive
      yield();
      delay(80); // Cap frame rate to ~10-12 FPS to avoid thermal throttling
    }

    netLog("[HTTP] MJPEG stream closed; returning camera to detection mode\n");

    // Restore detect mode after user closes streaming tab
    switchToDetectMode();
    client.stop();
    pendingFaceClientActive = false;
    return;
  }

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
    yield();
    client.stop();
    pendingFaceClientActive = false;
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
        if (pendingUpload) {
          persistUploadIntent(true);
        }
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
  bool faceFound = false;
  bool lowLight = false;
  if (!cameraSleepMode) {
    bool detectReady = cameraInDetectMode || switchToDetectMode();
    if (!detectReady) {
      netLog("[HTTP] Face check failed: camera detect mode unavailable\n");
      const char *result = "CAMERA_ERROR\r\n";
      char resp[128];
      snprintf(resp, sizeof(resp),
               "HTTP/1.1 200 OK\r\n"
               "Content-Type: text/plain\r\n"
               "Content-Length: %d\r\n"
               "Connection: close\r\n\r\n"
               "%s",
               (int)strlen(result), result);
      client.print(resp);
      client.flush();
      yield();
      client.stop();
      pendingFaceClientActive = false;
      return;
    }
    faceFound = runFaceDetectionWindow(FACE_DETECT_WINDOW_MS, &lowLight);
  }

  const char *result;
  if (faceFound) {
    personCount++;
    netLog("[HTTP] Face detected!\n");
    result = "FACE_OK";
    queueSingleUpload("HTTP");
  } else {
    netLog(lowLight ? "[HTTP] No face detected (low light)\n"
                    : "[HTTP] No face detected\n");
    result = lowLight ? "NO_FACE_LOW_LIGHT" : "NO_FACE";
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
  yield();
  client.stop();
  pendingFaceClientActive = false;
}

void loop() {
  handleSerialCommands();
  maintainWiFiConnection();
  maybePublishWriteMetrics(millis());
  handleFaceStatusClient(); // On-demand face-check via WiFi (from proxy)
  handleUartFaceCommand();  // On-demand face-check via UART (from Tester)

  // Deferred upload — runs AFTER face-check response was sent
  // This avoids deadlock (CAM uploads to the same proxy that requested
  // face-check)
  if (pendingUpload && !uploadInProgress && millis() >= uploadRetryAtMs) {
    uploadInProgress = true;
    netLog("[UPLOAD] Deferred capture + upload starting...\n");
    bool uploadOk = false;

    if (payloadStoreReady && pendingObjectPath[0] != '\0' &&
        SPIFFS.exists(CAM_PENDING_FILE)) {
      netLog("[UPLOAD] Replaying persisted payload...\n");
      uploadOk = uploadPendingPayloadFromStorage();
    }

    if (!uploadOk) {
      uploadOk = captureHighResAndUpload();
    }

    if (uploadOk) {
      pendingUpload = false;
      pendingUploadRetryCount = 0;
      persistUploadIntent(false);
      netLog("[UPLOAD] Done. One photo captured.\n");
    } else {
      if (pendingUploadRetryCount < 255) {
        pendingUploadRetryCount++;
      }

      if (pendingUploadRetryCount >= MAX_UPLOAD_RETRIES) {
        netLog("[UPLOAD] Retry limit reached (%u). Dropping pending intent.\n",
               (unsigned int)pendingUploadRetryCount);
        pendingUpload = false;
        clearPendingPayloadStorage();
        persistUploadIntent(false);
      } else {
        unsigned long backoff = UPLOAD_RETRY_BASE_MS;
        for (uint8_t i = 1; i < pendingUploadRetryCount; i++) {
          if (backoff >= (UPLOAD_RETRY_MAX_MS / 2)) {
            backoff = UPLOAD_RETRY_MAX_MS;
            break;
          }
          backoff *= 2;
        }
        if (backoff > UPLOAD_RETRY_MAX_MS) {
          backoff = UPLOAD_RETRY_MAX_MS;
        }

        uploadRetryAtMs = millis() + backoff;
        persistUploadIntent(true);
        netLog("[UPLOAD] Failed; retry #%u in %lums.\n",
               (unsigned int)pendingUploadRetryCount,
               (unsigned long)backoff);
      }
    }
    uploadInProgress = false;
  }

  if (camRebootPending && millis() >= camRebootAtMs) {
    netLog("[POWER] Remote reboot requested. Restarting CAM...\n");
    delay(50);
    ESP.restart();
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
      if (pendingUpload) {
        persistUploadIntent(true);
      }
    }
  }

  netLog("[UART] Face check requested from Tester board\n");

  if (cameraSleepMode) {
    Serial2.println("NO_FACE");
    return;
  }

  // Run repeated face detection in a window (old working logic style)
  bool lowLight = false;
  bool detected = runFaceDetectionWindow(FACE_DETECT_WINDOW_MS, &lowLight);

  // Respond IMMEDIATELY
  if (detected) {
    netLog("[UART] Face DETECTED\n");
    Serial2.println("FACE_OK");
    queueSingleUpload("UART");
  } else {
    Serial.println(lowLight ? F("[UART] No face (low light)") : F("[UART] No face"));
    Serial2.println(lowLight ? "NO_FACE_LOW_LIGHT" : "NO_FACE");
  }
}
