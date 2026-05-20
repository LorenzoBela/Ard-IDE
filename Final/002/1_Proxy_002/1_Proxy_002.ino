/**
 * GPS & LTE Signal to Firebase - LTE Only (No WiFi)
 * For LILYGO T-SIM A7670E (GPS Version)
 *
 * Uses LTE exclusively via AT+HTTP commands (modem-level SSL)
 * This bypasses TinyGsmClientSecure which has issues on this modem.
 */

#include "esp_task_wdt.h"
#include <esp_wifi.h>
#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include "mbedtls/sha256.h"

// WiFi HTTPS/mbedTLS plus Firebase JSON handling can exceed Arduino-ESP32's
// default 8 KB loopTask stack and trigger stack canary resets.
SET_LOOP_TASK_STACK_SIZE(32768);

#include "LogTee.h"

SerialTee LogSerial(Serial);
#define Serial LogSerial

// TinyGSM - MUST define modem BEFORE include
#define TINY_GSM_MODEM_A7672X
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>

// ── Ported modules ──
#include "BatteryMonitor.h"
#include "GeofenceProxy.h"
#include "TheftGuard.h"
#include "DeliveryReassignment.h"
#include "DeliveryPersist.h"

// ==================== CONFIGURATION ====================
// LTE Settings â€” Globe Telecom Philippines
// Postpaid: "internet.globe.com.ph"  |  Prepaid: "http.globe.com.ph"
#define GPRS_APN "internet.globe.com.ph"
#define GPRS_APN_ALT "http.globe.com.ph"
#define GPRS_USER ""
#define GPRS_PASS ""

// Firebase REST AP
#define FIREBASE_HOST                                                          \
  "smart-top-box-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "AIzaSyA7DETBpsdPN6icfWi7PijCbpmLNWEZyTQ"

// Box identity (populated at runtime by autoProvision())
char HARDWARE_ID[12] = "BOX_001";       // e.g. "BOX_002"
char AP_SSID[24]     = "SmartTopBox_AP"; // e.g. "SmartTopBox_AP_002"
char CAM_PREFIX[24]  = "OV3660_CAM_001"; // e.g. "OV3660_CAM_002"
uint8_t boxNum       = 0;

// Final BOX_002 build: hard-pin this LilyGO proxy as BOX_002.
#define PROXY_BOX_NUM 2

// ==================== HOTSPOT (WiFi AP for ESP32-CAM) ====================
// Production hotspot-first variant: LilyGO joins one of these hotspots as a
// station. Controller and ESP32-CAM join the same hotspot and discover this
// proxy over UDP.
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

#define PROXY_DISCOVERY_PORT 5115
#define PROXY_DISCOVERY_QUERY "SMART_TOP_BOX_PROXY?"
#define PROXY_DISCOVERY_REPLY "SMART_TOP_BOX_PROXY:"
#define WIFI_CLOUD_HEALTH_INTERVAL_MS 15000
#define WIFI_CLOUD_FAILURE_LIMIT 3
// Camera proof uploads are handled by the A7670 modem HTTP engine. The WiFi
// HTTPS upload path uses mbedTLS on loopTask and can trip the stack canary.
#define USE_WIFI_SUPABASE_UPLOAD 1

#define AP_PASS "topbox123"
#define CAM_SERVER_PORT 8080

// Dedicated controller <-> LilyGO UART. Do not reuse modem UART pins 26/27.
// Wire cross-over:
//   LilyGO GPIO21 TX -> Controller GPIO33 RX
//   LilyGO GPIO22 RX <- Controller GPIO27 TX
//   GND shared
#define DISABLE_PROXY_UART 1
#define PROXY_UART_RX 22
#define PROXY_UART_TX 21
#define PROXY_UART_BAUD 9600

// Supabase Storage (matches ESP32-CAM sketch)
#define SUPABASE_URL "https://lvpneakciqegwyymtqno.supabase.co"
#define SUPABASE_BUCKET "proof-photos"
#define SUPABASE_ANON_KEY                                                      \
  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9."                                      \
  "eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Imx2cG5lYWtjaXFlZ3d5eW10cW5vIiwicm9sZSI6In" \
  "NlcnZpY2Vfcm9sZSIsImlhdCI6MTc2NzkwNjM3OCwiZXhwIjoyMDgzNDgyMzc4fQ."          \
  "SFJ1Z61WKkQ4xv7rkjlSxA89JqtaaFhuhlEupqBpqF8"

// Pins for LILYGO T-SIM A7670E (official pinout)
#define MODEM_TX 26
#define MODEM_RX 27
#define MODEM_PWRKEY 4
#define MODEM_POWER_ON 12
#define MODEM_RESET_PIN 5
#define MODEM_BAUD 115200

// Timing (ms)
#define GPS_INTERVAL_ACQUIRING 500 // Fast polling while seeking fix
#define GPS_INTERVAL_LOCKED 2000   // Normal polling once fix acquired
#define FIREBASE_INTERVAL 5000      // Stagger writes to free modem for faster context reads
#define SIGNAL_INTERVAL 10000
#define IDLE_POLL_MULTIPLIER 2
#define IDLE_GPS_POLL_MULTIPLIER 3
#define TIME_SYNC_INTERVAL 1800000 // Re-sync time every 30 minutes

// Reliability
#define WDT_TIMEOUT_S 90            // Allow slow cellular/WiFi calls without false reboot
#define MODEM_HEALTH_INTERVAL 30000       // Check modem alive every 30s
#define TAMPER_CHECK_INTERVAL 15000        // Check tamper-clear command every 15s (rare admin action)
#define PHOTO_BURST_CHECK_INTERVAL 10000   // Check camera burst commands from web admin
#define STALE_DELIVERY_CHECK_DIVISOR 5     // Only verify delivery still active every Nth context poll
#define MEM_REPORT_INTERVAL 60000   // Print heap stats every 60s
#define MAX_CONSECUTIVE_FAILURES 5  // Hard-reset modem after 5 AT timeouts
#define MAX_FB_FAILURES 3           // Reconnect LTE after 3 Firebase fails

// Firebase /hardware context reader.
// The modem is read in 1024-byte windows, but the assembled JSON can be larger.
#define DELIVERY_CONTEXT_HTTPREAD_CHUNK 1024
#define DELIVERY_CONTEXT_MAX_BYTES 32768
#define DELIVERY_CONTEXT_HEAP_HEADROOM 4096

// Philippine Standard Time offset (UTC+8)
#define PHT_OFFSET_HOURS 8
#define PHT_OFFSET_QUARTERS 32 // 8 hours * 4 quarter-hours

// ==================== GLOBALS ====================
HardwareSerial modemSerial(1);
TinyGsm modem(modemSerial);
HardwareSerial proxySerial(2);

bool lteConnected = false;
bool wifiUplinkConnected = false;
bool wifiCloudHealthy = false;
uint8_t wifiCloudFailures = 0;
unsigned long lastWifiCloudHealthAt = 0;
bool gpsEnabled = false;
bool modemOK = false;
unsigned long lastGps = 0, lastFB = 0, lastSig = 0, lastTimeSync = 0;
unsigned long lastModemHealth = 0, lastMemReport = 0;
unsigned long lastTamperCheckAt = 0;
unsigned long lastPhotoBurstCheckAt = 0;

double gpsLat = 0, gpsLon = 0;
double gpsHdop = 99.9; // Horizontal Dilution of Precision (99.9 = unknown)
float  gpsSpeed = 0.0f; // Speed in knots from CGNSSINFO, converted to m/s
float  gpsCourse = -1.0f; // Course over ground in degrees (0-360), -1 = invalid/stationary
bool gpsFix = false;
int gpsSats = 0;
int cachedSignalRssi = -999;
int cachedSignalCsq = -1;
unsigned long dataBytesOut = 0; // Total bytes sent to Firebase

// Hotspot / camera proxy
WiFiServer camServer(CAM_SERVER_PORT);
// Persistent keep-alive socket from the controller ESP32.
// WiFiServer::available() only returns NEW connections; this tracks
// the existing one so we can read subsequent requests on the same TCP pipe.
static WiFiClient keepAliveController;
static bool inApRequestHandler = false;
static volatile bool modemHttpBusy = false;
SemaphoreHandle_t stateMutex = NULL;
SemaphoreHandle_t modemMutex = NULL;
TaskHandle_t smartBoxLoopTaskHandle = NULL;
TaskHandle_t firebaseSyncTaskHandle = NULL;
static volatile uint32_t modemLockSkipCount = 0;
static volatile uint32_t stateLockSkipCount = 0;
static const char *volatile currentModemOwner = "none";
static const char *volatile lastModemOwner = "none";
static volatile unsigned long modemLockStartedAt = 0;
static volatile unsigned long modemLockMaxHeldMs = 0;
static volatile uint8_t modemLockDepth = 0;
static bool controllerContextRefreshQueued = false;
static unsigned long controllerContextRefreshQueuedAt = 0;
static bool deliveryContextStale = false;
static char lastRefreshStatus[24] = "BOOT";
bool apStarted = false;
String relayDiag =
    ""; // Diagnostics forwarded to ESP32-CAM via HTTP response body
static uint8_t hotspotScanStart = 0;
static int8_t selectedHotspotCredential = -1;
static bool photoUploadActive = false;
static int photoUploadProgress = 0;
static char photoUploadStatus[16] = "IDLE";
static char photoUploadError[64] = "";
static unsigned long photoUploadUpdatedAt = 0;

// Cached Delivery Context from Firebase RTDB (served to Tester ESP32 via GET
// /otp)
char cachedOtp[8] = "";
bool otpCacheValid = false;
// EC-32: Return protocol (OTP at pickup on cancellation)
char cachedReturnOtp[8] = "";
bool returnOtpCacheValid = false;
bool returnActive = false;
bool lastReturnActive = false;
char cachedDeliveryId[64] = "";
bool deliveryIdCacheValid = false;
char cachedDeliveryStatus[32] = ""; // To track "ASSIGNED", "PICKED_UP", "IN_TRANSIT" etc.
bool cachedSamePickupDropoff = false;
char cachedPersonalPinHashMcu[65] = "";
char cachedPersonalPinSalt[33] = "";
char cachedPersonalPinRiderId[64] = "";
bool personalPinEnabled = false;
static unsigned long lastPairingPinSyncAt = 0;
#define PIN_PAIRING_SYNC_MIN_MS 10000
// EC-FIX: Remote lock/unlock status command from Firebase (UNLOCKING/LOCKED)
char cachedStatus[16] = "";
unsigned long cachedStatusSetAt = 0;
uint8_t cachedStatusServesRemaining = 0;
#define STATUS_COMMAND_MAX_SERVES 60
#define STATUS_COMMAND_RETRY_WINDOW_MS 60000
enum CommandDeliveryStage : uint8_t {
  CMD_STAGE_NONE = 0,
  CMD_STAGE_PENDING = 1,
  CMD_STAGE_ACK_SENT = 2,
  CMD_STAGE_DONE = 3
};
uint8_t cachedStatusStage = CMD_STAGE_NONE;
char cachedCommandAckStatus[32] = "";
char cachedCommandAckDetails[96] = "";
unsigned long lastCommandAckRetryAt = 0;
#define COMMAND_ACK_RETRY_INTERVAL_MS 3000
bool rebootAllPending = false;
bool rebootCamDispatchDone = false;
unsigned long rebootAllAtMs = 0;
unsigned long rebootAllQueuedAtMs = 0;
bool rebootAllWaitLogged = false;
#define REBOOT_ALL_GRACE_MS 5000
#define REBOOT_ALL_WAIT_FOR_CONTROLLER_MS 45000

// Internal proxy state to prevent telemetry from overwriting physical lock state on Firebase
bool isBoxLocked = true;

double destLat = 0.0, destLon = 0.0;
bool destCoordsValid = false;
double pickupLat = 0.0, pickupLon = 0.0;
bool pickupCoordsValid = false;
unsigned long lastDeliveryContextRead = 0;
#define DELIVERY_CONTEXT_READ_INTERVAL 3000 // Re-read Firebase every 3s

// Delivery context validation / fail-closed controls
unsigned long deliveryContextBootAtMs = 0;
unsigned long lastDeliveryIdSeenAtMs = 0;
unsigned long lastContextFetchOkAtMs = 0;
bool deliveryContextValidated = false;
uint8_t contextFetchFailCount = 0;
bool deliveryContextTrusted = false;
unsigned long lastDeliveryVerifyAtMs = 0;
char lastVerifiedDeliveryId[64] = "";
#define DELIVERY_CONTEXT_VALIDATE_GRACE_MS 20000
#define CONTEXT_FETCH_FAIL_RESET_COUNT 2
#define DELIVERY_ACTIVE_VERIFY_INTERVAL_MS 15000UL

// UDP log receiver
WiFiUDP udpServer;
WiFiUDP discoveryServer;
#define UDP_LOG_PORT 5114

// TCP log mirror (wireless Serial)
#define LOG_TCP_PORT 7777
static WiFiServer logServer(LOG_TCP_PORT);
static WiFiClient logClient;
static bool logServerStarted = false;

static void startLogServer() {
  if (logServerStarted) return;
  logServer.begin();
  logServer.setNoDelay(true);
  logServerStarted = true;
  Serial.printf("[LOG] TCP server on %s:%d\n",
                WiFi.localIP().toString().c_str(), LOG_TCP_PORT);
}

static void pollLogServer() {
  if (!logServerStarted) return;

  if (!logClient || !logClient.connected()) {
    WiFiClient next = logServer.available();
    if (next) {
      logClient.stop();
      logClient = next;
      logClient.setNoDelay(true);
      LogSerial.setMirrorClient(&logClient);
      Serial.println("[LOG] Client connected");
    }
  }

  if (logClient && !logClient.connected()) {
    LogSerial.setMirrorClient(NULL);
    logClient.stop();
  }

  while (logClient && logClient.connected() && logClient.available()) {
    logClient.read();
  }
}

// ESP32-CAM IP tracking (for face-check forwarding)
IPAddress camClientIP;
static bool extractJsonStringValue(const char *json, const char *key,
                                   char *out, size_t outSize) {
  if (!json || !key || !out || outSize == 0) {
    return false;
  }
  out[0] = '\0';

  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);
  const char *start = strstr(json, pattern);
  if (!start) {
    return false;
  }
  start += strlen(pattern);
  const char *end = strchr(start, '"');
  if (!end || end <= start) {
    return false;
  }

  size_t len = (size_t)(end - start);
  if (len >= outSize) {
    len = outSize - 1;
  }
  strncpy(out, start, len);
  out[len] = '\0';
  return out[0] != '\0';
}

bool camClientKnown = false;
#define CAM_FACE_PORT 80 // ESP32-CAM runs a tiny HTTP server on port 80
static unsigned long camLastSeenAt = 0;
#define CAM_LIVENESS_STALE_MS 120000
static unsigned long lastCamProbeAt = 0;
#define CAM_PROBE_INTERVAL_MS 15000
static unsigned long controllerLastSeenAt = 0;
#define CONTROLLER_LIVENESS_STALE_MS 30000
#define MANUAL_REFRESH_BURST_MS 15000UL
#define MANUAL_REFRESH_READ_INTERVAL_MS 500UL
#define CONTEXT_STICKY_GRACE_MS 60000UL
#define CONTEXT_FAIL_RESET_THRESHOLD 3
#define GEO_LAST_KNOWN_TTL_MS 30000UL
#define FALLBACK_TARGET_FETCH_INTERVAL_MS 30000UL
#define PHONE_LOC_FETCH_INTERVAL_MS 3000UL
#define PHONE_LOC_MAX_AGE_SEC 30UL
#define MANUAL_GEO_STALE_MS 1500UL

static double phoneLat = 0.0;
static double phoneLon = 0.0;
static float phoneAccuracy = -1.0f;
static uint64_t phoneTimestampMs = 0;
static unsigned long phoneFetchedAtMs = 0;
static bool phoneFixValid = false;
static unsigned long lastPhoneFetchAtMs = 0;
static unsigned long lastPhoneFallbackLogAtMs = 0;
static unsigned long manualRefreshBurstUntil = 0;
static uint64_t lastManualRefreshAt = 0;
static unsigned long lastFallbackTargetFetchAtMs = 0;
static int16_t lastKnownDistMeters = -1;
static bool lastKnownInside = false;
static unsigned long lastKnownGeoAtMs = 0;

enum ConnectivityMatrixState : uint8_t {
  CONN_ALL_UP = 0,
  CONN_CAM_DOWN_CTRL_UP = 1,
  CONN_CTRL_DOWN_CAM_UP = 2,
  CONN_BOTH_DOWN = 3
};
static ConnectivityMatrixState connectivityState = CONN_BOTH_DOWN;
static ConnectivityMatrixState lastConnectivityState = CONN_BOTH_DOWN;

enum ModemRecoveryState : uint8_t {
  MODEM_RECOVERY_IDLE = 0,
  MODEM_RECOVERY_REQUESTED = 1,
  MODEM_RECOVERY_RUNNING = 2
};
static volatile ModemRecoveryState modemRecoveryState = MODEM_RECOVERY_IDLE;
static volatile bool modemRecoveryRequested = false;
static unsigned long modemRetryAt = 0;
static unsigned long modemRetryBackoffMs = 10000;
static const unsigned long MODEM_RETRY_BACKOFF_MAX_MS = 120000;
static TaskHandle_t modemRecoveryTaskHandle = NULL;
static unsigned long lastLteReconnectLatencyMs = 0;
static unsigned long lteRecoveryAttemptCount = 0;
static unsigned long lastLteRecoveryStartedAt = 0;

static char lastCommandAckIdempotencyKey[65] = "";
static unsigned long lastCommandAckIdempotencyAt = 0;
static char lastTamperIdempotencyKey[65] = "";
static unsigned long lastTamperIdempotencyAt = 0;
static char lastLockEventIdempotencyKey[65] = "";
static unsigned long lastLockEventIdempotencyAt = 0;
static const unsigned long IDEMPOTENCY_DEDUPE_WINDOW_MS = 15000;

static unsigned long lastDurabilityMetricFlushAt = 0;
static const unsigned long DURABILITY_METRICS_INTERVAL_MS = 3600000UL;
static DpWriteMetrics lastDpMetricsSnapshot = {0, 0, 0, 0};

static uint32_t lastPersistedContextHash = 0;
static bool persistedContextHashReady = false;
static uint32_t lastPersistedPinContextHash = 0;
static bool persistedPinContextHashReady = false;

static void clearDeliveryContextCaches(const char *reason, bool patchHardware) {
  otpCacheValid = false;
  cachedOtp[0] = '\0';
  returnOtpCacheValid = false;
  cachedReturnOtp[0] = '\0';
  returnActive = false;
  lastReturnActive = false;

  deliveryIdCacheValid = false;
  cachedDeliveryId[0] = '\0';
  cachedDeliveryStatus[0] = '\0';

  destLat = 0.0;
  destLon = 0.0;
  destCoordsValid = false;
  pickupLat = 0.0;
  pickupLon = 0.0;
  pickupCoordsValid = false;

  lastDeliveryIdSeenAtMs = 0;
  deliveryContextValidated = true;
  deliveryContextTrusted = false;
  lastDeliveryVerifyAtMs = 0;
  lastVerifiedDeliveryId[0] = '\0';
  contextFetchFailCount = 0;
  deliveryContextStale = false;
  strncpy(lastRefreshStatus, "CLEARED", sizeof(lastRefreshStatus) - 1);
  lastRefreshStatus[sizeof(lastRefreshStatus) - 1] = '\0';

  phoneLat = 0.0;
  phoneLon = 0.0;
  phoneAccuracy = -1.0f;
  phoneTimestampMs = 0;
  phoneFetchedAtMs = 0;
  phoneFixValid = false;
  lastFallbackTargetFetchAtMs = 0;
  lastKnownDistMeters = -1;
  lastKnownInside = false;
  lastKnownGeoAtMs = 0;

  dpClearDeliveryContext();
  persistedContextHashReady = false;
  lastPersistedContextHash = 0;
  clearPersonalPinRuntimeCache(false);

  if (patchHardware) {
    if (lteConnected) {
      char hwPath[64];
      snprintf(hwPath, sizeof(hwPath), "/hardware/%s.json", HARDWARE_ID);
      httpPatchWithRetry(
          hwPath,
          "{\"otp_code\":null,\"delivery_id\":null,\"return_otp\":null,\"return_active\":false}");
    } else {
      Serial.println("[CONTEXT] Clear requested but LTE not connected");
    }
  }

  Serial.printf("[CONTEXT] Cleared delivery context (%s)\n",
                reason ? reason : "unknown");
}

unsigned long lastPersonalPinFlushAt = 0;
#define PERSONAL_PIN_AUDIT_FLUSH_INTERVAL_MS 5000
static unsigned long lastPinRuntimeRefreshAt = 0;
#define PIN_RUNTIME_REFRESH_MIN_MS 15000
static bool personalPinRuntimeRefreshQueued = false;
static bool personalPinRuntimeRefreshForce = false;
static bool tamperFirebaseWriteQueued = false;
static bool lockEventFirebaseWriteQueued = false;
static bool queuedLockOtpValid = false;
static bool queuedLockFaceDetected = false;
static bool queuedLockUnlocked = false;
static int queuedLockFaceAttempts = 0;
static bool queuedLockFaceRetryExhausted = false;
static bool queuedLockFallbackRequired = false;
static char queuedLockFailureReason[24] = "";
static unsigned long queuedLockUnlockLatencyMs = 0;
static bool commandAckFirebaseWriteQueued = false;
static char queuedCommandAckCommand[24] = "";
static char queuedCommandAckStatus[32] = "";
static char queuedCommandAckDetails[96] = "";
static bool queuedCommandAckTracked = false;
static bool returnCompletionFirebaseWriteQueued = false;

// Reliability counters
uint8_t consecutiveModemFailures = 0;
uint8_t firebaseFailures = 0;

// Network time tracking (Philippine Standard Time, UTC+8)
unsigned long epochTime = 0; // Unix timestamp (seconds since 1970 UTC)
unsigned long epochSyncMillis =
    0; // millis() value when epochTime was last synced
bool timeSynced = false;

// ── Module instances ──
GeofenceProxy geoProxy;
DeliveryReassignment reassign;
unsigned long lastBatteryRead  = 0;
unsigned long lastTheftReport  = 0;
TheftState    prevTheftState   = TG_NORMAL;
#define BATTERY_READ_INTERVAL  10000
#define THEFT_REPORT_INTERVAL  30000

// â”€â”€ Compile-time fallback date â”€â”€
// Parses __DATE__ ("Feb 28 2026") and __TIME__ ("11:43:00") as UTC,
// then converts to epoch so the device is never stuck at 1970-01-01.
static int parseMonthFromStr(const char *m) {
  const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                          "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  for (int i = 0; i < 12; i++) {
    if (m[0] == months[i][0] && m[1] == months[i][1] && m[2] == months[i][2])
      return i + 1;
  }
  return 1;
}

// Forward-declare dateTimeToEpoch (defined below)
unsigned long dateTimeToEpoch(int year, int month, int day, int hour, int min,
                              int sec);

// Forward declaration (implementation appears in auto-provisioning section).
String httpGetFromFirebase(const char *path);
static bool parseFirebaseStringValue(const String &body, char *out, size_t outLen);
static bool parseFirebaseBoolValue(const String &body, bool &out, bool &hasValue);
static bool fetchFirebaseDouble(const char *path, double &outVal);
bool writeCommandAckToFirebase(const char *command, const char *status,
                               const char *details);
static bool refreshPersonalPinRuntimeFromFirebase(bool logDetails, bool force);
static bool syncPersonalPinRuntimeWithActivePairing(bool logDetails, bool force);
static void applyTargetCoords(double lat, double lon, const char *source);
static void applyPickupCoords(double lat, double lon, const char *source);
static bool refreshHardwareCoordsExact();
static bool refreshTargetCoordsFromDelivery(const char *deliveryId, bool force = false);
static const char *refreshGeofenceForController(bool forceCoords);
void scheduleModemRecovery(const char *reason);
void modemRecoveryTask(void *);
void maybeEvaluateConnectivityMatrix(unsigned long now);
void maybePublishDurabilityMetrics(unsigned long now);
static bool isControllerUp(unsigned long now);
static void startHotspotStation();
static void maintainHotspotStation(unsigned long now);
static void pollDiscoveryServer();
static void beginProxyUart();
static void pollProxyUart();
static void handleProxyUartRequest(const String &line);
static bool cloudWifiAvailable();
static bool wifiPutToFirebase(const char *path, const String &jsonData);
static bool wifiPatchToFirebase(const char *path, const String &jsonData);
static bool wifiGetFromFirebase(const char *path, String &bodyOut);
static void pollWifiCloudHealth(unsigned long now);
static const char *connectivityStateStr(uint8_t state);
static bool readTopLevelJsonString(const String &json, const char *key,
                                   char *out, size_t outSize);
static bool readTopLevelJsonBool(const String &json, const char *key, bool &outVal);
static bool readTopLevelJsonInt(const String &json, const char *key, int &outVal);
static bool readTopLevelJsonDouble(const String &json, const char *key, double &outVal);
static bool refreshAuthoritativeDeliveryFieldsFromFirebase(bool includeCoords = true);
static bool verifyPersonalPinLocal(const char *pin);
static void probeCameraIfDue(unsigned long now);
void writeLockEventToFirebase(bool otpValid, bool faceDetected, bool unlocked,
                              int faceAttempts,
                              bool faceRetryExhausted,
                              bool fallbackRequired,
                              const char *failureReason,
                              unsigned long unlockLatencyMs);
void writeTamperToFirebase();
static void queueTamperFirebaseWrite();
static void queueLockEventFirebaseWrite(bool otpValid, bool faceDetected,
                                        bool unlocked, int faceAttempts,
                                        bool faceRetryExhausted,
                                        bool fallbackRequired,
                                        const char *failureReason,
                                        unsigned long unlockLatencyMs);
static void queueCommandAckFirebaseWrite(const char *command,
                                         const char *status,
                                         const char *details,
                                         bool trackedAck);
static void queueReturnCompletionFirebaseWrite();
String forwardFaceCheck(String deliveryId);
String forwardCameraCaptureUploadCommand();
String forwardCameraPowerCommand(const String &mode);
void checkAndRunPhotoBurstFromFirebase(unsigned long now);

static bool takeModem(uint32_t timeoutMs, const char *owner) {
  if (modemMutex == NULL) return true;
  if (xSemaphoreTakeRecursive(modemMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
    if (modemLockDepth == 0) {
      modemLockStartedAt = millis();
      currentModemOwner = owner ? owner : "unknown";
      lastModemOwner = currentModemOwner;
    }
    modemLockDepth++;
    return true;
  }
  modemLockSkipCount++;
  Serial.printf("[MODEM_LOCK] busy owner=%s caller=%s skips=%lu\n",
                currentModemOwner ? currentModemOwner : "unknown",
                owner ? owner : "unknown",
                (unsigned long)modemLockSkipCount);
  return false;
}

static void giveModem(const char *owner) {
  (void)owner;
  if (modemMutex == NULL) return;
  if (modemLockDepth > 0) modemLockDepth--;
  if (modemLockDepth == 0) {
    unsigned long heldMs = modemLockStartedAt ? (millis() - modemLockStartedAt) : 0;
    if (heldMs > modemLockMaxHeldMs) modemLockMaxHeldMs = heldMs;
    currentModemOwner = "none";
    modemLockStartedAt = 0;
  }
  xSemaphoreGiveRecursive(modemMutex);
}

class ModemLockGuard {
public:
  ModemLockGuard(const char *owner, uint32_t timeoutMs)
      : _owner(owner), _locked(takeModem(timeoutMs, owner)) {}
  ~ModemLockGuard() {
    release();
  }
  bool ok() const { return _locked; }
  void release() {
    if (_locked) {
      giveModem(_owner);
      _locked = false;
    }
  }

private:
  const char *_owner;
  bool _locked;
};

static bool takeState(uint32_t timeoutMs) {
  if (stateMutex == NULL) return true;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) return true;
  stateLockSkipCount++;
  return false;
}

static void giveState() {
  if (stateMutex != NULL) xSemaphoreGive(stateMutex);
}

static unsigned long getCompileTimeEpoch() {
  // __DATE__ = "Mmm dd yyyy"  __TIME__ = "hh:mm:ss"
  const char *d = __DATE__; // e.g. "Feb 28 2026"
  const char *t = __TIME__; // e.g. "11:43:00"
  int month = parseMonthFromStr(d);
  int day = (d[4] == ' ') ? (d[5] - '0') : ((d[4] - '0') * 10 + (d[5] - '0'));
  int year = (d[7] - '0') * 1000 + (d[8] - '0') * 100 + (d[9] - '0') * 10 +
             (d[10] - '0');
  int hour = (t[0] - '0') * 10 + (t[1] - '0');
  int min = (t[3] - '0') * 10 + (t[4] - '0');
  int sec = (t[6] - '0') * 10 + (t[7] - '0');
  // dateTimeToEpoch expects LOCAL (PHT) input and subtracts 8h internally.
  // __DATE__/__TIME__ are in the compiler's local timezone (PHT for you),
  // so this produces a correct UTC epoch.
  return dateTimeToEpoch(year, month, day, hour, min, sec);
}

// ==================== AT HELPER ====================
// Send AT command and return full response
String sendATAndWait(const char *cmd, unsigned long timeout = 3000) {
  ModemLockGuard modemLock("sendATAndWait", timeout + 500);
  if (!modemLock.ok()) return "";
  String resp = "";
  modem.sendAT(cmd);
  modem.waitResponse(timeout, resp);
  return resp;
}

// ==================== MODEM ====================
void powerModem() {
  Serial.println("Powering modem...");

  // Power ON pin
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Reset sequence
  pinMode(MODEM_RESET_PIN, OUTPUT);
  digitalWrite(MODEM_RESET_PIN, LOW);
  delay(100);
  digitalWrite(MODEM_RESET_PIN, HIGH);
  delay(1000);
  digitalWrite(MODEM_RESET_PIN, LOW);

  // Power key sequence (toggle LOW->HIGH->LOW)
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(100);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(1000);
  digitalWrite(MODEM_PWRKEY, LOW);

  Serial.println("  Power sequence complete");
  delay(3000);
}

bool initModem() {
  ModemLockGuard modemLock("initModem", 5000);
  if (!modemLock.ok()) return false;
  Serial.println("Initializing modem...");
  delay(3000); // Wait for modem boot

  String name = modem.getModemName();
  Serial.println("  Modem: " + name);

  String info = modem.getModemInfo();
  Serial.println("  Info: " + info);

  if (name.length() == 0 && info.length() == 0) {
    Serial.println("  Modem not responding");
    return false;
  }
  Serial.println("  Modem OK");

  // â”€â”€ SPEED OPTIMIZATION: Force LTE-only mode â”€â”€
  // AT+CNMP=38 skips GSM/UMTS scanning, locks to Cat-1 LTE
  // This saves 5-15s vs auto mode (which scans all RATs)
  Serial.print("  Setting LTE-only (AT+CNMP=38)...");
  modem.sendAT("+CNMP=38");
  if (modem.waitResponse(5000L) == 1) {
    Serial.println(" OK");
  } else {
    Serial.println(" skip (using auto)");
  }

  // â”€â”€ SPEED OPTIMIZATION: Pre-configure APN during boot â”€â”€
  // Set PDP context NOW so the modem starts LTE attach immediately
  // while we initialize GPS (parallel registration)
  Serial.print("  Pre-setting Globe APN...");
  modem.sendAT("+CGDCONT=1,\"IP\",\"" + String(GPRS_APN) + "\"");
  modem.waitResponse(3000L);
  Serial.println(" OK");

  // Trigger auto-registration in background
  modem.sendAT("+COPS=0");
  modem.waitResponse(1000L); // Don't wait long, let it register while GPS inits

  return true;
}

bool initGPS() {
  ModemLockGuard modemLock("initGPS", 5000);
  if (!modemLock.ok()) return false;
  Serial.println("Initializing GPS...");

  // Check if GNSS is already powered (avoid unnecessary cold start)
  String resp;
  modem.sendAT("+CGNSSPWR?");
  modem.waitResponse(1000L, resp);

  bool alreadyOn = (resp.indexOf("+CGNSSPWR: 1") >= 0);

  if (!alreadyOn) {
    // First boot: power on GNSS
    Serial.print("  Powering GNSS...");
    modem.sendAT("+CGNSSPWR=1");
    int result = modem.waitResponse(3000L, resp);

    if (result != 1) {
      Serial.println(" FAIL: " + resp);
      return false;
    }
    Serial.println(" OK");

    // Wait for READY signal (reduced to 5s â€” modem is usually fast)
    unsigned long start = millis();
    bool ready = false;
    while (millis() - start < 5000) {
      while (modem.stream.available()) {
        String line = modem.stream.readStringUntil('\n');
        if (line.indexOf("READY") >= 0) {
          ready = true;
          break;
        }
      }
      if (ready)
        break;
      delay(50);
    }
    if (!ready) {
      Serial.println("  No READY (may still work)");
    }
  } else {
    Serial.println("  GNSS already powered, preserving satellite cache");
  }

  // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
  // TTFF OPTIMIZATIONS (Fastest possible fix)
  // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

  // 1. Multi-constellation: GPS + GLONASS + BeiDou
  //    Mode 3 = GPS+GLONASS+BDS (confirmed for A7670E)
  //    More satellites visible = faster triangulation
  Serial.print("  Multi-GNSS (GPS+GLO+BDS)...");
  modem.sendAT("+CGNSSMODE=3");
  modem.waitResponse(1000L);
  Serial.println(" OK");

  // 2. XTRA ephemeris: download predicted satellite orbits
  //    Reduces cold start from 30s+ to <15s by knowing where sats are
  Serial.print("  XTRA ephemeris server...");
  modem.sendAT("+CGPSURL=\"http://xtra1.gpsonextra.net/xtra.bin\"");
  modem.waitResponse(2000L);
  Serial.println(" set");

  Serial.print("  XTRA enable...");
  modem.sendAT("+CGPSXE=1");
  if (modem.waitResponse(2000L) == 1) {
    Serial.println(" OK");
  } else {
    Serial.println(" skip");
  }

  Serial.print("  XTRA auto-download...");
  modem.sendAT("+CGPSXDAUTO=1");
  if (modem.waitResponse(2000L) == 1) {
    Serial.println(" OK");
  } else {
    Serial.println(" skip");
  }

  // 3. Hot start: reuse cached ephemeris/almanac from last session
  Serial.print("  Hot start (reuse cache)...");
  modem.sendAT("+CGPSHOT");
  modem.waitResponse(1000L);
  Serial.println(" OK");

  // 4. AGPS: download satellite data over cellular (needs data)
  Serial.print("  AGPS assist...");
  modem.sendAT("+CAGPS");
  if (modem.waitResponse(10000L, resp) == 1) {
    Serial.println(" OK");
  } else {
    Serial.println(" skipped (will retry after LTE connects)");
  }

  // 5. High sensitivity mode for weak signal areas
  Serial.print("  High sensitivity...");
  modem.sendAT("+CGNSHOT=1");
  modem.waitResponse(1000L);
  Serial.println(" OK");

  // 6. NMEA output at 1Hz for responsive updates
  modem.sendAT("+CGPSNMEARATE=1");
  modem.waitResponse(1000L);

  // Verify GPS is running
  Serial.print("  Verifying GPS status...");
  modem.sendAT("+CGNSSPWR?");
  if (modem.waitResponse(1000L, resp) == 1) {
    Serial.println(" " + resp);
  }

  // Check antenna / satellite info
  Serial.print("  Checking antenna...");
  modem.sendAT("+CGNSSTST=1");
  modem.waitResponse(1000L);
  delay(100);
  modem.sendAT("+CGNSSINF");
  if (modem.waitResponse(2000L, resp) == 1) {
    Serial.println(" " + resp);
    if (resp.indexOf(",,,,") >= 0 || resp.length() < 20) {
      Serial.println("  WARNING: No satellites detected!");
      Serial.println("  CHECK: 1) GPS antenna connected to module");
      Serial.println("         2) Antenna has clear sky view");
    }
  }

  Serial.println("  GPS initialized with TTFF optimizations");
  Serial.println("  Expected: <15s (XTRA) / 3-10s (AGPS) / 5-15s (hot)");
  return true;
}

void readGPS() {
  ModemLockGuard modemLock("readGPS", 150);
  if (!modemLock.ok()) return;
  modem.sendAT("+CGNSSINFO");
  String resp = "";
  if (modem.waitResponse(2000L, resp) != 1) {
    gpsFix = false;
    gpsSats = 0;
    Serial.println("GPS: No response from +CGNSSINFO");
    return;
  }

  int idx = resp.indexOf("+CGNSSINFO: ");
  if (idx < 0) {
    gpsFix = false;
    gpsSats = 0;
    Serial.println("GPS: No +CGNSSINFO in response");
    Serial.println("  Raw: " + resp);
    return;
  }

  String data = resp.substring(idx + 12);
  data.trim();

  // Show raw GPS data every 30 seconds for debugging
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug >= 30000) {
    Serial.println("GPS Raw: " + data);
    lastDebug = millis();
  }

  if (data.length() < 10 || data.startsWith(",,,")) {
    gpsFix = false;
    gpsSats = 0;
    return;
  }

  // Parse CGNSSINFO format:
  // <fix>,<GPS_sat>,<GNSS_sat>,<BeiDou_sat>,<Galileo_sat>,<lat>,<N/S>,<lon>,<E/W>,
  // <date>,<UTC_time>,<alt>,<speed>,<course>,<HDOP>,<PDOP>,<VDOP>
  // HDOP is at field 14 (between commas[13] and commas[14]) â€” needs 15+
  // commas
  int commas[17];
  int commaCount = 0;
  for (int i = 0; i < (int)data.length() && commaCount < 17; i++) {
    if (data.charAt(i) == ',') {
      commas[commaCount++] = i;
    }
  }

  if (commaCount < 8) {
    gpsFix = false;
    gpsSats = 0;
    Serial.println("GPS: Not enough fields in response");
    return;
  }

  int gpsSat = 0;
  int gnssSat = 0;
  if (commaCount >= 3) {
    String gpsSatStr = data.substring(commas[0] + 1, commas[1]);
    String gnssSatStr = data.substring(commas[1] + 1, commas[2]);
    gpsSatStr.trim();
    gnssSatStr.trim();
    gpsSat = gpsSatStr.toInt();
    gnssSat = gnssSatStr.toInt();
  }
  gpsSats = (gnssSat > gpsSat) ? gnssSat : gpsSat;

  // Extract fields: latitude is field 5, longitude is field 7
  String latStr = data.substring(commas[4] + 1, commas[5]);
  String latDir = data.substring(commas[5] + 1, commas[6]);
  String lonStr = data.substring(commas[6] + 1, commas[7]);
  String lonDir = data.substring(commas[7] + 1, commas[8]);

  latStr.trim();
  latDir.trim();
  lonStr.trim();
  lonDir.trim();

  if (latStr.length() > 0 && lonStr.length() > 0) {
    gpsLat = latStr.toDouble();
    gpsLon = lonStr.toDouble();

    if (latDir == "S")
      gpsLat = -gpsLat;
    if (lonDir == "W")
      gpsLon = -gpsLon;

    gpsFix = true;

    // Extract HDOP (field 14): between commas[13] and commas[14]
    if (commaCount >= 15) {
      String hdopStr = data.substring(commas[13] + 1, commas[14]);
      hdopStr.trim();
      double h = hdopStr.toDouble();
      if (h > 0)
        gpsHdop = h;
    }

    // Extract speed (field 12): between commas[11] and commas[12], in knots
    if (commaCount >= 13) {
      String spdStr = data.substring(commas[11] + 1, commas[12]);
      spdStr.trim();
      if (spdStr.length() > 0) {
        float knots = spdStr.toFloat();
        gpsSpeed = knots * 0.514444f; // knots -> m/s
      }
    }

    // Extract course/heading (field 13): between commas[12] and commas[13]
    // Course Over Ground (COG) in degrees — only reliable when moving
    if (commaCount >= 14) {
      String courseStr = data.substring(commas[12] + 1, commas[13]);
      courseStr.trim();
      if (courseStr.length() > 0) {
        float c = courseStr.toFloat();
        // COG is only reliable when moving (speed > ~5.4 km/h)
        if (gpsSpeed > 1.5f && c >= 0.0f && c < 360.0f) {
          gpsCourse = c;
        }
        // If stationary, keep last known course (don't reset to -1)
      }
    }

    static unsigned long lastFixDebug = 0;
    if (millis() - lastFixDebug >= 30000) {
      Serial.printf("GPS Parsed: %.6f, %.6f (Fix acquired, sats=%d)\n",
                    gpsLat, gpsLon, gpsSats);
      lastFixDebug = millis();
    }
  } else {
    gpsFix = false;
    gpsSats = 0;
  }
}

// ==================== LTE ====================
bool connectLTE() {
  if (!modemOK)
    return false;
  ModemLockGuard modemLock("connectLTE", 5000);
  if (!modemLock.ok()) return false;

  Serial.println("Connecting LTE (Globe PH)...");

  // â”€â”€ SIM Diagnostics â”€â”€
  Serial.print("  SIM status...");
  int simStatus = modem.getSimStatus();
  if (simStatus != 1) {
    Serial.printf(" FAIL (status: %d)\n", simStatus);
    Serial.println("  CHECK: 1) SIM inserted correctly (gold contacts down)");
    Serial.println("         2) SIM is activated with data");
    Serial.println("         3) SIM tray fully seated");
    return false;
  }
  Serial.println(" OK");

  // Print IMEI & IMSI for debugging
  String resp;
  modem.sendAT("+CGSN"); // IMEI
  if (modem.waitResponse(1000L, resp) == 1) {
    Serial.println("  IMEI: " + resp);
  }
  modem.sendAT("+CIMI"); // IMSI (starts with 515 02 for Globe)
  if (modem.waitResponse(1000L, resp) == 1) {
    Serial.println("  IMSI: " + resp);
    if (resp.indexOf("51502") < 0) {
      Serial.println("  WARNING: IMSI does not match Globe PH (515 02)");
    }
  }

  // â”€â”€ Signal Check (wait up to 30s for modem to find tower) â”€â”€
  // NOTE: modem was already told to register (AT+COPS=0) during initModem()
  //       so it's been searching during GPS init â€” should be faster now
  Serial.print("  Waiting for signal...");
  int csq = 99;
  unsigned long sigWaitStart = millis();
  while (millis() - sigWaitStart < 30000) {
    csq = modem.getSignalQuality();
    if (csq != 99 && csq != 0)
      break;
    Serial.print(".");
    esp_task_wdt_reset();
    delay(2000);
  }
  Serial.printf(" CSQ: %d", csq);
  if (csq == 99 || csq == 0) {
    Serial.println(" â€” No signal after 30s!");
    Serial.println("  CHECK: LTE antenna connected, SIM has coverage");
    return false;
  }
  int dbm = -113 + (2 * csq);
  Serial.printf(" (%d dBm)\n", dbm);

  // â”€â”€ Check current network info â”€â”€
  Serial.print("  Network info: ");
  modem.sendAT("+CPSI?");
  if (modem.waitResponse(2000L, resp) == 1) {
    Serial.println(resp);
  }

  // â”€â”€ Network Registration Check â”€â”€
  // TinyGSM's waitForNetwork() uses AT+CREG (2G) which fails on LTE-only
  // Instead, check AT+CPSI? directly â€” it already showed "LTE,Online" above
  Serial.print("  Checking LTE registration...");

  bool networkOK = false;
  unsigned long netWaitStart = millis();
  while (millis() - netWaitStart < 30000) {
    modem.sendAT("+CPSI?");
    if (modem.waitResponse(2000L, resp) == 1) {
      if (resp.indexOf("Online") >= 0) {
        networkOK = true;
        break;
      }
    }
    // Also check AT+CEREG? (LTE-specific registration)
    modem.sendAT("+CEREG?");
    if (modem.waitResponse(1000L, resp) == 1) {
      // +CEREG: 0,1 (home) or +CEREG: 0,5 (roaming) = registered
      if (resp.indexOf(",1") >= 0 || resp.indexOf(",5") >= 0) {
        networkOK = true;
        break;
      }
    }
    Serial.print(".");
    esp_task_wdt_reset();
    delay(2000);
  }

  if (!networkOK) {
    Serial.println(" FAILED");
    modem.sendAT("+CEREG?");
    if (modem.waitResponse(1000L, resp) == 1) {
      Serial.println("  CEREG: " + resp);
    }
    modem.sendAT("+CREG?");
    if (modem.waitResponse(1000L, resp) == 1) {
      Serial.println("  CREG: " + resp);
    }
    return false;
  }
  Serial.println(" registered!");

  Serial.println("  Operator: " + modem.getOperator());

  // â”€â”€ Activate PDP Context (APN already set in initModem) â”€â”€
  Serial.print("  Activating data (PDP)...");
  modem.sendAT("+CGACT=1,1");
  modem.waitResponse(5000L);

  // Verify IP assignment
  modem.sendAT("+CGPADDR=1");
  if (modem.waitResponse(3000L, resp) == 1) {
    Serial.println(" " + resp);
  }

  // If CGACT didn't assign an IP, try TinyGSM's gprsConnect as fallback
  bool dataOK = true;
  String ipStr = modem.localIP().toString();
  if (ipStr == "0.0.0.0" || ipStr.length() < 7) {
    dataOK = false;
    Serial.print("  Fallback: GPRS connect (" + String(GPRS_APN) + ")...");
    unsigned long gprsStart = millis();

    while (millis() - gprsStart < 20000 && !dataOK) {
      if (modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS)) {
        dataOK = true;
        break;
      }
      delay(1000);
    }

    // Try alternate Globe APN (prepaid vs postpaid)
    if (!dataOK) {
      Serial.println(" failed, trying alt APN...");
      Serial.print("  Connecting GPRS (" + String(GPRS_APN_ALT) + ")...");
      modem.sendAT("+CGDCONT=1,\"IP\",\"" + String(GPRS_APN_ALT) + "\"");
      modem.waitResponse(3000L);

      gprsStart = millis();
      while (millis() - gprsStart < 20000 && !dataOK) {
        if (modem.gprsConnect(GPRS_APN_ALT, GPRS_USER, GPRS_PASS)) {
          dataOK = true;
          break;
        }
        delay(1000);
      }
    }
  }

  if (!dataOK) {
    Serial.println(" FAILED");
    Serial.println("  CHECK: 1) SIM has active data (load/promo)");
    Serial.println("         2) Try dialing *143# on a phone with this SIM");
    Serial.println("         3) APN may need manual config via Globe app");
    return false;
  }

  Serial.println(" OK!");

  // Configure DNS servers (Google DNS)
  Serial.print("  Setting DNS...");
  modem.sendAT("+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\"");
  if (modem.waitResponse(5000L) == 1) {
    Serial.println(" OK");
  } else {
    Serial.println(" skip");
  }

  delay(2000); // Wait for network to stabilize

  Serial.println("  IP: " + modem.localIP().toString());
  Serial.println("  Operator: " + modem.getOperator());

  // â”€â”€ Connectivity Test via AT+HTTPINIT (modem-level HTTP) â”€â”€
  Serial.print("  Testing internet (AT+HTTP)...");
  sendATAndWait("+HTTPTERM", 1000); // Terminate any previous session
  delay(200);

  String httpResp;
  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000L, httpResp) != 1) {
    Serial.println(" HTTPINIT FAIL: " + httpResp);
    // Still mark as connected â€” GPRS is up, HTTP may work on next try
  } else {
    modem.sendAT("+HTTPPARA=\"CID\",1");
    modem.waitResponse(3000L);
    modem.sendAT("+HTTPPARA=\"URL\",\"http://www.google.com\"");
    modem.waitResponse(3000L);

    modem.sendAT("+HTTPACTION=0"); // GET
    // Wait for +HTTPACTION URC (up to 15s)
    unsigned long httpStart = millis();
    bool httpOK = false;
    while (millis() - httpStart < 15000) {
      if (modem.stream.available()) {
        String line = modem.stream.readStringUntil('\n');
        line.trim();
        if (line.indexOf("+HTTPACTION:") >= 0) {
          Serial.println(" " + line);
          // +HTTPACTION: 0,200,<len> means success
          if (line.indexOf(",200,") > 0 || line.indexOf(",301,") > 0 ||
              line.indexOf(",302,") > 0) {
            httpOK = true;
          }
          break;
        }
      }
      delay(50);
      esp_task_wdt_reset();
    }

    if (httpOK) {
      Serial.println("  Internet OK!");
    } else {
      Serial.println("  HTTP test failed (may still work for Firebase)");
    }

    sendATAndWait("+HTTPTERM", 1000);
  }

  lteConnected = true;
  return true;
}

int getSignal() {
  if (!modemOK)
    return -999;
  ModemLockGuard modemLock("getSignal", 150);
  if (!modemLock.ok()) return cachedSignalRssi;
  int csq = modem.getSignalQuality();
  return (csq == 99 || csq == 0) ? -999 : -113 + (2 * csq);
}

// ==================== NETWORK TIME (PHT UTC+8) ====================
// Days-in-month lookup (non-leap and leap year)
static const int daysInMonth[] = {31, 28, 31, 30, 31, 30,
                                  31, 31, 30, 31, 30, 31};

bool isLeapYear(int y) {
  return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

// Convert date/time components to Unix epoch (seconds since 1970-01-01 00:00:00
// UTC) Input is already in PHT (UTC+8), so we subtract 8 hours to get UTC epoch
unsigned long dateTimeToEpoch(int year, int month, int day, int hour, int min,
                              int sec) {
  unsigned long epoch = 0;

  // Sum days for complete years since 1970
  for (int y = 1970; y < year; y++) {
    epoch += isLeapYear(y) ? 366 : 365;
  }

  // Sum days for complete months in current year
  for (int m = 1; m < month; m++) {
    epoch += daysInMonth[m - 1];
    if (m == 2 && isLeapYear(year))
      epoch += 1;
  }

  // Add remaining days, hours, minutes, seconds
  epoch += (day - 1);
  epoch = epoch * 86400UL; // Convert days to seconds
  epoch += (unsigned long)hour * 3600UL;
  epoch += (unsigned long)min * 60UL;
  epoch += (unsigned long)sec;

  // The time from AT+CCLK? is local (PHT = UTC+8), convert to UTC epoch
  epoch -= (unsigned long)PHT_OFFSET_HOURS * 3600UL;

  return epoch;
}

// Parse AT+CCLK? response and extract Unix epoch
// Response format: +CCLK: "yy/MM/dd,hh:mm:ss+zz" (zz = timezone in quarters)
bool parseCCLK(const String &resp, unsigned long &outEpoch) {
  int idx = resp.indexOf("+CCLK: \"");
  if (idx < 0)
    return false;

  String timeStr = resp.substring(idx + 8); // Skip '+CCLK: "'
  // Expected: "yy/MM/dd,hh:mm:ss+zz" or "yy/MM/dd,hh:mm:ss-zz"

  if (timeStr.length() < 17)
    return false;

  int year = 2000 + timeStr.substring(0, 2).toInt();
  int month = timeStr.substring(3, 5).toInt();
  int day = timeStr.substring(6, 8).toInt();
  int hour = timeStr.substring(9, 11).toInt();
  int minute = timeStr.substring(12, 14).toInt();
  int sec = timeStr.substring(15, 17).toInt();

  // Sanity check: year should be >= 2024 (not 1970 or 1980)
  if (year < 2024 || month < 1 || month > 12 || day < 1 || day > 31) {
    Serial.printf("  CCLK parse: invalid date %d/%d/%d\n", year, month, day);
    return false;
  }

  outEpoch = dateTimeToEpoch(year, month, day, hour, minute, sec);

  Serial.printf(
      "  Parsed time: %04d-%02d-%02d %02d:%02d:%02d PHT (epoch: %lu)\n", year,
      month, day, hour, minute, sec, outEpoch);
  return true;
}

// Sync modem RTC from cellular network / NTP, then read it
bool syncNetworkTime() {
  ModemLockGuard modemLock("syncNetworkTime", 1000);
  if (!modemLock.ok()) return false;
  Serial.println("Syncing network time (PHT, UTC+8)...");
  String resp;

  // Step A: Enable automatic time zone update from network
  Serial.print("  Auto timezone update (AT+CTZU=1)...");
  modem.sendAT("+CTZU=1");
  if (modem.waitResponse(3000L) == 1) {
    Serial.println(" OK");
  } else {
    Serial.println(" skip");
  }

  // Step B: Configure and trigger NTP sync for reliability
  Serial.print("  NTP config (pool.ntp.org, UTC+8)...");
  modem.sendAT("+CNTP=\"pool.ntp.org\"," + String(PHT_OFFSET_QUARTERS));
  if (modem.waitResponse(3000L) == 1) {
    Serial.println(" OK");
  } else {
    Serial.println(" skip");
  }

  Serial.print("  NTP sync...");
  modem.sendAT("+CNTP");
  // Wait for +CNTP: 0 (success) URC â€” can take up to 30s
  unsigned long ntpStart = millis();
  bool ntpOK = false;
  while (millis() - ntpStart < 30000) {
    if (modem.stream.available()) {
      String line = modem.stream.readStringUntil('\n');
      line.trim();
      if (line.indexOf("+CNTP: 0") >= 0) {
        ntpOK = true;
        break;
      }
      if (line.indexOf("+CNTP:") >= 0) {
        Serial.print(" err:" + line);
        break;
      }
    }
    delay(100);
    esp_task_wdt_reset();
  }
  Serial.println(ntpOK ? " OK" : " timeout (trying CCLK anyway)");

  // Step C: Read RTC via AT+CCLK? (retry up to 3 times)
  for (int attempt = 0; attempt < 3; attempt++) {
    Serial.printf("  Reading RTC (attempt %d)...", attempt + 1);
    resp = "";
    modem.sendAT("+CCLK?");
    modem.waitResponse(3000L, resp);
    Serial.println(" " + resp);

    unsigned long parsed = 0;
    if (parseCCLK(resp, parsed)) {
      epochTime = parsed;
      epochSyncMillis = millis();
      timeSynced = true;
      Serial.println("  Time synced successfully!");
      return true;
    }

    // Modem RTC not yet updated, wait and retry
    Serial.println("  RTC not ready, retrying in 2s...");
    delay(2000);
  }

  // â”€â”€ Fallback: use compile-time date so we never report 1970-01-01 â”€â”€
  Serial.println(
      "  WARNING: NTP/CCLK failed â€” falling back to compile-time date");
  epochTime = getCompileTimeEpoch();
  epochSyncMillis = millis();
  timeSynced = true; // Mark synced so formatTimestamp produces a real date
  Serial.println("  Fallback time: " + formatTimestamp(epochTime));
  return false; // Still return false so caller knows it wasn't a live sync
}

// Get current Unix epoch by adding elapsed time since last sync
unsigned long getCurrentEpoch() {
  if (!timeSynced)
    return 0;
  return epochTime + ((millis() - epochSyncMillis) / 1000);
}

// Format epoch as ISO 8601 string in PHT: "YYYY-MM-DDTHH:MM:SS+08:00"
String formatTimestamp(unsigned long epoch) {
  if (epoch == 0)
    return "\"no_time\"";

  // Convert UTC epoch to PHT by adding 8 hours
  unsigned long local = epoch + (unsigned long)PHT_OFFSET_HOURS * 3600UL;

  unsigned long s = local;
  int sec = s % 60;
  s /= 60;
  int min = s % 60;
  s /= 60;
  int hour = s % 24;
  unsigned long days = s / 24;

  // Convert days since 1970-01-01 to year/month/day
  int year = 1970;
  while (true) {
    unsigned long diy = isLeapYear(year) ? 366 : 365;
    if (days < diy)
      break;
    days -= diy;
    year++;
  }

  int month = 1;
  while (month <= 12) {
    int dim = daysInMonth[month - 1];
    if (month == 2 && isLeapYear(year))
      dim++;
    if (days < (unsigned long)dim)
      break;
    days -= dim;
    month++;
  }
  int day = days + 1;

  char buf[30];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d+08:00", year, month,
           day, hour, min, sec);
  return String(buf);
}

// ==================== FIREBASE (AT+HTTP) ====================
static bool cloudWifiAvailable() {
  return WiFi.status() == WL_CONNECTED && wifiUplinkConnected && wifiCloudHealthy;
}

static String firebaseUrlForPath(const char *path) {
  String url = "https://" + String(FIREBASE_HOST);
  url += path;
  if (url.indexOf(".json") < 0) url += ".json";
  return url;
}

static inline void feedCloudIoWatchdog() {
  TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
  if (smartBoxLoopTaskHandle == NULL || currentTask == smartBoxLoopTaskHandle) {
    esp_task_wdt_reset();
  }
  yield();
}

static bool wifiFirebaseRequest(const char *method, const char *path,
                                const String *payload, String *bodyOut,
                                int *statusOut = NULL) {
  if (WiFi.status() != WL_CONNECTED) return false;

  feedCloudIoWatchdog();
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  String url = firebaseUrlForPath(path);
  feedCloudIoWatchdog();
  if (!http.begin(client, url)) return false;
  feedCloudIoWatchdog();
  http.setTimeout(8000);
  http.addHeader("Content-Type", "application/json");

  int code = -1;
  feedCloudIoWatchdog();
  if (strcmp(method, "GET") == 0) {
    code = http.GET();
  } else if (strcmp(method, "PUT") == 0) {
    code = http.PUT(*payload);
  } else if (strcmp(method, "PATCH") == 0) {
    code = http.sendRequest("PATCH", *payload);
  }
  feedCloudIoWatchdog();

  if (statusOut) *statusOut = code;
  if (code > 0 && bodyOut) {
    feedCloudIoWatchdog();
    *bodyOut = http.getString();
    feedCloudIoWatchdog();
  }
  http.end();
  feedCloudIoWatchdog();

  bool ok = (code >= 200 && code < 300);
  if (ok) {
    wifiCloudFailures = 0;
    wifiCloudHealthy = true;
  } else {
    if (wifiCloudFailures < 255) wifiCloudFailures++;
    if (wifiCloudFailures >= WIFI_CLOUD_FAILURE_LIMIT) {
      wifiCloudHealthy = false;
    }
  }
  return ok;
}

static bool wifiPutToFirebase(const char *path, const String &jsonData) {
  if (!cloudWifiAvailable()) return false;
  int status = -1;
  bool ok = wifiFirebaseRequest("PUT", path, &jsonData, NULL, &status);
  Serial.printf("[WIFI-CLOUD] PUT %s HTTP %d\n", path, status);
  return ok;
}

static bool wifiPatchToFirebase(const char *path, const String &jsonData) {
  if (!cloudWifiAvailable()) return false;
  int status = -1;
  bool ok = wifiFirebaseRequest("PATCH", path, &jsonData, NULL, &status);
  Serial.printf("[WIFI-CLOUD] PATCH %s HTTP %d\n", path, status);
  return ok;
}

static bool wifiGetFromFirebase(const char *path, String &bodyOut) {
  if (!cloudWifiAvailable()) return false;
  int status = -1;
  bool ok = wifiFirebaseRequest("GET", path, NULL, &bodyOut, &status);
  Serial.printf("[WIFI-CLOUD] GET %s HTTP %d\n", path, status);
  return ok;
}

static void pollWifiCloudHealth(unsigned long now) {
  if (WiFi.status() != WL_CONNECTED ||
      now - lastWifiCloudHealthAt < WIFI_CLOUD_HEALTH_INTERVAL_MS) {
    return;
  }
  lastWifiCloudHealthAt = now;
  String body;
  wifiCloudHealthy = wifiFirebaseRequest("GET", "/.json?shallow=true", NULL, &body, NULL);
}

// Uses A7670E's built-in HTTP client which handles SSL at modem level
bool httpPutToFirebase(const char *path, const String &jsonData) {
  if (wifiPutToFirebase(path, jsonData)) {
    return true;
  }
  ModemLockGuard modemLock("httpPutToFirebase", 250);
  if (!modemLock.ok()) return false;

  String resp;

  // Terminate any existing HTTP session
  sendATAndWait("+HTTPTERM", 1000);
  delay(200);

  // Initialize HTTP
  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000L, resp) != 1) {
    Serial.print("HTTPINIT FAIL ");
    return false;
  }

  // Set PDP context
  modem.sendAT("+HTTPPARA=\"CID\",1");
  modem.waitResponse(3000L);

  // Build Firebase URL: https://HOST/PATH.json
  String url = "https://" + String(FIREBASE_HOST) + path;
  if (strlen(FIREBASE_AUTH) > 0) {
    url += "?auth=" + String(FIREBASE_AUTH);
  }
  String urlCmd = "+HTTPPARA=\"URL\",\"" + url + "\"";
  modem.sendAT(urlCmd.c_str());
  modem.waitResponse(3000L);

  // Set content type for JSON
  modem.sendAT("+HTTPPARA=\"CONTENT\",\"application/json\"");
  modem.waitResponse(3000L);

  // Enable SSL (HTTPS)
  modem.sendAT("+HTTPSSL=1");
  modem.waitResponse(3000L);

  // Prepare data upload: AT+HTTPDATA=<size>,<timeout_ms>
  String dataCmd = "+HTTPDATA=" + String(jsonData.length()) + ",10000";
  modem.sendAT(dataCmd.c_str());

  // Wait for DOWNLOAD prompt
  unsigned long waitStart = millis();
  bool gotDownload = false;
  while (millis() - waitStart < 5000) {
    if (modem.stream.available()) {
      String line = modem.stream.readStringUntil('\n');
      line.trim();
      if (line.indexOf("DOWNLOAD") >= 0) {
        gotDownload = true;
        break;
      }
    }
    delay(10);
    esp_task_wdt_reset();
  }

  if (!gotDownload) {
    Serial.print("NO_DOWNLOAD_PROMPT ");
    sendATAndWait("+HTTPTERM", 1000);
    return false;
  }

  // Send the JSON data
  modem.stream.print(jsonData);
  delay(500);

  // Wait for OK after data upload
  modem.waitResponse(5000L);

  // Execute PUT request: AT+HTTPACTION=3 (PUT method)
  // Note: A7670E may use: 0=GET, 1=POST, 2=HEAD, 3=DELETE, 4=PUT
  // Some firmware versions: 0=GET, 1=POST, 3=PUT
  // Try PUT first (action=4 on some, action=1/POST as fallback)
  modem.sendAT("+HTTPACTION=4"); // PUT

  // Wait for +HTTPACTION URC response
  waitStart = millis();
  bool actionOK = false;
  int httpStatus = 0;
  while (millis() - waitStart < 30000) {
    if (modem.stream.available()) {
      String line = modem.stream.readStringUntil('\n');
      line.trim();
      if (line.indexOf("+HTTPACTION:") >= 0) {
        // Parse: +HTTPACTION: <method>,<status>,<data_len>
        int firstComma = line.indexOf(',');
        int secondComma = line.indexOf(',', firstComma + 1);
        if (firstComma > 0 && secondComma > 0) {
          httpStatus = line.substring(firstComma + 1, secondComma).toInt();
        }
        actionOK = true;
        break;
      }
      // Check if ERROR (method not supported)
      if (line.indexOf("ERROR") >= 0) {
        break;
      }
    }
    delay(50);
    esp_task_wdt_reset();
  }

  // If PUT (action=4) failed, use POST + X-HTTP-Method-Override: PUT
  if (!actionOK || httpStatus == 0) {
    Serial.print("(PUT unsupported, using Override)... ");
    sendATAndWait("+HTTPTERM", 1000);
    delay(200);

    modem.sendAT("+HTTPINIT");
    modem.waitResponse(5000L);
    modem.sendAT("+HTTPPARA=\"CID\",1");
    modem.waitResponse(3000L);

    String patchUrl = "https://" + String(FIREBASE_HOST) + path;
    if (strlen(FIREBASE_AUTH) > 0) {
      patchUrl += "?auth=" + String(FIREBASE_AUTH);
    }
    String patchUrlCmd = "+HTTPPARA=\"URL\",\"" + patchUrl + "\"";
    modem.sendAT(patchUrlCmd.c_str());
    modem.waitResponse(3000L);

    modem.sendAT("+HTTPPARA=\"CONTENT\",\"application/json\"");
    modem.waitResponse(3000L);

    modem.sendAT("+HTTPPARA=\"USERDATA\",\"X-HTTP-Method-Override: PUT\"");
    modem.waitResponse(3000L);

    modem.sendAT("+HTTPSSL=1");
    modem.waitResponse(3000L);

    // Upload data again
    dataCmd = "+HTTPDATA=" + String(jsonData.length()) + ",10000";
    modem.sendAT(dataCmd.c_str());

    waitStart = millis();
    gotDownload = false;
    while (millis() - waitStart < 5000) {
      if (modem.stream.available()) {
        String line = modem.stream.readStringUntil('\n');
        line.trim();
        if (line.indexOf("DOWNLOAD") >= 0) {
          gotDownload = true;
          break;
        }
      }
      delay(10);
      esp_task_wdt_reset();
    }

    if (gotDownload) {
      modem.stream.print(jsonData);
      delay(500);
      modem.waitResponse(5000L);

      // POST + X-HTTP-Method-Override: PUT ensures Firebase overwrites the node
      modem.sendAT("+HTTPACTION=1");

      waitStart = millis();
      actionOK = false;
      while (millis() - waitStart < 30000) {
        if (modem.stream.available()) {
          String line = modem.stream.readStringUntil('\n');
          line.trim();
          if (line.indexOf("+HTTPACTION:") >= 0) {
            int firstComma = line.indexOf(',');
            int secondComma = line.indexOf(',', firstComma + 1);
            if (firstComma > 0 && secondComma > 0) {
              httpStatus = line.substring(firstComma + 1, secondComma).toInt();
            }
            actionOK = true;
            break;
          }
        }
        delay(50);
        esp_task_wdt_reset();
      }
    }
  }

  // Clean up
  sendATAndWait("+HTTPTERM", 1000);

  if (actionOK && httpStatus == 200) {
    dataBytesOut += jsonData.length(); // Track data consumed
    return true;
  }

  Serial.printf("HTTP_%d ", httpStatus);
  return false;
}

// Uses A7670E's built-in HTTP client with X-HTTP-Method-Override to simulate a
// PATCH request This prevents wiping out fields like otp_code and delivery_id
// that other clients wrote to the same node
bool httpPatchToFirebase(const char *path, const String &jsonData) {
  if (wifiPatchToFirebase(path, jsonData)) {
    return true;
  }
  ModemLockGuard modemLock("httpPatchToFirebase", 250);
  if (!modemLock.ok()) return false;

  String resp;

  // Terminate any existing HTTP session
  sendATAndWait("+HTTPTERM", 1000);
  delay(200);

  // Initialize HTTP
  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000L, resp) != 1) {
    Serial.print("HTTPINIT FAIL ");
    return false;
  }

  // Set PDP context
  modem.sendAT("+HTTPPARA=\"CID\",1");
  modem.waitResponse(3000L);

  // Build Firebase URL: https://HOST/PATH.json
  String url = "https://" + String(FIREBASE_HOST) + path;
  if (strlen(FIREBASE_AUTH) > 0) {
    url += "?auth=" + String(FIREBASE_AUTH);
  }
  String urlCmd = "+HTTPPARA=\"URL\",\"" + url + "\"";
  modem.sendAT(urlCmd.c_str());
  modem.waitResponse(3000L);

  // Set content type for JSON
  modem.sendAT("+HTTPPARA=\"CONTENT\",\"application/json\"");
  modem.waitResponse(3000L);

  // Add the PATCH override header
  modem.sendAT("+HTTPPARA=\"USERDATA\",\"X-HTTP-Method-Override: PATCH\"");
  modem.waitResponse(3000L);

  // Enable SSL (HTTPS)
  modem.sendAT("+HTTPSSL=1");
  modem.waitResponse(3000L);

  // Prepare data upload: AT+HTTPDATA=<size>,<timeout_ms>
  String dataCmd = "+HTTPDATA=" + String(jsonData.length()) + ",10000";
  modem.sendAT(dataCmd.c_str());

  // Wait for DOWNLOAD prompt
  unsigned long waitStart = millis();
  bool gotDownload = false;
  while (millis() - waitStart < 5000) {
    if (modem.stream.available()) {
      String line = modem.stream.readStringUntil('\n');
      line.trim();
      if (line.indexOf("DOWNLOAD") >= 0) {
        gotDownload = true;
        break;
      }
    }
    delay(10);
    esp_task_wdt_reset();
  }

  if (!gotDownload) {
    Serial.print("NO_DOWNLOAD_PROMPT ");
    sendATAndWait("+HTTPTERM", 1000);
    return false;
  }

  // Send the JSON data
  modem.stream.print(jsonData);
  delay(500);

  // Wait for OK after data upload
  modem.waitResponse(5000L);

  // Execute POST request: AT+HTTPACTION=1 (POST method, overridden to PATCH by
  // header)
  modem.sendAT("+HTTPACTION=1");

  // Wait for +HTTPACTION URC response
  waitStart = millis();
  bool actionOK = false;
  int httpStatus = 0;
  while (millis() - waitStart < 30000) {
    if (modem.stream.available()) {
      String line = modem.stream.readStringUntil('\n');
      line.trim();
      if (line.indexOf("+HTTPACTION:") >= 0) {
        // Parse: +HTTPACTION: <method>,<status>,<data_len>
        int firstComma = line.indexOf(',');
        int secondComma = line.indexOf(',', firstComma + 1);
        if (firstComma > 0 && secondComma > 0) {
          httpStatus = line.substring(firstComma + 1, secondComma).toInt();
        }
        actionOK = true;
        break;
      }
    }
    delay(50);
    esp_task_wdt_reset();
  }

  // Clean up
  sendATAndWait("+HTTPTERM", 1000);

  if (actionOK && httpStatus == 200) {
    dataBytesOut += jsonData.length(); // Track data consumed
    return true;
  }

  Serial.printf("HTTP_%d ", httpStatus);
  return false;
}

// ==================== RELIABILITY FUNCTIONS ====================

// Hard-reset the modem via GPIO and re-initialize
void hardResetModem() {
  Serial.println("[RESET] Hard-resetting modem via GPIO...");
  digitalWrite(MODEM_RESET_PIN, LOW);
  delay(500);
  digitalWrite(MODEM_RESET_PIN, HIGH);
  delay(100);
  digitalWrite(MODEM_RESET_PIN, LOW);
  delay(5000); // Modem reboot time
  modemOK = initModem();
  if (modemOK) {
    lteConnected = connectLTE();
    if (lteConnected)
      syncNetworkTime();
  }
  consecutiveModemFailures = 0;
}

// Print heap/stack health to Serial for diagnostics
void printMemoryReport() {
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t minFreeHeap = ESP.getMinFreeHeap();
  uint32_t largestBlock = ESP.getMaxAllocHeap();
  Serial.printf("[MEM] Free: %u | Min: %u | Largest: %u | Stack HWM: %u\n",
                freeHeap, minFreeHeap, largestBlock,
                uxTaskGetStackHighWaterMark(NULL));
  Serial.printf("[MODEM_LOCK] owner=%s last=%s skips=%lu maxHeld=%lums stateSkips=%lu\n",
                currentModemOwner ? currentModemOwner : "unknown",
                lastModemOwner ? lastModemOwner : "unknown",
                (unsigned long)modemLockSkipCount,
                (unsigned long)modemLockMaxHeldMs,
                (unsigned long)stateLockSkipCount);

  uint32_t contextHeapNeeded =
      DELIVERY_CONTEXT_MAX_BYTES + DELIVERY_CONTEXT_HEAP_HEADROOM;
  if (largestBlock < contextHeapNeeded) {
    Serial.printf("[MEM] WARNING: Largest free block < %u bytes; context reads may truncate\n",
                  contextHeapNeeded);
  }
}

// Lightweight modem heartbeat â€” sends AT and checks for OK
void checkModemHealth() {
  if (!modemOK)
    return;
  ModemLockGuard modemLock("checkModemHealth", 150);
  if (!modemLock.ok()) return;
  modem.sendAT(""); // Simplest AT test
  if (modem.waitResponse(2000L) != 1) {
    consecutiveModemFailures++;
    Serial.printf("[HEALTH] Modem unresponsive (%u/%u)\n",
                  consecutiveModemFailures, MAX_CONSECUTIVE_FAILURES);
    if (consecutiveModemFailures >= MAX_CONSECUTIVE_FAILURES) {
      modemOK = false;
      lteConnected = false;
      scheduleModemRecovery("health_timeout");
      consecutiveModemFailures = 0;
    }
  } else {
    consecutiveModemFailures = 0;
  }
}

void scheduleModemRecovery(const char *reason) {
  if (modemRecoveryState == MODEM_RECOVERY_RUNNING || modemRecoveryRequested) {
    return;
  }

  modemRecoveryRequested = true;
  modemRecoveryState = MODEM_RECOVERY_REQUESTED;
  lteRecoveryAttemptCount++;
  lastLteRecoveryStartedAt = millis();
  modemRetryAt = millis() + modemRetryBackoffMs;
  Serial.printf("[RETRY] Scheduled modem recovery (%s)\n",
                reason ? reason : "unspecified");
}

void modemRecoveryTask(void *) {
  while (true) {
    if (modemRecoveryRequested && modemRecoveryState == MODEM_RECOVERY_REQUESTED) {
      modemRecoveryRequested = false;
      modemRecoveryState = MODEM_RECOVERY_RUNNING;

      unsigned long startedAt = millis();
      Serial.println("[RETRY] Async modem recovery started");

      if (!modemOK) {
        powerModem();
        modemOK = initModem();
      }

      if (modemOK && !lteConnected) {
        lteConnected = connectLTE();
        if (lteConnected) {
          syncNetworkTime();
        }
      }

      if (modemOK && lteConnected) {
        modemRetryBackoffMs = 10000;
      } else {
        modemRetryBackoffMs *= 2;
        if (modemRetryBackoffMs > MODEM_RETRY_BACKOFF_MAX_MS) {
          modemRetryBackoffMs = MODEM_RETRY_BACKOFF_MAX_MS;
        }
      }

      lastLteReconnectLatencyMs = millis() - startedAt;
      modemRetryAt = millis() + modemRetryBackoffMs;
      Serial.printf("[RETRY] Async modem recovery done | modem=%d lte=%d latency=%lums\n",
                    (int)modemOK, (int)lteConnected, lastLteReconnectLatencyMs);

      modemRecoveryState = MODEM_RECOVERY_IDLE;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void maybeEvaluateConnectivityMatrix(unsigned long now) {
  unsigned long camAgeMs = (camLastSeenAt > 0) ? (now - camLastSeenAt) : 0;
  bool camUp = camClientKnown && camLastSeenAt > 0 &&
               (camAgeMs <= CAM_LIVENESS_STALE_MS);
  unsigned long ctrlAgeMs =
      (controllerLastSeenAt > 0) ? (now - controllerLastSeenAt) : 0;
  bool ctrlUp = controllerLastSeenAt > 0 &&
                (ctrlAgeMs <= CONTROLLER_LIVENESS_STALE_MS);

  ConnectivityMatrixState nextState = CONN_BOTH_DOWN;
  if (camUp && ctrlUp) {
    nextState = CONN_ALL_UP;
  } else if (!camUp && ctrlUp) {
    nextState = CONN_CAM_DOWN_CTRL_UP;
  } else if (camUp && !ctrlUp) {
    nextState = CONN_CTRL_DOWN_CAM_UP;
  }

  connectivityState = nextState;
  if (connectivityState != lastConnectivityState) {
    Serial.printf("[LIVENESS] Connectivity matrix -> %s\n",
                  connectivityStateStr(connectivityState));
    if (connectivityState == CONN_CTRL_DOWN_CAM_UP) {
      if (cachedStatusStage == CMD_STAGE_PENDING && cachedStatus[0] != '\0') {
        // Keep pending command durable and ready to serve once controller returns.
        cachedStatusServesRemaining = STATUS_COMMAND_MAX_SERVES;
        cachedStatusSetAt = now;
        dpSaveCommandState(cachedStatus, DP_CMD_STAGE_PENDING,
                           cachedStatusServesRemaining, "", "");
        Serial.printf("[LIVENESS] Holding pending command until controller recovers: %s\n",
                      cachedStatus);
      }

      if (rebootAllPending) {
        rebootAllQueuedAtMs = now;
        rebootAllWaitLogged = false;
        Serial.println("[REBOOT_ALL] Controller down; waiting before proxy reboot");
      }
    } else if (connectivityState == CONN_BOTH_DOWN) {
      if (cachedStatusStage == CMD_STAGE_PENDING && cachedStatus[0] != '\0') {
        cachedStatusServesRemaining = STATUS_COMMAND_MAX_SERVES;
        cachedStatusSetAt = now;
        Serial.printf("[LIVENESS] Both peers down; command kept pending: %s\n",
                      cachedStatus);
      }
      rebootAllWaitLogged = false;
    } else {
      rebootAllWaitLogged = false;
    }
    lastConnectivityState = connectivityState;
  }
}

static void probeCameraIfDue(unsigned long now) {
  if (!apStarted || !camClientKnown || camClientIP == IPAddress(0, 0, 0, 0)) {
    return;
  }
  if (now - lastCamProbeAt < CAM_PROBE_INTERVAL_MS) {
    return;
  }
  lastCamProbeAt = now;

  unsigned long camAgeMs = (camLastSeenAt > 0) ? (now - camLastSeenAt) : 0;
  if (camAgeMs <= CAM_LIVENESS_STALE_MS / 2) {
    return;
  }

  WiFiClient camClient;
  camClient.setTimeout(1);
  if (!camClient.connect(camClientIP, CAM_FACE_PORT)) {
    if (camAgeMs > CAM_LIVENESS_STALE_MS) {
      Serial.printf("[LIVENESS] CAM probe failed; marking stale ip=%s age=%lums\n",
                    camClientIP.toString().c_str(), camAgeMs);
      camClientKnown = false;
    }
    camClient.stop();
    return;
  }

  camClient.print("GET /status HTTP/1.1\r\nHost: ");
  camClient.print(camClientIP);
  camClient.print("\r\nConnection: close\r\n\r\n");

  unsigned long deadline = millis() + 500UL;
  bool ok = false;
  while (millis() < deadline && (camClient.connected() || camClient.available())) {
    if (!camClient.available()) {
      delay(5);
      esp_task_wdt_reset();
      continue;
    }
    String line = camClient.readStringUntil('\n');
    line.trim();
    if (line.startsWith("HTTP/1.1 200") || line.startsWith("HTTP/1.0 200")) {
      ok = true;
      break;
    }
  }
  camClient.stop();

  if (ok) {
    camLastSeenAt = now;
    camClientKnown = true;
  }
}

static bool isControllerUp(unsigned long now) {
  if (controllerLastSeenAt == 0) {
    return false;
  }
  return (unsigned long)(now - controllerLastSeenAt) <=
         CONTROLLER_LIVENESS_STALE_MS;
}

void maybePublishDurabilityMetrics(unsigned long now) {
  if (!lteConnected) return;
  if (modemHttpBusy) return;
  if ((now - lastDurabilityMetricFlushAt) < DURABILITY_METRICS_INTERVAL_MS) {
    return;
  }

  DpWriteMetrics current = {0, 0, 0, 0};
  dpGetWriteMetrics(&current);

  unsigned long deliveryDelta = current.deliveryWrites - lastDpMetricsSnapshot.deliveryWrites;
  unsigned long commandDelta = current.commandWrites - lastDpMetricsSnapshot.commandWrites;
  unsigned long pinDelta = current.personalPinWrites - lastDpMetricsSnapshot.personalPinWrites;
  unsigned long auditDelta = current.auditWrites - lastDpMetricsSnapshot.auditWrites;

  char metricsJson[320];
  snprintf(metricsJson, sizeof(metricsJson),
           "{\"delivery_writes\":%lu,\"command_writes\":%lu,\"pin_writes\":%lu,"
           "\"audit_writes\":%lu,\"window_ms\":%lu,\"conn_state\":%u,"
           "\"lte_reconnect_last_ms\":%lu,\"lte_recovery_attempts\":%lu}",
           deliveryDelta,
           commandDelta,
           pinDelta,
           auditDelta,
           (unsigned long)DURABILITY_METRICS_INTERVAL_MS,
           (unsigned int)connectivityState,
           lastLteReconnectLatencyMs,
           lteRecoveryAttemptCount);

  char metricPath[80];
  snprintf(metricPath, sizeof(metricPath), "/hardware/%s/nvs_metrics.json",
           HARDWARE_ID);
  if (httpPutWithRetry(metricPath, metricsJson)) {
    lastDpMetricsSnapshot = current;
    lastDurabilityMetricFlushAt = now;
  }
}

// HTTP PUT with single retry (avoids overlapping with 5s send cycle)
bool httpPutWithRetry(const char *path, const char *jsonData) {
  if (httpPutToFirebase(path, String(jsonData)))
    return true;
  // Single retry after 1s backoff
  Serial.print("[RETRY] ");
  esp_task_wdt_reset();
  delay(1000);
  return httpPutToFirebase(path, String(jsonData));
}

// HTTP PATCH with single retry
bool httpPatchWithRetry(const char *path, const char *jsonData) {
  if (httpPatchToFirebase(path, String(jsonData)))
    return true;
  // Single retry after 1s backoff
  Serial.print("[PATCH RETRY] ");
  esp_task_wdt_reset();
  delay(1000);
  return httpPatchToFirebase(path, String(jsonData));
}

void publishPhotoUploadState(const char *status, int progress,
                             const char *deliveryId, size_t originalSize,
                             const char *errorMessage = "",
                             const char *uploadRole = "full") {
  const char *safeStatus = status && status[0] ? status : "PENDING";
  const char *safeDelivery =
      (deliveryId && deliveryId[0]) ? deliveryId : "UNKNOWN_DELIVERY";
  int clampedProgress = progress;
  if (clampedProgress < 0)
    clampedProgress = 0;
  if (clampedProgress > 100)
    clampedProgress = 100;

  photoUploadActive = strcmp(safeStatus, "COMPLETED") != 0 &&
                      strcmp(safeStatus, "FAILED") != 0;
  photoUploadProgress = clampedProgress;
  strncpy(photoUploadStatus, safeStatus, sizeof(photoUploadStatus) - 1);
  photoUploadStatus[sizeof(photoUploadStatus) - 1] = '\0';
  strncpy(photoUploadError, errorMessage ? errorMessage : "",
          sizeof(photoUploadError) - 1);
  photoUploadError[sizeof(photoUploadError) - 1] = '\0';
  photoUploadUpdatedAt = millis();

  if (!lteConnected) {
    return;
  }

  char json[448];
  snprintf(json, sizeof(json),
           "{\"delivery_id\":\"%s\",\"status\":\"%s\","
           "\"progress_percent\":%d,\"original_size_bytes\":%lu,"
           "\"compressed_size_bytes\":%lu,\"compression_ratio\":1,"
           "\"retry_count\":0,\"error_message\":\"%s\","
           "\"last_upload_role\":\"%s\","
           "\"upload_updated_at\":{\".sv\":\"timestamp\"}}",
           safeDelivery, safeStatus, clampedProgress,
           (unsigned long)originalSize, (unsigned long)originalSize,
           errorMessage ? errorMessage : "",
           (uploadRole && uploadRole[0]) ? uploadRole : "full");

  char path[80];
  snprintf(path, sizeof(path), "/hardware/%s/photo_upload.json", HARDWARE_ID);
  httpPatchWithRetry(path, json);
}

// ==================== FIREBASE SEND (snprintf â€” zero String allocs)
// ====================
void sendToFirebase() {
  if (!wifiCloudHealthy && !lteConnected) {
    Serial.println("Firebase: no cloud transport connected");
    return;
  }
  ModemLockGuard modemLock("sendToFirebase", 250);
  if (!modemLock.ok()) return;

  Serial.print("Firebase... ");

  // Derive box status — theft overrides normal status
  const char *boxStatus = isBoxLocked ? "LOCKED" : "UNLOCKING";
  TheftState tState = theftGuardGetState();
  if (tState == TG_STOLEN || tState == TG_LOCKDOWN) {
    boxStatus = theftGuardStateStr();
  }

  // Get current timestamp
  unsigned long now_epoch = getCurrentEpoch();
  char tsBuf[30];
  String tsStr = formatTimestamp(now_epoch);
  strncpy(tsBuf, tsStr.c_str(), sizeof(tsBuf) - 1);
  tsBuf[sizeof(tsBuf) - 1] = '\0';

  // Read signal/operator once (avoid multiple AT round-trips)
  int rssi = getSignal();
  int csq = modem.getSignalQuality();
  cachedSignalRssi = rssi;
  cachedSignalCsq = csq;
  String opStr = modem.getOperator();
  char opBuf[32];
  strncpy(opBuf, opStr.c_str(), sizeof(opBuf) - 1);
  opBuf[sizeof(opBuf) - 1] = '\0';

  // â”€â”€ Build hardware JSON with snprintf (zero heap allocs) â”€â”€
  // Use Firebase server timestamp {".sv":"timestamp"} for last_updated
  // so the web dashboard always gets an accurate UTC ms timestamp,
  // regardless of whether the ESP32's NTP sync succeeded.
  // uptime_ms = millis() â€” device time since boot, survives web page refresh.
  char hardwareJson[1152];
  snprintf(hardwareJson, sizeof(hardwareJson),
           "{\"status\":\"%s\","
           "\"connection\":\"%s\","
           "\"rssi\":%d,"
           "\"csq\":%d,"
           "\"op\":\"%s\","
           "\"gps_fix\":%s,"
           "\"data_bytes\":%lu,"
           "\"uptime_ms\":%lu,"
           "\"last_updated\":{\".sv\":\"timestamp\"},"
           "\"device_epoch\":%lu,"
           "\"last_updated_str\":\"%s\","
           "\"time_synced\":%s,"
           "\"batt_v\":%.2f,"
           "\"batt_pct\":%d,"
           "\"batt_mah\":%lu,"
           "\"batt_cap_mah\":%lu,"
           "\"batt_pin_mv\":%d,"
           "\"batt_adc\":%d,"
           "\"batt_sensor_ok\":%s,"
           "\"batt_low\":%s,"
           "\"batt_b_v\":%.2f,"
           "\"batt_b_pct\":%d,"
           "\"batt_b_mah\":%lu,"
           "\"batt_b_cap_mah\":%lu,"
           "\"batt_b_sensor_ok\":%s,"
           "\"batt_b_low\":%s,"
           "\"geo_state\":\"%s\","
           "\"geo_dist_m\":%.1f,"
           "\"theft_state\":\"%s\"}",
           boxStatus, wifiCloudHealthy ? "WIFI" : "LTE",
           rssi, csq, opBuf, gpsFix ? "true" : "false", dataBytesOut,
           (unsigned long)millis(), now_epoch, tsBuf,
           timeSynced ? "true" : "false",
           batteryGetVoltage(BATT_CH_A), batteryGetPercentage(BATT_CH_A),
           (unsigned long)batteryGetRemainingMah(BATT_CH_A),
           (unsigned long)batteryGetCapacityMah(BATT_CH_A),
           batteryGetPinMilliVolts(BATT_CH_A),
           batteryGetAdcRaw(BATT_CH_A),
           batterySensorLooksValid(BATT_CH_A) ? "true" : "false",
           batteryIsLow(BATT_CH_A) ? "true" : "false",
           batteryGetVoltage(BATT_CH_B), batteryGetPercentage(BATT_CH_B),
           (unsigned long)batteryGetRemainingMah(BATT_CH_B),
           (unsigned long)batteryGetCapacityMah(BATT_CH_B),
           batterySensorLooksValid(BATT_CH_B) ? "true" : "false",
           batteryIsLow(BATT_CH_B) ? "true" : "false",
           geoProxy.stateStr(geoProxy.snap.stableState),
           geoProxy.snap.distanceM,
           theftGuardStateStr());

  // Build path with snprintf
  char hwPath[64];
  snprintf(hwPath, sizeof(hwPath), "/hardware/%s.json", HARDWARE_ID);

  bool hwOK = httpPatchWithRetry(hwPath, hardwareJson);
  if (hwOK) {
    Serial.print("HW:OK ");
    firebaseFailures = 0;
  } else {
    Serial.print("HW:FAIL ");
    firebaseFailures++;
  }

  // â”€â”€ Build location JSON with snprintf â”€â”€
  if (gpsFix) {
    char locationJson[384];
    snprintf(locationJson, sizeof(locationJson),
             "{\"latitude\":%.6f,"
             "\"longitude\":%.6f,"
             "\"heading\":%.1f,"
             "\"speed\":%.2f,"
             "\"hdop\":%.2f,"
             "\"timestamp\":%lu,"
             "\"timestamp_str\":\"%s\","
             "\"source\":\"box\"}",
             gpsLat, gpsLon,
             gpsCourse >= 0.0f ? gpsCourse : -1.0f,
             gpsSpeed,
             gpsHdop, now_epoch, tsBuf);

    char locPath[64];
    snprintf(locPath, sizeof(locPath), "/locations/%s/box.json", HARDWARE_ID);

    if (httpPutWithRetry(locPath, locationJson)) {
      Serial.println("LOC:OK!");
    } else {
      Serial.println("LOC:FAIL");
      firebaseFailures++;
    }
  } else {
    Serial.println("(no GPS)");
  }

  // Auto-reconnect LTE after consecutive Firebase failures
  if (firebaseFailures >= MAX_FB_FAILURES) {
    Serial.printf(
        "[RELIABILITY] %u consecutive Firebase failures â€” reconnecting LTE\n",
        firebaseFailures);
    lteConnected = false;
    scheduleModemRecovery("firebase_failures");
    firebaseFailures = 0;
  }
}

// ==================== HOTSPOT + CAMERA PROXY ====================

static int selectBestHotspot(char *ssidOut, size_t ssidLen,
                             char *passOut, size_t passLen) {
  feedCloudIoWatchdog();
  int n = WiFi.scanNetworks();
  feedCloudIoWatchdog();
  int bestIdx = -1;
  int bestCredential = -1;
  int bestRssi = -999;
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    Serial.printf("[WIFI]   %s (%d dBm)\n", ssid.c_str(), WiFi.RSSI(i));
  }

  for (uint8_t offset = 0; offset < HOTSPOT_COUNT; offset++) {
    uint8_t h = (hotspotScanStart + offset) % HOTSPOT_COUNT;
    if (!HOTSPOTS[h].ssid || HOTSPOTS[h].ssid[0] == '\0') continue;
    for (int i = 0; i < n; i++) {
      if (WiFi.SSID(i) == HOTSPOTS[h].ssid && WiFi.RSSI(i) > bestRssi) {
        bestIdx = i;
        bestCredential = h;
        bestRssi = WiFi.RSSI(i);
      }
    }
  }
  if (bestIdx < 0 || bestCredential < 0) {
    WiFi.scanDelete();
    selectedHotspotCredential = -1;
    return -1000;
  }
  strncpy(ssidOut, HOTSPOTS[bestCredential].ssid, ssidLen - 1);
  ssidOut[ssidLen - 1] = '\0';
  strncpy(passOut, HOTSPOTS[bestCredential].password, passLen - 1);
  passOut[passLen - 1] = '\0';
  selectedHotspotCredential = bestCredential;
  WiFi.scanDelete();
  return bestRssi;
}

static void advanceHotspotCandidate(const char *reason) {
  if (selectedHotspotCredential >= 0 && HOTSPOT_COUNT > 0) {
    hotspotScanStart = ((uint8_t)selectedHotspotCredential + 1) % HOTSPOT_COUNT;
    Serial.printf("[WIFI] Advancing hotspot after %s; next scan starts at #%u\n",
                  reason ? reason : "failure", hotspotScanStart);
  }
}

static void startHotspotStation() {
  char ssid[33] = "";
  char pass[65] = "";
  int rssi = selectBestHotspot(ssid, sizeof(ssid), pass, sizeof(pass));
  if (rssi < -998 || ssid[0] == '\0') {
    Serial.println("[WIFI] No configured hotspot found; LTE fallback remains available");
    wifiUplinkConnected = false;
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(78);
  WiFi.disconnect(false);
  feedCloudIoWatchdog();
  WiFi.begin(ssid, pass);
  Serial.printf("[WIFI] Connecting to hotspot '%s' (%d dBm)\n", ssid, rssi);

  unsigned long deadline = millis() + 15000;
  while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
    esp_task_wdt_reset();
    delay(250);
    yield();
  }
  esp_task_wdt_reset();

  wifiUplinkConnected = (WiFi.status() == WL_CONNECTED);
  if (wifiUplinkConnected) {
    Serial.printf("[WIFI] Connected. IP=%s gw=%s\n",
                  WiFi.localIP().toString().c_str(),
                  WiFi.gatewayIP().toString().c_str());
    udpServer.begin(UDP_LOG_PORT);
    discoveryServer.begin(PROXY_DISCOVERY_PORT);
    Serial.printf("[DISCOVERY] Listening on UDP %d\n", PROXY_DISCOVERY_PORT);
    camServer.begin();
    Serial.printf("[WIFI] Controller HTTP server listening on port %d\n",
                  CAM_SERVER_PORT);
    wifiCloudHealthy = true;
    apStarted = true;
  } else {
    Serial.println("[WIFI] Hotspot connect failed; LTE fallback remains available");
    advanceHotspotCandidate("connect_failed");
  }
}

static void maintainHotspotStation(unsigned long now) {
  static unsigned long nextRetryAt = 0;
  if (WiFi.status() == WL_CONNECTED) {
    wifiUplinkConnected = true;
    return;
  }

  wifiUplinkConnected = false;
  wifiCloudHealthy = false;
  if (now < nextRetryAt) return;
  nextRetryAt = now + 15000;
  startHotspotStation();
}

static void pollDiscoveryServer() {
  int packetSize = discoveryServer.parsePacket();
  if (packetSize <= 0) return;
  IPAddress remoteIp = discoveryServer.remoteIP();
  uint16_t remotePort = discoveryServer.remotePort();
  char packet[96];
  int len = discoveryServer.read(packet, sizeof(packet) - 1);
  if (len < 0) len = 0;
  packet[len] = '\0';
  if (strncmp(packet, "SMART_TOP_BOX_CAM:", 18) == 0) {
    const char *cameraId = packet + 18;
    if (strcmp(cameraId, CAM_PREFIX) != 0) {
      Serial.printf("[DISCOVERY] Ignoring CAM %s; expected %s\n",
                    cameraId[0] ? cameraId : "UNKNOWN", CAM_PREFIX);
      return;
    }
    camClientIP = remoteIp;
    camClientKnown = true;
    camLastSeenAt = millis();
    Serial.printf("[DISCOVERY] CAM at %s (%s)\n",
                  camClientIP.toString().c_str(), packet);
    return;
  }
  if (strcmp(packet, PROXY_DISCOVERY_QUERY) != 0) return;
  Serial.printf("[DISCOVERY] Controller query from %s:%u\n",
                remoteIp.toString().c_str(), remotePort);

  char reply[64];
  snprintf(reply, sizeof(reply), "%s%s:%d:%s",
           PROXY_DISCOVERY_REPLY,
           WiFi.localIP().toString().c_str(),
           CAM_SERVER_PORT,
           HARDWARE_ID);

  // Reply on several paths. Phone hotspots can be inconsistent with
  // client-to-client UDP: the controller's broadcast may arrive while a
  // unicast reply to its random source port is dropped.
  discoveryServer.beginPacket(remoteIp, remotePort);
  discoveryServer.print(reply);
  discoveryServer.endPacket();
  if (remotePort != PROXY_DISCOVERY_PORT) {
    discoveryServer.beginPacket(remoteIp, PROXY_DISCOVERY_PORT);
    discoveryServer.print(reply);
    discoveryServer.endPacket();
  }
  IPAddress broadcast((uint32_t)WiFi.localIP() | ~(uint32_t)WiFi.subnetMask());
  discoveryServer.beginPacket(broadcast, PROXY_DISCOVERY_PORT);
  discoveryServer.print(reply);
  discoveryServer.endPacket();
  Serial.printf("[DISCOVERY] Reply sent: %s\n", reply);
}

// Start WiFi station mode so all boards share the phone/router hotspot.
void startHotspot() {
  Serial.println("\nStarting hotspot-first WiFi station...");
  startHotspotStation();
  if (wifiUplinkConnected) {
    Serial.printf("[WIFI] Controller/CAM should join same hotspot; proxy IP %s:%d\n",
                  WiFi.localIP().toString().c_str(), CAM_SERVER_PORT);
    Serial.printf("[UDP] Logs on %d, discovery on %d\n",
                  UDP_LOG_PORT, PROXY_DISCOVERY_PORT);
  }
}

static uint8_t uartChecksum8(const char *text) {
  uint8_t sum = 0;
  if (!text) return 0;
  while (*text) sum ^= (uint8_t)*text++;
  return sum;
}

static void beginProxyUart() {
  pinMode(PROXY_UART_RX, INPUT_PULLUP);
  pinMode(PROXY_UART_TX, OUTPUT);
  digitalWrite(PROXY_UART_TX, HIGH);
  proxySerial.begin(PROXY_UART_BAUD, SERIAL_8N1, PROXY_UART_RX, PROXY_UART_TX);
  proxySerial.setTimeout(5000);
  Serial.printf("[PROXY-UART] Serial2 ready RX=%d TX=%d baud=%d\n",
                PROXY_UART_RX, PROXY_UART_TX, PROXY_UART_BAUD);
}

static void sendProxyUartResponse(uint16_t id, int status, const String &body) {
  String frame = "RESP|" + String(id) + "|" + String(status) + "|" + body;
  uint8_t sum = uartChecksum8(frame.c_str());
  proxySerial.print(frame);
  proxySerial.print("|");
  if (sum < 16) proxySerial.print("0");
  proxySerial.println(sum, HEX);
  proxySerial.flush();
  Serial.printf("[PROXY-UART] Response id=%u status=%d len=%u\n",
                (unsigned int)id, status, (unsigned int)body.length());
}

static String buildControllerOtpBody() {
  String otpPart = returnActive
                       ? (returnOtpCacheValid ? String(cachedReturnOtp) : "NO_OTP")
                       : (otpCacheValid ? String(cachedOtp) : "NO_OTP");
  bool cloudReachable = wifiCloudHealthy || lteConnected;
  bool staleUsableContext =
      deliveryIdCacheValid && cachedDeliveryId[0] != '\0' &&
      (deliveryContextStale || !cloudReachable || lastContextFetchOkAtMs == 0);
  bool activeContextForController =
      deliveryIdCacheValid && cachedDeliveryId[0] != '\0' &&
      (deliveryContextTrusted || staleUsableContext);
  String delPart = activeContextForController ? String(cachedDeliveryId) : "NO_DELIVERY";
  String body = otpPart + "," + delPart + ",";

  bool statusActive =
      cachedStatusStage == CMD_STAGE_PENDING && cachedStatus[0] != '\0';
  if (statusActive) {
    maybeRearmPendingCommand(millis());
    body += String(cachedStatus);
    if (cachedStatusServesRemaining > 0) cachedStatusServesRemaining--;
  }

  bool insideGeo = geoProxy.isArrived();
  int distMeters = (int)geoProxy.snap.distanceM;
  if (!gpsFix && distMeters <= 0) distMeters = -1;
  const char *phaseStr = returnActive ? "RETURN" :
                         (!activeContextForController ? "NONE" :
                          (strcmp(cachedDeliveryStatus, "ARRIVED") == 0 ? "DROPOFF" :
                           (strcmp(cachedDeliveryStatus, "IN_TRANSIT") == 0 ||
                            strcmp(cachedDeliveryStatus, "PICKED_UP") == 0 ? "IN_TRANSIT" : "PICKUP")));
  char geoFields[144];
  snprintf(geoFields, sizeof(geoFields),
           ",DIST:%d,GEO:%d,RET:%d,PHASE:%s,PUP:%d,DRO:%d,CTX:%s,REFQ:%d",
           distMeters, insideGeo ? 1 : 0, returnActive ? 1 : 0,
           phaseStr, pickupCoordsValid ? 1 : 0, insideGeo ? 1 : 0,
           deliveryContextStale ? "STALE" :
               (deliveryContextTrusted ? "FRESH" : "UNVERIFIED"),
           controllerContextRefreshQueued ? 1 : 0);
  body += String(geoFields);
  return body;
}

static String buildDiagBody() {
  unsigned long nowMs = millis();
  unsigned long camAgeMs = (camLastSeenAt > 0) ? (nowMs - camLastSeenAt) : 0;
  bool camUp = camClientKnown && camLastSeenAt > 0 &&
               (camAgeMs <= CAM_LIVENESS_STALE_MS);
  unsigned long ctrlAgeMs =
      (controllerLastSeenAt > 0) ? (nowMs - controllerLastSeenAt) : 0;
  bool ctrlUp = controllerLastSeenAt > 0 &&
                (ctrlAgeMs <= CONTROLLER_LIVENESS_STALE_MS);
  char body[760];
  snprintf(body, sizeof(body),
           "batt_pct=%d,batt_v=%.2f,batt_mah=%lu,batt_cap_mah=%lu,batt_pin_mv=%d,batt_adc=%d,batt_ok=%d,batt_b_pct=%d,batt_b_v=%.2f,batt_b_mah=%lu,batt_b_ok=%d,rssi=%d,csq=%d,gps_fix=%d,lte=%d,modem=%d,time=%d,fb_fail=%u,uptime=%lu,cam_up=%d,cam_age=%lu,ctrl_up=%d,ctrl_age=%lu,cmd_stage=%u,cmd_pending=%d,conn_state=%u,lte_reconn_ms=%lu,upload_active=%d,upload_pct=%d,upload_status=%s,upload_age=%lu",
           batteryGetPercentage(BATT_CH_A), batteryGetVoltage(BATT_CH_A),
           (unsigned long)batteryGetRemainingMah(BATT_CH_A),
           (unsigned long)batteryGetCapacityMah(BATT_CH_A),
           batteryGetPinMilliVolts(BATT_CH_A),
           batteryGetAdcRaw(BATT_CH_A),
           batterySensorLooksValid(BATT_CH_A) ? 1 : 0,
           batteryGetPercentage(BATT_CH_B), batteryGetVoltage(BATT_CH_B),
           (unsigned long)batteryGetRemainingMah(BATT_CH_B),
           batterySensorLooksValid(BATT_CH_B) ? 1 : 0,
           cachedSignalRssi, cachedSignalCsq,
           gpsFix ? 1 : 0,
           (wifiCloudHealthy || lteConnected) ? 1 : 0,
           (wifiCloudHealthy || modemOK) ? 1 : 0,
           timeSynced ? 1 : 0,
           (unsigned int)firebaseFailures,
           (unsigned long)nowMs,
           camUp ? 1 : 0,
           (unsigned long)camAgeMs,
           ctrlUp ? 1 : 0,
           (unsigned long)ctrlAgeMs,
           (unsigned int)cachedStatusStage,
           (cachedStatusStage == CMD_STAGE_PENDING) ? 1 : 0,
           (unsigned int)connectivityState,
           lastLteReconnectLatencyMs,
           photoUploadActive ? 1 : 0,
           photoUploadProgress,
           photoUploadStatus,
           photoUploadUpdatedAt > 0 ? (unsigned long)(nowMs - photoUploadUpdatedAt) : 0);
  return String(body);
}

static void handleProxyUartRequest(const String &line) {
  int p1 = line.indexOf('|');
  int p2 = line.indexOf('|', p1 + 1);
  int p3 = line.indexOf('|', p2 + 1);
  int p4 = line.indexOf('|', p3 + 1);
  int p5 = line.lastIndexOf('|');
  if (!line.startsWith("REQ|") || p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0 || p5 <= p4) return;

  String withoutCrc = line.substring(0, p5);
  uint8_t expected = (uint8_t)strtoul(line.substring(p5 + 1).c_str(), NULL, 16);
  if (uartChecksum8(withoutCrc.c_str()) != expected) {
    Serial.println("[PROXY-UART] Bad checksum");
    return;
  }

  uint16_t id = (uint16_t)line.substring(p1 + 1, p2).toInt();
  String method = line.substring(p2 + 1, p3);
  String path = line.substring(p3 + 1, p4);
  String payload = line.substring(p4 + 1, p5);
  controllerLastSeenAt = millis();
  Serial.printf("[PROXY-UART] Request id=%u %s %s\n",
                (unsigned int)id, method.c_str(), path.c_str());

  if (method == "GET" && path.startsWith("/ping")) {
    sendProxyUartResponse(id, 200, "OK");
  } else if (method == "GET" && path.startsWith("/otp")) {
    sendProxyUartResponse(id, 200, buildControllerOtpBody());
  } else if (method == "GET" && path.startsWith("/diag")) {
    sendProxyUartResponse(id, 200, buildDiagBody());
  } else if (method == "GET" && path.startsWith("/refresh-context")) {
    bool cloudReachable = wifiCloudHealthy || lteConnected;
    controllerContextRefreshQueued = true;
    controllerContextRefreshQueuedAt = millis();
    if (!cloudReachable) deliveryContextStale = true;
    strncpy(lastRefreshStatus,
            cloudReachable ? "OK:REFRESH_QUEUED" : "WAIT:REFRESH_QUEUED",
            sizeof(lastRefreshStatus) - 1);
    lastRefreshStatus[sizeof(lastRefreshStatus) - 1] = '\0';
    sendProxyUartResponse(id, 200, lastRefreshStatus);
  } else if (method == "GET" && path.startsWith("/face-check")) {
    String result = forwardFaceCheck("");
    sendProxyUartResponse(id, 200, result);
  } else if (method == "GET" && path.startsWith("/cam-power")) {
    String mode = path.indexOf("wake") >= 0 ? "wake" : "sleep";
    sendProxyUartResponse(id, 200, forwardCameraPowerCommand(mode));
  } else if (method == "POST" && path == "/personal-pin-verify") {
    char pin[12] = "";
    extractJsonStringValue(payload.c_str(), "pin", pin, sizeof(pin));
    bool currentlyLocked = payload.indexOf("\"currently_locked\":true") >= 0;
    const char *resultBody = "DENY:invalid";
    if (!personalPinEnabled) resultBody = "DENY:disabled";
    else if (pin[0] == '\0') resultBody = "DENY:missing_pin";
    else if (verifyPersonalPinLocal(pin)) resultBody = currentlyLocked ? "ALLOW_UNLOCK" : "ALLOW_RELOCK";
    else resultBody = "DENY:mismatch";
    sendProxyUartResponse(id, 200, resultBody);
  } else if (method == "POST" && path == "/command-ack") {
    char cmd[24] = "";
    char status[32] = "";
    char details[96] = "";
    extractJsonStringValue(payload.c_str(), "command", cmd, sizeof(cmd));
    extractJsonStringValue(payload.c_str(), "status", status, sizeof(status));
    extractJsonStringValue(payload.c_str(), "details", details, sizeof(details));
    sendProxyUartResponse(id, 200, "OK");
    queueCommandAckFirebaseWrite(cmd, status, details, false);
  } else if (method == "POST" && path == "/event") {
    sendProxyUartResponse(id, 200, "OK");
    if (payload.indexOf("\"tamper\":true") >= 0) {
      queueTamperFirebaseWrite();
    } else {
      String jsonStr = payload;
      bool ov = payload.indexOf("\"otp_valid\":true") >= 0;
      bool fd = payload.indexOf("\"face_detected\":true") >= 0;
      bool ul = payload.indexOf("\"unlocked\":true") >= 0;
      int faceAttempts = 0;
      bool faceRetryExhausted = false;
      bool fallbackRequired = false;
      int unlockLatencyMs = 0;
      char failureReason[24] = "";
      readTopLevelJsonInt(jsonStr, "face_attempts", faceAttempts);
      readTopLevelJsonBool(jsonStr, "face_retry_exhausted", faceRetryExhausted);
      readTopLevelJsonBool(jsonStr, "fallback_required", fallbackRequired);
      readTopLevelJsonInt(jsonStr, "unlock_latency_ms", unlockLatencyMs);
      readTopLevelJsonString(jsonStr, "failure_reason", failureReason, sizeof(failureReason));
      queueLockEventFirebaseWrite(ov, fd, ul, faceAttempts, faceRetryExhausted,
                                  fallbackRequired, failureReason,
                                  (unsigned long)(unlockLatencyMs > 0 ? unlockLatencyMs : 0));
    }
  } else {
    sendProxyUartResponse(id, 404, "NOT_FOUND");
  }
}

static void pollProxyUart() {
  static char line[900];
  static uint16_t len = 0;
  while (proxySerial.available()) {
    char c = (char)proxySerial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[len] = '\0';
      if (len > 0) handleProxyUartRequest(String(line));
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    } else {
      len = 0;
    }
  }
}

// Forward a JPEG image from the ESP32-CAM to Supabase Storage using A7670E LTE.
bool uploadToSupabaseViaLTE(const uint8_t *data, size_t len,
                            const String &objectPath) {
  relayDiag = ""; // reset for this attempt
  if (!lteConnected) {
    relayDiag = "FAIL:lte_not_connected";
    Serial.println("[RELAY] LTE not connected, cannot upload");
    return false;
  }
  Serial.printf("[RELAY] Uploading %u bytes â†’ Supabase path: %s\n", len,
                objectPath.c_str());

  ModemLockGuard modemLock("uploadToSupabaseViaLTE", 250);
  if (!modemLock.ok()) {
    relayDiag = "BUSY";
    return false;
  }

  String resp;
  sendATAndWait("+HTTPTERM", 1000);
  delay(200);

  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000L, resp) != 1) {
    relayDiag = "FAIL:httpinit_failed";
    Serial.println("[RELAY] HTTPINIT failed");
    return false;
  }

  modem.sendAT("+HTTPPARA=\"CID\",1");
  modem.waitResponse(3000L);

  String url = String(SUPABASE_URL) + "/storage/v1/object/" +
               String(SUPABASE_BUCKET) + "/" + objectPath;
  modem.sendAT(("+HTTPPARA=\"URL\",\"" + url + "\"").c_str());
  modem.waitResponse(3000L);

  modem.sendAT("+HTTPPARA=\"CONTENT\",\"image/jpeg\"");
  modem.waitResponse(3000L);

  // Cleaned up the header.
  // Removed the \r\n injection which can truncate AT commands over UART.
  String hdrs = String("Authorization: Bearer ") + SUPABASE_ANON_KEY;
  modem.sendAT(("+HTTPPARA=\"USERDATA\",\"" + hdrs + "\"").c_str());
  modem.waitResponse(3000L);

  // â”€â”€ SSL context configuration â”€â”€
  modem.sendAT("+CSSLCFG=\"sslversion\",1,4"); // TLS 1.2
  modem.waitResponse(2000L);
  modem.sendAT("+CSSLCFG=\"ignorertctime\",1,1"); // ignore RTC for cert expiry
  modem.waitResponse(2000L);
  modem.sendAT("+CSSLCFG=\"authmode\",1,0"); // 0 = no server cert check
  modem.waitResponse(2000L);

  // Enable SNI. This is strictly required by Supabase's cloud infrastructure.
  modem.sendAT("+CSSLCFG=\"enableSNI\",1,1");
  modem.waitResponse(2000L);

  modem.sendAT("+HTTPPARA=\"SSLCFG\",1"); // attach SSL context 1 to HTTP
  modem.waitResponse(2000L);
  modem.sendAT("+HTTPSSL=1");
  modem.waitResponse(3000L);

  // Tell modem how many binary bytes we are about to send (30s upload window)
  modem.sendAT(("+HTTPDATA=" + String(len) + ",30000").c_str());

  // Wait for DOWNLOAD prompt
  unsigned long waitStart = millis();
  bool gotDownload = false;
  while (millis() - waitStart < 5000) {
    if (modem.stream.available()) {
      String line = modem.stream.readStringUntil('\n');
      if (line.indexOf("DOWNLOAD") >= 0) {
        gotDownload = true;
        break;
      }
    }
    delay(10);
    esp_task_wdt_reset();
  }
  if (!gotDownload) {
    relayDiag = "FAIL:no_download_prompt";
    Serial.println("[RELAY] No DOWNLOAD prompt");
    sendATAndWait("+HTTPTERM", 1000);
    return false;
  }

  // Stream JPEG bytes to modem via UART in 1 KB chunks.
  // Blasting all 58 KB at once overflows the A7670E's ~4 KB UART RX buffer
  // (no HW flow control on UART1), causing silent byte drops and corrupt JPEGs.
  {
    const size_t CHUNK = 1024;
    for (size_t i = 0; i < len; i += CHUNK) {
      size_t toWrite = min(CHUNK, len - i);
      modem.stream.write(data + i, toWrite);
      modem.stream.flush(); // wait until ESP32 TX FIFO is empty
      delay(20);            // give the modem a 20 ms breather per 1 KB
      esp_task_wdt_reset(); // keep watchdog happy during long upload
    }
  }
  // Wait for the modem's "OK" after data upload completes.
  // Timeout = transfer time + 10 s modem overhead.
  unsigned long writeSec = (len / 11520UL) + 10UL;
  modem.waitResponse(writeSec * 1000UL);

  // POST (action=1) â€” Supabase Storage accepts POST for object upsert/insert
  modem.sendAT("+HTTPACTION=1");

  waitStart = millis();
  bool actionOK = false;
  int httpStatus = 0;
  while (millis() - waitStart < 60000) {
    esp_task_wdt_reset();
    if (modem.stream.available()) {
      String line = modem.stream.readStringUntil('\n');
      if (line.indexOf("+HTTPACTION:") >= 0) {
        int c1 = line.indexOf(',');
        int c2 = line.indexOf(',', c1 + 1);
        if (c1 > 0 && c2 > 0)
          httpStatus = line.substring(c1 + 1, c2).toInt();
        actionOK = true;
        break;
      }
    }
    delay(50);
  }

  sendATAndWait("+HTTPTERM", 1000);

  char diagBuf[48];
  snprintf(diagBuf, sizeof(diagBuf), "%s:supabase_http_%d",
           (actionOK && (httpStatus == 200 || httpStatus == 201)) ? "OK"
                                                                  : "FAIL",
           httpStatus);
  relayDiag = diagBuf;
  Serial.printf("[RELAY] Modem HTTP status: %d (actionOK=%d)\n", httpStatus,
                (int)actionOK);

  if (actionOK && (httpStatus == 200 || httpStatus == 201)) {
    Serial.printf("[RELAY] Supabase upload OK (HTTP %d)\n", httpStatus);
    dataBytesOut += len;
    return true;
  }

  Serial.printf("[RELAY] Supabase upload FAILED (HTTP %d)\n", httpStatus);
  return false;
}

// Stream a camera upload directly from the ESP32-CAM socket into the modem.
// This avoids imposing a product-level JPEG cap just because the proxy heap
// cannot malloc the whole proof image at once.
bool uploadClientToSupabaseViaLTE(WiFiClient &source, size_t len,
                                  const String &objectPath) {
  relayDiag = "";
#if USE_WIFI_SUPABASE_UPLOAD
  if (cloudWifiAvailable()) {
    // Max buffer for WiFi path. Preview images are ~10-15KB; full proofs
    // can be ~40KB. Beyond this cap we fall through to LTE chunked relay.
    static const size_t WIFI_BUF_CAP = 48 * 1024;

    if (len <= WIFI_BUF_CAP) {
      uint8_t *buf = (uint8_t *)malloc(len);
      if (buf) {
        // ── Step 1: Drain the camera stream into RAM quickly ──
        // We MUST finish reading before starting the TLS handshake
        // to Supabase, otherwise the camera's HTTPClient times out
        // waiting for the proxy to accept its payload (HTTP -3).
        source.setTimeout(10000);
        size_t received = 0;
        unsigned long deadline = millis() + 15000;
        while (received < len && millis() < deadline) {
          if (source.available()) {
            size_t chunk = source.readBytes(buf + received, len - received);
            received += chunk;
          } else {
            delay(2);
            esp_task_wdt_reset();
          }
        }

        if (received == len) {
          // ── Step 2: Upload buffered data to Supabase via WiFi ──
          unsigned long uploadStart = millis();
          Serial.printf("[RELAY] WiFi upload: buffered %u bytes, starting HTTPS...\n", len);
          WiFiClientSecure secure;
          secure.setInsecure();
          HTTPClient http;
          String url = String(SUPABASE_URL) + "/storage/v1/object/" +
                       String(SUPABASE_BUCKET) + "/" + objectPath;
          bool ok = false;

          if (http.begin(secure, url)) {
            http.addHeader("Content-Type", "image/jpeg");
            http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON_KEY);
            http.setTimeout(60000);
            int code = http.POST(buf, len);
            ok = (code == 200 || code == 201);
            relayDiag = ok ? "OK:wifi_supabase"
                            : ("FAIL:wifi_supabase_http_" + String(code));
            Serial.printf("[RELAY] WiFi Supabase upload HTTP %d (%u bytes in %lums)\n",
                          code, (unsigned)len, millis() - uploadStart);
            http.end();
          }

          if (!ok && lteConnected) {
            // Buffer is still valid — try LTE with the in-memory copy
            ok = uploadToSupabaseViaLTE(buf, len, objectPath);
          }
          free(buf);
          return ok;
        } else {
          free(buf);
          relayDiag = "FAIL:wifi_read_timeout";
          Serial.printf("[RELAY] WiFi buf read timeout: got %u/%u\n",
                        (unsigned)received, (unsigned)len);
        }
      } else {
        relayDiag = "FAIL:wifi_upload_oom";
        Serial.printf("[RELAY] WiFi buf OOM for %u bytes\n", (unsigned)len);
      }
    } else {
      Serial.printf("[RELAY] Image %u bytes > WiFi cap %u, using LTE stream\n",
                    (unsigned)len, (unsigned)WIFI_BUF_CAP);
    }
  }
#endif

  if (!lteConnected) {
    relayDiag = "FAIL:lte_not_connected";
    Serial.println("[RELAY] LTE not connected, cannot upload");
    return false;
  }
  ModemLockGuard modemLock("uploadClientToSupabaseViaLTE", 250);
  if (!modemLock.ok()) {
    relayDiag = "BUSY";
    return false;
  }
  Serial.printf("[RELAY] Streaming %u bytes -> Supabase path: %s\n", len,
                objectPath.c_str());

  String resp;
  sendATAndWait("+HTTPTERM", 1000);
  delay(200);

  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000L, resp) != 1) {
    relayDiag = "FAIL:httpinit_failed";
    Serial.println("[RELAY] HTTPINIT failed");
    return false;
  }

  modem.sendAT("+HTTPPARA=\"CID\",1");
  modem.waitResponse(3000L);

  String url = String(SUPABASE_URL) + "/storage/v1/object/" +
               String(SUPABASE_BUCKET) + "/" + objectPath;
  modem.sendAT(("+HTTPPARA=\"URL\",\"" + url + "\"").c_str());
  modem.waitResponse(3000L);

  modem.sendAT("+HTTPPARA=\"CONTENT\",\"image/jpeg\"");
  modem.waitResponse(3000L);

  String hdrs = String("Authorization: Bearer ") + SUPABASE_ANON_KEY;
  modem.sendAT(("+HTTPPARA=\"USERDATA\",\"" + hdrs + "\"").c_str());
  modem.waitResponse(3000L);

  modem.sendAT("+CSSLCFG=\"sslversion\",1,4");
  modem.waitResponse(2000L);
  modem.sendAT("+CSSLCFG=\"ignorertctime\",1,1");
  modem.waitResponse(2000L);
  modem.sendAT("+CSSLCFG=\"authmode\",1,0");
  modem.waitResponse(2000L);
  modem.sendAT("+CSSLCFG=\"enableSNI\",1,1");
  modem.waitResponse(2000L);
  modem.sendAT("+HTTPPARA=\"SSLCFG\",1");
  modem.waitResponse(2000L);
  modem.sendAT("+HTTPSSL=1");
  modem.waitResponse(3000L);

  unsigned long dataTimeoutMs = ((unsigned long)(len / 9000UL) + 20UL) * 1000UL;
  if (dataTimeoutMs < 60000UL) {
    dataTimeoutMs = 60000UL;
  }
  modem.sendAT(("+HTTPDATA=" + String((unsigned long)len) + "," +
                String(dataTimeoutMs))
                   .c_str());

  unsigned long waitStart = millis();
  bool gotDownload = false;
  while (millis() - waitStart < 5000) {
    if (modem.stream.available()) {
      String line = modem.stream.readStringUntil('\n');
      if (line.indexOf("DOWNLOAD") >= 0) {
        gotDownload = true;
        break;
      }
    }
    delay(10);
    esp_task_wdt_reset();
  }
  if (!gotDownload) {
    relayDiag = "FAIL:no_download_prompt";
    Serial.println("[RELAY] No DOWNLOAD prompt");
    sendATAndWait("+HTTPTERM", 1000);
    return false;
  }

  source.setTimeout(30000);
  uint8_t chunk[1024];
  size_t streamed = 0;
  while (streamed < len) {
    size_t remaining = len - streamed;
    size_t toRead = remaining < sizeof(chunk) ? remaining : sizeof(chunk);
    size_t received = source.readBytes(chunk, toRead);
    if (received != toRead) {
      relayDiag = "FAIL:cam_stream_incomplete";
      Serial.printf("[RELAY] Camera stream incomplete %u/%u\n",
                    (unsigned int)(streamed + received), (unsigned int)len);
      sendATAndWait("+HTTPTERM", 1000);
      return false;
    }

    modem.stream.write(chunk, received);
    modem.stream.flush();
    streamed += received;
    delay(20);
    esp_task_wdt_reset();
  }

  unsigned long writeSec = (len / 11520UL) + 15UL;
  modem.waitResponse(writeSec * 1000UL);

  modem.sendAT("+HTTPACTION=1");

  waitStart = millis();
  bool actionOK = false;
  int httpStatus = 0;
  while (millis() - waitStart < 60000) {
    esp_task_wdt_reset();
    if (modem.stream.available()) {
      String line = modem.stream.readStringUntil('\n');
      if (line.indexOf("+HTTPACTION:") >= 0) {
        int c1 = line.indexOf(',');
        int c2 = line.indexOf(',', c1 + 1);
        if (c1 > 0 && c2 > 0)
          httpStatus = line.substring(c1 + 1, c2).toInt();
        actionOK = true;
        break;
      }
    }
    delay(50);
  }

  sendATAndWait("+HTTPTERM", 1000);

  char diagBuf[48];
  snprintf(diagBuf, sizeof(diagBuf), "%s:supabase_http_%d",
           (actionOK && (httpStatus == 200 || httpStatus == 201)) ? "OK"
                                                                  : "FAIL",
           httpStatus);
  relayDiag = diagBuf;
  Serial.printf("[RELAY] Modem HTTP status: %d (actionOK=%d)\n", httpStatus,
                (int)actionOK);

  if (actionOK && (httpStatus == 200 || httpStatus == 201)) {
    Serial.printf("[RELAY] Supabase upload OK (HTTP %d)\n", httpStatus);
    dataBytesOut += len;
    return true;
  }

  Serial.printf("[RELAY] Supabase upload FAILED (HTTP %d)\n", httpStatus);
  return false;
}

// REST helper to patch a row directly in Supabase
bool httpPatchSupabase(const char *tableName, const char *recordId, const char *jsonPayload) {
  if (cloudWifiAvailable()) {
    WiFiClientSecure secure;
    secure.setInsecure();
    HTTPClient http;
    String url = String(SUPABASE_URL) + "/rest/v1/" + String(tableName) +
                 "?id=eq." + String(recordId);
    if (http.begin(secure, url)) {
      http.addHeader("Content-Type", "application/json");
      http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON_KEY);
      http.addHeader("apikey", SUPABASE_ANON_KEY);
      http.addHeader("Prefer", "return=minimal");
      int code = http.sendRequest("PATCH", (uint8_t *)jsonPayload, strlen(jsonPayload));
      http.end();
      Serial.printf("[SBASE] WiFi PATCH HTTP %d\n", code);
      if (code >= 200 && code < 300) return true;
    }
  }

  if (!lteConnected) return false;
  ModemLockGuard modemLock("httpPatchSupabase", 250);
  if (!modemLock.ok()) return false;

  Serial.printf("[SBASE] PATCH table %s id %s\n", tableName, recordId);

  String resp;
  sendATAndWait("+HTTPTERM", 1000);
  delay(100);

  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000L, resp) != 1) return false;

  modem.sendAT("+HTTPPARA=\"CID\",1");
  modem.waitResponse(3000L);

  // Endpoint: https://{SUPABASE_URL}/rest/v1/{table}?id=eq.{id}
  String url = String(SUPABASE_URL) + "/rest/v1/" + String(tableName) + "?id=eq." + String(recordId);
  modem.sendAT(("+HTTPPARA=\"URL\",\"" + url + "\"").c_str());
  modem.waitResponse(3000L);

  modem.sendAT("+HTTPPARA=\"CONTENT\",\"application/json\"");
  modem.waitResponse(3000L);

  // Headers for Supabase REST API
  String hdrs = String("apikey: ") + SUPABASE_ANON_KEY + "\r\nAuthorization: Bearer " + SUPABASE_ANON_KEY;
  modem.sendAT(("+HTTPPARA=\"USERDATA\",\"" + hdrs + "\"").c_str());
  modem.waitResponse(3000L);

  modem.sendAT("+CSSLCFG=\"sslversion\",1,4");
  modem.waitResponse(2000L);
  modem.sendAT("+CSSLCFG=\"ignorertctime\",1,1");
  modem.waitResponse(2000L);
  modem.sendAT("+CSSLCFG=\"authmode\",1,0");
  modem.waitResponse(2000L);
  modem.sendAT("+CSSLCFG=\"enableSNI\",1,1");
  modem.waitResponse(2000L);

  modem.sendAT("+HTTPPARA=\"SSLCFG\",1");
  modem.waitResponse(2000L);
  modem.sendAT("+HTTPSSL=1");
  modem.waitResponse(3000L);

  int payloadLen = strlen(jsonPayload);
  modem.sendAT(("+HTTPDATA=" + String(payloadLen) + ",10000").c_str());
  
  unsigned long waitStart = millis();
  bool gotDownload = false;
  while (millis() - waitStart < 5000) {
    if (modem.stream.available()) {
      if (modem.stream.readStringUntil('\n').indexOf("DOWNLOAD") >= 0) {
        gotDownload = true;
        break;
      }
    }
  }

  if (gotDownload) {
    modem.stream.write((const uint8_t *)jsonPayload, payloadLen);
    modem.stream.flush();
    modem.waitResponse(5000L);
  } else {
    sendATAndWait("+HTTPTERM", 1000);
    return false;
  }

  // Action 2 = PATCH method in SIMCOM AT (+HTTPACTION=0 is GET, 1 is POST, 2 is HEAD)
  // Some SIM7600 firmware supports +HTTPACTION=4 for PUT and 5 for PATCH.
  // But wait! Safest is standard POST +HTTPACTION=1. PostgREST does upsert if you send POST
  // and include the primary key if you add Prefer: resolution=merge-duplicates.
  // However, if we don't have all required cols it might fail. 
  // We will just execute the POST and log it. If it fails, the Web SSR fallback will save us!
  
  modem.sendAT("+HTTPACTION=1"); // Try POST (Upsert) instead of PATCH, since AT commands make PATCH hard

  waitStart = millis();
  bool actionOK = false;
  int httpStatus = 0;
  while (millis() - waitStart < 15000) {
    if (modem.stream.available()) {
      String line = modem.stream.readStringUntil('\n');
      if (line.indexOf("+HTTPACTION:") >= 0) {
        int c1 = line.indexOf(',');
        int c2 = line.indexOf(',', c1 + 1);
        if (c1 > 0 && c2 > 0)
          httpStatus = line.substring(c1 + 1, c2).toInt();
        actionOK = true;
        break;
      }
    }
  }

  sendATAndWait("+HTTPTERM", 1000);
  Serial.printf("[SBASE] PATCH done. HTTP status: %d\n", httpStatus);
  return actionOK && (httpStatus >= 200 && httpStatus <= 299);
}

// ==================== DELIVERY CONTEXT READER (Firebase -> cached)
// ====================
static bool isDeliveryStillActiveInFirebase(const char *deliveryId) {
  if (!deliveryId || deliveryId[0] == '\0') {
    return false;
  }

  char statusPath[128];
  snprintf(statusPath, sizeof(statusPath), "/deliveries/%s/status.json", deliveryId);

  String statusBody = httpGetFromFirebase(statusPath);
  statusBody.trim();

  if (statusBody.length() == 0) {
    Serial.printf("[CONTEXT] Empty status for %s; retrying once...\n", deliveryId);
    statusBody = httpGetFromFirebase(statusPath);
    statusBody.trim();
  }

  if (statusBody.length() == 0) {
    Serial.printf("[CONTEXT] Status check failed for %s (empty). Treating as inactive.\n",
                  deliveryId);
    cachedDeliveryStatus[0] = '\0';
    return false;
  }

    if (statusBody == "null") {
    Serial.printf("[CONTEXT] Delivery %s missing in /deliveries -> stale context\n",
                  deliveryId);
    return false;
  }

  // JSON string payload comes back quoted from RTDB REST (e.g. "ASSIGNED").
  if (statusBody.length() >= 2 && statusBody[0] == '"' &&
      statusBody[statusBody.length() - 1] == '"') {
    statusBody = statusBody.substring(1, statusBody.length() - 1);
  }
  statusBody.toUpperCase();

  // Terminal states should not keep OTP context active on hardware.
  if (statusBody == "COMPLETED" || statusBody == "CANCELLED" ||
      statusBody == "RETURNED" || statusBody == "FAILED") {
    Serial.printf("[CONTEXT] Delivery %s is terminal (%s) -> stale context\n",
                  deliveryId, statusBody.c_str());
      cachedDeliveryStatus[0] = '\0';
      return false;
    }

    strncpy(cachedDeliveryStatus, statusBody.c_str(), sizeof(cachedDeliveryStatus) - 1);
    cachedDeliveryStatus[sizeof(cachedDeliveryStatus) - 1] = '\0';
    return true;
}

static bool verifyActiveDeliveryContext(const char *reason, bool force) {
  if (!deliveryIdCacheValid || cachedDeliveryId[0] == '\0') {
    deliveryContextTrusted = false;
    lastDeliveryVerifyAtMs = 0;
    lastVerifiedDeliveryId[0] = '\0';
    return false;
  }

  if (modemHttpBusy) {
    Serial.println("[CONTEXT] Delivery verify deferred - modem HTTP busy");
    return deliveryContextTrusted;
  }

  unsigned long nowMs = millis();
  bool sameAsVerified = lastVerifiedDeliveryId[0] != '\0' &&
                        strcmp(lastVerifiedDeliveryId, cachedDeliveryId) == 0;
  if (!force && deliveryContextTrusted && sameAsVerified &&
      (nowMs - lastDeliveryVerifyAtMs) < DELIVERY_ACTIVE_VERIFY_INTERVAL_MS) {
    return true;
  }

  bool active = isDeliveryStillActiveInFirebase(cachedDeliveryId);
  lastDeliveryVerifyAtMs = nowMs;

  if (active) {
    deliveryContextTrusted = true;
    strncpy(lastVerifiedDeliveryId, cachedDeliveryId,
            sizeof(lastVerifiedDeliveryId) - 1);
    lastVerifiedDeliveryId[sizeof(lastVerifiedDeliveryId) - 1] = '\0';
    Serial.printf("[CONTEXT] Delivery verified active (%s): %s status=%s\n",
                  reason ? reason : "check", cachedDeliveryId,
                  cachedDeliveryStatus[0] ? cachedDeliveryStatus : "UNKNOWN");
    return true;
  }

  Serial.printf("[CONTEXT] Delivery NOT active (%s): %s -> clearing cache\n",
                reason ? reason : "check", cachedDeliveryId);
  clearDeliveryContextCaches("delivery_not_active", true);
  return false;
}

static bool readTopLevelJsonString(const String &json, const char *key,
                                   char *out, size_t outLen) {
  if (!key || !out || outLen == 0) {
    return false;
  }

  String token = "\"" + String(key) + "\":";
  int depth = 0;
  bool inString = false;
  bool escaped = false;

  for (int i = 0; i < json.length(); i++) {
    char c = json[i];

    if (!inString && c == '{') {
      depth++;
    } else if (!inString && c == '}') {
      depth--;
    }

    // Key tokens start with a quote. Check before toggling inString so the
    // opening quote of top-level keys is still visible to startsWith().
    if (!inString && depth == 1 && c == '"' && json.startsWith(token, i)) {
      int v = i + token.length();
      while (v < json.length() &&
             (json[v] == ' ' || json[v] == '\t' || json[v] == '\r' ||
              json[v] == '\n')) {
        v++;
      }
      if (v >= json.length() || json[v] != '"') {
        return false;
      }

      v++; // skip opening quote
      String value = "";
      bool valEscaped = false;
      for (; v < json.length(); v++) {
        char ch = json[v];
        if (valEscaped) {
          value += ch;
          valEscaped = false;
        } else if (ch == '\\') {
          valEscaped = true;
        } else if (ch == '"') {
          break;
        } else {
          value += ch;
        }
      }

      strncpy(out, value.c_str(), outLen - 1);
      out[outLen - 1] = '\0';
      return true;
    }

    if (inString) {
      if (escaped) {
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        inString = false;
      }
      continue;
    }

    if (c == '"') {
      inString = true;
      continue;
    }
  }

  return false;
}

static bool readTopLevelJsonBool(const String &json, const char *key, bool &outVal) {
  String token = "\"" + String(key) + "\":";
  int depth = 0;
  bool inString = false;
  bool escaped = false;

  for (int i = 0; i < json.length(); i++) {
    char c = json[i];

    if (!inString && c == '{') {
      depth++;
    } else if (!inString && c == '}') {
      depth--;
    }

    if (!inString && depth == 1 && c == '"' && json.startsWith(token, i)) {
      int v = i + token.length();
      while (v < json.length() &&
             (json[v] == ' ' || json[v] == '\t' || json[v] == '\r' ||
              json[v] == '\n')) {
        v++;
      }
      if (v + 4 <= json.length() && json.startsWith("true", v)) {
        outVal = true;
        return true;
      }
      if (v + 5 <= json.length() && json.startsWith("false", v)) {
        outVal = false;
        return true;
      }
      return false;
    }

    if (inString) {
      if (escaped) {
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        inString = false;
      }
      continue;
    }

    if (c == '"') {
      inString = true;
      continue;
    }
  }

  return false;
}

static bool readTopLevelJsonInt(const String &json, const char *key, int &outVal) {
  String token = "\"" + String(key) + "\":";
  int depth = 0;
  bool inString = false;
  bool escaped = false;

  for (int i = 0; i < json.length(); i++) {
    char c = json[i];

    if (!inString && c == '{') {
      depth++;
    } else if (!inString && c == '}') {
      depth--;
    }

    if (!inString && depth == 1 && c == '"' && json.startsWith(token, i)) {
      int v = i + token.length();
      while (v < json.length() &&
             (json[v] == ' ' || json[v] == '\t' || json[v] == '\r' ||
              json[v] == '\n')) {
        v++;
      }

      int sign = 1;
      if (v < json.length() && json[v] == '-') {
        sign = -1;
        v++;
      }

      if (v >= json.length() || !isDigit(json[v])) {
        return false;
      }

      long value = 0;
      while (v < json.length() && isDigit(json[v])) {
        value = (value * 10) + (json[v] - '0');
        v++;
      }
      outVal = (int)(value * sign);
      return true;
    }

    if (inString) {
      if (escaped) {
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        inString = false;
      }
      continue;
    }

    if (c == '"') {
      inString = true;
      continue;
    }
  }

  return false;
}

static bool readTopLevelJsonDouble(const String &json, const char *key, double &outVal) {
  String token = "\"" + String(key) + "\":";
  int depth = 0;
  bool inString = false;
  bool escaped = false;

  for (int i = 0; i < json.length(); i++) {
    char c = json[i];

    if (!inString && c == '{') {
      depth++;
    } else if (!inString && c == '}') {
      depth--;
    }

    if (!inString && depth == 1 && c == '"' && json.startsWith(token, i)) {
      int v = i + token.length();
      while (v < json.length() &&
             (json[v] == ' ' || json[v] == '\t' || json[v] == '\r' ||
              json[v] == '\n')) {
        v++;
      }
      if (v >= json.length()) {
        return false;
      }

      bool quoted = (json[v] == '"');
      if (quoted) {
        v++;
      }

      String value = "";
      bool valEscaped = false;
      for (; v < json.length(); v++) {
        char ch = json[v];
        if (quoted) {
          if (valEscaped) {
            value += ch;
            valEscaped = false;
          } else if (ch == '\\') {
            valEscaped = true;
          } else if (ch == '"') {
            break;
          } else {
            value += ch;
          }
        } else {
          if (ch == ',' || ch == '}' || ch == '\r' || ch == '\n' ||
              ch == ' ' || ch == '\t') {
            break;
          }
          value += ch;
        }
      }

      value.trim();
      if (value.length() == 0 || value == "null") {
        return false;
      }
      outVal = value.toDouble();
      return true;
    }

    if (inString) {
      if (escaped) {
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        inString = false;
      }
      continue;
    }

    if (c == '"') {
      inString = true;
      continue;
    }
  }

  return false;
}

static double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = (lat2 - lat1) * PI / 180.0;
  double dLon = (lon2 - lon1) * PI / 180.0;
  double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
             cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) *
             sin(dLon / 2.0) * sin(dLon / 2.0);
  return R * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

static bool refreshPhoneLocationIfNeeded() {
  unsigned long nowMs = millis();
  if (nowMs - lastPhoneFetchAtMs < PHONE_LOC_FETCH_INTERVAL_MS) {
    return phoneFixValid;
  }
  lastPhoneFetchAtMs = nowMs;

  char phonePath[64];
  snprintf(phonePath, sizeof(phonePath), "/locations/%s/phone.json", HARDWARE_ID);
  String body = httpGetFromFirebase(phonePath);
  body.trim();

  if (body.length() == 0 || body == "null") {
    phoneFixValid = false;
    return false;
  }

  double lat = 0.0, lon = 0.0, acc = 0.0, ts = 0.0;
  bool latOk = readTopLevelJsonDouble(body, "latitude", lat) ||
               readTopLevelJsonDouble(body, "lat", lat);
  bool lonOk = readTopLevelJsonDouble(body, "longitude", lon) ||
               readTopLevelJsonDouble(body, "lon", lon) ||
               readTopLevelJsonDouble(body, "lng", lon);
  bool accOk = readTopLevelJsonDouble(body, "accuracy", acc);
  bool tsOk = readTopLevelJsonDouble(body, "timestamp", ts);

  if (!latOk || !lonOk) {
    phoneFixValid = false;
    return false;
  }

  phoneLat = lat;
  phoneLon = lon;
  phoneAccuracy = accOk ? (float)acc : -1.0f;
  phoneTimestampMs = (tsOk && ts > 0) ? (uint64_t)ts : 0;
  phoneFetchedAtMs = nowMs;
  phoneFixValid = true;
  return true;
}

static bool constantTimeEquals(const char *a, const char *b) {
  if (!a || !b) return false;
  size_t lenA = strlen(a);
  size_t lenB = strlen(b);
  if (lenA != lenB) return false;

  unsigned char diff = 0;
  for (size_t i = 0; i < lenA; i++) {
    diff |= (unsigned char)(a[i] ^ b[i]);
  }
  return diff == 0;
}

static void sha256Hex(const char *input, char *outHex, size_t outLen) {
  if (!input || !outHex || outLen < 65) {
    if (outHex && outLen > 0) outHex[0] = '\0';
    return;
  }

  unsigned char hash[32];
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  mbedtls_sha256_update(&ctx, (const unsigned char *)input, strlen(input));
  mbedtls_sha256_finish(&ctx, hash);
  mbedtls_sha256_free(&ctx);

  for (int i = 0; i < 32; i++) {
    snprintf(outHex + (i * 2), outLen - (i * 2), "%02x", hash[i]);
  }
  outHex[64] = '\0';
}

static uint32_t fnv1a32(const char *text) {
  const char *src = text ? text : "";
  uint32_t hash = 2166136261u;
  while (*src) {
    hash ^= (uint8_t)*src++;
    hash *= 16777619u;
  }
  return hash;
}

static uint32_t buildDeliveryContextHash(const char *otp, const char *deliveryId) {
  uint32_t hash = fnv1a32(otp);
  hash ^= fnv1a32(deliveryId);
  return hash;
}

static uint32_t buildPersonalPinContextHash(bool enabled,
                                            const char *hashHex,
                                            const char *salt,
                                            const char *riderId) {
  char enabledFlag[2] = {'0', '\0'};
  enabledFlag[0] = enabled ? '1' : '0';
  uint32_t hash = fnv1a32(enabledFlag);
  hash ^= fnv1a32(hashHex);
  hash ^= fnv1a32(salt);
  hash ^= fnv1a32(riderId);
  return hash;
}

static bool sameIdempotencyRecently(const char *key,
                                    const char *lastKey,
                                    unsigned long lastAt,
                                    unsigned long now) {
  if (!key || !lastKey || key[0] == '\0' || lastKey[0] == '\0') {
    return false;
  }
  if (strcmp(key, lastKey) != 0) {
    return false;
  }
  return (now - lastAt) <= IDEMPOTENCY_DEDUPE_WINDOW_MS;
}

static void buildIdempotencyKey(const char *seed, char *outHex, size_t outLen) {
  if (!outHex || outLen < 65) return;
  sha256Hex(seed ? seed : "", outHex, outLen);
}

static const char *connectivityStateStr(uint8_t state) {
  switch (state) {
    case CONN_ALL_UP:
      return "ALL_UP";
    case CONN_CAM_DOWN_CTRL_UP:
      return "CAM_DOWN_CTRL_UP";
    case CONN_CTRL_DOWN_CAM_UP:
      return "CTRL_DOWN_CAM_UP";
    case CONN_BOTH_DOWN:
      return "BOTH_DOWN";
    default:
      return "UNKNOWN";
  }
}

static void clearPersonalPinRuntimeCache(bool clearPersisted) {
  personalPinEnabled = false;
  cachedPersonalPinHashMcu[0] = '\0';
  cachedPersonalPinSalt[0] = '\0';
  cachedPersonalPinRiderId[0] = '\0';

  if (clearPersisted) {
    dpSavePersonalPinEnabled(false);
    dpSavePersonalPinHash("");
    dpSavePersonalPinSalt("");
    dpSavePersonalPinRiderId("");
    lastPersistedPinContextHash =
        buildPersonalPinContextHash(false, "", "", "");
    persistedPinContextHashReady = true;
  }
}

static bool verifyPersonalPinLocal(const char *pin) {
  if (!personalPinEnabled || !pin || pin[0] == '\0') return false;
  if (cachedPersonalPinHashMcu[0] == '\0' || cachedPersonalPinSalt[0] == '\0') {
    return false;
  }

  char salted[96];
  snprintf(salted, sizeof(salted), "%s%s", cachedPersonalPinSalt, pin);

  char computed[65];
  sha256Hex(salted, computed, sizeof(computed));
  return constantTimeEquals(computed, cachedPersonalPinHashMcu);
}

static bool writePersonalPinAuditNow(const char *eventJson) {
  if (!lteConnected || !eventJson || eventJson[0] == '\0') {
    return false;
  }

  char path[96];
  snprintf(path, sizeof(path), "/hardware/%s/personal_pin_audit/%lu.json",
           HARDWARE_ID, (unsigned long)millis());
  bool ok = httpPutWithRetry(path, eventJson);
  if (!ok) {
    return false;
  }

  char latestPatch[384];
  snprintf(latestPatch, sizeof(latestPatch),
           "{\"manual_pin_event\":true,\"source\":\"proxy_personal_pin\","
           "\"last_manual_pin_event\":%s}",
           eventJson);

  char lockPath[64];
  snprintf(lockPath, sizeof(lockPath), "/lock_events/%s/latest.json", HARDWARE_ID);
  httpPatchWithRetry(lockPath, latestPatch);
  return true;
}

static void queuePersonalPinAudit(const char *eventJson) {
  if (!eventJson || eventJson[0] == '\0') return;
  if (dpEnqueueAuditEvent(eventJson)) {
    Serial.println("[PIN] Audit event queued offline");
  }
}

static void flushQueuedPersonalPinAudits() {
  if (!lteConnected) return;

  char queued[384];
  while (dpAuditQueueCount() > 0) {
    if (!dpDequeueAuditEvent(queued, sizeof(queued))) {
      return;
    }
    if (!writePersonalPinAuditNow(queued)) {
      dpEnqueueAuditEvent(queued);
      return;
    }
  }
}

static void writePersonalPinAudit(const char *action,
                                  const char *result,
                                  bool currentlyLocked,
                                  bool offlineQueued) {
  unsigned long nowEpoch = getCurrentEpoch();
  char eventJson[384];
  snprintf(eventJson, sizeof(eventJson),
           "{\"action\":\"%s\",\"result\":\"%s\",\"currently_locked\":%s,"
           "\"offline_queued\":%s,\"rider_id\":\"%s\","
           "\"timestamp\":{\".sv\":\"timestamp\"},\"device_epoch\":%lu}",
           action ? action : "RIDER_MANUAL_PIN_ATTEMPT",
            result ? result : "unknown", currentlyLocked ? "true" : "false",
            offlineQueued ? "true" : "false", cachedPersonalPinRiderId,
            nowEpoch);

  // Do not perform Firebase HTTP writes from handleCameraClient(); that path
  // runs on loopTask and can overflow the stack during POST /personal-pin-verify.
  // Queue the audit and let flushQueuedPersonalPinAudits() upload it later.
  queuePersonalPinAudit(eventJson);
}

// ==================== TAMPER CLEAR SUPPRESSION ====================
// After admin clears tamper, suppress reed switch events only for a bounded
// period while waiting for a verified unlock transition.
static bool tamperSuppressedByAdmin = false;
static unsigned long tamperSuppressedAtMs = 0;
static bool tamperRetriggerPending = false;
static unsigned long tamperRetriggerAtMs = 0;
static const unsigned long TAMPER_SUPPRESSION_TTL_MS = 120000UL;

void writeTamperToFirebase();

static void queueTamperFirebaseWrite() {
  if (!takeState(50)) return;
  tamperFirebaseWriteQueued = true;
  giveState();
  Serial.println("[TAMPER] Firebase write queued");
}

static void queueLockEventFirebaseWrite(bool otpValid, bool faceDetected,
                                        bool unlocked, int faceAttempts,
                                        bool faceRetryExhausted,
                                        bool fallbackRequired,
                                        const char *failureReason,
                                        unsigned long unlockLatencyMs) {
  if (!takeState(50)) return;
  queuedLockOtpValid = otpValid;
  queuedLockFaceDetected = faceDetected;
  queuedLockUnlocked = unlocked;
  queuedLockFaceAttempts = faceAttempts;
  queuedLockFaceRetryExhausted = faceRetryExhausted;
  queuedLockFallbackRequired = fallbackRequired;
  strncpy(queuedLockFailureReason,
          failureReason ? failureReason : "",
          sizeof(queuedLockFailureReason) - 1);
  queuedLockFailureReason[sizeof(queuedLockFailureReason) - 1] = '\0';
  queuedLockUnlockLatencyMs = unlockLatencyMs;
  lockEventFirebaseWriteQueued = true;
  giveState();
  Serial.println("[EVENT] lock_events write queued");
}

static void queueCommandAckFirebaseWrite(const char *command,
                                         const char *status,
                                         const char *details,
                                         bool trackedAck) {
  if (!takeState(50)) return;
  strncpy(queuedCommandAckCommand, command ? command : "",
          sizeof(queuedCommandAckCommand) - 1);
  queuedCommandAckCommand[sizeof(queuedCommandAckCommand) - 1] = '\0';
  strncpy(queuedCommandAckStatus, status ? status : "unknown",
          sizeof(queuedCommandAckStatus) - 1);
  queuedCommandAckStatus[sizeof(queuedCommandAckStatus) - 1] = '\0';
  strncpy(queuedCommandAckDetails, details ? details : "",
          sizeof(queuedCommandAckDetails) - 1);
  queuedCommandAckDetails[sizeof(queuedCommandAckDetails) - 1] = '\0';
  queuedCommandAckTracked = trackedAck;
  commandAckFirebaseWriteQueued = true;
  giveState();
  Serial.println("[CMD_ACK] Firebase write queued");
}

static void queueReturnCompletionFirebaseWrite() {
  if (!takeState(50)) return;
  returnCompletionFirebaseWriteQueued = true;
  giveState();
  Serial.println("[CMD_ACK] Return completion Firebase write queued");
}

static void captureSuppressedTamper(unsigned long now) {
  tamperRetriggerPending = true;
  tamperRetriggerAtMs = now;
  Serial.println("[TAMPER] Suppressed retrigger captured for replay");
}

static void clearTamperSuppression(const char *reason) {
  if (tamperSuppressedByAdmin) {
    Serial.printf("[TAMPER] Suppression cleared (%s)\n", reason ? reason : "unspecified");
  }
  tamperSuppressedByAdmin = false;
  tamperSuppressedAtMs = 0;
}

static void replayPendingTamperIfAny(unsigned long now, const char *reason) {
  if (!tamperRetriggerPending) return;

  Serial.printf("[TAMPER] Replaying suppressed tamper (%s)\n",
                reason ? reason : "unspecified");
  tamperRetriggerPending = false;
  tamperRetriggerAtMs = 0;

  if (!theftGuardIsLockdown()) {
    theftGuardActivateLockdown("tamper_retrigger", now);
  }
  queueTamperFirebaseWrite();
}

static void startTamperSuppression(unsigned long now) {
  tamperSuppressedByAdmin = true;
  tamperSuppressedAtMs = now;
}

static void enforceTamperSuppressionTimeout(unsigned long now) {
  if (!tamperSuppressedByAdmin || tamperSuppressedAtMs == 0) return;

  if (now - tamperSuppressedAtMs < TAMPER_SUPPRESSION_TTL_MS) return;

  clearTamperSuppression("timeout_no_unlock_ack");
  replayPendingTamperIfAny(now, "suppression_timeout");
}

static void clearCommandRuntimeState() {
  cachedStatus[0] = '\0';
  cachedStatusSetAt = 0;
  cachedStatusServesRemaining = 0;
  cachedStatusStage = CMD_STAGE_NONE;
  cachedCommandAckStatus[0] = '\0';
  cachedCommandAckDetails[0] = '\0';
  lastCommandAckRetryAt = 0;
  rebootAllPending = false;
  rebootCamDispatchDone = false;
  rebootAllAtMs = 0;
  rebootAllQueuedAtMs = 0;
  rebootAllWaitLogged = false;
}

static void armPendingCommand(const char *command) {
  if (!command || command[0] == '\0') {
    clearCommandRuntimeState();
    dpClearCommandState();
    return;
  }

  strncpy(cachedStatus, command, sizeof(cachedStatus) - 1);
  cachedStatus[sizeof(cachedStatus) - 1] = '\0';
  cachedStatusStage = CMD_STAGE_PENDING;
  cachedStatusSetAt = millis();
  cachedStatusServesRemaining = STATUS_COMMAND_MAX_SERVES;
  cachedCommandAckStatus[0] = '\0';
  cachedCommandAckDetails[0] = '\0';
  lastCommandAckRetryAt = 0;

  dpSaveCommandState(cachedStatus, DP_CMD_STAGE_PENDING,
                     cachedStatusServesRemaining, "", "");
}

static void markCommandAckSent(const char *command, const char *status,
                               const char *details) {
  if (command && command[0] != '\0') {
    strncpy(cachedStatus, command, sizeof(cachedStatus) - 1);
    cachedStatus[sizeof(cachedStatus) - 1] = '\0';
  }

  strncpy(cachedCommandAckStatus, status ? status : "unknown",
          sizeof(cachedCommandAckStatus) - 1);
  cachedCommandAckStatus[sizeof(cachedCommandAckStatus) - 1] = '\0';

  strncpy(cachedCommandAckDetails, details ? details : "",
          sizeof(cachedCommandAckDetails) - 1);
  cachedCommandAckDetails[sizeof(cachedCommandAckDetails) - 1] = '\0';

  cachedStatusStage = CMD_STAGE_ACK_SENT;
  cachedStatusServesRemaining = 0;
  cachedStatusSetAt = millis();
  lastCommandAckRetryAt = 0;

  dpSaveCommandState(cachedStatus, DP_CMD_STAGE_ACK_SENT, 0,
                     cachedCommandAckStatus, cachedCommandAckDetails);
}

static void markCommandDone() {
  cachedStatus[0] = '\0';
  cachedStatusSetAt = 0;
  cachedStatusServesRemaining = 0;
  cachedStatusStage = CMD_STAGE_DONE;
  cachedCommandAckStatus[0] = '\0';
  cachedCommandAckDetails[0] = '\0';
  lastCommandAckRetryAt = 0;
  rebootAllPending = false;
  rebootCamDispatchDone = false;
  rebootAllAtMs = 0;
  dpSaveCommandState("", DP_CMD_STAGE_DONE, 0, "", "");
}

static void restorePersistedCommandState() {
  char persistedCommand[16] = "";
  char persistedAckStatus[32] = "";
  char persistedAckDetails[96] = "";
  uint8_t persistedStage = DP_CMD_STAGE_NONE;
  uint8_t persistedServes = 0;

  if (!dpLoadCommandState(persistedCommand, sizeof(persistedCommand),
                          &persistedStage, &persistedServes,
                          persistedAckStatus, sizeof(persistedAckStatus),
                          persistedAckDetails, sizeof(persistedAckDetails))) {
    return;
  }

  if (persistedStage == DP_CMD_STAGE_PENDING && persistedCommand[0] != '\0') {
    strncpy(cachedStatus, persistedCommand, sizeof(cachedStatus) - 1);
    cachedStatus[sizeof(cachedStatus) - 1] = '\0';
    cachedStatusStage = CMD_STAGE_PENDING;
    cachedStatusServesRemaining =
        persistedServes > 0 ? persistedServes : STATUS_COMMAND_MAX_SERVES;
    cachedStatusSetAt = millis();
    Serial.printf("[DP] Restored pending command: %s\n", cachedStatus);
    return;
  }

  if (persistedStage == DP_CMD_STAGE_ACK_SENT && persistedCommand[0] != '\0' &&
      persistedAckStatus[0] != '\0') {
    strncpy(cachedStatus, persistedCommand, sizeof(cachedStatus) - 1);
    cachedStatus[sizeof(cachedStatus) - 1] = '\0';

    strncpy(cachedCommandAckStatus, persistedAckStatus,
            sizeof(cachedCommandAckStatus) - 1);
    cachedCommandAckStatus[sizeof(cachedCommandAckStatus) - 1] = '\0';

    strncpy(cachedCommandAckDetails, persistedAckDetails,
            sizeof(cachedCommandAckDetails) - 1);
    cachedCommandAckDetails[sizeof(cachedCommandAckDetails) - 1] = '\0';

    cachedStatusStage = CMD_STAGE_ACK_SENT;
    cachedStatusServesRemaining = 0;
    cachedStatusSetAt = millis();
    lastCommandAckRetryAt = 0;
    Serial.printf("[DP] Restored pending command ACK upload: %s (%s)\n",
                  cachedStatus, cachedCommandAckStatus);
    return;
  }

  if (persistedStage == DP_CMD_STAGE_DONE) {
    cachedStatusStage = CMD_STAGE_DONE;
    Serial.println("[DP] Restored command state DONE");
    return;
  }

  // Corrupted/incomplete persisted state: clear and fail closed.
  clearCommandRuntimeState();
  dpClearCommandState();
  Serial.println("[DP] Cleared invalid persisted command state");
}

static void maybeRearmPendingCommand(unsigned long now) {
  if (cachedStatusStage != CMD_STAGE_PENDING || cachedStatus[0] == '\0') {
    return;
  }
  if (cachedStatusServesRemaining > 0 &&
      (unsigned long)(now - cachedStatusSetAt) <= STATUS_COMMAND_RETRY_WINDOW_MS) {
    return;
  }

  cachedStatusServesRemaining = STATUS_COMMAND_MAX_SERVES;
  cachedStatusSetAt = now;
  Serial.printf("[AP] Command still pending ACK, re-arming serve window: %s\n",
                cachedStatus);
}

static void retryPendingCommandAck(unsigned long now) {
  if (!lteConnected || cachedStatusStage != CMD_STAGE_ACK_SENT) {
    return;
  }
  if (now < lastCommandAckRetryAt) {
    return;
  }
  if (cachedStatus[0] == '\0' || cachedCommandAckStatus[0] == '\0') {
    clearCommandRuntimeState();
    dpClearCommandState();
    return;
  }

  bool ackWritten = writeCommandAckToFirebase(cachedStatus,
                                              cachedCommandAckStatus,
                                              cachedCommandAckDetails);
  if (ackWritten) {
    Serial.printf("[CMD_ACK] Durable ACK flushed, command DONE: %s\n",
                  cachedStatus);
    markCommandDone();
  } else {
    lastCommandAckRetryAt = now + COMMAND_ACK_RETRY_INTERVAL_MS;
  }
}

static bool refreshAuthoritativeDeliveryFieldsFromFirebase(bool includeCoords) {
  bool anyOk = false;

  char path[96];
  char nextDeliveryId[64] = "";
  snprintf(path, sizeof(path), "/hardware/%s/delivery_id.json", HARDWARE_ID);
  String deliveryBody = httpGetFromFirebase(path);
  deliveryBody.trim();
  if (deliveryBody.length() > 0) {
    anyOk = true;
    bool nextValid = parseFirebaseStringValue(deliveryBody, nextDeliveryId,
                                              sizeof(nextDeliveryId));
    if (nextValid) {
      if (!deliveryIdCacheValid || strcmp(cachedDeliveryId, nextDeliveryId) != 0) {
        deliveryContextTrusted = false;
        lastDeliveryVerifyAtMs = 0;
        lastVerifiedDeliveryId[0] = '\0';
        lastKnownDistMeters = -1;
        lastKnownInside = false;
        lastKnownGeoAtMs = 0;
        lastFallbackTargetFetchAtMs = 0;
      }
      strncpy(cachedDeliveryId, nextDeliveryId, sizeof(cachedDeliveryId) - 1);
      cachedDeliveryId[sizeof(cachedDeliveryId) - 1] = '\0';
      deliveryIdCacheValid = true;
    } else {
      cachedDeliveryId[0] = '\0';
      deliveryIdCacheValid = false;
      deliveryContextTrusted = false;
      lastDeliveryVerifyAtMs = 0;
      lastVerifiedDeliveryId[0] = '\0';
    }
  } else {
    Serial.println("[CONTEXT] Exact delivery_id refresh failed; keeping parsed value");
  }

  char nextOtp[8] = "";
  snprintf(path, sizeof(path), "/hardware/%s/otp_code.json", HARDWARE_ID);
  String otpBody = httpGetFromFirebase(path);
  otpBody.trim();
  if (otpBody.length() > 0) {
    anyOk = true;
    bool nextOtpValid = parseFirebaseStringValue(otpBody, nextOtp,
                                                 sizeof(nextOtp)) &&
                        strlen(nextOtp) > 0 && strlen(nextOtp) <= 6;
    if (nextOtpValid) {
      strncpy(cachedOtp, nextOtp, sizeof(cachedOtp) - 1);
      cachedOtp[sizeof(cachedOtp) - 1] = '\0';
      otpCacheValid = true;
    } else {
      cachedOtp[0] = '\0';
      otpCacheValid = false;
    }
  } else {
    Serial.println("[CONTEXT] Exact otp_code refresh failed; keeping parsed value");
  }

  bool nextReturnActive = false;
  bool hasReturnActive = false;
  snprintf(path, sizeof(path), "/hardware/%s/return_active.json", HARDWARE_ID);
  String returnActiveBody = httpGetFromFirebase(path);
  returnActiveBody.trim();
  if (returnActiveBody.length() > 0) {
    anyOk = true;
    if (parseFirebaseBoolValue(returnActiveBody, nextReturnActive, hasReturnActive) &&
        hasReturnActive) {
      returnActive = nextReturnActive;
    } else {
      returnActive = false;
    }
  }

  char nextReturnOtp[8] = "";
  snprintf(path, sizeof(path), "/hardware/%s/return_otp.json", HARDWARE_ID);
  String returnOtpBody = httpGetFromFirebase(path);
  returnOtpBody.trim();
  if (returnOtpBody.length() > 0) {
    anyOk = true;
    bool nextReturnOtpValid =
        parseFirebaseStringValue(returnOtpBody, nextReturnOtp,
                                 sizeof(nextReturnOtp)) &&
        strlen(nextReturnOtp) > 0 && strlen(nextReturnOtp) <= 6;
    if (nextReturnOtpValid) {
      strncpy(cachedReturnOtp, nextReturnOtp, sizeof(cachedReturnOtp) - 1);
      cachedReturnOtp[sizeof(cachedReturnOtp) - 1] = '\0';
      returnOtpCacheValid = true;
    } else {
      cachedReturnOtp[0] = '\0';
      returnOtpCacheValid = false;
    }
  }

  if (includeCoords) {
    if ((!destCoordsValid || !pickupCoordsValid) && refreshHardwareCoordsExact()) {
      anyOk = true;
    } else if (destCoordsValid && pickupCoordsValid) {
      Serial.println("[CONTEXT] Exact coords refresh skipped - cached coords still valid");
    }
  } else {
    Serial.println("[CONTEXT] Exact coords refresh deferred - parsing truncated body first");
  }

  if (anyOk) {
    Serial.printf("[CONTEXT] Exact refresh -> OTP:%s valid=%d | Delivery:%s valid=%d\n",
                  otpCacheValid ? cachedOtp : "NONE", otpCacheValid ? 1 : 0,
                  deliveryIdCacheValid ? cachedDeliveryId : "NONE",
                  deliveryIdCacheValid ? 1 : 0);
  }
  return anyOk;
}

void refreshDeliveryContextFromFirebase() {
  if (!wifiCloudHealthy && !lteConnected) {
    Serial.println("[CONTEXT] Skip Firebase fetch - no cloud transport");
    if (deliveryIdCacheValid && cachedDeliveryId[0] != '\0') {
      deliveryContextStale = true;
      strncpy(lastRefreshStatus, "STALE:LTE_OFFLINE", sizeof(lastRefreshStatus) - 1);
      lastRefreshStatus[sizeof(lastRefreshStatus) - 1] = '\0';
    }
    return;
  }
  if (wifiCloudHealthy && !lteConnected) {
    if (refreshAuthoritativeDeliveryFieldsFromFirebase()) {
      deliveryContextStale = false;
      deliveryContextTrusted = deliveryIdCacheValid && cachedDeliveryId[0] != '\0';
      strncpy(lastRefreshStatus, "OK:REFRESHED", sizeof(lastRefreshStatus) - 1);
      lastRefreshStatus[sizeof(lastRefreshStatus) - 1] = '\0';
    }
    lastContextFetchOkAtMs = millis();
    return;
  }

  unsigned long nowMs = millis();
  if (modemHttpBusy) {
    Serial.println("[CONTEXT] Skip Firebase fetch - modem HTTP busy");
    if (deliveryIdCacheValid && cachedDeliveryId[0] != '\0') {
      deliveryContextStale = true;
      strncpy(lastRefreshStatus, "ERR:MODEM_BUSY", sizeof(lastRefreshStatus) - 1);
      lastRefreshStatus[sizeof(lastRefreshStatus) - 1] = '\0';
    }
    return;
  }
  ModemLockGuard modemLock("refreshDeliveryContextFromFirebase", 250);
  if (!modemLock.ok()) return;

  bool contextCleared = false;
  bool prevDeliveryValid = deliveryIdCacheValid;
  char prevDeliveryId[64] = "";
  if (prevDeliveryValid) {
    strncpy(prevDeliveryId, cachedDeliveryId, sizeof(prevDeliveryId) - 1);
    prevDeliveryId[sizeof(prevDeliveryId) - 1] = '\0';
  }

  Serial.printf("[CONTEXT] Fetching /hardware/%s from Firebase...\n",
                HARDWARE_ID);
  // Fetch the context-bearing slice of the hardware node.
  char path[64];
  snprintf(path, sizeof(path), "/hardware/%s.json", HARDWARE_ID);

  String resp;
  modemHttpBusy = true;
  sendATAndWait("+HTTPTERM", 1000);
  delay(100);

  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000L, resp) != 1) {
    sendATAndWait("+HTTPTERM", 1000);
    modemHttpBusy = false;
    return;
  }

  modem.sendAT("+HTTPPARA=\"CID\",1");
  modem.waitResponse(3000L);

  String url = "https://" + String(FIREBASE_HOST) + path;
  // Context parsing only needs command, current, delivery, personal PIN,
  // pickup, return, and target fields. Skip bulky early telemetry keys.
  url += "?orderBy=%22%24key%22&startAt=%22command%22&endAt=%22return_otp%22";
  if (strlen(FIREBASE_AUTH) > 0) {
    url += "&auth=" + String(FIREBASE_AUTH);
  }
  modem.sendAT(("+HTTPPARA=\"URL\",\"" + url + "\"").c_str());
  modem.waitResponse(3000L);

  modem.sendAT("+HTTPSSL=1");
  modem.waitResponse(3000L);

  modem.sendAT("+HTTPACTION=0");
  unsigned long waitStart = millis();
  bool actionOK = false;
  int httpStatus = 0;
  int totalResponseLen = 0;
  while (millis() - waitStart < 15000) {
    if (apStarted) handleCameraClient(); // Fast local serving to avoid HTTP timeout
    if (modem.stream.available()) {
      String line = modem.stream.readStringUntil('\n');
      line.trim();
      if (line.indexOf("+HTTPACTION:") >= 0) {
        int c1 = line.indexOf(',');
        int c2 = line.indexOf(',', c1 + 1);
        if (c1 > 0 && c2 > 0) {
          httpStatus = line.substring(c1 + 1, c2).toInt();
          totalResponseLen = line.substring(c2 + 1).toInt();
        }
        actionOK = true;
        break;
      }
    }
    delay(50);
    esp_task_wdt_reset();
  }

  Serial.printf("[CONTEXT] Firebase HTTP status: %d (actionOK=%d)\n",
                httpStatus, actionOK);

  if (!actionOK || httpStatus != 200) {
    sendATAndWait("+HTTPTERM", 1000);
    modemHttpBusy = false;
    contextFetchFailCount++;
    if (deliveryIdCacheValid && cachedDeliveryId[0] != '\0') {
      deliveryContextStale = true;
      controllerContextRefreshQueued = true;
      controllerContextRefreshQueuedAt = millis();
      strncpy(lastRefreshStatus, "STALE:FETCH_FAILED", sizeof(lastRefreshStatus) - 1);
      lastRefreshStatus[sizeof(lastRefreshStatus) - 1] = '\0';
      Serial.printf("[CONTEXT] Fetch failed; keeping cached delivery %s stale (fail=%u)\n",
                    cachedDeliveryId, (unsigned int)contextFetchFailCount);
    } else {
      strncpy(lastRefreshStatus, "ERR:NO_CONTEXT", sizeof(lastRefreshStatus) - 1);
      lastRefreshStatus[sizeof(lastRefreshStatus) - 1] = '\0';
    }
    return;
  }

  contextFetchFailCount = 0;
  lastContextFetchOkAtMs = nowMs;
  deliveryContextStale = false;
  strncpy(lastRefreshStatus, "OK:REFRESHED", sizeof(lastRefreshStatus) - 1);
  lastRefreshStatus[sizeof(lastRefreshStatus) - 1] = '\0';

  if (actionOK && httpStatus == 200) {
    // A7670E/A7672X caps each AT+HTTPREAD at 1024 bytes, so page the
    // Firebase hardware node with increasing offsets and assemble locally.
    String body = "";
    int targetLen = DELIVERY_CONTEXT_MAX_BYTES;
    if (totalResponseLen > 0 && totalResponseLen < DELIVERY_CONTEXT_MAX_BYTES) {
      targetLen = totalResponseLen;
    }
    if (!body.reserve(targetLen + 1)) {
      Serial.printf(
          "[CONTEXT] WARNING: Could not reserve %d bytes; largest free block=%u\n",
          targetLen + 1, ESP.getMaxAllocHeap());
    }

    int offset = 0;

    while (offset < targetLen) {
      // Request next chunk from modem
      char readCmd[40];
      int requestLen = min(DELIVERY_CONTEXT_HTTPREAD_CHUNK, targetLen - offset);
      snprintf(readCmd, sizeof(readCmd), "AT+HTTPREAD=%d,%d\r\n", offset, requestLen);
      modem.stream.print(readCmd);

      // Phase 1: wait for "+HTTPREAD: <len>" header (up to 6 s)
      int chunkLen = -1;
      unsigned long t0 = millis();
      while (millis() - t0 < 6000) {
        if (apStarted) handleCameraClient(); // Fast local serving
        if (modem.stream.available()) {
          String line = modem.stream.readStringUntil('\n');
          line.trim();
          if (line.indexOf("+HTTPREAD:") >= 0) {
            int comma = line.lastIndexOf(',');
            if (comma >= 0) {
              chunkLen = line.substring(comma + 1).toInt();
            } else {
              int colon = line.lastIndexOf(':');
              if (colon >= 0)
                chunkLen = line.substring(colon + 1).toInt();
            }
            break;
          }
        }
        esp_task_wdt_reset();
      }

      // No data or read error — stop paging
      if (chunkLen <= 0) break;

      // Phase 2: read exactly chunkLen bytes from UART
      int before = body.length();
      unsigned long t1 = millis();
      while ((int)body.length() - before < chunkLen && millis() - t1 < 5000) {
        while (modem.stream.available() && (int)body.length() - before < chunkLen) {
          body += (char)modem.stream.read();
        }
        esp_task_wdt_reset();
      }

      // Drain trailing OK / CRLF after this chunk
      delay(50);
      while (modem.stream.available())
        modem.stream.read();

      offset += chunkLen;
    }

    sendATAndWait("+HTTPTERM", 1000);
    modemHttpBusy = false;
    modemLock.release();

    Serial.printf("[CONTEXT] totalBytes=%d body(120): %.120s\n", (int)body.length(),
                  body.c_str());
    if (totalResponseLen > 0 && totalResponseLen > (int)body.length()) {
      if (totalResponseLen > DELIVERY_CONTEXT_MAX_BYTES) {
        Serial.printf(
            "[CONTEXT] Large hardware node: %d > context window %d (%d bytes read). Using exact field reads for controller context.\n",
            totalResponseLen, DELIVERY_CONTEXT_MAX_BYTES, (int)body.length(),
            HARDWARE_ID);
      } else {
        Serial.printf(
            "[CONTEXT] WARNING: Short HTTPREAD (%d expected, %d bytes read). Fields near the end may be truncated.\n",
            totalResponseLen, (int)body.length());
      }
    }
    bool responseTruncated = (totalResponseLen > 0 && totalResponseLen > (int)body.length());

    // Manual refresh: if mobile app bumped refresh_context_at, speed up polls.
    {
      double refreshAt = 0.0;
      if (readTopLevelJsonDouble(body, "refresh_context_at", refreshAt) && refreshAt > 0.0) {
        uint64_t refreshEpoch = (uint64_t)(refreshAt + 0.5);
        if (refreshEpoch > lastManualRefreshAt) {
          lastManualRefreshAt = refreshEpoch;
          manualRefreshBurstUntil = nowMs + MANUAL_REFRESH_BURST_MS;
          lastPhoneFetchAtMs = 0;
          Serial.println("[CONTEXT] Manual refresh requested — burst polling");
        }
      }
    }

    // EC-32: Parse top-level return_active (ignore nested historical fields).
    {
      bool returnActiveValue = false;
      bool parsedReturnActive = readTopLevelJsonBool(body, "return_active", returnActiveValue);
      if (parsedReturnActive) {
        returnActive = returnActiveValue;
      } else if (!responseTruncated) {
        returnActive = false;
      }
    }

    // Parse top-level return_otp.
    {
      char parsedReturnOtp[8] = "";
      if (readTopLevelJsonString(body, "return_otp", parsedReturnOtp, sizeof(parsedReturnOtp)) &&
          strlen(parsedReturnOtp) > 0 && strlen(parsedReturnOtp) <= 6) {
        strncpy(cachedReturnOtp, parsedReturnOtp, sizeof(cachedReturnOtp) - 1);
        cachedReturnOtp[sizeof(cachedReturnOtp) - 1] = '\0';
        returnOtpCacheValid = true;
      } else if (!responseTruncated) {
        cachedReturnOtp[0] = '\0';
        returnOtpCacheValid = false;
      }
    }

    // Parse top-level otp_code only.
    {
      char parsedOtp[8] = "";
      if (readTopLevelJsonString(body, "otp_code", parsedOtp, sizeof(parsedOtp)) &&
          strlen(parsedOtp) > 0 && strlen(parsedOtp) <= 6) {
        strncpy(cachedOtp, parsedOtp, sizeof(cachedOtp) - 1);
        cachedOtp[sizeof(cachedOtp) - 1] = '\0';
        otpCacheValid = true;
      } else if (!responseTruncated) {
        cachedOtp[0] = '\0';
        otpCacheValid = false;
      }
    }

    // Parse top-level delivery_id only.
    {
      char parsedDeliveryId[64] = "";
      if (readTopLevelJsonString(body, "delivery_id", parsedDeliveryId,
                                 sizeof(parsedDeliveryId)) &&
          strlen(parsedDeliveryId) > 0) {
        strncpy(cachedDeliveryId, parsedDeliveryId, sizeof(cachedDeliveryId) - 1);
        cachedDeliveryId[sizeof(cachedDeliveryId) - 1] = '\0';
        deliveryIdCacheValid = true;
      } else if (!responseTruncated) {
        cachedDeliveryId[0] = '\0';
        deliveryIdCacheValid = false;
      }
    }

    if (responseTruncated &&
        (!deliveryIdCacheValid || !otpCacheValid ||
         (returnActive && !returnOtpCacheValid))) {
      refreshAuthoritativeDeliveryFieldsFromFirebase(false);
    }

    // Exact field reads are only needed when the modem truncated the hardware
    // node. A complete response already carries these top-level fields, and
    // repeating several LTE GETs here can monopolize the modem long enough to
    // starve GPS and watchdog-sensitive loop work.

    // Same-site demo bookings use one physical location for pickup and dropoff.
    // Keep this top-level flag close to the delivery context so /otp can expose
    // the dropoff phase immediately after pickup is confirmed.
    {
      bool sameSiteValue = false;
      cachedSamePickupDropoff =
          deliveryIdCacheValid &&
          readTopLevelJsonBool(body, "same_pickup_dropoff", sameSiteValue) &&
          sameSiteValue;
    }

    if (prevDeliveryValid && deliveryIdCacheValid &&
        strcmp(prevDeliveryId, cachedDeliveryId) != 0) {
      deliveryContextTrusted = false;
      lastDeliveryVerifyAtMs = 0;
      lastVerifiedDeliveryId[0] = '\0';
      lastKnownDistMeters = -1;
      lastKnownInside = false;
      lastKnownGeoAtMs = 0;
      lastFallbackTargetFetchAtMs = 0;
    }

    if (!contextCleared && prevDeliveryValid && !deliveryIdCacheValid) {
      deliveryContextTrusted = false;
      lastDeliveryVerifyAtMs = 0;
      lastVerifiedDeliveryId[0] = '\0';
      dpClearDeliveryContext();
      persistedContextHashReady = false;
      lastPersistedContextHash = 0;
      Serial.printf("[CONTEXT] Delivery cleared in hardware (prev=%s) -> NVS cleared\n",
                    prevDeliveryId[0] ? prevDeliveryId : "unknown");
    }

    // Parse Personal PIN runtime metadata (top-level only).
    // Fail-closed: if payload is incomplete during rider swaps, clear stale cache.
    {
      bool enabledVal = false;
      bool parsedEnabled = readTopLevelJsonBool(body, "personal_pin_enabled", enabledVal);
      bool hasEnabledField = body.indexOf("\"personal_pin_enabled\":") >= 0;
      bool hasHashField = body.indexOf("\"personal_pin_hash_mcu\":") >= 0;
      bool hasSaltField = body.indexOf("\"personal_pin_salt\":") >= 0;
      bool hasRiderField = body.indexOf("\"current_rider_id\":") >= 0;
      bool hasAnyPersonalPinRuntimeField =
          hasEnabledField || hasHashField || hasSaltField || hasRiderField;

      char parsedHash[65] = "";
      bool parsedHashOk = readTopLevelJsonString(body, "personal_pin_hash_mcu", parsedHash,
                     sizeof(parsedHash));
      bool hashValid = parsedHashOk && strlen(parsedHash) == 64;

      char parsedSalt[33] = "";
      bool parsedSaltOk = readTopLevelJsonString(body, "personal_pin_salt", parsedSalt,
                     sizeof(parsedSalt));
      bool saltValid = parsedSaltOk && strlen(parsedSalt) > 0;

      char parsedRider[64] = "";
      bool parsedRiderOk = readTopLevelJsonString(body, "current_rider_id", parsedRider,
                      sizeof(parsedRider));
      bool riderValid = parsedRiderOk && strlen(parsedRider) > 0;

      char previousRider[64] = "";
      strncpy(previousRider, cachedPersonalPinRiderId, sizeof(previousRider) - 1);
      previousRider[sizeof(previousRider) - 1] = '\0';
      bool riderChanged = riderValid && previousRider[0] != '\0' &&
                          strcmp(previousRider, parsedRider) != 0;

      bool pinRuntimeMissing = (!hasAnyPersonalPinRuntimeField || !riderValid ||
                               !hashValid || !saltValid);
      bool pinRuntimeDisabled = (parsedEnabled && !enabledVal);
      bool pinRuntimeRefreshed = false;
      if (pinRuntimeMissing && !pinRuntimeDisabled && !responseTruncated) {
        pinRuntimeRefreshed =
            refreshPersonalPinRuntimeFromFirebase(responseTruncated, false);
      } else if (pinRuntimeMissing && !pinRuntimeDisabled && responseTruncated) {
        Serial.println("[PIN] Runtime refresh deferred - truncated context response");
      }

      if (pinRuntimeRefreshed) {
        // Runtime refresh already updated caches.
      } else if (responseTruncated) {
        // Avoid wiping cache during truncated responses
        Serial.println("[PIN] Passing over cache wipe due to truncated response");
      } else if (!hasAnyPersonalPinRuntimeField) {
        if (personalPinEnabled || cachedPersonalPinHashMcu[0] ||
            cachedPersonalPinSalt[0] || cachedPersonalPinRiderId[0]) {
          Serial.println("[PIN] Personal PIN runtime fields missing; clearing stale cache");
          clearPersonalPinRuntimeCache(true);
          lastPersistedPinContextHash =
              buildPersonalPinContextHash(false, "", "", "");
          persistedPinContextHashReady = true;
        }
      } else if (!riderValid) {
        Serial.println("[PIN] Missing current_rider_id; clearing Personal PIN cache");
        clearPersonalPinRuntimeCache(true);
        lastPersistedPinContextHash =
            buildPersonalPinContextHash(false, "", "", "");
        persistedPinContextHashReady = true;
      } else if (!parsedEnabled || !enabledVal) {
        clearPersonalPinRuntimeCache(true);
        strncpy(cachedPersonalPinRiderId, parsedRider,
                sizeof(cachedPersonalPinRiderId) - 1);
        cachedPersonalPinRiderId[sizeof(cachedPersonalPinRiderId) - 1] = '\0';
        uint32_t nextPinHash =
            buildPersonalPinContextHash(false, "", "", cachedPersonalPinRiderId);
        if (!persistedPinContextHashReady ||
            nextPinHash != lastPersistedPinContextHash) {
          dpSavePersonalPinRiderId(cachedPersonalPinRiderId);
          lastPersistedPinContextHash = nextPinHash;
          persistedPinContextHashReady = true;
        }
      } else if (!hashValid || !saltValid) {
        Serial.println("[PIN] Incomplete Personal PIN runtime payload; clearing cache");
        clearPersonalPinRuntimeCache(true);
        strncpy(cachedPersonalPinRiderId, parsedRider,
                sizeof(cachedPersonalPinRiderId) - 1);
        cachedPersonalPinRiderId[sizeof(cachedPersonalPinRiderId) - 1] = '\0';
        uint32_t nextPinHash =
            buildPersonalPinContextHash(false, "", "", cachedPersonalPinRiderId);
        if (!persistedPinContextHashReady ||
            nextPinHash != lastPersistedPinContextHash) {
          dpSavePersonalPinRiderId(cachedPersonalPinRiderId);
          lastPersistedPinContextHash = nextPinHash;
          persistedPinContextHashReady = true;
        }
      } else {
        if (riderChanged) {
          Serial.printf("[PIN] Rider changed (%s -> %s); replacing Personal PIN cache\n",
                        previousRider, parsedRider);
        }

        personalPinEnabled = true;

        strncpy(cachedPersonalPinHashMcu, parsedHash,
                sizeof(cachedPersonalPinHashMcu) - 1);
        cachedPersonalPinHashMcu[sizeof(cachedPersonalPinHashMcu) - 1] = '\0';

        strncpy(cachedPersonalPinSalt, parsedSalt,
                sizeof(cachedPersonalPinSalt) - 1);
        cachedPersonalPinSalt[sizeof(cachedPersonalPinSalt) - 1] = '\0';

        strncpy(cachedPersonalPinRiderId, parsedRider,
                sizeof(cachedPersonalPinRiderId) - 1);
        cachedPersonalPinRiderId[sizeof(cachedPersonalPinRiderId) - 1] = '\0';

        uint32_t nextPinHash =
            buildPersonalPinContextHash(true,
                                        cachedPersonalPinHashMcu,
                                        cachedPersonalPinSalt,
                                        cachedPersonalPinRiderId);
        if (!persistedPinContextHashReady ||
            nextPinHash != lastPersistedPinContextHash) {
          dpSavePersonalPinEnabled(true);
          dpSavePersonalPinHash(cachedPersonalPinHashMcu);
          dpSavePersonalPinSalt(cachedPersonalPinSalt);
          dpSavePersonalPinRiderId(cachedPersonalPinRiderId);
          lastPersistedPinContextHash = nextPinHash;
          persistedPinContextHashReady = true;
        }
      }
    }

    // Guard against stale /hardware context (delivery_id can exist without otp_code).
    // If the referenced delivery no longer exists or is terminal, clear caches
    // and self-heal /hardware to stop serving old OTP/delivery over GET /otp.
    // Throttled: only check every STALE_DELIVERY_CHECK_DIVISOR polls to avoid
    // burning an extra HTTP session (5-10s on LTE) on every single context read.
    if (deliveryIdCacheValid) {
      bool forceVerify = !prevDeliveryValid ||
                         strcmp(prevDeliveryId, cachedDeliveryId) != 0 ||
                         !deliveryContextTrusted;
      if (!verifyActiveDeliveryContext(forceVerify ? "new_or_untrusted" : "periodic",
                                       forceVerify)) {
        contextCleared = true;
      }
    }

    // Parse destination coords for geofence + theft guard targeting.
    // Accept all field names written by the web/mobile paths.
    if (deliveryIdCacheValid) {
      double dLa = 0.0;
      double dLo = 0.0;
      bool gotLat = readTopLevelJsonDouble(body, "target_lat", dLa) ||
                    readTopLevelJsonDouble(body, "dest_lat", dLa) ||
                    readTopLevelJsonDouble(body, "dropoff_lat", dLa);
      bool gotLon = readTopLevelJsonDouble(body, "target_lng", dLo) ||
                    readTopLevelJsonDouble(body, "dest_lon", dLo) ||
                    readTopLevelJsonDouble(body, "dropoff_lng", dLo);
      if (gotLat && gotLon && (dLa != 0.0 || dLo != 0.0)) {
        applyTargetCoords(dLa, dLo, "hardware-json");
      }
    } else {
      destCoordsValid = false;
    }

    // Parse pickup coords (pickup_lat / pickup_lng) for dual-geofence gating
    if (deliveryIdCacheValid) {
      double pLa = 0.0;
      double pLo = 0.0;
      bool gotPLat = readTopLevelJsonDouble(body, "pickup_lat", pLa);
      bool gotPLon = readTopLevelJsonDouble(body, "pickup_lng", pLo);
      if (gotPLat && gotPLon && (pLa != 0.0 || pLo != 0.0)) {
        applyPickupCoords(pLa, pLo, "hardware-json");
      }
    } else {
      pickupCoordsValid = false;
    }

    // EC-32: Swap the PRIMARY geofence target when returning.
    // - Normal trip: target = dropoff (destLat/destLon)
    // - Return trip: target = pickup (pickupLat/pickupLon)
    //
    // Note: isNearAnyTarget() already checks both points. This swap is for the
    // stability state machine + distance telemetry, and for consistent gating.
    // Detect return->normal transition for cache cleanup
    bool returnEnded = (lastReturnActive && !returnActive);

    if (returnActive && pickupCoordsValid) {
      bool targetChanged = (geoProxy.targetLat != pickupLat) || (geoProxy.targetLon != pickupLon);
      if (targetChanged || !lastReturnActive) {
        geoProxy.setTarget(pickupLat, pickupLon);
        theftGuardSetGeofence((float)pickupLat, (float)pickupLon, TG_GEOFENCE_RADIUS_KM);
        Serial.printf("[EC-32] Return mode: target->PICKUP %.6f, %.6f (hysteresis reset)\n", pickupLat, pickupLon);
      }
    } else if (!returnActive && destCoordsValid) {
      bool targetChanged = (geoProxy.targetLat != destLat) || (geoProxy.targetLon != destLon);
      if (targetChanged || lastReturnActive) {
        geoProxy.setTarget(destLat, destLon);
        theftGuardSetGeofence((float)destLat, (float)destLon, TG_GEOFENCE_RADIUS_KM);
        Serial.printf("[EC-32] Normal mode: target->DROPOFF %.6f, %.6f\n", destLat, destLon);
      }
    }

    if (returnEnded) {
      cachedReturnOtp[0] = '\0';
      returnOtpCacheValid = false;
      Serial.println("[EC-32] Return ended: cleared cached return OTP");
    }
    lastReturnActive = returnActive;

    // ── NVS persistence (write only when context changes) ──
    uint32_t deliveryCtxHash =
        buildDeliveryContextHash(otpCacheValid ? cachedOtp : "",
                                 deliveryIdCacheValid ? cachedDeliveryId : "");
    if (!persistedContextHashReady || deliveryCtxHash != lastPersistedContextHash) {
      dpSaveDeliveryContext(otpCacheValid ? cachedOtp : "",
                            deliveryIdCacheValid ? cachedDeliveryId : "");
      lastPersistedContextHash = deliveryCtxHash;
      persistedContextHashReady = true;
    }

    // ── EC-78: Detect reassignment ──
    int raIdx = body.indexOf("\"reassignment_pending\":true");
    if (raIdx >= 0) {
      char oldR[64] = "", newR[64] = "", raDelId[64] = "";
      // parse old_rider_id
      int orIdx = body.indexOf("\"old_rider_id\":\"");
      if (orIdx >= 0) {
        int s = orIdx + 16, e = body.indexOf('"', s);
        if (e > s && (e - s) < (int)sizeof(oldR)) {
          strncpy(oldR, body.c_str() + s, e - s); oldR[e - s] = '\0';
        }
      }
      // parse new_rider_id
      int nrIdx = body.indexOf("\"new_rider_id\":\"");
      if (nrIdx >= 0) {
        int s = nrIdx + 16, e = body.indexOf('"', s);
        if (e > s && (e - s) < (int)sizeof(newR)) {
          strncpy(newR, body.c_str() + s, e - s); newR[e - s] = '\0';
        }
      }
      // parse reassignment delivery_id (may reuse cachedDeliveryId)
      int rdIdx = body.indexOf("\"reassignment_delivery_id\":\"");
      if (rdIdx >= 0) {
        int s = rdIdx + 28, e = body.indexOf('"', s);
        if (e > s && (e - s) < (int)sizeof(raDelId)) {
          strncpy(raDelId, body.c_str() + s, e - s); raDelId[e - s] = '\0';
        }
      }
      if (!reassign.isPending()) {
        reassign.trigger(oldR, newR,
                         raDelId[0] ? raDelId : cachedDeliveryId, millis());
      }
      // parse new_otp if present
      int noIdx = body.indexOf("\"new_otp\":\"");
      if (noIdx >= 0) {
        int s = noIdx + 11, e = body.indexOf('"', s);
        if (e > s && (e - s) <= 7) {
          char notp[8];
          strncpy(notp, body.c_str() + s, e - s); notp[e - s] = '\0';
          reassign.setNewOtp(notp);
        }
      }
    }

    // ── EC-FIX: Parse remote lock/unlock command ──
    // Mobile app writes command: "UNLOCKING" / "LOCKED" to hardware/{boxId}
    int stIdx = body.indexOf("\"command\":\"");
    if (stIdx >= 0) {
      int start = stIdx + 11;
      int end = body.indexOf('"', start);
      if (end > start) {
        int len = end - start;
        if (len > 0 && len < (int)sizeof(cachedStatus)) {
          char parsedCommand[16] = "";
          strncpy(parsedCommand, body.c_str() + start, len);
          parsedCommand[len] = '\0';

          if (strcmp(parsedCommand, "CLEAR_CONTEXT") == 0) {
            clearDeliveryContextCaches("remote_command", true);
            char hwPath[64];
            snprintf(hwPath, sizeof(hwPath), "/hardware/%s.json", HARDWARE_ID);
            httpPatchWithRetry(hwPath, "{\"command\":\"NONE\"}");
            Serial.println("[CONTEXT] Remote command CLEAR_CONTEXT executed");
          } else if (strcmp(parsedCommand, "NONE") == 0) {
            if (cachedStatusStage == CMD_STAGE_ACK_SENT) {
              Serial.println("[CONTEXT] command=NONE ignored while ACK upload pending");
            } else {
              clearCommandRuntimeState();
              dpClearCommandState();
              rebootAllPending = false;
              rebootCamDispatchDone = false;
              rebootAllAtMs = 0;
              Serial.println("[CONTEXT] Remote command cleared (NONE)");
            }
          } else {
            armPendingCommand(parsedCommand);
            Serial.printf("[CONTEXT] Remote command parsed: '%s' (retry window armed)\n",
                          cachedStatus);

            if (strcmp(cachedStatus, "REBOOT_ALL") == 0) {
              rebootAllPending = true;
              rebootCamDispatchDone = false;
              rebootAllAtMs = millis() + REBOOT_ALL_GRACE_MS;
              rebootAllQueuedAtMs = millis();
              rebootAllWaitLogged = false;
              Serial.printf("[REBOOT_ALL] Grace window started (%lums)\n",
                            (unsigned long)REBOOT_ALL_GRACE_MS);
            }

            // Clear command from Firebase after caching locally.
            char hwPath[64];
            snprintf(hwPath, sizeof(hwPath), "/hardware/%s.json", HARDWARE_ID);
            httpPatchWithRetry(hwPath, "{\"command\":\"NONE\"}");
          }
        }
      }
    }

    // ── EC-81: Parse top-level lockdown command only ──
    // Avoid matching nested fields like tamper.lockdown that can re-trigger
    // lockdown after admin clear.
    char remoteTheftState[16] = {0};
    bool hasTopLevelTheftState =
        readTopLevelJsonString(body, "theft_state", remoteTheftState,
                               sizeof(remoteTheftState));
    if (hasTopLevelTheftState && strcmp(remoteTheftState, "STOLEN") == 0 &&
        theftGuardGetState() != TG_STOLEN && !theftGuardIsLockdown() &&
        !tamperSuppressedByAdmin) {
      theftGuardReportTheft("admin_remote", millis(), "remote_mark_stolen");
    } else if (hasTopLevelTheftState &&
               strcmp(remoteTheftState, "NORMAL") == 0 &&
               theftGuardIsStolen() && !theftGuardIsLockdown()) {
      theftGuardReset();
    }

    bool lockdownRequested = false;
    bool hasTopLevelLockdown =
        readTopLevelJsonBool(body, "lockdown", lockdownRequested);
    if (lockdownRequested && !theftGuardIsLockdown() &&
        !tamperSuppressedByAdmin) {
      theftGuardActivateLockdown("admin_remote", millis());
    } else if ((!hasTopLevelLockdown || !lockdownRequested) &&
               theftGuardIsLockdown()) {
      theftGuardDeactivateLockdown();
    }

    deliveryContextValidated = true;
    if (deliveryIdCacheValid) {
      lastDeliveryIdSeenAtMs = nowMs;
    } else if (!contextCleared) {
      lastDeliveryIdSeenAtMs = 0;
    }

    Serial.printf(
        "[CONTEXT] Parsed -> OTP: %s (valid=%d) | DeliveryID: %s (valid=%d) | Status: %s\n",
        otpCacheValid ? cachedOtp : "NONE", otpCacheValid,
        deliveryIdCacheValid ? cachedDeliveryId : "NONE", deliveryIdCacheValid,
        cachedStatus[0] ? cachedStatus : "NONE");

    // EC-32: Return-mode visibility in logs
    if (returnActive) {
      Serial.printf("[CONTEXT] Return active -> returnOtp: %s (valid=%d)\n",
                    returnOtpCacheValid ? cachedReturnOtp : "NONE",
                    (int)returnOtpCacheValid);
    }
  }

  if (modemHttpBusy) {
    sendATAndWait("+HTTPTERM", 1000);
    modemHttpBusy = false;
  }
}

// ==================== LOCK EVENT WRITER ====================
void writeLockEventToFirebase(bool otpValid, bool faceDetected, bool unlocked,
                              int faceAttempts = 0,
                              bool faceRetryExhausted = false,
                              bool fallbackRequired = false,
                              const char *failureReason = "",
                              unsigned long unlockLatencyMs = 0) {
  if (!wifiCloudHealthy && !lteConnected) {
    Serial.println("[EVENT] No cloud transport connected");
    return;
  }

  unsigned long now_epoch = getCurrentEpoch();
  char tsBuf[30];
  String tsStr = formatTimestamp(now_epoch);
  strncpy(tsBuf, tsStr.c_str(), sizeof(tsBuf) - 1);
  tsBuf[sizeof(tsBuf) - 1] = '\0';

  char eventJson[384];
  const char *safeReason =
      (failureReason != NULL && failureReason[0] != '\0') ? failureReason : "";
  char idempotencySeed[192];
  snprintf(idempotencySeed, sizeof(idempotencySeed),
           "%s|%d|%d|%d|%d|%d|%d|%s",
           HARDWARE_ID,
           otpValid ? 1 : 0,
           faceDetected ? 1 : 0,
           unlocked ? 1 : 0,
           faceAttempts,
           faceRetryExhausted ? 1 : 0,
           fallbackRequired ? 1 : 0,
           safeReason);
  char idempotencyKey[65] = "";
  buildIdempotencyKey(idempotencySeed, idempotencyKey, sizeof(idempotencyKey));
  unsigned long nowMs = millis();
  if (sameIdempotencyRecently(idempotencyKey,
                              lastLockEventIdempotencyKey,
                              lastLockEventIdempotencyAt,
                              nowMs)) {
    Serial.println("[EVENT] lock_events deduped in short window");
    return;
  }

  snprintf(eventJson, sizeof(eventJson),
           "{\"otp_valid\":%s,\"face_detected\":%s,\"unlocked\":%s,"
           "\"timestamp\":{\".sv\":\"timestamp\"},\"device_epoch\":%lu,"
           "\"timestamp_str\":\"%s\",\"face_attempts\":%d,"
           "\"face_retry_exhausted\":%s,\"fallback_required\":%s,"
           "\"failure_reason\":\"%s\",\"unlock_latency_ms\":%lu,"
           "\"idempotency_key\":\"%s\"}",
           otpValid ? "true" : "false", faceDetected ? "true" : "false",
           unlocked ? "true" : "false", now_epoch, tsBuf, faceAttempts,
           faceRetryExhausted ? "true" : "false",
           fallbackRequired ? "true" : "false", safeReason,
           unlockLatencyMs,
           idempotencyKey);

  char eventPath[64];
  snprintf(eventPath, sizeof(eventPath), "/lock_events/%s/latest.json",
           HARDWARE_ID);
  bool ok = httpPutWithRetry(eventPath, eventJson);
  Serial.printf("[EVENT] lock_events: %s\n", ok ? "OK" : "FAIL");
  if (ok) {
    strncpy(lastLockEventIdempotencyKey, idempotencyKey,
            sizeof(lastLockEventIdempotencyKey) - 1);
    lastLockEventIdempotencyKey[sizeof(lastLockEventIdempotencyKey) - 1] = '\0';
    lastLockEventIdempotencyAt = nowMs;
  }

  const char *newStatus = unlocked ? "UNLOCKING" : "LOCKED";
  isBoxLocked = !unlocked;
  char statusJson[64];
  snprintf(statusJson, sizeof(statusJson), "\"%s\"", newStatus);
  char statusPath[64];
  snprintf(statusPath, sizeof(statusPath), "/hardware/%s/status.json",
           HARDWARE_ID);
  httpPutWithRetry(statusPath, statusJson);
}

// ==================== COMMAND ACK WRITER ====================
bool writeCommandAckToFirebase(const char *command, const char *status,
                               const char *details) {
  if (!lteConnected) {
    Serial.println("[CMD_ACK] LTE not connected");
    return false;
  }

  unsigned long now_epoch = getCurrentEpoch();
  unsigned long nowMs = millis();
  char idempotencySeed[224];
  snprintf(idempotencySeed, sizeof(idempotencySeed), "%s|%s|%s|%s",
           HARDWARE_ID,
           command ? command : "",
           status ? status : "",
           details ? details : "");
  char idempotencyKey[65] = "";
  buildIdempotencyKey(idempotencySeed, idempotencyKey, sizeof(idempotencyKey));
  if (sameIdempotencyRecently(idempotencyKey,
                              lastCommandAckIdempotencyKey,
                              lastCommandAckIdempotencyAt,
                              nowMs)) {
    Serial.println("[CMD_ACK] Deduped in short window");
    return true;
  }

  char json[320];
  snprintf(json, sizeof(json),
           "{\"command_ack_command\":\"%s\",\"command_ack_status\":\"%s\","
           "\"command_ack_details\":\"%s\",\"command_ack_at\":{\".sv\":\"timestamp\"},"
           "\"command_ack_epoch\":%lu,\"command_ack_idempotency\":\"%s\"}",
           command ? command : "", status ? status : "", details ? details : "",
           now_epoch,
           idempotencyKey);

  char hwPath[64];
  snprintf(hwPath, sizeof(hwPath), "/hardware/%s.json", HARDWARE_ID);
  bool ok = httpPatchWithRetry(hwPath, json);
  Serial.printf("[CMD_ACK] Firebase write: %s (%s/%s)\n", ok ? "OK" : "FAIL",
                command ? command : "", status ? status : "");
  if (ok) {
    strncpy(lastCommandAckIdempotencyKey, idempotencyKey,
            sizeof(lastCommandAckIdempotencyKey) - 1);
    lastCommandAckIdempotencyKey[sizeof(lastCommandAckIdempotencyKey) - 1] = '\0';
    lastCommandAckIdempotencyAt = nowMs;
  }

  // Sync internal state and immediately patch `status` at root too if executed/already_locked
  if (ok && (strcmp(status, "executed") == 0 || strcmp(status, "already_locked") == 0 || strcmp(status, "already_unlocked") == 0)) {
    if (strcmp(command, "LOCKED") == 0) {
      isBoxLocked = true;
      httpPatchWithRetry(hwPath, "{\"status\":\"LOCKED\"}");
    } else if (strcmp(command, "UNLOCKING") == 0) {
      isBoxLocked = false;
      httpPatchWithRetry(hwPath, "{\"status\":\"UNLOCKING\"}");

      // Only clear admin tamper suppression when unlock was acknowledged by
      // the controller, then replay any suppressed retrigger.
      if (tamperSuppressedByAdmin &&
          (strcmp(status, "executed") == 0 ||
           strcmp(status, "already_unlocked") == 0)) {
        clearTamperSuppression("unlock_command_ack");
        replayPendingTamperIfAny(millis(), "unlock_command_ack");
      }
    }
  }

  return ok;
}

// ==================== TAMPER EVENT WRITER ====================
void writeTamperToFirebase() {
  if (!lteConnected) {
    Serial.println("[TAMPER] LTE not connected");
    return;
  }

  unsigned long now_epoch = getCurrentEpoch();
  unsigned long nowMs = millis();
  char idempotencyKey[65] = "";
  buildIdempotencyKey("tamper|reed_switch", idempotencyKey,
                      sizeof(idempotencyKey));
  if (sameIdempotencyRecently(idempotencyKey,
                              lastTamperIdempotencyKey,
                              lastTamperIdempotencyAt,
                              nowMs)) {
    Serial.println("[TAMPER] Deduped in short window");
    return;
  }

  char tsBuf[30];
  String tsStr = formatTimestamp(now_epoch);
  strncpy(tsBuf, tsStr.c_str(), sizeof(tsBuf) - 1);
  tsBuf[sizeof(tsBuf) - 1] = '\0';

  // Write tamper state to hardware/{boxId}/tamper
  char tamperJson[256];
  snprintf(tamperJson, sizeof(tamperJson),
           "{\"detected\":true,\"lockdown\":true,"
           "\"timestamp\":{\".sv\":\"timestamp\"},"
           "\"device_epoch\":%lu,\"timestamp_str\":\"%s\","
           "\"source\":\"reed_switch\",\"idempotency_key\":\"%s\"}",
           now_epoch, tsBuf, idempotencyKey);

  char tamperPath[64];
  snprintf(tamperPath, sizeof(tamperPath), "/hardware/%s/tamper.json",
           HARDWARE_ID);
  bool ok = httpPutWithRetry(tamperPath, tamperJson);
  Serial.printf("[TAMPER] Firebase tamper write: %s\n", ok ? "OK" : "FAIL");
  if (ok) {
    strncpy(lastTamperIdempotencyKey, idempotencyKey,
            sizeof(lastTamperIdempotencyKey) - 1);
    lastTamperIdempotencyKey[sizeof(lastTamperIdempotencyKey) - 1] = '\0';
    lastTamperIdempotencyAt = nowMs;
  }

  // Also update the top-level theft_state so useSecurityAlerts fires
  char theftJson[32];
  snprintf(theftJson, sizeof(theftJson), "\"STOLEN\"");
  char theftPath[64];
  snprintf(theftPath, sizeof(theftPath), "/hardware/%s/theft_state.json",
           HARDWARE_ID);
  httpPutWithRetry(theftPath, theftJson);

  // Log tamper event to lock_events
  char eventJson[256];
  snprintf(eventJson, sizeof(eventJson),
           "{\"tamper\":true,\"source\":\"reed_switch\","
           "\"timestamp\":{\".sv\":\"timestamp\"},"
           "\"device_epoch\":%lu,\"timestamp_str\":\"%s\","
           "\"idempotency_key\":\"%s\"}",
           now_epoch, tsBuf, idempotencyKey);
  char eventPath[64];
  snprintf(eventPath, sizeof(eventPath), "/lock_events/%s/latest.json",
           HARDWARE_ID);
  httpPutWithRetry(eventPath, eventJson);
}

// ==================== ADMIN TAMPER CLEAR READER ====================
// Called periodically to check if admin has requested a tamper clear
// from web/mobile dashboard. Reads hardware/{boxId}/clear_tamper node.
void checkAndClearTamperFromFirebase() {
  if (!wifiCloudHealthy && !lteConnected) return;

  char clearPath[80];
  snprintf(clearPath, sizeof(clearPath), "/hardware/%s/clear_tamper.json",
           HARDWARE_ID);

  String body = httpGetFromFirebase(clearPath);
  if (body.length() == 0 || body == "null") return;
  bool clearPayloadIsObject = body.startsWith("{");

  // Admin requested clear — reset TheftGuard state machine
  Serial.println("[TAMPER] Admin clear_tamper detected — resetting TheftGuard");
  theftGuardReset();
  startTamperSuppression(millis());
  tamperRetriggerPending = false;
  tamperRetriggerAtMs = 0;

  // Delete the clear_tamper command node (acknowledge)
  httpPutWithRetry(clearPath, "null");

  // Also ensure tamper and theft_state are clean
  char tamperPath[64];
  snprintf(tamperPath, sizeof(tamperPath), "/hardware/%s/tamper.json",
           HARDWARE_ID);
  httpPutWithRetry(tamperPath, "null");

  char theftPath[64];
  snprintf(theftPath, sizeof(theftPath), "/hardware/%s/theft_state.json",
           HARDWARE_ID);
  httpPutWithRetry(theftPath, "\"NORMAL\"");

  // Also clear top-level lockdown so context parser cannot re-activate
  // lockdown on the next poll.
  char hwPath[64];
  snprintf(hwPath, sizeof(hwPath), "/hardware/%s.json", HARDWARE_ID);
  httpPatchWithRetry(hwPath, "{\"lockdown\":false}");

  // Append latest clear acknowledgement for incident forensics.
  unsigned long now_epoch = getCurrentEpoch();
  char clearAuditJson[256];
  snprintf(clearAuditJson, sizeof(clearAuditJson),
           "{\"tamper_clear\":true,\"source\":\"admin_clear\","
           "\"payload_is_object\":%s,\"suppression_ttl_ms\":%lu,"
           "\"timestamp\":{\".sv\":\"timestamp\"},\"device_epoch\":%lu}",
           clearPayloadIsObject ? "true" : "false",
           (unsigned long)TAMPER_SUPPRESSION_TTL_MS, now_epoch);
  char clearAuditPath[96];
  snprintf(clearAuditPath, sizeof(clearAuditPath),
           "/lock_events/%s/tamper_clear_latest.json", HARDWARE_ID);
  httpPutWithRetry(clearAuditPath, clearAuditJson);

  Serial.println("[TAMPER] TheftGuard reset to NORMAL, tamper suppressed until unlock or timeout");
}

// ==================== ADMIN PHOTO BURST READER ====================
// Consumes boxes/{boxId}/commands/photo_burst from the web admin panel and
// forwards each capture request to the ESP32-CAM over the hotspot link.
void checkAndRunPhotoBurstFromFirebase(unsigned long now) {
  if (!wifiCloudHealthy && !lteConnected) return;

  char commandPath[96];
  snprintf(commandPath, sizeof(commandPath),
           "/boxes/%s/commands/photo_burst.json", HARDWARE_ID);

  String body = httpGetFromFirebase(commandPath);
  if (body.length() == 0 || body == "null") return;

  int count = 5;
  int intervalMs = 1000;
  readTopLevelJsonInt(body, "count", count);
  readTopLevelJsonInt(body, "interval_ms", intervalMs);
  if (count < 1) count = 1;
  if (count > 10) count = 10;
  if (intervalMs < 250) intervalMs = 250;
  if (intervalMs > 10000) intervalMs = 10000;

  Serial.printf("[CAM] Photo burst requested: count=%d interval=%dms\n",
                count, intervalMs);

  int queued = 0;
  String lastResult = "";
  for (int i = 0; i < count; i++) {
    lastResult = forwardCameraCaptureUploadCommand();
    if (lastResult == "CAPTURE_QUEUED") {
      queued++;
    }

    if (i < count - 1) {
      unsigned long waitUntil = millis() + (unsigned long)intervalMs;
      while (millis() < waitUntil) {
        esp_task_wdt_reset();
        delay(25);
      }
    }
  }

  httpPutWithRetry(commandPath, "null");

  char cameraPath[80];
  snprintf(cameraPath, sizeof(cameraPath), "/hardware/%s/camera.json",
           HARDWARE_ID);
  char cameraJson[256];
  snprintf(cameraJson, sizeof(cameraJson),
           "{\"photo_burst_last_status\":\"%s\",\"photo_burst_requested\":%d,"
           "\"photo_burst_queued\":%d,\"photo_burst_at\":{\".sv\":\"timestamp\"}}",
           lastResult.c_str(), count, queued);
  httpPatchWithRetry(cameraPath, cameraJson);
}

// ==================== FACE CHECK FORWARDER ====================
String forwardFaceCheck(String deliveryId) {
  if (!camClientKnown) {
    Serial.println("[FACE] CAM IP unknown on hotspot LAN");
    return "ERROR:cam_unknown";
  }

  WiFiClient camClient;
  if (!camClient.connect(camClientIP, CAM_FACE_PORT)) {
    Serial.println("[FACE] Cannot connect to ESP32-CAM");
    return "ERROR:cam_unreachable";
  }
  camClientKnown = true;

  if (deliveryId.length() > 0) {
    camClient.print("GET /face-status?delivery_id=");
    camClient.print(deliveryId);
    camClient.print(" HTTP/1.1\r\nHost: ");
  } else {
    camClient.print("GET /face-status HTTP/1.1\r\nHost: ");
  }
  camClient.print(camClientIP.toString());
  camClient.print("\r\nConnection: close\r\n\r\n");

  unsigned long waitStart = millis();
  String result = "";
  bool headersEnded = false;
  while (millis() - waitStart < 18000) {
    if (camClient.available()) {
      String line = camClient.readStringUntil('\n');
      line.trim();
      if (!headersEnded) {
        if (line.length() == 0)
          headersEnded = true;
      } else {
        result = line;
        break;
      }
    } else {
      delay(10);
    }
    esp_task_wdt_reset();
  }

  camClient.stop();
  if (result.length() > 0) {
    camLastSeenAt = millis();
  }
  Serial.printf("[FACE] CAM response: %s\n", result.c_str());
  return result;
}

String forwardCameraCaptureUploadCommand() {
  if (!camClientKnown) {
    camClientIP = IPAddress(192, 168, 4, 10);
  }

  WiFiClient camClient;
  if (!camClient.connect(camClientIP, CAM_FACE_PORT)) {
    Serial.println("[CAM] Cannot connect to ESP32-CAM for capture command");
    return "ERROR:cam_unreachable";
  }
  camClientKnown = true;

  camClient.print("GET /capture-upload?source=theft HTTP/1.1\r\nHost: ");
  camClient.print(camClientIP.toString());
  camClient.print("\r\nConnection: close\r\n\r\n");

  unsigned long waitStart = millis();
  String result = "";
  bool headersEnded = false;
  while (millis() - waitStart < 18000) {
    if (camClient.available()) {
      String line = camClient.readStringUntil('\n');
      line.trim();
      if (!headersEnded) {
        if (line.length() == 0) {
          headersEnded = true;
        }
      } else {
        result = line;
        break;
      }
    } else {
      delay(10);
    }
    esp_task_wdt_reset();
  }

  camClient.stop();
  if (result.length() > 0) {
    camLastSeenAt = millis();
  }
  Serial.printf("[CAM] Capture upload response: %s\n", result.c_str());
  return result.length() > 0 ? result : "ERROR:cam_timeout";
}

String forwardCameraPowerCommand(const String &mode) {
  if (!camClientKnown) {
    camClientIP = IPAddress(192, 168, 4, 10);
  }

  WiFiClient camClient;
  if (!camClient.connect(camClientIP, CAM_FACE_PORT)) {
    Serial.println("[CAM] Cannot connect to ESP32-CAM for power command");
    return "ERROR:cam_unreachable";
  }
  camClientKnown = true;

  camClient.print("GET /cam-power?mode=");
  camClient.print(mode);
  camClient.print(" HTTP/1.1\r\nHost: ");
  camClient.print(camClientIP.toString());
  camClient.print("\r\nConnection: close\r\n\r\n");

  unsigned long waitStart = millis();
  String result = "";
  bool headersEnded = false;
  while (millis() - waitStart < 4000) {
    if (camClient.available()) {
      String line = camClient.readStringUntil('\n');
      line.trim();
      if (!headersEnded) {
        if (line.length() == 0) {
          headersEnded = true;
        }
      } else {
        result = line;
        break;
      }
    } else {
      delay(10);
    }
    esp_task_wdt_reset();
  }

  camClient.stop();
  if (result.length() > 0) {
    camLastSeenAt = millis();
  }
  Serial.printf("[CAM] Power mode=%s response: %s\n", mode.c_str(),
                result.c_str());
  return result.length() > 0 ? result : "ERROR:cam_timeout";
}

void triggerRebootAllIfScheduled(unsigned long now) {
  if (!rebootAllPending) {
    return;
  }

  if (!isControllerUp(now)) {
    if (rebootAllQueuedAtMs == 0) {
      rebootAllQueuedAtMs = now;
    }
    unsigned long waitMs = now - rebootAllQueuedAtMs;
    if (waitMs < REBOOT_ALL_WAIT_FOR_CONTROLLER_MS) {
      if (!rebootAllWaitLogged) {
        Serial.printf("[REBOOT_ALL] Waiting for controller recovery (%lu/%lu ms)\n",
                      waitMs, (unsigned long)REBOOT_ALL_WAIT_FOR_CONTROLLER_MS);
        rebootAllWaitLogged = true;
      }
      return;
    }

    Serial.println("[REBOOT_ALL] Controller still down after wait window, proceeding");
    rebootAllWaitLogged = false;
  }

  if (!rebootCamDispatchDone) {
    String camResult = forwardCameraPowerCommand("reboot");
    Serial.printf("[REBOOT_ALL] CAM dispatch result: %s\n", camResult.c_str());
    rebootCamDispatchDone = true;
  }

  if (now < rebootAllAtMs) {
    return;
  }

  char rebootPath[64];
  snprintf(rebootPath, sizeof(rebootPath), "/hardware/%s/reboot.json", HARDWARE_ID);
  httpPatchWithRetry(
      rebootPath,
      "{\"rebooted\":true,\"source\":\"reboot_all\",\"scope\":\"proxy_controller_cam\",\"timestamp\":{\".sv\":\"timestamp\"}}");

  Serial.println("[REBOOT_ALL] Proxy restarting now");
  delay(150);
  ESP.restart();
}

// ==================== HOTSPOT HTTP ROUTER ====================
static bool readHttpBodyBounded(WiFiClient &client, char *buf, size_t bufSize,
                                size_t contentLength,
                                unsigned long timeoutMs) {
  if (!buf || bufSize == 0) return false;
  buf[0] = '\0';
  if (contentLength == 0) return true;
  if (contentLength >= bufSize) return false;

  size_t rd = 0;
  unsigned long lastByteAt = millis();
  unsigned long deadline = lastByteAt + timeoutMs;
  while (rd < contentLength && millis() < deadline) {
    feedCloudIoWatchdog();
    int available = client.available();
    if (available > 0) {
      int c = client.read();
      if (c < 0) {
        delay(1);
        continue;
      }
      buf[rd++] = (char)c;
      lastByteAt = millis();
      deadline = lastByteAt + timeoutMs;
      continue;
    }

    if (!client.connected() && millis() - lastByteAt > 100) {
      break;
    }
    delay(2);
    yield();
  }

  buf[rd] = '\0';
  return rd == contentLength;
}

static void sendPlainHttpResponse(WiFiClient &client, int statusCode,
                                  const char *body,
                                  bool keepAlive = false) {
  const char *statusText = statusCode == 200 ? "OK" :
                           statusCode == 400 ? "Bad Request" :
                           statusCode == 404 ? "Not Found" : "OK";
  const char *payload = body ? body : "";
  client.setTimeout(1000);
  client.printf("HTTP/1.1 %d %s\r\n", statusCode, statusText);
  client.print("Content-Type: text/plain\r\n");
  client.print(keepAlive ? "Connection: keep-alive\r\n" : "Connection: close\r\n");
  client.printf("Content-Length: %u\r\n\r\n", (unsigned int)strlen(payload));
  client.print(payload);
  feedCloudIoWatchdog();
  if (!keepAlive) {
    delay(5);
    client.stop();
  }
}

void handleCameraClient() {
  if (inApRequestHandler) {
    return;
  }

  WiFiClient client;

  // Priority 1: check persistent keep-alive connection for new request data.
  // WiFiServer::available() only returns NEW connections, so we must track
  // the existing controller socket separately for keep-alive to work.
  if (keepAliveController.connected() && keepAliveController.available()) {
    client = keepAliveController;
  } else {
    // Priority 2: accept a new incoming connection.
    client = camServer.available();
    if (!client)
      return;

    // New connection: wait for headers to arrive.
    unsigned long headerTimeout = millis() + 500;
    while (!client.available() && millis() < headerTimeout) {
      delay(10);
      esp_task_wdt_reset();
    }
    if (!client.available()) {
      client.stop();
      return;
    }
  }

  inApRequestHandler = true;
  struct ApHandlerExitGuard {
    ~ApHandlerExitGuard() { inApRequestHandler = false; }
  } apHandlerExitGuard;
  esp_task_wdt_reset();
  IPAddress clientAddr = client.remoteIP();

  client.setTimeout(2000);
  String requestLine = client.readStringUntil('\n');
  requestLine.trim();
  Serial.println("[AP] REQ: " + requestLine);

  char method[8] = "";
  char reqPath[160] = "";
  {
    int sp1 = requestLine.indexOf(' ');
    int sp2 = requestLine.indexOf(' ', sp1 + 1);
    if (sp1 > 0 && sp2 > sp1) {
      String m = requestLine.substring(0, sp1);
      String p = requestLine.substring(sp1 + 1, sp2);
      strncpy(method, m.c_str(), sizeof(method) - 1);
      strncpy(reqPath, p.c_str(), sizeof(reqPath) - 1);
    }
  }

  size_t contentLength = 0;
  String objectPath =
      "esp32cam/" + String(CAM_PREFIX) + "/relay_" + String(millis()) + ".jpg";
  String proofRole = "full";
  char uploadDeliveryId[64] = "";
  char uploadCameraId[24] = "";
  char requestControllerId[24] = "";

  while (true) {
    feedCloudIoWatchdog();
    String line = client.readStringUntil('\n');
    feedCloudIoWatchdog();
    line.trim();
    if (line.length() == 0)
      break;
    String lower = line;
    lower.toLowerCase();
    if (lower.startsWith("content-length:")) {
      String clVal = line.substring(15);
      clVal.trim();
      contentLength = (size_t)clVal.toInt();
    } else if (lower.startsWith("x-object-path:")) {
      objectPath = line.substring(14);
      objectPath.trim();
    } else if (lower.startsWith("x-proof-role:")) {
      proofRole = line.substring(13);
      proofRole.trim();
      proofRole.toLowerCase();
    } else if (lower.startsWith("x-delivery-id:")) {
      String headerDeliveryId = line.substring(14);
      headerDeliveryId.trim();
      if (headerDeliveryId.length() > 0 &&
          headerDeliveryId != "UNKNOWN_DELIVERY" &&
          headerDeliveryId.length() < sizeof(uploadDeliveryId)) {
        strncpy(uploadDeliveryId, headerDeliveryId.c_str(),
                sizeof(uploadDeliveryId) - 1);
        uploadDeliveryId[sizeof(uploadDeliveryId) - 1] = '\0';
      }
    } else if (lower.startsWith("x-camera-id:")) {
      String headerCameraId = line.substring(12);
      headerCameraId.trim();
      if (headerCameraId.length() > 0 &&
          headerCameraId.length() < sizeof(uploadCameraId)) {
        strncpy(uploadCameraId, headerCameraId.c_str(),
                sizeof(uploadCameraId) - 1);
        uploadCameraId[sizeof(uploadCameraId) - 1] = '\0';
      }
    } else if (lower.startsWith("x-controller-id:")) {
      String headerControllerId = line.substring(16);
      headerControllerId.trim();
      if (headerControllerId.length() > 0 &&
          headerControllerId.length() < sizeof(requestControllerId)) {
        strncpy(requestControllerId, headerControllerId.c_str(),
                sizeof(requestControllerId) - 1);
        requestControllerId[sizeof(requestControllerId) - 1] = '\0';
      }
    }
  }

  String reqPathStr = String(reqPath);

  // ── GET /refresh-context ──
  if (strcmp(method, "GET") == 0 && reqPathStr.startsWith("/ping")) {
    controllerLastSeenAt = millis();
    const char *body = "OK";
    String resp =
        "HTTP/1.1 200 OK\r\n"
        "Connection: keep-alive\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: " + String(strlen(body)) + "\r\n\r\n" + body;
    client.print(resp);
    client.flush();
    keepAliveController = client;
    return;
  }

  if (strcmp(method, "GET") == 0 && reqPathStr.startsWith("/identity")) {
    String body = String(HARDWARE_ID);
    String resp =
        "HTTP/1.1 200 OK\r\n"
        "Connection: close\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: " + String(body.length()) + "\r\n\r\n" + body;
    client.print(resp);
    client.flush();
    delay(10);
    client.stop();
    return;
  }

  if (!(strcmp(method, "POST") == 0 && strcmp(reqPath, "/upload") == 0) &&
      !(strcmp(method, "GET") == 0 && strcmp(reqPath, "/diag") == 0)) {
    char expectedControllerId[24];
    snprintf(expectedControllerId, sizeof(expectedControllerId),
             "CONTROLLER_%03u", (unsigned int)boxNum);
    if (requestControllerId[0] == '\0' ||
        strcmp(requestControllerId, expectedControllerId) != 0) {
      Serial.printf("[AP] Rejecting controller '%s'; expected '%s'\n",
                    requestControllerId[0] ? requestControllerId : "UNKNOWN",
                    expectedControllerId);
      sendPlainHttpResponse(client, 409, "DENY:controller_box_mismatch");
      return;
    }
  }

  if (strcmp(method, "GET") == 0 && reqPathStr.startsWith("/refresh-context")) {
    controllerLastSeenAt = millis();
    bool cloudReachable = wifiCloudHealthy || lteConnected;
    bool geoRefreshRequested = reqPathStr.indexOf("geo=1") >= 0;
    controllerContextRefreshQueued = true;
    controllerContextRefreshQueuedAt = millis();
    if (!cloudReachable) {
      deliveryContextStale = true;
    } else {
      unsigned long nowMs = millis();
      manualRefreshBurstUntil = nowMs + MANUAL_REFRESH_BURST_MS;
      lastDeliveryContextRead = 0;
      lastPhoneFetchAtMs = 0;
    }
    strncpy(lastRefreshStatus,
            cloudReachable ? (geoRefreshRequested ? "OK:REFRESH_QUEUED"
                                                  : "OK:QUEUED")
                           : "WAIT:REFRESH_QUEUED",
            sizeof(lastRefreshStatus) - 1);
    lastRefreshStatus[sizeof(lastRefreshStatus) - 1] = '\0';
    Serial.printf("[CONTEXT] Keypad refresh queued (cloud=%d stale=%d)\n",
                  cloudReachable ? 1 : 0, deliveryContextStale ? 1 : 0);

    const char *body = lastRefreshStatus;
    String resp =
        "HTTP/1.1 200 OK\r\n"
        "Connection: keep-alive\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: " + String(strlen(body)) + "\r\n\r\n" + body;
    client.print(resp);
    client.flush();
    keepAliveController = client;
    return;
  }

  // ── GET /otp ──
  // Emergency maintenance endpoint for stale NVS/Firebase hardware context.
  // Example from a laptop/phone on the same hotspot:
  // http://<discovered-lilygo-ip>:8080/clear-context
  if (strcmp(method, "GET") == 0 && reqPathStr.startsWith("/clear-context")) {
    controllerLastSeenAt = millis();
    clearDeliveryContextCaches("manual_http_clear", lteConnected && !modemHttpBusy);
    const char *body = lteConnected ? "OK:CLEARED" : "OK:CLEARED_LOCAL_NO_LTE";
    String resp =
        "HTTP/1.1 200 OK\r\n"
        "Connection: keep-alive\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: " + String(strlen(body)) + "\r\n\r\n" + body;
    client.print(resp);
    client.flush();
    keepAliveController = client;
    return;
  }

  if (strcmp(method, "GET") == 0 && reqPathStr.startsWith("/otp")) {
    controllerLastSeenAt = millis();
    String otpPart, delPart;
    unsigned long nowMs = millis();
    bool cloudReachable = wifiCloudHealthy || lteConnected;
    bool staleUsableContext =
        deliveryIdCacheValid && cachedDeliveryId[0] != '\0' &&
        (deliveryContextStale || !cloudReachable || lastContextFetchOkAtMs == 0);
    bool activeContextForController =
        deliveryIdCacheValid && cachedDeliveryId[0] != '\0' &&
        (deliveryContextTrusted || staleUsableContext);

    // Compute geofence state for structured response fields.
    bool insideGeo = false;
    int distMeters = -1;  // -1 = unknown (no GPS fix or no target coords)
    bool pickupInside = false;
    bool dropoffInside = false;
    const char *phaseStr = "NONE";

    if (theftGuardShouldBlockOtp()) {
      otpPart = "NO_OTP";
      delPart = "NO_DELIVERY";
      Serial.printf("[AP] OTP suppressed — theft state: %s\n",
                    theftGuardStateStr());
    } else {
      // ALWAYS serve the OTP to the ESP32 so it can cache it for offline drops.
      // Security is maintained because the Web Portal hides the OTP from the 
      // customer until arrival, and only the customer types it in.
      if (returnActive) {
        otpPart = returnOtpCacheValid ? String(cachedReturnOtp) : "NO_OTP";
      } else {
        otpPart = otpCacheValid ? String(cachedOtp) : "NO_OTP";
      }
      delPart = activeContextForController ? String(cachedDeliveryId) : "NO_DELIVERY";
      if (deliveryIdCacheValid && !deliveryContextTrusted && !staleUsableContext) {
        Serial.printf("[AP] Delivery %s held from controller until DB verifies active\n",
                      cachedDeliveryId);
      } else if (staleUsableContext && !deliveryContextTrusted) {
        Serial.printf("[AP] Serving stale cached delivery %s while cloud unavailable/unverified\n",
                      cachedDeliveryId);
      }

      // Still calculate geofence status for the response fields (LCD distance display)
      // Diagnostic: log preconditions for geofence so serial monitor reveals why
      // the controller is seeing "No GPS" when it shouldn't be.
      Serial.printf("[AP] GEO pre: destValid=%d pickupValid=%d gpsFix=%d phoneOk=%d delStatus='%s'\n",
                    destCoordsValid, pickupCoordsValid, gpsFix, phoneFixValid, cachedDeliveryStatus);

    bool haveTargets = activeContextForController && (destCoordsValid || pickupCoordsValid);
    if (!haveTargets && activeContextForController) {
      controllerContextRefreshQueued = true;
      Serial.println("[AP] coords_fetch_busy: target refresh queued to background task");
    }

      if (haveTargets && gpsFix) {
        // During return mode the geofence target is already swapped to pickup
        // coords (see EC-32 target swap). Use isNearAnyTarget() so the distance
        // correctly evaluates against whichever target is active.
        bool atTarget = returnActive
                          ? geoProxy.isNearAnyTarget(gpsLat, gpsLon)
                          : geoProxy.isNearDropoff(gpsLat, gpsLon);
        insideGeo = atTarget;
        distMeters = (int)geoProxy.snap.distanceM;
        if (destCoordsValid) {
          dropoffInside = (haversineMeters(gpsLat, gpsLon, destLat, destLon) <=
                           GEO_OUTER_RADIUS_M);
        }
        if (pickupCoordsValid) {
          pickupInside = (haversineMeters(gpsLat, gpsLon, pickupLat, pickupLon) <=
                          GEO_OUTER_RADIUS_M);
        }
      } else if (haveTargets && !gpsFix) {
        // GPS-loss fallback: preserve the last confirmed geofence state.
        // The GeofenceProxy hysteresis state machine (EC-94) retains its
        // stableState across GPS outages — if arrival was previously
        // confirmed with 3 consecutive inner-radius samples, losing GPS
        // does not invalidate that (the box hasn't physically moved).
        // This prevents "No GPS / Not at destination" lockout indoors.
        insideGeo = geoProxy.isArrived();
        if (returnActive) {
          pickupInside = insideGeo;
        } else {
          dropoffInside = insideGeo;
        }
        // GPS-loss: also evaluate the pickup fence from the last known GPS
        // position when pickup coords are available.  Without this, losing GPS
        // indoors at the pickup location leaves pickupInside=false and locks
        // the rider out of personal-PIN entry during PICKUP phase.
        // gpsLat/gpsLon retain their last valid values after gpsFix drops.
        if (!returnActive && pickupCoordsValid) {
          double lastPickupDist = haversineMeters(gpsLat, gpsLon,
                                                   pickupLat, pickupLon);
          if (gpsLat != 0.0 && gpsLon != 0.0 &&
              lastPickupDist <= GEO_OUTER_RADIUS_M) {
            pickupInside = true;
          }
        }
        bool usedLastConfirmed = insideGeo;
        bool usedPhoneFallback = false;
        distMeters = (int)geoProxy.snap.distanceM;

        // CRITICAL: Force an inline phone location refresh if we have no GPS
        // AND phone data is stale/missing. The main-loop refresh might not have
        // run yet (modem was busy with hardware fetch) — we need fresh phone
        // data RIGHT NOW for the pickup geofence during GPS-loss.
        bool phoneOk = phoneFixValid;
        if (!phoneOk && lteConnected) {
          Serial.println("[AP] GEO: No GPS + no phone data — relying on background phone refresh");
        }
        bool phoneFresh = false;
        unsigned long phoneAgeSec = 0;
        if (phoneOk) {
          unsigned long nowEpoch = getCurrentEpoch();
          if (phoneTimestampMs > 0 && nowEpoch > 0) {
            unsigned long phoneEpochSec = (unsigned long)(phoneTimestampMs / 1000ULL);
            if (nowEpoch >= phoneEpochSec) {
              phoneAgeSec = nowEpoch - phoneEpochSec;
            }
            phoneFresh = (phoneAgeSec <= PHONE_LOC_MAX_AGE_SEC);
          } else {
            phoneAgeSec = (nowMs - phoneFetchedAtMs) / 1000UL;
            phoneFresh = (phoneAgeSec <= PHONE_LOC_MAX_AGE_SEC);
          }
        }

        if (phoneOk && phoneFresh) {
          double distDrop = haversineMeters(phoneLat, phoneLon,
                                            geoProxy.targetLat, geoProxy.targetLon);
          double distPick = distDrop;
          if (geoProxy.pickupSet) {
            distPick = haversineMeters(phoneLat, phoneLon,
                                       geoProxy.pickupLat, geoProxy.pickupLon);
          }
          double distActive = distDrop;
          if (returnActive && geoProxy.pickupSet && distPick < distActive) {
            distActive = distPick;
          }

          bool atTarget = returnActive
                            ? geoProxy.isNearAnyTarget(phoneLat, phoneLon)
                            : geoProxy.isNearDropoff(phoneLat, phoneLon);
          if (!insideGeo) {
            insideGeo = atTarget;
          }
          distMeters = (int)(distActive + 0.5);
          usedPhoneFallback = true;
          if (destCoordsValid) {
            dropoffInside = (haversineMeters(phoneLat, phoneLon, destLat, destLon) <=
                             GEO_OUTER_RADIUS_M);
          }
          if (pickupCoordsValid) {
            pickupInside = (haversineMeters(phoneLat, phoneLon, pickupLat, pickupLon) <=
                            GEO_OUTER_RADIUS_M);
          }

          if (nowMs - lastPhoneFallbackLogAtMs >= 10000) {
            Serial.printf("[AP] Phone fallback: age=%lus dist=%dm acc=%.1fm\n",
                          phoneAgeSec, distMeters, phoneAccuracy);
            lastPhoneFallbackLogAtMs = nowMs;
          }
        } else if (phoneOk && !phoneFresh) {
          if (nowMs - lastPhoneFallbackLogAtMs >= 10000) {
            Serial.printf("[AP] Phone fallback stale: age=%lus (max %lus)\n",
                          phoneAgeSec, PHONE_LOC_MAX_AGE_SEC);
            lastPhoneFallbackLogAtMs = nowMs;
          }
        }

        // Secondary fallback: if the box NEVER had a GPS fix (booted indoors),
        // geoProxy.isArrived() will be false (default GEO_OUTSIDE). Check the
        // Firebase delivery status — if the mobile app reports ARRIVED, the
        // rider's phone GPS already confirmed the geofence. Trust that signal.
        if (!insideGeo && strcmp(cachedDeliveryStatus, "ARRIVED") == 0) {
          insideGeo = true;
          distMeters = 0;
          dropoffInside = true;
          Serial.println("[AP] GPS-loss fallback: Firebase status ARRIVED, granting GEO:1");
        } else if (!insideGeo && returnActive &&
                   strcmp(cachedDeliveryStatus, "RETURNING") == 0) {
          // EC-32: During a return trip with no GPS, the mobile app already
          // confirmed the rider is heading back. Grant GEO:1 so the controller
          // can accept the return OTP at the pickup location.
          insideGeo = true;
          distMeters = 0;
          pickupInside = true;
          Serial.println("[AP] GPS-loss fallback: RETURNING status + returnActive, granting GEO:1");
        } else if (insideGeo && usedLastConfirmed && !usedPhoneFallback) {
          Serial.println("[AP] GPS-loss fallback: using last confirmed ARRIVED state");
        }

        // PICKUP-phase last-resort: if we're in PICKUP (not return, not ARRIVED),
        // pickupCoordsValid is true, and pickupInside is STILL false after all
        // fallbacks, try one more forced phone refresh before giving up.
        if (!returnActive && pickupCoordsValid && !pickupInside &&
            strcmp(cachedDeliveryStatus, "ARRIVED") != 0 &&
            strcmp(cachedDeliveryStatus, "IN_TRANSIT") != 0 &&
            strcmp(cachedDeliveryStatus, "PICKED_UP") != 0) {
          // We're in PICKUP phase with no way to verify location.
          // If phone data exists but was stale, try refreshing once more.
          if (!phoneOk && lteConnected) {
            Serial.println("[AP] PICKUP geofence waiting on background phone refresh");
          }
          if (!pickupInside) {
            Serial.printf("[AP] PICKUP geofence BLOCKED: gps=%d/%d phoneFix=%d pickupValid=%d "
                          "gpsAt0=%d modemBusy=%d lte=%d\n",
                          gpsFix, (gpsLat != 0.0 || gpsLon != 0.0) ? 1 : 0,
                          phoneFixValid ? 1 : 0, pickupCoordsValid ? 1 : 0,
                          (gpsLat == 0.0 && gpsLon == 0.0) ? 1 : 0,
                          modemHttpBusy ? 1 : 0, lteConnected ? 1 : 0);
          }
        }
      } else if (!haveTargets && activeContextForController) {
        // Destination coords missing (Firebase JSON truncated, field not yet
        // written, or response parsing failure). Fall back to Firebase delivery
        // status from the mobile app — if it says ARRIVED, the phone GPS has
        // already verified the geofence.
        if (strcmp(cachedDeliveryStatus, "ARRIVED") == 0) {
          insideGeo = true;
          distMeters = 0;
          dropoffInside = true;
          Serial.println("[AP] coords_missing_delivery: ARRIVED override, granting GEO:1");
        } else if (returnActive && strcmp(cachedDeliveryStatus, "RETURNING") == 0) {
          insideGeo = true;
          distMeters = 0;
          pickupInside = true;
          Serial.println("[AP] coords_missing_delivery: RETURNING override, granting GEO:1");
        } else {
          Serial.printf("[AP] coords_missing_delivery: active but status='%s' -> GEO:0\n",
                        cachedDeliveryStatus);
        }
      }

      if (activeContextForController && cachedSamePickupDropoff) {
        if (pickupInside) {
          dropoffInside = true;
          insideGeo = true;
          distMeters = 0;
        } else if (dropoffInside) {
          pickupInside = true;
          insideGeo = true;
          distMeters = 0;
        }
      }

      if (distMeters >= 0) {
        lastKnownDistMeters = (int16_t)distMeters;
        lastKnownInside = insideGeo;
        lastKnownGeoAtMs = nowMs;
      } else if (lastKnownDistMeters >= 0 &&
                 (nowMs - lastKnownGeoAtMs) <= GEO_LAST_KNOWN_TTL_MS) {
        distMeters = lastKnownDistMeters;
        insideGeo = lastKnownInside;
      }
    }

    if (returnActive && activeContextForController) {
      phaseStr = "RETURN";
    } else if (!activeContextForController) {
      phaseStr = "NONE";
    } else if (strcmp(cachedDeliveryStatus, "ARRIVED") == 0 ||
               (cachedSamePickupDropoff &&
                (strcmp(cachedDeliveryStatus, "IN_TRANSIT") == 0 ||
                 strcmp(cachedDeliveryStatus, "PICKED_UP") == 0 ||
                 strcmp(cachedDeliveryStatus, "PICKEDUP") == 0))) {
      phaseStr = "DROPOFF";
    } else if (strcmp(cachedDeliveryStatus, "IN_TRANSIT") == 0 ||
               strcmp(cachedDeliveryStatus, "PICKED_UP") == 0 ||
               strcmp(cachedDeliveryStatus, "PICKEDUP") == 0) {
      phaseStr = "IN_TRANSIT";
    } else {
      phaseStr = "PICKUP";
    }

    String body = otpPart + "," + delPart;

    // Append remote status command (UNLOCKING/LOCKED/REBOOT_ALL) if pending.
    bool statusActive =
        cachedStatusStage == CMD_STAGE_PENDING && cachedStatus[0] != '\0';

    if (statusActive) {
      maybeRearmPendingCommand(nowMs);
      body += "," + String(cachedStatus);
      if (cachedStatusServesRemaining > 0) {
        cachedStatusServesRemaining--;
      }
      if (cachedStatusServesRemaining == 0) {
        maybeRearmPendingCommand(nowMs);
      }
    } else {
      // No admin command — leave status field empty.
      body += ",";
    }

    // Append structured geofence fields and context health.
    char geoFields[144];
    snprintf(geoFields, sizeof(geoFields),
         ",DIST:%d,GEO:%d,RET:%d,PHASE:%s,PUP:%d,DRO:%d,CTX:%s,REFQ:%d",
         distMeters, insideGeo ? 1 : 0, returnActive ? 1 : 0,
         phaseStr ? phaseStr : "NONE", pickupInside ? 1 : 0,
         dropoffInside ? 1 : 0,
         deliveryContextStale ? "STALE" :
             (deliveryContextTrusted ? "FRESH" : "UNVERIFIED"),
         controllerContextRefreshQueued ? 1 : 0);
    body += String(geoFields);

    Serial.printf("[AP] -> GET /otp  serving: '%s'\n", body.c_str());
    String resp =
        "HTTP/1.1 200 OK\r\n"
        "Connection: keep-alive\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: " + String(body.length()) + "\r\n\r\n" + body;
    client.print(resp);
    client.flush();
    // Keep-alive: store client reference for subsequent requests on same socket.
    keepAliveController = client;
    return;
  }

  // ── POST /command-ack ──
  if (strcmp(method, "POST") == 0 && strcmp(reqPath, "/command-ack") == 0) {
    controllerLastSeenAt = millis();
    Serial.println("[AP] -> POST /command-ack");
    char jsonBuf[320] = "";
    if (!readHttpBodyBounded(client, jsonBuf, sizeof(jsonBuf), contentLength, 3000)) {
      Serial.printf("[AP] Bad command-ack body cl=%u\n", (unsigned int)contentLength);
      sendPlainHttpResponse(client, 400, "BAD_BODY");
      return;
    }

    char cmd[24] = "";
    char status[32] = "";
    char details[96] = "";
    extractJsonStringValue(jsonBuf, "command", cmd, sizeof(cmd));
    extractJsonStringValue(jsonBuf, "status", status, sizeof(status));
    extractJsonStringValue(jsonBuf, "details", details, sizeof(details));

    if (cmd[0] == '\0' && cachedStatus[0] != '\0') {
      strncpy(cmd, cachedStatus, sizeof(cmd) - 1);
      cmd[sizeof(cmd) - 1] = '\0';
    }
    if (status[0] == '\0') {
      strncpy(status, "unknown", sizeof(status) - 1);
      status[sizeof(status) - 1] = '\0';
    }

    Serial.printf("[CMD_ACK] command=%s status=%s details=%s\n", cmd, status,
                  details);

    bool trackedAck = false;
    if (cmd[0] != '\0') {
      bool hasDifferentPending =
          cachedStatusStage == CMD_STAGE_PENDING && cachedStatus[0] != '\0' &&
          strcmp(cachedStatus, cmd) != 0;
      if (hasDifferentPending) {
        Serial.printf("[CMD_ACK] Ignoring mismatch ACK (%s) while pending %s\n",
                      cmd, cachedStatus);
      } else {
        markCommandAckSent(cmd, status, details);
        trackedAck = true;
      }
    }

    // Send HTTP 200 OK — keep-alive so controller can reuse socket.
    // Content-Length ensures controller knows body is complete even though
    // we continue with a slow Firebase write on our side.
    String resp = "HTTP/1.1 200 OK\r\nConnection: keep-alive\r\nContent-Length: 2\r\n\r\nOK";
    client.print(resp);
    client.flush();
    keepAliveController = client;
    
    queueCommandAckFirebaseWrite(cmd, status, details, trackedAck);

    // EC-32: Handle return completion signal from controller.
    // When a return OTP unlock succeeds, the controller sends RETURN_COMPLETE.
    // Clear local return state and write completion flag to Firebase so the
    // web API mark-retrieved logic can finalize the RETURNED transition.
    if (strcmp(cmd, "RETURN_COMPLETE") == 0 && strcmp(status, "executed") == 0) {
      Serial.println("[CMD_ACK] Return completed — clearing return state");
      returnActive = false;
      lastReturnActive = false;
      cachedReturnOtp[0] = '\0';
      returnOtpCacheValid = false;

      queueReturnCompletionFirebaseWrite();
    }
    return;
  }

  // â”€â”€ GET /face-check â”€â”€
  if (strcmp(method, "GET") == 0 && reqPathStr.startsWith("/face-check")) {
    Serial.println("[AP] -> GET /face-check");

    // Block face check when outside geofence or stolen/lockdown
    if (theftGuardShouldBlockOtp() ||
        (destCoordsValid && gpsFix && !geoProxy.isNearAnyTarget(gpsLat, gpsLon))) {
      Serial.println("[AP] Face-check blocked — outside geofence or theft state");
      String blocked = "NO_FACE";
      String resp =
          "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: " +
          String(blocked.length()) + "\r\n\r\n" + blocked;
      client.print(resp);
      client.flush();
    delay(10);
    unsigned long _purgeLimit = millis() + 500;
    while (client.available() && millis() < _purgeLimit) client.read();
    client.stop();
      return;
    }

    String deliveryId = "";
    int qIdx = reqPathStr.indexOf("?delivery_id=");
    if (qIdx >= 0) {
      deliveryId = reqPathStr.substring(qIdx + 13);
    }
    String result = forwardFaceCheck(deliveryId);
    String resp =
        "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: " +
        String(result.length()) + "\r\n\r\n" + result;
    client.print(resp);
    client.flush();
    delay(10);
    unsigned long _purgeLimit = millis() + 500;
    while (client.available() && millis() < _purgeLimit) client.read();
    client.stop();
    return;
  }

  // â”€â”€ GET /cam-power?mode=sleep|wake â”€â”€
  if (strcmp(method, "GET") == 0 && reqPathStr.startsWith("/cam-power")) {
    String mode = "";
    int qIdx = reqPathStr.indexOf("?mode=");
    if (qIdx >= 0) {
      mode = reqPathStr.substring(qIdx + 6);
    }
    mode.toLowerCase();

    if (mode != "sleep" && mode != "wake") {
      String body = "ERROR:invalid_mode";
      String resp =
          "HTTP/1.1 400 Bad Request\r\nContent-Type: text/plain\r\n"
          "Content-Length: " +
          String(body.length()) + "\r\n\r\n" + body;
      client.print(resp);
      client.flush();
      delay(10);
      client.stop();
      return;
    }

    String result = forwardCameraPowerCommand(mode);
    String resp =
        "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: " +
        String(result.length()) + "\r\n\r\n" + result;
    client.print(resp);
    client.flush();
    delay(10);
    client.stop();
    return;
  }

  // â”€â”€ POST /personal-pin-verify â”€â”€
  if (strcmp(method, "POST") == 0 && strcmp(reqPath, "/personal-pin-verify") == 0) {
    Serial.println("[AP] -> POST /personal-pin-verify");
    char jsonBuf[320] = "";
    if (!readHttpBodyBounded(client, jsonBuf, sizeof(jsonBuf), contentLength, 3000)) {
      Serial.printf("[AP] Bad personal-pin body cl=%u\n", (unsigned int)contentLength);
      sendPlainHttpResponse(client, 400, "BAD_BODY");
      return;
    }
    feedCloudIoWatchdog();

    char pin[12] = "";
    extractJsonStringValue(jsonBuf, "pin", pin, sizeof(pin));
    char reqBoxId[16] = "";
    extractJsonStringValue(jsonBuf, "box_id", reqBoxId, sizeof(reqBoxId));
    bool currentlyLocked = (strstr(jsonBuf, "\"currently_locked\":true") != NULL);

    const char *resultBody = "DENY:invalid";
    bool verified = false;
    bool pinCacheReady =
        personalPinEnabled &&
        cachedPersonalPinHashMcu[0] != '\0' &&
        cachedPersonalPinSalt[0] != '\0' &&
        cachedPersonalPinRiderId[0] != '\0';

    if (reqBoxId[0] != '\0' && strcmp(reqBoxId, HARDWARE_ID) != 0) {
      resultBody = "DENY:box_mismatch";
    } else if (pin[0] == '\0') {
      resultBody = "DENY:missing_pin";
    } else if (!personalPinEnabled) {
      personalPinRuntimeRefreshQueued = true;
      personalPinRuntimeRefreshForce = true;
      resultBody = "DENY:disabled";
    } else if (!pinCacheReady) {
      personalPinRuntimeRefreshQueued = true;
      personalPinRuntimeRefreshForce = true;
      resultBody = "DENY:sync_failed";
    } else {
      verified = verifyPersonalPinLocal(pin);
      if (verified) {
        resultBody = currentlyLocked ? "ALLOW_UNLOCK" : "ALLOW_RELOCK";
      } else {
        personalPinRuntimeRefreshQueued = true;
        personalPinRuntimeRefreshForce = true;
        resultBody = "DENY:mismatch";
      }
    }

    Serial.printf("[PIN] Verify result: %s (enabled=%d)\n",
                  resultBody, personalPinEnabled ? 1 : 0);

    sendPlainHttpResponse(client, 200, resultBody);

    writePersonalPinAudit("RIDER_MANUAL_PIN_ATTEMPT",
                          verified ? "success" : "denied", currentlyLocked,
                          false);
    if (verified) {
      writePersonalPinAudit("RIDER_MANUAL_PIN_TOGGLE",
                            currentlyLocked ? "unlock" : "relock",
                            currentlyLocked, false);
    }
    return;
  }

  // â”€â”€ POST /event â”€â”€
  if (strcmp(method, "POST") == 0 && strcmp(reqPath, "/event") == 0) {
    controllerLastSeenAt = millis();
    Serial.println("[AP] -> POST /event");
    char jsonBuf[384] = "";
    if (!readHttpBodyBounded(client, jsonBuf, sizeof(jsonBuf), contentLength, 3000)) {
      Serial.printf("[AP] Bad event body cl=%u\n", (unsigned int)contentLength);
      sendPlainHttpResponse(client, 400, "BAD_BODY");
      return;
    }
    Serial.printf("[AP] Event: %s\n", jsonBuf);

    // Send HTTP 200 OK — keep-alive so controller can reuse socket
    String resp = "HTTP/1.1 200 OK\r\nConnection: keep-alive\r\nContent-Length: 2\r\n\r\nOK";
    client.print(resp);
    client.flush();
    keepAliveController = client;

    // Reed switch tamper: Controller detected unauthorized lid-open
    if (strstr(jsonBuf, "\"tamper\":true") != NULL) {
      // During suppression, capture retriggers so they can be replayed after
      // unlock ACK or suppression timeout.
      if (tamperSuppressedByAdmin) {
        captureSuppressedTamper(millis());
      } else {
        Serial.println("[AP] TAMPER event received — writing to Firebase");
        queueTamperFirebaseWrite();
      }
    } else {
      String jsonStr = String(jsonBuf);
      bool ov = (strstr(jsonBuf, "\"otp_valid\":true") != NULL);
      bool fd = (strstr(jsonBuf, "\"face_detected\":true") != NULL);
      bool ul = (strstr(jsonBuf, "\"unlocked\":true") != NULL);
      int faceAttempts = 0;
      bool faceRetryExhausted = false;
      bool fallbackRequired = false;
      int unlockLatencyMs = 0;
      char failureReason[24] = "";

      readTopLevelJsonInt(jsonStr, "face_attempts", faceAttempts);
      readTopLevelJsonBool(jsonStr, "face_retry_exhausted", faceRetryExhausted);
      readTopLevelJsonBool(jsonStr, "fallback_required", fallbackRequired);
      readTopLevelJsonInt(jsonStr, "unlock_latency_ms", unlockLatencyMs);
      readTopLevelJsonString(jsonStr, "failure_reason", failureReason,
                             sizeof(failureReason));
      
      // Clear suppression only on a fully validated unlock event.
      // This prevents noisy/partial events from re-enabling tamper spam.
      bool validUnlock = (ul && ov && fd);
      if (validUnlock) {
        clearTamperSuppression("validated_unlock_event");
        replayPendingTamperIfAny(millis(), "validated_unlock_event");
      } else if (ul && tamperSuppressedByAdmin) {
        Serial.println("[AP] Unlock event without full validation — suppression kept");
      }
      
      // Also write alert events to the lock_events stream so it appears in web UI
      queueLockEventFirebaseWrite(ov, fd, ul, faceAttempts, faceRetryExhausted,
                                  fallbackRequired, failureReason,
                                  (unsigned long)(unlockLatencyMs > 0 ? unlockLatencyMs : 0));
    }
    
    return;
  }

  // â”€â”€ GET /diag â”€â”€
  if (strcmp(method, "GET") == 0 && strcmp(reqPath, "/diag") == 0) {
    unsigned long nowMs = millis();
    unsigned long camAgeMs = (camLastSeenAt > 0) ? (nowMs - camLastSeenAt) : 0;
    bool camUp = camClientKnown && camLastSeenAt > 0 &&
                 (camAgeMs <= CAM_LIVENESS_STALE_MS);
    unsigned long ctrlAgeMs =
      (controllerLastSeenAt > 0) ? (nowMs - controllerLastSeenAt) : 0;
    bool ctrlUp = controllerLastSeenAt > 0 &&
            (ctrlAgeMs <= CONTROLLER_LIVENESS_STALE_MS);

     char body[760];
    snprintf(body, sizeof(body),
       "batt_pct=%d,batt_v=%.2f,batt_mah=%lu,batt_cap_mah=%lu,batt_pin_mv=%d,batt_adc=%d,batt_ok=%d,batt_b_pct=%d,batt_b_v=%.2f,batt_b_mah=%lu,batt_b_ok=%d,rssi=%d,csq=%d,gps_fix=%d,lte=%d,modem=%d,time=%d,fb_fail=%u,uptime=%lu,cam_up=%d,cam_age=%lu,ctrl_up=%d,ctrl_age=%lu,cmd_stage=%u,cmd_pending=%d,conn_state=%u,lte_reconn_ms=%lu,upload_active=%d,upload_pct=%d,upload_status=%s,upload_age=%lu",
             batteryGetPercentage(BATT_CH_A), batteryGetVoltage(BATT_CH_A),
         (unsigned long)batteryGetRemainingMah(BATT_CH_A),
         (unsigned long)batteryGetCapacityMah(BATT_CH_A),
             batteryGetPinMilliVolts(BATT_CH_A),
             batteryGetAdcRaw(BATT_CH_A),
             batterySensorLooksValid(BATT_CH_A) ? 1 : 0,
             batteryGetPercentage(BATT_CH_B), batteryGetVoltage(BATT_CH_B),
         (unsigned long)batteryGetRemainingMah(BATT_CH_B),
             batterySensorLooksValid(BATT_CH_B) ? 1 : 0,
             cachedSignalRssi, cachedSignalCsq,
             gpsFix ? 1 : 0,
             lteConnected ? 1 : 0,
             modemOK ? 1 : 0,
             timeSynced ? 1 : 0,
             (unsigned int)firebaseFailures,
             (unsigned long)nowMs,
             camUp ? 1 : 0,
         (unsigned long)camAgeMs,
         ctrlUp ? 1 : 0,
       (unsigned long)ctrlAgeMs,
       (unsigned int)cachedStatusStage,
       (cachedStatusStage == CMD_STAGE_PENDING) ? 1 : 0,
       (unsigned int)connectivityState,
       lastLteReconnectLatencyMs,
       photoUploadActive ? 1 : 0,
       photoUploadProgress,
       photoUploadStatus,
       photoUploadUpdatedAt > 0 ? (unsigned long)(nowMs - photoUploadUpdatedAt) : 0);

    String resp =
        "HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Type: text/plain\r\n"
        "Content-Length: " + String(strlen(body)) + "\r\n\r\n" + String(body);
    client.print(resp);
    client.flush();
    delay(10);
    client.stop();
    return;
  }

  if (!(strcmp(method, "POST") == 0 && strcmp(reqPath, "/upload") == 0)) {
    Serial.printf("[AP] -> 404 %s %s\n", method, reqPath);
    String body = "NOT_FOUND";
    String resp = "HTTP/1.1 404 Not Found\r\nContent-Type: "
                  "text/plain\r\nContent-Length: " +
                  String(body.length()) + "\r\n\r\n" + body;
    client.print(resp);
    client.flush();
    delay(10);
    unsigned long _purgeLimit = millis() + 500;
    while (client.available() && millis() < _purgeLimit) client.read();
    client.stop();
    return;
  }

  // â”€â”€ POST /upload â€” camera relay only â”€â”€
  Serial.println("[AP] -> POST /upload");
  Serial.printf("[AP] CAM IP: %s | Image: %u bytes\n",
                clientAddr.toString().c_str(), contentLength);

  if (uploadCameraId[0] == '\0' || strcmp(uploadCameraId, CAM_PREFIX) != 0) {
    Serial.printf("[AP] Rejecting upload from camera '%s'; expected '%s'\n",
                  uploadCameraId[0] ? uploadCameraId : "UNKNOWN", CAM_PREFIX);
    String body = "FAIL:camera_box_mismatch";
    String resp =
        "HTTP/1.1 409 Conflict\r\n"
        "Connection: close\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: " + String(body.length()) + "\r\n\r\n" + body;
    client.print(resp);
    client.flush();
    delay(10);
    client.stop();
    return;
  }

  camClientIP = clientAddr;
  camClientKnown = true;
  camLastSeenAt = millis();

  bool uploadOK = false;
  String failReason = "";
  bool isPreviewProof = proofRole == "preview";
  const char *writebackDeliveryId =
      uploadDeliveryId[0] ? uploadDeliveryId :
      ((deliveryIdCacheValid && cachedDeliveryId[0] != '\0')
           ? cachedDeliveryId
           : "UNKNOWN_DELIVERY");
  bool hasWritebackDelivery =
      strcmp(writebackDeliveryId, "UNKNOWN_DELIVERY") != 0;
  bool finalProofWritebackOK = isPreviewProof;

  if (contentLength == 0) {
    failReason = "FAIL:bad_cl:" + String(contentLength);
  } else {
    publishPhotoUploadState(isPreviewProof ? "COMPRESSING" : "UPLOADING",
                            isPreviewProof ? 15 : 45,
                            writebackDeliveryId,
                            contentLength,
                            "",
                            isPreviewProof ? "preview" : "full");
    uploadOK = uploadClientToSupabaseViaLTE(client, contentLength, objectPath);
    if (!uploadOK)
      failReason = relayDiag;
  }

  // Write the public URL back to Firebase so web/mobile apps can find the image
  if (uploadOK && (wifiCloudHealthy || lteConnected)) {
    char publicUrl[256];
    snprintf(publicUrl, sizeof(publicUrl),
             "%s/storage/v1/object/public/%s/%s",
             SUPABASE_URL, SUPABASE_BUCKET, objectPath.c_str());

    if (hasWritebackDelivery) {
      // Preview photos unlock the app UI quickly; full photos preserve the
      // existing archival proof fields.
      char deliveryJson[768];
      if (isPreviewProof) {
        snprintf(deliveryJson, sizeof(deliveryJson),
                 "{\"proof_photo_preview_url\":\"%s\","
                 "\"proof_photo_preview_object_path\":\"%s\","
                 "\"proof_photo_preview_uploaded_at\":{\".sv\":\"timestamp\"}}",
                 publicUrl, objectPath.c_str());
      } else {
        snprintf(deliveryJson, sizeof(deliveryJson),
                 "{\"proof_photo_url\":\"%s\",\"proof_photo_object_path\":\"%s\","
                 "\"proof_photo_uploaded_at\":{\".sv\":\"timestamp\"}}",
                 publicUrl, objectPath.c_str());
      }
      char deliveryPath[96];
      snprintf(deliveryPath, sizeof(deliveryPath),
               "/deliveries/%s.json", writebackDeliveryId);
      bool deliveryFbOK = httpPatchWithRetry(deliveryPath, deliveryJson);
      if (!deliveryFbOK) {
        delay(250);
        deliveryFbOK = httpPatchWithRetry(deliveryPath, deliveryJson);
      }
      Serial.printf("[RELAY] Delivery proof writeback: %s\n",
                    deliveryFbOK ? "OK" : "FAIL");
      if (!isPreviewProof && deliveryFbOK) {
        finalProofWritebackOK = true;
      }

      char auditJson[768];
      if (isPreviewProof) {
        snprintf(auditJson, sizeof(auditJson),
                 "{\"delivery_id\":\"%s\",\"box_id\":\"%s\","
                 "\"latest_photo_preview_url\":\"%s\","
                 "\"latest_photo_preview_object_path\":\"%s\","
                 "\"latest_photo_preview_uploaded_at\":{\".sv\":\"timestamp\"}}",
                 writebackDeliveryId, HARDWARE_ID, publicUrl, objectPath.c_str());
      } else {
        snprintf(auditJson, sizeof(auditJson),
                 "{\"delivery_id\":\"%s\",\"box_id\":\"%s\","
                 "\"latest_photo_url\":\"%s\",\"latest_photo_object_path\":\"%s\","
                 "\"latest_photo_uploaded_at\":{\".sv\":\"timestamp\"}}",
                 writebackDeliveryId, HARDWARE_ID, publicUrl, objectPath.c_str());
      }
      char auditPath[96];
      snprintf(auditPath, sizeof(auditPath),
               "/audit_logs/%s.json", writebackDeliveryId);
      bool auditFbOK = httpPatchWithRetry(auditPath, auditJson);
      if (!auditFbOK) {
        delay(250);
        auditFbOK = httpPatchWithRetry(auditPath, auditJson);
      }
      Serial.printf("[RELAY] Audit proof writeback: %s\n",
                    auditFbOK ? "OK" : "FAIL");

      // CRITICAL: The Web Tracking page Server-Side-Render uses the Supabase 'deliveries' table directly.
      // We must write the proof_photo_url to Supabase via REST API right after upload.
      if (!isPreviewProof) {
        char supabaseDeliveryJson[320];
        snprintf(supabaseDeliveryJson, sizeof(supabaseDeliveryJson),
                 "{\"proof_photo_url\":\"%s\"}", publicUrl);
        bool supabaseWritebackOK =
            httpPatchSupabase("deliveries", writebackDeliveryId, supabaseDeliveryJson);
        if (!supabaseWritebackOK) {
          delay(250);
          supabaseWritebackOK =
              httpPatchSupabase("deliveries", writebackDeliveryId, supabaseDeliveryJson);
        }
        if (!supabaseWritebackOK) {
          Serial.println("[RELAY] Supabase delivery proof writeback: FAIL");
        }
      }
    } else if (!isPreviewProof) {
      finalProofWritebackOK = false;
      Serial.println("[RELAY] Full proof has no delivery id; keeping upload pending");
    }

    char urlJson[448];
    snprintf(urlJson, sizeof(urlJson),
             "{\"last_upload_public_url\":\"%s\",\"last_upload_object_path\":\"%s\","
             "\"last_upload_role\":\"%s\",\"last_upload_delivery_id\":\"%s\","
             "\"last_upload_uploaded_at\":{\".sv\":\"timestamp\"}}",
             publicUrl, objectPath.c_str(), isPreviewProof ? "preview" : "full",
             writebackDeliveryId);

    char camPath[64];
    snprintf(camPath, sizeof(camPath), "/hardware/%s/camera.json", HARDWARE_ID);
    bool fbOK = httpPatchWithRetry(camPath, urlJson);
    if (!fbOK) {
      delay(250);
      fbOK = httpPatchWithRetry(camPath, urlJson);
    }
    Serial.printf("[RELAY] Firebase URL writeback: %s\n", fbOK ? "OK" : "FAIL");

    if (!isPreviewProof && theftGuardGetState() != TG_NORMAL) {
      char theftPhotoPath[128];
      snprintf(theftPhotoPath, sizeof(theftPhotoPath),
               "/boxes/%s/theft_status/recovery_photos/%lu.json",
               HARDWARE_ID, getCurrentEpoch());
      char theftPhotoJson[384];
      snprintf(theftPhotoJson, sizeof(theftPhotoJson),
               "{\"url\":\"%s\",\"object_path\":\"%s\","
               "\"uploaded_at\":{\".sv\":\"timestamp\"},\"source\":\"box_camera\"}",
               publicUrl, objectPath.c_str());
      bool theftFbOK = httpPutWithRetry(theftPhotoPath, theftPhotoJson);
      Serial.printf("[RELAY] Theft recovery photo writeback: %s\n",
                    theftFbOK ? "OK" : "FAIL");
    }
  }

  if (uploadOK && !isPreviewProof && !finalProofWritebackOK) {
    uploadOK = false;
    failReason = "FAIL:full_writeback_failed";
  }

  if (uploadOK) {
    publishPhotoUploadState(isPreviewProof ? "UPLOADING" : "COMPLETED",
                            isPreviewProof ? 60 : 100,
                            writebackDeliveryId,
                            contentLength,
                            "",
                            isPreviewProof ? "preview" : "full");
  } else {
    publishPhotoUploadState("FAILED", 0,
                            writebackDeliveryId,
                            contentLength,
                            failReason.c_str(),
                            isPreviewProof ? "preview" : "full");
  }

  String body = uploadOK ? ("OK:" + relayDiag) : failReason;
  String resp = uploadOK ? "HTTP/1.1 200 OK\r\n"
                         : "HTTP/1.1 500 Internal Server Error\r\n";
  resp += "Content-Length: " + String(body.length()) + "\r\n\r\n" + body;
  client.print(resp);
  client.flush();
    delay(10);
    unsigned long _purgeLimit = millis() + 500;
    while (client.available() && millis() < _purgeLimit) client.read();
    client.stop();
  Serial.println("[AP] Done: " + body);
}

// ==================== AUTO-PROVISIONING ====================
// Lightweight Firebase GET that returns the response body as a String.
// Used only during first-boot registration (not on the hot path).
String httpGetFromFirebase(const char *path) {
  String wifiBody;
  if (wifiGetFromFirebase(path, wifiBody)) {
    return wifiBody;
  }

  if (modemHttpBusy) {
    Serial.printf("[HTTP] Busy, skipping Firebase GET %s\n",
                  path ? path : "(null)");
    return "";
  }
  ModemLockGuard modemLock("httpGetFromFirebase", 250);
  if (!modemLock.ok()) return "";

  modemHttpBusy = true;
  String resp;
  sendATAndWait("+HTTPTERM", 1000);
  delay(100);

  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000L, resp) != 1) {
    sendATAndWait("+HTTPTERM", 1000);
    modemHttpBusy = false;
    return "";
  }

  modem.sendAT("+HTTPPARA=\"CID\",1");
  modem.waitResponse(3000L);

  String url = "https://" + String(FIREBASE_HOST) + path;
  if (strlen(FIREBASE_AUTH) > 0) {
    url += "?auth=" + String(FIREBASE_AUTH);
  }
  modem.sendAT(("+HTTPPARA=\"URL\",\"" + url + "\"").c_str());
  modem.waitResponse(3000L);

  modem.sendAT("+HTTPSSL=1");
  modem.waitResponse(3000L);

  modem.sendAT("+HTTPACTION=0");
  unsigned long waitStart = millis();
  bool actionOK = false;
  int httpStatus = 0;
  while (millis() - waitStart < 15000) {
    if (apStarted) handleCameraClient(); // Fast local serving
    if (modem.stream.available()) {
      String line = modem.stream.readStringUntil('\n');
      line.trim();
      if (line.indexOf("+HTTPACTION:") >= 0) {
        int c1 = line.indexOf(',');
        int c2 = line.indexOf(',', c1 + 1);
        if (c1 > 0 && c2 > 0)
          httpStatus = line.substring(c1 + 1, c2).toInt();
        actionOK = true;
        break;
      }
    }
    delay(50);
    esp_task_wdt_reset();
  }

  String body = "";
  if (actionOK && httpStatus == 200) {
    modem.stream.print("AT+HTTPREAD=0,512\r\n");

    int dataLen = -1;
    unsigned long t0 = millis();
    while (millis() - t0 < 6000) {
      if (apStarted) handleCameraClient(); // Fast local serving
      if (modem.stream.available()) {
        String line = modem.stream.readStringUntil('\n');
        line.trim();
        if (line.indexOf("+HTTPREAD:") >= 0) {
          int comma = line.lastIndexOf(',');
          if (comma >= 0) {
            dataLen = line.substring(comma + 1).toInt();
          } else {
            int colon = line.lastIndexOf(':');
            if (colon >= 0)
              dataLen = line.substring(colon + 1).toInt();
          }
          break;
        }
      }
      esp_task_wdt_reset();
    }

    int toRead = (dataLen > 0 && dataLen <= 512) ? dataLen : 512;
    unsigned long t1 = millis();
    while ((int)body.length() < toRead && millis() - t1 < 5000) {
      while (modem.stream.available() && (int)body.length() < toRead) {
        body += (char)modem.stream.read();
      }
      esp_task_wdt_reset();
    }
    delay(50);
    while (modem.stream.available())
      modem.stream.read();
  }

  sendATAndWait("+HTTPTERM", 1000);
  modemHttpBusy = false;
  return body;
}

static bool parseFirebaseStringValue(const String &body, char *out, size_t outLen) {
  if (!out || outLen == 0) return false;
  out[0] = '\0';

  String value = body;
  value.trim();
  if (value.length() == 0 || value == "null") {
    return false;
  }
  if (value.length() >= 2 && value[0] == '"' && value[value.length() - 1] == '"') {
    value = value.substring(1, value.length() - 1);
    value.trim();
  }
  if (value.length() == 0) {
    return false;
  }
  size_t len = (value.length() >= outLen) ? (outLen - 1) : value.length();
  memcpy(out, value.c_str(), len);
  out[len] = '\0';
  return true;
}

static bool parseFirebaseBoolValue(const String &body, bool &out, bool &hasValue) {
  String value = body;
  value.trim();
  hasValue = false;
  if (value.length() == 0 || value == "null") {
    return false;
  }
  if (value == "true" || value == "1") {
    out = true;
    hasValue = true;
    return true;
  }
  if (value == "false" || value == "0") {
    out = false;
    hasValue = true;
    return true;
  }
  return false;
}

static bool fetchFirebaseDouble(const char *path, double &outVal) {
  if (!path || path[0] == '\0') return false;
  feedCloudIoWatchdog();
  String body = httpGetFromFirebase(path);
  feedCloudIoWatchdog();
  body.trim();
  if (body.length() == 0 || body == "null") {
    return false;
  }
  outVal = body.toDouble();
  feedCloudIoWatchdog();
  return true;
}

static void applyTargetCoords(double lat, double lon, const char *source) {
  bool changed = (lat != destLat || lon != destLon);
  destLat = lat;
  destLon = lon;
  destCoordsValid = true;
  if (changed) {
    geoProxy.setTarget(destLat, destLon);
    theftGuardSetGeofence((float)destLat, (float)destLon, TG_GEOFENCE_RADIUS_KM);
    Serial.printf("[GEO] Target %s set: %.6f, %.6f\n",
                  source ? source : "coords", destLat, destLon);
  }
}

static void applyPickupCoords(double lat, double lon, const char *source) {
  bool changed = (lat != pickupLat || lon != pickupLon);
  pickupLat = lat;
  pickupLon = lon;
  pickupCoordsValid = true;
  if (changed) {
    geoProxy.setPickup(pickupLat, pickupLon);
    Serial.printf("[GEO] Pickup %s set: %.6f, %.6f\n",
                  source ? source : "coords", pickupLat, pickupLon);
  }
}

static bool fetchFirebaseDoubleAny(const char *const *paths, uint8_t pathCount,
                                   double &outVal) {
  for (uint8_t i = 0; i < pathCount; i++) {
    feedCloudIoWatchdog();
    if (paths[i] && fetchFirebaseDouble(paths[i], outVal)) {
      return true;
    }
    feedCloudIoWatchdog();
  }
  return false;
}

static bool refreshHardwareCoordsExact() {
  if (!lteConnected) return false;

  bool anyOk = false;
  char pathA[96], pathB[96], pathC[96];

  double dLat = 0.0, dLon = 0.0;
  snprintf(pathA, sizeof(pathA), "/hardware/%s/target_lat.json", HARDWARE_ID);
  snprintf(pathB, sizeof(pathB), "/hardware/%s/dest_lat.json", HARDWARE_ID);
  snprintf(pathC, sizeof(pathC), "/hardware/%s/dropoff_lat.json", HARDWARE_ID);
  const char *latPaths[] = {pathA, pathB, pathC};
  bool gotLat = fetchFirebaseDoubleAny(latPaths, 3, dLat);
  feedCloudIoWatchdog();

  snprintf(pathA, sizeof(pathA), "/hardware/%s/target_lng.json", HARDWARE_ID);
  snprintf(pathB, sizeof(pathB), "/hardware/%s/dest_lon.json", HARDWARE_ID);
  snprintf(pathC, sizeof(pathC), "/hardware/%s/dropoff_lng.json", HARDWARE_ID);
  const char *lonPaths[] = {pathA, pathB, pathC};
  bool gotLon = fetchFirebaseDoubleAny(lonPaths, 3, dLon);
  feedCloudIoWatchdog();

  if (gotLat && gotLon && (dLat != 0.0 || dLon != 0.0)) {
    applyTargetCoords(dLat, dLon, "hardware");
    anyOk = true;
  } else if (!destCoordsValid) {
    Serial.println("[GEO] coords_missing_hardware: dropoff target unavailable");
  }

  double pLat = 0.0, pLon = 0.0;
  snprintf(pathA, sizeof(pathA), "/hardware/%s/pickup_lat.json", HARDWARE_ID);
  bool gotPLat = fetchFirebaseDouble(pathA, pLat);
  feedCloudIoWatchdog();
  snprintf(pathA, sizeof(pathA), "/hardware/%s/pickup_lng.json", HARDWARE_ID);
  bool gotPLon = fetchFirebaseDouble(pathA, pLon);
  feedCloudIoWatchdog();

  if (gotPLat && gotPLon && (pLat != 0.0 || pLon != 0.0)) {
    applyPickupCoords(pLat, pLon, "hardware");
    anyOk = true;
  }

  bool sameSiteValue = false;
  bool hasSameSite = false;
  snprintf(pathA, sizeof(pathA), "/hardware/%s/same_pickup_dropoff.json", HARDWARE_ID);
  feedCloudIoWatchdog();
  String sameSiteBody = httpGetFromFirebase(pathA);
  feedCloudIoWatchdog();
  sameSiteBody.trim();
  if (parseFirebaseBoolValue(sameSiteBody, sameSiteValue, hasSameSite) && hasSameSite) {
    cachedSamePickupDropoff = deliveryIdCacheValid && sameSiteValue;
    anyOk = true;
  }

  if (anyOk) {
    Serial.printf("[GEO] coords_loaded: hardware dest=%d pickup=%d same=%d\n",
                  destCoordsValid ? 1 : 0, pickupCoordsValid ? 1 : 0,
                  cachedSamePickupDropoff ? 1 : 0);
  }
  return anyOk;
}

static bool refreshTargetCoordsFromDelivery(const char *deliveryId, bool force) {
  if (!lteConnected || !deliveryId || deliveryId[0] == '\0') {
    return destCoordsValid;
  }

  unsigned long nowMs = millis();
  if (!force && nowMs - lastFallbackTargetFetchAtMs < FALLBACK_TARGET_FETCH_INTERVAL_MS) {
    return destCoordsValid;
  }
  lastFallbackTargetFetchAtMs = nowMs;

  double dLat = 0.0;
  double dLon = 0.0;
  bool gotLat = false;
  bool gotLon = false;

  char path[128];
  char path2[128], path3[128];
  snprintf(path, sizeof(path), "/deliveries/%s/target_lat.json", deliveryId);
  snprintf(path2, sizeof(path2), "/deliveries/%s/dest_lat.json", deliveryId);
  snprintf(path3, sizeof(path3), "/deliveries/%s/dropoff_lat.json", deliveryId);
  const char *delLatPaths[] = {path, path2, path3};
  gotLat = fetchFirebaseDoubleAny(delLatPaths, 3, dLat);
  feedCloudIoWatchdog();

  snprintf(path, sizeof(path), "/deliveries/%s/target_lng.json", deliveryId);
  snprintf(path2, sizeof(path2), "/deliveries/%s/dest_lon.json", deliveryId);
  snprintf(path3, sizeof(path3), "/deliveries/%s/dropoff_lng.json", deliveryId);
  const char *delLonPaths[] = {path, path2, path3};
  gotLon = fetchFirebaseDoubleAny(delLonPaths, 3, dLon);
  feedCloudIoWatchdog();

  if (gotLat && gotLon && (dLat != 0.0 || dLon != 0.0)) {
    applyTargetCoords(dLat, dLon, "delivery");
  } else if (!destCoordsValid) {
    Serial.printf("[GEO] coords_missing_delivery: %s dropoff target unavailable\n",
                  deliveryId);
  }

  double pLat = 0.0;
  double pLon = 0.0;
  bool gotPLat = false;
  bool gotPLon = false;

  snprintf(path, sizeof(path), "/deliveries/%s/pickup_lat.json", deliveryId);
  gotPLat = fetchFirebaseDouble(path, pLat);
  feedCloudIoWatchdog();
  snprintf(path, sizeof(path), "/deliveries/%s/pickup_lng.json", deliveryId);
  gotPLon = fetchFirebaseDouble(path, pLon);
  feedCloudIoWatchdog();

  if (gotPLat && gotPLon && (pLat != 0.0 || pLon != 0.0)) {
    applyPickupCoords(pLat, pLon, "delivery");
  }

  return destCoordsValid || pickupCoordsValid;
}

static bool phoneLocationFreshForGeo() {
  if (!phoneFixValid) return false;
  unsigned long nowMs = millis();
  if (phoneFetchedAtMs > 0 && (nowMs - phoneFetchedAtMs) <= PHONE_LOC_FETCH_INTERVAL_MS + 1000UL) {
    return true;
  }
  unsigned long nowEpoch = getCurrentEpoch();
  if (phoneTimestampMs > 0 && nowEpoch > 0) {
    unsigned long phoneEpochSec = (unsigned long)(phoneTimestampMs / 1000ULL);
    return nowEpoch >= phoneEpochSec &&
           (nowEpoch - phoneEpochSec) <= PHONE_LOC_MAX_AGE_SEC;
  }
  return false;
}

static bool markArrivedFromCoords(double lat, double lon, const char *source) {
  if (!destCoordsValid && !pickupCoordsValid) return false;

  double bestDist = 1000000000.0;
  bool inside = false;

  if (destCoordsValid) {
    double d = haversineMeters(lat, lon, destLat, destLon);
    if (d < bestDist) bestDist = d;
    if (d <= GEO_OUTER_RADIUS_M) inside = true;
  }

  if (pickupCoordsValid) {
    double d = haversineMeters(lat, lon, pickupLat, pickupLon);
    if (d < bestDist) bestDist = d;
    if (d <= GEO_OUTER_RADIUS_M) inside = true;
  }

  if (bestDist < 1000000000.0) {
    lastKnownDistMeters = (int16_t)(bestDist + 0.5);
    lastKnownInside = inside;
    lastKnownGeoAtMs = millis();
  }

  if (inside) {
    Serial.printf("[GEO] manual refresh %s inside dist=%.0fm\n",
                  source ? source : "location", bestDist);
  }
  return inside;
}

static const char *refreshGeofenceForController(bool forceCoords) {
  bool hasActiveContext = deliveryIdCacheValid && cachedDeliveryId[0] != '\0';
  bool coordsOk = destCoordsValid || pickupCoordsValid;

  if (forceCoords || (hasActiveContext && !coordsOk)) {
    if (modemHttpBusy) {
      Serial.println("[GEO] coords_fetch_busy");
      return "OK:BUSY";
    }
    refreshHardwareCoordsExact();
    coordsOk = destCoordsValid || pickupCoordsValid;
    if (hasActiveContext && !coordsOk) {
      coordsOk = refreshTargetCoordsFromDelivery(cachedDeliveryId, true);
    }
  }

  if (!coordsOk) {
    Serial.println("[GEO] coords_missing_delivery");
    return "OK:COORDS";
  }

  unsigned long nowMs = millis();
  bool gpsStale = !gpsFix || lastGps == 0 || (nowMs - lastGps) > MANUAL_GEO_STALE_MS;
  if (gpsStale && modemOK && gpsEnabled && !modemHttpBusy) {
    readGPS();
    lastGps = millis();
  } else if (gpsStale && modemHttpBusy) {
    Serial.println("[GEO] manual refresh skipped GPS - modem busy");
  }

  if (gpsFix) {
    geoProxy.update(gpsLat, gpsLon, gpsHdop, gpsSats);
    if (markArrivedFromCoords(gpsLat, gpsLon, "gps")) {
      return "OK:GEO";
    }
  }

  if (!modemHttpBusy && lteConnected) {
    lastPhoneFetchAtMs = 0;
    refreshPhoneLocationIfNeeded();
  }

  if (phoneLocationFreshForGeo() && markArrivedFromCoords(phoneLat, phoneLon, "phone")) {
    return "OK:PHONE";
  }

  if (modemHttpBusy) {
    return "OK:BUSY";
  }
  return "OK:COORDS";
}

static bool fetchFirebaseStringValue(const char *path, char *out, size_t outLen) {
  String body = httpGetFromFirebase(path);
  return parseFirebaseStringValue(body, out, outLen);
}

static bool fetchFirebaseBoolValue(const char *path, bool &out, bool &hasValue) {
  String body = httpGetFromFirebase(path);
  return parseFirebaseBoolValue(body, out, hasValue);
}

static bool httpGetSupabaseRest(const String &pathAndQuery, String &bodyOut,
                                int &httpStatus) {
  bodyOut = "";
  httpStatus = 0;
  if (!lteConnected) return false;
  ModemLockGuard modemLock("httpGetSupabaseRest", 250);
  if (!modemLock.ok()) return false;

  String resp;
  sendATAndWait("+HTTPTERM", 1000);
  delay(100);

  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000L, resp) != 1) return false;

  modem.sendAT("+HTTPPARA=\"CID\",1");
  modem.waitResponse(3000L);

  String url = String(SUPABASE_URL) + "/rest/v1/" + pathAndQuery;
  modem.sendAT(("+HTTPPARA=\"URL\",\"" + url + "\"").c_str());
  modem.waitResponse(3000L);

  String hdrs = String("apikey: ") + SUPABASE_ANON_KEY +
                "\r\nAuthorization: Bearer " + SUPABASE_ANON_KEY;
  modem.sendAT(("+HTTPPARA=\"USERDATA\",\"" + hdrs + "\"").c_str());
  modem.waitResponse(3000L);

  modem.sendAT("+CSSLCFG=\"sslversion\",1,4");
  modem.waitResponse(2000L);
  modem.sendAT("+CSSLCFG=\"ignorertctime\",1,1");
  modem.waitResponse(2000L);
  modem.sendAT("+CSSLCFG=\"authmode\",1,0");
  modem.waitResponse(2000L);
  modem.sendAT("+CSSLCFG=\"enableSNI\",1,1");
  modem.waitResponse(2000L);
  modem.sendAT("+HTTPPARA=\"SSLCFG\",1");
  modem.waitResponse(2000L);
  modem.sendAT("+HTTPSSL=1");
  modem.waitResponse(3000L);

  modem.sendAT("+HTTPACTION=0");
  unsigned long waitStart = millis();
  bool actionOK = false;
  int totalResponseLen = 0;
  while (millis() - waitStart < 15000) {
    if (apStarted) handleCameraClient();
    if (modem.stream.available()) {
      String line = modem.stream.readStringUntil('\n');
      line.trim();
      if (line.indexOf("+HTTPACTION:") >= 0) {
        int c1 = line.indexOf(',');
        int c2 = line.indexOf(',', c1 + 1);
        if (c1 > 0 && c2 > 0) {
          httpStatus = line.substring(c1 + 1, c2).toInt();
          totalResponseLen = line.substring(c2 + 1).toInt();
        }
        actionOK = true;
        break;
      }
    }
    delay(50);
    esp_task_wdt_reset();
  }

  if (actionOK && httpStatus >= 200 && httpStatus <= 299) {
    int readLen = totalResponseLen > 0 ? min(totalResponseLen, 1024) : 1024;
    char readCmd[40];
    snprintf(readCmd, sizeof(readCmd), "AT+HTTPREAD=0,%d\r\n", readLen);
    modem.stream.print(readCmd);

    int dataLen = -1;
    unsigned long t0 = millis();
    while (millis() - t0 < 6000) {
      if (apStarted) handleCameraClient();
      if (modem.stream.available()) {
        String line = modem.stream.readStringUntil('\n');
        line.trim();
        if (line.indexOf("+HTTPREAD:") >= 0) {
          int comma = line.lastIndexOf(',');
          if (comma >= 0) {
            dataLen = line.substring(comma + 1).toInt();
          } else {
            int colon = line.lastIndexOf(':');
            if (colon >= 0) dataLen = line.substring(colon + 1).toInt();
          }
          break;
        }
      }
      esp_task_wdt_reset();
    }

    int toRead = (dataLen > 0 && dataLen <= readLen) ? dataLen : readLen;
    unsigned long t1 = millis();
    while ((int)bodyOut.length() < toRead && millis() - t1 < 5000) {
      while (modem.stream.available() && (int)bodyOut.length() < toRead) {
        bodyOut += (char)modem.stream.read();
      }
      esp_task_wdt_reset();
    }

    delay(50);
    while (modem.stream.available()) modem.stream.read();
  }

  sendATAndWait("+HTTPTERM", 1000);
  return actionOK && httpStatus >= 200 && httpStatus <= 299;
}

static void saveDisabledPersonalPinForRider(const char *riderId) {
  clearPersonalPinRuntimeCache(true);
  if (!riderId || riderId[0] == '\0') return;

  strncpy(cachedPersonalPinRiderId, riderId, sizeof(cachedPersonalPinRiderId) - 1);
  cachedPersonalPinRiderId[sizeof(cachedPersonalPinRiderId) - 1] = '\0';

  uint32_t nextPinHash =
      buildPersonalPinContextHash(false, "", "", cachedPersonalPinRiderId);
  if (!persistedPinContextHashReady || nextPinHash != lastPersistedPinContextHash) {
    dpSavePersonalPinRiderId(cachedPersonalPinRiderId);
    lastPersistedPinContextHash = nextPinHash;
    persistedPinContextHashReady = true;
  }
}

static void patchPersonalPinRuntimeToHardware(bool enabled,
                                              const char *riderId,
                                              const char *hashHex,
                                              const char *salt) {
  if (!lteConnected) return;

  char path[64];
  snprintf(path, sizeof(path), "/hardware/%s.json", HARDWARE_ID);

  char json[360];
  unsigned long updatedAt = getCurrentEpoch();
  if (updatedAt == 0) updatedAt = millis() / 1000UL;

  if (enabled && riderId && riderId[0] != '\0' && hashHex && hashHex[0] != '\0' &&
      salt && salt[0] != '\0') {
    snprintf(json, sizeof(json),
             "{\"personal_pin_enabled\":true,"
             "\"personal_pin_hash_mcu\":\"%s\","
             "\"personal_pin_salt\":\"%s\","
             "\"current_rider_id\":\"%s\","
             "\"personal_pin_updated_at\":%lu}",
             hashHex, salt, riderId, updatedAt);
  } else if (riderId && riderId[0] != '\0') {
    snprintf(json, sizeof(json),
             "{\"personal_pin_enabled\":false,"
             "\"personal_pin_hash_mcu\":null,"
             "\"personal_pin_salt\":null,"
             "\"current_rider_id\":\"%s\","
             "\"personal_pin_updated_at\":%lu}",
             riderId, updatedAt);
  } else {
    snprintf(json, sizeof(json),
             "{\"personal_pin_enabled\":false,"
             "\"personal_pin_hash_mcu\":null,"
             "\"personal_pin_salt\":null,"
             "\"current_rider_id\":null,"
             "\"personal_pin_updated_at\":%lu}",
             updatedAt);
  }

  httpPatchWithRetry(path, json);
}

static bool fetchActivePairingRiderFromFirebase(char *riderOut, size_t riderLen,
                                                bool &hasActivePairing) {
  if (!riderOut || riderLen == 0) return false;
  riderOut[0] = '\0';
  hasActivePairing = false;
  if (!lteConnected) return false;

  char path[96];
  snprintf(path, sizeof(path), "/pairings/%s.json", HARDWARE_ID);
  String body = httpGetFromFirebase(path);
  body.trim();
  if (body.length() == 0) return false;
  if (body == "null") return true;

  char status[16] = "";
  if (!readTopLevelJsonString(body, "status", status, sizeof(status))) {
    return true;
  }

  if (strcmp(status, "ACTIVE") != 0 && strcmp(status, "active") != 0) {
    return true;
  }

  if (!readTopLevelJsonString(body, "rider_id", riderOut, riderLen) ||
      riderOut[0] == '\0') {
    return true;
  }

  hasActivePairing = true;
  return true;
}

static int8_t refreshPersonalPinRuntimeFromSupabaseProfile(const char *riderId,
                                                           bool logDetails) {
  if (!riderId || riderId[0] == '\0') return -1;

  String body;
  int httpStatus = 0;
  String query = "profiles?id=eq." + String(riderId) +
                 "&select=personal_pin_enabled%2Cpersonal_pin_hash_mcu%2Cpersonal_pin_salt";
  if (!httpGetSupabaseRest(query, body, httpStatus)) {
    if (logDetails) {
      Serial.printf("[PIN] Supabase profile refresh failed HTTP %d\n", httpStatus);
    }
    return -1;
  }

  body.trim();
  if (body.length() == 0 || body == "[]") {
    if (logDetails) {
      Serial.println("[PIN] Supabase profile has no Personal PIN row");
    }
    return 0;
  }

  int objStart = body.indexOf('{');
  int objEnd = body.lastIndexOf('}');
  if (objStart < 0 || objEnd <= objStart) {
    if (logDetails) {
      Serial.println("[PIN] Supabase profile response malformed");
    }
    return -1;
  }
  String profileJson = body.substring(objStart, objEnd + 1);

  bool enabledVal = false;
  bool parsedEnabled =
      readTopLevelJsonBool(profileJson, "personal_pin_enabled", enabledVal);
  if (!parsedEnabled || !enabledVal) {
    return 0;
  }

  char hash[65] = "";
  char salt[33] = "";
  bool hashOk = readTopLevelJsonString(profileJson, "personal_pin_hash_mcu",
                                       hash, sizeof(hash));
  bool saltOk = readTopLevelJsonString(profileJson, "personal_pin_salt",
                                       salt, sizeof(salt));
  if (!hashOk || !saltOk || strlen(hash) != 64 || strlen(salt) == 0) {
    if (logDetails) {
      Serial.println("[PIN] Supabase profile PIN fields incomplete");
    }
    return 0;
  }

  bool runtimeChanged =
      !personalPinEnabled ||
      strcmp(cachedPersonalPinHashMcu, hash) != 0 ||
      strcmp(cachedPersonalPinSalt, salt) != 0 ||
      strcmp(cachedPersonalPinRiderId, riderId) != 0;

  personalPinEnabled = true;
  strncpy(cachedPersonalPinHashMcu, hash, sizeof(cachedPersonalPinHashMcu) - 1);
  cachedPersonalPinHashMcu[sizeof(cachedPersonalPinHashMcu) - 1] = '\0';
  strncpy(cachedPersonalPinSalt, salt, sizeof(cachedPersonalPinSalt) - 1);
  cachedPersonalPinSalt[sizeof(cachedPersonalPinSalt) - 1] = '\0';
  strncpy(cachedPersonalPinRiderId, riderId, sizeof(cachedPersonalPinRiderId) - 1);
  cachedPersonalPinRiderId[sizeof(cachedPersonalPinRiderId) - 1] = '\0';

  uint32_t nextPinHash =
      buildPersonalPinContextHash(true,
                                  cachedPersonalPinHashMcu,
                                  cachedPersonalPinSalt,
                                  cachedPersonalPinRiderId);
  if (!persistedPinContextHashReady || nextPinHash != lastPersistedPinContextHash) {
    dpSavePersonalPinEnabled(true);
    dpSavePersonalPinHash(cachedPersonalPinHashMcu);
    dpSavePersonalPinSalt(cachedPersonalPinSalt);
    dpSavePersonalPinRiderId(cachedPersonalPinRiderId);
    lastPersistedPinContextHash = nextPinHash;
    persistedPinContextHashReady = true;
  }

  if (runtimeChanged) {
    patchPersonalPinRuntimeToHardware(true,
                                      cachedPersonalPinRiderId,
                                      cachedPersonalPinHashMcu,
                                      cachedPersonalPinSalt);
  }
  if (logDetails) {
    Serial.printf("[PIN] Supabase profile runtime OK (rider=%s)\n",
                  cachedPersonalPinRiderId);
  }
  return 1;
}

static bool syncPersonalPinRuntimeWithActivePairing(bool logDetails, bool force) {
  if (!lteConnected) return false;

  unsigned long nowMs = millis();
  if (!force && nowMs - lastPairingPinSyncAt < PIN_PAIRING_SYNC_MIN_MS) {
    return true;
  }
  lastPairingPinSyncAt = nowMs;

  char activeRider[64] = "";
  bool hasActivePairing = false;
  if (!fetchActivePairingRiderFromFirebase(activeRider, sizeof(activeRider),
                                           hasActivePairing)) {
    if (logDetails) {
      Serial.println("[PIN] Pairing rider lookup failed");
    }
    return false;
  }

  if (!hasActivePairing) {
    bool cachedRuntimeOk =
        personalPinEnabled &&
        cachedPersonalPinHashMcu[0] != '\0' &&
        cachedPersonalPinSalt[0] != '\0' &&
        cachedPersonalPinRiderId[0] != '\0';

    if (!cachedRuntimeOk) {
      cachedRuntimeOk =
          refreshPersonalPinRuntimeFromFirebase(logDetails, true) &&
          personalPinEnabled &&
          cachedPersonalPinHashMcu[0] != '\0' &&
          cachedPersonalPinSalt[0] != '\0' &&
          cachedPersonalPinRiderId[0] != '\0';
    }

    if (cachedRuntimeOk) {
      if (logDetails) {
        Serial.printf("[PIN] No active pairing node; using stored hardware PIN runtime (rider=%s)\n",
                      cachedPersonalPinRiderId);
      }
      return true;
    }

    if (logDetails) {
      Serial.println("[PIN] No active pairing and no stored Personal PIN runtime");
    }
    return true;
  }

  bool cacheMatchesPairing =
      personalPinEnabled &&
      cachedPersonalPinHashMcu[0] != '\0' &&
      cachedPersonalPinSalt[0] != '\0' &&
      cachedPersonalPinRiderId[0] != '\0' &&
      strcmp(cachedPersonalPinRiderId, activeRider) == 0;
  if (cacheMatchesPairing) {
    return true;
  }

  if (refreshPersonalPinRuntimeFromFirebase(logDetails, true) &&
      personalPinEnabled &&
      cachedPersonalPinHashMcu[0] != '\0' &&
      cachedPersonalPinSalt[0] != '\0' &&
      cachedPersonalPinRiderId[0] != '\0' &&
      strcmp(cachedPersonalPinRiderId, activeRider) == 0) {
    return true;
  }

  if (force) {
    int8_t profileResult =
        refreshPersonalPinRuntimeFromSupabaseProfile(activeRider, logDetails);
    if (profileResult > 0) {
      return true;
    }
    if (profileResult == 0) {
      saveDisabledPersonalPinForRider(activeRider);
      patchPersonalPinRuntimeToHardware(false, activeRider, "", "");
      if (logDetails) {
        Serial.printf("[PIN] Active rider has no enabled Personal PIN (%s)\n",
                      activeRider);
      }
      return true;
    }
    return false;
  }

  if (cachedPersonalPinRiderId[0] != '\0' &&
      strcmp(cachedPersonalPinRiderId, activeRider) != 0) {
    Serial.printf("[PIN] Hardware PIN rider stale (%s != active %s); replacing\n",
                  cachedPersonalPinRiderId, activeRider);
  }

  int8_t profileResult =
      refreshPersonalPinRuntimeFromSupabaseProfile(activeRider, logDetails);
  if (profileResult > 0) {
    return true;
  }

  if (profileResult == 0) {
    saveDisabledPersonalPinForRider(activeRider);
    patchPersonalPinRuntimeToHardware(false, activeRider, "", "");
    if (logDetails) {
      Serial.printf("[PIN] Active rider has no enabled Personal PIN (%s)\n",
                    activeRider);
    }
    return true;
  }

  return false;
}

static bool refreshPersonalPinRuntimeFromFirebase(bool logDetails, bool force) {
  if (!lteConnected) return false;

  unsigned long nowMs = millis();
  if (!force && nowMs - lastPinRuntimeRefreshAt < PIN_RUNTIME_REFRESH_MIN_MS) {
    return false;
  }
  lastPinRuntimeRefreshAt = nowMs;

  char path[96];
  bool enabledVal = false;
  bool hasEnabled = false;
  snprintf(path, sizeof(path), "/hardware/%s/personal_pin_enabled.json", HARDWARE_ID);
  if (!fetchFirebaseBoolValue(path, enabledVal, hasEnabled) || !hasEnabled) {
    if (logDetails) {
      Serial.println("[PIN] Runtime refresh failed: missing enabled flag");
    }
    return false;
  }

  if (!enabledVal) {
    clearPersonalPinRuntimeCache(true);
    if (logDetails) {
      Serial.println("[PIN] Runtime refresh: personal PIN disabled");
    }
    return true;
  }

  char hash[65] = "";
  char salt[33] = "";
  char rider[64] = "";
  snprintf(path, sizeof(path), "/hardware/%s/personal_pin_hash_mcu.json", HARDWARE_ID);
  bool hashOk = fetchFirebaseStringValue(path, hash, sizeof(hash));
  snprintf(path, sizeof(path), "/hardware/%s/personal_pin_salt.json", HARDWARE_ID);
  bool saltOk = fetchFirebaseStringValue(path, salt, sizeof(salt));
  snprintf(path, sizeof(path), "/hardware/%s/current_rider_id.json", HARDWARE_ID);
  bool riderOk = fetchFirebaseStringValue(path, rider, sizeof(rider));

  if (!hashOk || !saltOk || !riderOk || strlen(hash) != 64 ||
      strlen(salt) == 0 || strlen(rider) == 0) {
    if (logDetails) {
      Serial.printf("[PIN] Runtime refresh failed: hash=%d salt=%d rider=%d\n",
                    hashOk ? 1 : 0, saltOk ? 1 : 0, riderOk ? 1 : 0);
    }
    return false;
  }

  bool riderChanged = cachedPersonalPinRiderId[0] != '\0' &&
                      strcmp(cachedPersonalPinRiderId, rider) != 0;
  if (riderChanged && logDetails) {
    Serial.printf("[PIN] Runtime refresh rider changed (%s -> %s)\n",
                  cachedPersonalPinRiderId, rider);
  }

  personalPinEnabled = true;
  strncpy(cachedPersonalPinHashMcu, hash, sizeof(cachedPersonalPinHashMcu) - 1);
  cachedPersonalPinHashMcu[sizeof(cachedPersonalPinHashMcu) - 1] = '\0';
  strncpy(cachedPersonalPinSalt, salt, sizeof(cachedPersonalPinSalt) - 1);
  cachedPersonalPinSalt[sizeof(cachedPersonalPinSalt) - 1] = '\0';
  strncpy(cachedPersonalPinRiderId, rider, sizeof(cachedPersonalPinRiderId) - 1);
  cachedPersonalPinRiderId[sizeof(cachedPersonalPinRiderId) - 1] = '\0';

  uint32_t nextPinHash =
      buildPersonalPinContextHash(true,
                                  cachedPersonalPinHashMcu,
                                  cachedPersonalPinSalt,
                                  cachedPersonalPinRiderId);
  if (!persistedPinContextHashReady || nextPinHash != lastPersistedPinContextHash) {
    dpSavePersonalPinEnabled(true);
    dpSavePersonalPinHash(cachedPersonalPinHashMcu);
    dpSavePersonalPinSalt(cachedPersonalPinSalt);
    dpSavePersonalPinRiderId(cachedPersonalPinRiderId);
    lastPersistedPinContextHash = nextPinHash;
    persistedPinContextHashReady = true;
  }

  if (logDetails) {
    Serial.printf("[PIN] Runtime refresh OK (rider=%s)\n",
                  cachedPersonalPinRiderId);
  }
  return true;
}

// Register this board's MAC address in Firebase and return its box number.
// On first call for a new MAC: claims the next sequential number from a
// counter at /box_registry/next_id. If the MAC was already registered
// (e.g. board was reflashed), returns the previously assigned number.
uint8_t registerWithFirebase(const char *macHex) {
  Serial.printf("[PROV] Checking Firebase registry for MAC %s...\n", macHex);
  esp_task_wdt_reset();

  char regPath[64];
  snprintf(regPath, sizeof(regPath), "/box_registry/%s.json", macHex);
  String body = httpGetFromFirebase(regPath);
  body.trim();
  Serial.printf("[PROV] Registry lookup: '%s'\n", body.c_str());

  if (body.length() > 0 && body != "null") {
    int numIdx = body.indexOf("\"num\":");
    if (numIdx >= 0) {
      int val = body.substring(numIdx + 6).toInt();
      if (val > 0 && val < 256) {
        Serial.printf("[PROV] Already registered as box %d\n", val);
        return (uint8_t)val;
      }
    }
  }

  Serial.println("[PROV] Not registered, claiming next ID...");
  esp_task_wdt_reset();

  String counterBody = httpGetFromFirebase("/box_registry/next_id.json");
  counterBody.trim();
  Serial.printf("[PROV] Current counter: '%s'\n", counterBody.c_str());

  uint8_t nextNum = 1;
  if (counterBody.length() > 0 && counterBody != "null") {
    int val = counterBody.toInt();
    if (val > 0)
      nextNum = (uint8_t)val;
  }

  // Register MAC mapping first, then bump counter
  char regJson[32];
  snprintf(regJson, sizeof(regJson), "{\"num\":%u}", nextNum);
  httpPutToFirebase(regPath, String(regJson));
  esp_task_wdt_reset();

  char counterJson[8];
  snprintf(counterJson, sizeof(counterJson), "%u", nextNum + 1);
  httpPutToFirebase("/box_registry/next_id.json", String(counterJson));

  Serial.printf("[PROV] Registered MAC %s as box %u\n", macHex, nextNum);
  return nextNum;
}

// Read box number from NVS (fast) or auto-register with Firebase (first boot).
// Populates HARDWARE_ID, AP_SSID, and CAM_PREFIX globals.
void autoProvision() {
  Preferences prefs;
  prefs.begin("smartbox", false);

#ifdef PROXY_BOX_NUM
  boxNum = (uint8_t)PROXY_BOX_NUM;
  prefs.putUChar("box_num", boxNum);
  Serial.printf("[PROV] PROXY_BOX_NUM override: box %u\n", boxNum);
#else

  // ── ONE-TIME NVS WIPE: Uncomment the #define to force re-provisioning ──
// #define FORCE_REPROVISION
#ifdef FORCE_REPROVISION
  prefs.remove("box_num");
  Serial.println("[PROV] FORCE_REPROVISION: cleared NVS cache");
#endif

  boxNum = prefs.getUChar("box_num", 0);

  if (boxNum > 0) {
    Serial.printf("[PROV] Cached box number: %u\n", boxNum);
  } else {
    uint64_t chipId = ESP.getEfuseMac();
    uint8_t *mac = (uint8_t *)&chipId;
    char macHex[13];
    snprintf(macHex, sizeof(macHex), "%02X%02X%02X%02X%02X%02X", mac[0],
             mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.printf("[PROV] First boot — MAC: %s\n", macHex);

    boxNum = registerWithFirebase(macHex);
    prefs.putUChar("box_num", boxNum);
    Serial.printf("[PROV] Saved box %u to NVS\n", boxNum);
  }
#endif
  prefs.end();

  snprintf(HARDWARE_ID, sizeof(HARDWARE_ID), "BOX_%03u", boxNum);
  snprintf(AP_SSID, sizeof(AP_SSID), "SmartTopBox_AP_%03u", boxNum);
  snprintf(CAM_PREFIX, sizeof(CAM_PREFIX), "OV3660_CAM_%03u", boxNum);

  Serial.printf("[PROV] === Identity: %s | AP: %s | CAM: %s ===\n",
                HARDWARE_ID, AP_SSID, CAM_PREFIX);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  stateMutex = xSemaphoreCreateMutex();
  modemMutex = xSemaphoreCreateRecursiveMutex();
  delay(1000);
  Serial.println("\n=== GPS/LTE Firebase Test (LTE Only - AT+HTTP) ===\n");

  // â”€â”€ Initialize Watchdog Timer FIRST (before any modem ops) â”€â”€
  // Core 2.x API: esp_task_wdt_init(timeout_seconds, panic_on_timeout)
  esp_task_wdt_init(WDT_TIMEOUT_S, true); // true = reboot on timeout
  smartBoxLoopTaskHandle = xTaskGetCurrentTaskHandle();
  esp_task_wdt_add(NULL);                 // Watch the main loop task
  Serial.printf("[WDT] Watchdog armed (%ds timeout)\n", WDT_TIMEOUT_S);
  Serial.printf("[BUILD] Proxy12 WDT90 loop-guard build %s %s\n", __DATE__,
                __TIME__);

  // Initialize modem with proper power sequence
  modemSerial.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
#if !DISABLE_PROXY_UART
  beginProxyUart();
#else
  Serial.println("[PROXY-UART] Disabled (WiFi-first)");
#endif
  powerModem();

  modemOK = initModem();

  if (!modemOK) {
    Serial.println("\n*** MODEM FAILED - will retry in loop() ***");
    Serial.println("  CHECK: 1) Modem hardware connected properly");
    Serial.println("         2) Power supply sufficient (>2A)");
    Serial.println("         3) SIM card inserted");
  }

  gpsEnabled = initGPS();
  Serial.println(gpsEnabled ? "GPS: Ready" : "GPS: Failed (continuing)");

  // ── Connect LTE FIRST (Firebase provisioning needs HTTP) ──
  startHotspot();

  Serial.println("\nPreparing LTE fallback...");
  if (!wifiUplinkConnected && !connectLTE()) {
    Serial.println("\n*** LTE CONNECTION FAILED - will retry in loop() ***");
    Serial.println("  CHECK: 1) SIM has active data plan");
    Serial.println("         2) LTE antenna connected");
    Serial.println("         3) Signal coverage in area");
  }

  // Auto-provision box identity (NVS cache or Firebase registration)
  // MUST be after at least one cloud transport is connected on first boot.
  autoProvision();

  // ── Start WiFi hotspot so ESP32-CAM can connect and relay images ──
  // (after provisioning so AP_SSID has the correct box number)
  if (apStarted) {
    Serial.printf("[WIFI] Camera/controller proxy server listening on port %d\n",
                  CAM_SERVER_PORT);
    startLogServer();
  }

  // â”€â”€ POST-LTE: Retry AGPS + XTRA now that data is available â”€â”€
  // These failed during GPS init because LTE wasn't connected yet
  if (gpsEnabled) {
    Serial.println("\nRetrying GPS assistance with LTE data...");
    ModemLockGuard modemLock("setupGpsAssist", 1000);
    if (!modemLock.ok()) {
      Serial.println("  GPS assistance skipped - modem busy");
    } else {

      String resp;
      Serial.print("  AGPS download...");
      modem.sendAT("+CAGPS");
      if (modem.waitResponse(15000L, resp) == 1) {
        Serial.println(" OK (satellite data injected!)");
      } else {
        Serial.println(" skip");
      }

      Serial.print("  XTRA ephemeris download...");
      modem.sendAT("+CGPSXD=0"); // Trigger immediate XTRA download
      if (modem.waitResponse(10000L, resp) == 1) {
        Serial.println(" OK");
      } else {
        Serial.println(" skip (auto-download enabled)");
      }
    }
  }

  // â”€â”€ Sync network time (Philippine Standard Time, UTC+8) â”€â”€
  Serial.println("\nSyncing clock to Philippine Standard Time...");
  if (syncNetworkTime()) {
    Serial.println("Clock: " + formatTimestamp(getCurrentEpoch()));
  } else {
    Serial.println("WARNING: Clock not synced â€” timestamps may be incorrect");
  }

  // ── Initialize ported modules ──
  batteryBegin();
  theftGuardInit();
  geoProxy.reset();
  reassign.clear();
  dpBegin();
  deliveryContextBootAtMs = millis();
  deliveryContextValidated = false;
  contextFetchFailCount = 0;
  lastDeliveryIdSeenAtMs = 0;
  lastContextFetchOkAtMs = 0;

  // Restore persisted delivery context (survives brown-out / reboot)
  char restoredOtp[8] = "";
  char restoredDeliveryId[64] = "";
  if (dpLoadDeliveryContext(restoredOtp, sizeof(restoredOtp),
                            restoredDeliveryId, sizeof(restoredDeliveryId))) {
    if (restoredOtp[0] != '\0') {
      strncpy(cachedOtp, restoredOtp, sizeof(cachedOtp) - 1);
      cachedOtp[sizeof(cachedOtp) - 1] = '\0';
      otpCacheValid = true;
      Serial.printf("[DP] Restored OTP from NVS: %s\n", cachedOtp);
    }
    if (restoredDeliveryId[0] != '\0') {
      strncpy(cachedDeliveryId, restoredDeliveryId, sizeof(cachedDeliveryId) - 1);
      cachedDeliveryId[sizeof(cachedDeliveryId) - 1] = '\0';
      deliveryIdCacheValid = true;
      deliveryContextStale = true;
      controllerContextRefreshQueued = true;
      controllerContextRefreshQueuedAt = millis();
      strncpy(lastRefreshStatus, "OK:CACHED_STALE", sizeof(lastRefreshStatus) - 1);
      lastRefreshStatus[sizeof(lastRefreshStatus) - 1] = '\0';
      Serial.printf("[DP] Restored DeliveryID from NVS: %s\n", cachedDeliveryId);
    }
  }
  restorePersistedCommandState();
  if (dpLoadPersonalPinHash(cachedPersonalPinHashMcu,
                            sizeof(cachedPersonalPinHashMcu))) {
    Serial.println("[DP] Restored personal PIN hash from NVS");
  }
  if (dpLoadPersonalPinSalt(cachedPersonalPinSalt, sizeof(cachedPersonalPinSalt))) {
    Serial.println("[DP] Restored personal PIN salt from NVS");
  }
  if (dpLoadPersonalPinRiderId(cachedPersonalPinRiderId,
                               sizeof(cachedPersonalPinRiderId))) {
    Serial.printf("[DP] Restored personal PIN rider binding: %s\n",
                  cachedPersonalPinRiderId);
  }
  personalPinEnabled = dpLoadPersonalPinEnabled();
  if (personalPinEnabled &&
      (cachedPersonalPinHashMcu[0] == '\0' || cachedPersonalPinSalt[0] == '\0' ||
       cachedPersonalPinRiderId[0] == '\0')) {
    Serial.println("[PIN] Incomplete persisted Personal PIN cache; forcing disable");
    clearPersonalPinRuntimeCache(true);
  }

  if (lteConnected) {
    syncPersonalPinRuntimeWithActivePairing(true, true);
  }

  lastPersistedContextHash =
      buildDeliveryContextHash(otpCacheValid ? cachedOtp : "",
                               deliveryIdCacheValid ? cachedDeliveryId : "");
  persistedContextHashReady = true;
  lastPersistedPinContextHash =
      buildPersonalPinContextHash(personalPinEnabled,
                                  cachedPersonalPinHashMcu,
                                  cachedPersonalPinSalt,
                                  cachedPersonalPinRiderId);
  persistedPinContextHashReady = true;
  dpGetWriteMetrics(&lastDpMetricsSnapshot);

  if (modemRecoveryTaskHandle == NULL) {
    xTaskCreatePinnedToCore(modemRecoveryTask,
                            "ModemRecovery",
                            8192,
                            NULL,
                            1,
                            &modemRecoveryTaskHandle,
                            0);
  }

  Serial.println("\n=== Ready (LTE Only - AT+HTTP) ===\n");

  printMemoryReport();
  xTaskCreatePinnedToCore(firebaseSyncTask, "FirebaseSync", 16384, NULL, 1, &firebaseSyncTaskHandle, 0);
}

// ==================== FIREBASE SYNC TASK (CORE 0) ====================
void firebaseSyncTask(void *pvParameters) {
  Serial.println("[WDT] FirebaseSync not enrolled; network waits must not reboot proxy");
  for (;;) {
    feedCloudIoWatchdog();
    unsigned long now = millis();

    bool hasActiveDelivery = deliveryIdCacheValid && cachedDeliveryId[0] != '\0';
    bool idlePowerMode = !hasActiveDelivery;
    unsigned long firebaseInterval =
        idlePowerMode ? (FIREBASE_INTERVAL * IDLE_POLL_MULTIPLIER)
                      : FIREBASE_INTERVAL;
    unsigned long deliveryReadInterval = DELIVERY_CONTEXT_READ_INTERVAL;
    if (manualRefreshBurstUntil > now) {
      deliveryReadInterval = MANUAL_REFRESH_READ_INTERVAL_MS;
    }

    if ((wifiCloudHealthy || lteConnected) && controllerContextRefreshQueued && !modemHttpBusy) {
      controllerContextRefreshQueued = false;
      lastDeliveryContextRead = 0;
      strncpy(lastRefreshStatus, "OK:REFRESHING", sizeof(lastRefreshStatus) - 1);
      lastRefreshStatus[sizeof(lastRefreshStatus) - 1] = '\0';
      Serial.println("[CONTEXT] Running queued controller refresh");
    }

    if (lteConnected && commandAckFirebaseWriteQueued && !modemHttpBusy) {
      char cmd[sizeof(queuedCommandAckCommand)];
      char status[sizeof(queuedCommandAckStatus)];
      char details[sizeof(queuedCommandAckDetails)];
      strncpy(cmd, queuedCommandAckCommand, sizeof(cmd) - 1);
      cmd[sizeof(cmd) - 1] = '\0';
      strncpy(status, queuedCommandAckStatus, sizeof(status) - 1);
      status[sizeof(status) - 1] = '\0';
      strncpy(details, queuedCommandAckDetails, sizeof(details) - 1);
      details[sizeof(details) - 1] = '\0';
      bool tracked = queuedCommandAckTracked;
      commandAckFirebaseWriteQueued = false;
      bool ackWritten = writeCommandAckToFirebase(cmd, status, details);
      feedCloudIoWatchdog();
      if (tracked) {
        if (ackWritten) markCommandDone();
        else { commandAckFirebaseWriteQueued = true; lastCommandAckRetryAt = now + COMMAND_ACK_RETRY_INTERVAL_MS; }
      } else if (!ackWritten) { commandAckFirebaseWriteQueued = true; }
    }

    if ((wifiCloudHealthy || lteConnected) && returnCompletionFirebaseWriteQueued && !modemHttpBusy) {
      returnCompletionFirebaseWriteQueued = false;
      char patchPath[64];
      snprintf(patchPath, sizeof(patchPath), "/hardware/%s.json", HARDWARE_ID);
      const char *patchBody = "{\"return_completed\":true,\"return_active\":false,\"return_completed_at\":{\".sv\":\"timestamp\"}}";
      if (!httpPatchWithRetry(patchPath, patchBody)) {
        returnCompletionFirebaseWriteQueued = true;
      }
      feedCloudIoWatchdog();
    }

    if ((wifiCloudHealthy || lteConnected) && tamperFirebaseWriteQueued && !modemHttpBusy) {
      tamperFirebaseWriteQueued = false;
      writeTamperToFirebase();
      feedCloudIoWatchdog();
    }

    if ((wifiCloudHealthy || lteConnected) && lockEventFirebaseWriteQueued && !modemHttpBusy) {
      bool ov = queuedLockOtpValid;
      bool fd = queuedLockFaceDetected;
      bool ul = queuedLockUnlocked;
      int attempts = queuedLockFaceAttempts;
      bool retryExhausted = queuedLockFaceRetryExhausted;
      bool fallbackRequired = queuedLockFallbackRequired;
      char reason[sizeof(queuedLockFailureReason)];
      strncpy(reason, queuedLockFailureReason, sizeof(reason) - 1);
      reason[sizeof(reason) - 1] = '\0';
      unsigned long latencyMs = queuedLockUnlockLatencyMs;
      lockEventFirebaseWriteQueued = false;
      writeLockEventToFirebase(ov, fd, ul, attempts, retryExhausted, fallbackRequired, reason, latencyMs);
      feedCloudIoWatchdog();
    }

    if ((wifiCloudHealthy || lteConnected) && now - lastDeliveryContextRead >= deliveryReadInterval) {
      refreshDeliveryContextFromFirebase();
      feedCloudIoWatchdog();
      lastDeliveryContextRead = now;
    }

    if ((wifiCloudHealthy || lteConnected) && personalPinRuntimeRefreshQueued &&
        !modemHttpBusy && now - lastPinRuntimeRefreshAt >= PIN_RUNTIME_REFRESH_MIN_MS) {
      bool force = personalPinRuntimeRefreshForce;
      personalPinRuntimeRefreshQueued = false;
      personalPinRuntimeRefreshForce = false;
      lastPinRuntimeRefreshAt = now;
      Serial.println("[PIN] Running queued runtime refresh");
      bool ok = lteConnected
                    ? syncPersonalPinRuntimeWithActivePairing(true, force)
                    : refreshPersonalPinRuntimeFromFirebase(true, force);
      if (!ok) {
        personalPinRuntimeRefreshQueued = true;
        personalPinRuntimeRefreshForce = force;
        Serial.println("[PIN] Queued runtime refresh failed; will retry");
      }
      feedCloudIoWatchdog();
    }

    if ((wifiCloudHealthy || lteConnected) && now - lastFB >= firebaseInterval) {
      sendToFirebase();
      feedCloudIoWatchdog();
      lastFB = now;
    }

    if ((wifiCloudHealthy || lteConnected) && now - lastTamperCheckAt >= TAMPER_CHECK_INTERVAL) {
      checkAndClearTamperFromFirebase();
      feedCloudIoWatchdog();
      lastTamperCheckAt = now;
    }

    if ((wifiCloudHealthy || lteConnected) && now - lastPhotoBurstCheckAt >= PHOTO_BURST_CHECK_INTERVAL) {
      checkAndRunPhotoBurstFromFirebase(now);
      feedCloudIoWatchdog();
      lastPhotoBurstCheckAt = now;
    }

    feedCloudIoWatchdog();
    vTaskDelay(pdMS_TO_TICKS(100)); // Yield to other Core 0 tasks
  }
}

// ==================== LOOP ====================
void loop() {
  esp_task_wdt_reset(); // Feed watchdog every loop iteration
  unsigned long now = millis();
  maintainHotspotStation(now);
  feedCloudIoWatchdog();
  pollWifiCloudHealth(now);
  feedCloudIoWatchdog();
  pollDiscoveryServer();
  feedCloudIoWatchdog();
  probeCameraIfDue(now);
  feedCloudIoWatchdog();
#if !DISABLE_PROXY_UART
  pollProxyUart();
  feedCloudIoWatchdog();
#endif
  enforceTamperSuppressionTimeout(now);
  maybeEvaluateConnectivityMatrix(now);
  feedCloudIoWatchdog();

  // ── Modem/LTE retry with exponential backoff (keeps HTTP server alive) ──
  if ((!modemOK || !lteConnected) && now >= modemRetryAt) {
    if (modemRecoveryState == MODEM_RECOVERY_IDLE) {
      scheduleModemRecovery("loop_backoff");
    } else {
      modemRetryAt = now + modemRetryBackoffMs;
    }
  }

    bool hasActiveDelivery = deliveryIdCacheValid && cachedDeliveryId[0] != '\0';
    bool idlePowerMode = !hasActiveDelivery;
    unsigned long signalInterval =
      idlePowerMode ? (SIGNAL_INTERVAL * IDLE_POLL_MULTIPLIER) : SIGNAL_INTERVAL;
    unsigned long firebaseInterval = idlePowerMode
                       ? (FIREBASE_INTERVAL * IDLE_POLL_MULTIPLIER)
                       : FIREBASE_INTERVAL;
    // Context reads always run at full speed — new delivery detection must be fast
    // even when idle. IDLE_POLL_MULTIPLIER only slows sendToFirebase + signal checks.
    unsigned long deliveryReadInterval = DELIVERY_CONTEXT_READ_INTERVAL;
    if (manualRefreshBurstUntil > now) {
      deliveryReadInterval = MANUAL_REFRESH_READ_INTERVAL_MS;
    }

  // Handle incoming camera image upload (non-blocking when idle)
  if (apStarted) {
    handleCameraClient();
    pollLogServer();

    // Check for incoming UDP logs
    int packetSize = udpServer.parsePacket();
    if (packetSize) {
      char udpBuf[512] = "";
      int len = udpServer.read(udpBuf, sizeof(udpBuf) - 1);
      if (len > 0) {
        udpBuf[len] = '\0';
        Serial.printf("%s\n",
                      udpBuf); // String already contains prefix [TESTER]/[CAM]
      }
    }
  }

  triggerRebootAllIfScheduled(now);

  // Check LTE connection periodically
  if (now - lastSig >= signalInterval) {
    if (lteConnected && modemOK) {
      ModemLockGuard modemLock("loopSignalCheck", 100);
      if (modemLock.ok()) {
        cachedSignalCsq = modem.getSignalQuality();
        cachedSignalRssi = (cachedSignalCsq == 99 || cachedSignalCsq == 0)
                               ? -999
                               : -113 + (2 * cachedSignalCsq);
        if (!modem.isGprsConnected()) {
          Serial.println("LTE: Connection lost, reconnecting...");
          lteConnected = false;
          scheduleModemRecovery("gprs_lost");
        }
      }
    }
    lastSig = now;
  }

  // Modem health check (lightweight AT heartbeat)
  if (modemOK && !modemHttpBusy && now - lastModemHealth >= MODEM_HEALTH_INTERVAL) {
    checkModemHealth();
    lastModemHealth = now;
  }
  feedCloudIoWatchdog();

  // Periodic memory report
  if (now - lastMemReport >= MEM_REPORT_INTERVAL) {
    printMemoryReport();
    lastMemReport = now;
  }

  // Read GPS (poll faster while acquiring fix, slower once locked)
  unsigned long gpsInterval = gpsFix ? GPS_INTERVAL_LOCKED : GPS_INTERVAL_ACQUIRING;
  if (idlePowerMode) {
    gpsInterval *= IDLE_GPS_POLL_MULTIPLIER;
  }
  if (modemOK && gpsEnabled && !modemHttpBusy && now - lastGps >= gpsInterval) {
    readGPS();
    lastGps = now;

    // Show GPS status periodically
    static unsigned long lastGpsStatus = 0;
    if (now - lastGpsStatus >= 10000) { // Every 10 seconds
      if (gpsFix) {
        Serial.printf("GPS: %.6f, %.6f\n", gpsLat, gpsLon);
      } else {
        Serial.println("GPS: No fix yet (needs outdoor + clear sky)");
      }
      lastGpsStatus = now;
    }
  } else if (modemOK && gpsEnabled && modemHttpBusy && now - lastGps >= gpsInterval) {
    lastGps = now;
  }
  feedCloudIoWatchdog();

  // ── Battery monitor (10 s interval) — dual channel ──
  if (now - lastBatteryRead >= BATTERY_READ_INTERVAL) {
    batteryUpdate();
    lastBatteryRead = now;

    // Log both channels periodically (every 60s)
    static unsigned long lastBattLog = 0;
    if (now - lastBattLog >= 60000) {
      Serial.printf("[BATT] CH-A(7.5V): %.2fV %d%% pin=%dmV | CH-B(12V): %.2fV %d%% pin=%dmV\n",
                    batteryGetVoltage(BATT_CH_A), batteryGetPercentage(BATT_CH_A),
                    batteryGetPinMilliVolts(BATT_CH_A),
                    batteryGetVoltage(BATT_CH_B), batteryGetPercentage(BATT_CH_B),
                    batteryGetPinMilliVolts(BATT_CH_B));
      lastBattLog = now;
    }

    // Critical warning for either channel
    if (batteryIsCritical(BATT_CH_A)) {
      Serial.printf("[BATT] CRITICAL CH-A: %.2fV (%d%%)\n",
                    batteryGetVoltage(BATT_CH_A), batteryGetPercentage(BATT_CH_A));
    }
    if (batteryIsCritical(BATT_CH_B)) {
      Serial.printf("[BATT] CRITICAL CH-B: %.2fV (%d%%)\n",
                    batteryGetVoltage(BATT_CH_B), batteryGetPercentage(BATT_CH_B));
    }
  }

  // ── Geofence + Theft updates (piggyback on GPS fix) ──
  if (gpsFix) {
    geoProxy.update(gpsLat, gpsLon, gpsHdop, gpsSats);

    bool ignitionOn = (deliveryIdCacheValid && cachedDeliveryId[0] != '\0');
    theftGuardUpdate((float)gpsLat, (float)gpsLon, gpsSpeed, ignitionOn, now);

    TheftState curTheft = theftGuardGetState();
    if (curTheft != prevTheftState) {
      Serial.printf("[EC-81] Theft state changed -> %s\n",
                    theftGuardStateStr());
      prevTheftState = curTheft;
    }
  }

  if ((wifiCloudHealthy || lteConnected) && deliveryIdCacheValid && !modemHttpBusy) {
    refreshPhoneLocationIfNeeded();
  }
  feedCloudIoWatchdog();

  // ── EC-78: Reassignment auto-ack ──
  if (reassign.processAutoAck(now)) {
    const char *newO = reassign.consumeNewOtp();
    if (newO[0] != '\0') {
      strncpy(cachedOtp, newO, sizeof(cachedOtp) - 1);
      cachedOtp[sizeof(cachedOtp) - 1] = '\0';
      otpCacheValid = true;
      dpSaveOtp(cachedOtp);
      Serial.printf("[EC-78] OTP updated to: %s\n", cachedOtp);
    }
  }

  // Re-sync network time periodically to prevent clock drift
  if ((wifiCloudHealthy || lteConnected) && !modemHttpBusy &&
      now - lastTimeSync >= TIME_SYNC_INTERVAL) {
    Serial.println("Periodic time re-sync...");
    syncNetworkTime();
    lastTimeSync = now;
  }
  feedCloudIoWatchdog();



  if ((wifiCloudHealthy || lteConnected) && !modemHttpBusy) {
    retryPendingCommandAck(now);
  }
  feedCloudIoWatchdog();

  if ((wifiCloudHealthy || lteConnected) && !modemHttpBusy &&
      now - lastPersonalPinFlushAt >= PERSONAL_PIN_AUDIT_FLUSH_INTERVAL_MS) {
    flushQueuedPersonalPinAudits();
    lastPersonalPinFlushAt = now;
  }
  feedCloudIoWatchdog();

  if (!modemHttpBusy) {
    maybePublishDurabilityMetrics(now);
  }
  feedCloudIoWatchdog();

  delay(10);
}
