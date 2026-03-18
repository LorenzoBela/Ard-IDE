/**
 * GPS & LTE Signal to Firebase - LTE Only (No WiFi)
 * For LILYGO T-SIM A7670E (GPS Version)
 *
 * Uses LTE exclusively via AT+HTTP commands (modem-level SSL)
 * This bypasses TinyGsmClientSecure which has issues on this modem.
 */

#include "esp_task_wdt.h"
#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "mbedtls/sha256.h"

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

// ==================== HOTSPOT (WiFi AP for ESP32-CAM) ====================
// The ESP32-CAM connects to this network; images are relayed to Supabase via
// LTE.
#define AP_PASS "topbox123"
#define CAM_SERVER_PORT 8080

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
#define FIREBASE_INTERVAL 3000
#define SIGNAL_INTERVAL 10000
#define IDLE_POLL_MULTIPLIER 2
#define IDLE_GPS_POLL_MULTIPLIER 3
#define TIME_SYNC_INTERVAL 1800000 // Re-sync time every 30 minutes

// Reliability
#define WDT_TIMEOUT_S 30            // Watchdog reboot after 30s hang
#define MODEM_HEALTH_INTERVAL 30000 // Check modem alive every 30s
#define MEM_REPORT_INTERVAL 60000   // Print heap stats every 60s
#define MAX_CONSECUTIVE_FAILURES 5  // Hard-reset modem after 5 AT timeouts
#define MAX_FB_FAILURES 3           // Reconnect LTE after 3 Firebase fails

// Philippine Standard Time offset (UTC+8)
#define PHT_OFFSET_HOURS 8
#define PHT_OFFSET_QUARTERS 32 // 8 hours * 4 quarter-hours

// ==================== GLOBALS ====================
HardwareSerial modemSerial(1);
TinyGsm modem(modemSerial);

bool lteConnected = false;
bool gpsEnabled = false;
bool modemOK = false;
unsigned long lastGps = 0, lastFB = 0, lastSig = 0, lastTimeSync = 0;
unsigned long lastModemHealth = 0, lastMemReport = 0;

double gpsLat = 0, gpsLon = 0;
double gpsHdop = 99.9; // Horizontal Dilution of Precision (99.9 = unknown)
float  gpsSpeed = 0.0f; // Speed in knots from CGNSSINFO, converted to m/s
bool gpsFix = false;
int cachedSignalRssi = -999;
int cachedSignalCsq = -1;
unsigned long dataBytesOut = 0; // Total bytes sent to Firebase

// Hotspot / camera proxy
WiFiServer camServer(CAM_SERVER_PORT);
bool apStarted = false;
String relayDiag =
    ""; // Diagnostics forwarded to ESP32-CAM via HTTP response body

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
char cachedPersonalPinHashMcu[65] = "";
char cachedPersonalPinSalt[33] = "";
char cachedPersonalPinRiderId[64] = "";
bool personalPinEnabled = false;
// EC-FIX: Remote lock/unlock status command from Firebase (UNLOCKING/LOCKED)
char cachedStatus[16] = "";
unsigned long cachedStatusSetAt = 0;
uint8_t cachedStatusServesRemaining = 0;
#define STATUS_COMMAND_MAX_SERVES 60
#define STATUS_COMMAND_RETRY_WINDOW_MS 60000

// Internal proxy state to prevent telemetry from overwriting physical lock state on Firebase
bool isBoxLocked = true;

double destLat = 0.0, destLon = 0.0;
bool destCoordsValid = false;
double pickupLat = 0.0, pickupLon = 0.0;
bool pickupCoordsValid = false;
unsigned long lastDeliveryContextRead = 0;
#define DELIVERY_CONTEXT_READ_INTERVAL 3000 // Re-read Firebase every 3s

// UDP log receiver
WiFiUDP udpServer;
#define UDP_LOG_PORT 5114

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

unsigned long lastPersonalPinFlushAt = 0;
#define PERSONAL_PIN_AUDIT_FLUSH_INTERVAL_MS 5000

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
  modem.sendAT("+CGNSSINFO");
  String resp = "";
  if (modem.waitResponse(2000L, resp) != 1) {
    gpsFix = false;
    Serial.println("GPS: No response from +CGNSSINFO");
    return;
  }

  int idx = resp.indexOf("+CGNSSINFO: ");
  if (idx < 0) {
    gpsFix = false;
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
    Serial.println("GPS: Not enough fields in response");
    return;
  }

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

    static unsigned long lastFixDebug = 0;
    if (millis() - lastFixDebug >= 30000) {
      Serial.printf("GPS Parsed: %.6f, %.6f (Fix acquired!)\n", gpsLat, gpsLon);
      lastFixDebug = millis();
    }
  } else {
    gpsFix = false;
  }
}

// ==================== LTE ====================
bool connectLTE() {
  if (!modemOK)
    return false;

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
// Uses A7670E's built-in HTTP client which handles SSL at modem level
bool httpPutToFirebase(const char *path, const String &jsonData) {
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
  Serial.printf("[MEM] Free: %u | Min: %u | Largest: %u | Stack HWM: %u\n",
                ESP.getFreeHeap(), ESP.getMinFreeHeap(), ESP.getMaxAllocHeap(),
                uxTaskGetStackHighWaterMark(NULL));

  if (ESP.getMaxAllocHeap() < 10240) {
    Serial.println("[MEM] WARNING: Largest free block < 10KB!");
  }
}

// Lightweight modem heartbeat â€” sends AT and checks for OK
void checkModemHealth() {
  if (!modemOK)
    return;
  modem.sendAT(""); // Simplest AT test
  if (modem.waitResponse(2000L) != 1) {
    consecutiveModemFailures++;
    Serial.printf("[HEALTH] Modem unresponsive (%u/%u)\n",
                  consecutiveModemFailures, MAX_CONSECUTIVE_FAILURES);
    if (consecutiveModemFailures >= MAX_CONSECUTIVE_FAILURES) {
      hardResetModem();
    }
  } else {
    consecutiveModemFailures = 0;
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

// ==================== FIREBASE SEND (snprintf â€” zero String allocs)
// ====================
void sendToFirebase() {
  if (!lteConnected) {
    Serial.println("Firebase: LTE not connected");
    return;
  }

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
  char hardwareJson[768];
  snprintf(hardwareJson, sizeof(hardwareJson),
           "{\"status\":\"%s\","
           "\"connection\":\"LTE\","
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
           "\"batt_low\":%s,"
           "\"geo_state\":\"%s\","
           "\"geo_dist_m\":%.1f,"
           "\"theft_state\":\"%s\"}",
           boxStatus, rssi, csq, opBuf, gpsFix ? "true" : "false", dataBytesOut,
           (unsigned long)millis(), now_epoch, tsBuf,
           timeSynced ? "true" : "false",
           batteryGetVoltage(), batteryGetPercentage(),
           batteryIsLow() ? "true" : "false",
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
    char locationJson[320];
    snprintf(locationJson, sizeof(locationJson),
             "{\"latitude\":%.6f,"
             "\"longitude\":%.6f,"
             "\"hdop\":%.2f,"
             "\"timestamp\":%lu,"
             "\"timestamp_str\":\"%s\","
             "\"source\":\"box\"}",
             gpsLat, gpsLon, gpsHdop, now_epoch, tsBuf);

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
    connectLTE();
    firebaseFailures = 0;
  }
}

// ==================== HOTSPOT + CAMERA PROXY ====================

// Start WiFi SoftAP so the ESP32-CAM can connect and send images.
void startHotspot() {
  Serial.println("\nStarting WiFi hotspot for ESP32-CAM...");
  WiFi.mode(WIFI_AP);
  IPAddress apIP(192, 168, 4, 1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  if (WiFi.softAP(AP_SSID, AP_PASS)) {
    apStarted = true;
    Serial.printf("[AP] SSID: %s | Pass: %s | IP: %s\n", AP_SSID, AP_PASS,
                  WiFi.softAPIP().toString().c_str());

    // Start UDP server to receive remote logs over WiFi
    udpServer.begin(UDP_LOG_PORT);
    Serial.printf("[UDP] Listening for remote logs on port %d\n", UDP_LOG_PORT);
  } else {
    Serial.println("[AP] Failed to start hotspot!");
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

// REST helper to patch a row directly in Supabase
bool httpPatchSupabase(const char *tableName, const char *recordId, const char *jsonPayload) {
  if (!lteConnected) return false;

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
      Serial.printf("[CONTEXT] Network failure checking %s (empty body). Assuming active.\n", deliveryId);
      return true;
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

  if (!writePersonalPinAuditNow(eventJson)) {
    queuePersonalPinAudit(eventJson);
  }
}

// ==================== TAMPER CLEAR SUPPRESSION ====================
// After admin clears tamper, permanently suppress new reed switch
// events until the box is officially unlocked again.
static bool tamperSuppressedByAdmin = false;

void refreshDeliveryContextFromFirebase() {
  if (!lteConnected) {
    Serial.println("[CONTEXT] Skip Firebase fetch — LTE not connected");
    return;
  }

  Serial.printf("[CONTEXT] Fetching /hardware/%s from Firebase...\n",
                HARDWARE_ID);
  // Fetch the entire hardware node since it contains both OTP and delivery_id
  char path[64];
  snprintf(path, sizeof(path), "/hardware/%s.json", HARDWARE_ID);

  String resp;
  sendATAndWait("+HTTPTERM", 1000);
  delay(200);

  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000L, resp) != 1)
    return;

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

  if (actionOK && httpStatus == 200) {
    // A7670E/A7672X caps each AT+HTTPREAD at 1024 bytes.
    // Firebase returns JSON keys alphabetically, so "background_location_status"
    // fills the first chunk, pushing "delivery_id" and "otp_code" past 1024.
    // Solution: read in 1024-byte chunks with increasing offsets.
    #define CTX_CHUNK  1024
    #define CTX_MAX    16384

    String body = "";
    body.reserve(CTX_MAX); // Pre-allocate to avoid realloc fragmentation
    int offset = 0;

    while (offset < CTX_MAX) {
      // Request next chunk from modem
      char readCmd[40];
      snprintf(readCmd, sizeof(readCmd), "AT+HTTPREAD=%d,%d\r\n", offset, CTX_CHUNK);
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
      delay(100);
      while (modem.stream.available())
        modem.stream.read();

      offset += chunkLen;

      // If modem returned fewer bytes than requested, we've read everything
      if (chunkLen < CTX_CHUNK) break;
    }

    #undef CTX_CHUNK
    #undef CTX_MAX

    Serial.printf("[CONTEXT] totalBytes=%d body(120): %.120s\n", (int)body.length(),
                  body.c_str());
    if (totalResponseLen > 0 && totalResponseLen > (int)body.length()) {
      Serial.printf(
          "[CONTEXT] WARNING: Response %d > CTX_MAX window (%d bytes read) — fields near the end may be truncated!\n",
          totalResponseLen, (int)body.length());
    }

    // EC-32: Parse top-level return_active (ignore nested historical fields).
    {
      bool parsedReturnActive = false;
      bool returnActiveValue = false;
      parsedReturnActive = readTopLevelJsonBool(body, "return_active", returnActiveValue);
      returnActive = parsedReturnActive ? returnActiveValue : false;
    }

    // Parse top-level return_otp.
    {
      char parsedReturnOtp[8] = "";
      if (readTopLevelJsonString(body, "return_otp", parsedReturnOtp, sizeof(parsedReturnOtp)) &&
          strlen(parsedReturnOtp) > 0 && strlen(parsedReturnOtp) <= 6) {
        strncpy(cachedReturnOtp, parsedReturnOtp, sizeof(cachedReturnOtp) - 1);
        cachedReturnOtp[sizeof(cachedReturnOtp) - 1] = '\0';
        returnOtpCacheValid = true;
      } else {
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
      } else {
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
      } else {
        cachedDeliveryId[0] = '\0';
        deliveryIdCacheValid = false;
      }
    }

    // Parse Personal PIN runtime metadata (top-level only).
    {
      bool parsedEnabled = false;
      bool enabledVal = false;
      parsedEnabled = readTopLevelJsonBool(body, "personal_pin_enabled", enabledVal);
      if (parsedEnabled) {
        personalPinEnabled = enabledVal;
        dpSavePersonalPinEnabled(personalPinEnabled);
      }

      char parsedHash[65] = "";
      if (readTopLevelJsonString(body, "personal_pin_hash_mcu", parsedHash,
                                 sizeof(parsedHash)) && strlen(parsedHash) == 64) {
        strncpy(cachedPersonalPinHashMcu, parsedHash,
                sizeof(cachedPersonalPinHashMcu) - 1);
        cachedPersonalPinHashMcu[sizeof(cachedPersonalPinHashMcu) - 1] = '\0';
        dpSavePersonalPinHash(cachedPersonalPinHashMcu);
      }

      char parsedSalt[33] = "";
      if (readTopLevelJsonString(body, "personal_pin_salt", parsedSalt,
                                 sizeof(parsedSalt)) && strlen(parsedSalt) > 0) {
        strncpy(cachedPersonalPinSalt, parsedSalt, sizeof(cachedPersonalPinSalt) - 1);
        cachedPersonalPinSalt[sizeof(cachedPersonalPinSalt) - 1] = '\0';
        dpSavePersonalPinSalt(cachedPersonalPinSalt);
      }

      char parsedRider[64] = "";
      if (readTopLevelJsonString(body, "current_rider_id", parsedRider,
                                 sizeof(parsedRider)) && strlen(parsedRider) > 0) {
        strncpy(cachedPersonalPinRiderId, parsedRider,
                sizeof(cachedPersonalPinRiderId) - 1);
        cachedPersonalPinRiderId[sizeof(cachedPersonalPinRiderId) - 1] = '\0';
        dpSavePersonalPinRiderId(cachedPersonalPinRiderId);
      }
    }

    // Guard against stale /hardware context (e.g., deliveries node was wiped).
    // If the referenced delivery no longer exists or is terminal, clear caches
    // and self-heal /hardware to stop serving old OTP/delivery over GET /otp.
    if (otpCacheValid && deliveryIdCacheValid) {
      if (!isDeliveryStillActiveInFirebase(cachedDeliveryId)) {
        otpCacheValid = false;
        cachedOtp[0] = '\0';
        deliveryIdCacheValid = false;
        cachedDeliveryId[0] = '\0';
        returnOtpCacheValid = false;
        cachedReturnOtp[0] = '\0';
        returnActive = false;

        dpClear();

        char hwPath[64];
        snprintf(hwPath, sizeof(hwPath), "/hardware/%s.json", HARDWARE_ID);
        bool clearOk = httpPatchWithRetry(
            hwPath,
            "{\"otp_code\":null,\"delivery_id\":null,\"return_otp\":null,\"return_active\":false}");
        Serial.printf("[CONTEXT] Cleared stale hardware delivery context: %s\n",
                      clearOk ? "OK" : "FAIL");
      }
    }

    // Parse destination coords for geofence + theft guard targeting.
    // Mobile app writes "target_lat"/"target_lng"; accept both that and
    // the legacy "dest_lat"/"dest_lon" so either naming convention works.
    int dLatIdx = body.indexOf("\"dest_lat\":");
    int dLonIdx = body.indexOf("\"dest_lon\":");
    if (dLatIdx < 0) dLatIdx = body.indexOf("\"target_lat\":");
    if (dLonIdx < 0) dLonIdx = body.indexOf("\"target_lng\":");
    int dLatValOffset = (body.indexOf("\"target_lat\":") == dLatIdx) ? 13 : 11;
    int dLonValOffset = (body.indexOf("\"target_lng\":") == dLonIdx) ? 13 : 11;
    if (dLatIdx >= 0 && dLonIdx >= 0) {
      double dLa = body.substring(dLatIdx + dLatValOffset).toDouble();
      double dLo = body.substring(dLonIdx + dLonValOffset).toDouble();
      if (dLa != 0.0 || dLo != 0.0) {
        bool changed = (dLa != destLat || dLo != destLon);
        destLat = dLa;
        destLon = dLo;
        destCoordsValid = true;
        if (changed) {
          geoProxy.setTarget(destLat, destLon);
          theftGuardSetGeofence((float)destLat, (float)destLon,
                               TG_GEOFENCE_RADIUS_KM);
          Serial.printf("[GEO] Target set: %.6f, %.6f\n", destLat, destLon);
        }
      }
    } else if (!deliveryIdCacheValid) {
      destCoordsValid = false;
    }

    // Parse pickup coords (pickup_lat / pickup_lng) for dual-geofence gating
    int pLatIdx = body.indexOf("\"pickup_lat\":");
    int pLonIdx = body.indexOf("\"pickup_lng\":");
    if (pLatIdx >= 0 && pLonIdx >= 0) {
      double pLa = body.substring(pLatIdx + 13).toDouble();
      double pLo = body.substring(pLonIdx + 13).toDouble();
      if (pLa != 0.0 || pLo != 0.0) {
        bool changed = (pLa != pickupLat || pLo != pickupLon);
        pickupLat = pLa;
        pickupLon = pLo;
        pickupCoordsValid = true;
        if (changed) {
          geoProxy.setPickup(pickupLat, pickupLon);
          Serial.printf("[GEO] Pickup set: %.6f, %.6f\n", pickupLat, pickupLon);
        }
      }
    } else if (!deliveryIdCacheValid) {
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
        Serial.printf("[EC-32] Return mode: target->PICKUP %.6f, %.6f\n", pickupLat, pickupLon);
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

    // ── NVS persistence ──
    if (otpCacheValid)          dpSaveOtp(cachedOtp);
    if (deliveryIdCacheValid)   dpSaveDeliveryId(cachedDeliveryId);

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
          strncpy(cachedStatus, body.c_str() + start, len);
          cachedStatus[len] = '\0';
          if (strcmp(cachedStatus, "NONE") == 0) {
            cachedStatus[0] = '\0';
            cachedStatusSetAt = 0;
            cachedStatusServesRemaining = 0;
            Serial.println("[CONTEXT] Remote command cleared (NONE)");
          } else {
            cachedStatusSetAt = millis();
            cachedStatusServesRemaining = STATUS_COMMAND_MAX_SERVES;
            Serial.printf("[CONTEXT] Remote command parsed: '%s' (retry window armed)\n",
                          cachedStatus);

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

  sendATAndWait("+HTTPTERM", 1000);
}

// ==================== LOCK EVENT WRITER ====================
void writeLockEventToFirebase(bool otpValid, bool faceDetected, bool unlocked,
                              int faceAttempts = 0,
                              bool faceRetryExhausted = false,
                              bool fallbackRequired = false,
                              const char *failureReason = "") {
  if (!lteConnected) {
    Serial.println("[EVENT] LTE not connected");
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
  snprintf(eventJson, sizeof(eventJson),
           "{\"otp_valid\":%s,\"face_detected\":%s,\"unlocked\":%s,"
           "\"timestamp\":{\".sv\":\"timestamp\"},\"device_epoch\":%lu,"
           "\"timestamp_str\":\"%s\",\"face_attempts\":%d,"
           "\"face_retry_exhausted\":%s,\"fallback_required\":%s,"
           "\"failure_reason\":\"%s\"}",
           otpValid ? "true" : "false", faceDetected ? "true" : "false",
           unlocked ? "true" : "false", now_epoch, tsBuf, faceAttempts,
           faceRetryExhausted ? "true" : "false",
           fallbackRequired ? "true" : "false", safeReason);

  char eventPath[64];
  snprintf(eventPath, sizeof(eventPath), "/lock_events/%s/latest.json",
           HARDWARE_ID);
  bool ok = httpPutWithRetry(eventPath, eventJson);
  Serial.printf("[EVENT] lock_events: %s\n", ok ? "OK" : "FAIL");

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
void writeCommandAckToFirebase(const char *command, const char *status,
                               const char *details) {
  if (!lteConnected) {
    Serial.println("[CMD_ACK] LTE not connected");
    return;
  }

  unsigned long now_epoch = getCurrentEpoch();
  char json[320];
  snprintf(json, sizeof(json),
           "{\"command_ack_command\":\"%s\",\"command_ack_status\":\"%s\","
           "\"command_ack_details\":\"%s\",\"command_ack_at\":{\".sv\":\"timestamp\"},"
           "\"command_ack_epoch\":%lu}",
           command ? command : "", status ? status : "", details ? details : "",
           now_epoch);

  char hwPath[64];
  snprintf(hwPath, sizeof(hwPath), "/hardware/%s.json", HARDWARE_ID);
  bool ok = httpPatchWithRetry(hwPath, json);
  Serial.printf("[CMD_ACK] Firebase write: %s (%s/%s)\n", ok ? "OK" : "FAIL",
                command ? command : "", status ? status : "");

  // Sync internal state and immediately patch `status` at root too if executed/already_locked
  if (ok && (strcmp(status, "executed") == 0 || strcmp(status, "already_locked") == 0 || strcmp(status, "already_unlocked") == 0)) {
    if (strcmp(command, "LOCKED") == 0) {
      isBoxLocked = true;
      httpPatchWithRetry(hwPath, "{\"status\":\"LOCKED\"}");
    } else if (strcmp(command, "UNLOCKING") == 0) {
      isBoxLocked = false;
      httpPatchWithRetry(hwPath, "{\"status\":\"UNLOCKING\"}");

      // Only clear admin tamper suppression when unlock was acknowledged by
      // the controller.
      if (tamperSuppressedByAdmin &&
          (strcmp(status, "executed") == 0 ||
           strcmp(status, "already_unlocked") == 0)) {
        tamperSuppressedByAdmin = false;
        Serial.println("[TAMPER] Suppression cleared by unlock command ACK");
      }
    }
  }
}

// ==================== TAMPER EVENT WRITER ====================
void writeTamperToFirebase() {
  if (!lteConnected) {
    Serial.println("[TAMPER] LTE not connected");
    return;
  }

  unsigned long now_epoch = getCurrentEpoch();
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
           "\"source\":\"reed_switch\"}",
           now_epoch, tsBuf);

  char tamperPath[64];
  snprintf(tamperPath, sizeof(tamperPath), "/hardware/%s/tamper.json",
           HARDWARE_ID);
  bool ok = httpPutWithRetry(tamperPath, tamperJson);
  Serial.printf("[TAMPER] Firebase tamper write: %s\n", ok ? "OK" : "FAIL");

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
           "\"device_epoch\":%lu,\"timestamp_str\":\"%s\"}",
           now_epoch, tsBuf);
  char eventPath[64];
  snprintf(eventPath, sizeof(eventPath), "/lock_events/%s/latest.json",
           HARDWARE_ID);
  httpPutWithRetry(eventPath, eventJson);
}

// ==================== ADMIN TAMPER CLEAR READER ====================
// Called periodically to check if admin has requested a tamper clear
// from web/mobile dashboard. Reads hardware/{boxId}/clear_tamper node.
void checkAndClearTamperFromFirebase() {
  if (!lteConnected) return;

  char clearPath[80];
  snprintf(clearPath, sizeof(clearPath), "/hardware/%s/clear_tamper.json",
           HARDWARE_ID);

  String body = httpGetFromFirebase(clearPath);
  if (body.length() == 0 || body == "null") return;

  // Admin requested clear — reset TheftGuard state machine
  Serial.println("[TAMPER] Admin clear_tamper detected — resetting TheftGuard");
  theftGuardReset();
  tamperSuppressedByAdmin = true; // Start suppression — ignore new tamper events until unlock

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

  Serial.println("[TAMPER] TheftGuard reset to NORMAL, tamper permanently suppressed until unlock");
}

// ==================== FACE CHECK FORWARDER ====================
String forwardFaceCheck(String deliveryId) {
  // If CAM IP not yet discovered from /upload, try default SoftAP client IP
  if (!camClientKnown) {
    Serial.println(
        "[FACE] CAM IP not from /upload -- trying default 192.168.4.10");
    camClientIP = IPAddress(192, 168, 4, 10);
    // Don't set camClientKnown yet -- let /upload confirm later
  }

  WiFiClient camClient;
  if (!camClient.connect(camClientIP, CAM_FACE_PORT)) {
    Serial.println("[FACE] Cannot connect to ESP32-CAM");
    return "ERROR:cam_unreachable";
  }

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
  while (millis() - waitStart < 12000) {
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
  Serial.printf("[FACE] CAM response: %s\n", result.c_str());
  return result;
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
  Serial.printf("[CAM] Power mode=%s response: %s\n", mode.c_str(),
                result.c_str());
  return result.length() > 0 ? result : "ERROR:cam_timeout";
}

// ==================== HOTSPOT HTTP ROUTER ====================
void handleCameraClient() {
  WiFiClient client = camServer.available();
  if (!client)
    return;

  Serial.println("[AP] Client connected");
  esp_task_wdt_reset();

  IPAddress clientAddr = client.remoteIP();

  unsigned long headerTimeout = millis() + 500;
  while (!client.available() && millis() < headerTimeout) {
    delay(10);
    esp_task_wdt_reset();
  }
  if (!client.available()) {
    client.stop();
    return;
  }

  client.setTimeout(2000);
  String requestLine = client.readStringUntil('\n');
  requestLine.trim();
  Serial.println("[AP] REQ: " + requestLine);

  char method[8] = "";
  char reqPath[32] = "";
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

  while (true) {
    String line = client.readStringUntil('\n');
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
    }
  }

  String reqPathStr = String(reqPath);

  // ── GET /otp ──
  if (strcmp(method, "GET") == 0 && reqPathStr.startsWith("/otp")) {
    String otpPart, delPart;
    String geoOverride = "";

    if (theftGuardShouldBlockOtp()) {
      otpPart = "NO_OTP";
      delPart = "NO_DELIVERY";
      Serial.printf("[AP] OTP suppressed — theft state: %s\n",
                    theftGuardStateStr());
    } else if (destCoordsValid && gpsFix) {
      bool atPickup = geoProxy.isNearPickup(gpsLat, gpsLon);
      bool atDropoff = geoProxy.isNearDropoff(gpsLat, gpsLon);

      if (atDropoff) {
        // At dropoff: serve normally (Controller shows Enter PIN)
        if (returnActive) {
          otpPart = returnOtpCacheValid ? String(cachedReturnOtp) : "NO_OTP";
        } else {
          otpPart = otpCacheValid ? String(cachedOtp) : "NO_OTP";
        }
        delPart = deliveryIdCacheValid ? String(cachedDeliveryId) : "NO_DELIVERY";
      } else if (atPickup) {
        // At pickup: NO OTP, tell Controller we are at pickup -> Ongoing Pickup
        otpPart = "NO_OTP";
        delPart = deliveryIdCacheValid ? String(cachedDeliveryId) : "NO_DELIVERY";
        geoOverride = "GEO_PICKUP";
      } else {
        // In transit
        otpPart = "NO_OTP";
        delPart = deliveryIdCacheValid ? String(cachedDeliveryId) : "NO_DELIVERY";
        if (String(cachedDeliveryStatus) == "PICKED_UP" || String(cachedDeliveryStatus) == "IN_TRANSIT" || String(cachedDeliveryStatus) == "\"PICKED_UP\"") {
          geoOverride = "GEO_TRANSIT_DROP";
        } else {
          geoOverride = "GEO_TRANSIT_PICK";
        }
        Serial.printf("[AP] OTP suppressed — outside geofence (%.0fm) Status: %s\n",
                      geoProxy.snap.distanceM, geoOverride.c_str());
      }
    } else {
      // Fail closed when geofence cannot be evaluated (no GPS fix / invalid coords).
      // This keeps OTP/PIN strictly gated by location verification.
      otpPart = "NO_OTP";
      delPart = deliveryIdCacheValid ? String(cachedDeliveryId) : "NO_DELIVERY";
      if (String(cachedDeliveryStatus) == "PICKED_UP" || String(cachedDeliveryStatus) == "IN_TRANSIT" || String(cachedDeliveryStatus) == "\"PICKED_UP\"") {
        geoOverride = "GEO_TRANSIT_DROP";
      } else {
        geoOverride = "GEO_TRANSIT_PICK";
      }
      Serial.printf("[AP] OTP suppressed — geofence unavailable (gpsFix=%d destValid=%d) Status: %s\n",
                    gpsFix ? 1 : 0, destCoordsValid ? 1 : 0, geoOverride.c_str());
    }

    String body = otpPart + "," + delPart;
    // EC-FIX: Append remote status command (UNLOCKING/LOCKED) if present.
    // Keep command available for a short retry window so controller can catch it.
    unsigned long nowMs = millis();
    bool statusActive =
        cachedStatus[0] != '\0' &&
        cachedStatusServesRemaining > 0 &&
        (unsigned long)(nowMs - cachedStatusSetAt) <= STATUS_COMMAND_RETRY_WINDOW_MS;

    // Priority: status command > geo override > return mode (they are mutually exclusive actions).
    if (statusActive) {
      body += "," + String(cachedStatus);
      if (cachedStatusServesRemaining > 0) {
        cachedStatusServesRemaining--;
      }
      if (cachedStatusServesRemaining == 0) {
        Serial.printf("[AP] Status command consumed max serves: %s\n", cachedStatus);
        cachedStatus[0] = '\0';
        cachedStatusSetAt = 0;
      }
    } else if (geoOverride != "") {
      body += "," + geoOverride;
    } else if (returnActive) {
      // Drop stale command if retry window elapsed.
      if (cachedStatus[0] != '\0' &&
          (unsigned long)(nowMs - cachedStatusSetAt) > STATUS_COMMAND_RETRY_WINDOW_MS) {
        Serial.printf("[AP] Status command expired after retry window: %s\n", cachedStatus);
        cachedStatus[0] = '\0';
        cachedStatusSetAt = 0;
        cachedStatusServesRemaining = 0;
      }
      body += ",RETURNING";
    }
    Serial.printf("[AP] -> GET /otp  serving: '%s'\n", body.c_str());
    String resp =
        "HTTP/1.1 200 OK\r\n"
        "Connection: close\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: " + String(body.length()) + "\r\n\r\n" + body;
    client.print(resp);
    // Increased delay before closing to ensure ESP32 proxy client receives data
    client.flush();
    delay(150);

    // PURGE REMAINING BYTES to prevent ghost connections
    unsigned long _purgeLimit = millis() + 500;
    while (client.available() && millis() < _purgeLimit) {
      client.read();
    }

    client.stop();
    return;
  }

  // ── POST /command-ack ──
  if (strcmp(method, "POST") == 0 && strcmp(reqPath, "/command-ack") == 0) {
    Serial.println("[AP] -> POST /command-ack");
    char jsonBuf[320] = "";
    if (contentLength > 0 && contentLength < sizeof(jsonBuf)) {
      client.setTimeout(5000);
      size_t rd = client.readBytes(jsonBuf, contentLength);
      jsonBuf[rd] = '\0';
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

    if (cmd[0] != '\0' && strcmp(cachedStatus, cmd) == 0) {
      cachedStatus[0] = '\0';
      cachedStatusServesRemaining = 0;
      cachedStatusSetAt = 0;
      Serial.println("[AP] Cleared cachedStatus locally upon ACK");
    }

    // Send HTTP 200 OK before blocking on Firebase write
    String resp = "HTTP/1.1 200 OK\r\nContent-Length: 2\r\n\r\nOK";
    client.print(resp);
    client.flush();
    delay(10);
    client.stop(); // Disconnect ESP32 immediately to prevent -11 timeout
    
    // Now write to Firebase (takes a few seconds)
    writeCommandAckToFirebase(cmd, status, details);
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
    if (contentLength > 0 && contentLength < sizeof(jsonBuf)) {
      client.setTimeout(5000);
      size_t rd = client.readBytes(jsonBuf, contentLength);
      jsonBuf[rd] = '\0';
    }

    char pin[12] = "";
    extractJsonStringValue(jsonBuf, "pin", pin, sizeof(pin));
    bool currentlyLocked = (strstr(jsonBuf, "\"currently_locked\":true") != NULL);

    const char *resultBody = "DENY:invalid";
    bool verified = false;

    if (!personalPinEnabled) {
      resultBody = "DENY:disabled";
    } else if (pin[0] == '\0') {
      resultBody = "DENY:missing_pin";
    } else {
      verified = verifyPersonalPinLocal(pin);
      if (verified) {
        resultBody = currentlyLocked ? "ALLOW_UNLOCK" : "ALLOW_RELOCK";
      } else {
        resultBody = "DENY:mismatch";
      }
    }

    String resp =
        "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: " +
        String(strlen(resultBody)) + "\r\n\r\n" + resultBody;
    client.print(resp);
    client.flush();
    delay(10);
    client.stop();

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
    Serial.println("[AP] -> POST /event");
    char jsonBuf[384] = "";
    if (contentLength > 0 && contentLength < sizeof(jsonBuf)) {
      client.setTimeout(5000);
      size_t rd = client.readBytes(jsonBuf, contentLength);
      jsonBuf[rd] = '\0';
    }
    Serial.printf("[AP] Event: %s\n", jsonBuf);

    // Send HTTP 200 OK before blocking on Firebase write
    String resp = "HTTP/1.1 200 OK\r\nContent-Length: 2\r\n\r\nOK";
    client.print(resp);
    client.flush();
    delay(10);
    client.stop(); // Disconnect ESP32 immediately to prevent -11 timeout

    // Reed switch tamper: Controller detected unauthorized lid-open
    if (strstr(jsonBuf, "\"tamper\":true") != NULL) {
      // Check suppression — ignore re-triggers after admin clear until next valid unlock
      if (tamperSuppressedByAdmin) {
        Serial.println("[AP] TAMPER suppressed — awaiting next valid unlock");
      } else {
        Serial.println("[AP] TAMPER event received — writing to Firebase");
        writeTamperToFirebase();
      }
    } else {
      String jsonStr = String(jsonBuf);
      bool ov = (strstr(jsonBuf, "\"otp_valid\":true") != NULL);
      bool fd = (strstr(jsonBuf, "\"face_detected\":true") != NULL);
      bool ul = (strstr(jsonBuf, "\"unlocked\":true") != NULL);
      int faceAttempts = 0;
      bool faceRetryExhausted = false;
      bool fallbackRequired = false;
      char failureReason[24] = "";

      readTopLevelJsonInt(jsonStr, "face_attempts", faceAttempts);
      readTopLevelJsonBool(jsonStr, "face_retry_exhausted", faceRetryExhausted);
      readTopLevelJsonBool(jsonStr, "fallback_required", fallbackRequired);
      readTopLevelJsonString(jsonStr, "failure_reason", failureReason,
                             sizeof(failureReason));
      
      // Clear suppression only on a fully validated unlock event.
      // This prevents noisy/partial events from re-enabling tamper spam.
      bool validUnlock = (ul && ov && fd);
      if (validUnlock) {
        if (tamperSuppressedByAdmin) {
          Serial.println("[AP] Valid unlock event — tamper suppression cleared");
        }
        tamperSuppressedByAdmin = false;
      } else if (ul && tamperSuppressedByAdmin) {
        Serial.println("[AP] Unlock event without full validation — suppression kept");
      }
      
      // Also write alert events to the lock_events stream so it appears in web UI
      writeLockEventToFirebase(ov, fd, ul, faceAttempts, faceRetryExhausted,
                               fallbackRequired, failureReason);
    }
    
    return;
  }

  // â”€â”€ GET /diag â”€â”€
  if (strcmp(method, "GET") == 0 && strcmp(reqPath, "/diag") == 0) {
    char body[192];
    snprintf(body, sizeof(body),
             "batt_pct=%d,batt_v=%.2f,rssi=%d,csq=%d,gps_fix=%d,lte=%d,modem=%d,time=%d,fb_fail=%u,uptime=%lu",
             batteryGetPercentage(), batteryGetVoltage(),
             cachedSignalRssi, cachedSignalCsq,
             gpsFix ? 1 : 0,
             lteConnected ? 1 : 0,
             modemOK ? 1 : 0,
             timeSynced ? 1 : 0,
             (unsigned int)firebaseFailures,
             (unsigned long)millis());

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
  camClientIP = clientAddr;
  camClientKnown = true;
  Serial.printf("[AP] CAM IP: %s | Image: %u bytes\n",
                camClientIP.toString().c_str(), contentLength);

  bool uploadOK = false;
  String failReason = "";
  if (contentLength == 0 || contentLength > 400000) {
    failReason = "FAIL:bad_cl:" + String(contentLength);
  } else {
    uint8_t *buf = (uint8_t *)malloc(contentLength);
    if (!buf) {
      failReason = "FAIL:malloc_oom";
    } else {
      client.setTimeout(30000);
      size_t received = client.readBytes(buf, contentLength);
      esp_task_wdt_reset();
      if (received == contentLength) {
        uploadOK = uploadToSupabaseViaLTE(buf, contentLength, objectPath);
        if (!uploadOK)
          failReason = relayDiag;
      } else {
        failReason =
            "FAIL:incomplete:" + String(received) + "/" + String(contentLength);
      }
      free(buf);
    }
  }

  // Write the public URL back to Firebase so web/mobile apps can find the image
  if (uploadOK && lteConnected) {
    char publicUrl[256];
    snprintf(publicUrl, sizeof(publicUrl),
             "%s/storage/v1/object/public/%s/%s",
             SUPABASE_URL, SUPABASE_BUCKET, objectPath.c_str());

    char urlJson[320];
    snprintf(urlJson, sizeof(urlJson),
             "{\"last_upload_public_url\":\"%s\"}", publicUrl);

    char camPath[64];
    snprintf(camPath, sizeof(camPath), "/hardware/%s/camera.json", HARDWARE_ID);
    bool fbOK = httpPatchWithRetry(camPath, urlJson);
    Serial.printf("[RELAY] Firebase URL writeback: %s\n", fbOK ? "OK" : "FAIL");

    if (deliveryIdCacheValid && cachedDeliveryId[0] != '\0') {
      char auditJson[320];
      snprintf(auditJson, sizeof(auditJson),
               "{\"latest_photo_url\":\"%s\"}", publicUrl);
      char auditPath[96];
      snprintf(auditPath, sizeof(auditPath),
               "/audit_logs/%s.json", cachedDeliveryId);
      httpPatchWithRetry(auditPath, auditJson);

      // IMPORTANT: Write the proof_photo_url directly to the delivery document
      // so the web/mobile app sync engines pick it up correctly.
      char deliveryJson[320];
      snprintf(deliveryJson, sizeof(deliveryJson),
               "{\"proof_photo_url\":\"%s\"}", publicUrl);
      char deliveryPath[96];
      snprintf(deliveryPath, sizeof(deliveryPath),
               "/deliveries/%s.json", cachedDeliveryId);
      httpPatchWithRetry(deliveryPath, deliveryJson);

      // CRITICAL: The Web Tracking page Server-Side-Render uses the Supabase 'deliveries' table directly.
      // We must write the proof_photo_url to Supabase via REST API right after upload.
      httpPatchSupabase("deliveries", cachedDeliveryId, deliveryJson);
    }
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
  String resp;
  sendATAndWait("+HTTPTERM", 1000);
  delay(200);

  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000L, resp) != 1)
    return "";

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
    delay(100);
    while (modem.stream.available())
      modem.stream.read();
  }

  sendATAndWait("+HTTPTERM", 1000);
  return body;
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
  delay(1000);
  Serial.println("\n=== GPS/LTE Firebase Test (LTE Only - AT+HTTP) ===\n");

  // â”€â”€ Initialize Watchdog Timer FIRST (before any modem ops) â”€â”€
  // Core 2.x API: esp_task_wdt_init(timeout_seconds, panic_on_timeout)
  esp_task_wdt_init(WDT_TIMEOUT_S, true); // true = reboot on timeout
  esp_task_wdt_add(NULL);                 // Watch the main loop task
  Serial.printf("[WDT] Watchdog armed (%ds timeout)\n", WDT_TIMEOUT_S);

  // Auto-provision box identity (NVS cache or Firebase registration)
  autoProvision();

  // â”€â”€ Start WiFi hotspot so ESP32-CAM can connect and relay images â”€â”€
  startHotspot();
  if (apStarted) {
    camServer.begin();
    Serial.printf("[AP] Camera proxy server listening on port %d\n",
                  CAM_SERVER_PORT);
    Serial.printf("[AP] ESP32-CAM should connect to SSID '%s' / '%s'\n",
                  AP_SSID, AP_PASS);
  }

  // Initialize modem with proper power sequence
  modemSerial.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
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

  Serial.println("\nConnecting via LTE...");
  if (!connectLTE()) {
    Serial.println("\n*** LTE CONNECTION FAILED - will retry in loop() ***");
    Serial.println("  CHECK: 1) SIM has active data plan");
    Serial.println("         2) LTE antenna connected");
    Serial.println("         3) Signal coverage in area");
  }

  // â”€â”€ POST-LTE: Retry AGPS + XTRA now that data is available â”€â”€
  // These failed during GPS init because LTE wasn't connected yet
  if (gpsEnabled) {
    Serial.println("\nRetrying GPS assistance with LTE data...");

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

  // Restore persisted delivery context (survives brown-out / reboot)
  if (dpLoadOtp(cachedOtp, sizeof(cachedOtp))) {
    otpCacheValid = true;
    Serial.printf("[DP] Restored OTP from NVS: %s\n", cachedOtp);
  }
  if (dpLoadDeliveryId(cachedDeliveryId, sizeof(cachedDeliveryId))) {
    deliveryIdCacheValid = true;
    Serial.printf("[DP] Restored DeliveryID from NVS: %s\n", cachedDeliveryId);
  }
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

  Serial.println("\n=== Ready (LTE Only - AT+HTTP) ===\n");

  printMemoryReport();
}

// ==================== LOOP ====================
void loop() {
  esp_task_wdt_reset(); // Feed watchdog every loop iteration
  unsigned long now = millis();

  // ── Modem/LTE retry with exponential backoff (keeps HTTP server alive) ──
  static unsigned long modemRetryAt = 0;
  static unsigned long modemRetryBackoffMs = 10000; // start at 10s
  const unsigned long MODEM_RETRY_BACKOFF_MAX_MS = 120000; // cap at 120s

  if ((!modemOK || !lteConnected) && now >= modemRetryAt) {
    Serial.println("[RETRY] Modem/LTE health check...");

    if (!modemOK) {
      Serial.println("[RETRY] Reinitializing modem...");
      powerModem();
      modemOK = initModem();
    }

    if (modemOK && !lteConnected) {
      Serial.println("[RETRY] Reconnecting LTE...");
      lteConnected = connectLTE();
      if (lteConnected) {
        // On first successful LTE after boot or outage, resync time.
        syncNetworkTime();
      }
    }

    if (!modemOK || !lteConnected) {
      modemRetryBackoffMs *= 2;
      if (modemRetryBackoffMs > MODEM_RETRY_BACKOFF_MAX_MS) {
        modemRetryBackoffMs = MODEM_RETRY_BACKOFF_MAX_MS;
      }
    } else {
      modemRetryBackoffMs = 10000;
    }

    modemRetryAt = now + modemRetryBackoffMs;
  }

    bool hasActiveDelivery = deliveryIdCacheValid && cachedDeliveryId[0] != '\0';
    bool idlePowerMode = !hasActiveDelivery;
    unsigned long signalInterval =
      idlePowerMode ? (SIGNAL_INTERVAL * IDLE_POLL_MULTIPLIER) : SIGNAL_INTERVAL;
    unsigned long firebaseInterval = idlePowerMode
                       ? (FIREBASE_INTERVAL * IDLE_POLL_MULTIPLIER)
                       : FIREBASE_INTERVAL;
    unsigned long deliveryReadInterval =
      idlePowerMode ? (DELIVERY_CONTEXT_READ_INTERVAL * IDLE_POLL_MULTIPLIER)
            : DELIVERY_CONTEXT_READ_INTERVAL;

  // Handle incoming camera image upload (non-blocking when idle)
  if (apStarted) {
    handleCameraClient();

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

  // Check LTE connection periodically
  if (now - lastSig >= signalInterval) {
    if (lteConnected && modemOK) {
      cachedSignalCsq = modem.getSignalQuality();
      cachedSignalRssi = (cachedSignalCsq == 99 || cachedSignalCsq == 0)
                             ? -999
                             : -113 + (2 * cachedSignalCsq);
      if (!modem.isGprsConnected()) {
        Serial.println("LTE: Connection lost, reconnecting...");
        lteConnected = false;
        connectLTE();
      }
    }
    lastSig = now;
  }

  // Modem health check (lightweight AT heartbeat)
  if (modemOK && now - lastModemHealth >= MODEM_HEALTH_INTERVAL) {
    checkModemHealth();
    lastModemHealth = now;
  }

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
  if (modemOK && gpsEnabled && now - lastGps >= gpsInterval) {
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
  }

  // ── Battery monitor (10 s interval) ──
  if (now - lastBatteryRead >= BATTERY_READ_INTERVAL) {
    batteryUpdate();
    lastBatteryRead = now;
    if (batteryIsCritical()) {
      Serial.printf("[BATT] CRITICAL: %.2fV (%d%%)\n",
                    batteryGetVoltage(), batteryGetPercentage());
    }
  }

  // ── Geofence + Theft updates (piggyback on GPS fix) ──
  if (gpsFix) {
    geoProxy.update(gpsLat, gpsLon, gpsHdop, 0);

    bool ignitionOn = (deliveryIdCacheValid && cachedDeliveryId[0] != '\0');
    theftGuardUpdate((float)gpsLat, (float)gpsLon, gpsSpeed, ignitionOn, now);

    TheftState curTheft = theftGuardGetState();
    if (curTheft != prevTheftState) {
      Serial.printf("[EC-81] Theft state changed -> %s\n",
                    theftGuardStateStr());
      prevTheftState = curTheft;
    }
  }

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
  if (lteConnected && now - lastTimeSync >= TIME_SYNC_INTERVAL) {
    Serial.println("Periodic time re-sync...");
    syncNetworkTime();
    lastTimeSync = now;
  }

  // Send to Firebase
  if (lteConnected && now - lastFB >= firebaseInterval) {
    sendToFirebase();
    lastFB = now;
  }

  // Refresh cached Delivery Context from Firebase (for Tester ESP32 /otp
  // endpoint)
  if (lteConnected && now - lastDeliveryContextRead >= deliveryReadInterval) {
    refreshDeliveryContextFromFirebase();
    checkAndClearTamperFromFirebase();
    lastDeliveryContextRead = now;
  }

  if (lteConnected && now - lastPersonalPinFlushAt >= PERSONAL_PIN_AUDIT_FLUSH_INTERVAL_MS) {
    flushQueuedPersonalPinAudits();
    lastPersonalPinFlushAt = now;
  }

  delay(100);
}
