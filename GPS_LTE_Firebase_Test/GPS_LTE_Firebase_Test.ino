/**
 * GPS & LTE Signal to Firebase - LTE Only (No WiFi)
 * For LILYGO T-SIM A7670E (GPS Version)
 *
 * Uses LTE exclusively via AT+HTTP commands (modem-level SSL)
 * This bypasses TinyGsmClientSecure which has issues on this modem.
 */

#include "esp_task_wdt.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// TinyGSM - MUST define modem BEFORE include
#define TINY_GSM_MODEM_A7672X
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>

// ==================== CONFIGURATION ====================
// LTE Settings â€” Globe Telecom Philippines
// Postpaid: "internet.globe.com.ph"  |  Prepaid: "http.globe.com.ph"
#define GPRS_APN "internet.globe.com.ph"
#define GPRS_APN_ALT "http.globe.com.ph"
#define GPRS_USER ""
#define GPRS_PASS ""

// Firebase REST API
#define FIREBASE_HOST                                                          \
  "smart-top-box-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "AIzaSyA7DETBpsdPN6icfWi7PijCbpmLNWEZyTQ"
#define HARDWARE_ID "BOX_001"

// ==================== HOTSPOT (WiFi AP for ESP32-CAM) ====================
// The ESP32-CAM connects to this network; images are relayed to Supabase via
// LTE.
#define AP_SSID "SmartTopBox_AP"
#define AP_PASS "topbox123"
#define CAM_SERVER_PORT 8080

// Supabase Storage (matches ESP32-CAM sketch)
#define SUPABASE_URL "https://lvpneakciqegwyymtqno.supabase.co"
#define SUPABASE_BUCKET "r3"
#define SUPABASE_ANON_KEY                                                      \
  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9."                                      \
  "eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Imx2cG5lYWtjaXFlZ3d5eW10cW5vIiwicm9sZSI6Im" \
  "Fub24iLCJpYXQiOjE3Njc5MDYzNzgsImV4cCI6MjA4MzQ4MjM3OH0."                     \
  "liZ3l1u18H7WwIc72P9JgBTp9b7zUlLfPUhCAndW9uU"

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
#define FIREBASE_INTERVAL 5000
#define SIGNAL_INTERVAL 10000
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
bool gpsFix = false;
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
char cachedDeliveryId[64] = "";
bool deliveryIdCacheValid = false;
unsigned long lastDeliveryContextRead = 0;
#define DELIVERY_CONTEXT_READ_INTERVAL 15000 // Re-read Firebase every 15s

// UDP log receiver
WiFiUDP udpServer;
#define UDP_LOG_PORT 5114

// ESP32-CAM IP tracking (for face-check forwarding)
IPAddress camClientIP;
bool camClientKnown = false;
#define CAM_FACE_PORT 80 // ESP32-CAM runs a tiny HTTP server on port 80

// Reliability counters
uint8_t consecutiveModemFailures = 0;
uint8_t firebaseFailures = 0;

// Network time tracking (Philippine Standard Time, UTC+8)
unsigned long epochTime = 0; // Unix timestamp (seconds since 1970 UTC)
unsigned long epochSyncMillis =
    0; // millis() value when epochTime was last synced
bool timeSynced = false;

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

  // If PUT (action=4) failed, try with PATCH via POST workaround
  if (!actionOK || httpStatus == 0) {
    // Some A7670E firmware doesn't support action=4 (PUT)
    // Fallback: use PATCH with URL parameter
    Serial.print("(PUT unsupported, using PATCH)... ");
    sendATAndWait("+HTTPTERM", 1000);
    delay(200);

    // Re-init with PATCH URL (Firebase supports PATCH via REST)
    modem.sendAT("+HTTPINIT");
    modem.waitResponse(5000L);
    modem.sendAT("+HTTPPARA=\"CID\",1");
    modem.waitResponse(3000L);

    // Firebase REST API accepts PATCH on the URL to update data
    String patchUrl = "https://" + String(FIREBASE_HOST) + path;
    if (strlen(FIREBASE_AUTH) > 0) {
      patchUrl += "?auth=" + String(FIREBASE_AUTH);
    }
    String patchUrlCmd = "+HTTPPARA=\"URL\",\"" + patchUrl + "\"";
    modem.sendAT(patchUrlCmd.c_str());
    modem.waitResponse(3000L);

    modem.sendAT("+HTTPPARA=\"CONTENT\",\"application/json\"");
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

      // Use POST (action=1) â€” Firebase will overwrite the node
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

// ==================== FIREBASE SEND (snprintf â€” zero String allocs)
// ====================
void sendToFirebase() {
  if (!lteConnected) {
    Serial.println("Firebase: LTE not connected");
    return;
  }

  Serial.print("Firebase... ");

  // Derive box status from current state
  const char *boxStatus = "IDLE";
  if (gpsFix) {
    boxStatus = "ACTIVE";
  } else if (lteConnected) {
    boxStatus = "STANDBY";
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
  String opStr = modem.getOperator();
  char opBuf[32];
  strncpy(opBuf, opStr.c_str(), sizeof(opBuf) - 1);
  opBuf[sizeof(opBuf) - 1] = '\0';

  // â”€â”€ Build hardware JSON with snprintf (zero heap allocs) â”€â”€
  // Use Firebase server timestamp {".sv":"timestamp"} for last_updated
  // so the web dashboard always gets an accurate UTC ms timestamp,
  // regardless of whether the ESP32's NTP sync succeeded.
  // uptime_ms = millis() â€” device time since boot, survives web page refresh.
  char hardwareJson[512];
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
           "\"time_synced\":%s}",
           boxStatus, rssi, csq, opBuf, gpsFix ? "true" : "false", dataBytesOut,
           (unsigned long)millis(), now_epoch, tsBuf,
           timeSynced ? "true" : "false");

  // Build path with snprintf
  char hwPath[64];
  snprintf(hwPath, sizeof(hwPath), "/hardware/%s.json", HARDWARE_ID);

  bool hwOK = httpPutWithRetry(hwPath, hardwareJson);
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
    Serial.printf("[RELAY] Supabase upload FAILED (HTTP %d)\n", httpStatus);
    return false;
  }

  // ==================== DELIVERY CONTEXT READER (Firebase -> cached)
  // ====================
  void refreshDeliveryContextFromFirebase() {
    if (!lteConnected)
      return;

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
    while (millis() - waitStart < 15000) {
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

    if (actionOK && httpStatus == 200) {
      // Read a larger chunk to ensure we capture both fields from the JSON
      modem.sendAT("+HTTPREAD=0,1024");
      resp = "";
      modem.waitResponse(5000L, resp);

      // Parse otp_code
      int otpIdx = resp.indexOf("\"otp_code\":\"");
      if (otpIdx >= 0) {
        int start = otpIdx + 12;
        int end = resp.indexOf('"', start);
        if (end > start) {
          int len = end - start;
          if (len > 0 && len <= 6) {
            strncpy(cachedOtp, resp.c_str() + start, len);
            cachedOtp[len] = '\0';
            otpCacheValid = true;
          }
        }
      } else {
        otpCacheValid = false;
      }

      // Parse delivery_id
      int delIdx = resp.indexOf("\"delivery_id\":\"");
      if (delIdx >= 0) {
        int start = delIdx + 15;
        int end = resp.indexOf('"', start);
        if (end > start) {
          int len = end - start;
          if (len > 0 && len < sizeof(cachedDeliveryId)) {
            strncpy(cachedDeliveryId, resp.c_str() + start, len);
            cachedDeliveryId[len] = '\0';
            deliveryIdCacheValid = true;
          }
        }
      } else {
        deliveryIdCacheValid = false;
      }

      Serial.printf("[CONTEXT] OTP: %s | Delivery ID: %s\n",
                    otpCacheValid ? cachedOtp : "NONE",
                    deliveryIdCacheValid ? cachedDeliveryId : "NONE");
    }

    sendATAndWait("+HTTPTERM", 1000);
  }

  // ==================== LOCK EVENT WRITER ====================
  void writeLockEventToFirebase(bool otpValid, bool faceDetected,
                                bool unlocked) {
    if (!lteConnected) {
      Serial.println("[EVENT] LTE not connected");
      return;
    }

    unsigned long now_epoch = getCurrentEpoch();
    char tsBuf[30];
    String tsStr = formatTimestamp(now_epoch);
    strncpy(tsBuf, tsStr.c_str(), sizeof(tsBuf) - 1);
    tsBuf[sizeof(tsBuf) - 1] = '\0';

    char eventJson[256];
    snprintf(eventJson, sizeof(eventJson),
             "{\"otp_valid\":%s,\"face_detected\":%s,\"unlocked\":%s,"
             "\"timestamp\":{\".sv\":\"timestamp\"},\"device_epoch\":%lu,"
             "\"timestamp_str\":\"%s\"}",
             otpValid ? "true" : "false", faceDetected ? "true" : "false",
             unlocked ? "true" : "false", now_epoch, tsBuf);

    char eventPath[64];
    snprintf(eventPath, sizeof(eventPath), "/lock_events/%s/latest.json",
             HARDWARE_ID);
    bool ok = httpPutWithRetry(eventPath, eventJson);
    Serial.printf("[EVENT] lock_events: %s\n", ok ? "OK" : "FAIL");

    const char *newStatus = unlocked ? "UNLOCKING" : "LOCKED";
    char statusJson[64];
    snprintf(statusJson, sizeof(statusJson), "\"%s\"", newStatus);
    char statusPath[64];
    snprintf(statusPath, sizeof(statusPath), "/hardware/%s/status.json",
             HARDWARE_ID);
    httpPutWithRetry(statusPath, statusJson);
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

  // ==================== HOTSPOT HTTP ROUTER ====================
  void handleCameraClient() {
    WiFiClient client = camServer.available();
    if (!client)
      return;

    Serial.println("[AP] Client connected");
    esp_task_wdt_reset();

    IPAddress clientAddr = client.remoteIP();

    unsigned long headerTimeout = millis() + 8000;
    while (!client.available() && millis() < headerTimeout) {
      delay(10);
      esp_task_wdt_reset();
    }
    if (!client.available()) {
      client.stop();
      return;
    }

    client.setTimeout(5000);
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
        "esp32cam/OV3660_CAM_001/relay_" + String(millis()) + ".jpg";

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

    // â”€â”€ GET /otp â”€â”€
    if (strcmp(method, "GET") == 0 && reqPathStr.startsWith("/otp")) {
      Serial.println("[AP] -> GET /otp");
      String otpPart = otpCacheValid ? String(cachedOtp) : "NO_OTP";
      String delPart =
          deliveryIdCacheValid ? String(cachedDeliveryId) : "NO_DELIVERY";
      String body = otpPart + "," + delPart;
      String resp =
          "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: " +
          String(body.length()) + "\r\n\r\n" + body;
      client.print(resp);
      delay(50);
      client.stop();
      return;
    }

    // â”€â”€ GET /face-check â”€â”€
    if (strcmp(method, "GET") == 0 && reqPathStr.startsWith("/face-check")) {
      Serial.println("[AP] -> GET /face-check");
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
      delay(50);
      client.stop();
      return;
    }

    // â”€â”€ POST /event â”€â”€
    if (strcmp(method, "POST") == 0 && strcmp(reqPath, "/event") == 0) {
      Serial.println("[AP] -> POST /event");
      char jsonBuf[256] = "";
      if (contentLength > 0 && contentLength < sizeof(jsonBuf)) {
        client.setTimeout(5000);
        size_t rd = client.readBytes(jsonBuf, contentLength);
        jsonBuf[rd] = '\0';
      }
      Serial.printf("[AP] Event: %s\n", jsonBuf);
      bool ov = (strstr(jsonBuf, "\"otp_valid\":true") != NULL);
      bool fd = (strstr(jsonBuf, "\"face_detected\":true") != NULL);
      bool ul = (strstr(jsonBuf, "\"unlocked\":true") != NULL);
      writeLockEventToFirebase(ov, fd, ul);
      String resp = "HTTP/1.1 200 OK\r\nContent-Length: 2\r\n\r\nOK";
      client.print(resp);
      delay(50);
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
      delay(50);
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
          failReason = "FAIL:incomplete:" + String(received) + "/" +
                       String(contentLength);
        }
        free(buf);
      }
    }

    String body = uploadOK ? ("OK:" + relayDiag) : failReason;
    String resp = uploadOK ? "HTTP/1.1 200 OK\r\n"
                           : "HTTP/1.1 500 Internal Server Error\r\n";
    resp += "Content-Length: " + String(body.length()) + "\r\n\r\n" + body;
    client.print(resp);
    delay(50);
    client.stop();
    Serial.println("[AP] Done: " + body);
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

    // Initialize modem with proper power sequence
    modemSerial.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
    powerModem();

    modemOK = initModem();

    if (!modemOK) {
      Serial.println("\n*** MODEM FAILED - Cannot proceed without LTE ***");
      Serial.println("  CHECK: 1) Modem hardware connected properly");
      Serial.println("         2) Power supply sufficient (>2A)");
      Serial.println("         3) SIM card inserted");
      while (1)
        delay(1000);
    }

    gpsEnabled = initGPS();
    Serial.println(gpsEnabled ? "GPS: Ready" : "GPS: Failed (continuing)");

    Serial.println("\nConnecting via LTE...");
    if (!connectLTE()) {
      Serial.println("\n*** LTE CONNECTION FAILED ***");
      Serial.println("  CHECK: 1) SIM has active data plan");
      Serial.println("         2) LTE antenna connected");
      Serial.println("         3) Signal coverage in area");
      while (1)
        delay(1000);
    }

    // â”€â”€ Start WiFi hotspot so ESP32-CAM can connect and relay images via
    // LTE â”€â”€
    startHotspot();
    if (apStarted) {
      camServer.begin();
      Serial.printf("[AP] Camera proxy server listening on port %d\n",
                    CAM_SERVER_PORT);
      Serial.printf("[AP] ESP32-CAM should connect to SSID '%s' / '%s'\n",
                    AP_SSID, AP_PASS);
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
      Serial.println(
          "WARNING: Clock not synced â€” timestamps may be incorrect");
    }

    Serial.println("\n=== Ready (LTE Only - AT+HTTP) ===\n");

    printMemoryReport(); // Baseline memory snapshot
  }

  // ==================== LOOP ====================
  void loop() {
    esp_task_wdt_reset(); // Feed watchdog every loop iteration
    unsigned long now = millis();

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
          Serial.printf(
              "%s\n",
              udpBuf); // String already contains prefix [TESTER]/[CAM]
        }
      }
    }

    // Check LTE connection periodically
    if (now - lastSig >= SIGNAL_INTERVAL) {
      if (lteConnected && modemOK) {
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
    unsigned long gpsInterval =
        gpsFix ? GPS_INTERVAL_LOCKED : GPS_INTERVAL_ACQUIRING;
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

    // Re-sync network time periodically to prevent clock drift
    if (lteConnected && now - lastTimeSync >= TIME_SYNC_INTERVAL) {
      Serial.println("Periodic time re-sync...");
      syncNetworkTime();
      lastTimeSync = now;
    }

    // Send to Firebase
    if (lteConnected && now - lastFB >= FIREBASE_INTERVAL) {
      sendToFirebase();
      lastFB = now;
    }

    // Refresh cached Delivery Context from Firebase (for Tester ESP32 /otp
    // endpoint)
    if (lteConnected &&
        now - lastDeliveryContextRead >= DELIVERY_CONTEXT_READ_INTERVAL) {
      refreshDeliveryContextFromFirebase();
      lastDeliveryContextRead = now;
    }

    delay(100);
  }