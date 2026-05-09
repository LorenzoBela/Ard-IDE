/**
 * ProxyClient.cpp — HTTP communication with the LilyGO proxy
 *
 * Rules enforced:
 *   - WiFi reconnect with exponential backoff (Article 2.4)
 *   - char[] buffers for JSON payloads (Article 2.2)
 *   - No delay() in any function callable from loop() (Article 2.1)
 */

#include "ProxyClient.h"
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ── Box identity (populated by scanForProxy()) ──
char WIFI_SSID[24]    = "";
char HARDWARE_ID[12]  = "";

// ── UDP logger ──
static WiFiUDP udpClient;

// ── Delivery context ──
char  currentOtp[8]          = "";
char  activeDeliveryId[64]   = "";
bool  hasActiveDelivery      = false;
String lastStatusCommand     = "";
int16_t geoDistMeters        = -1;
bool    geoInsideFence       = false;
bool    isReturning          = false;
char    deliveryPhase[12]    = "NONE";
bool    pickupInsideFence    = false;
bool    dropoffInsideFence   = false;

// ── WiFi backoff state ──
static unsigned long wifiRetryAt    = 0;
static unsigned long wifiBackoffMs  = WIFI_RETRY_BASE_MS;
static unsigned long wifiRescanAt   = 0;

// ── Fetch reliability controls ──
static const uint16_t FETCH_HTTP_TIMEOUT_MS = 3000;

static void hardResetDeliveryContext(const char *reason) {
  if (!hasActiveDelivery && currentOtp[0] == '\0' && activeDeliveryId[0] == '\0' &&
      lastStatusCommand.length() == 0) {
    return;
  }

  currentOtp[0] = '\0';
  activeDeliveryId[0] = '\0';
  hasActiveDelivery = false;
  lastStatusCommand = "";
  geoDistMeters = -1;
  geoInsideFence = false;
  isReturning = false;
  strncpy(deliveryPhase, "NONE", sizeof(deliveryPhase) - 1);
  deliveryPhase[sizeof(deliveryPhase) - 1] = '\0';
  pickupInsideFence = false;
  dropoffInsideFence = false;
  netLog("[FETCH] Hard reset delivery context (%s)\n",
         reason ? reason : "unknown");
}

static String normalizeStatusToken(const String &rawToken) {
  String token = rawToken;
  token.trim();

  // Some upstream paths may include JSON quotes around text values.
  if (token.length() >= 2 && token[0] == '"' && token[token.length() - 1] == '"') {
    token = token.substring(1, token.length() - 1);
    token.trim();
  }

  if (token == "UNLOCKING" || token == "LOCKED" || token == "REBOOT_ALL") {
    return token;
  }

  return "";
}

static bool parseDiagField(const String &body, const char *key, String &out) {
  String token = String(key) + "=";
  int start = body.indexOf(token);
  if (start < 0) {
    return false;
  }

  start += token.length();
  int end = body.indexOf(',', start);
  if (end < 0) {
    end = body.length();
  }

  out = body.substring(start, end);
  out.trim();
  return out.length() > 0;
}

// ==================== LOGGING ====================
void netLog(const char *format, ...) {
  char buf[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  Serial.print(buf);

  if (WiFi.status() == WL_CONNECTED) {
    udpClient.beginPacket(PROXY_HOST, UDP_LOG_PORT);
    udpClient.print("[ESP32] ");
    udpClient.print(buf);
    udpClient.endPacket();
  }
}

// ==================== WIFI ====================
bool scanForProxy() {
  Serial.println("[WIFI] Scanning for SmartTopBox_AP_*...");
  int n = WiFi.scanNetworks();
  Serial.printf("[WIFI] Found %d networks\n", n);

  int bestIdx = -1;
  int bestRssi = -999;
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    Serial.printf("[WIFI]   %s (%d dBm)\n", ssid.c_str(), WiFi.RSSI(i));
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
    snprintf(HARDWARE_ID, sizeof(HARDWARE_ID), "BOX_%03d", num);

    Serial.printf("[WIFI] Selected: %s (%d dBm) -> %s\n",
                  WIFI_SSID, bestRssi, HARDWARE_ID);
    WiFi.scanDelete();
    return true;
  }

  WiFi.scanDelete();
  Serial.println("[WIFI] No SmartTopBox_AP_* found!");
  return false;
}

void startWiFiConnection() {
  WiFi.mode(WIFI_STA);
  // SoftAP link is local and latency-sensitive; disable modem sleep to reduce
  // intermittent HTTP read timeouts (e.g., HTTP -11) on frequent /otp polling.
  WiFi.setSleep(false);
  if (WIFI_SSID[0] == '\0') {
    scanForProxy();
  }
  if (WIFI_SSID[0] != '\0') {
    WiFi.begin((const char *)WIFI_SSID, WIFI_PASSWORD);
  }
  wifiRetryAt  = millis() + WIFI_FIRST_RETRY_DELAY_MS;
  wifiRescanAt = millis() + WIFI_RESCAN_INTERVAL_MS;
  wifiBackoffMs = WIFI_RETRY_BASE_MS;
}

// Forward declaration — defined below with persistent connection helpers.
static void closePersistentConnection();

void maintainWiFiConnection(unsigned long now) {
  if (WiFi.status() == WL_CONNECTED) {
    wifiBackoffMs = WIFI_RETRY_BASE_MS;
    wifiRetryAt = now + WIFI_FIRST_RETRY_DELAY_MS;
    wifiRescanAt = now + WIFI_RESCAN_INTERVAL_MS;
    return;
  }

  if (now >= wifiRescanAt) {
    netLog("[WIFI] Lost link; rescanning AP list...\n");
    scanForProxy();
    wifiRescanAt = now + WIFI_RESCAN_INTERVAL_MS;
  }

  if (now < wifiRetryAt) return;

  netLog("[WIFI] Retry (backoff %lums)...\n", wifiBackoffMs);
  closePersistentConnection(); // Kill stale TCP before WiFi reset
  WiFi.disconnect();
  if (WIFI_SSID[0] == '\0') {
    scanForProxy();
  }
  if (WIFI_SSID[0] != '\0') {
    WiFi.begin((const char *)WIFI_SSID, WIFI_PASSWORD);
  }

  unsigned long jitter = (unsigned long)random(0, WIFI_RETRY_JITTER_MS + 1);
  wifiRetryAt = now + wifiBackoffMs + jitter;
  if (wifiBackoffMs < WIFI_RETRY_MAX_MS) {
    wifiBackoffMs *= 2;
    if (wifiBackoffMs > WIFI_RETRY_MAX_MS) {
      wifiBackoffMs = WIFI_RETRY_MAX_MS;
    }
  }
}

// ── Persistent connection for high-frequency /otp polling ──
// Reusing the TCP socket across polls eliminates ~100-200ms of
// TCP handshake overhead per request on the local SoftAP link.
static WiFiClient persistentClient;
static bool       persistentConnected = false;

static void closePersistentConnection() {
  if (persistentConnected) {
    persistentClient.stop();
    persistentConnected = false;
  }
}

static bool ensurePersistentConnection() {
  if (persistentConnected && persistentClient.connected()) {
    return true;
  }
  // Tear down stale socket before reconnecting.
  persistentClient.stop();
  persistentConnected = false;

  persistentClient.setTimeout(FETCH_HTTP_TIMEOUT_MS / 1000); // seconds
  if (!persistentClient.connect(PROXY_HOST, PROXY_PORT)) {
    netLog("[FETCH] TCP connect failed\n");
    return false;
  }
  persistentConnected = true;
  return true;
}

/** Read one complete HTTP response from the persistent socket.
 *  Returns the HTTP status code (e.g. 200), or -1 on error.
 *  Writes the response body into bodyOut. */
static int readHttpResponse(String &bodyOut, unsigned long timeoutMs) {
  unsigned long deadline = millis() + timeoutMs;
  int statusCode = -1;
  int contentLength = -1;
  bool headersDone = false;
  bodyOut = "";

  // ── Read status line + headers ──
  while (millis() < deadline) {
    if (!persistentClient.connected()) return -1;
    if (!persistentClient.available()) {
      yield();
      continue;
    }
    String line = persistentClient.readStringUntil('\n');
    line.trim();

    if (statusCode < 0) {
      // Parse "HTTP/1.1 200 OK"
      if (line.startsWith("HTTP/")) {
        int spaceIdx = line.indexOf(' ');
        if (spaceIdx > 0) {
          statusCode = line.substring(spaceIdx + 1).toInt();
        }
      }
      continue;
    }

    if (line.length() == 0) {
      headersDone = true;
      break;
    }

    // Parse Content-Length header
    if (line.startsWith("Content-Length:") || line.startsWith("content-length:")) {
      contentLength = line.substring(line.indexOf(':') + 1).toInt();
    }
  }

  if (!headersDone || statusCode < 0) return -1;

  // ── Read body ──
  if (contentLength > 0) {
    char buf[256];
    int toRead = (contentLength < (int)sizeof(buf) - 1) ? contentLength : (int)sizeof(buf) - 1;
    int rd = 0;
    while (rd < toRead && millis() < deadline) {
      if (persistentClient.available()) {
        buf[rd++] = persistentClient.read();
      } else {
        yield();
      }
    }
    buf[rd] = '\0';
    bodyOut = String(buf);
  } else if (contentLength == 0) {
    bodyOut = "";
  } else {
    // No Content-Length header — read until timeout or connection close.
    // This is a fallback; the proxy should always send Content-Length.
    while (millis() < deadline && persistentClient.connected()) {
      if (persistentClient.available()) {
        bodyOut += (char)persistentClient.read();
      } else {
        // Brief idle gap — if we already have data, assume body is done.
        if (bodyOut.length() > 0) break;
        yield();
      }
    }
  }

  return statusCode;
}

// How many consecutive fetch failures before wiping the delivery context.
// At ~1Hz polling this gives ~5s of tolerance for a brief WiFi blip.
static uint8_t fetchFailCount = 0;
static const uint8_t FETCH_FAIL_RESET_THRESHOLD = 5;

void fetchDeliveryContext() {
  if (WiFi.status() != WL_CONNECTED) {
    netLog("[FETCH] Skip — WiFi not connected\n");
    closePersistentConnection();
    return;
  }

  // Ensure persistent TCP socket is alive.
  if (!ensurePersistentConnection()) {
    fetchFailCount++;
    netLog("[FETCH] TCP connect failed (%u/%u)\n", fetchFailCount, FETCH_FAIL_RESET_THRESHOLD);
    if (fetchFailCount >= FETCH_FAIL_RESET_THRESHOLD) {
      fetchFailCount = 0;
      hardResetDeliveryContext("tcp_connect_failed");
    }
    return;
  }

  // Send raw HTTP/1.1 GET with keep-alive.
  persistentClient.printf(
      "GET /otp HTTP/1.1\r\n"
      "Host: %s:%d\r\n"
      "Connection: keep-alive\r\n"
      "\r\n",
      PROXY_HOST, PROXY_PORT);

  String body = "";
  int code = readHttpResponse(body, FETCH_HTTP_TIMEOUT_MS);

  if (code < 0) {
    netLog("[FETCH] Read failed — reconnecting\n");
    closePersistentConnection();
    // One retry with fresh connection
    if (!ensurePersistentConnection()) {
      fetchFailCount++;
      netLog("[FETCH] TCP retry failed (%u/%u)\n", fetchFailCount, FETCH_FAIL_RESET_THRESHOLD);
      if (fetchFailCount >= FETCH_FAIL_RESET_THRESHOLD) {
        fetchFailCount = 0;
        hardResetDeliveryContext("tcp_retry_failed");
      }
      return;
    }
    persistentClient.printf(
        "GET /otp HTTP/1.1\r\n"
        "Host: %s:%d\r\n"
        "Connection: keep-alive\r\n"
        "\r\n",
        PROXY_HOST, PROXY_PORT);
    body = "";
    code = readHttpResponse(body, FETCH_HTTP_TIMEOUT_MS);
    if (code < 0) {
      fetchFailCount++;
      netLog("[FETCH] Retry failed (%u/%u)\n", fetchFailCount, FETCH_FAIL_RESET_THRESHOLD);
      closePersistentConnection();
      if (fetchFailCount >= FETCH_FAIL_RESET_THRESHOLD) {
        fetchFailCount = 0;
        hardResetDeliveryContext("http_get_failed");
      }
      return;
    }
  }

  netLog("[FETCH] HTTP %d\n", code);
  if (code != 200) {
    fetchFailCount++;
    netLog("[FETCH] Non-200 response (%u/%u)\n", fetchFailCount, FETCH_FAIL_RESET_THRESHOLD);
    if (fetchFailCount >= FETCH_FAIL_RESET_THRESHOLD) {
      fetchFailCount = 0;
      hardResetDeliveryContext("http_non_200");
    }
    return;
  }

  // Successful fetch — reset the failure counter.
  fetchFailCount = 0;

  bool wasActive = hasActiveDelivery;
  lastStatusCommand = "";
  geoDistMeters = -1;
  geoInsideFence = false;
  isReturning = false;
  strncpy(deliveryPhase, "NONE", sizeof(deliveryPhase) - 1);
  deliveryPhase[sizeof(deliveryPhase) - 1] = '\0';
  pickupInsideFence = false;
  dropoffInsideFence = false;

  if (code == 200) {
    netLog("[FETCH] Body: '%s'\n", body.c_str());

    // New format: OTP,DELIVERY_ID,STATUS,DIST:N,GEO:0|1,RET:0|1,PHASE:*,PUP:0|1,DRO:0|1
    // Fields are comma-separated. STATUS may be empty. DIST/GEO/RET are
    // key:value pairs appended after the first 3 positional fields.
    int firstComma  = body.indexOf(',');
    int secondComma = (firstComma > 0) ? body.indexOf(',', firstComma + 1) : -1;
    int thirdComma  = (secondComma > 0) ? body.indexOf(',', secondComma + 1) : -1;

    if (firstComma > 0) {
      String otpPart = body.substring(0, firstComma);
      String delPart;
      String statusPart = "";

      if (secondComma > 0) {
        delPart = body.substring(firstComma + 1, secondComma);
        if (thirdComma > 0) {
          statusPart = body.substring(secondComma + 1, thirdComma);
        } else {
          statusPart = body.substring(secondComma + 1);
        }
      } else {
        delPart = body.substring(firstComma + 1);
      }

      // Parse admin command from STATUS field
      lastStatusCommand = normalizeStatusToken(statusPart);

      // Parse structured fields (DIST:N, GEO:0|1, RET:0|1) from remaining body
      int distIdx = body.indexOf("DIST:");
      if (distIdx >= 0) {
        int valStart = distIdx + 5;
        int valEnd = body.indexOf(',', valStart);
        if (valEnd < 0) valEnd = body.length();
        geoDistMeters = (int16_t)body.substring(valStart, valEnd).toInt();
      }
      int geoIdx = body.indexOf("GEO:");
      if (geoIdx >= 0) {
        geoInsideFence = (body.charAt(geoIdx + 4) == '1');
      }
      int retIdx = body.indexOf("RET:");
      if (retIdx >= 0) {
        isReturning = (body.charAt(retIdx + 4) == '1');
      }

      // Parse phase + pickup/dropoff fence flags (optional, fail-closed)
      bool phaseParsed = false;
      int phaseIdx = body.indexOf("PHASE:");
      if (phaseIdx >= 0) {
        int valStart = phaseIdx + 6;
        int valEnd = body.indexOf(',', valStart);
        if (valEnd < 0) valEnd = body.length();
        String phase = body.substring(valStart, valEnd);
        phase.trim();
        if (phase.length() > 0 && phase.length() < (int)sizeof(deliveryPhase)) {
          strncpy(deliveryPhase, phase.c_str(), sizeof(deliveryPhase) - 1);
          deliveryPhase[sizeof(deliveryPhase) - 1] = '\0';
          phaseParsed = true;
        }
      }

      int pupIdx = body.indexOf("PUP:");
      if (pupIdx >= 0) {
        pickupInsideFence = (body.charAt(pupIdx + 4) == '1');
      }
      int droIdx = body.indexOf("DRO:");
      if (droIdx >= 0) {
        dropoffInsideFence = (body.charAt(droIdx + 4) == '1');
      }

      if (!phaseParsed) {
        strncpy(deliveryPhase, "NONE", sizeof(deliveryPhase) - 1);
        deliveryPhase[sizeof(deliveryPhase) - 1] = '\0';
      }

      bool validOtp = (otpPart != "NO_OTP" && otpPart != "null" &&
                       otpPart.length() > 0 && otpPart.length() <= 6);
      bool validDel = (delPart != "NO_DELIVERY" && delPart != "null" &&
                       delPart.length() > 0);

      if (validOtp) {
        strncpy(currentOtp, otpPart.c_str(), sizeof(currentOtp) - 1);
        currentOtp[sizeof(currentOtp) - 1] = '\0';
      } else {
        currentOtp[0] = '\0';
      }
      if (validDel) {
        strncpy(activeDeliveryId, delPart.c_str(), sizeof(activeDeliveryId) - 1);
        activeDeliveryId[sizeof(activeDeliveryId) - 1] = '\0';
      } else {
        activeDeliveryId[0] = '\0';
      }

      hasActiveDelivery = validDel;
      if (!hasActiveDelivery) {
        strncpy(deliveryPhase, "NONE", sizeof(deliveryPhase) - 1);
        deliveryPhase[sizeof(deliveryPhase) - 1] = '\0';
        pickupInsideFence = false;
        dropoffInsideFence = false;
      }
            netLog("[FETCH] otp=%d del=%d dist=%d geo=%d ret=%d phase=%s pup=%d dro=%d cmd='%s'\n",
              validOtp, validDel, geoDistMeters, geoInsideFence ? 1 : 0,
              isReturning ? 1 : 0, deliveryPhase,
              pickupInsideFence ? 1 : 0, dropoffInsideFence ? 1 : 0,
              lastStatusCommand.c_str());
    } else {
      // Legacy format (just OTP, no delivery_id)
      if (body != "NO_OTP" && body != "null" && body.length() > 0 &&
          body.length() <= 6) {
        strncpy(currentOtp, body.c_str(), sizeof(currentOtp) - 1);
        currentOtp[sizeof(currentOtp) - 1] = '\0';
        hasActiveDelivery = true;
      } else {
        currentOtp[0] = '\0';
        hasActiveDelivery = false;
      }
    }
  }

  // Drive state transitions based on delivery availability
  // (State machine transitions are handled in the main .ino — this function
  //  only updates the shared globals.)
}

bool requestContextRefresh() {
  if (WiFi.status() != WL_CONNECTED) {
    netLog("[REFRESH] Skip — WiFi not connected\n");
    closePersistentConnection();
    return false;
  }

  if (!ensurePersistentConnection()) {
    netLog("[REFRESH] TCP connect failed\n");
    return false;
  }

  persistentClient.printf(
      "GET /refresh-context HTTP/1.1\r\n"
      "Host: %s:%d\r\n"
      "Connection: keep-alive\r\n"
      "\r\n",
      PROXY_HOST, PROXY_PORT);

  String body = "";
  int code = readHttpResponse(body, FETCH_HTTP_TIMEOUT_MS);
  if (code < 0) {
    netLog("[REFRESH] Read failed — reconnecting\n");
    closePersistentConnection();
    if (!ensurePersistentConnection()) {
      netLog("[REFRESH] TCP retry failed\n");
      return false;
    }
    persistentClient.printf(
        "GET /refresh-context HTTP/1.1\r\n"
        "Host: %s:%d\r\n"
        "Connection: keep-alive\r\n"
        "\r\n",
        PROXY_HOST, PROXY_PORT);
    body = "";
    code = readHttpResponse(body, FETCH_HTTP_TIMEOUT_MS);
    if (code < 0) {
      netLog("[REFRESH] Retry failed\n");
      closePersistentConnection();
      return false;
    }
  }

  body.trim();
  netLog("[REFRESH] HTTP %d body='%s'\n", code, body.c_str());
  return (code == 200 && body.startsWith("OK"));
}

bool fetchDiagnostics(ControllerDiagData &out) {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  WiFiClient client;
  HTTPClient http;
  char url[64];
  snprintf(url, sizeof(url), "http://%s:%d/diag", PROXY_HOST, PROXY_PORT);

  http.setTimeout(CONTROLLER_DIAG_HTTP_TIMEOUT_MS);
  http.setReuse(false);
  http.begin(client, url);

  int code = http.GET();
  if (code != 200) {
    netLog("[DIAG] GET /diag HTTP %d\n", code);
    http.end();
    return false;
  }

  String body = http.getString();
  http.end();

  String field;
  ControllerDiagData parsed = out;
  bool parsedAny = false;

  if (parseDiagField(body, "batt_pct", field)) {
    parsed.battPct = field.toInt();
    parsedAny = true;
  }
  if (parseDiagField(body, "batt_v", field)) {
    parsed.battVoltage = field.toFloat();
    parsedAny = true;
  }
  if (parseDiagField(body, "rssi", field)) {
    parsed.rssi = field.toInt();
    parsedAny = true;
  }
  if (parseDiagField(body, "csq", field)) {
    parsed.csq = field.toInt();
    parsedAny = true;
  }
  if (parseDiagField(body, "gps_fix", field)) {
    parsed.gpsFix = (field.toInt() != 0);
    parsedAny = true;
  }
  if (parseDiagField(body, "lte", field)) {
    parsed.lteConnected = (field.toInt() != 0);
    parsedAny = true;
  }
  if (parseDiagField(body, "modem", field)) {
    parsed.modemOk = (field.toInt() != 0);
    parsedAny = true;
  }
  if (parseDiagField(body, "time", field)) {
    parsed.timeSynced = (field.toInt() != 0);
    parsedAny = true;
  }
  if (parseDiagField(body, "cam_up", field)) {
    parsed.camUp = (field.toInt() != 0);
    parsedAny = true;
  }
  if (parseDiagField(body, "ctrl_up", field)) {
    parsed.controllerUp = (field.toInt() != 0);
    parsedAny = true;
  }
  if (parseDiagField(body, "fb_fail", field)) {
    parsed.firebaseFailures = field.toInt();
    parsedAny = true;
  }
  if (parseDiagField(body, "cmd_stage", field)) {
    parsed.commandStage = field.toInt();
    parsedAny = true;
  }
  if (parseDiagField(body, "cmd_pending", field)) {
    parsed.commandPending = (field.toInt() != 0);
    parsedAny = true;
  }
  if (parseDiagField(body, "conn_state", field)) {
    parsed.connectivityState = field.toInt();
    parsedAny = true;
  }
  if (parseDiagField(body, "lte_reconn_ms", field)) {
    parsed.lteReconnectMs = (unsigned long)field.toInt();
    parsedAny = true;
  }
  if (parseDiagField(body, "upload_active", field)) {
    parsed.photoUploadActive = (field.toInt() != 0);
    parsedAny = true;
  }
  if (parseDiagField(body, "upload_pct", field)) {
    parsed.photoUploadProgress = field.toInt();
    parsedAny = true;
  }
  if (parseDiagField(body, "upload_age", field)) {
    parsed.photoUploadAgeMs = (unsigned long)field.toInt();
    parsedAny = true;
  }
  if (parseDiagField(body, "cam_age", field)) {
    parsed.camAgeMs = (unsigned long)field.toInt();
    parsedAny = true;
  }
  if (parseDiagField(body, "ctrl_age", field)) {
    parsed.controllerAgeMs = (unsigned long)field.toInt();
    parsedAny = true;
  }
  if (parseDiagField(body, "uptime", field)) {
    parsed.proxyUptimeMs = (unsigned long)field.toInt();
    parsedAny = true;
  }

  if (!parsedAny) {
    netLog("[DIAG] GET /diag parse miss: '%s'\n", body.c_str());
    return false;
  }

  if (parsed.battPct < 0) parsed.battPct = 0;
  if (parsed.battPct > 100) parsed.battPct = 100;
  if (parsed.csq < -1) parsed.csq = -1;
  if (parsed.csq > 31) parsed.csq = 31;
  if (parsed.firebaseFailures < 0) parsed.firebaseFailures = 0;
  if (parsed.commandStage < 0) parsed.commandStage = 0;
  if (parsed.photoUploadProgress < 0) parsed.photoUploadProgress = 0;
  if (parsed.photoUploadProgress > 100) parsed.photoUploadProgress = 100;

  out = parsed;
  return true;
}

// ── Generic raw POST through persistent connection ──
// Reuses the keep-alive socket for fire-and-forget POSTs during the
// unlock flow, avoiding ~150ms TCP handshake per call.
static int sendRawPost(const char *path, const char *json) {
  if (WiFi.status() != WL_CONNECTED) return -1;
  if (!ensurePersistentConnection()) return -1;

  int jsonLen = strlen(json);
  persistentClient.printf(
      "POST %s HTTP/1.1\r\n"
      "Host: %s:%d\r\n"
      "Connection: keep-alive\r\n"
      "Content-Type: application/json\r\n"
      "Content-Length: %d\r\n"
      "\r\n"
      "%s",
      path, PROXY_HOST, PROXY_PORT, jsonLen, json);

  String body;
  int code = readHttpResponse(body, FETCH_HTTP_TIMEOUT_MS);

  if (code < 0) {
    netLog("[POST] %s failed — reconnecting\n", path);
    closePersistentConnection();
    if (!ensurePersistentConnection()) return -1;
    persistentClient.printf(
        "POST %s HTTP/1.1\r\n"
        "Host: %s:%d\r\n"
        "Connection: keep-alive\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "\r\n"
        "%s",
        path, PROXY_HOST, PROXY_PORT, jsonLen, json);
    code = readHttpResponse(body, FETCH_HTTP_TIMEOUT_MS);
    if (code < 0) {
      closePersistentConnection();
      return -1;
    }
  }
  return code;
}

// ==================== REPORT EVENT ====================
bool reportEventToProxy(bool otpValid,
                        bool faceDetected,
                        bool unlocked,
                        bool thermalCutoff,
                        uint8_t faceAttempts,
                        bool faceRetryExhausted,
                        bool fallbackRequired,
                        const char *failureReason,
                        unsigned long unlockLatencyMs) {
  if (WiFi.status() != WL_CONNECTED)
    return false;

  const char *safeReason =
      (failureReason != NULL && failureReason[0] != '\0') ? failureReason : "";

  char json[448];
  snprintf(json, sizeof(json),
           "{\"otp_valid\":%s,\"face_detected\":%s,\"unlocked\":%s,\"box_id\":"
           "\"%s\",\"delivery_id\":\"%s\",\"thermal_cutoff\":%s,"
           "\"face_attempts\":%u,\"face_retry_exhausted\":%s,"
           "\"fallback_required\":%s,\"failure_reason\":\"%s\","
           "\"unlock_latency_ms\":%lu}",
           otpValid ? "true" : "false", faceDetected ? "true" : "false",
           unlocked ? "true" : "false", HARDWARE_ID, activeDeliveryId,
           thermalCutoff ? "true" : "false", (unsigned int)faceAttempts,
           faceRetryExhausted ? "true" : "false",
           fallbackRequired ? "true" : "false", safeReason,
           unlockLatencyMs);

  int code = sendRawPost("/event", json);
  Serial.printf("[EVENT] POST → HTTP %d\n", code);

  return (code == 200 || code == 201);
}

// ==================== ALERT REPORTING ====================
bool reportAlertToProxy(const char *alertType, const char *details) {
  if (WiFi.status() != WL_CONNECTED)
    return false;

  char json[256];
  snprintf(json, sizeof(json),
           "{\"alert_type\":\"%s\",\"details\":\"%s\","
           "\"box_id\":\"%s\",\"delivery_id\":\"%s\"}",
           alertType, details, HARDWARE_ID, activeDeliveryId);

  int code = sendRawPost("/event", json);
  Serial.printf("[ALERT] %s → HTTP %d\n", alertType, code);

  return (code == 200 || code == 201);
}

// ==================== TAMPER REPORT ====================
bool reportTamperToProxy() {
  if (WiFi.status() != WL_CONNECTED)
    return false;

  char json[256];
  snprintf(json, sizeof(json),
           "{\"alert_type\":\"REED_TAMPER\",\"details\":\"lid_opened_no_unlock\","
           "\"box_id\":\"%s\",\"delivery_id\":\"%s\",\"tamper\":true}",
           HARDWARE_ID, activeDeliveryId);

  int code = sendRawPost("/event", json);
  Serial.printf("[TAMPER] Report → HTTP %d\n", code);

  return (code == 200 || code == 201);
}

// ==================== COMMAND ACK REPORT ====================
bool reportCommandAckToProxy(const char *command, const char *status, const char *details) {
  if (WiFi.status() != WL_CONNECTED)
    return false;

  char json[320];
  snprintf(json, sizeof(json),
           "{\"command\":\"%s\",\"status\":\"%s\",\"details\":\"%s\","
           "\"box_id\":\"%s\",\"delivery_id\":\"%s\"}",
           command ? command : "", status ? status : "", details ? details : "",
           HARDWARE_ID, activeDeliveryId);

  int code = sendRawPost("/command-ack", json);
  Serial.printf("[CMD_ACK] %s/%s -> HTTP %d\n", command ? command : "", status ? status : "", code);

  return (code == 200 || code == 201);
}

bool requestCameraPowerMode(bool wakeMode) {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  HTTPClient http;
  char url[96];
  snprintf(url, sizeof(url), "http://%s:%d/cam-power?mode=%s", PROXY_HOST,
           PROXY_PORT, wakeMode ? "wake" : "sleep");

  http.setTimeout(2500);
  http.begin(url);
  int code = http.GET();

  bool ok = false;
  if (code == 200) {
    String body = http.getString();
    body.trim();
    ok = body.indexOf("CAM_") >= 0 || body.indexOf("OK") >= 0;
    netLog("[CAM] mode=%s proxy=%s\n", wakeMode ? "wake" : "sleep",
           body.c_str());
  } else {
    netLog("[CAM] mode=%s HTTP %d\n", wakeMode ? "wake" : "sleep", code);
  }

  http.end();
  return ok;
}

int requestPersonalPinToggle(const char *pin, bool currentlyLocked) {
  if (WiFi.status() != WL_CONNECTED || !pin || pin[0] == '\0') {
    return 0;
  }

  HTTPClient http;
  char url[96];
  snprintf(url, sizeof(url), "http://%s:%d/personal-pin-verify", PROXY_HOST,
           PROXY_PORT);

  char json[256];
  snprintf(json, sizeof(json),
           "{\"pin\":\"%s\",\"box_id\":\"%s\",\"delivery_id\":\"%s\","
           "\"currently_locked\":%s,\"source\":\"controller_keypad\"}",
           pin, HARDWARE_ID, activeDeliveryId, currentlyLocked ? "true" : "false");

  http.setTimeout(30000);
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST((uint8_t *)json, strlen(json));
  if (code != 200) {
    Serial.printf("[PIN] verify HTTP %d\n", code);
    http.end();
    return -2;
  }

  String body = http.getString();
  body.trim();
  http.end();

  if (body.indexOf("ALLOW_UNLOCK") >= 0) {
    return 1;
  }
  if (body.indexOf("ALLOW_RELOCK") >= 0) {
    return 2;
  }
  if (body.indexOf("DENY:disabled") >= 0 ||
      body.indexOf("DENY:missing_pin") >= 0 ||
      body.indexOf("DENY:invalid") >= 0 ||
      body.indexOf("DENY:sync_failed") >= 0 ||
      body.indexOf("DENY:box_mismatch") >= 0) {
    return -1;
  }
  return 0;
}

// ==================== FACE CHECK ====================
int requestFaceCheck() {
  // ── Primary: WiFi via proxy ──
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    char url[128];
    if (strlen(activeDeliveryId) > 0 &&
        strcmp(activeDeliveryId, "NO_DELIVERY") != 0 &&
        strcmp(activeDeliveryId, "null") != 0) {
      snprintf(url, sizeof(url), "http://%s:%d/face-check?delivery_id=%s",
               PROXY_HOST, PROXY_PORT, activeDeliveryId);
    } else {
      snprintf(url, sizeof(url), "http://%s:%d/face-check", PROXY_HOST,
               PROXY_PORT);
    }

    http.setTimeout(FACE_CHECK_WIFI_TIMEOUT_MS);
    http.begin(url);
    int code = http.GET();

    if (code == 200) {
      String body = http.getString();
      body.trim();
      http.end();

      if (body.indexOf("FACE_OK") >= 0) return 1;
      if (body.indexOf("NO_FACE_LOW_LIGHT") >= 0) return 2;
      if (body.indexOf("NO_FACE") >= 0) return 0;

      Serial.printf("[FACE] HTTP 200 but body error: %s\n", body.c_str());
    } else {
      Serial.printf("[FACE] HTTP %d\n", code);
      http.end();
    }
  }

  // ── Fallback: UART Serial2 to ESP32-CAM directly ──
  Serial.println(F("[FACE] WiFi down — trying UART fallback..."));

  // Flush stale data
  while (Serial2.available()) Serial2.read();

  if (strlen(activeDeliveryId) > 0 &&
      strcmp(activeDeliveryId, "NO_DELIVERY") != 0 &&
      strcmp(activeDeliveryId, "null") != 0) {
    Serial2.printf("FACE?,%s\n", activeDeliveryId);
  } else {
    Serial2.println("FACE?");
  }

  unsigned long uartStart = millis();
  char uartBuf[32] = "";
  uint8_t uartLen = 0;

  while (millis() - uartStart < FACE_CHECK_UART_TIMEOUT_MS) {
    if (Serial2.available()) {
      char c = Serial2.read();
      if (c == '\n' || c == '\r') {
        if (uartLen > 0) break;
      } else if (uartLen < sizeof(uartBuf) - 1) {
        uartBuf[uartLen++] = c;
      }
    }
    // Yield CPU without blocking scheduler; keep retry paths millis-driven.
    yield();
  }
  uartBuf[uartLen] = '\0';

  Serial.printf("[FACE] UART response: '%s'\n", uartBuf);

  if (strstr(uartBuf, "FACE_OK") != NULL) return 1;
  if (strstr(uartBuf, "NO_FACE_LOW_LIGHT") != NULL) return 2;
  if (strstr(uartBuf, "NO_FACE") != NULL) return 0;

  Serial.println(F("[FACE] UART fallback failed"));
  return -1;
}
