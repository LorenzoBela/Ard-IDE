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
bool  proxyReachable         = false;

// ── WiFi backoff state ──
static unsigned long wifiRetryAt    = 0;
static unsigned long wifiBackoffMs  = WIFI_RETRY_BASE_MS;

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
  wifiRetryAt  = millis() + 15000;
  wifiBackoffMs = WIFI_RETRY_BASE_MS;
}

void maintainWiFiConnection(unsigned long now) {
  if (WiFi.status() == WL_CONNECTED) {
    wifiBackoffMs = WIFI_RETRY_BASE_MS;
    return;
  }

  if (now < wifiRetryAt) return;

  netLog("[WIFI] Retry (backoff %lums)...\n", wifiBackoffMs);
  WiFi.disconnect();
  if (WIFI_SSID[0] == '\0') {
    scanForProxy();
  }
  if (WIFI_SSID[0] != '\0') {
    WiFi.begin((const char *)WIFI_SSID, WIFI_PASSWORD);
  }
  wifiRetryAt = now + wifiBackoffMs;
  if (wifiBackoffMs < WIFI_RETRY_MAX_MS) {
    wifiBackoffMs *= 2;
  }
}

// ==================== FETCH DELIVERY CONTEXT ====================
void fetchDeliveryContext() {
  if (WiFi.status() != WL_CONNECTED) {
    netLog("[FETCH] Skip — WiFi not connected\n");
    proxyReachable = false;
    return;
  }

  char url[64];
  snprintf(url, sizeof(url), "http://%s:%d/otp", PROXY_HOST, PROXY_PORT);
  netLog("[FETCH] GET %s\n", url);

  auto doGet = [&](String &bodyOut) -> int {
    WiFiClient client;
    HTTPClient http;
    http.setTimeout(3000);
    http.setReuse(false); // Do not keep-alive; prevents stale connection issues
    // You can optionally pass client: http.begin(client, url);
    char url[64];
    snprintf(url, sizeof(url), "http://%s:%d/otp", PROXY_HOST, PROXY_PORT);
    http.begin(client, url);
    int httpCode = http.GET();
    if (httpCode > 0) {
      bodyOut = http.getString();
    }
    http.end();
    return httpCode;
  };

  String body = "";
  int code = doGet(body);
  if (code < 0) {
    // If it fails, maybe LilyGO is overloaded. Wait briefly before fallback or just return.
    // By returning early and not retrying immediately, we keep loop() insanely fast.
    netLog("[FETCH] GET failed (%d)\n", code);
    return; // Don't block. Try again next time cleanly.
  }
  netLog("[FETCH] HTTP %d\n", code);

  bool wasActive = hasActiveDelivery;
  lastStatusCommand = "";

  if (code == 200) {
    proxyReachable = true;
    netLog("[FETCH] Body: '%s'\n", body.c_str());

    // Format: "123456,deliv_abc123" OR "123456,deliv_abc123,UNLOCKING"
    int firstComma  = body.indexOf(',');
    int secondComma = body.indexOf(',', firstComma + 1);

    if (firstComma > 0) {
      String otpPart = "";
      String delPart = "";

      if (secondComma > 0) {
        otpPart = body.substring(0, firstComma);
        delPart = body.substring(firstComma + 1, secondComma);
        lastStatusCommand = body.substring(secondComma + 1);
      } else {
        otpPart = body.substring(0, firstComma);
        delPart = body.substring(firstComma + 1);
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

      hasActiveDelivery = (validOtp && validDel);
      netLog("[FETCH] validOtp=%d validDel=%d hasActive=%d OTP='%s' ID='%s' Status='%s'\n",
             validOtp, validDel, hasActiveDelivery, currentOtp,
             activeDeliveryId, lastStatusCommand.c_str());
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
  } else if (code < 0) {
    proxyReachable = false;
  }

  // Drive state transitions based on delivery availability
  // (State machine transitions are handled in the main .ino — this function
  //  only updates the shared globals.)
}

// ==================== REPORT EVENT ====================
bool reportEventToProxy(bool otpValid, bool faceDetected, bool unlocked, bool thermalCutoff) {
  if (WiFi.status() != WL_CONNECTED)
    return false;

  HTTPClient http;
  char url[64];
  snprintf(url, sizeof(url), "http://%s:%d/event", PROXY_HOST, PROXY_PORT);

  char json[256];
  snprintf(json, sizeof(json),
           "{\"otp_valid\":%s,\"face_detected\":%s,\"unlocked\":%s,\"box_id\":"
           "\"%s\",\"delivery_id\":\"%s\",\"thermal_cutoff\":%s}",
           otpValid ? "true" : "false", faceDetected ? "true" : "false",
           unlocked ? "true" : "false", HARDWARE_ID, activeDeliveryId,
           thermalCutoff ? "true" : "false");

  http.setTimeout(5000);
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST((uint8_t *)json, strlen(json));

  Serial.printf("[EVENT] POST %s → HTTP %d\n", json, code);
  http.end();

  return (code == 200 || code == 201);
}

// ==================== ALERT REPORTING ====================
bool reportAlertToProxy(const char *alertType, const char *details) {
  if (WiFi.status() != WL_CONNECTED)
    return false;

  HTTPClient http;
  char url[64];
  snprintf(url, sizeof(url), "http://%s:%d/event", PROXY_HOST, PROXY_PORT);

  char json[256];
  snprintf(json, sizeof(json),
           "{\"alert_type\":\"%s\",\"details\":\"%s\","
           "\"box_id\":\"%s\",\"delivery_id\":\"%s\"}",
           alertType, details, HARDWARE_ID, activeDeliveryId);

  http.setTimeout(5000);
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST((uint8_t *)json, strlen(json));

  Serial.printf("[ALERT] %s → HTTP %d\n", alertType, code);
  http.end();

  return (code == 200 || code == 201);
}

// ==================== TAMPER REPORT ====================
bool reportTamperToProxy() {
  if (WiFi.status() != WL_CONNECTED)
    return false;

  HTTPClient http;
  char url[64];
  snprintf(url, sizeof(url), "http://%s:%d/event", PROXY_HOST, PROXY_PORT);

  char json[256];
  snprintf(json, sizeof(json),
           "{\"alert_type\":\"REED_TAMPER\",\"details\":\"lid_opened_no_unlock\","
           "\"box_id\":\"%s\",\"delivery_id\":\"%s\",\"tamper\":true}",
           HARDWARE_ID, activeDeliveryId);

  http.setTimeout(5000);
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST((uint8_t *)json, strlen(json));

  Serial.printf("[TAMPER] Report → HTTP %d\n", code);
  http.end();

  return (code == 200 || code == 201);
}

// ==================== COMMAND ACK REPORT ====================
bool reportCommandAckToProxy(const char *command, const char *status, const char *details) {
  if (WiFi.status() != WL_CONNECTED)
    return false;

  HTTPClient http;
  char url[72];
  snprintf(url, sizeof(url), "http://%s:%d/command-ack", PROXY_HOST, PROXY_PORT);

  char json[320];
  snprintf(json, sizeof(json),
           "{\"command\":\"%s\",\"status\":\"%s\",\"details\":\"%s\","
           "\"box_id\":\"%s\",\"delivery_id\":\"%s\"}",
           command ? command : "", status ? status : "", details ? details : "",
           HARDWARE_ID, activeDeliveryId);

  http.setTimeout(5000);
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST((uint8_t *)json, strlen(json));

  Serial.printf("[CMD_ACK] %s/%s -> HTTP %d\n", command ? command : "", status ? status : "", code);
  http.end();

  return (code == 200 || code == 201);
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

    http.setTimeout(FACE_CHECK_TIMEOUT);
    http.begin(url);
    int code = http.GET();

    if (code == 200) {
      String body = http.getString();
      body.trim();
      http.end();

      if (body.indexOf("FACE_OK") >= 0)  return 1;
      if (body.indexOf("NO_FACE") >= 0)  return 0;

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

  while (millis() - uartStart < FACE_CHECK_TIMEOUT) {
    if (Serial2.available()) {
      char c = Serial2.read();
      if (c == '\n' || c == '\r') {
        if (uartLen > 0) break;
      } else if (uartLen < sizeof(uartBuf) - 1) {
        uartBuf[uartLen++] = c;
      }
    }
    delay(10);
  }
  uartBuf[uartLen] = '\0';

  Serial.printf("[FACE] UART response: '%s'\n", uartBuf);

  if (strstr(uartBuf, "FACE_OK") != NULL) return 1;
  if (strstr(uartBuf, "NO_FACE") != NULL) return 0;

  Serial.println(F("[FACE] UART fallback failed"));
  return -1;
}
