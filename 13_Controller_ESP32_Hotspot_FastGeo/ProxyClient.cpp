/**
 * ProxyClient.cpp — HTTP communication with the LilyGO proxy
 *
 * Rules enforced:
 *   - WiFi reconnect with exponential backoff (Article 2.4)
 *   - char[] buffers for JSON payloads (Article 2.2)
 *   - No delay() in any function callable from loop() (Article 2.1)
 */

#include "ProxyClient.h"
#include <esp_wifi.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#ifndef DISABLE_PROXY_UART
#define DISABLE_PROXY_UART 0
#endif

#ifndef DISABLE_PROXY_KEEPALIVE
#define DISABLE_PROXY_KEEPALIVE 0
#endif

#ifndef PROXY_UART_TIMEOUT_MS
#define PROXY_UART_TIMEOUT_MS 300
#endif

// ── Box identity (populated by scanForProxy()) ──
char WIFI_SSID[33]     = "";
char WIFI_PASSWORD[65] = "";
char HARDWARE_ID[12]   = "";
char PROXY_HOST[16]    = "";

// ── UDP logger ──
static WiFiUDP udpClient;
static HardwareSerial proxySerial(1);
static bool proxyUartStarted = false;
static SemaphoreHandle_t proxyUartMutex = NULL;
static SemaphoreHandle_t proxyIoMutex = NULL;
static QueueHandle_t proxyPostQueue = NULL;
static TaskHandle_t proxyPostTaskHandle = NULL;
static TaskHandle_t proxyGeoTaskHandle = NULL;
static uint16_t proxyRequestId = 1;
static unsigned long proxyUartRetryAt = 0;
static uint8_t proxyUartFailCount = 0;
static uint8_t proxyUartResetCount = 0;
static volatile uint32_t proxyPostQueuedCount = 0;
static volatile uint32_t proxyPostSentCount = 0;
static volatile uint32_t proxyPostFailCount = 0;
static volatile uint32_t proxyPostDropCount = 0;
static volatile uint32_t proxyIoBusyCount = 0;
static const char *volatile proxyIoOwner = "none";

struct ProxyPostJob {
  char path[32];
  char json[512];
  uint8_t attempts;
};

static void proxyPostTask(void *pvParameters);
static void proxyGeoTask(void *pvParameters);

static void ensureProxyAsync() {
  if (proxyIoMutex == NULL) {
    proxyIoMutex = xSemaphoreCreateRecursiveMutex();
  }
  if (proxyPostQueue == NULL) {
    proxyPostQueue = xQueueCreate(10, sizeof(ProxyPostJob));
  }
  if (proxyPostTaskHandle == NULL && proxyPostQueue != NULL) {
    xTaskCreatePinnedToCore(proxyPostTask, "ProxyPost", 6144, NULL, 1,
                            &proxyPostTaskHandle, 0);
  }
  if (proxyGeoTaskHandle == NULL) {
    xTaskCreatePinnedToCore(proxyGeoTask, "ProxyGeo", 6144, NULL, 1,
                            &proxyGeoTaskHandle, 0);
  }
}

static bool takeProxyIo(uint32_t timeoutMs, const char *owner) {
  ensureProxyAsync();
  if (proxyIoMutex == NULL) return true;
  if (xSemaphoreTakeRecursive(proxyIoMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
    proxyIoOwner = owner ? owner : "unknown";
    return true;
  }
  proxyIoBusyCount++;
  return false;
}

static void giveProxyIo() {
  if (proxyIoMutex == NULL) return;
  proxyIoOwner = "none";
  xSemaphoreGiveRecursive(proxyIoMutex);
}

class ProxyIoGuard {
public:
  ProxyIoGuard(const char *owner, uint32_t timeoutMs)
      : _locked(takeProxyIo(timeoutMs, owner)) {}
  ~ProxyIoGuard() {
    if (_locked) giveProxyIo();
  }
  bool ok() const { return _locked; }

private:
  bool _locked;
};

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
static unsigned long wifiConnectStartedAt = 0;
static uint8_t wifiReconnectAttempts = 0;
static uint8_t proxyBssid[6] = {0};
static bool proxyBssidKnown = false;
static int32_t proxyChannel = 0;
static uint8_t hotspotScanStart = 0;
static int8_t selectedHotspotCredential = -1;
static unsigned long nextProxyDiscoveryAt = 0;
static unsigned long nextProxySubnetProbeAt = 0;
static unsigned long proxyMissingReconnectAt = 0;
static uint8_t proxyDiscoveryMisses = 0;
static uint8_t subnetProbeHost = 2;
static const unsigned long PROXY_DISCOVERY_FAST_MS = 700UL;
static const unsigned long PROXY_SUBNET_PROBE_FAST_MS = 1000UL;
static const unsigned long PROXY_RECONNECT_DELAY_MS = 1000UL;
static const unsigned long PROXY_PROBE_TIMEOUT_MS = 90UL;
static const uint8_t PROXY_NEARBY_RADIUS = 8;
static const uint8_t PROXY_SUBNET_PROBE_HOSTS = 18;
static const uint8_t PROXY_RECONNECT_MISS_LIMIT = 8;

static void clearDiscoveredProxy(const char *reason) {
  if (PROXY_HOST[0] != '\0') {
    Serial.printf("[DISCOVERY] Clearing proxy %s (%s)\n",
                  PROXY_HOST, reason ? reason : "unknown");
  }
  PROXY_HOST[0] = '\0';
  nextProxyDiscoveryAt = 0;
}

static void rememberDiscoveredProxy(const IPAddress &ip, const char *hardwareId,
                                    const char *source) {
  String ipStr = ip.toString();
  strncpy(PROXY_HOST, ipStr.c_str(), sizeof(PROXY_HOST) - 1);
  PROXY_HOST[sizeof(PROXY_HOST) - 1] = '\0';
  if (hardwareId && hardwareId[0] != '\0') {
    strncpy(HARDWARE_ID, hardwareId, sizeof(HARDWARE_ID) - 1);
    HARDWARE_ID[sizeof(HARDWARE_ID) - 1] = '\0';
  } else if (HARDWARE_ID[0] == '\0') {
    snprintf(HARDWARE_ID, sizeof(HARDWARE_ID), "BOX_001");
  }
  proxyDiscoveryMisses = 0;
  nextProxyDiscoveryAt = millis() + 5000UL;
  nextProxySubnetProbeAt = millis() + 15000UL;
  proxyMissingReconnectAt = 0;
  Serial.printf("[DISCOVERY] Proxy at %s:%d via %s\n",
                PROXY_HOST, PROXY_PORT, source ? source : "probe");
}

static bool pingProxyAt(const IPAddress &ip, unsigned long timeoutMs) {
  WiFiClient client;
  client.setTimeout(1);
  bool connected = false;
#if defined(ESP_ARDUINO_VERSION_MAJOR)
  connected = client.connect(ip, PROXY_PORT, (int32_t)timeoutMs);
#else
  connected = client.connect(ip, PROXY_PORT);
#endif
  if (!connected) {
    client.stop();
    return false;
  }

  client.printf(
      "GET /ping HTTP/1.1\r\n"
      "Host: %s:%d\r\n"
      "Connection: close\r\n"
      "\r\n",
      ip.toString().c_str(), PROXY_PORT);

  unsigned long deadline = millis() + timeoutMs;
  bool sawHttpOk = false;
  bool sawBodyOk = false;
  while (millis() < deadline && (client.connected() || client.available())) {
    if (!client.available()) {
      yield();
      continue;
    }
    String line = client.readStringUntil('\n');
    line.trim();
    if (line.startsWith("HTTP/1.1 200") || line.startsWith("HTTP/1.0 200")) {
      sawHttpOk = true;
    }
    if (line == "OK") {
      sawBodyOk = true;
      break;
    }
  }
  client.stop();
  return sawHttpOk || sawBodyOk;
}

static bool tryProxyCandidate(const IPAddress &ip, const char *source,
                              unsigned long timeoutMs = PROXY_PROBE_TIMEOUT_MS) {
  if (ip == IPAddress(0, 0, 0, 0) || ip == WiFi.localIP()) return false;
  if (pingProxyAt(ip, timeoutMs)) {
    rememberDiscoveredProxy(ip, NULL, source);
    return true;
  }
  return false;
}

static bool probeLikelyProxyHosts(uint8_t maxHosts = PROXY_SUBNET_PROBE_HOSTS) {
  if (WiFi.status() != WL_CONNECTED) return false;

  IPAddress gateway = WiFi.gatewayIP();
  if (tryProxyCandidate(gateway, "gateway")) return true;

  IPAddress local = WiFi.localIP();
  uint8_t tried = 0;

  // Phone hotspots often hand out adjacent DHCP leases. Probe the local
  // neighborhood before walking the whole /24 so recovery does not wait on
  // dozens of dead addresses.
  for (uint8_t delta = 1; delta <= PROXY_NEARBY_RADIUS && tried < maxHosts; delta++) {
    int lowerHost = (int)local[3] - (int)delta;
    if (lowerHost > 1 && lowerHost < 255) {
      IPAddress candidate(local[0], local[1], local[2], (uint8_t)lowerHost);
      tried++;
      if ((uint32_t)candidate != (uint32_t)gateway &&
          tryProxyCandidate(candidate, "nearby")) return true;
    }
    int upperHost = (int)local[3] + (int)delta;
    if (upperHost > 1 && upperHost < 255 && tried < maxHosts) {
      IPAddress candidate(local[0], local[1], local[2], (uint8_t)upperHost);
      tried++;
      if ((uint32_t)candidate != (uint32_t)gateway &&
          tryProxyCandidate(candidate, "nearby")) return true;
    }
  }

  while (tried < maxHosts) {
    if (subnetProbeHost == 0 || subnetProbeHost == 255) subnetProbeHost = 2;
    IPAddress candidate(local[0], local[1], local[2], subnetProbeHost);
    subnetProbeHost++;
    tried++;
    if ((uint32_t)candidate == (uint32_t)local ||
        (uint32_t)candidate == (uint32_t)gateway) {
      continue;
    }
    if (tryProxyCandidate(candidate, "subnet")) return true;
  }

  return false;
}

#if !DISABLE_PROXY_UART
static uint8_t checksum8(const char *text) {
  uint8_t sum = 0;
  if (!text) return 0;
  while (*text) {
    sum ^= (uint8_t)*text++;
  }
  return sum;
}

static void startProxyUart() {
  if (proxyUartMutex == NULL) {
    proxyUartMutex = xSemaphoreCreateMutex();
  }
  if (proxyUartStarted) return;
  pinMode(PROXY_UART_RX, INPUT_PULLUP);
  pinMode(PROXY_UART_TX, OUTPUT);
  digitalWrite(PROXY_UART_TX, HIGH);
  proxySerial.begin(PROXY_UART_BAUD, SERIAL_8N1, PROXY_UART_RX, PROXY_UART_TX);
  proxySerial.setTimeout(PROXY_UART_TIMEOUT_MS);
  proxyUartStarted = true;
  Serial.printf("[PROXY-UART] Serial1 ready RX=%d TX=%d baud=%d\n",
                PROXY_UART_RX, PROXY_UART_TX, PROXY_UART_BAUD);
}

static void resetProxyUart(const char *reason) {
  Serial.printf("[PROXY-UART] Resetting Serial1 (%s)\n",
                reason ? reason : "unknown");
  proxySerial.end();
  vTaskDelay(pdMS_TO_TICKS(20));
  pinMode(PROXY_UART_RX, INPUT_PULLUP);
  pinMode(PROXY_UART_TX, OUTPUT);
  digitalWrite(PROXY_UART_TX, HIGH);
  proxySerial.begin(PROXY_UART_BAUD, SERIAL_8N1, PROXY_UART_RX, PROXY_UART_TX);
  proxySerial.setTimeout(PROXY_UART_TIMEOUT_MS);
  while (proxySerial.available()) proxySerial.read();
}

static bool proxyUartRequest(const char *method, const char *path,
                             const char *payload, String &bodyOut,
                             unsigned long timeoutMs = PROXY_UART_TIMEOUT_MS) {
  startProxyUart();
  bool haveMutex = false;
  if (proxyUartMutex != NULL) {
    haveMutex = (xSemaphoreTake(proxyUartMutex,
                                pdMS_TO_TICKS(timeoutMs + 250UL)) == pdTRUE);
    if (!haveMutex) {
      Serial.printf("[PROXY-UART] Busy timeout %s %s\n",
                    method ? method : "GET", path ? path : "/");
      return false;
    }
  }
  while (proxySerial.available()) proxySerial.read();

  bool canRetry = (method != NULL && strcmp(method, "GET") == 0);
  uint8_t attempts = canRetry ? PROXY_UART_GET_RETRIES : 1;
  if (attempts < 1) attempts = 1;

  uint16_t lastId = 0;
  uint16_t totalRxBytes = 0;
  uint8_t totalBadChecksum = 0;
  uint8_t totalWrongId = 0;
  uint8_t totalMalformed = 0;
  char firstLine[96] = "";

  for (uint8_t attempt = 1; attempt <= attempts; attempt++) {
    uint16_t id = proxyRequestId++;
    if (proxyRequestId == 0) proxyRequestId = 1;
    lastId = id;

    char frame[760];
    snprintf(frame, sizeof(frame), "REQ|%u|%s|%s|%s",
             (unsigned int)id, method ? method : "GET", path ? path : "/",
             payload ? payload : "");
    uint8_t sum = checksum8(frame);
    proxySerial.printf("%s|%02X\n", frame, sum);
    proxySerial.flush();

    unsigned long deadline = millis() + timeoutMs;
    char line[900];
    uint16_t len = 0;

    while (millis() < deadline) {
      while (proxySerial.available()) {
        char c = (char)proxySerial.read();
        totalRxBytes++;
        if (c == '\r') continue;
        if (c == '\n') {
          line[len] = '\0';
          len = 0;

          String resp(line);
          if (firstLine[0] == '\0') {
            strncpy(firstLine, line, sizeof(firstLine) - 1);
            firstLine[sizeof(firstLine) - 1] = '\0';
          }
          if (!resp.startsWith("RESP|")) {
            totalMalformed++;
            continue;
          }
          int p1 = resp.indexOf('|', 5);
          int p2 = (p1 >= 0) ? resp.indexOf('|', p1 + 1) : -1;
          int p3 = (p2 >= 0) ? resp.lastIndexOf('|') : -1;
          if (p1 < 0 || p2 < 0 || p3 <= p2) {
            totalMalformed++;
            continue;
          }

          uint16_t respId = (uint16_t)resp.substring(5, p1).toInt();
          if (respId != id) {
            totalWrongId++;
            continue;
          }

          String withoutCrc = resp.substring(0, p3);
          uint8_t expected = (uint8_t)strtoul(resp.substring(p3 + 1).c_str(), NULL, 16);
          if (checksum8(withoutCrc.c_str()) != expected) {
            totalBadChecksum++;
            Serial.println(F("[PROXY-UART] Ignored bad checksum"));
            continue;
          }

          int status = resp.substring(p1 + 1, p2).toInt();
          bodyOut = resp.substring(p2 + 1, p3);
          Serial.printf("[PROXY-UART] Response id=%u status=%d len=%u attempt=%u\n",
                        (unsigned int)id, status,
                        (unsigned int)bodyOut.length(), (unsigned int)attempt);
          proxyUartResetCount = 0;
          if (haveMutex) xSemaphoreGive(proxyUartMutex);
          return status >= 200 && status < 300;
        }
        if (len < sizeof(line) - 1) {
          line[len++] = c;
        } else {
          len = 0;
        }
      }
      yield();
    }

    if (attempt < attempts) {
      vTaskDelay(pdMS_TO_TICKS(30));
      while (proxySerial.available()) proxySerial.read();
    }
  }

  Serial.printf("[PROXY-UART] Timeout id=%u %s %s attempts=%u rxBytes=%u badCrc=%u wrongId=%u malformed=%u first='%s'\n",
                (unsigned int)lastId, method ? method : "GET", path ? path : "/",
                (unsigned int)attempts, (unsigned int)totalRxBytes,
                (unsigned int)totalBadChecksum, (unsigned int)totalWrongId,
                (unsigned int)totalMalformed, firstLine);

  if (totalRxBytes == 0 || totalBadChecksum > 0 || totalMalformed > 1) {
    if (proxyUartResetCount < 255) proxyUartResetCount++;
    if (proxyUartResetCount >= 3) {
      resetProxyUart(totalRxBytes == 0 ? "no_rx" : "bad_frames");
      proxyUartResetCount = 0;
    }
  }
  if (haveMutex) xSemaphoreGive(proxyUartMutex);
  return false;
}

static bool proxyUartPrimaryRequest(const char *method, const char *path,
                                    const char *payload, String &bodyOut,
                                    unsigned long timeoutMs,
                                    bool force = false) {
  unsigned long now = millis();
  if (!force && now < proxyUartRetryAt) {
    return false;
  }

  if (proxyUartRequest(method, path, payload, bodyOut, timeoutMs)) {
    proxyUartRetryAt = 0;
    proxyUartFailCount = 0;
    return true;
  }

  if (proxyUartFailCount < 20) {
    proxyUartFailCount++;
  }
  unsigned long cooldown = 5000UL + ((unsigned long)proxyUartFailCount * 500UL);
  if (cooldown > 10000UL) {
    cooldown = 10000UL;
  }
  proxyUartRetryAt = now + cooldown;
  return false;
}
#else
static void startProxyUart() {}

static void resetProxyUart(const char *reason) {
  (void)reason;
}

static bool proxyUartRequest(const char *method, const char *path,
                             const char *payload, String &bodyOut,
                             unsigned long timeoutMs = 0) {
  (void)method;
  (void)path;
  (void)payload;
  (void)bodyOut;
  (void)timeoutMs;
  return false;
}

static bool proxyUartPrimaryRequest(const char *method, const char *path,
                                    const char *payload, String &bodyOut,
                                    unsigned long timeoutMs,
                                    bool force = false) {
  (void)method;
  (void)path;
  (void)payload;
  (void)bodyOut;
  (void)timeoutMs;
  (void)force;
  return false;
}
#endif

static bool discoverProxyUdp(unsigned long timeoutMs = 1200) {
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiUDP discovery;
  // Use a fixed receive port. Some phone hotspots pass broadcast discovery
  // but drop replies to random high UDP source ports.
  if (!discovery.begin(PROXY_DISCOVERY_PORT)) {
    Serial.printf("[DISCOVERY] UDP begin(%d) failed\n", PROXY_DISCOVERY_PORT);
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
      nextQueryAt = now + 250;
    }

    int size = discovery.parsePacket();
    if (size > 0) {
      int len = discovery.read(packet, sizeof(packet) - 1);
      if (len < 0) len = 0;
      packet[len] = '\0';
      if (strncmp(packet, PROXY_DISCOVERY_REPLY,
                  strlen(PROXY_DISCOVERY_REPLY)) == 0) {
        const char *payload = packet + strlen(PROXY_DISCOVERY_REPLY);
        char replyIp[16] = "";
        int replyPort = PROXY_PORT;
        char replyHardware[12] = "";

        int parsed = sscanf(payload, "%15[^:]:%d:%11s",
                            replyIp, &replyPort, replyHardware);
        IPAddress ip;
        if (parsed >= 1 && ip.fromString(replyIp)) {
          rememberDiscoveredProxy(ip, parsed >= 3 ? replyHardware : NULL, "udp");
        } else {
          ip = discovery.remoteIP();
          rememberDiscoveredProxy(ip, parsed >= 3 ? replyHardware : NULL, "udp_remote");
        }
        Serial.printf("[DISCOVERY] Proxy at %s:%d (%s)\n",
                      PROXY_HOST, replyPort, packet);
        discovery.stop();
        return true;
      }
    }
    delay(20);
  }
  discovery.stop();
  if (proxyDiscoveryMisses < 255) proxyDiscoveryMisses++;
  Serial.printf("[DISCOVERY] Proxy discovery timed out (%u)\n",
                proxyDiscoveryMisses);
  if (proxyDiscoveryMisses >= 1 && millis() >= nextProxySubnetProbeAt) {
    nextProxySubnetProbeAt = millis() + PROXY_SUBNET_PROBE_FAST_MS;
    Serial.println(F("[DISCOVERY] UDP failed; probing likely proxy IPs"));
    if (probeLikelyProxyHosts()) {
      return true;
    }
  }
  return false;
}

// ── Fetch reliability controls ──
static const uint16_t FETCH_HTTP_TIMEOUT_MS = 5000;
static const uint16_t REFRESH_HTTP_TIMEOUT_MS = 2500;
static const unsigned long GEO_CONTEXT_FRESH_MS = 6000UL;
static const unsigned long GEO_BACKGROUND_ACTIVE_MS = 3000UL;
static const unsigned long GEO_BACKGROUND_IDLE_MS = 10000UL;
static unsigned long lastFetchSuccessAt = 0;
static unsigned long lastGeoContextAt = 0;
static uint8_t fetchFailCount = 0;
static bool geoContextKnown = false;
static const uint8_t FETCH_FAIL_RESET_THRESHOLD = 20;

bool isGeoContextFresh(unsigned long maxAgeMs) {
  if (!geoContextKnown || lastGeoContextAt == 0) return false;
  return (millis() - lastGeoContextAt) <= maxAgeMs;
}

unsigned long getGeoContextAgeMs() {
  if (!geoContextKnown || lastGeoContextAt == 0) return 0xFFFFFFFFUL;
  return millis() - lastGeoContextAt;
}

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
  geoContextKnown = false;
  lastGeoContextAt = 0;
  strncpy(deliveryPhase, "NONE", sizeof(deliveryPhase) - 1);
  deliveryPhase[sizeof(deliveryPhase) - 1] = '\0';
  pickupInsideFence = false;
  dropoffInsideFence = false;
  netLog("[FETCH] Hard reset delivery context (%s)\n",
         reason ? reason : "unknown");
}

static void noteFetchFailure(const char *stage) {
  if (fetchFailCount < 255) {
    fetchFailCount++;
  }

  netLog("[FETCH] %s failed (%u/%u)\n",
         stage ? stage : "request",
         fetchFailCount,
         FETCH_FAIL_RESET_THRESHOLD);

  if (fetchFailCount < FETCH_FAIL_RESET_THRESHOLD) {
    return;
  }

  unsigned long now = millis();
  bool hasFreshContext =
      hasActiveDelivery &&
      lastFetchSuccessAt > 0 &&
      (now - lastFetchSuccessAt) <= FETCH_CONTEXT_STICKY_MS;

  if (hasFreshContext) {
    netLog("[FETCH] Preserving cached delivery during WiFi blip (%lums old)\n",
           now - lastFetchSuccessAt);
    fetchFailCount = FETCH_FAIL_RESET_THRESHOLD / 2;
    return;
  }

  fetchFailCount = 0;
  hardResetDeliveryContext(stage);
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
#if CONTROLLER_VERBOSE_LOGS
void netLog(const char *format, ...) {
  char buf[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  Serial.print(buf);

  if (WiFi.status() == WL_CONNECTED && PROXY_HOST[0] != '\0') {
    udpClient.beginPacket(PROXY_HOST, UDP_LOG_PORT);
    udpClient.print("[ESP32] ");
    udpClient.print(buf);
    udpClient.endPacket();
  }
}
#endif

// ==================== WIFI ====================
bool scanForProxy() {
#if CONTROLLER_VERBOSE_LOGS
  Serial.println("[WIFI] Scanning for configured hotspots...");
#endif
  int n = WiFi.scanNetworks();
#if CONTROLLER_VERBOSE_LOGS
  Serial.printf("[WIFI] Found %d networks\n", n);
#endif

  int bestIdx = -1;
  int bestRssi = -999;
  int bestCredential = -1;
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
#if CONTROLLER_VERBOSE_LOGS
    Serial.printf("[WIFI]   %s (%d dBm)\n", ssid.c_str(), WiFi.RSSI(i));
#endif
  }

  for (uint8_t offset = 0; offset < HOTSPOT_COUNT; offset++) {
    uint8_t h = (hotspotScanStart + offset) % HOTSPOT_COUNT;
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
    proxyChannel = WiFi.channel(bestIdx);
    uint8_t *bssid = WiFi.BSSID(bestIdx);
    if (bssid) {
      memcpy(proxyBssid, bssid, sizeof(proxyBssid));
      proxyBssidKnown = true;
    } else {
      proxyBssidKnown = false;
    }

    snprintf(HARDWARE_ID, sizeof(HARDWARE_ID), "BOX_001");
    selectedHotspotCredential = bestCredential;

#if CONTROLLER_VERBOSE_LOGS
    Serial.printf("[WIFI] Selected hotspot: %s (%d dBm, ch %ld) -> %s\n",
                  WIFI_SSID, bestRssi, (long)proxyChannel, HARDWARE_ID);
#endif
    WiFi.scanDelete();
    return true;
  }

  WiFi.scanDelete();
#if CONTROLLER_VERBOSE_LOGS
  Serial.println("[WIFI] No configured hotspot found!");
#endif
  return false;
}

static void advanceHotspotCandidate(const char *reason) {
  if (selectedHotspotCredential >= 0 && HOTSPOT_COUNT > 0) {
    hotspotScanStart = ((uint8_t)selectedHotspotCredential + 1) % HOTSPOT_COUNT;
    Serial.printf("[WIFI] Advancing hotspot after %s; next scan starts at #%u\n",
                  reason ? reason : "failure", hotspotScanStart);
  }
  WIFI_SSID[0] = '\0';
  WIFI_PASSWORD[0] = '\0';
  proxyBssidKnown = false;
  proxyChannel = 0;
}

static void beginProxyConnection(const char *reason) {
  if (WIFI_SSID[0] == '\0') {
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(78); // 19.5 dBm, units are quarter-dBm.

#if CONTROLLER_VERBOSE_LOGS
  Serial.printf("[WIFI] Begin %s -> %s", reason ? reason : "connect", WIFI_SSID);
#endif
  if (proxyChannel > 0 && proxyBssidKnown) {
#if CONTROLLER_VERBOSE_LOGS
    Serial.printf(" (ch %ld, BSSID locked)", (long)proxyChannel);
#endif
    WiFi.begin((const char *)WIFI_SSID, WIFI_PASSWORD, proxyChannel, proxyBssid, true);
  } else {
    WiFi.begin((const char *)WIFI_SSID, WIFI_PASSWORD);
  }
#if CONTROLLER_VERBOSE_LOGS
  Serial.println();
#endif

  wifiConnectStartedAt = millis();
  nextProxyDiscoveryAt = 0;
  nextProxySubnetProbeAt = 0;
  proxyMissingReconnectAt = 0;
  proxyDiscoveryMisses = 0;
  subnetProbeHost = 2;
  if (wifiReconnectAttempts < 255) {
    wifiReconnectAttempts++;
  }
}

void startWiFiConnection() {
  ensureProxyAsync();
  startProxyUart();
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  // SoftAP link is local and latency-sensitive; disable modem sleep to reduce
  // intermittent HTTP read timeouts (e.g., HTTP -11) on frequent /otp polling.
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(78);
  if (WIFI_SSID[0] == '\0') {
    scanForProxy();
  }
  beginProxyConnection("initial");
  wifiRetryAt  = millis() + WIFI_FIRST_RETRY_DELAY_MS;
  wifiRescanAt = millis() + WIFI_RESCAN_INTERVAL_MS;
  wifiBackoffMs = WIFI_RETRY_BASE_MS;
}

// Forward declaration — defined below with persistent connection helpers.
static void closePersistentConnection();

void maintainWiFiConnection(unsigned long now) {
  if (WiFi.status() == WL_CONNECTED) {
    if (PROXY_HOST[0] == '\0' && now >= nextProxyDiscoveryAt) {
      nextProxyDiscoveryAt = now + PROXY_DISCOVERY_FAST_MS;
      if (!discoverProxyUdp(250) && proxyDiscoveryMisses >= 1 &&
          now >= nextProxySubnetProbeAt) {
        nextProxySubnetProbeAt = now + PROXY_SUBNET_PROBE_FAST_MS;
        probeLikelyProxyHosts(PROXY_SUBNET_PROBE_HOSTS);
      }
    }
    if (PROXY_HOST[0] == '\0' && proxyDiscoveryMisses >= PROXY_RECONNECT_MISS_LIMIT) {
      if (proxyMissingReconnectAt == 0) {
        proxyMissingReconnectAt = now + PROXY_RECONNECT_DELAY_MS;
      } else if (now >= proxyMissingReconnectAt) {
        Serial.println(F("[DISCOVERY] Proxy still missing; rescanning hotspots"));
        closePersistentConnection();
        advanceHotspotCandidate("proxy_missing");
        WiFi.disconnect();
        scanForProxy();
        beginProxyConnection("proxy_missing");
        proxyDiscoveryMisses = 0;
        proxyMissingReconnectAt = 0;
        return;
      }
    } else {
      proxyMissingReconnectAt = 0;
    }
    wifiBackoffMs = WIFI_RETRY_BASE_MS;
    wifiRetryAt = now + WIFI_FIRST_RETRY_DELAY_MS;
    wifiRescanAt = now + WIFI_RESCAN_INTERVAL_MS;
    wifiConnectStartedAt = 0;
    wifiReconnectAttempts = 0;
    return;
  }

  if (wifiConnectStartedAt > 0 &&
      (now - wifiConnectStartedAt) < WIFI_CONNECT_ATTEMPT_MS) {
    return;
  }

  if (now >= wifiRescanAt || WIFI_SSID[0] == '\0') {
    netLog("[WIFI] Lost link; rescanning AP list...\n");
    scanForProxy();
    wifiRescanAt = now + WIFI_RESCAN_INTERVAL_MS;
  }

  if (now < wifiRetryAt) return;

  netLog("[WIFI] Retry (backoff %lums)...\n", wifiBackoffMs);
  closePersistentConnection(); // Kill stale TCP before WiFi reset
  clearDiscoveredProxy("wifi_retry");
  advanceHotspotCandidate("connect_failed");
  if (wifiReconnectAttempts >= 3) {
    WiFi.disconnect();
  }
  scanForProxy();
  beginProxyConnection("retry");

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
  if (PROXY_HOST[0] == '\0' && !discoverProxyUdp(PROXY_DISCOVERY_FAST_MS) &&
      !probeLikelyProxyHosts(PROXY_SUBNET_PROBE_HOSTS + 6)) {
    netLog("[FETCH] Proxy discovery failed\n");
    return false;
  }
  // Tear down stale socket before reconnecting.
  persistentClient.stop();
  persistentConnected = false;

  persistentClient.setTimeout(FETCH_HTTP_TIMEOUT_MS / 1000); // seconds
  if (!persistentClient.connect(PROXY_HOST, PROXY_PORT)) {
    netLog("[FETCH] TCP connect failed\n");
    clearDiscoveredProxy("tcp_connect_failed");
    return false;
  }
  persistentConnected = true;
  return true;
}

static int httpGetOnce(const char *path, String &bodyOut,
                       unsigned long timeoutMs = FETCH_HTTP_TIMEOUT_MS) {
  bodyOut = "";
  if (WiFi.status() != WL_CONNECTED) return -1;
  if (PROXY_HOST[0] == '\0' && !discoverProxyUdp(PROXY_DISCOVERY_FAST_MS) &&
      !probeLikelyProxyHosts(PROXY_SUBNET_PROBE_HOSTS + 6)) {
    return -1;
  }

  WiFiClient client;
  client.setTimeout(timeoutMs / 1000);
  Serial.printf("[HTTP] GET http://%s:%d%s\n", PROXY_HOST, PROXY_PORT, path);
  Serial.flush();
  if (!client.connect(PROXY_HOST, PROXY_PORT)) {
    Serial.println(F("[HTTP] TCP connect failed"));
    Serial.flush();
    clearDiscoveredProxy("http_connect_failed");
    return -1;
  }

  client.printf(
      "GET %s HTTP/1.1\r\n"
      "Host: %s:%d\r\n"
      "Connection: close\r\n"
      "\r\n",
      path, PROXY_HOST, PROXY_PORT);

  unsigned long deadline = millis() + timeoutMs;
  int statusCode = -1;
  int contentLength = -1;
  bool headersDone = false;

  while (millis() < deadline) {
    if (!client.connected() && !client.available()) break;
    if (!client.available()) {
      yield();
      continue;
    }
    String line = client.readStringUntil('\n');
    line.trim();
    if (statusCode < 0) {
      if (line.startsWith("HTTP/")) {
        int spaceIdx = line.indexOf(' ');
        if (spaceIdx > 0) statusCode = line.substring(spaceIdx + 1).toInt();
      }
      continue;
    }
    if (line.length() == 0) {
      headersDone = true;
      break;
    }
    if (line.startsWith("Content-Length:") || line.startsWith("content-length:")) {
      contentLength = line.substring(line.indexOf(':') + 1).toInt();
    }
  }

  if (!headersDone || statusCode < 0) {
    client.stop();
    Serial.println(F("[HTTP] No complete response headers"));
    Serial.flush();
    return -1;
  }

  if (contentLength >= 0) {
    while ((int)bodyOut.length() < contentLength && millis() < deadline) {
      if (client.available()) {
        bodyOut += (char)client.read();
      } else {
        yield();
      }
    }
  } else {
    while (millis() < deadline && (client.connected() || client.available())) {
      if (client.available()) {
        bodyOut += (char)client.read();
      } else if (bodyOut.length() > 0) {
        break;
      } else {
        yield();
      }
    }
  }
  client.stop();
  bodyOut.trim();
  Serial.printf("[HTTP] status=%d body='%s'\n", statusCode, bodyOut.c_str());
  Serial.flush();
  return statusCode;
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
    if (!persistentClient.connected() && !persistentClient.available()) return -1;
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

static void parseOtpBody(const String &body, bool updateStatusCommand = true) {
  fetchFailCount = 0;
  lastFetchSuccessAt = millis();

  if (updateStatusCommand) {
    lastStatusCommand = "";
  }
  geoDistMeters = -1;
  geoInsideFence = false;
  isReturning = false;
  geoContextKnown = false;
  strncpy(deliveryPhase, "NONE", sizeof(deliveryPhase) - 1);
  deliveryPhase[sizeof(deliveryPhase) - 1] = '\0';
  pickupInsideFence = false;
  dropoffInsideFence = false;

  netLog("[FETCH] Body: '%s'\n", body.c_str());

  int firstComma  = body.indexOf(',');
  int secondComma = (firstComma > 0) ? body.indexOf(',', firstComma + 1) : -1;
  int thirdComma  = (secondComma > 0) ? body.indexOf(',', secondComma + 1) : -1;

  if (firstComma > 0) {
    String otpPart = body.substring(0, firstComma);
    String delPart;
    String statusPart = "";

    if (secondComma > 0) {
      delPart = body.substring(firstComma + 1, secondComma);
      statusPart = (thirdComma > 0) ? body.substring(secondComma + 1, thirdComma)
                                    : body.substring(secondComma + 1);
    } else {
      delPart = body.substring(firstComma + 1);
    }

    String parsedStatusCommand = normalizeStatusToken(statusPart);
    if (updateStatusCommand) {
      lastStatusCommand = parsedStatusCommand;
    }

    bool sawGeoToken = false;
    int distIdx = body.indexOf("DIST:");
    if (distIdx >= 0) {
      int valStart = distIdx + 5;
      int valEnd = body.indexOf(',', valStart);
      if (valEnd < 0) valEnd = body.length();
      geoDistMeters = (int16_t)body.substring(valStart, valEnd).toInt();
      sawGeoToken = true;
    }
    int geoIdx = body.indexOf("GEO:");
    if (geoIdx >= 0) {
      geoInsideFence = (body.charAt(geoIdx + 4) == '1');
      sawGeoToken = true;
    }
    int retIdx = body.indexOf("RET:");
    if (retIdx >= 0) isReturning = (body.charAt(retIdx + 4) == '1');

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
    if (!phaseParsed) {
      strncpy(deliveryPhase, "NONE", sizeof(deliveryPhase) - 1);
      deliveryPhase[sizeof(deliveryPhase) - 1] = '\0';
    }

    int pupIdx = body.indexOf("PUP:");
    if (pupIdx >= 0) {
      pickupInsideFence = (body.charAt(pupIdx + 4) == '1');
      sawGeoToken = true;
    }
    int droIdx = body.indexOf("DRO:");
    if (droIdx >= 0) {
      dropoffInsideFence = (body.charAt(droIdx + 4) == '1');
      sawGeoToken = true;
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
    geoContextKnown = hasActiveDelivery && sawGeoToken;
    if (geoContextKnown) {
      lastGeoContextAt = millis();
    } else {
      lastGeoContextAt = 0;
    }
    netLog("[FETCH] otp=%d del=%d dist=%d geo=%d ret=%d phase=%s pup=%d dro=%d cmd='%s'\n",
           validOtp, validDel, geoDistMeters, geoInsideFence ? 1 : 0,
           isReturning ? 1 : 0, deliveryPhase,
           pickupInsideFence ? 1 : 0, dropoffInsideFence ? 1 : 0,
           lastStatusCommand.c_str());
    return;
  }

  if (body != "NO_OTP" && body != "null" && body.length() > 0 &&
      body.length() <= 6) {
    strncpy(currentOtp, body.c_str(), sizeof(currentOtp) - 1);
    currentOtp[sizeof(currentOtp) - 1] = '\0';
    hasActiveDelivery = true;
    geoContextKnown = false;
    lastGeoContextAt = 0;
  } else {
    currentOtp[0] = '\0';
    activeDeliveryId[0] = '\0';
    hasActiveDelivery = false;
    geoContextKnown = false;
    lastGeoContextAt = 0;
  }
}

static bool fetchDeliveryContextInternal(bool updateStatusCommand) {
  String uartBody;

  if (WiFi.status() != WL_CONNECTED) {
    closePersistentConnection();
    if (proxyUartPrimaryRequest("GET", "/otp", "", uartBody,
                                PROXY_UART_TIMEOUT_MS)) {
      netLog("[FETCH] UART fallback OK\n");
      parseOtpBody(uartBody, updateStatusCommand);
      return true;
    }
    netLog("[FETCH] Skip — no primary transport available\n");
    return false;
  }

  ProxyIoGuard io(updateStatusCommand ? "fetchDeliveryContext" : "geoCacheFetch", 100);
  if (!io.ok()) {
    if (updateStatusCommand) noteFetchFailure("proxy_io_busy");
    return false;
  }

#if DISABLE_PROXY_KEEPALIVE
  String body = "";
  int code = httpGetOnce("/otp", body, FETCH_HTTP_TIMEOUT_MS);
  netLog("[FETCH] HTTP %d\n", code);
  if (code != 200) {
    if (updateStatusCommand) noteFetchFailure("http_non_200");
    return false;
  }
  parseOtpBody(body, updateStatusCommand);
  return true;
#else
  // Ensure persistent TCP socket is alive.
  if (!ensurePersistentConnection()) {
    if (updateStatusCommand) noteFetchFailure("tcp_connect");
    return false;
  }

  // Send raw HTTP/1.1 GET with keep-alive.
  persistentClient.printf(
      "GET /otp HTTP/1.1\r\n"
      "Host: %s:%d\r\n"
      "Connection: keep-alive\r\n"
      "\r\n",
      PROXY_HOST, PROXY_PORT);

  String body = "";
  int code = readHttpResponse(body, REFRESH_HTTP_TIMEOUT_MS);

  if (code < 0) {
    netLog("[FETCH] Read failed — reconnecting\n");
    closePersistentConnection();
    // One retry with fresh connection
    if (!ensurePersistentConnection()) {
      if (updateStatusCommand) noteFetchFailure("tcp_retry");
      return false;
    }
    persistentClient.printf(
        "GET /otp HTTP/1.1\r\n"
        "Host: %s:%d\r\n"
        "Connection: keep-alive\r\n"
        "\r\n",
        PROXY_HOST, PROXY_PORT);
    body = "";
    code = readHttpResponse(body, REFRESH_HTTP_TIMEOUT_MS);
    if (code < 0) {
      if (updateStatusCommand) noteFetchFailure("http_retry");
      closePersistentConnection();
      return false;
    }
  }

  netLog("[FETCH] HTTP %d\n", code);
  if (code != 200) {
    if (updateStatusCommand) noteFetchFailure("http_non_200");
    return false;
  }

  parseOtpBody(body, updateStatusCommand);
  return true;
#endif
}

bool fetchDeliveryContext() {
  return fetchDeliveryContextInternal(true);
}

bool requestContextRefresh() {
  String uartBody;

  if (WiFi.status() != WL_CONNECTED) {
    closePersistentConnection();
    if (proxyUartPrimaryRequest("GET", "/refresh-context", "", uartBody,
                                PROXY_UART_TIMEOUT_MS)) {
      uartBody.trim();
      netLog("[REFRESH] UART fallback body='%s'\n", uartBody.c_str());
      return uartBody.startsWith("OK");
    }
    netLog("[REFRESH] Skip — no primary transport available\n");
    return false;
  }

  ProxyIoGuard io("requestContextRefresh", 250);
  if (!io.ok()) return false;

#if DISABLE_PROXY_KEEPALIVE
  String body = "";
  int code = httpGetOnce("/refresh-context?geo=1", body, FETCH_HTTP_TIMEOUT_MS);
  body.trim();
  netLog("[REFRESH] HTTP %d body='%s'\n", code, body.c_str());
  if (code == 200 && body.startsWith("OK")) {
    return true;
  }
  return false;
#else
  if (!ensurePersistentConnection()) {
    netLog("[REFRESH] TCP connect failed\n");
    return false;
  }

  persistentClient.printf(
      "GET /refresh-context?geo=1 HTTP/1.1\r\n"
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
        "GET /refresh-context?geo=1 HTTP/1.1\r\n"
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
  if (code == 200 && body.startsWith("OK")) {
    return true;
  }
  return false;
#endif
}

bool requestProxyPing() {
  String uartBody;

  if (WiFi.status() != WL_CONNECTED) {
    closePersistentConnection();
    if (proxyUartPrimaryRequest("GET", "/ping", "", uartBody, PROXY_UART_TIMEOUT_MS)) {
      uartBody.trim();
      return uartBody.startsWith("OK");
    }
    return false;
  }

  ProxyIoGuard io("requestProxyPing", 50);
  if (!io.ok()) return false;

#if DISABLE_PROXY_KEEPALIVE
  String body = "";
  int code = httpGetOnce("/ping", body, 1000);
  body.trim();
  if (code == 200 && body.startsWith("OK")) {
    return true;
  }
  return false;
#else
  if (!ensurePersistentConnection()) {
    return false;
  }

  persistentClient.printf(
      "GET /ping HTTP/1.1\r\n"
      "Host: %s:%d\r\n"
      "Connection: keep-alive\r\n"
      "\r\n",
      PROXY_HOST, PROXY_PORT);

  String body = "";
  int code = readHttpResponse(body, 1000);
  if (code < 0) {
    closePersistentConnection();
    return false;
  }

  body.trim();
  if (code == 200 && body.startsWith("OK")) {
    return true;
  }
  return false;
#endif
}

bool fetchDiagnostics(ControllerDiagData &out) {
  String uartDiagBody;
  if (WiFi.status() != WL_CONNECTED) {
    if (proxyUartRequest("GET", "/diag", "", uartDiagBody, CONTROLLER_DIAG_HTTP_TIMEOUT_MS + 800)) {
      String field;
      ControllerDiagData parsed = out;
      bool parsedAny = false;

      if (parseDiagField(uartDiagBody, "batt_pct", field)) { parsed.battPct = field.toInt(); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "batt_v", field)) { parsed.battVoltage = field.toFloat(); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "rssi", field)) { parsed.rssi = field.toInt(); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "csq", field)) { parsed.csq = field.toInt(); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "gps_fix", field)) { parsed.gpsFix = (field.toInt() != 0); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "lte", field)) { parsed.lteConnected = (field.toInt() != 0); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "modem", field)) { parsed.modemOk = (field.toInt() != 0); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "time", field)) { parsed.timeSynced = (field.toInt() != 0); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "cam_up", field)) { parsed.camUp = (field.toInt() != 0); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "ctrl_up", field)) { parsed.controllerUp = (field.toInt() != 0); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "fb_fail", field)) { parsed.firebaseFailures = field.toInt(); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "cmd_stage", field)) { parsed.commandStage = field.toInt(); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "cmd_pending", field)) { parsed.commandPending = (field.toInt() != 0); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "conn_state", field)) { parsed.connectivityState = field.toInt(); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "lte_reconn_ms", field)) { parsed.lteReconnectMs = (unsigned long)field.toInt(); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "upload_active", field)) { parsed.photoUploadActive = (field.toInt() != 0); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "upload_pct", field)) { parsed.photoUploadProgress = field.toInt(); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "upload_age", field)) { parsed.photoUploadAgeMs = (unsigned long)field.toInt(); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "cam_age", field)) { parsed.camAgeMs = (unsigned long)field.toInt(); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "ctrl_age", field)) { parsed.controllerAgeMs = (unsigned long)field.toInt(); parsedAny = true; }
      if (parseDiagField(uartDiagBody, "uptime", field)) { parsed.proxyUptimeMs = (unsigned long)field.toInt(); parsedAny = true; }
      if (parsedAny) {
        out = parsed;
        return true;
      }
    }
    return false;
  }
  if (PROXY_HOST[0] == '\0' && !discoverProxyUdp(PROXY_DISCOVERY_FAST_MS) &&
      !probeLikelyProxyHosts(PROXY_SUBNET_PROBE_HOSTS + 6)) return false;

  ProxyIoGuard io("fetchDiagnostics", 50);
  if (!io.ok()) return false;

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
  ProxyIoGuard io("sendRawPost", 2500);
  if (!io.ok()) {
    return -2;
  }
  String uartBody;

  if (WiFi.status() != WL_CONNECTED) {
    if (proxyUartPrimaryRequest("POST", path, json, uartBody,
                                PROXY_UART_TIMEOUT_MS)) {
      return 200;
    }
    return -1;
  }
#if DISABLE_PROXY_KEEPALIVE
  if (PROXY_HOST[0] == '\0' && !discoverProxyUdp(PROXY_DISCOVERY_FAST_MS) &&
      !probeLikelyProxyHosts(PROXY_SUBNET_PROBE_HOSTS + 6)) {
    return -1;
  }
  WiFiClient client;
  HTTPClient http;
  char url[96];
  snprintf(url, sizeof(url), "http://%s:%d%s", PROXY_HOST, PROXY_PORT, path);
  http.setTimeout(FETCH_HTTP_TIMEOUT_MS);
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST((uint8_t *)json, strlen(json));
  http.end();
  return code;
#else
  if (!ensurePersistentConnection()) {
    return -1;
  }

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
    if (!ensurePersistentConnection()) {
      return -1;
    }
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
#endif
}

static bool enqueueProxyPost(const char *path, const char *json) {
  ensureProxyAsync();
  if (proxyPostQueue == NULL || path == NULL || json == NULL) return false;

  ProxyPostJob job;
  memset(&job, 0, sizeof(job));
  strncpy(job.path, path, sizeof(job.path) - 1);
  strncpy(job.json, json, sizeof(job.json) - 1);
  job.attempts = 0;

  if (xQueueSend(proxyPostQueue, &job, 0) != pdTRUE) {
    ProxyPostJob dropped;
    if (xQueueReceive(proxyPostQueue, &dropped, 0) == pdTRUE) {
      proxyPostDropCount++;
    }
    if (xQueueSend(proxyPostQueue, &job, 0) != pdTRUE) {
      proxyPostDropCount++;
      return false;
    }
  }
  proxyPostQueuedCount++;
  return true;
}

static void proxyPostTask(void *pvParameters) {
  (void)pvParameters;
  ProxyPostJob job;
  for (;;) {
    if (proxyPostQueue != NULL &&
        xQueueReceive(proxyPostQueue, &job, pdMS_TO_TICKS(250)) == pdTRUE) {
      int code = sendRawPost(job.path, job.json);
      if (code == 200 || code == 201) {
        proxyPostSentCount++;
      } else {
        proxyPostFailCount++;
        job.attempts++;
        if (job.attempts < 3) {
          vTaskDelay(pdMS_TO_TICKS(500 + (job.attempts * 500)));
          if (xQueueSend(proxyPostQueue, &job, 0) != pdTRUE) {
            proxyPostDropCount++;
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

static void proxyGeoTask(void *pvParameters) {
  (void)pvParameters;
  unsigned long lastPollAt = 0;

  for (;;) {
    unsigned long now = millis();
    unsigned long interval = hasActiveDelivery ? GEO_BACKGROUND_ACTIVE_MS
                                               : GEO_BACKGROUND_IDLE_MS;
    bool staleActiveGeo =
        hasActiveDelivery && !isGeoContextFresh(GEO_CONTEXT_FRESH_MS);

    if (WiFi.status() == WL_CONNECTED &&
        PROXY_HOST[0] != '\0' &&
        (staleActiveGeo || now - lastPollAt >= interval)) {
      lastPollAt = now;

      if (hasActiveDelivery) {
        requestContextRefresh();
      }
      fetchDeliveryContextInternal(false);
    }

    vTaskDelay(pdMS_TO_TICKS(250));
  }
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

  bool queued = enqueueProxyPost("/event", json);
#if CONTROLLER_VERBOSE_LOGS
  Serial.printf("[EVENT] queued=%d\n", queued ? 1 : 0);
#endif

  return queued;
}

// ==================== ALERT REPORTING ====================
bool reportAlertToProxy(const char *alertType, const char *details) {
  char json[256];
  snprintf(json, sizeof(json),
           "{\"alert_type\":\"%s\",\"details\":\"%s\","
           "\"box_id\":\"%s\",\"delivery_id\":\"%s\"}",
           alertType, details, HARDWARE_ID, activeDeliveryId);

  bool queued = enqueueProxyPost("/event", json);
#if CONTROLLER_VERBOSE_LOGS
  Serial.printf("[ALERT] %s queued=%d\n", alertType, queued ? 1 : 0);
#endif

  return queued;
}

// ==================== TAMPER REPORT ====================
bool reportTamperToProxy() {
  char json[256];
  snprintf(json, sizeof(json),
           "{\"alert_type\":\"REED_TAMPER\",\"details\":\"lid_opened_no_unlock\","
           "\"box_id\":\"%s\",\"delivery_id\":\"%s\",\"tamper\":true}",
           HARDWARE_ID, activeDeliveryId);

  bool queued = enqueueProxyPost("/event", json);
#if CONTROLLER_VERBOSE_LOGS
  Serial.printf("[TAMPER] queued=%d\n", queued ? 1 : 0);
#endif

  return queued;
}

// ==================== COMMAND ACK REPORT ====================
bool reportCommandAckToProxy(const char *command, const char *status, const char *details) {
  char json[320];
  snprintf(json, sizeof(json),
           "{\"command\":\"%s\",\"status\":\"%s\",\"details\":\"%s\","
           "\"box_id\":\"%s\",\"delivery_id\":\"%s\"}",
           command ? command : "", status ? status : "", details ? details : "",
           HARDWARE_ID, activeDeliveryId);

  bool queued = enqueueProxyPost("/command-ack", json);
#if CONTROLLER_VERBOSE_LOGS
  Serial.printf("[CMD_ACK] %s/%s queued=%d\n", command ? command : "", status ? status : "", queued ? 1 : 0);
#endif

  return queued;
}

bool requestCameraPowerMode(bool wakeMode) {
  String uartBody;
  const char *path = wakeMode ? "/cam-power?mode=wake" : "/cam-power?mode=sleep";

  if (WiFi.status() != WL_CONNECTED) {
    if (proxyUartPrimaryRequest("GET", path, "", uartBody,
                                PROXY_UART_TIMEOUT_MS)) {
      uartBody.trim();
      return uartBody.indexOf("CAM_") >= 0 || uartBody.indexOf("OK") >= 0;
    }
    return false;
  }
  if (PROXY_HOST[0] == '\0' && !discoverProxyUdp(PROXY_DISCOVERY_FAST_MS) &&
      !probeLikelyProxyHosts(PROXY_SUBNET_PROBE_HOSTS + 6)) {
    return false;
  }

  ProxyIoGuard io("requestCameraPowerMode", 250);
  if (!io.ok()) return false;

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

static int parsePersonalPinResponse(const String &body) {
  if (body.indexOf("ALLOW_UNLOCK") >= 0) return 1;
  if (body.indexOf("ALLOW_RELOCK") >= 0) return 2;
  if (body.indexOf("DENY:disabled") >= 0) return -3;
  if (body.indexOf("DENY:sync_failed") >= 0) return -4;
  if (body.indexOf("DENY:missing_pin") >= 0 ||
      body.indexOf("DENY:invalid") >= 0 ||
      body.indexOf("DENY:box_mismatch") >= 0) {
    return -1;
  }
  return 0;
}

int requestPersonalPinToggle(const char *pin, bool currentlyLocked) {
  if (!pin || pin[0] == '\0') {
    return 0;
  }

  char json[256];
  snprintf(json, sizeof(json),
           "{\"pin\":\"%s\",\"box_id\":\"%s\",\"delivery_id\":\"%s\","
           "\"currently_locked\":%s,\"source\":\"controller_keypad\"}",
           pin, HARDWARE_ID, activeDeliveryId, currentlyLocked ? "true" : "false");

  String uartBody;

  if (WiFi.status() != WL_CONNECTED) {
    if (proxyUartPrimaryRequest("POST", "/personal-pin-verify", json, uartBody,
                                PROXY_UART_TIMEOUT_MS)) {
      uartBody.trim();
      return parsePersonalPinResponse(uartBody);
    }
    return 0;
  }
  if (PROXY_HOST[0] == '\0' && !discoverProxyUdp(PROXY_DISCOVERY_FAST_MS) &&
      !probeLikelyProxyHosts(PROXY_SUBNET_PROBE_HOSTS + 6)) {
    return -2;
  }

  ProxyIoGuard io("requestPersonalPinToggle", 250);
  if (!io.ok()) return -2;

  HTTPClient http;
  char url[96];
  snprintf(url, sizeof(url), "http://%s:%d/personal-pin-verify", PROXY_HOST,
           PROXY_PORT);

  http.setTimeout(5000);
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST((uint8_t *)json, strlen(json));
  if (code != 200) {
#if CONTROLLER_VERBOSE_LOGS
    Serial.printf("[PIN] verify HTTP %d\n", code);
#endif
    http.end();
    return -2;
  }

  String body = http.getString();
  body.trim();
  http.end();

  int parsed = parsePersonalPinResponse(body);
  if (parsed != 0) {
    return parsed;
  }
  return 0;
}

// ==================== FACE CHECK ====================
int requestFaceCheck() {
  char facePath[128];
  if (strlen(activeDeliveryId) > 0 &&
      strcmp(activeDeliveryId, "NO_DELIVERY") != 0 &&
      strcmp(activeDeliveryId, "null") != 0) {
    snprintf(facePath, sizeof(facePath), "/face-check?delivery_id=%s", activeDeliveryId);
  } else {
    snprintf(facePath, sizeof(facePath), "/face-check");
  }

  String uartBody;

  // ── WiFi via proxy ──
  if (WiFi.status() == WL_CONNECTED) {
    if (PROXY_HOST[0] == '\0' && !discoverProxyUdp(PROXY_DISCOVERY_FAST_MS) &&
        !probeLikelyProxyHosts(PROXY_SUBNET_PROBE_HOSTS + 6)) {
#if CONTROLLER_VERBOSE_LOGS
      Serial.println(F("[FACE] Proxy discovery failed"));
#endif
    } else {
    ProxyIoGuard io("requestFaceCheck", 250);
    if (!io.ok()) return -1;
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

#if CONTROLLER_VERBOSE_LOGS
      Serial.printf("[FACE] HTTP 200 but body error: %s\n", body.c_str());
#endif
    } else {
#if CONTROLLER_VERBOSE_LOGS
      Serial.printf("[FACE] HTTP %d\n", code);
#endif
      http.end();
    }
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    return -1;
  }

  // ── Fallback: UART Serial2 to ESP32-CAM directly ──
#if CONTROLLER_VERBOSE_LOGS
      Serial.println(F("[FACE] WiFi down — trying UART fallback..."));
#endif

  if (proxyUartPrimaryRequest("GET", facePath, "", uartBody,
                              FACE_CHECK_WIFI_TIMEOUT_MS)) {
    uartBody.trim();
    if (uartBody.indexOf("FACE_OK") >= 0) return 1;
    if (uartBody.indexOf("NO_FACE_LOW_LIGHT") >= 0) return 2;
    if (uartBody.indexOf("NO_FACE") >= 0) return 0;
  }

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

#if CONTROLLER_VERBOSE_LOGS
  Serial.printf("[FACE] UART response: '%s'\n", uartBuf);
#endif

  if (strstr(uartBuf, "FACE_OK") != NULL) return 1;
  if (strstr(uartBuf, "NO_FACE_LOW_LIGHT") != NULL) return 2;
  if (strstr(uartBuf, "NO_FACE") != NULL) return 0;

#if CONTROLLER_VERBOSE_LOGS
  Serial.println(F("[FACE] UART fallback failed"));
#endif
  return -1;
}
