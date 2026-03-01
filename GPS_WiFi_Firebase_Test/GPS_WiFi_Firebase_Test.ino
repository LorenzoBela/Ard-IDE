/**
 * GPS & LTE/WiFi Signal to Firebase - With WiFi Fallback
 * For LILYGO T-SIM A7670E (GPS Version)
 * 
 * Tries LTE first, falls back to WiFi if LTE fails
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

// TinyGSM - MUST define modem BEFORE include
#define TINY_GSM_MODEM_A7672X
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>

// ==================== CONFIGURATION ====================
// LTE Settings — Globe Telecom Philippines
// Postpaid: "internet.globe.com.ph"  |  Prepaid: "http.globe.com.ph"
#define GPRS_APN      "internet.globe.com.ph"
#define GPRS_APN_ALT  "http.globe.com.ph"
#define GPRS_USER     ""
#define GPRS_PASS     ""

// WiFi Fallback Settings
#define WIFI_SSID     "RAJ_VIRUS2"
#define WIFI_PASS     "I@mjero4ever"

// Firebase REST API
#define FIREBASE_HOST "smart-top-box-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "AIzaSyA7DETBpsdPN6icfWi7PijCbpmLNWEZyTQ"
#define HARDWARE_ID   "TEST_BOX_001"

// Pins for LILYGO T-SIM A7670E (official pinout)
#define MODEM_TX        26
#define MODEM_RX        27
#define MODEM_PWRKEY    4
#define MODEM_POWER_ON  12
#define MODEM_RESET_PIN 5
#define MODEM_BAUD      115200

// Timing (ms)
#define GPS_INTERVAL_ACQUIRING 500   // Fast polling while seeking fix
#define GPS_INTERVAL_LOCKED    2000  // Normal polling once fix acquired
#define FIREBASE_INTERVAL      5000
#define SIGNAL_INTERVAL        10000

// ==================== GLOBALS ====================
HardwareSerial modemSerial(1);
TinyGsm modem(modemSerial);
TinyGsmClientSecure lteClient(modem);
WiFiClientSecure wifiClient;

bool lteConnected = false;
bool wifiConnected = false;
bool gpsEnabled = false;
bool modemOK = false;
unsigned long lastGps = 0, lastFB = 0, lastSig = 0;

double gpsLat = 0, gpsLon = 0;
bool gpsFix = false;

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
  delay(3000);  // Wait for modem boot
  
  String name = modem.getModemName();
  Serial.println("  Modem: " + name);
  
  String info = modem.getModemInfo();
  Serial.println("  Info: " + info);
  
  if (name.length() > 0 || info.length() > 0) {
    Serial.println("  Modem OK");
    return true;
  }
  
  Serial.println("  Modem not responding");
  return false;
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
    
    // Wait for READY signal
    unsigned long start = millis();
    bool ready = false;
    while (millis() - start < 10000) {
      while (modem.stream.available()) {
        String line = modem.stream.readStringUntil('\n');
        if (line.indexOf("READY") >= 0) {
          ready = true;
          break;
        }
      }
      if (ready) break;
      delay(100);
    }
    if (!ready) {
      Serial.println("  No READY (may still work)");
    }
  } else {
    Serial.println("  GNSS already powered, preserving satellite cache");
  }
  
  // === TTFF OPTIMIZATIONS ===
  
  // 1. Multi-constellation: GPS + GLONASS + BeiDou + Galileo (more sats = faster fix)
  Serial.print("  Multi-GNSS (GPS+GLO+BDS+GAL)...");
  modem.sendAT("+CGNSSMODE=15");
  modem.waitResponse(1000L);
  Serial.println(" OK");
  
  // 2. Hot start: reuse cached ephemeris/almanac from last session
  Serial.print("  Hot start...");
  modem.sendAT("+CGNSRST=0");  // 0=hot, 1=warm, 2=cold
  modem.waitResponse(1000L);
  Serial.println(" OK");
  
  // 3. AGPS: download satellite data over cellular/internet
  Serial.print("  AGPS assist...");
  modem.sendAT("+CAGPS");
  if (modem.waitResponse(10000L, resp) == 1) {
    Serial.println(" OK");
  } else {
    Serial.println(" skipped (no data connection yet)");
  }
  
  // 4. High sensitivity mode for faster satellite acquisition
  Serial.print("  High sensitivity...");
  modem.sendAT("+CGNSHOT=1");
  modem.waitResponse(1000L);
  Serial.println(" OK");
  
  // Check GPS power status
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
      Serial.println("         2) Antenna voltage 2.8-3.3V on center pin");
      Serial.println("         3) Device outdoors with clear sky view");
    }
  }
  
  Serial.println("  GPS initialized with TTFF optimizations");
  Serial.println("  Expected fix: 3-10s (AGPS) / 5-15s (hot) / 30s+ (cold)");
  return true;
}

void readGPS() {
  // Use AT command to get GPS data
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
  
  // Parse CGNSSINFO format: <fix>,<sats>,,<beidou>,<galileo>,<lat>,<N/S>,<lon>,<E/W>,<date>,<time>,...
  // Example: 3,13,,03,00,14.5174828,N,121.0555191,E,090226,091920.00,...
  int commas[15];
  int commaCount = 0;
  for (int i = 0; i < data.length() && commaCount < 15; i++) {
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
    // A7670E returns decimal degrees directly, no conversion needed
    gpsLat = latStr.toDouble();
    gpsLon = lonStr.toDouble();
    
    // Apply direction
    if (latDir == "S") gpsLat = -gpsLat;
    if (lonDir == "W") gpsLon = -gpsLon;
    
    gpsFix = true;
    
    // Debug output
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
  if (!modemOK) return false;
  
  Serial.println("Connecting LTE (Globe PH)...");
  
  // ── SIM Diagnostics ──
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
  modem.sendAT("+CGSN");  // IMEI
  if (modem.waitResponse(1000L, resp) == 1) {
    Serial.println("  IMEI: " + resp);
  }
  modem.sendAT("+CIMI");  // IMSI (starts with 515 02 for Globe)
  if (modem.waitResponse(1000L, resp) == 1) {
    Serial.println("  IMSI: " + resp);
    if (resp.indexOf("51502") < 0) {
      Serial.println("  WARNING: IMSI does not match Globe PH (515 02)");
    }
  }
  
  // ── Signal Check ──
  int csq = modem.getSignalQuality();
  Serial.printf("  Signal quality (CSQ): %d", csq);
  if (csq == 99 || csq == 0) {
    Serial.println(" — No signal!");
    Serial.println("  CHECK: LTE antenna connected, SIM has coverage");
    return false;
  }
  int dbm = -113 + (2 * csq);
  Serial.printf(" (%d dBm)\n", dbm);
  
  // ── Set PDP Context with correct Globe APN ──
  Serial.println("  Setting PDP context (Globe APN)...");
  modem.sendAT("+CGDCONT=1,\"IP\",\"" + String(GPRS_APN) + "\"");
  modem.waitResponse(3000L);
  
  // ── Network Registration ──
  Serial.print("  Waiting for network...");
  modem.sendAT("+COPS=0");  // Automatic operator selection
  modem.waitResponse(5000L);
  
  bool networkOK = false;
  for (int i = 0; i < 3; i++) {
    if (i > 0) Serial.print(" retry ");
    if (modem.waitForNetwork(60000L)) {
      networkOK = true;
      Serial.println(" registered!");
      break;
    }
    Serial.print(".");
  }
  
  if (!networkOK) {
    Serial.println(" network timeout");
    // Check registration status for diagnostics
    modem.sendAT("+CREG?");
    if (modem.waitResponse(1000L, resp) == 1) {
      Serial.println("  CREG: " + resp);
    }
    return false;
  }
  
  Serial.println("  Operator: " + modem.getOperator());
  
  // ── GPRS Connect (try primary APN, fallback to alt) ──
  Serial.print("  Connecting GPRS (" + String(GPRS_APN) + ")...");
  unsigned long gprsStart = millis();
  bool gprsOK = false;
  
  while (millis() - gprsStart < 20000 && !gprsOK) {
    if (modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS)) {
      gprsOK = true;
      break;
    }
    delay(1000);
  }
  
  // Fallback: try alternate Globe APN (prepaid vs postpaid)
  if (!gprsOK) {
    Serial.println(" failed, trying alt APN...");
    Serial.print("  Connecting GPRS (" + String(GPRS_APN_ALT) + ")...");
    modem.sendAT("+CGDCONT=1,\"IP\",\"" + String(GPRS_APN_ALT) + "\"");
    modem.waitResponse(3000L);
    
    gprsStart = millis();
    while (millis() - gprsStart < 20000 && !gprsOK) {
      if (modem.gprsConnect(GPRS_APN_ALT, GPRS_USER, GPRS_PASS)) {
        gprsOK = true;
        break;
      }
      delay(1000);
    }
  }
  
  if (!gprsOK) {
    Serial.println(" FAILED");
    Serial.println("  CHECK: 1) SIM has active data (load/promo)");
    Serial.println("         2) Try dialing *143# on a phone with this SIM");
    Serial.println("         3) APN may need manual config via Globe app");
    return false;
  }
  
  Serial.println(" OK!");
  
  // ── Activate PDP context explicitly ──
  modem.sendAT("+CGACT=1,1");
  modem.waitResponse(5000L);
  
  // Configure DNS servers (Google DNS)
  Serial.print("  Setting DNS...");
  modem.sendAT("+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\"");
  if (modem.waitResponse(5000L) == 1) {
    Serial.println(" OK");
  } else {
    Serial.println(" skip");
  }
  
  delay(2000);  // Wait for network to stabilize
  
  Serial.println("  IP: " + modem.localIP().toString());
  Serial.println("  Operator: " + modem.getOperator());
  
  // Test DNS with simple HTTP GET
  Serial.print("  Testing internet...");
  if (lteClient.connect("www.google.com", 80)) {
    lteClient.stop();
    Serial.println(" OK");
  } else {
    Serial.println(" FAIL - no internet access");
    Serial.println("  Your SIM/APN may not have data enabled");
  }
  
  lteConnected = true;
  return true;
}

int getSignal() {
  if (!modemOK) return -999;
  int csq = modem.getSignalQuality();
  return (csq == 99 || csq == 0) ? -999 : -113 + (2 * csq);
}

// ==================== WIFI FALLBACK ====================
bool connectWiFi() {
  Serial.print("Connecting WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  int attempts = 20;
  while (WiFi.status() != WL_CONNECTED && attempts-- > 0) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" OK - IP: " + WiFi.localIP().toString());
    wifiConnected = true;
    
    // Configure SSL client with proper timeouts
    wifiClient.setInsecure(); // Skip cert validation for Firebase
    wifiClient.setHandshakeTimeout(30);  // 30 second SSL handshake timeout
    wifiClient.setTimeout(10000);  // 10 second read/write timeout
    
    Serial.println("  WiFi RSSI: " + String(WiFi.RSSI()) + " dBm");
    return true;
  }
  Serial.println(" failed");
  return false;
}

// ==================== FIREBASE ====================
void sendToFirebase() {
  if (!lteConnected && !wifiConnected) {
    Serial.println("Firebase: No connection available");
    return;
  }
  
  Serial.print("Firebase... ");
  
  // Build JSON for hardware status
  String hardwareJson = "{";
  hardwareJson += "\"connection\":\"" + String(lteConnected ? "LTE" : "WiFi") + "\",";
  if (lteConnected) {
    hardwareJson += "\"rssi\":" + String(getSignal()) + ",";
    hardwareJson += "\"csq\":" + String(modem.getSignalQuality()) + ",";
    hardwareJson += "\"op\":\"" + modem.getOperator() + "\",";
  } else {
    hardwareJson += "\"rssi\":" + String(WiFi.RSSI()) + ",";
    hardwareJson += "\"ssid\":\"" + String(WIFI_SSID) + "\",";
  }
  hardwareJson += "\"gps_fix\":" + String(gpsFix ? "true" : "false");
  hardwareJson += ",\"last_updated\":" + String(millis());
  hardwareJson += "}";
  
  // Build JSON for location (separate path for efficiency)
  String locationJson = "";
  if (gpsFix) {
    locationJson = "{";
    locationJson += "\"latitude\":" + String(gpsLat, 6) + ",";
    locationJson += "\"longitude\":" + String(gpsLon, 6) + ",";
    locationJson += "\"timestamp\":" + String(millis()) + ",";
    locationJson += "\"source\":\"box\"";
    locationJson += "}";
  }
  
  // Connect to Firebase
  bool connected = false;
  Serial.print("connecting... ");
  
  if (lteConnected) {
    // For LTE, try HTTP first (port 80) to test basic connectivity
    // Firebase supports both HTTP and HTTPS
    connected = lteClient.connect(FIREBASE_HOST, 80);
    if (!connected) {
      Serial.println("FAIL (HTTP connection failed)");
      Serial.println("  DNS or network issue - check APN settings");
      return;
    }
  } else if (wifiConnected) {
    connected = wifiClient.connect(FIREBASE_HOST, 443);
    if (!connected) {
      Serial.println("FAIL (SSL handshake failed)");
      return;
    }
  }
  
  if (!connected) {
    Serial.println("FAIL");
    return;
  }
  
  Serial.print("sending... ");
  
  // Use appropriate client
  Client* activeClient = lteConnected ? (Client*)&lteClient : (Client*)&wifiClient;
  
  // Send hardware status
  String hardwarePath = "/hardware/" + String(HARDWARE_ID) + ".json";
  activeClient->print("PUT "); activeClient->print(hardwarePath); activeClient->println(" HTTP/1.1");
  activeClient->print("Host: "); activeClient->println(FIREBASE_HOST);
  activeClient->println("Content-Type: application/json");
  activeClient->print("Content-Length: "); activeClient->println(hardwareJson.length());
  activeClient->println("Connection: keep-alive");
  activeClient->println();
  activeClient->println(hardwareJson);
  
  // Wait for response
  unsigned long t = millis();
  bool gotResponse = false;
  while (activeClient->connected() && millis() - t < 10000) {
    if (activeClient->available()) {
      String line = activeClient->readStringUntil('\n');
      if (line.startsWith("HTTP/1.1")) {
        if (line.indexOf("200") > 0) {
          Serial.print("HW:OK ");
        } else {
          Serial.print("HW:ERR ");
        }
        gotResponse = true;
        break;
      }
    }
    delay(10);
  }
  
  if (!gotResponse) {
    Serial.print("HW:TIMEOUT ");
  }
  
  // Clear buffer
  while (activeClient->available()) activeClient->read();
  
  // Send location if GPS has fix
  if (gpsFix && locationJson.length() > 0) {
    // Reconnect if needed
    if (!activeClient->connected()) {
      if (lteConnected) {
        activeClient->connect(FIREBASE_HOST, 80);
      } else {
        activeClient->connect(FIREBASE_HOST, 443);
      }
      delay(100);
    }
    
    String locationPath = "/locations/" + String(HARDWARE_ID) + "/box.json";
    activeClient->print("PUT "); activeClient->print(locationPath); activeClient->println(" HTTP/1.1");
    activeClient->print("Host: "); activeClient->println(FIREBASE_HOST);
    activeClient->println("Content-Type: application/json");
    activeClient->print("Content-Length: "); activeClient->println(locationJson.length());
    activeClient->println("Connection: close");
    activeClient->println();
    activeClient->println(locationJson);
    
    // Wait for response
    t = millis();
    gotResponse = false;
    while (activeClient->connected() && millis() - t < 10000) {
      if (activeClient->available()) {
        String line = activeClient->readStringUntil('\n');
        if (line.startsWith("HTTP/1.1")) {
          if (line.indexOf("200") > 0) {
            Serial.println("LOC:OK!");
          } else {
            Serial.println("LOC:ERR");
          }
          gotResponse = true;
          break;
        }
      }
      delay(10);
    }
    
    if (!gotResponse) {
      Serial.println("LOC:TIMEOUT");
    }
  } else {
    Serial.println("(no GPS)");
  }
  
  // Clear any remaining data
  while (activeClient->available()) {
    activeClient->read();
  }
  
  activeClient->stop();
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== GPS/LTE/WiFi Firebase Test ===\n");
  
  // Initialize modem with proper power sequence
  modemSerial.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  powerModem();
  
  modemOK = initModem();
  
  if (modemOK) {
    gpsEnabled = initGPS();
    Serial.println(gpsEnabled ? "GPS: Ready" : "GPS: Failed (continuing)");
    
    // Try WiFi first for testing (LTE data may not be enabled)
    Serial.println("\nAttempting WiFi connection...");
    if (!connectWiFi()) {
      // If WiFi fails, try LTE
      Serial.println("\nWiFi failed, trying LTE...");
      connectLTE();
    }
  } else {
    Serial.println("Modem not available, using WiFi only");
    connectWiFi();
  }
  
  if (!lteConnected && !wifiConnected) {
    Serial.println("\n*** NO CONNECTION! Check LTE/WiFi settings ***");
    while(1) delay(1000);
  }
  
  Serial.println("\n=== Ready ===");
  Serial.println("Connection: " + String(lteConnected ? "LTE" : "WiFi") + "\n");
}

// ==================== LOOP ====================
void loop() {
  unsigned long now = millis();
  
  // Check connection periodically
  if (now - lastSig >= SIGNAL_INTERVAL) {
    if (lteConnected && modemOK) {
      if (!modem.isGprsConnected()) {
        lteConnected = false;
        if (!connectLTE()) connectWiFi();
      }
    } else if (wifiConnected) {
      if (WiFi.status() != WL_CONNECTED) {
        wifiConnected = false;
        connectWiFi();
      }
    }
    lastSig = now;
  }
  
  // Read GPS (poll faster while acquiring fix, slower once locked)
  unsigned long gpsInterval = gpsFix ? GPS_INTERVAL_LOCKED : GPS_INTERVAL_ACQUIRING;
  if (modemOK && gpsEnabled && now - lastGps >= gpsInterval) {
    readGPS();
    lastGps = now;
    
    // Show GPS status periodically
    static unsigned long lastGpsStatus = 0;
    if (now - lastGpsStatus >= 10000) {  // Every 10 seconds
      if (gpsFix) {
        Serial.printf("GPS: %.6f, %.6f\n", gpsLat, gpsLon);
      } else {
        Serial.println("GPS: No fix yet (needs outdoor + clear sky)");
      }
      lastGpsStatus = now;
    }
  }
  
  // Send to Firebase
  if ((lteConnected || wifiConnected) && now - lastFB >= FIREBASE_INTERVAL) {
    sendToFirebase();
    lastFB = now;
  }
  
  delay(100);
}
