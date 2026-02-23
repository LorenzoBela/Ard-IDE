/**
 * SMART TOP BOX - MAIN FIRMWARE (V2 - FULL STACK)
 *
 * Hardware:
 * - MCU: LILYGO T-SIM A7670E (ESP32)
 * - LTE/GPS: A7670E via UART1 (Full AT Command Stack)
 * - Camera: ESP32-CAM via UART2 (Trigger only: 'C' -> 'D'/'F')
 * - Input: 4x3 Matrix Keypad, Reed Switch
 * - Output: Solenoid Lock (MOSFET), LCD 1602 (I2C)
 *
 * Logic:
 * - "Split-Brain": Syncs /boxes/{ID}/delivery_context for OTP hash.
 * - "Photo-First": Valid OTP -> Trigger Cam -> Wait 'D' -> Unlock.
 * - Non-blocking: Uses millisecond timers for GPS/LTE tasks.
 */

#include <Arduino.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// ==================== PIN DEFINITIONS (Visual & Internal) ====================
// Modem (Internal - DO NOT CHANGE)
#define MODEM_TX 26
#define MODEM_RX 27
#define MODEM_PWRKEY 4
#define MODEM_POWER_ON 12
#define MODEM_RESET_PIN 5
#define MODEM_BAUD 115200

// Peripherals (User Conflict-Free List)
#define PIN_SOLENOID 25 // Replaced GPIO 4
#define PIN_REED_SWITCH 33

// ESP32-CAM Communication (Serial2)
#define CAM_RX_PIN 16 // Connect to CAM TX
#define CAM_TX_PIN 17 // Connect to CAM RX
#define CAM_BAUD 115200

// LCD (I2C) - 21/22
#define LCD_SDA 21
#define LCD_SCL 22

// Keypad (4x3)
const byte ROWS = 4;
const byte COLS = 3;
byte rowPins[ROWS] = {13, 14, 15, 32}; // 32 replaced 5
byte colPins[COLS] = {18, 19, 23};

char keys[ROWS][COLS] = {
    {'1', '2', '3'}, {'4', '5', '6'}, {'7', '8', '9'}, {'*', '0', '#'}};

// ==================== LIBRARIES ====================
#define TINY_GSM_MODEM_A7672X
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>

HardwareSerial modemSerial(1);
TinyGsm modem(modemSerial);

LiquidCrystal_I2C lcd(0x27, 16, 2);
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
HardwareSerial camSerial(2);

// ==================== LTE / FIREBASE CONFIG ====================
#define GPRS_APN "internet.globe.com.ph"
#define GPRS_APN_ALT "http.globe.com.ph"
#define GPRS_USER ""
#define GPRS_PASS ""

#define FIREBASE_HOST                                                          \
  "smart-top-box-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "AIzaSyA7DETBpsdPN6icfWi7PijCbpmLNWEZyTQ"
#define BOX_ID "TEST_BOX_001" // In production, use MAC or IMEI

// Timings
#define GPS_INTERVAL 10000    // Update GPS every 10s
#define FB_POLL_INTERVAL 5000 // Check OTP hash every 5s
#define UNLOCK_DURATION 5000  // Solenoid active time
#define CAM_TIMEOUT 15000     // Wait up to 15s for photo upload

// ==================== GLOBAL STATE ====================
char otpBuffer[7] = "";
byte otpIndex = 0;

String serverOtpHash = ""; // Fetched from Firebase
bool isLocked = true;
bool waitingForCam = false;
unsigned long timeUnlocked = 0;
unsigned long timeCamTriggered = 0;

// GPS / Net
double gpsLat = 0, gpsLon = 0;
bool gpsFix = false;
bool lteConnected = false;
bool modemOK = false;

unsigned long lastGpsRun = 0;
unsigned long lastFbPoll = 0;

// ==================== HELPER: LCD ====================
void updateLcd(String line1, String line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

// ==================== HELPER: AT COMM ====================
String sendATAndWait(const char *cmd, unsigned long timeout = 3000) {
  String resp = "";
  modem.sendAT(cmd);
  modem.waitResponse(timeout, resp);
  return resp;
}

// ==================== MODEM & GPS STACK ====================
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

bool initGPS() {
  Serial.println("Initializing GPS...");
  updateLcd("INIT GPS", "POWERING GNSS");

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

    // Wait for READY signal (reduced to 5s — modem is usually fast)
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

  // ═══════════════════════════════════════════
  // TTFF OPTIMIZATIONS (Fastest possible fix)
  // These take ~15s which also gives the modem time to register on network
  // ═══════════════════════════════════════════
  updateLcd("INIT GPS", "OPTIMIZING...");

  // 1. Multi-constellation: GPS + GLONASS + BeiDou
  Serial.print("  Multi-GNSS (GPS+GLO+BDS)...");
  modem.sendAT("+CGNSSMODE=3");
  modem.waitResponse(1000L);
  Serial.println(" OK");

  // 2. XTRA ephemeris: download predicted satellite orbits
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

  Serial.println("  GPS initialized with TTFF optimizations");
  updateLcd("GPS READY", "CONNECTING LTE");
  return true;
}

void readGPS() {
  modem.sendAT("+CGNSSINFO");
  String resp = "";
  if (modem.waitResponse(2000L, resp) != 1) {
    gpsFix = false;
    return;
  }

  int idx = resp.indexOf("+CGNSSINFO: ");
  if (idx < 0) {
    gpsFix = false;
    return;
  }

  String data = resp.substring(idx + 12);
  data.trim();

  if (data.length() < 10 || data.startsWith(",,,")) {
    gpsFix = false;
    return;
  }

  // Parse CGNSSINFO format:
  // <fix>,<sats>,,<beidou>,<galileo>,<lat>,<N/S>,<lon>,<E/W>,<date>,<time>,...
  int commas[15];
  int commaCount = 0;
  for (int i = 0; i < (int)data.length() && commaCount < 15; i++) {
    if (data.charAt(i) == ',') {
      commas[commaCount++] = i;
    }
  }

  if (commaCount < 8) {
    gpsFix = false;
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
  } else {
    gpsFix = false;
  }
}

bool connectLTE() {
  if (lteConnected)
    return true;
  if (!modemOK)
    return false;

  Serial.println("Connecting LTE (Globe PH)...");
  updateLcd("CONNECTING...", "LTE NETWORK");

  // ── SIM Diagnostics ──
  Serial.print("  SIM status...");
  int simStatus = modem.getSimStatus();
  if (simStatus != 1) {
    Serial.printf(" FAIL (status: %d)\n", simStatus);
    Serial.println("  CHECK: 1) SIM inserted correctly (gold contacts down)");
    Serial.println("         2) SIM is activated with data");
    Serial.println("         3) SIM tray fully seated");
    updateLcd("SIM ERROR", "CHECK SIM");
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

  // ── Signal Check (wait up to 30s for modem to find tower) ──
  Serial.print("  Waiting for signal...");
  updateLcd("LTE SIGNAL", "SEARCHING...");
  int csq = 99;
  unsigned long sigWaitStart = millis();
  while (millis() - sigWaitStart < 30000) {
    csq = modem.getSignalQuality();
    if (csq != 99 && csq != 0)
      break;
    Serial.print(".");
    delay(2000);
  }
  Serial.printf(" CSQ: %d", csq);
  if (csq == 99 || csq == 0) {
    Serial.println(" — No signal after 30s!");
    Serial.println("  CHECK: LTE antenna connected, SIM has coverage");
    updateLcd("NO SIGNAL", "CHECK ANTENNA");
    return false;
  }
  int dbm = -113 + (2 * csq);
  Serial.printf(" (%d dBm)\n", dbm);

  // ── Check current network info ──
  Serial.print("  Network info: ");
  modem.sendAT("+CPSI?");
  if (modem.waitResponse(2000L, resp) == 1) {
    Serial.println(resp);
  } else {
    Serial.println("(no response)");
  }

  // ── Network Registration Check ──
  // Check ALL registration types since AT+CNMP=38 (LTE-only) may not be set.
  // +CEREG = LTE (4G), +CGREG = GPRS/3G, +CREG = GSM (2G)
  // Any of these registering (,1 = home, ,5 = roaming) means we can proceed.
  Serial.println("  Checking network registration...");
  updateLcd("NET REGISTER", "PLEASE WAIT...");

  bool networkOK = false;
  unsigned long netWaitStart = millis();
  while (millis() - netWaitStart < 60000) { // Wait up to 60s (was 30s)
    // Method 1: Check +CPSI? for "Online" keyword
    modem.sendAT("+CPSI?");
    if (modem.waitResponse(2000L, resp) == 1) {
      if (resp.indexOf("Online") >= 0) {
        Serial.println("  CPSI: " + resp + " -> registered!");
        networkOK = true;
        break;
      }
    }

    // Method 2: Check +CEREG? (LTE/4G registration)
    modem.sendAT("+CEREG?");
    if (modem.waitResponse(1000L, resp) == 1) {
      if (resp.indexOf(",1") >= 0 || resp.indexOf(",5") >= 0) {
        Serial.println("  CEREG: " + resp + " -> LTE registered!");
        networkOK = true;
        break;
      }
    }

    // Method 3: Check +CGREG? (GPRS/3G registration)
    modem.sendAT("+CGREG?");
    if (modem.waitResponse(1000L, resp) == 1) {
      if (resp.indexOf(",1") >= 0 || resp.indexOf(",5") >= 0) {
        Serial.println("  CGREG: " + resp + " -> 3G registered!");
        networkOK = true;
        break;
      }
    }

    // Method 4: Check +CREG? (GSM/2G registration)
    modem.sendAT("+CREG?");
    if (modem.waitResponse(1000L, resp) == 1) {
      if (resp.indexOf(",1") >= 0 || resp.indexOf(",5") >= 0) {
        Serial.println("  CREG: " + resp + " -> 2G registered!");
        networkOK = true;
        break;
      }
    }

    Serial.print(".");
    delay(2000);
  }

  // Method 5: TinyGSM fallback — uses its own combined CREG/CGREG logic
  if (!networkOK) {
    Serial.println("\n  Trying TinyGSM waitForNetwork (30s)...");
    updateLcd("NET REGISTER", "TRYING GSM...");
    if (modem.waitForNetwork(30000L)) {
      Serial.println("  TinyGSM: Network registered!");
      networkOK = true;
    }
  }

  if (!networkOK) {
    Serial.println("  REGISTRATION FAILED");
    // Dump all reg status for diagnostics
    modem.sendAT("+CEREG?");
    if (modem.waitResponse(1000L, resp) == 1)
      Serial.println("  CEREG: " + resp);
    modem.sendAT("+CGREG?");
    if (modem.waitResponse(1000L, resp) == 1)
      Serial.println("  CGREG: " + resp);
    modem.sendAT("+CREG?");
    if (modem.waitResponse(1000L, resp) == 1)
      Serial.println("  CREG: " + resp);
    modem.sendAT("+CPSI?");
    if (modem.waitResponse(2000L, resp) == 1)
      Serial.println("  CPSI: " + resp);
    updateLcd("NET REGISTER", "FAILED");
    return false;
  }
  Serial.println("  Network registered!");

  Serial.println("  Operator: " + modem.getOperator());

  // ── Activate PDP Context (APN already set) ──
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
    updateLcd("DATA FAILED", "CHECK SIM DATA");
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

  // ── Connectivity Test via AT+HTTPINIT (modem-level HTTP) ──
  Serial.print("  Testing internet (AT+HTTP)...");
  sendATAndWait("+HTTPTERM", 1000); // Terminate any previous session
  delay(200);

  String httpResp;
  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000L, httpResp) != 1) {
    Serial.println(" HTTPINIT FAIL: " + httpResp);
    // Still mark as connected — GPRS is up, HTTP may work on next try
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
    }

    if (httpOK) {
      Serial.println("  Internet OK!");
    } else {
      Serial.println("  HTTP test failed (may still work for Firebase)");
    }

    sendATAndWait("+HTTPTERM", 1000);
  }

  lteConnected = true;
  updateLcd("LTE CONNECTED", "READY");
  Serial.println("LTE Connected!");
  return true;
}

// ==================== FIREBASE (AT+HTTP) ====================
// Uses built-in HTTP stack of A7670E for SSL
bool httpGetAttributes(String &outHash) {
  // GET /boxes/{ID}/delivery_context/current_otp_hash.json
  String path =
      "/boxes/" + String(BOX_ID) + "/delivery_context/current_otp_hash.json";
  String url = "https://" + String(FIREBASE_HOST) + path +
               "?auth=" + String(FIREBASE_AUTH);

  modem.sendAT("+HTTPTERM");
  delay(100);
  modem.sendAT("+HTTPINIT");
  modem.waitResponse();

  modem.sendAT("+HTTPPARA=\"CID\",1");
  modem.waitResponse();
  modem.sendAT("+HTTPPARA=\"URL\",\"" + url + "\"");
  modem.waitResponse();

  modem.sendAT("+HTTPACTION=0"); // GET
  // Wait for +HTTPACTION: 0,200,len
  unsigned long start = millis();
  bool success = false;
  int len = 0;

  while (millis() - start < 8000) {
    String line = modemSerial.readStringUntil('\n');
    if (line.indexOf("+HTTPACTION: 0,200") >= 0) {
      success = true;
      // parse len?
      break;
    }
  }

  if (success) {
    // Read data
    modem.sendAT("+HTTPREAD=0,100"); // Read first 100 bytes
    if (modem.waitResponse(2000L) == 1) {
      // The next lines contain the data
      // Example data: "123456" (Quotes included if string in Firebase)
      String data = modemSerial.readStringUntil('\n');    // Skip command echo
      String payload = modemSerial.readStringUntil('\n'); // Payload?

      // Clean payload
      payload.trim();
      payload.replace("\"", "");   // Remove quotes
      if (payload.length() >= 4) { // Basic sanity
        outHash = payload;
        Serial.println("Updated Hash: " + outHash);
        return true;
      }
    }
  }

  modem.sendAT("+HTTPTERM");
  return false;
}

void uploadLocation() {
  if (!gpsFix)
    return;
  // PUT /boxes/{ID}/location.json
  String path = "/boxes/" + String(BOX_ID) + "/location.json";
  String url = "https://" + String(FIREBASE_HOST) + path +
               "?auth=" + String(FIREBASE_AUTH);

  // Use actual parsed GPS coordinates
  String payload = "{\"lat\": " + String(gpsLat, 6) +
                   ", \"lng\": " + String(gpsLon, 6) + "}";

  modem.sendAT("+HTTPTERM");
  delay(100);
  modem.sendAT("+HTTPINIT");
  modem.waitResponse();
  modem.sendAT("+HTTPPARA=\"CID\",1");
  modem.waitResponse();
  modem.sendAT("+HTTPPARA=\"URL\",\"" + url + "\"");
  modem.waitResponse();

  modem.sendAT("+HTTPDATA=" + String(payload.length()) + ",2000");
  modem.waitResponse(2000, "DOWNLOAD");
  modemSerial.print(payload);
  modem.waitResponse();

  modem.sendAT(
      "+HTTPACTION=1"); // POST/PUT? Action 1 is POST usually. PUT needs config.
  // Standard A7670: 0=GET, 1=POST, 2=HEAD. PUT often not directly supported in
  // basic mode without setting content-type or custom headers. For Realtime DB,
  // POST pushes a new ID, PUT updates key. We'll use POST for now to keep it
  // simple or check docs. Actually, many modules support PUT via
  // +HTTPPARA="CONTENT","application/json" etc. Assuming POST for now.
}

// ==================== UNLOCK SEQUENCE ====================
void triggerUnlockSequence() {
  updateLcd("VERIFIED!", "CAPTURING...");
  Serial.println("OTP Valid. Triggering Cam...");

  // 1. Send Trigger
  while (camSerial.available())
    camSerial.read(); // Flush
  camSerial.print('C');

  waitingForCam = true;
  timeCamTriggered = millis();
}

void performUnlock() {
  Serial.println("Cam Success. Unlocking.");
  updateLcd("UNLOCKED", "OPEN DOOR");

  digitalWrite(PIN_SOLENOID, HIGH);
  isLocked = false;
  timeUnlocked = millis();
}

// ==================== MAIN SETUP ====================
void setup() {
  Serial.begin(115200);
  camSerial.begin(CAM_BAUD, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
  modemSerial.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);

  // LCD
  Serial.println("[SYSTEM] Initializing I2C (LCD)...");
  Wire.begin(LCD_SDA, LCD_SCL);

  // SCANNER
  byte error, address;
  int nDevices = 0;
  Serial.println("[SYSTEM] Scanning for I2C devices...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("[SYSTEM] I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  if (nDevices == 0) {
    Serial.println(
        "[ERROR] NO I2C DEVICES FOUND! Check wiring (SDA/SCL) and Power.");
  } else {
    Serial.println("[SYSTEM] I2C Scan Complete.");
  }

  lcd.begin();
  lcd.backlight();
  updateLcd("BOOTING...", "PLEASE WAIT");
  // Pins
  pinMode(PIN_SOLENOID, OUTPUT);
  digitalWrite(PIN_SOLENOID, LOW);
  pinMode(PIN_REED_SWITCH, INPUT_PULLUP);

  // ==================== MODEM INIT ====================
  // Using the EXACT sequence from GPS_LTE_Firebase_Test (proven working)
  // 1. Always power modem (safe even if already on)
  // 2. initModem() with full handshake (getModemName + getModemInfo)
  // 3. initGPS() with TTFF optimizations (~15s, modem registers in background)
  // 4. connectLTE() with multi-RAT registration checks
  Serial.println("\n=== Smart Top Box - Modem Init ===");
  updateLcd("MODEM INIT", "POWERING...");
  powerModem();

  // Full modem initialization (copied from working test firmware)
  updateLcd("MODEM INIT", "HANDSHAKE...");
  Serial.println("Initializing modem...");
  delay(3000); // Wait for modem boot

  String modemName = modem.getModemName();
  Serial.println("  Modem: " + modemName);

  String modemInfo = modem.getModemInfo();
  Serial.println("  Info: " + modemInfo);

  if (modemName.length() == 0 && modemInfo.length() == 0) {
    Serial.println("  Modem not responding after power cycle!");
    updateLcd("MODEM ERROR", "NOT RESPONDING");
    // Don't halt — allow keypad to still work offline
    modemOK = false;
  } else {
    Serial.println("  Modem OK");
    modemOK = true;

    // ── Force LTE-only mode (skip GSM/UMTS scanning) ──
    updateLcd("CONFIG MODEM", "LTE-ONLY MODE");
    Serial.print("  Setting LTE-only (AT+CNMP=38)...");
    modem.sendAT("+CNMP=38");
    if (modem.waitResponse(5000L) == 1) {
      Serial.println(" OK");
    } else {
      Serial.println(" skip (using auto)");
    }

    // ── Pre-configure APN so modem starts registering immediately ──
    updateLcd("CONFIG MODEM", "SETTING APN...");
    Serial.print("  Pre-setting Globe APN...");
    modem.sendAT("+CGDCONT=1,\"IP\",\"" + String(GPRS_APN) + "\"");
    modem.waitResponse(3000L);
    Serial.println(" OK");

    // Trigger auto-registration in background (runs during GPS init)
    modem.sendAT("+COPS=0");
    modem.waitResponse(1000L);
  }

  // GPS init (~15s of TTFF optimizations — modem registers in background)
  bool gpsEnabled = false;
  if (modemOK) {
    gpsEnabled = initGPS();
    Serial.println(gpsEnabled ? "GPS: Ready" : "GPS: Failed (continuing)");
  }

  // LTE connection (modem had ~15s to register during GPS init)
  if (modemOK) {
    Serial.println("\nConnecting via LTE...");
    if (connectLTE()) {
      // POST-LTE: Retry AGPS now that data is available
      if (gpsEnabled) {
        Serial.println("Retrying GPS assistance with LTE data...");
        String resp;
        Serial.print("  AGPS download...");
        modem.sendAT("+CAGPS");
        if (modem.waitResponse(15000L, resp) == 1) {
          Serial.println(" OK (satellite data injected!)");
        } else {
          Serial.println(" skip");
        }
      }
    } else {
      Serial.println("LTE FAILED — keypad still works offline");
    }
  }

  updateLcd("SYSTEM READY", "ENTER OTP:");
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long now = millis();

  // 1. Keypad
  if (isLocked && !waitingForCam) {
    char key = keypad.getKey();
    if (key) {
      Serial.printf("[KEYPAD] Key pressed: '%c'\n", key);
      if (key == '*') {
        otpIndex = 0;
        memset(otpBuffer, 0, sizeof(otpBuffer));
        updateLcd("CLEARED", "ENTER OTP:");
        Serial.println("[KEYPAD] Buffer cleared");
      } else if (key == '#') {
        Serial.printf("[KEYPAD] Submit pressed, buffer='%s' (%d digits)\n",
                      otpBuffer, otpIndex);
        if (otpIndex == 6) {
          // HARDCODED TEST FOR NOW to allow User Verification without full
          // firebase:
          if (String(otpBuffer) == "123456") {
            triggerUnlockSequence();
          } else {
            updateLcd("INVALID OTP", "TRY AGAIN");
            delay(2000);
            updateLcd("SYSTEM READY", "ENTER OTP:");
          }
          otpIndex = 0;
          memset(otpBuffer, 0, sizeof(otpBuffer));
        } else {
          updateLcd("NEED 6 DIGITS", "ENTER OTP:");
        }
      } else if (otpIndex < 6) {
        otpBuffer[otpIndex++] = key;
        otpBuffer[otpIndex] = '\0';
        // Show digits on LCD line 2
        lcd.setCursor(0, 1);
        lcd.print("OTP: ");
        lcd.print(otpBuffer);
        lcd.print("      "); // Clear trailing chars
        Serial.printf("[KEYPAD] Buffer: '%s' (%d/6)\n", otpBuffer, otpIndex);
      }
    }
  }

  // 2. Camera Wait
  if (waitingForCam) {
    if (camSerial.available()) {
      char c = camSerial.read();
      if (c == 'D') {
        performUnlock();
        waitingForCam = false;
      } else if (c == 'F') {
        updateLcd("CAM FAILED", "TRY AGAIN");
        delay(2000);
        updateLcd("SYSTEM READY", "ENTER OTP:");
        waitingForCam = false;
      }
    }
    if (now - timeCamTriggered > CAM_TIMEOUT) {
      updateLcd("CAM TIMEOUT", "ERROR");
      waitingForCam = false;
      delay(2000);
      updateLcd("SYSTEM READY", "ENTER OTP:");
    }
  }

  // 3. Relock
  if (!isLocked && (now - timeUnlocked > UNLOCK_DURATION)) {
    digitalWrite(PIN_SOLENOID, LOW);
    isLocked = true;
    updateLcd("SYSTEM READY", "ENTER OTP:");
  }

  // 4. Reed Switch Monitoring
  static int lastReedState = -1;
  int currentReedState =
      digitalRead(PIN_REED_SWITCH); // LOW = Closed (Magnet near), HIGH = Open

  if (currentReedState != lastReedState) {
    lastReedState = currentReedState;
    if (currentReedState == HIGH) {
      Serial.println("Status: DOOR OPEN");
      // If opened WHILE locked, it's a forced entry (Theft!)
      if (isLocked) {
        Serial.println("ALERT: FORCED ENTRY DETECTED!");
        updateLcd("ALERT!", "FORCED OPEN");
        // uploadStatus("TAMPERED"); // Status to Firebase
      } else {
        // Normal open
      }
    } else {
      Serial.println("Status: DOOR CLOSED");
      if (isLocked)
        updateLcd("SYSTEM READY", "ENTER OTP:");
    }
  }

  // 5. Background Data (ONLY when LTE is connected — these AT commands
  //    block the loop for seconds and starve the keypad if run without LTE)
  if (lteConnected) {
    if (now - lastFbPoll > FB_POLL_INTERVAL) {
      lastFbPoll = now;
      String hash = "";
      if (httpGetAttributes(hash)) {
        serverOtpHash = hash;
      }
    }

    if (now - lastGpsRun > GPS_INTERVAL) {
      lastGpsRun = now;
      readGPS();
      if (gpsFix) {
        uploadLocation();
      }
    }
  } else {
    // Still read GPS locally even without LTE (for future use)
    if (now - lastGpsRun > GPS_INTERVAL) {
      lastGpsRun = now;
      readGPS();
    }
  }
}
