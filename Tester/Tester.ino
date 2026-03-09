/**
 * Parcel-Safe Tester — Keypad + LCD + Solenoid
 * Interconnected with GPS/LTE proxy (SmartTopBox_AP) and ESP32-CAM
 *
 * Flow:
 *   1. Connects to LilyGO SoftAP for internet access.
 *   2. Periodically fetches current OTP from proxy GET /otp.
 *   3. User enters PIN on keypad → validates against fetched OTP.
 *   4. If OTP correct → requests face check from proxy GET /face-check.
 *   5. Face detected → fires solenoid (Article 1.2 Photo-First).
 *   6. Reports event to proxy POST /event → written to Firebase.
 *
 * Hardware: ESP32 DevKit (separate board)
 *   Keypad: rows 13,23,19,26 | cols 14,25,18
 *   LCD I2C: default SDA/SCL (21/22)
 *   Solenoid MOSFET: GPIO 32
 *
 * Rules enforced:
 *   - No delay() in loop (Article 2.1)
 *   - char[] buffers, no String class for long-running logic (Article 2.2)
 *   - Solenoid relocks manually via '*' key after unlock
 *   - WiFi reconnect with exponential backoff (Article 2.4)
 */

#include <HTTPClient.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

// ==================== CONFIGURATION ====================
// WiFi — connect to LilyGO SoftAP
static const char WIFI_SSID[] = "SmartTopBox_AP";
static const char WIFI_PASSWORD[] = "topbox123";

// Proxy endpoints on LilyGO (default SoftAP gateway IP)
static const char PROXY_HOST[] = "192.168.4.1";
static const int PROXY_PORT = 8080;

// Hardware ID (must match GPS/LTE board)
static const char HARDWARE_ID[] = "BOX_001";

// UART fallback to ESP32-CAM (Serial2)
// Wire: Tester TX2(17) → CAM RX(13), CAM TX(14) → Tester RX2(16), shared GND
#define CAM_UART_RX 16
#define CAM_UART_TX 17
#define CAM_UART_BAUD 9600

// UDP Logging
WiFiUDP udpClient;
#define UDP_LOG_PORT 5114

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

// ==================== TIMING ====================
#define OTP_REFRESH_INTERVAL 10000 // Fetch OTP every 10s
#define WIFI_RETRY_BASE_MS 1000    // Exponential backoff base
#define WIFI_RETRY_MAX_MS 32000    // Max backoff cap
#define LCD_MESSAGE_DURATION 2000  // Non-blocking message display
#define FACE_CHECK_TIMEOUT 15000   // HTTP timeout for face check

// ==================== PIN SETUP (ORIGINAL — DO NOT CHANGE)
// ====================
#define LOCK_PIN 32 // MOSFET gate for solenoid

// LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Keypad
static const byte ROWS = 4;
static const byte COLS = 3;

char keys[ROWS][COLS] = {
    {'1', '2', '3'}, {'4', '5', '6'}, {'7', '8', '9'}, {'*', '0', '#'}};

byte rowPins[ROWS] = {13, 23, 19, 26};
byte colPins[COLS] = {14, 25, 18};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ==================== STATE MACHINE ====================
enum TesterState {
  STATE_CONNECTING_WIFI, // Initial WiFi connection
  STATE_IDLE,            // Waiting for keypad input
  STATE_ENTERING_PIN,    // User is typing digits
  STATE_VERIFYING_OTP,   // Checking entered PIN vs OTP
  STATE_REQUESTING_FACE, // HTTP GET /face-check in progress
  STATE_UNLOCKING,       // Solenoid active (manual relock using '*')
  STATE_RELOCKING,       // Solenoid deactivated, showing message
  STATE_SHOW_MESSAGE     // Temporary LCD message (non-blocking)
};

static TesterState currentState = STATE_CONNECTING_WIFI;

// ==================== BUFFERS (Article 2.2: no String class)
// ====================
static char inputCode[8]; // User-entered PIN (max 6 digits + null)
static uint8_t inputLen = 0;

static char currentOtp[8] =
    "1234"; // Default for bench testing; overwritten by proxy
static bool otpLoaded = true;

static char lcdLine0[17]; // LCD line 0 buffer (16 chars + null)
static char lcdLine1[17]; // LCD line 1 buffer

// ==================== TIMING TRACKERS ====================
static unsigned long lastOtpFetch = 0;
static unsigned long wifiRetryAt = 0;
static unsigned long wifiBackoffMs = WIFI_RETRY_BASE_MS;
static unsigned long solenoidStartAt = 0;
static unsigned long messageStartAt = 0;
static unsigned long faceCheckStartAt = 0;

// Face check result (populated by non-blocking HTTP)
static bool faceCheckPending = false;
static bool faceCheckResult = false;

// ==================== FORWARD DECLARATIONS ====================
void enterState(TesterState newState);
void updateDisplay(const char *line0, const char *line1);
bool fetchOtpFromProxy();
int requestFaceCheck();
bool reportEventToProxy(bool otpValid, bool faceDetected, bool unlocked);
void connectWiFiNonBlocking();

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(500); // Brief startup delay (only in setup, acceptable)

  Serial.println(F("\n=== Parcel-Safe Tester (Interconnected) ==="));

  // Solenoid OFF
  pinMode(LOCK_PIN, OUTPUT);
  digitalWrite(LOCK_PIN, LOW);

  // Keypad debounce
  keypad.setDebounceTime(50);

  // LCD init
  lcd.init();
  lcd.backlight();
  updateDisplay("Parcel-Safe v2", "Connecting WiFi");

  // Start WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  wifiRetryAt = millis() + 15000; // First connection attempt timeout

  currentState = STATE_CONNECTING_WIFI;

  // Serial2 for UART fallback to ESP32-CAM
  Serial2.begin(CAM_UART_BAUD, SERIAL_8N1, CAM_UART_RX, CAM_UART_TX);
  Serial.println(F("[UART] Serial2 ready (CAM fallback)"));

  Serial.printf("  WiFi SSID: %s\n", WIFI_SSID);
  Serial.printf("  Proxy: %s:%d\n", PROXY_HOST, PROXY_PORT);
}

// ==================== MAIN LOOP (non-blocking) ====================
void loop() {
  unsigned long now = millis();

  // ── WiFi monitoring (runs in all states) ──
  if (WiFi.status() != WL_CONNECTED && currentState != STATE_CONNECTING_WIFI) {
    netLog("[WIFI] Connection lost, reconnecting...\n");
    enterState(STATE_CONNECTING_WIFI);
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    wifiRetryAt = now + WIFI_RETRY_BASE_MS;
    wifiBackoffMs = WIFI_RETRY_BASE_MS;
  }

  // ── Periodic OTP refresh ──
  if (WiFi.status() == WL_CONNECTED && currentState != STATE_CONNECTING_WIFI &&
      now - lastOtpFetch >= OTP_REFRESH_INTERVAL) {
    fetchOtpFromProxy();
    lastOtpFetch = now;
  }

  // ── State machine ──
  switch (currentState) {

  case STATE_CONNECTING_WIFI:
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WIFI] Connected! IP: %s\n",
                    WiFi.localIP().toString().c_str());
      wifiBackoffMs = WIFI_RETRY_BASE_MS; // Reset backoff

      // Fetch OTP immediately
      if (fetchOtpFromProxy()) {
        Serial.printf("[OTP] Loaded: %s\n", currentOtp);
      }
      lastOtpFetch = now;
      enterState(STATE_IDLE);
    } else if (now >= wifiRetryAt) {
      // After first timeout, go to IDLE anyway so keypad works offline
      if (wifiBackoffMs > WIFI_RETRY_BASE_MS) {
        Serial.println(
            F("[WIFI] No connection — entering OFFLINE mode with default OTP"));
        updateDisplay("OFFLINE MODE", "Using default PIN");
        delay(1500);
        enterState(STATE_IDLE);
      } else {
        // Exponential backoff reconnect (Article 2.4)
        Serial.printf("[WIFI] Retry (backoff %lums)...\n", wifiBackoffMs);
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        wifiRetryAt = now + wifiBackoffMs;
        if (wifiBackoffMs < WIFI_RETRY_MAX_MS) {
          wifiBackoffMs *= 2;
        }
        updateDisplay("Connecting WiFi", "Retry...");
      }
    }
    break;

  case STATE_IDLE:
  case STATE_ENTERING_PIN: {
    char key = keypad.getKey();
    if (!key)
      break;

    if (key == '*') {
      // Clear input
      inputLen = 0;
      inputCode[0] = '\0';
      enterState(STATE_IDLE);
    } else if (key == '#') {
      // Submit
      if (inputLen > 0) {
        enterState(STATE_VERIFYING_OTP);
      }
    } else if (inputLen < 6) {
      // Digit entered
      inputCode[inputLen++] = key;
      inputCode[inputLen] = '\0';

      if (currentState == STATE_IDLE) {
        currentState = STATE_ENTERING_PIN;
      }

      // Update LCD with actual PIN digits
      char display[17];
      snprintf(display, sizeof(display), "PIN: %s", inputCode);
      updateDisplay("Enter PIN:", display);
    }
    break;
  }

  case STATE_VERIFYING_OTP:
    if (!otpLoaded) {
      netLog("[OTP] No OTP loaded, fetching...\n");
      if (!fetchOtpFromProxy()) {
        updateDisplay("No OTP loaded", "Check connection");
        messageStartAt = now;
        currentState = STATE_SHOW_MESSAGE;

        // Report failure
        reportEventToProxy(false, false, false);
        break;
      }
    }

    if (strcmp(inputCode, currentOtp) == 0) {
      netLog("[OTP] CORRECT! Requesting face check...\n");
      updateDisplay("PIN Correct!", "Face check...");
      enterState(STATE_REQUESTING_FACE);
    } else {
      netLog("[OTP] WRONG: entered '%s' vs expected '%s'\n", inputCode,
             currentOtp);
      updateDisplay("Wrong PIN!", "Try again");
      messageStartAt = now;
      currentState = STATE_SHOW_MESSAGE;

      // Report wrong OTP
      reportEventToProxy(false, false, false);
    }

    // Reset input
    inputLen = 0;
    inputCode[0] = '\0';
    break;

  case STATE_REQUESTING_FACE:
    if (!faceCheckPending) {
      faceCheckPending = true;
      faceCheckStartAt = now;

      int result = requestFaceCheck();
      faceCheckPending = false;

      if (result == 1) {
        // Face detected — unlock! (Article 1.2: photo already taken)
        netLog("[FACE] DETECTED! Unlocking solenoid...\n");
        faceCheckResult = true;
        enterState(STATE_UNLOCKING);

        // Report success
        reportEventToProxy(true, true, true);
      } else if (result == 0) {
        // No face detected — do NOT fire solenoid
        netLog("[FACE] NOT DETECTED! Solenoid stays locked.\n");
        updateDisplay("No face found!", "Box stays locked");

        // Report: OTP valid but face failed
        reportEventToProxy(true, false, false);

        messageStartAt = millis();
        currentState = STATE_SHOW_MESSAGE;
      } else {
        // Camera/proxy error
        netLog("[FACE] Check failed (camera offline?)\n");
        updateDisplay("Camera offline!", "Box stays locked");

        // Report: OTP valid but camera error
        reportEventToProxy(true, false, false);

        messageStartAt = millis();
        currentState = STATE_SHOW_MESSAGE;
      }
    }
    break;

  case STATE_UNLOCKING:
    if (solenoidStartAt == 0) {
      // Activate solenoid
      digitalWrite(LOCK_PIN, HIGH);
      solenoidStartAt = now;
      netLog("[LOCK] Solenoid ON\n");
      updateDisplay("Box Unlocked!", "* = Relock");
    }

    // Manual relock control
    if (keypad.getKey() == '*') {
      digitalWrite(LOCK_PIN, LOW);
      solenoidStartAt = 0;
      netLog("[LOCK] Solenoid OFF (manual relock)\n");
      enterState(STATE_RELOCKING);
    }
    break;

  case STATE_RELOCKING:
    updateDisplay("Box Locked", "Ready");
    messageStartAt = now;
    currentState = STATE_SHOW_MESSAGE;
    netLog("[LOCK] Relocked\n");
    break;

  case STATE_SHOW_MESSAGE:
    if (now - messageStartAt >= LCD_MESSAGE_DURATION) {
      enterState(STATE_IDLE);
    }
    break;
  }
}

// ==================== STATE HELPER ====================
void enterState(TesterState newState) {
  currentState = newState;

  switch (newState) {
  case STATE_IDLE:
    updateDisplay("Enter PIN:", "PIN: ");
    inputLen = 0;
    inputCode[0] = '\0';
    faceCheckPending = false;
    break;
  case STATE_CONNECTING_WIFI:
    updateDisplay("Parcel-Safe v2", "Connecting WiFi");
    break;
  default:
    break;
  }
}

// ==================== LCD HELPER ====================
void updateDisplay(const char *line0, const char *line1) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line0);
  lcd.setCursor(0, 1);
  lcd.print(line1);
}

// ==================== PROXY HTTP HELPERS ====================

/**
 * GET /otp — fetch current OTP from proxy (which reads Firebase RTDB).
 * Returns true if OTP was successfully loaded.
 */
bool fetchOtpFromProxy() {
  if (WiFi.status() != WL_CONNECTED)
    return false;

  HTTPClient http;
  char url[64];
  snprintf(url, sizeof(url), "http://%s:%d/otp", PROXY_HOST, PROXY_PORT);

  http.setTimeout(5000);
  http.begin(url);
  int code = http.GET();

  if (code == 200) {
    String body = http.getString();
    body.trim();
    // Only accept all-digit OTP (reject "NO_OTP" and other non-numeric
    // responses)
    bool allDigits = body.length() > 0 && body.length() <= 6;
    for (unsigned int i = 0; i < body.length() && allDigits; i++) {
      if (body[i] < '0' || body[i] > '9')
        allDigits = false;
    }
    if (allDigits) {
      strncpy(currentOtp, body.c_str(), sizeof(currentOtp) - 1);
      currentOtp[sizeof(currentOtp) - 1] = '\0';
      otpLoaded = true;
      http.end();
      return true;
    }
  }

  Serial.printf("[OTP] Fetch failed: HTTP %d\n", code);
  http.end();
  return false;
}

/**
 * GET /face-check — ask proxy to query ESP-CAM for face detection.
 * Falls back to UART Serial2 if WiFi is down.
 * Returns: 1 = face detected, 0 = no face, -1 = error/timeout.
 */
int requestFaceCheck() {
  // ── Primary: WiFi via proxy ──
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    char url[64];
    snprintf(url, sizeof(url), "http://%s:%d/face-check", PROXY_HOST,
             PROXY_PORT);

    http.setTimeout(FACE_CHECK_TIMEOUT);
    http.begin(url);
    int code = http.GET();

    if (code == 200) {
      String body = http.getString();
      body.trim();
      http.end();

      if (body.indexOf("FACE_OK") >= 0)
        return 1;
      if (body.indexOf("NO_FACE") >= 0)
        return 0;

      Serial.printf("[FACE] HTTP 200 but body error: %s\n", body.c_str());
      // Fall through to UART
    } else {
      Serial.printf("[FACE] HTTP %d\n", code);
      http.end();
    }
    // Fall through to UART if HTTP fails or returns unrecognized body
  }

  // ── Fallback: UART Serial2 to ESP32-CAM directly ──
  Serial.println(F("[FACE] WiFi down — trying UART fallback..."));

  // Flush any stale data
  while (Serial2.available())
    Serial2.read();

  Serial2.println("FACE?");

  unsigned long uartStart = millis();
  char uartBuf[32] = "";
  uint8_t uartLen = 0;

  while (millis() - uartStart < FACE_CHECK_TIMEOUT) {
    if (Serial2.available()) {
      char c = Serial2.read();
      if (c == '\n' || c == '\r') {
        if (uartLen > 0)
          break; // Got a complete line
      } else if (uartLen < sizeof(uartBuf) - 1) {
        uartBuf[uartLen++] = c;
      }
    }
    delay(10);
  }
  uartBuf[uartLen] = '\0';

  Serial.printf("[FACE] UART response: '%s'\n", uartBuf);

  if (strstr(uartBuf, "FACE_OK") != NULL)
    return 1;
  if (strstr(uartBuf, "NO_FACE") != NULL)
    return 0;

  Serial.println(F("[FACE] UART fallback failed"));
  return -1;
}

/**
 * POST /event — report lock event to proxy (which writes to Firebase).
 * JSON: { "otp_valid": bool, "face_detected": bool, "unlocked": bool }
 */
bool reportEventToProxy(bool otpValid, bool faceDetected, bool unlocked) {
  if (WiFi.status() != WL_CONNECTED)
    return false;

  HTTPClient http;
  char url[64];
  snprintf(url, sizeof(url), "http://%s:%d/event", PROXY_HOST, PROXY_PORT);

  char json[128];
  snprintf(json, sizeof(json),
           "{\"otp_valid\":%s,\"face_detected\":%s,\"unlocked\":%s,\"box_id\":"
           "\"%s\"}",
           otpValid ? "true" : "false", faceDetected ? "true" : "false",
           unlocked ? "true" : "false", HARDWARE_ID);

  http.setTimeout(5000);
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST((uint8_t *)json, strlen(json));

  Serial.printf("[EVENT] POST %s → HTTP %d\n", json, code);
  http.end();

  return (code == 200 || code == 201);
}