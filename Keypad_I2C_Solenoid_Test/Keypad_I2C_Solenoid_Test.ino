/**
 * KEYPAD + I2C LCD + SOLENOID — Integration Test
 *
 * Hardware:  LILYGO T-SIM A7670E (ESP32)
 * Purpose:  Enter 6-digit OTP on the keypad, display it on the I2C LCD,
 *           and engage the solenoid if the code matches "123456".
 *
 * Pinout (mirrors Smart_Top_Box_Main):
 *   Keypad Rows : GPIO 13, 14, 15, 32
 *   Keypad Cols : GPIO 18, 19, 23
 *   LCD SDA     : GPIO 21
 *   LCD SCL     : GPIO 22
 *   Solenoid    : GPIO 25 (driven via MOSFET — active HIGH)
 *
 * Controls:
 *   0-9  → Digit input (max 6 digits)
 *   *    → Clear / reset input
 *   #    → Submit OTP
 *
 * Solenoid safety: Article 2.3 — hard limit of 5 000 ms activation.
 */

#include <Arduino.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// ==================== PIN DEFINITIONS ====================
// These match Smart_Top_Box_Main.ino exactly.
#define PIN_SOLENOID 25

#define LCD_SDA 21
#define LCD_SCL 22

const byte ROWS = 4;
const byte COLS = 3;
byte rowPins[ROWS] = {14, 23, 19, 15};
byte colPins[COLS] = {13, 5, 18};

char keys[ROWS][COLS] = {
    {'1', '2', '3'}, {'4', '5', '6'}, {'7', '8', '9'}, {'*', '0', '#'}};

// ==================== PERIPHERALS ====================
LiquidCrystal_I2C lcd(0x27, 16, 2);
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ==================== CONFIGURATION ====================
static const char CORRECT_OTP[] = "123456";        // Test OTP
static const unsigned long SOLENOID_MAX_MS = 5000; // Art. 2.3 thermal limit

// ==================== STATE ====================
char otpBuffer[7] = ""; // 6 digits + null
byte otpIndex = 0;
bool solenoidActive = false;
unsigned long solenoidOnTime = 0;

// ==================== HELPERS ====================

/** Update both lines of the 16×2 LCD in one call. */
void lcdPrint(const char *line1, const char *line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

/** Show the current OTP entry state on the LCD. */
void showEntryState() {
  lcd.setCursor(0, 0);
  lcd.print("ENTER OTP:      "); // pad to clear old chars

  char display[17]; // 16 chars + null
  // Build a visual like "12____" (typed digits + underscores)
  byte i = 0;
  for (; i < otpIndex; i++) {
    display[i] = otpBuffer[i];
  }
  for (; i < 6; i++) {
    display[i] = '_';
  }
  display[6] = '\0';

  lcd.setCursor(0, 1);
  lcd.print(display);
  lcd.print("          "); // clear trailing chars
}

/** Reset the input buffer and refresh the LCD. */
void clearInput() {
  otpIndex = 0;
  memset(otpBuffer, 0, sizeof(otpBuffer));
  showEntryState();
}

/** Activate solenoid (non-blocking, guarded by thermal limit). */
void solenoidOpen() {
  if (solenoidActive)
    return; // already open
  digitalWrite(PIN_SOLENOID, HIGH);
  solenoidActive = true;
  solenoidOnTime = millis();
  Serial.println("[SOLENOID] OPEN");
}

/** Deactivate solenoid. */
void solenoidClose() {
  if (!solenoidActive)
    return;
  digitalWrite(PIN_SOLENOID, LOW);
  solenoidActive = false;
  Serial.println("[SOLENOID] CLOSED");
}

// ==================== I2C SCANNER ====================
void scanI2C() {
  Serial.println("[I2C] Scanning bus (SDA=21, SCL=22)...");
  byte found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("[I2C]   Device at 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("[I2C]   ** NO DEVICES FOUND — check wiring / power **");
  } else {
    Serial.printf("[I2C]   %d device(s) detected.\n", found);
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("========================================");
  Serial.println(" KEYPAD + I2C LCD + SOLENOID TEST");
  Serial.println(" Correct OTP: 123456");
  Serial.println("========================================");

  // — I2C bus —
  Wire.begin(LCD_SDA, LCD_SCL);
  scanI2C();

  // — LCD —
  lcd.begin();
  lcd.backlight();
  lcdPrint("SELF-TEST", "INITIALIZING...");
  Serial.println("[LCD] Initialized at 0x27");

  // — Solenoid pin —
  pinMode(PIN_SOLENOID, OUTPUT);
  digitalWrite(PIN_SOLENOID, LOW);
  Serial.println("[SOLENOID] Pin 25 configured (LOW)");

  // Brief solenoid pulse so the tester can hear/feel it click
  Serial.println("[SOLENOID] Quick click test...");
  lcdPrint("SELF-TEST", "SOLENOID CLICK");
  digitalWrite(PIN_SOLENOID, HIGH);
  delay(200); // 200 ms — safe, well below thermal limit
  digitalWrite(PIN_SOLENOID, LOW);
  Serial.println("[SOLENOID] Click OK");

  delay(500);

  // Ready
  clearInput();
  Serial.println("[SYSTEM] Ready.  * = clear, # = submit");
}

// ==================== LOOP ====================
void loop() {
  // ── Solenoid thermal guard (Art. 2.3) ──
  if (solenoidActive && (millis() - solenoidOnTime >= SOLENOID_MAX_MS)) {
    Serial.println("[SOLENOID] Thermal limit reached — forcing close");
    solenoidClose();
    lcdPrint("TIMEOUT", "DOOR RE-LOCKED");
    delay(1500);
    clearInput();
  }

  // ── Keypad polling ──
  char key = keypad.getKey();
  if (key == NO_KEY)
    return;

  Serial.printf("[KEYPAD] Pressed: %c\n", key);

  // — CLEAR —
  if (key == '*') {
    if (solenoidActive) {
      solenoidClose(); // also re-lock if open
    }
    clearInput();
    Serial.println("[INPUT] Cleared");
    return;
  }

  // — SUBMIT —
  if (key == '#') {
    if (otpIndex != 6) {
      Serial.printf("[INPUT] Rejected: only %d digit(s)\n", otpIndex);
      lcdPrint("NEED 6 DIGITS!", "");
      delay(1000);
      showEntryState();
      return;
    }

    Serial.printf("[INPUT] Submitted OTP: %s\n", otpBuffer);

    if (strcmp(otpBuffer, CORRECT_OTP) == 0) {
      // ✓ Correct
      Serial.println("[OTP] MATCH — unlocking solenoid");
      lcdPrint("OTP CORRECT!", "SOLENOID OPEN");
      solenoidOpen();

      // The solenoid stays open until the thermal guard closes it,
      // or the user presses '*' to manually re-lock & reset.
    } else {
      // ✗ Wrong
      Serial.println("[OTP] WRONG — access denied");
      lcdPrint("WRONG OTP!", otpBuffer);
      delay(2000);
      clearInput();
    }
    return;
  }

  // — DIGIT (0-9) —
  if (otpIndex < 6) {
    otpBuffer[otpIndex++] = key;
    otpBuffer[otpIndex] = '\0';
    showEntryState();
  }
  // Silently ignore extra digits beyond the 6th.
}
