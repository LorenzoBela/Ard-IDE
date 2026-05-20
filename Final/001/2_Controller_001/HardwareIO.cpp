/**
 * HardwareIO.cpp — LCD and Keypad implementation
 *
 * Solenoid control has moved to LockSafety.cpp.
 *
 * Keypad scanning uses a CUSTOM matrix scan instead of the Keypad library's
 * internal scanKeys(). The library sets inactive rows to INPUT (floating),
 * which causes GPIO 13 (JTAG MTCK pin) to drift LOW and ghost Column 1
 * reads — pressing '5' or '8' would falsely register as '2'.
 *
 * Our scan explicitly uses INPUT_PULLUP on inactive rows so they stay HIGH
 * and cannot create parasitic paths through pressed keys.
 */

#include "HardwareIO.h"
#include <Wire.h>
#include <Arduino.h>

// ── LCD ──
static LiquidCrystal_I2C lcd(DISPLAY_I2C_ADDR, 16, 2);
static bool lcdBacklightEnabled = true;
static bool manualBacklightOverride = false;
static bool lcdReady = false;

// ── Keypad (manual scan — no Keypad.h library) ──
static const char keys[KP_ROWS][KP_COLS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}
};
static const uint8_t rowPins[KP_ROWS] = {13, 23, 19, 26};
static const uint8_t colPins[KP_COLS] = {14, 25, 18};

static const unsigned long KEYPAD_DEBOUNCE_MS = 50;
static const unsigned long KEYPAD_BUFFER_COOLDOWN_MS = 60;

static QueueHandle_t keyQueue = NULL;

// ── Manual hold tracking for EC-82 stuck detection ──
static volatile char heldKeyAtomic = 0;
static char  lastScanKey = 0;
static unsigned long lastScanKeyAt = 0;
static unsigned long keyReleasedAt = 0;
static const unsigned long HOLD_THRESHOLD_MS = 500;

// ── Custom matrix scanner ──
// Scans one key at a time. Inactive rows are INPUT_PULLUP (HIGH) to prevent
// GPIO 13 JTAG drift from ghosting column reads.
static char scanMatrix() {
  char foundKey = 0;

  for (uint8_t r = 0; r < KP_ROWS; r++) {
    // Drive this row LOW
    pinMode(rowPins[r], OUTPUT);
    digitalWrite(rowPins[r], LOW);

    // Brief settle time for the row to stabilize
    delayMicroseconds(20);

    // Read each column
    for (uint8_t c = 0; c < KP_COLS; c++) {
      if (digitalRead(colPins[c]) == LOW) {
        foundKey = keys[r][c];
      }
    }

    // Deactivate row — INPUT_PULLUP keeps it HIGH so it cannot ghost
    pinMode(rowPins[r], INPUT_PULLUP);

    // If we found a key, stop scanning (single-key mode)
    if (foundKey) break;
  }

  return foundKey;
}

static void initLcdSequence(bool fast) {
  Serial.println(F("[HW] LCD init start"));
  Serial.flush();
  Wire.begin(21, 22);
  Wire.setTimeOut(50);
  Wire.setClock(100000);
  delay(fast ? 20 : 80);

  Wire.beginTransmission(DISPLAY_I2C_ADDR);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    lcdReady = false;
    lcdBacklightEnabled = false;
    Serial.printf("[HW] LCD not detected at 0x%02X (err=%u), skipping LCD init\n",
                  DISPLAY_I2C_ADDR, err);
    Serial.flush();
    return;
  }

  for (int i = 0; i < 2; i++) {
    lcd.begin();
    delay(fast ? 20 : 50);
    lcd.clear();
    lcd.home();
  }
  lcd.backlight();
  lcdBacklightEnabled = true;
  lcdReady = true;
  Serial.println(F("[HW] LCD init done"));
  Serial.flush();
}

/**
 * Keypad task — Core 1, priority 2.
 *
 * Custom matrix scan with software debounce. No Keypad.h library.
 * Inactive rows use INPUT_PULLUP to prevent JTAG GPIO 13 ghosting.
 */
static void keypadTask(void *pvParameters) {
  (void)pvParameters;

  static char prevKey = 0;
  static unsigned long debounceStart = 0;
  static bool debounced = false;
  static unsigned long lastBufferTime = 0;

  while (true) {
    char raw = scanMatrix();
    unsigned long now = millis();

    if (raw != 0 && raw == prevKey) {
      // Same key seen consecutively — debounce it
      if (!debounced && (now - debounceStart >= KEYPAD_DEBOUNCE_MS)) {
        debounced = true;

        // Buffer the press (with cooldown to avoid repeats)
        if (now - lastBufferTime > KEYPAD_BUFFER_COOLDOWN_MS) {
          if (keyQueue) {
            xQueueSend(keyQueue, &raw, 0);
          }
          lastBufferTime = now;
          CTRL_LOG_PRINTF("[KEYPAD] Buffered: %c\n", raw);
        }

        // Track for hold detection
        lastScanKey = raw;
        lastScanKeyAt = now;
      }

      // Update held key status
      if (debounced && lastScanKey != 0) {
        unsigned long held = now - lastScanKeyAt;
        if (held >= HOLD_THRESHOLD_MS) {
          heldKeyAtomic = lastScanKey;
        }
      }
    } else if (raw != 0 && raw != prevKey) {
      // New key — start debounce timer
      prevKey = raw;
      debounceStart = now;
      debounced = false;
    } else {
      // No key pressed — released
      if (prevKey != 0) {
        prevKey = 0;
        debounced = false;
        heldKeyAtomic = 0;
        lastScanKey = 0;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz scan
  }
}

// ==================== INIT ====================
void initHardwareIO() {
  Serial.println(F("[HW] init start"));
  Serial.flush();

  // Set all column pins as INPUT_PULLUP (read side of matrix)
  for (uint8_t c = 0; c < KP_COLS; c++) {
    pinMode(colPins[c], INPUT_PULLUP);
  }

  // Set all row pins as INPUT_PULLUP (idle HIGH, prevents JTAG ghost on GPIO 13)
  for (uint8_t r = 0; r < KP_ROWS; r++) {
    pinMode(rowPins[r], INPUT_PULLUP);
  }

  Serial.println(F("[HW] keypad configured (custom scan, no Keypad.h)"));
  Serial.flush();

  keyQueue = xQueueCreate(10, sizeof(char));
  if (!keyQueue) {
    Serial.println(F("[HW] key queue create failed"));
    Serial.flush();
  }

  initLcdSequence(false);
  updateDisplay("Parcel-Safe v2", "Connecting WiFi");
  Serial.println(F("[HW] LCD splash written"));
  Serial.flush();

  Serial2.begin(CAM_UART_BAUD, SERIAL_8N1, CAM_UART_RX, CAM_UART_TX);
  CTRL_LOG_PRINTLN(F("[UART] Serial2 ready (CAM fallback)"));
  Serial.println(F("[HW] Serial2 ready"));
  Serial.flush();

  BaseType_t taskOk = xTaskCreatePinnedToCore(keypadTask, "KeypadTask", 4096, NULL, 2, NULL, 1);
  Serial.println(taskOk == pdPASS ? F("[HW] keypad task started") : F("[HW] keypad task failed"));
  Serial.flush();
}

// ==================== LCD ====================
void updateDisplay(const char *line0, const char *line1) {
  if (!lcdReady) return;
  char buf[17];
  lcd.setCursor(0, 0);
  snprintf(buf, sizeof(buf), "%-16s", line0 ? line0 : "");
  lcd.print(buf);
  lcd.setCursor(0, 1);
  snprintf(buf, sizeof(buf), "%-16s", line1 ? line1 : "");
  lcd.print(buf);
}

void displayBacklightOn() {
  if (!lcdReady) return;
  if (!lcdBacklightEnabled) {
    lcd.backlight();
    lcdBacklightEnabled = true;
  }
}

void displayBacklightOff() {
  if (manualBacklightOverride) return;
  if (!lcdReady) return;
  if (lcdBacklightEnabled) {
    lcd.noBacklight();
    lcdBacklightEnabled = false;
  }
}

void recoverDisplay() {
  initLcdSequence(true);
}

bool toggleBacklightOverride() {
  manualBacklightOverride = !manualBacklightOverride;
  if (manualBacklightOverride) {
    displayBacklightOn();
  }
  return manualBacklightOverride;
}

// ==================== KEYPAD ====================
char readKeypad() {
  char k = '\0';
  if (!keyQueue) return k;
  xQueueReceive(keyQueue, &k, 0);
  return k;
}

void drainKeypadBuffer() {
  if (!keyQueue) return;
  char k = '\0';
  while (xQueueReceive(keyQueue, &k, 0) == pdTRUE) {
  }
}

char getHeldKey() {
  return heldKeyAtomic;
}
