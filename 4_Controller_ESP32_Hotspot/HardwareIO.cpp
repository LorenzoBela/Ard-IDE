/**
 * HardwareIO.cpp — LCD and Keypad implementation
 *
 * Solenoid control has moved to LockSafety.cpp.
 */

#include "HardwareIO.h"
#include <Keypad.h>
#include <Wire.h>
#include <Arduino.h>

// ── LCD ──
static LiquidCrystal_I2C lcd(DISPLAY_I2C_ADDR, 16, 2);
static bool lcdBacklightEnabled = true;
static bool manualBacklightOverride = false;
static bool lcdReady = false;

// ── Keypad ──
static char keys[KP_ROWS][KP_COLS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}
};
static uint8_t rowPins[KP_ROWS] = {13, 23, 19, 26};
static uint8_t colPins[KP_COLS] = {14, 25, 18};
static Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, KP_ROWS, KP_COLS);

static const unsigned long KEYPAD_DEBOUNCE_MS = 80;
static const unsigned long KEYPAD_BUFFER_COOLDOWN_MS = 120;

static QueueHandle_t keyQueue = NULL;

// Thread-safe held-key tracking (written by keypadTask on Core 1,
// read by main loop on Core 0 for EC-82 stuck detection).
static volatile char heldKeyAtomic = 0;

static void initLcdSequence(bool fast) {
  Serial.println(F("[HW] LCD init start"));
  Serial.flush();
  Wire.begin(21, 22);
  Wire.setTimeOut(50);
  Wire.setClock(100000);
  delay(fast ? 20 : 80); // Fast re-sync after noise vs cold-boot settle

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

static void keypadTask(void *pvParameters) {
  (void)pvParameters;
  static unsigned long lastBufferTime = 0;

  while (true) {
    if (keypad.getKeys()) {
      int newPresses = 0;
      char lastKey = 0;
      
      for (int i = 0; i < LIST_MAX; i++) {
        if (!keypad.key[i].stateChanged) continue;

        if (keypad.key[i].kstate == PRESSED && keypad.key[i].kchar != 0) {
          newPresses++;
          lastKey = keypad.key[i].kchar;
        }
      }

      if (newPresses == 1) {
        if (millis() - lastBufferTime > KEYPAD_BUFFER_COOLDOWN_MS) {
          if (keyQueue) {
            xQueueSend(keyQueue, &lastKey, 0);
          }
          lastBufferTime = millis();
          CTRL_LOG_PRINTF("[KEYPAD] Buffered: %c\n", lastKey);
        }
      } else if (newPresses > 1) {
        CTRL_LOG_PRINTF("[KEYPAD] Ignored ambiguous scan (%d simultaneous)\n", newPresses);
      }
    }

    // Track held key for EC-82 stuck detection + long-press '0' LCD recovery.
    // Scan all key slots — the last one in HOLD state wins (bypasses ghost '2' HOLD).
    char held = 0;
    for (int i = 0; i < LIST_MAX; i++) {
      if (keypad.key[i].kstate == HOLD && keypad.key[i].kchar != 0) {
        held = keypad.key[i].kchar;
      }
    }
    heldKeyAtomic = held;

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ==================== INIT ====================
void initHardwareIO() {
  Serial.println(F("[HW] init start"));
  Serial.flush();

  keypad.setDebounceTime(KEYPAD_DEBOUNCE_MS);
  keypad.setHoldTime(500);  // HOLD state after 500ms (long-press '0' checks 1200ms separately)
  Serial.println(F("[HW] keypad configured"));
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
  xQueueReceive(keyQueue, &k, 0); // Non-blocking read from buffer
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
