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

static QueueHandle_t keyQueue = NULL;

// Thread-safe held-key tracking (written by keypadTask on Core 1,
// read by main loop on Core 0 for EC-82 stuck detection).
static volatile char heldKeyAtomic = 0;

static void initLcdSequence(bool fast) {
  Wire.begin(21, 22);
  Wire.setClock(100000);
  delay(fast ? 20 : 800); // Fast re-sync after noise vs cold-boot settle
  for (int i = 0; i < 2; i++) {
    lcd.begin();
    delay(fast ? 20 : 50);
    lcd.clear();
    lcd.home();
  }
  lcd.backlight();
  lcdBacklightEnabled = true;
}

static void keypadTask(void *pvParameters) {
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

      // If matrix ghosting happens, '2' (Row 0) and '0' (Row 3) both trigger in the same tick.
      // Because keypad.key[] is populated in matrix scan order, the true physical
      // key ('0', row 3) will always overwrite the ghost key ('2', row 0) in lastKey.
      if (newPresses > 0) {
        if (millis() - lastBufferTime > 150) { // Strict software cooldown prevents "00" bounce
          xQueueSend(keyQueue, &lastKey, 0);
          lastBufferTime = millis();
          Serial.printf("[KEYPAD] Buffered: %c (from %d simultaneous)\n", lastKey, newPresses);
        }
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
  keypad.setDebounceTime(100);
  keypad.setHoldTime(500);  // HOLD state after 500ms (long-press '0' checks 1200ms separately)
  
  keyQueue = xQueueCreate(10, sizeof(char));
  xTaskCreatePinnedToCore(keypadTask, "KeypadTask", 2048, NULL, 2, NULL, 1);

  initLcdSequence(false);
  updateDisplay("Parcel-Safe v2", "Connecting WiFi");

  Serial2.begin(CAM_UART_BAUD, SERIAL_8N1, CAM_UART_RX, CAM_UART_TX);
  Serial.println(F("[UART] Serial2 ready (CAM fallback)"));
}

// ==================== LCD ====================
void updateDisplay(const char *line0, const char *line1) {
  char buf[17];
  lcd.setCursor(0, 0);
  snprintf(buf, sizeof(buf), "%-16s", line0);
  lcd.print(buf);
  lcd.setCursor(0, 1);
  snprintf(buf, sizeof(buf), "%-16s", line1);
  lcd.print(buf);
}

void displayBacklightOn() {
  if (!lcdBacklightEnabled) {
    lcd.backlight();
    lcdBacklightEnabled = true;
  }
}

void displayBacklightOff() {
  if (manualBacklightOverride) return;
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
