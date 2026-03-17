/**
 * HardwareIO.cpp — LCD and Keypad implementation
 *
 * Solenoid control has moved to LockSafety.cpp.
 */

#include "HardwareIO.h"
#include <Wire.h>
#include <Arduino.h>

// ── LCD ──
static LiquidCrystal_I2C lcd(DISPLAY_I2C_ADDR, 16, 2);
static bool lcdBacklightEnabled = true;

// ── Keypad ──
static char keys[KP_ROWS][KP_COLS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}
};
static byte rowPins[KP_ROWS] = {13, 23, 19, 26};
static byte colPins[KP_COLS] = {14, 25, 18};
static Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, KP_ROWS, KP_COLS);

static QueueHandle_t keyQueue = NULL;

static void keypadTask(void *pvParameters) {
  while (true) {
    char k = keypad.getKey();
    if (k) {
      xQueueSend(keyQueue, &k, 0); // Non-blocking send
      Serial.printf("[KEYPAD] Buffered: %c\n", k);
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // Poll every 20ms
  }
}

// ==================== INIT ====================
void initHardwareIO() {
  keypad.setDebounceTime(50);
  
  keyQueue = xQueueCreate(10, sizeof(char));
  xTaskCreatePinnedToCore(keypadTask, "KeypadTask", 2048, NULL, 2, NULL, 1);

  Wire.begin(21, 22);
  delay(500);           // LCD Vcc rail settle time
  lcd.begin();
  lcd.backlight();
  lcdBacklightEnabled = true;
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
  if (lcdBacklightEnabled) {
    lcd.noBacklight();
    lcdBacklightEnabled = false;
  }
}

// ==================== KEYPAD ====================
char readKeypad() {
  char k = '\0';
  xQueueReceive(keyQueue, &k, 0); // Non-blocking read from buffer
  return k;
}

Keypad& getKeypad() {
  return keypad;
}
