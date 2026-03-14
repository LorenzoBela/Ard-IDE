/**
 * Tester Lock Keypad LCD
 *
 * Purpose:
 *   Hardware-only logic test for keypad + LCD + solenoid lock.
 *   No WiFi, no camera, no cloud.
 *
 * Flow:
 *   1. Enter PIN on keypad.
 *   2. Press '#' to verify.
 *   3. Correct PIN -> unlock solenoid.
 *   4. Press '*' to relock.
 *
 * Hardware (same as Tester.ino):
 *   Keypad rows: 13, 23, 19, 26
 *   Keypad cols: 14, 25, 18
 *   LCD I2C: SDA/SCL default (21/22), address 0x27
 *   Solenoid MOSFET gate: GPIO 32
 */

#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// ==================== CONFIG ====================
#define LOCK_PIN 32
#define LCD_MESSAGE_DURATION 2000

// Test PIN (change as needed)
static const char TEST_PIN[] = "1234";

// LCD 16x2
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Keypad 4x3
static const byte ROWS = 4;
static const byte COLS = 3;

char keys[ROWS][COLS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}};

byte rowPins[ROWS] = {13, 23, 19, 26};
byte colPins[COLS] = {14, 25, 18};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ==================== STATE ====================
enum TestState {
  STATE_IDLE,
  STATE_ENTERING_PIN,
  STATE_VERIFYING_PIN,
  STATE_UNLOCKED,
  STATE_SHOW_MESSAGE
};

static TestState currentState = STATE_IDLE;

// Buffers (no dynamic String for runtime state)
static char inputCode[8];
static uint8_t inputLen = 0;

// Timers
static unsigned long messageStartAt = 0;

// Return state after showing temporary message
static TestState stateAfterMessage = STATE_IDLE;

void updateDisplay(const char *line0, const char *line1) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line0);
  lcd.setCursor(0, 1);
  lcd.print(line1);
}

void showTempMessage(const char *line0, const char *line1, TestState nextState) {
  updateDisplay(line0, line1);
  stateAfterMessage = nextState;
  messageStartAt = millis();
  currentState = STATE_SHOW_MESSAGE;
}

void resetInput() {
  inputLen = 0;
  inputCode[0] = '\0';
}

void showPinEntry() {
  char line1[17];
  snprintf(line1, sizeof(line1), "PIN: %s", inputCode);
  updateDisplay("Enter PIN:", line1);
}

void enterIdle() {
  resetInput();
  currentState = STATE_IDLE;
  updateDisplay("Enter PIN:", "PIN: ");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LOCK_PIN, OUTPUT);
  digitalWrite(LOCK_PIN, LOW); // Start locked

  keypad.setDebounceTime(50);

  lcd.init();
  lcd.backlight();

  updateDisplay("Lock Keypad LCD", "Initializing...");
  delay(800);

  enterIdle();
  Serial.println("[TEST] Ready: keypad + lcd + lock");
}

void loop() {
  unsigned long now = millis();

  switch (currentState) {
  case STATE_IDLE:
  case STATE_ENTERING_PIN: {
    char key = keypad.getKey();
    if (!key) {
      break;
    }

    if (key == '*') {
      resetInput();
      enterIdle();
      Serial.println("[KEYPAD] Input cleared");
    } else if (key == '#') {
      if (inputLen > 0) {
        currentState = STATE_VERIFYING_PIN;
      }
    } else if (inputLen < 6) {
      inputCode[inputLen++] = key;
      inputCode[inputLen] = '\0';

      if (currentState == STATE_IDLE) {
        currentState = STATE_ENTERING_PIN;
      }

      showPinEntry();
      Serial.printf("[KEYPAD] PIN now: %s\n", inputCode);
    }
    break;
  }

  case STATE_VERIFYING_PIN:
    if (strcmp(inputCode, TEST_PIN) == 0) {
      digitalWrite(LOCK_PIN, HIGH);
      Serial.println("[LOCK] UNLOCKED");
      resetInput();
      showTempMessage("PIN Correct!", "Lock: OPEN", STATE_UNLOCKED);
    } else {
      Serial.printf("[LOCK] WRONG PIN: %s\n", inputCode);
      resetInput();
      showTempMessage("Wrong PIN", "Try again", STATE_IDLE);
    }
    break;

  case STATE_UNLOCKED: {
    updateDisplay("Box Unlocked!", "* = Relock");

    char key = keypad.getKey();
    if (key == '*') {
      digitalWrite(LOCK_PIN, LOW);
      Serial.println("[LOCK] RELOCKED");
      showTempMessage("Box Locked", "Ready", STATE_IDLE);
    }
    break;
  }

  case STATE_SHOW_MESSAGE:
    if (now - messageStartAt >= LCD_MESSAGE_DURATION) {
      if (stateAfterMessage == STATE_IDLE) {
        enterIdle();
      } else if (stateAfterMessage == STATE_UNLOCKED) {
        currentState = STATE_UNLOCKED;
      } else {
        currentState = stateAfterMessage;
      }
    }
    break;
  }
}
