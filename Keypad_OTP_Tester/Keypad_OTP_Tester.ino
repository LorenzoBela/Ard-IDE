#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>


// ===== KEYPAD SETUP =====
const byte ROWS = 4;
const byte COLS = 3;

char keys[ROWS][COLS] = {
    {'1', '2', '3'}, {'4', '5', '6'}, {'7', '8', '9'}, {'*', '0', '#'}};

// Keypad pins (ESP32) - Updated to new pinout
byte rowPins[ROWS] = {14, 26, 25, 18}; // R1-R4
byte colPins[COLS] = {19, 12, 27};     // C1-C3

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ===== LCD SETUP =====
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16x2 LCD

// Buffer to store pressed keys (max 6 digits)
char keyBuffer[7] = ""; // 6 chars + null terminator
byte keyIndex = 0;

void setup() {
  Serial.begin(115200);

  // Initialize I2C
  Wire.begin(21, 22); // SDA = 21, SCL = 22

  lcd.init();      // initialize the LCD
  lcd.backlight(); // turn on backlight

  // Initial Display
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ENTER OTP:");
  lcd.setCursor(0, 1);
  lcd.print("______"); // 6-digit placeholder
}

void clearBuffer() {
  keyIndex = 0;
  memset(keyBuffer, 0, sizeof(keyBuffer));
  lcd.setCursor(0, 1);
  lcd.print("______");
}

void loop() {
  char key = keypad.getKey();

  if (key != NO_KEY) {
    Serial.print("Pressed: ");
    Serial.println(key);

    // CLEAR Logic: * resets input
    if (key == '*') {
      clearBuffer();
      lcd.setCursor(0, 0);
      lcd.print("CLEARED      ");
      delay(500);
      lcd.setCursor(0, 0);
      lcd.print("ENTER OTP:   ");
    }
    // SUBMIT Logic: # validates length and "submits"
    else if (key == '#') {
      if (keyIndex == 6) {
        // Valid length - Simulate submit
        Serial.print("SUBMITTING OTP: ");
        Serial.println(keyBuffer);

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("OTP RECEIVED:");
        lcd.setCursor(0, 1);
        lcd.print(keyBuffer);

        delay(2000); // Show result for 2 seconds

        // Reset state
        clearBuffer();
        lcd.setCursor(0, 0);
        lcd.print("ENTER OTP:   ");
      } else {
        // Invalid length
        Serial.println("ERROR: Need 6 digits");

        lcd.setCursor(0, 0);
        lcd.print("NEED 6 DIGITS");
        delay(1000);
        lcd.setCursor(0, 0);
        lcd.print("ENTER OTP:   ");
      }
    }
    // DIGIT Logic: append if buffer not full
    else {
      if (keyIndex < 6) {
        keyBuffer[keyIndex++] = key;
        keyBuffer[keyIndex] = '\0';

        // Update display to show current buffer
        lcd.setCursor(0, 1);
        lcd.print("______"); // clear line first
        lcd.setCursor(0, 1);
        lcd.print(keyBuffer); // print digits
      }
    }
  }
}
