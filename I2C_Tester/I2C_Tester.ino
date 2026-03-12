#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// If 0x27 does not work, try changing this to 0x3F
#define I2C_ADDR 0x27

LiquidCrystal_I2C lcd(I2C_ADDR, 16, 2);

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- I2C LCD Test ---");
  Serial.printf("Attempting to connect at address 0x%02X\n", I2C_ADDR);

  // Initialize I2C communication with ESP32 standard pins
  // SDA = GPIO 21, SCL = GPIO 22
  Wire.begin(21, 22);
  delay(500); // Give the LCD power a moment to stabilize

  // Initialize the LCD
  lcd.begin(); // Most ESP32 libraries prefer init() over begin()
  lcd.backlight();

  // Display test message
  lcd.setCursor(0, 0);
  lcd.print("I2C Connection");
  lcd.setCursor(0, 1);
  lcd.print("Successful! :)");

  Serial.println("Setup complete, text sent to LCD.");
}

void loop() {
  // Do nothing, just leave the text on the screen
  delay(1000);
}
