/**
 * KEYPAD RAW PIN SCANNER
 *
 * Purpose: Determine the REAL row/col-to-GPIO mapping of your 4x3 keypad.
 *
 * How it works:
 *   - All 7 keypad GPIOs are tested.
 *   - Each pin is driven LOW one at a time while the others are INPUT_PULLUP.
 *   - When you press a key, two pins become connected.
 *   - The Serial Monitor prints exactly which GPIOs are bridged.
 *
 * Instructions:
 *   1. Upload this sketch.
 *   2. Open Serial Monitor at 115200 baud.
 *   3. Press each key ONE AT A TIME and note the output.
 *   4. Share the output so we can fix the mapping.
 *
 * Pinout under test (all 7 keypad GPIOs from Smart_Top_Box_Main):
 *   Current "row" pins: 13, 14, 15, 32
 *   Current "col" pins: 18, 19, 23
 */

#include <Arduino.h>

// All 7 GPIOs connected to the keypad connector
static const byte ALL_PINS[] = {13, 14, 15, 5, 18, 19, 23};
static const byte PIN_COUNT = 7;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=============================================");
  Serial.println(" KEYPAD RAW PIN SCANNER");
  Serial.println(" Press each key one at a time.");
  Serial.println(" Note which GPIOs are printed for each key.");
  Serial.println("=============================================");
  Serial.println();
}

void loop() {
  // For each pin: drive it LOW, set all others to INPUT_PULLUP,
  // then check which other pins read LOW (meaning they're connected
  // to the driven pin through the pressed button).

  for (byte d = 0; d < PIN_COUNT; d++) {
    // Drive pin d LOW
    pinMode(ALL_PINS[d], OUTPUT);
    digitalWrite(ALL_PINS[d], LOW);

    // Set every other pin to INPUT_PULLUP
    for (byte r = 0; r < PIN_COUNT; r++) {
      if (r == d)
        continue;
      pinMode(ALL_PINS[r], INPUT_PULLUP);
    }

    delayMicroseconds(50); // let pullups settle

    // Read the other pins
    for (byte r = 0; r < PIN_COUNT; r++) {
      if (r == d)
        continue;
      if (digitalRead(ALL_PINS[r]) == LOW) {
        // Found a connection!
        Serial.printf(">> KEY DETECTED:  GPIO %2d  <-->  GPIO %2d\n",
                      ALL_PINS[d], ALL_PINS[r]);

        // Wait for key release to avoid spam
        while (digitalRead(ALL_PINS[r]) == LOW) {
          delay(10);
        }
        Serial.println("   (released)");
        delay(200); // debounce
      }
    }

    // Reset the driven pin back to input so it doesn't interfere
    pinMode(ALL_PINS[d], INPUT_PULLUP);
  }
}
