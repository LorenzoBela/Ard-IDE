#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// --- LOCK SETUP ---
#define LOCK_PIN 32 // Papunta sa PWM ng MOSFET

// --- LCD SETUP ---
LiquidCrystal_I2C lcd(0x27, 16, 2);

// --- KEYPAD SETUP ---
const byte ROWS = 4;
const byte COLS = 3;

char keys[ROWS][COLS] = {
    {'1', '2', '3'}, {'4', '5', '6'}, {'7', '8', '9'}, {'*', '0', '#'}};

// YUNG CUSTOM PIN MAPPING MO NA GUMAGANA:
byte rowPins[ROWS] = {13, 23, 27, 26};
byte colPins[COLS] = {14, 25, 18};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// --- PASSCODE LOGIC ---
String inputCode = "";
const String correctCode = "1234"; // Default password natin

void setup() {
  Serial.begin(115200);

  Serial2.begin(115200, SERIAL_8N1, 16,
                17); // Use Serial2 for ESP-CAM (RX: 16, TX: 17)

  // Setup the Lock Pin
  pinMode(LOCK_PIN, OUTPUT);
  digitalWrite(LOCK_PIN, LOW); // Naka-lock dapat sa simula

  keypad.setDebounceTime(50);

  lcd.begin();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Parcel-Safe v1");
  lcd.setCursor(0, 1);
  lcd.print("System Ready...");
  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Enter PIN:");
}

void loop() {
  forwardSerial(); // DEBUG: Constant monitoring of ESP-CAM Serial

  char key = keypad.getKey();

  if (key) {
    // Kapag pinindot ang '*' (Clear/Cancel)
    if (key == '*') {
      inputCode = "";
      lcd.setCursor(0, 1);
      lcd.print("                "); // Clear the bottom row
    }
    // Kapag pinindot ang '#' (Enter)
    else if (key == '#') {
      if (inputCode == correctCode) {
        unlockDoor();
      } else {
        lcd.setCursor(0, 1);
        lcd.print("Wrong PIN!      ");
        delay(2000);
        inputCode = ""; // Reset
        lcd.setCursor(0, 1);
        lcd.print("                "); // Clear
      }
    }
    // Kapag number ang pinindot
    else {
      if (inputCode.length() < 4) {
        inputCode += key;
        lcd.setCursor(0, 1);
        lcd.print(inputCode);
      }
    }
  }
}

// Function para pitikin yung Lock
void unlockDoor() {
  lcd.setCursor(0, 1);
  lcd.print("Box Unlocked!   ");

  Serial2.println("CAPTURE"); // Tell ESP32-CAM to take a photo!

  digitalWrite(LOCK_PIN, HIGH); // Pitik sa solenoid (Bukas!)
  delay(3000);                  // Bukas ng 3 segundo
  digitalWrite(LOCK_PIN, LOW);  // Patay kuryente (Sasarado ulit)

  lcd.setCursor(0, 1);
  lcd.print("Taking Photo... ");

  // Hintayin yung reply ng ESP32-CAM (Max 20 seconds wait time)
  unsigned long startWait = millis();
  bool isComplete = false;

  while (millis() - startWait < 20000) {
    if (Serial2.available() > 0) {
      String msg = Serial2.readStringUntil('\n');
      msg.trim();

      Serial.print("ESP-CAM says: ");
      Serial.println(msg);

      // Checking keywords sent by the ESP-CAM
      if (msg.indexOf("Upload attempt") >= 0) {
        lcd.setCursor(0, 1);
        lcd.print("Uploading...    ");
      } else if (msg.indexOf("Upload success") >= 0) {
        lcd.setCursor(0, 1);
        lcd.print("Upload Success! ");
        isComplete = true;
        delay(2000); // Let the message stay for 2 seconds
        break;
      } else if (msg.indexOf("Final upload status: FAILED") >= 0 ||
                 msg.indexOf("Capture failed") >= 0) {
        lcd.setCursor(0, 1);
        lcd.print("Upload Failed!  ");
        isComplete = true;
        delay(2000);
        break;
      }
    }
  }

  // Kung lumampas ng 20 seconds at walang nareceive na success/fail
  if (!isComplete) {
    lcd.setCursor(0, 1);
    lcd.print("Cam Timeout!    ");
    Serial.println(
        "DEBUG: Cam Timeout (No success/fail message received in 20s).");
    delay(2000);
  }

  inputCode = ""; // Reset input
  lcd.setCursor(0, 1);
  lcd.print("                "); // Clear bottom row
}

// ----------------------------------------------------
// DEBUGGING HELPER: Ipasa ang Serial2 papuntang Serial
// ----------------------------------------------------
void forwardSerial() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    Serial.print(c); // Print sa USB Serial Monitor lahat ng sinasabi ng ESP-CAM
  }
}
