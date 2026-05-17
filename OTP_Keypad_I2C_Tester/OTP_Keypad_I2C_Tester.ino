#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

// --- LOCK SETUP ---
#define LOCK_PIN 32 // Papunta sa PWM ng MOSFET

// --- LCD SETUP ---
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// --- KEYPAD SETUP ---
const byte ROWS = 4; 
const byte COLS = 3; 

char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

// YUNG CUSTOM PIN MAPPING MO NA GUMAGANA:
byte rowPins[ROWS] = {19, 18, 25, 13};
byte colPins[COLS] = {23, 26, 14};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// --- PASSCODE LOGIC ---
String inputCode = "";
const String correctCode = "1234"; // Default password natin

void setup() {
  Serial.begin(115200);
  
  // Setup the Lock Pin
  pinMode(LOCK_PIN, OUTPUT);
  digitalWrite(LOCK_PIN, LOW); // Naka-lock dapat sa simula

  // Keep keypad column inputs stable (active LOW on press)
  for (byte c = 0; c < COLS; c++) {
    pinMode(colPins[c], INPUT_PULLUP);
  }
  
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
  char key = keypad.getKey();

  if (key) {
    Serial.print("Key pressed: ");
    Serial.println(key);

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
  
  digitalWrite(LOCK_PIN, HIGH); // Pitik sa solenoid (Bukas!)
  delay(3000);                  // Bukas ng 3 segundo
  digitalWrite(LOCK_PIN, LOW);  // Patay kuryente (Sasarado ulit)
  
  inputCode = ""; // Reset input
  lcd.setCursor(0, 1);
  lcd.print("                "); // Clear bottom row
}