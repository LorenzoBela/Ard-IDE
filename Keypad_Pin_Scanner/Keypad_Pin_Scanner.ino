#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <HardwareSerial.h>

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

// Yung working custom pins mo
byte rowPins[ROWS] = {13, 23, 19, 26}; 
byte colPins[COLS] = {14, 25, 18};    

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// --- SYSTEM LOGIC ---
String inputCode = "";
const String correctCode = "1234"; 
bool faceDetected = false; 

// Serial2 para makausap ang ESP32-CAM
#define CAM_SERIAL Serial2 

void setup() {
  Serial.begin(115200); 
  // LILYGO RX = GPIO 16, TX = GPIO 17
  CAM_SERIAL.begin(115200, SERIAL_8N1, 16, 17); 
  
  pinMode(LOCK_PIN, OUTPUT);
  digitalWrite(LOCK_PIN, LOW); // Lock the door
  
  keypad.setDebounceTime(50); 
  
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Parcel-Safe v1");
  delay(2000); 
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Look at Camera"); 
}

void loop() {
  // 1. MAKINIG SA ESP32-CAM
  if (CAM_SERIAL.available()) {
    String msg = CAM_SERIAL.readStringUntil('\n');
    msg.trim(); 
    Serial.println("From CAM: " + msg);
    
    if (msg == "FACE_YES" && !faceDetected) {
      faceDetected = true;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Enter PIN:");
      inputCode = "";
    } 
    else if (msg == "FACE_NO" && faceDetected) {
      faceDetected = false;
      inputCode = "";
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Look at Camera");
    }
    else if (msg == "PHOTO_DONE") {
      unlockDoor();
    }
  }

  // 2. BASAHIN ANG KEYPAD (Gagana lang kapag True ang faceDetected)
  if (faceDetected) {
    char key = keypad.getKey();

    if (key) {
      if (key == '*') {
        inputCode = "";
        lcd.setCursor(0, 1);
        lcd.print("                ");
      } 
      else if (key == '#') {
        if (inputCode == correctCode) {
          lcd.setCursor(0, 1);
          lcd.print("Verifying...    ");
          
          // UTUSAN ANG ESP32-CAM NA KUMUHA NG PICTURE
          CAM_SERIAL.println("TAKE_PHOTO"); 
          
        } else {
          lcd.setCursor(0, 1);
          lcd.print("Wrong PIN!      ");
          delay(2000);
          inputCode = "";
          lcd.setCursor(0, 1);
          lcd.print("                ");
        }
      } 
      else {
        if (inputCode.length() < 4) {
          inputCode += key;
          lcd.setCursor(0, 1);
          lcd.print(inputCode);
        }
      }
    }
  }
}

void unlockDoor() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Box Unlocked!");
  
  digitalWrite(LOCK_PIN, HIGH); // Pitik sa solenoid
  delay(3000);                  // Bukas ng 3 segundo
  digitalWrite(LOCK_PIN, LOW);  // Patay kuryente
  
  inputCode = ""; 
  faceDetected = false; 
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Look at Camera");
}