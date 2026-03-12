/**
 * DisplayHealth.cpp — LCD I2C health monitoring with fallback (EC-86)
 */

#include "DisplayHealth.h"
#include <Wire.h>

static DisplayStatus status       = DISP_OK;
static uint8_t       failCount    = 0;
static uint8_t       totalErrors  = 0;

void initDisplayHealth() {
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_STATUS_PIN, LOW);
  noTone(BUZZER_PIN);
}

bool checkDisplayHealth() {
  Wire.beginTransmission(DISPLAY_I2C_ADDR);
  uint8_t err = Wire.endTransmission();

  if (err == 0) {
    // ACK received — LCD is alive
    if (status == DISP_DEGRADED) {
      Serial.println(F("[EC-86] LCD recovered"));
    }
    failCount = 0;
    status = DISP_OK;
    return true;
  }

  // No ACK
  failCount++;
  totalErrors++;
  Serial.printf("[EC-86] LCD I2C fail #%u (err=%u)\n", failCount, err);

  if (failCount >= DISPLAY_MAX_FAILURES) {
    if (status != DISP_FAILED) {
      Serial.println(F("[EC-86] LCD FAILED — switching to fallback mode"));
    }
    status = DISP_FAILED;
  } else {
    status = DISP_DEGRADED;
  }
  return false;
}

DisplayStatus getDisplayStatus() { return status; }
bool isDisplayFailed()           { return status == DISP_FAILED; }
uint8_t getDisplayErrorCount()   { return totalErrors; }

// ── Fallback patterns (non-blocking where possible) ──

void fallbackKeyFeedback() {
  digitalWrite(LED_STATUS_PIN, HIGH);
  tone(BUZZER_PIN, 1200, 40);
  // Caller should schedule LED off after ~40 ms via maintainLockSafety tick
  // For simplicity, use a short blocking flash here (acceptable during keypress)
  delay(40);
  digitalWrite(LED_STATUS_PIN, LOW);
}

void fallbackSuccess() {
  // Rising triple-beep: 800 → 1200 → 1600 Hz
  for (int f = 800; f <= 1600; f += 400) {
    tone(BUZZER_PIN, f, 100);
    digitalWrite(LED_STATUS_PIN, HIGH);
    delay(120);
    digitalWrite(LED_STATUS_PIN, LOW);
    delay(30);
  }
  noTone(BUZZER_PIN);
}

void fallbackError() {
  // Descending double-beep: 400 → 200 Hz
  tone(BUZZER_PIN, 400, 150);
  digitalWrite(LED_STATUS_PIN, HIGH);
  delay(180);
  tone(BUZZER_PIN, 200, 150);
  delay(180);
  digitalWrite(LED_STATUS_PIN, LOW);
  noTone(BUZZER_PIN);
}

void fallbackLockout() {
  // Rapid triple low beep
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 300, 100);
    digitalWrite(LED_STATUS_PIN, HIGH);
    delay(130);
    digitalWrite(LED_STATUS_PIN, LOW);
    delay(70);
  }
  noTone(BUZZER_PIN);
}
