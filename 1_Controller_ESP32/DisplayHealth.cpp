/**
 * DisplayHealth.cpp — LCD I2C health monitoring with fallback (EC-86)
 */

#include "DisplayHealth.h"
#include <Wire.h>

static DisplayStatus status       = DISP_OK;
static uint8_t       failCount    = 0;
static uint8_t       totalErrors  = 0;

void initDisplayHealth() {
  // No hardware LED/Buzzer to initialize
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
  // No hardware support for LED/Buzzer feedback
}

void fallbackSuccess() {
  // No hardware support for LED/Buzzer feedback
}

void fallbackError() {
  // No hardware support for LED/Buzzer feedback
}

void fallbackLockout() {
  // No hardware support for LED/Buzzer feedback
}
