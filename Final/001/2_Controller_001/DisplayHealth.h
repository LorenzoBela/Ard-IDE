/**
 * DisplayHealth.h — LCD I2C health monitoring with fallback (EC-86)
 *
 * Ported from PlatformIO hardware/lib/DisplayControl.
 *
 * Features:
 *   - Periodic I2C ACK check to detect LCD failure
 *   - Tracks consecutive failures → DEGRADED → FAILED
 *   - LED flash + buzzer beep fallback when LCD is dead
 *   - Success/error/locked audio patterns for blind operation
 */

#ifndef DISPLAY_HEALTH_H
#define DISPLAY_HEALTH_H

#include <Arduino.h>
#include "Config.h"

enum DisplayStatus {
  DISP_OK,
  DISP_DEGRADED,   // Intermittent I2C failures
  DISP_FAILED      // LCD confirmed dead — fallback mode active
};

/** Initialise indicator pins. Call from setup() after Wire.begin(). */
void initDisplayHealth();

/**
 * Probe LCD I2C address. Call periodically (every DISPLAY_HEALTH_CHECK_MS).
 * Returns false if the LCD did not ACK.
 */
bool checkDisplayHealth();

/** Current aggregate status. */
DisplayStatus getDisplayStatus();

/** true when LCD is confirmed dead and fallback is active. */
bool isDisplayFailed();

/** Fallback: short LED blink + beep for each keypress digit. */
void fallbackKeyFeedback();

/** Fallback: rising triple-beep for unlock success. */
void fallbackSuccess();

/** Fallback: descending double-beep for wrong PIN / error. */
void fallbackError();

/** Fallback: rapid triple-beep for lockout warning. */
void fallbackLockout();

/** Consecutive I2C failure count (for proxy reporting). */
uint8_t getDisplayErrorCount();

#endif // DISPLAY_HEALTH_H
