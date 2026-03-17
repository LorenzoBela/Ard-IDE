/**
 * LockSafety.h — Solenoid lock control with safety features
 *
 * Ported from PlatformIO hardware/lib/LockControl (EC-21/22/95/96).
 *
 * Features:
 *   EC-21: Retry unlock up to 3x, report STUCK_CLOSED
 *   EC-22: Verify lock engaged via reed switch, report STUCK_OPEN
 *   EC-95: Reed switch debounce (50 ms)
 *   Reed ISR edge capture always armed for tamper invariance
 *   EC-96: Modeled coil temperature — blocks actuation if overheated
 *
 * Hardware:
 *   LOCK_PIN        → MOSFET gate (active HIGH = solenoid energised)
 *   REED_SWITCH_PIN → Magnetic sensor (HIGH = closed/locked, LOW = open)
 */

#ifndef LOCK_SAFETY_H
#define LOCK_SAFETY_H

#include <Arduino.h>
#include "Config.h"

enum LockStatus {
  LOCK_OK,
  LOCK_STUCK_CLOSED,     // EC-21: Solenoid won't open after retries
  LOCK_STUCK_OPEN,       // EC-22: Solenoid won't close (security risk)
  LOCK_OVERHEATED,       // EC-96: Thermal protection blocked actuation
  LOCK_THERMAL_CUTOFF    // Safety timeout — solenoid was forcibly de-energised
};

/** Initialise solenoid and reed switch pins. Call from setup(). */
void initLock();

/**
 * EC-21: Attempt to unlock with retry logic.
 * Retries up to LOCK_RETRY_MAX times, verifying reed switch each attempt.
 * Blocks briefly (retries use delay — only called on explicit unlock action).
 */
LockStatus tryUnlock(bool ignoreReed = false);

/**
 * EC-22: Attempt to lock and verify via reed switch.
 * Returns LOCK_STUCK_OPEN if reed switch still reads open after de-energising.
 */
LockStatus tryLock(bool ignoreReed = false);

/**
 * Call every loop() tick. Non-blocking.
 * - EC-96: Cools the thermal model over time
 * - EC-95: Updates debounced reed switch state
 * - Hard cutoff: de-energises solenoid after LOCK_MAX_ACTIVE_MS
 * Returns true if a thermal cutoff occurred this tick.
 */
bool maintainLockSafety(unsigned long now);

/** EC-95: Debounced reed switch state. true = box is closed/locked. */
bool isBoxLocked();

/** EC-96: true if modeled coil temperature exceeds safe threshold. */
bool isOverheated();

/** true if solenoid is currently energised. */
bool isSolenoidActive();

/** Number of retries on the last unlock attempt. */
uint8_t getLastRetryCount();

/** Human-readable status string for logging / proxy reporting. */
const char* lockStatusStr(LockStatus s);

/**
 * Tamper detection via reed switch.
 * Returns true if the lid opened while the solenoid was NOT energised
 * (i.e. nobody asked for an unlock — physical forced entry).
 * Latch stays set until clearTamper() is called.
 */
bool isTamperDetected();

/** Clear the tamper latch (call after reporting to Proxy). */
void clearTamper();

/**
 * Suppress tamper detection for an authorized unlock window.
 * Call when entering STATE_UNLOCKING so the legitimate lid-open
 * is not misinterpreted as tampering.
 */
void suppressTamper();

/** Re-enable tamper detection (call when relocking / leaving unlock state). */
void armTamper();

#endif // LOCK_SAFETY_H
